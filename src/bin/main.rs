#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

//! Single-motor car firmware, driven by the ZD USB gamepad receiver.
//!
//! Hardware:
//! - 2S 7.4 V LiPo: battery → TB6612 VM direct; a 5 V buck feeds the
//!   board's 5VIN. The USB-OTG pads on the board back are bridged so the
//!   receiver's VBUS comes from the 5 V buck output (same net as 5VIN) —
//!   a clean 5 V, within USB spec.
//! - Motor: 12 V-rated N30 4000 RPM, single rear drive (no diff), via
//!   TB6612 channel A: AIN1=G11, AIN2=G12, STBY=G10, PWMA=G13
//!   (LEDC Timer2/Ch1, 10 kHz). 7.4 V is under-voltage for a 12 V motor,
//!   so full duty is fine; heat comes from stall current, not voltage.
//! - Servo SG90/MG90S on GPIO14 (LEDC Timer0/Ch0, 50 Hz) via [`Steering`]
//! - USB receiver on native OTG port GPIO19 (D-) / GPIO20 (D+)
//!
//! Input mapping:
//! - Left stick Y → throttle (0 = full forward, 128 = centre, 255 = reverse)
//! - Right stick X → steering angle (0 = full left, 128 = centre, 255 = right)
//!
//! This bin owns the pin map, the peripherals and the chassis parameters only:
//! the USB session ([`UsbSession`]) and the control policy ([`Chassis`],
//! [`control`]) live in the library so the 1/10 car can share them.

use defmt::info;
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    Ledc, LowSpeed,
    channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb::otg::{Usb, embassy_usb_host::Driver};

use jocar_steer::chassis::Chassis;
use jocar_steer::control;
use jocar_steer::flash_log;
use jocar_steer::rc_pwm;
use jocar_steer::steering::Steering;
use jocar_steer::tb6612::Tb6612Single;
use jocar_steer::usb_session::UsbSession;
use esp_println as _;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("Panic: {}", defmt::Display2Format(info));
    // Safety: never leave the motor commanded after a panic. Reset the chip so
    // the LEDC channels / direction pins return to their reset state (no
    // output) instead of holding the last command.
    esp_hal::system::software_reset()
}

/// Static center offset in degrees to cancel residual servo mounting error.
const CENTER_TRIM_DEG: i32 = 0;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Strapping
    let _ = peripherals.GPIO0;
    let _ = peripherals.GPIO3;
    let _ = peripherals.GPIO45;
    let _ = peripherals.GPIO46;

    // flash/psram
    let _ = peripherals.GPIO26;
    let _ = peripherals.GPIO27;
    let _ = peripherals.GPIO28;
    let _ = peripherals.GPIO29;
    let _ = peripherals.GPIO30;
    let _ = peripherals.GPIO31;
    let _ = peripherals.GPIO32;
    // octal flash/psram, might never need it
    let _ = peripherals.GPIO33;
    let _ = peripherals.GPIO34;
    let _ = peripherals.GPIO35;
    let _ = peripherals.GPIO36;
    let _ = peripherals.GPIO37;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // ── Persistent event log (one unused flash sector) ────────────────
    // Records what happened while the console could not be attached; see
    // src/flash_log.rs. Created before the USB host starts so the flash access
    // (cache and interrupts disabled) cannot disturb USB timing.
    let mut log = flash_log::FlashLog::new(esp_storage::FlashStorage::new(peripherals.FLASH));
    log.record(
        flash_log::EV_BOOT,
        format_args!("boot {:?}", esp_hal::system::reset_reason()),
    );
    log.dump();

    // ── Control-phase watchdog ────────────────────────────────────────
    // TIMG1 is otherwise idle (TIMG0.timer0 belongs to esp-rtos). Enabled and
    // fed by the USB session's control loop; see src/usb_session.rs.
    let mut wdt = TimerGroup::new(peripherals.TIMG1).wdt;

    // ── Servo on GPIO14 via LEDC (50 Hz) ─────────────────────────────
    let mut ledc = Ledc::new(peripherals.LEDC);
    rc_pwm::init(&mut ledc);

    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    // Frequency and resolution come from the shared RC-pulse module so the
    // timer and the pulse→counts conversion cannot disagree (see rc_pwm).
    lstimer.configure(rc_pwm::timer_config()).unwrap();

    let servo_pin = peripherals.GPIO14;
    let mut ch = ledc.channel(channel::Number::Channel0, servo_pin);
    ch.configure(channel::config::Config {
        timer: &lstimer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    // ── Motor PWM on GPIO13 (PWMA) via LEDC (10 kHz) ──────────────────
    // NOTE: Timer1 and Channel2 were found to produce no output on this
    // setup, so the motor uses Timer2 + Channel1 (same as the archived PS2
    // firmware did).
    // 12-bit is load-bearing here: `Tb6612Single` writes abstract speed 1:1 as
    // raw counts clamped to ±4095, and `ControlConfig::motor_max_speed` is that
    // same full scale. Changing the resolution means changing all three.
    let mut motor_timer = ledc.timer::<LowSpeed>(timer::Number::Timer2);
    motor_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(10),
        })
        .unwrap();

    let motor_pwm = peripherals.GPIO13;
    let mut mch = ledc.channel(channel::Number::Channel1, motor_pwm);
    mch.configure(channel::config::Config {
        timer: &motor_timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    // ── Chassis parameters ───────────────────────────────────────────
    let cfg = control::ControlConfig {
        // 30° is the usable limit on this chassis — past it the front
        // wheels scrub so hard (no rear diff) the motor stalls.
        steer_max_deg: 30,
        // Full speed is safe: the N30 is 12 V-rated and the battery is 2S
        // (7.4 V), so we are under-voltage, not over. The motor heats from
        // stall current during turns, which the steer-throttle mix mitigates.
        motor_max_speed: 4095,
        // Time-based. The previous 512/tick was silently scaled by the
        // report rate; ~15.5 k/s restores the designed 33 ms-tick behaviour.
        motor_slew_rate_speed_s: 15_500,
        steer_slew_rate_deg_s: 400, // ≈ the designed 8°/33 ms
        rx_deadzone: 3,
        ly_deadzone: 3,
        // Cut 50 % at full lock: the mix compensates for the front-wheel
        // scrub of a chassis without a differential.
        steer_mix_num: 1,
        steer_mix_den: 2,
        // Start kick disabled (A/B switch on this chassis): the structural fix
        // for stall-current heat is more gear reduction, so the kick is being
        // re-validated on the car rather than assumed.
        kick_duration_ms: 0,
        kick_min_num: 3,
        kick_min_den: 10, // kick only above 30 % throttle
        // Short H-bridge coast on a direction reversal. This used to be "one
        // call", i.e. the last per-tick quantity in the control path; 5 ms is
        // the same thing expressed in time.
        reverse_coast_ms: 5,
        // A lost link must not leave the car at its last throttle. 2 s at full
        // speed is already ~16 m on the 1/10 car, so this is a safety limit,
        // not a comfort setting.
        failsafe_timeout_ms: 2000,
    };

    let steering = Steering::new(ch, CENTER_TRIM_DEG, cfg.steer_max_deg);
    info!(
        "Steering: offset={}°  max={}°  right stick → steer (G14)",
        CENTER_TRIM_DEG, cfg.steer_max_deg
    );

    // ── TB6612 channel A: AIN1=G11, AIN2=G12, STBY=G10, PWMA=G13 ─────
    let motors = Tb6612Single::new(
        peripherals.GPIO11, // AIN1
        peripherals.GPIO12, // AIN2
        peripherals.GPIO10, // STBY
        mch,
    );
    // Construction arms the driver (STBY high, duty 0 = coast).
    let mut chassis = Chassis::new(steering, motors, cfg);
    info!("Motor enabled: left stick Y → throttle");

    // ── USB OTG host on GPIO19 (D-) / GPIO20 (D+) ────────────────────
    let usb = Usb::new_fs(peripherals.USB_FS, peripherals.GPIO20, peripherals.GPIO19);
    let mut session = UsbSession::new(Driver::new(usb));
    session.run(&mut chassis, &mut log, &mut wdt).await
}
