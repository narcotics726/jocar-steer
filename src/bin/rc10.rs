#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

//! 1/10 car firmware (HSP 94123-class 4WD: brushed 540 + BDESC-S10E-RTR ESC,
//! MG996R steering servo), driven by the same ZD USB gamepad receiver as the
//! other car — the shared input path is [`crate::usb_session`], the control
//! policy is [`crate::chassis`].
//!
//! Hardware / pin map:
//! - **Steering servo** MG996R on **G14** (LEDC Timer0/Ch0, 50 Hz)
//! - **ESC throttle** on **G1** (LEDC Timer0/Ch1 — same timer, so both stay at
//!   50 Hz; Timer2 is free on this car)
//! - **Receiver** on the native OTG port, G19 (D-) / G20 (D+)
//! - Power: 2S → buck A (≥3 A) → servo, buck B → board 5VIN, and 2S direct to
//!   the ESC. **The ESC's BEC stays disconnected** (see the plan §3.5 for the
//!   wiring diagram and why: a 1.5–2.5 A servo stall on a 2 A BEC can reset the
//!   ESC itself).
//!
//! Differences from the `jocar-steer` bin are deliberate and all in the two
//! config structs below: steering-throttle mix **off** (this chassis has normal
//! steering geometry, so there is no scrub-induced stall to compensate),
//! start-kick **off** (any residual above neutral delays the ESC's neutral
//! detection and breaks reverse), and conservative limits until the first
//! on-car calibration.
//!
//! Every value that says "calibrate" below is a real calibration input: the
//! procedure is `servo-test` (pulse sweep) + a bench run with the wheels up.

use defmt::info;
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    Ledc, LowSpeed,
    channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb::otg::{Usb, embassy_usb_host::Driver};

use jocar_steer::chassis::Chassis;
use jocar_steer::control::ControlConfig;
use jocar_steer::esc::{Esc, EscConfig};
use jocar_steer::flash_log;
use jocar_steer::rc_pwm;
use jocar_steer::steering::Steering;
use jocar_steer::usb_session::UsbSession;
use esp_println as _;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("Panic: {}", defmt::Display2Format(info));
    // Safety: reset rather than hold. The LEDC stops with the chip, so the ESC
    // sees signal loss and drops to neutral by itself, and the next boot re-runs
    // the arming hold.
    esp_hal::system::software_reset()
}

/// Full-scale abstract speed. Used for *both* the control config and the ESC's
/// pulse scaling: if those two ever disagree the full-throttle point moves
/// silently, so they share one constant.
const MOTOR_MAX_SPEED: i32 = 4095;

/// Static center offset in degrees to cancel residual servo mounting error.
/// Calibrate on the car (procedure: `servo-test`, then trim by eye).
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
    // See src/flash_log.rs; read it back with `espflash read-flash` +
    // `tools/flashlog_decode.py` once the console is attached again.
    let mut log = flash_log::FlashLog::new(esp_storage::FlashStorage::new(peripherals.FLASH));
    log.record(
        flash_log::EV_BOOT,
        format_args!("boot {:?}", esp_hal::system::reset_reason()),
    );
    log.dump();

    // ── Control-phase watchdog (TIMG1 is otherwise idle) ──────────────
    let mut wdt = TimerGroup::new(peripherals.TIMG1).wdt;

    // ── One 50 Hz timer for servo + ESC (LEDC Timer0) ─────────────────
    // Both outputs are RC pulses on the same period, so they share a timer;
    // only the channels differ. Channel 2/Timer1 were found to produce no
    // output on this board earlier, hence Ch0 + Ch1 on Timer0.
    let mut ledc = Ledc::new(peripherals.LEDC);
    rc_pwm::init(&mut ledc);

    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    // Frequency and resolution come from the shared RC-pulse module, so the
    // timer config and the drivers' pulse→counts conversion cannot disagree.
    lstimer.configure(rc_pwm::timer_config()).unwrap();

    let mut servo_ch = ledc.channel(channel::Number::Channel0, peripherals.GPIO14);
    servo_ch
        .configure(channel::config::Config {
            timer: &lstimer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    let mut esc_ch = ledc.channel(channel::Number::Channel1, peripherals.GPIO1);
    esc_ch
        .configure(channel::config::Config {
            timer: &lstimer,
            duty_pct: 0, // `Esc::new` writes neutral immediately after this
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // ── Chassis parameters ───────────────────────────────────────────
    let cfg = ControlConfig {
        // Conservative until calibrated: this chassis can take noticeably more
        // (the plan estimates 45–60°), but the criterion is the wheels reaching
        // their mechanical stop *without* the servo buzzing at the end.
        steer_max_deg: 30,
        motor_max_speed: MOTOR_MAX_SPEED,
        // Un-calibrated first value, carried over from the other car. The ESC
        // is the actuator with its own soft start, so the slew here is about
        // not shocking the drivetrain, not about current limiting.
        motor_slew_rate_speed_s: 15_500,
        // The servo has its own buck on this car, so this cap is a comfort
        // setting rather than a rail-protection measure (it is already above
        // what an MG996R can physically do: ~350 °/s).
        steer_slew_rate_deg_s: 400,
        rx_deadzone: 3,
        ly_deadzone: 3,
        // Mix OFF: it exists to stop a differential-less chassis from stalling
        // while scrubbing through a turn. This one steers normally.
        steer_mix_num: 0,
        steer_mix_den: 1,
        // Kick OFF: boundary condition ② of the plan — the throttle channel may
        // carry no residual above neutral, or the ESC never detects neutral and
        // reverse stops working.
        kick_duration_ms: 0,
        kick_min_num: 3,
        kick_min_den: 10,
        // The ESC latches its direction and only releases the latch after a
        // neutral dwell; measured from the *demanded* direction, and the ramp is
        // frozen while it holds so the reverse onset is a ramp, not a step
        // (MotorSlew). Too short and reverse silently never engages — this is
        // boundary condition ① of the plan and section C of `servo-test` is how
        // it gets calibrated.
        reverse_coast_ms: 300,
        // Full throttle for 2 s is ~16 m on this car; this is a safety limit.
        failsafe_timeout_ms: 2000,
    };

    let esc_cfg = EscConfig {
        speed_full: MOTOR_MAX_SPEED,
        // Nominal RC endpoints. **Calibrate before driving**: procedure and
        // criteria are in the plan §3.2/§3.4 and the values come from
        // `servo-test` + a bench run with the wheels off the ground.
        neutral_us: 1500,
        // ~70 % of nominal span for the first run. Raise it once the endpoints
        // (and the ESC's cut-off behaviour) are known.
        forward_span_us: 350,
        // Reverse travel is often shorter on this class of ESC; keep it equal
        // for now and calibrate the point where reverse reliably engages.
        reverse_span_us: 350,
        // Must swallow what the stick still reports at rest *after*
        // `ly_deadzone: 3` has been applied: ~5 counts of residual + 3 counts of
        // deadzone = 8/128 of the range ≈ 256 units. Read the resting `ly=` off
        // the console (it prints every 100 ms) and adjust — too narrow and the
        // ESC never sees neutral, which kills reverse in a way that looks
        // exactly like dead hardware.
        deadzone: 256,
        // ESC needs a continuous neutral hold after power-up/reset before it
        // accepts throttle; the criterion is its own arm confirmation.
        arm_ms: 2000,
    };

    let steering = Steering::new(servo_ch, CENTER_TRIM_DEG, cfg.steer_max_deg);
    info!(
        "Steering: offset={}°  max={}°  right stick → steer (G14)",
        CENTER_TRIM_DEG, cfg.steer_max_deg
    );

    // Construction starts the neutral pulse train (the ESC's arming signal).
    let esc = Esc::new(esc_ch, esc_cfg);
    info!(
        "ESC on G1: neutral={}µs fwd={}µs rev={}µs deadzone={} arm={}ms rev-coast={}ms",
        esc_cfg.neutral_us,
        esc_cfg.forward_span_us,
        esc_cfg.reverse_span_us,
        esc_cfg.deadzone,
        esc_cfg.arm_ms,
        cfg.reverse_coast_ms
    );

    let mut chassis = Chassis::new(steering, esc, cfg);
    info!("Throttle: left stick Y → ESC (mix off, kick off)");

    // ── USB OTG host on GPIO19 (D-) / GPIO20 (D+) ────────────────────
    let usb = Usb::new_fs(peripherals.USB_FS, peripherals.GPIO20, peripherals.GPIO19);
    let mut session = UsbSession::new(Driver::new(usb));
    session.run(&mut chassis, &mut log, &mut wdt).await
}
