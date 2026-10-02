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
//! - **Steering servo** MG996R on **G14** (LEDC Timer0/Ch0, 50 Hz) — same pin on
//!   both cars, so a harness built for the other one carries it over unchanged
//! - **ESC throttle** on **G13** (LEDC Timer2/Ch1) — **temporarily borrowed from
//!   the other car's PWMA pin** so its adapter harness works as-is; the design
//!   pin is G1/Timer0-Ch1 (plan §3.6). See the swap note in `main` before
//!   moving it back, and never flash the other bin while the ESC is on this pin.
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
///
/// Bench-measured: at 1500 µs the wheels sat slightly right, and the front
/// tie-rod was re-adjusted mechanically, leaving a very slight offset. 2° to the
/// left (~11 µs) is the residual; nudge by ±1° if it still reads off on the car.
const CENTER_TRIM_DEG: i32 = 2;

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

    // ── 50 Hz RC outputs ─────────────────────────────────────────────
    // Timer0/Ch0 → G14: the steering servo (same pin as the other car).
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

    // Timer2/Ch1 → G13: the ESC throttle.
    //
    // **TEMPORARY PIN.** The design pin is G1 (plan §3.6) on Timer0/Ch1, which
    // keeps this output off the other car's PWMA pin so both harnesses can be
    // wired at once. G13 is that PWMA pin, borrowed here because the adapter
    // harness was built for the other car. Moving back is: this pin back to
    // `peripherals.GPIO1`, this channel back to Timer0 (Ch1), and its timer left
    // as-is when the two outputs share one timer again.
    //
    // Timer2/Ch1 is the one LEDC combination on this board known to drive PWMA
    // (Timer1 and Channel2 were found to produce no output). Note the same
    // physical pin carries a completely different signal depending on which bin
    // is flashed: a 50 Hz RC pulse here, a 10 kHz duty-cycle motor PWM in the
    // other car's firmware. So while the ESC is on G13, flash with
    // `cargo run --bin rc10` — a bare `cargo run` puts the other bin's 10 kHz
    // signal on this line.
    let mut esc_timer = ledc.timer::<LowSpeed>(timer::Number::Timer2);
    esc_timer.configure(rc_pwm::timer_config()).unwrap();

    let mut esc_ch = ledc.channel(channel::Number::Channel1, peripherals.GPIO13);
    esc_ch
        .configure(channel::config::Config {
            timer: &esc_timer,
            duty_pct: 0, // `Esc::new` writes neutral immediately after this
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // ── ESC parameters ───────────────────────────────────────────────
    let esc_cfg = EscConfig {
        speed_full: MOTOR_MAX_SPEED,
        // Bench-measured (servo-test sections B/E/F): the unit arms here,
        // forward runs from ~1550 µs up, and reverse is proportional from ~1450
        // down to at least 1050 µs.
        neutral_us: 1500,
        // First ground runs only: the measured forward band starts just above
        // 1500 µs, so a full stick here is ~1750 µs ≈ half of nominal travel
        // (4WD + 540 on 2S is already quick at that). Raise it once the car is
        // predictable — 500 µs is the nominal full-throttle endpoint.
        forward_span_us: 250,
        // The reverse band is at least 1450..1050, so full reverse
        // (neutral − 350 = 1150 µs) sits comfortably inside it.
        reverse_span_us: 350,
        // Must swallow what the stick still reports at rest *after*
        // `ly_deadzone: 3` has been applied: ~5 counts of residual + 3 counts of
        // deadzone = 8/128 of the range ≈ 256 units. Read the resting `ly=` off
        // the console (it prints every 100 ms) and adjust — too narrow and the
        // ESC never sees neutral, which kills reverse in a way that looks
        // exactly like dead hardware.
        deadzone: 256,
        // Continuous neutral hold after power-up/reset before throttle is
        // accepted; the criterion is the unit's own arm confirmation.
        arm_ms: 2000,
    };

    // ── Chassis parameters ───────────────────────────────────────────
    let cfg = ControlConfig {
        // Steering limits are *per side* because this linkage is not symmetric:
        // measured with `servo-test` H, at ±75° of servo travel *both* ends are
        // still 3–5° short of their mechanical stop and nothing buzzes, i.e. the
        // asymmetry lives in the knuckle stops, not in the servo's range. The
        // wheels turn further to the right, so the right limit is the one pulled
        // in — reducing the larger side is the safe direction, raising the
        // smaller one eats its margin.
        // Left 72 with the +2° trim sends 1911 µs — inside the tested 1084…1916 µs
        // band, and that band's upper end is why the left cannot go higher: +75°
        // would be 1927 µs, past anything verified.
        // More *maximum* angle is not available in software; a longer servo arm
        // (or a shorter knuckle arm) would only make the same maximum arrive
        // sooner, i.e. more responsive mid-stick.
        steer_max_left_deg: 72,
        steer_max_right_deg: 68,
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
        // The ESC's reverse protocol, measured on the bench (servo-test F/G). It
        // is *three* phases, not two: after a forward demand the first
        // sub-neutral pulse is a brake, and only a second one — after neutral —
        // is reverse.
        //   no brake pulse + 600 ms neutral + reverse → nothing
        //   400 ms brake  + 600 ms neutral + reverse → reverse at every depth
        //                                              from 1450 down to 1050 µs
        //   100 ms brake  + 600 ms neutral + reverse → reverse (50 ms: nothing)
        //   all brake durations tested + 50 ms neutral → reverse
        // So the measured minimums are brake ≤100 ms and neutral ≤50 ms; the
        // values below carry a margin because "too short" fails as *no reverse
        // at all*, which is the most confusing failure this car can produce.
        // The brake pulse is the 1400 µs that F1 used (a *shallow* one — its
        // depth does not need to follow the stick), expressed as a speed so a
        // change to the spans above cannot leave it in the wrong place.
        reversal_brake_speed: esc_cfg.speed_for_pulse(1400),
        reversal_brake_ms: 150,
        reversal_neutral_ms: 100,
        // Full throttle for 2 s is ~16 m on this car; this is a safety limit.
        failsafe_timeout_ms: 2000,
    };

    let steering = Steering::new(
        servo_ch,
        CENTER_TRIM_DEG,
        cfg.steer_max_left_deg.max(cfg.steer_max_right_deg),
    );
    info!(
        "Steering: offset={}°  limits L{}/R{}°  right stick → steer (G14)",
        CENTER_TRIM_DEG, cfg.steer_max_left_deg, cfg.steer_max_right_deg
    );

    // Construction starts the neutral pulse train (the ESC's arming signal).
    let esc = Esc::new(esc_ch, esc_cfg);
    info!(
        "ESC on G13 (temp pin): neutral={}µs fwd={}µs rev={}µs deadzone={} arm={}ms",
        esc_cfg.neutral_us,
        esc_cfg.forward_span_us,
        esc_cfg.reverse_span_us,
        esc_cfg.deadzone,
        esc_cfg.arm_ms
    );
    info!(
        "ESC reverse protocol: brake {} ({}ms) → neutral {}ms → ramp",
        cfg.reversal_brake_speed, cfg.reversal_brake_ms, cfg.reversal_neutral_ms
    );

    let mut chassis = Chassis::new(steering, esc, cfg);
    info!("Throttle: left stick Y → ESC (mix off, kick off)");

    // ── USB OTG host on GPIO19 (D-) / GPIO20 (D+) ────────────────────
    let usb = Usb::new_fs(peripherals.USB_FS, peripherals.GPIO20, peripherals.GPIO19);
    let mut session = UsbSession::new(Driver::new(usb));
    session.run(&mut chassis, &mut log, &mut wdt).await
}
