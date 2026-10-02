#![no_std]
#![no_main]
#![deny(clippy::large_stack_frames)]

//! 50 Hz RC pulse calibration tool — servo endpoints *and* ESC endpoints/dwell.
//!
//! ⚠ **Put the drive wheels off the ground before running this.** From section
//! B on, it commands the ESC, and the motor will spin. If it panics it resets and
//! starts the whole sequence again about 20 s later, so do not leave it
//! unattended with the wheels down.
//!
//! Pins (both on LEDC Timer0, 50 Hz):
//! - **G14** — steering servo channel (Timer0/Ch0)
//! - **G1** — ESC throttle channel (Timer0/Ch1)
//!
//! ⚠ **The throttle pin is currently moved to G13** (LEDC Timer2/Ch1), the other
//! car's PWMA pin, because the adapter harness routes it there — see the swap
//! note at the LEDC setup in `main`. It has to match `rc10.rs` or this tool
//! drives a pin with nothing on it.
//!
//! Only the device being calibrated needs to be connected. Until its own section
//! starts, the other channel emits **nothing** (duty 0), which is not the same as
//! neutral: an ESC reads silence as "no signal". That is why section B's arming
//! hold is the first thing the ESC ever sees from this tool.
//!
//! This tool is deliberately independent of the firmware's *drivers*
//! ([`Steering`](jocar_steer::steering::Steering), [`Esc`](jocar_steer::esc::Esc)):
//! it has to emit pulses those drivers would refuse to, and it must not inherit
//! the deadzone and dwell logic it is being used to calibrate. It does share the
//! RC-pulse timing with them, because that is about the LEDC hardware rather than
//! about driver policy (see `jocar_steer::rc_pwm`).
//!
//! # What it does, and the criterion for each section
//!
//! **A. Servo sweep (G14)** — 1500 → 1000 → 2000 → 1500 µs in 50 µs steps, each
//! step printed and held 400 ms, with only half a second at each end.
//! *Criterion:* where does the horn reach the mechanical stop (buzzing = too
//! far), and is 1500 µs really "wheels straight"? → `CENTER_TRIM_DEG` and
//! `steer_max_deg` in the bin. Do not let it sit at a stop: a stalled servo is
//! the one thing the local capacitor cannot save (plan §3.5).
//!
//! **B. ESC arm, then cold-start reverse, then forward probe (G1)** — neutral for
//! 5 s (the arm hold), then *straight to reverse* for 1.5 s, then
//! 1550/1600/1650/1700/1800 µs forward for 1 s each with 2 s of neutral in between.
//! *Criterion:* does reverse engage **without any preceding forward command**?
//! That is the plan's untested prediction (§3.3): the latch should be set by a
//! forward *demand*, and arming sends none. If it does not engage here, the same
//! is true in the firmware and a forward blip is the workaround to document.
//! Then: the smallest pulse that reliably turns the wheels, and which way →
//! `neutral_us`, `forward_span_us`.
//!
//! **E. Reverse-protocol probe (G1)** — three patterns, one per candidate
//! protocol: a deep pulse straight from neutral; forward → neutral → deep pulse;
//! and forward → first-sub-neutral-pulse (brake) → neutral → sub-neutral again.
//! *Criterion:* which pattern produces reverse at all, plus two readouts the code
//! cannot get — the ESC's own LED (lit = it is driving *something*, off = it
//! reads the pulse as neutral) and the wheels by hand (braked = a brake, free =
//! neutral). This runs before C because the first bench run produced no reverse
//! under either of the first two patterns, and in that state a dwell sweep
//! measures nothing.
//!
//! **C. Reverse-latch dwell sweep (G1)** — forward 1650 µs (2 s) → neutral for T
//! → reverse (1.5 s) → neutral (2 s), for T = 100…400 ms and for **two** reverse
//! magnitudes (1400 and 1200 µs).
//! *Criterion:* the smallest T at which reverse engages. Two magnitudes because
//! otherwise "the latch is still held" cannot be told apart from "the reverse
//! pulse is too small" — and this class of ESC often has a shorter reverse range.
//! The firmware holds demand-side neutral for `ControlConfig::reverse_coast_ms`;
//! this section measures how long that has to be (with margin). It runs *after*
//! section E, because it only means something once reverse is known to engage.
//!
//! **D. Park** — both channels to 1500 µs. Never leave the tool with a throttle
//! still commanded.
//!
//! Sections A, C, E and F are switched by the `RUN_*` constants below
//! `NEUTRAL_US`: A wears the steering linkage against its stop, C assumes a
//! reverse protocol that E has to establish first, E has already identified that
//! protocol, and F is the current question (minimal recipe + accepted depths).

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    Ledc, LowSpeed,
    channel::{self, ChannelHW, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_println as _;
use jocar_steer::rc_pwm::{self, pulse_to_counts};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("Panic: {}", defmt::Display2Format(info));
    esp_hal::system::software_reset()
}

const NEUTRAL_US: u32 = 1500;

/// Section A drives the steering linkage into its mechanical stop on purpose.
/// Turn it on when calibrating the servo; leave it off afterwards so repeat runs
/// of the ESC sections do not hold the servo against that stop every time.
const RUN_SERVO_SWEEP: bool = false;

/// Section C measures a dwell *assuming* reverse engages on
/// "forward → neutral → below-neutral". Leave it off until section E has shown
/// which protocol this ESC actually speaks — otherwise it is 45 s of nothing.
const RUN_DWELL_SWEEP: bool = false;

/// Section E (which reverse protocol?) has answered: the brake-then-reverse
/// pattern reaches reverse, and so does a cold start. Off by default now.
const RUN_PROTOCOL_PROBE: bool = false;

/// Section F (what is the *minimal* recipe, and which depths are accepted?) is
/// the current question.
const RUN_RECIPE_PROBE: bool = true;

/// Set a pulse and hold it, announcing the value on the console (the console is
/// the only readout: there is no input device attached to this tool).
async fn hold<C: ChannelHW>(ch: &mut C, pulse_us: u32, ms: u64, what: &str) {
    ch.set_duty_hw(pulse_to_counts(pulse_us));
    info!("  {} µs  ({} ms)  {}", pulse_us, ms, what);
    Timer::after(Duration::from_millis(ms)).await;
}

/// A. Servo endpoints. `what` labels the two ends of the travel.
#[allow(
    clippy::large_stack_frames,
    reason = "per-step console formatting across a sweeping loop; this tool runs standalone \
    (no USB stack, no control loop) so it has the stack to itself"
)]
async fn servo_sweep<C: ChannelHW>(ch: &mut C) {
    info!("A. servo sweep on G14 — watch for the mechanical stop (buzzing = too far)");
    hold(ch, NEUTRAL_US, 1500, "centre: are the wheels straight?").await;

    let mut us = NEUTRAL_US;
    while us > 1000 {
        us -= 50;
        hold(ch, us, 400, "").await;
    }
    hold(ch, 1000, 500, "short-pulse end — half a second only, do not stall it").await;

    while us < 2000 {
        us += 50;
        hold(ch, us, 400, "").await;
    }
    hold(ch, 2000, 500, "long-pulse end — half a second only, do not stall it").await;

    hold(ch, NEUTRAL_US, 1000, "back to centre").await;
}

/// B. Arm the ESC, test cold-start reverse, then find the smallest pulse that
/// turns the wheels.
#[allow(
    clippy::large_stack_frames,
    reason = "same per-step console formatting as the servo sweep"
)]
async fn esc_arm_and_probe<C: ChannelHW>(esc: &mut C) {
    info!("B. ESC on G1 — arming hold first, then cold-start reverse, then forward");
    hold(esc, NEUTRAL_US, 5000, "neutral: the ESC arms here (it should beep)").await;

    // The plan's prediction, tested before any forward demand exists.
    hold(esc, 1400, 1500, "COLD-START reverse: does it engage with no prior forward?").await;
    hold(esc, NEUTRAL_US, 2000, "neutral").await;

    for us in [1550, 1600, 1650, 1700, 1800] {
        hold(esc, us, 1000, "forward probe: do the wheels turn?").await;
        hold(esc, NEUTRAL_US, 2000, "neutral").await;
    }
}

/// E. Which reverse protocol does this ESC speak?
///
/// First run showed *no* reverse under either "cold" or "forward → neutral →
/// below-neutral", which is exactly the case the plan says not to blame on the
/// firmware. Three candidate protocols, one pattern each. The operator has two
/// readouts the code cannot get: the ESC's own LED (lit = the ESC is driving
/// *something*, off = it reads the pulse as neutral) and the wheels by hand.
#[allow(
    clippy::large_stack_frames,
    reason = "same per-step console formatting as the servo sweep"
)]
async fn reverse_protocol_probe<C: ChannelHW>(esc: &mut C) {
    info!("E. reverse protocol on G1 — for every pulse below neutral, watch the ESC LED");
    info!("   and try turning the wheels by hand: braked = the ESC IS driving (a brake),");
    info!("   free = it reads neutral, turning backwards = reverse.");

    // E1: a deep pulse straight from neutral, no forward demand before it.
    // Distinguishes "the reverse range starts lower than we probed" from
    // "a below-neutral demand is not reverse at all".
    hold(esc, NEUTRAL_US, 2000, "neutral").await;
    hold(esc, 1050, 2500, "E1 deep pulse from neutral — reverse? LED? braked?").await;
    hold(esc, NEUTRAL_US, 2000, "neutral").await;

    // E2: forward, brief neutral, then a deep pulse. Same shape as section C but
    // at the far end of the reverse range.
    hold(esc, 1700, 2000, "E2 forward").await;
    hold(esc, NEUTRAL_US, 500, "neutral").await;
    hold(esc, 1050, 2500, "E2 deep pulse after forward+neutral — reverse? LED? braked?").await;
    hold(esc, NEUTRAL_US, 2000, "neutral").await;

    // E3: the double-tap / brake-then-reverse pattern. On an ESC whose first
    // sub-neutral demand *is* the brake, this is the only pattern that reaches
    // reverse — and section C never produces it.
    hold(esc, 1700, 2000, "E3 forward").await;
    hold(esc, 1400, 600, "E3 first sub-neutral pulse (the brake, if that is the protocol)").await;
    hold(esc, NEUTRAL_US, 600, "E3 neutral (latch-release window)").await;
    hold(esc, 1400, 2500, "E3 sub-neutral again — reverse NOW? LED? wheels?").await;
    hold(esc, NEUTRAL_US, 2000, "neutral").await;
}

/// F. What is the *minimal* reverse recipe, and which pulse depths are accepted?
///
/// Established by the first two bench runs:
/// - 1400 µs from a fresh arm reverses (so the plan's prediction holds: the
///   direction latch is set by a forward *demand*, and arming sends none);
/// - 1050 µs never reversed, in any shape;
/// - "forward → neutral(≤400 ms) → 1400" (the first run's gate sweep) did not,
///   while "forward → 1400 → neutral(600 ms) → 1400" did.
///
/// So two things are still confounded — the extra sub-neutral pulse (a brake, if
/// that is the protocol) and the longer neutral window — and one thing is
/// unknown but load-bearing: the band of depths the ESC accepts as reverse. The
/// firmware's planned full-reverse pulse is 1150 µs, which may sit outside it.
#[allow(
    clippy::large_stack_frames,
    reason = "same per-step console formatting as the servo sweep"
)]
async fn reverse_recipe_probe<C: ChannelHW>(esc: &mut C) {
    info!("F. reverse recipe on G1 — note WHICH of these reverses; any motion counts");

    // F2 first: if the brake pulse turns out to be unnecessary, the firmware
    // needs no three-phase state machine at all — only a longer neutral window.
    for (brake_ms, label) in [
        (0u64, "F2a NO brake pulse, 600 ms neutral — does the longer window alone do it?"),
        (150, "F2b brake pulse 150 ms + 600 ms neutral — and this one?"),
    ] {
        hold(esc, 1700, 2000, "forward (sets the latch)").await;
        if brake_ms > 0 {
            hold(esc, 1400, brake_ms, "brake pulse").await;
        }
        hold(esc, NEUTRAL_US, 600, "neutral (the latch window)").await;
        hold(esc, 1400, 2000, label).await;
        hold(esc, NEUTRAL_US, 1500, "neutral").await;
    }

    // F1: which depths are accepted? The gate is the one that has worked
    // (1400 for 400 ms, then 600 ms neutral).
    info!("F1 depth sweep — gate: 1400 for 400 ms, then 600 ms neutral");
    for probe in [1450u32, 1400, 1350, 1300, 1250, 1200, 1150, 1100, 1050] {
        hold(esc, 1700, 2000, "forward").await;
        hold(esc, 1400, 400, "brake").await;
        hold(esc, NEUTRAL_US, 600, "neutral").await;
        hold(esc, probe, 1500, "probe: does THIS depth reverse?").await;
        hold(esc, NEUTRAL_US, 1500, "neutral").await;
    }
}

/// C. How long must neutral be held before a forward→reverse command works?
///
/// Run *after* section E: this sweep only means something once reverse is known
/// to engage at all.
#[allow(
    clippy::large_stack_frames,
    reason = "same per-step console formatting as the servo sweep"
)]
async fn reverse_dwell_sweep<C: ChannelHW>(esc: &mut C) {
    info!("C. reverse latch on G1 — forward → neutral(T) → reverse, two magnitudes");
    for reverse_us in [1400u32, 1200] {
        info!(
            "== reverse pulse {} µs (a too-small pulse looks the same as a held latch)",
            reverse_us
        );
        for t in [100u64, 200, 300, 400] {
            info!("-- T = {} ms: forward 2 s, neutral {} ms, reverse 1.5 s", t, t);
            hold(esc, 1650, 2000, "forward").await;
            hold(esc, NEUTRAL_US, t, "neutral (the dwell under test)").await;
            hold(esc, reverse_us, 1500, "reverse: did it engage?").await;
            hold(esc, NEUTRAL_US, 2000, "neutral").await;
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "main is the entry point; stack budget is generous here"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Reserve bootstrapping / flash pins (never touch these).
    let _ = peripherals.GPIO0;
    let _ = peripherals.GPIO3;
    let _ = peripherals.GPIO45;
    let _ = peripherals.GPIO46;

    // Flash / PSRAM (octal WROOM-1: G26–G37)
    let _ = peripherals.GPIO26;
    let _ = peripherals.GPIO27;
    let _ = peripherals.GPIO28;
    let _ = peripherals.GPIO29;
    let _ = peripherals.GPIO30;
    let _ = peripherals.GPIO31;
    let _ = peripherals.GPIO32;
    let _ = peripherals.GPIO33;
    let _ = peripherals.GPIO34;
    let _ = peripherals.GPIO35;
    let _ = peripherals.GPIO36;
    let _ = peripherals.GPIO37;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("servo-test: Embassy initialized");

    let mut ledc = Ledc::new(peripherals.LEDC);
    rc_pwm::init(&mut ledc);

    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
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
    // **TEMPORARY PIN — must match `rc10.rs`.** The firmware's design pin is G1
    // on Timer0/Ch1; G13 is the other car's PWMA pin, borrowed while its adapter
    // harness is in use. If these two disagree the tool drives a pin with nothing
    // on it and the throttle side looks dead. Moving back means changing this
    // pin, this channel back to Timer0/Ch1, and `rc10.rs` with it.
    let mut esc_timer = ledc.timer::<LowSpeed>(timer::Number::Timer2);
    esc_timer.configure(rc_pwm::timer_config()).unwrap();

    let mut esc_ch = ledc.channel(channel::Number::Channel1, peripherals.GPIO13);
    esc_ch
        .configure(channel::config::Config {
            timer: &esc_timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    info!("LEDC ready: 50 Hz — Ch0/Timer0 → G14 (servo), Ch1/Timer2 → G13 (ESC, temporary)");
    info!("WHEELS OFF THE GROUND. Starting in 3 s.");
    Timer::after(Duration::from_millis(3000)).await;

    if RUN_SERVO_SWEEP {
        servo_sweep(&mut servo_ch).await;
    } else {
        info!("(section A skipped: RUN_SERVO_SWEEP = false — servo already measured)");
    }
    esc_arm_and_probe(&mut esc_ch).await;
    if RUN_PROTOCOL_PROBE {
        reverse_protocol_probe(&mut esc_ch).await;
    } else {
        info!("(section E skipped: RUN_PROTOCOL_PROBE = false — protocol already identified)");
    }
    if RUN_RECIPE_PROBE {
        reverse_recipe_probe(&mut esc_ch).await;
    }
    if RUN_DWELL_SWEEP {
        reverse_dwell_sweep(&mut esc_ch).await;
    } else {
        info!("(section C skipped: RUN_DWELL_SWEEP = false — needs a known reverse protocol)");
    }

    // D. Park. Never leave a throttle commanded.
    servo_ch.set_duty_hw(pulse_to_counts(NEUTRAL_US));
    esc_ch.set_duty_hw(pulse_to_counts(NEUTRAL_US));
    info!("done — both channels parked at 1500 µs. Record the values above.");

    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
