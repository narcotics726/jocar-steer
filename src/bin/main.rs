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
//! Current hardware:
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
//! No dual mode: one motor + one driveshaft, so `DriveMode::Diff` and the
//! second TB6612 channel are not used here.

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb_host::{
    BusRoute, BusState, class::hub::{HubEvent, HubHandler}, handler::HandlerEvent,
};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    LSGlobalClkSource, Ledc, LowSpeed,
    channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::time::Rate;
use esp_hal::usb::otg::{Usb, embassy_usb_host::Driver};

use jocar_steer::control;
use jocar_steer::flash_log;
use jocar_steer::steering::Steering;
use jocar_steer::tb6612::Tb6612Single;
use jocar_steer::usb_gamepad::{GamepadHost, GamepadState};
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

// ── Control configuration (single motor + steering) ──────────────────

/// Static center offset in degrees to cancel residual servo mounting error.
/// Re-calibrate for the new chassis (was 3 on the old car).
const CENTER_TRIM_DEG: i32 = 0;

/// Steer-throttle mixing: at full steering lock the speed limit is scaled
/// to `max_speed * (1 - STEER_MIX_NUM / STEER_MIX_DEN)`. Without this, the
/// motor fights the front-wheel scrub during turns (no differential,
/// near-parallel steering geometry), which is the main source of stall
/// current and motor heat on this chassis.
const STEER_MIX_NUM: i32 = 1;
const STEER_MIX_DEN: i32 = 2; // cut 50 % at full lock

/// Start-kick duration, in milliseconds.
///
/// The kick briefly outputs the full allowed speed when the throttle jumps
/// from rest, so the motor can overcome static friction immediately instead
/// of waiting for the slew ramp. The slew state still advances underneath, so
/// the hand-back to the slew value is smooth.
///
/// A/B switch for this chassis: `0` disables the kick entirely. It is a
/// documented stopgap for stall-current heat (the structural fix is more gear
/// reduction), so it is being re-validated on the car rather than assumed.
const KICK_DURATION_MS: u64 = 0;
/// Minimum |target| (fraction of `max_speed`) that triggers a kick — keeps
/// light stick touches from causing full-speed bursts.
const KICK_MIN_NUM: i32 = 3;
const KICK_MIN_DEN: i32 = 10; // 30 %

/// How many times to retry enumerating a directly-attached device before
/// falling back to waiting for a fresh connection event.
const ENUM_ATTEMPTS: u8 = 3;

/// How long to wait for a device after one was already seen, before resetting
/// the chip to rebuild the USB host stack (see the main loop).
const RECONNECT_TIMEOUT_MS: u64 = 5000;

/// One-shot start-kick state, timed in physical time.
struct StartKick {
    /// Deadline of the active burst; `None` when no burst is running.
    kick_until: Option<Instant>,
    prev_was_zero: bool,
}

impl StartKick {
    fn new() -> Self {
        Self {
            kick_until: None,
            prev_was_zero: true,
        }
    }

    fn reset(&mut self) {
        self.kick_until = None;
        self.prev_was_zero = true;
    }

    /// Returns `Some(speed)` while the kick burst is active, `None` otherwise.
    /// `limit` is the current speed ceiling (mix already applied).
    fn tick(&mut self, target: i32, limit: i32, now: Instant) -> Option<i32> {
        if KICK_DURATION_MS == 0 {
            return None; // disabled (A/B switch)
        }
        let big_enough = target.abs() * KICK_MIN_DEN >= limit * KICK_MIN_NUM;
        if target == 0 || !big_enough {
            self.kick_until = None;
            self.prev_was_zero = true;
            return None;
        }
        if self.prev_was_zero {
            self.prev_was_zero = false;
            self.kick_until = Some(now + Duration::from_millis(KICK_DURATION_MS));
        }
        match self.kick_until {
            Some(until) if now < until => Some(limit * target.signum()),
            _ => None,
        }
    }
}

// ── Helpers shared by the direct and hub read loops ───────────────────

/// Apply one gamepad report: right stick → steering, left stick → motor.
///
/// Generic over the concrete LEDC channel types so both the direct and the
/// hub-fallback read loops can share the exact same control policy.
/// `dt_us` is the elapsed time since the previous report and drives the
/// time-based slew limits; `now` is used for the kick timer.
#[allow(
    clippy::too_many_arguments,
    reason = "control-policy inputs; folded into a Chassis struct in the next phase"
)]
fn drive<S, M>(
    gp: &GamepadState,
    steering: &mut Steering<S>,
    motors: &mut Tb6612Single<M>,
    motor_slew: &mut control::MotorSlew,
    kick: &mut StartKick,
    cfg: &control::ControlConfig,
    dt_us: u64,
    now: Instant,
) where
    S: esp_hal::ledc::channel::ChannelHW,
    M: esp_hal::ledc::channel::ChannelHW,
{
    let steer = control::rx_to_deg(gp.rx, cfg.rx_deadzone, cfg.steer_max_deg);
    steering.set_target(steer);
    steering.update(cfg.steer_slew_rate_deg_s, dt_us);

    // Steer-throttle mixing: speed ceiling shrinks linearly with steering
    // angle, reaching max_speed * (1 - NUM/DEN) at full lock.
    let steer_cut = steer.abs() * cfg.motor_max_speed * STEER_MIX_NUM
        / (cfg.steer_max_deg * STEER_MIX_DEN);
    let speed_limit = (cfg.motor_max_speed - steer_cut).max(0);

    let throttle = control::ly_to_speed(gp.ly, cfg.ly_deadzone, cfg.motor_max_speed);
    let target = throttle.clamp(-speed_limit, speed_limit);

    // Start kick overrides the slew output briefly; the slew still advances
    // underneath so the hand-back is smooth.
    let (slew_out, _) = motor_slew.update(target, target, dt_us);
    let m = kick.tick(target, speed_limit, now).unwrap_or(slew_out);
    motors.set_motor(m);
}

/// Stop everything: reset slew state, coast the motor, center the servo.
fn stop<S, M>(
    steering: &mut Steering<S>,
    motors: &mut Tb6612Single<M>,
    motor_slew: &mut control::MotorSlew,
    kick: &mut StartKick,
) where
    S: esp_hal::ledc::channel::ChannelHW,
    M: esp_hal::ledc::channel::ChannelHW,
{
    motor_slew.reset();
    kick.reset();
    motors.coast();
    steering.center();
}

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

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
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

    // ── Servo on GPIO14 via LEDC (50 Hz) ─────────────────────────────
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

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

    // ── Control configuration ────────────────────────────────────────
    let cfg = control::ControlConfig {
        // 30° is the usable limit on this chassis — past it the front
        // wheels scrub so hard (no rear diff) the motor stalls.
        steer_max_deg: 30,
        // Full speed is safe: the N30 is 12 V-rated and the battery is 2S
        // (7.4 V), so we are under-voltage, not over. The motor heats from
        // stall current during turns, which the steer-throttle mix mitigates.
        motor_max_speed: 4095,
        // Time-based now. The previous 512/tick was silently scaled by the
        // report rate; ~15.5 k/s restores the designed 33 ms-tick behaviour.
        motor_slew_rate_speed_s: 15_500,
        steer_slew_rate_deg_s: 400, // ≈ the designed 8°/33 ms
        rx_deadzone: 3,
        ly_deadzone: 3,
    };

    let mut steering = Steering::new(ch, CENTER_TRIM_DEG, cfg.steer_max_deg);
    info!(
        "Steering: offset={}°  max={}°  right stick → steer (G14)",
        CENTER_TRIM_DEG, cfg.steer_max_deg
    );

    // ── TB6612 channel A: AIN1=G11, AIN2=G12, STBY=G10, PWMA=G13 ─────
    let mut motors = Tb6612Single::new(
        peripherals.GPIO11, // AIN1
        peripherals.GPIO12, // AIN2
        peripherals.GPIO10, // STBY
        mch,
    );
    motors.enable();
    info!("Motor enabled: left stick Y → throttle");

    let mut motor_slew = control::MotorSlew::new(cfg.motor_slew_rate_speed_s);
    let mut kick = StartKick::new();

    // ── USB OTG host on GPIO19 (D-) / GPIO20 (D+) ────────────────────
    let usb = Usb::new_fs(peripherals.USB_FS, peripherals.GPIO20, peripherals.GPIO19);
    static BUS_STATE: BusState = BusState::new();
    let (mut bus_ctrl, bus) = embassy_usb_host::bus(Driver::new(usb), &BUS_STATE);

    // Set once a device has been seen: from then on, losing it and not
    // getting it back is treated as a wedged USB host.
    let mut had_session = false;
    loop {
        let speed = if had_session {
            // After a device is removed the root port is not always re-armed
            // for a re-attach (observed: no further connect events), and the
            // hub path can wait on its event queue forever too. A chip reset
            // rebuilds the whole USB host stack, so it is the only reliable
            // way back without a physical replug. Guarded by `had_session` so
            // a car booted with nothing plugged in waits instead of
            // reboot-looping.
            match embassy_time::with_timeout(
                Duration::from_millis(RECONNECT_TIMEOUT_MS),
                bus_ctrl.wait_for_connection(),
            )
            .await
            {
                Ok(s) => s,
                Err(_) => {
                    error!(
                        "no device for {} ms after a loss — resetting to recover USB",
                        RECONNECT_TIMEOUT_MS
                    );
                    log.record(flash_log::EV_RESET, format_args!("usb lost"));
                    // Replay the persisted log now: while the direct adapter
                    // occupies the OTG port the console is unattached, so the
                    // history is read after the user swaps the cable back.
                    log.dump();
                    esp_hal::system::software_reset()
                }
            }
        } else {
            // No device has been seen yet. Wait indefinitely — a car booted
            // with nothing attached must not reboot-loop — but keep replaying
            // the persisted log every period so that attaching the console
            // later still shows what a previous session recorded.
            loop {
                match embassy_time::with_timeout(
                    Duration::from_millis(RECONNECT_TIMEOUT_MS),
                    bus_ctrl.wait_for_connection(),
                )
                .await
                {
                    Ok(s) => break s,
                    Err(_) => log.dump(),
                }
            }
        };
        had_session = true;
        info!("Device connected at {:?}", speed);
        log.record(flash_log::EV_CONNECTED, format_args!("{:?}", speed));

        // Let the device finish its own power-up before the first control
        // transfer. A hub port inserts this delay implicitly; a direct
        // connection does not, which is one reason direct enumeration can
        // stall on this receiver.
        Timer::after(Duration::from_millis(200)).await;

        let mut config_buf = [0u8; 256];
        // Enumerate with in-place retries. Falling back to
        // `wait_for_connection` on failure wedges the loop: that call waits
        // for a *new* connection event, and a device that stays plugged in
        // never produces one — the car then needed a physical replug.
        // Each attempt stays bounded so a non-responding device cannot hang
        // the loop forever (seen as a stuck state on a flaky power rail).
        let mut enum_result: Result<_, ()> = Err(());
        for attempt in 1..=ENUM_ATTEMPTS {
            let r = embassy_time::with_timeout(
                Duration::from_secs(5),
                bus.enumerate(BusRoute::Direct(speed), &mut config_buf),
            )
            .await;
            // Retry on a timeout as well as on a transfer error (e.g. STALL).
            let succeeded = matches!(r, Ok(Ok(_)));
            match &r {
                Ok(Err(e)) => {
                    error!("Enumeration attempt {} failed: {:?}", attempt, e);
                    log.record(
                        flash_log::EV_ENUM_FAIL,
                        format_args!("{:?} try{}", e, attempt),
                    );
                }
                Err(_) => {
                    error!("Enumeration attempt {} timed out after 5s", attempt);
                    log.record(
                        flash_log::EV_ENUM_FAIL,
                        format_args!("timeout try{}", attempt),
                    );
                }
                Ok(Ok(_)) => {}
            }
            enum_result = r.map_err(|_| ());
            if succeeded {
                break;
            }
            Timer::after(Duration::from_millis(200)).await;
        }
        let enum_result = match enum_result {
            Ok(r) => r,
            Err(_) => {
                error!(
                    "Enumeration failed {} times — waiting for a reconnect",
                    ENUM_ATTEMPTS
                );
                Timer::after(Duration::from_millis(1000)).await;
                continue;
            }
        };
        match enum_result {
            Ok((enum_info, config_len)) => {
                info!(
                    "Enumerated: VID={:04x} PID={:04x}",
                    enum_info.device_desc.vendor_id,
                    enum_info.device_desc.product_id
                );
                log.record(
                    flash_log::EV_ENUM_OK,
                    format_args!(
                        "vid={:04x} pid={:04x}",
                        enum_info.device_desc.vendor_id,
                        enum_info.device_desc.product_id
                    ),
                );

                match GamepadHost::new(&bus, &config_buf[..config_len], &enum_info) {
                    Ok(h) => {
                        let mut hid = h;
                        log.record(flash_log::EV_IFACE, format_args!("iface0 ok"));
                        log.record(flash_log::EV_SESSION, format_args!("direct reading"));
                        info!("iface0 ready — reading gamepad reports");

                        let mut buf = [0u8; 64];
                        let mut last_log = Instant::now();
                        let mut last_report = Instant::now();
                        // Diagnostics: prove whether reports arrive at all, and
                        // whether they parse (each logged once per session).
                        let mut first_report_logged = false;
                        let mut parse_fail_logged = false;

                        loop {
                            match embassy_time::with_timeout(
                                Duration::from_millis(2000),
                                hid.read(&mut buf),
                            )
                            .await
                            {
                                Ok(Ok(n)) if n > 0 => {
                                    let now = Instant::now();
                                    let dt_us = (now - last_report).as_micros();
                                    last_report = now;
                                    if let Some(gp) = GamepadState::parse(&buf[..n]) {
                                        if !first_report_logged {
                                            first_report_logged = true;
                                            log.record(
                                                flash_log::EV_FIRST_REPORT,
                                                format_args!("n={}", n),
                                            );
                                        }
                                            drive(
                                                &gp,
                                                &mut steering,
                                                &mut motors,
                                                &mut motor_slew,
                                                &mut kick,
                                                &cfg,
                                                dt_us,
                                                now,
                                            );

                                        if last_log.elapsed() >= Duration::from_millis(100) {
                                            info!(
                                                "lx={} ly={} rx={} ry={} btns={:04x}",
                                                gp.lx, gp.ly, gp.rx, gp.ry, gp.buttons
                                            );
                                            last_log = Instant::now();
                                        }
                                    } else if !parse_fail_logged {
                                        // Reports arrive but the layout is not the
                                        // one this firmware knows (which mode is
                                        // the receiver in?).
                                        parse_fail_logged = true;
                                        let b0 = *buf.first().unwrap_or(&0);
                                        let b1 = *buf.get(1).unwrap_or(&0);
                                        log.record(
                                            flash_log::EV_PARSE_FAIL,
                                            format_args!("n={} b0={:02x} b1={:02x}", n, b0, b1),
                                        );
                                    }
                                }
                                Ok(Ok(_)) => {}
                                Ok(Err(e)) => {
                                    error!("HID read failed: {:?}", e);
                                    log.record(flash_log::EV_READ_ERR, format_args!("{:?}", e));
                                    break;
                                }
                                Err(_) => {
                                    if last_report.elapsed() >= Duration::from_secs(5) {
                                        error!("no reports for 5s — device gone?");
                                        log.record(
                                            flash_log::EV_STALE,
                                            format_args!("no reports 5s"),
                                        );
                                        break;
                                    }
                                }
                            }
                        }
                        stop(&mut steering, &mut motors, &mut motor_slew, &mut kick);
                        // The library requires the application to release the
                        // device address once the device is gone.
                        bus.state().free_address(enum_info.device_address);
                        log.record(flash_log::EV_LOST, format_args!("direct device gone"));
                        info!("Device disconnected, waiting for next");
                    }
                    Err(e) => {
                        // Not a ZD receiver — try registering it as a hub and
                        // service downstream ports.
                        log.record(flash_log::EV_IFACE, format_args!("no iface: {:?}", e));
                        info!("no direct gamepad — trying hub");
                        log.record(flash_log::EV_HUB, format_args!("registering"));
                        match HubHandler::<_, 8>::try_register(&bus, &enum_info).await {
                            Ok(mut hub) => {
                                log.record(
                                    flash_log::EV_HUB,
                                    format_args!("registered, waiting for events"),
                                );
                                info!("hub registered");

                                loop {
                                    match hub.wait_for_event().await {
                                        Ok(HandlerEvent::HandlerEvent(HubEvent::DeviceDetected {
                                            port,
                                            speed,
                                        })) => {
                                            info!("hub port {} connected speed={:?}", port, speed);
                                            let mut cfg_buf = [0u8; 256];
                                            let r = embassy_time::with_timeout(
                                                Duration::from_secs(5),
                                                hub.enumerate_port(&mut cfg_buf, port, speed),
                                            )
                                            .await;
                                            match r {
                                                Ok(Ok((ei, len))) => {
                                                    info!(
                                                        "hub port {} enumerated vid={:04x} pid={:04x}",
                                                        port,
                                                        ei.device_desc.vendor_id,
                                                        ei.device_desc.product_id
                                                    );
                                                    match GamepadHost::new(
                                                        &bus,
                                                        &cfg_buf[..len],
                                                        &ei,
                                                    ) {
                                                        Ok(h) => {
                                                            let mut hid = h;
                                                            log.record(
                                                                flash_log::EV_SESSION,
                                                                format_args!(
                                                                    "hub{} reading",
                                                                    port
                                                                ),
                                                            );
                                                            info!(
                                                                "iface0 ready on hub port {}",
                                                                port
                                                            );

                                                            let mut buf = [0u8; 64];
                                                            let mut last_log = Instant::now();
                                                            let mut last_report = Instant::now();

                                                            loop {
                                                                match embassy_time::with_timeout(
                                                                    Duration::from_millis(2000),
                                                                    hid.read(&mut buf),
                                                                )
                                                                .await
                                                                {
                                                                    Ok(Ok(n)) if n > 0 => {
                                                                        let now =
                                                                            Instant::now();
                                                                        let dt_us = (now
                                                                            - last_report)
                                                                            .as_micros();
                                                                        last_report = now;
                                                                        if let Some(gp) =
                                                                            GamepadState::parse(
                                                                                &buf[..n],
                                                                            )
                                                                        {
                                                                            drive(
                                                                                &gp,
                                                                                &mut steering,
                                                                                &mut motors,
                                                                                &mut motor_slew,
                                                                                &mut kick,
                                                                                &cfg,
                                                                                dt_us,
                                                                                now,
                                                                            );

                                                                            if last_log.elapsed()
                                                                                >= Duration::from_millis(
                                                                                    100,
                                                                                )
                                                                            {
                                                                                info!(
                                                                                    "lx={} ly={} rx={} ry={} btns={:04x}",
                                                                                    gp.lx,
                                                                                    gp.ly,
                                                                                    gp.rx,
                                                                                    gp.ry,
                                                                                    gp.buttons
                                                                                );
                                                                                last_log =
                                                                                    Instant::now();
                                                                            }
                                                                        }
                                                                    }
                                                                    Ok(Ok(_)) => {}
                                                                    Ok(Err(e)) => {
                                                                        error!(
                                                                            "HID read failed: {:?}",
                                                                            e
                                                                        );
                                                                        break;
                                                                    }
                                                                    Err(_) => {
                                                                        if last_report.elapsed()
                                                                            >= Duration::from_secs(
                                                                                5,
                                                                            )
                                                                        {
                                                                            error!(
                                                                                "no reports for 5s — device gone?"
                                                                            );
                                                                            break;
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                            stop(
                                                                &mut steering,
                                                                &mut motors,
                                                                &mut motor_slew,
                                                                &mut kick,
                                                            );
                                                            bus.state()
                                                                .free_address(ei.device_address);
                                                            log.record(
                                                                flash_log::EV_LOST,
                                                                format_args!(
                                                                    "hub port {} gone",
                                                                    port
                                                                ),
                                                            );
                                                            info!(
                                                                "Device on hub port {} gone",
                                                                port
                                                            );
                                                        }
                                                        Err(_) => {
                                                            info!(
                                                                "hub port {} not a gamepad",
                                                                port
                                                            );
                                                        }
                                                    }
                                                }
                                                Err(_) => {
                                                    error!(
                                                        "hub port {} enumeration timed out",
                                                        port
                                                    );
                                                }
                                                Ok(Err(_)) => {
                                                    error!(
                                                        "hub port {} enumeration failed",
                                                        port
                                                    );
                                                }
                                            }
                                        }
                                        Ok(_) => {}
                                        Err(e) => {
                                            error!("hub event error: {:?}", e);
                                            break;
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                error!("hub register failed: {:?}", e);
                                log.record(
                                    flash_log::EV_HUB,
                                    format_args!("reg failed: {:?}", e),
                                );
                                Timer::after(Duration::from_millis(500)).await;
                            }
                        }
                    }
                }
            }
            Err(e) => {
                error!("Enumerate failed: {:?}", e);
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }
}
