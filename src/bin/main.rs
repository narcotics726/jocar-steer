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
        steer_slew_rate_deg_s: 242, // ≈ the designed 8°/33 ms
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

    loop {
        let speed = bus_ctrl.wait_for_connection().await;
        info!("Device connected at {:?}", speed);

        let mut config_buf = [0u8; 256];
        // Bound the enumeration so a non-responding device cannot hang the
        // loop forever (seen as a stuck state on a flaky power rail).
        let enum_result = embassy_time::with_timeout(
            Duration::from_secs(5),
            bus.enumerate(BusRoute::Direct(speed), &mut config_buf),
        )
        .await;
        let enum_result = match enum_result {
            Ok(r) => r,
            Err(_) => {
                error!("Enumeration timed out after 5s");
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

                match GamepadHost::new(&bus, &config_buf[..config_len], &enum_info) {
                    Ok(h) => {
                        let mut hid = h;
                        info!("iface0 ready — reading gamepad reports");

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
                                    let now = Instant::now();
                                    let dt_us = (now - last_report).as_micros();
                                    last_report = now;
                                    if let Some(gp) = GamepadState::parse(&buf[..n]) {
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
                                    }
                                }
                                Ok(Ok(_)) => {}
                                Ok(Err(e)) => {
                                    error!("HID read failed: {:?}", e);
                                    break;
                                }
                                Err(_) => {
                                    if last_report.elapsed() >= Duration::from_secs(5) {
                                        error!("no reports for 5s — device gone?");
                                        break;
                                    }
                                }
                            }
                        }
                        stop(&mut steering, &mut motors, &mut motor_slew, &mut kick);
                        info!("Device disconnected, waiting for next");
                    }
                    Err(_) => {
                        // Not a ZD receiver — try registering it as a hub and
                        // service downstream ports.
                        info!("no direct gamepad — trying hub");
                        match HubHandler::<_, 8>::try_register(&bus, &enum_info).await {
                            Ok(mut hub) => {
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
