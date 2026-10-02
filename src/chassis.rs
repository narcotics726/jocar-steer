//! Chassis: binds the control policy to the actuators.
//!
//! [`Chassis`] owns the steering servo, the motor driver and the stateful
//! limiters (slew, kick, failsafe clock). The input layer only hands it axis
//! values and the current time; which input produced them (USB gamepad today,
//! a CRSF receiver tomorrow) is not its business.

use embassy_time::Instant;
use esp_hal::ledc::channel::ChannelHW;

use crate::control::{self, ControlConfig, MotorSlew, StartKick};
use crate::steering::Steering;

/// A motor output the control layer can drive, in abstract speed units
/// (±`ControlConfig::motor_max_speed`).
///
/// The trait exists because two real implementations do the same job with
/// different semantics: a TB6612 channel (direction pins + duty) and an ESC
/// (a 50 Hz pulse train). The differences that matter are in `stop`/`enable`:
/// a brushed driver coasts or brakes on a zero command, while an ESC reads a
/// missing signal as "no receiver" and must instead be held at neutral.
pub trait MotorDriver {
    /// Command a signed speed. Positive = forward.
    fn set_speed(&mut self, speed: i32);
    /// Command the safe idle state for this actuator.
    fn stop(&mut self);
    /// Put the actuator in a state where it will follow [`set_speed`].
    ///
    /// [`set_speed`]: MotorDriver::set_speed
    fn enable(&mut self);
}

/// Steering servo + motor driver + the state that the control policy needs.
pub struct Chassis<S, M> {
    steering: Steering<S>,
    motors: M,
    slew: MotorSlew,
    kick: StartKick,
    cfg: ControlConfig,
    /// Time of the last accepted report; `None` before the first one (and after
    /// [`halt`](Self::halt)). Drives `dt` and the failsafe window.
    last_report: Option<Instant>,
}

impl<S, M> Chassis<S, M>
where
    S: ChannelHW,
    M: MotorDriver,
{
    /// Build a chassis and arm its actuators.
    ///
    /// Arming is part of construction because both actuators' unarmed states
    /// are safe *and* required: the TB6612 goes to STBY=high (its direction
    /// pins and duty come from the driver and the bin's LEDC channel — a
    /// driver that arrives with a non-zero duty would be armed, not coasting),
    /// and an ESC must start emitting its neutral pulse train immediately
    /// (that is its arming sequence). A chassis that is never armed cannot
    /// drive anything, so there is no reason to expose the step.
    pub fn new(steering: Steering<S>, mut motors: M, cfg: ControlConfig) -> Self {
        motors.enable();
        Self {
            steering,
            motors,
            slew: MotorSlew::new(&cfg),
            kick: StartKick::new(),
            cfg,
            last_report: None,
        }
    }

    /// Apply one input report: steer, throttle, mix, slew, kick — then write the
    /// actuators.
    ///
    /// `steer_axis`/`throttle_axis` are raw stick bytes (0..255, ~128 centre),
    /// `now` is the report's arrival time. The elapsed time since the previous
    /// report is derived here, so the caller only has to be honest about `now`.
    pub fn on_report(&mut self, steer_axis: u8, throttle_axis: u8, now: Instant) {
        let dt_us = match self.last_report {
            Some(prev) => (now - prev).as_micros(),
            None => 0,
        };
        self.last_report = Some(now);

        let steer = control::rx_to_deg(steer_axis, self.cfg.rx_deadzone, self.cfg.steer_max_deg);
        self.steering.set_target(steer);
        self.steering.update(self.cfg.steer_slew_rate_deg_s, dt_us);

        let limit = control::speed_limit(steer, &self.cfg);
        let target = control::throttle_speed(throttle_axis, steer, &self.cfg);

        // The kick overrides the slewed value briefly; the slew still advances
        // underneath so the hand-back is smooth. It must **not** override a
        // reversal protocol: that is the actuator's requirement, and a kick
        // would jump straight to full speed in the very direction being denied.
        let slewed = self.slew.update(target, dt_us);
        let speed = match self.kick.tick(target, limit, now, &self.cfg) {
            Some(kick) if !self.slew.in_reversal() => kick,
            _ => slewed,
        };
        self.motors.set_speed(speed);
    }

    /// Whether the input has been silent for longer than
    /// [`ControlConfig::failsafe_timeout_ms`].
    ///
    /// The clock is the last **accepted** report: frames that arrive but fail
    /// to parse do not count as input, because nothing was applied from them
    /// and the last commanded speed would otherwise stay live.
    ///
    /// `false` before the first report — and that case is deliberately *not* a
    /// failure: a receiver can be attached and enumerated while its handset is
    /// still off, and in that state the chassis has never been commanded
    /// anything, so there is nothing to stop and no reason to tear the session
    /// down. (The old firmware seeded this clock at session start and reset the
    /// chip every ~11 s in that situation.)
    pub fn failsafe_expired(&self, now: Instant) -> bool {
        match self.last_report {
            Some(t) => (now - t).as_millis() >= self.cfg.failsafe_timeout_ms,
            None => false,
        }
    }

    /// The input-silence window this chassis was configured with.
    ///
    /// Read by the session layer when it reports *why* it stopped, so the log
    /// cannot disagree with the decision [`failsafe_expired`](Self::failsafe_expired)
    /// actually made.
    pub fn failsafe_timeout_ms(&self) -> u64 {
        self.cfg.failsafe_timeout_ms
    }

    /// Stop commanding the actuators and straighten the steering.
    ///
    /// Shared by "device gone" and "input went stale" (and called by the
    /// session layer *before* it records the stale event, so the motor is not
    /// left commanded across a flash erase): the motor coasts, the servo
    /// centres, and the limiters start from zero next time. Centring is
    /// deliberate — a lost link should not leave the wheels at lock. Idempotent.
    pub fn halt(&mut self) {
        self.slew.reset();
        self.kick.reset();
        self.motors.stop();
        self.steering.center();
        self.last_report = None;
    }
}
