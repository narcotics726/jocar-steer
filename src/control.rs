//! Control policy: stick axes → steering angle / motor speed, plus the
//! stateful, time-based limiters around them.
//!
//! Nothing here touches esp-hal: the caller supplies the clock (`Instant`,
//! `dt_us`) and the actuator writes happen in [`crate::chassis`]/drivers. That
//! is what lets one policy drive a TB6612 or an ESC, and it keeps the
//! host-test experiment (see the refactor plan) possible without a crate split.

use embassy_time::{Duration, Instant};

// ── Configuration ─────────────────────────────────────────────────────

/// Control parameters: one set per chassis.
///
/// Handling differences (steering travel, mixing, kick, failsafe window) live
/// here; actuator differences (duty vs pulse width, coast vs neutral) live in
/// the [`MotorDriver`](crate::chassis::MotorDriver) implementation.
#[derive(Clone, Copy)]
pub struct ControlConfig {
    /// Maximum steering deflection per side, in degrees.
    pub steer_max_deg: i32,
    /// Maximum motor speed, in abstract units (±`motor_max_speed`). The actuator
    /// maps them into its own domain — duty for a TB6612 channel, pulse width
    /// for an ESC.
    pub motor_max_speed: i32,
    /// Maximum motor speed change per second (abstract units/s). Expressed in
    /// physical time so the limit does not depend on the control loop rate or
    /// on how fast gamepad reports arrive.
    pub motor_slew_rate_speed_s: i32,
    /// Maximum commanded steering change per second (degrees/s). Caps the
    /// servo's peak current draw when the stick is slammed.
    pub steer_slew_rate_deg_s: i32,
    /// Deadzone around right-stick X centre (±counts).
    pub rx_deadzone: i32,
    /// Deadzone around left-stick Y centre (±counts).
    pub ly_deadzone: i32,
    /// Steer-throttle mix numerator: at full lock the speed ceiling is scaled
    /// to `motor_max_speed * (1 - steer_mix_num / steer_mix_den)`. `0/1`
    /// disables the mix (a chassis with normal steering geometry does not need
    /// it — see the plan's 1/10-car parameters).
    pub steer_mix_num: i32,
    /// Steer-throttle mix denominator; must not be 0.
    pub steer_mix_den: i32,
    /// Start-kick burst duration in milliseconds. `0` disables the kick.
    pub kick_duration_ms: u64,
    /// Minimum `|target| / limit` ratio (as `num/den`) that triggers a kick —
    /// keeps light stick touches from causing full-speed bursts.
    pub kick_min_num: i32,
    /// Denominator of the kick trigger ratio.
    pub kick_min_den: i32,
    /// Stop commanding the actuators after this long without an input report.
    pub failsafe_timeout_ms: u64,
}

// ── Pure mapping functions ────────────────────────────────────────────

/// Map left-stick Y to signed motor speed.
///
/// `ly` range: 0 = full-up/forward, 128 = centre, 255 = full-down/reverse.
/// The centre value 128 splits the range asymmetrically (±128 vs ±127),
/// so we use `/128` everywhere.  At the short end the error is 1/128 ≈
/// 0.8 %, which is invisible next to stick noise and the deadzone.
pub fn ly_to_speed(ly: u8, deadzone: i32, max_speed: i32) -> i32 {
    let centered = 128 - ly as i32;
    if centered.abs() <= deadzone {
        return 0;
    }
    centered * max_speed / 128
}

/// Map right-stick X to a steering angle in degrees.
///
/// `rx` range: 0 = full-left, 128 = centre, 255 = full-right.
/// Same `/128` rationale as [`ly_to_speed`].
pub fn rx_to_deg(rx: u8, deadzone: i32, max_deg: i32) -> i32 {
    let centered = rx as i32 - 128;
    if centered.abs() <= deadzone {
        return 0;
    }
    centered * max_deg / 128
}

/// Speed ceiling for a given steering angle: full `motor_max_speed` at centre,
/// `motor_max_speed * (1 - mix_num/mix_den)` at full lock.
///
/// Without this, the motor fights the front-wheel scrub during turns on a
/// chassis with no differential, which is the main source of stall current and
/// motor heat there.
pub fn speed_limit(steer_deg: i32, cfg: &ControlConfig) -> i32 {
    if cfg.steer_mix_num == 0 {
        return cfg.motor_max_speed;
    }
    let cut = steer_deg.abs() * cfg.motor_max_speed * cfg.steer_mix_num
        / (cfg.steer_max_deg * cfg.steer_mix_den);
    (cfg.motor_max_speed - cut).max(0)
}

/// Final throttle command for one report: stick → speed, mixed and clamped.
///
/// This is the single place where the throttle policy is decided, so both the
/// chassis and any future input source agree on what the stick means.
pub fn throttle_speed(ly: u8, steer_deg: i32, cfg: &ControlConfig) -> i32 {
    let limit = speed_limit(steer_deg, cfg);
    ly_to_speed(ly, cfg.ly_deadzone, cfg.motor_max_speed).clamp(-limit, limit)
}

// ── Motor slew-rate limiter ───────────────────────────────────────────

/// Stateful, time-based slew-rate limiter with coast-before-reverse protection.
///
/// Two-layer protection:
/// 1. **Slew-rate**: the change per call is capped at `rate_per_s × dt`, so the
///    limit is expressed in physical time and is independent of how often the
///    control loop runs (or how fast reports arrive). The previous per-tick
///    formulation silently scaled with the report rate. The sub-unit remainder
///    is carried across calls: a rate that covers less than one unit per call
///    must still advance, or it truncates to zero and the motor freezes.
/// 2. **Coast-before-reverse**: when the sign flips the output is forced to 0
///    for one call so the driver coasts (TB6612: IN1=IN2=0) rather than
///    reversing abruptly.
pub struct MotorSlew {
    current: i32,
    last: i32,
    /// Maximum speed change per second (abstract speed units/s).
    rate_per_s: i32,
    /// Sub-unit slew credit, in 1e-6 speed units.
    residual: i64,
}

impl MotorSlew {
    pub fn new(rate_per_s: i32) -> Self {
        Self {
            current: 0,
            last: 0,
            rate_per_s,
            residual: 0,
        }
    }

    /// Take the target speed plus the elapsed time since the previous call
    /// (`dt_us`, microseconds), apply slew + reverse protection, and return the
    /// value that should actually be written to the motor driver.
    pub fn update(&mut self, target: i32, dt_us: u64) -> i32 {
        // Layer 1: time-based slew limit — the most we may move is rate × dt,
        // with any sub-unit remainder carried over to the next call.
        let err = target - self.current;
        if err == 0 {
            self.residual = 0;
        } else {
            self.residual += self.rate_per_s as i64 * dt_us as i64;
            let budget = (self.residual / 1_000_000).clamp(0, i32::MAX as i64) as i32;
            if budget > 0 {
                let delta = err.clamp(-budget, budget);
                self.residual -= delta.abs() as i64 * 1_000_000;
                self.current += delta;
                if self.current == target {
                    self.residual = 0; // reached the goal; drop stale credit
                }
            }
        }

        // Layer 2: coast-before-reverse
        let cmd = if self.current.signum() * self.last.signum() < 0 && self.current != 0 {
            0
        } else {
            self.current
        };
        self.last = cmd;
        cmd
    }

    /// Reset all internal state to zero (call on stop / device disconnect).
    pub fn reset(&mut self) {
        self.current = 0;
        self.last = 0;
        self.residual = 0;
    }
}

// ── Start kick ────────────────────────────────────────────────────────

/// One-shot start-kick state, timed in physical time.
///
/// The kick briefly outputs the full allowed speed when the throttle jumps from
/// rest, so the motor can overcome static friction immediately instead of
/// waiting for the slew ramp. The slew state still advances underneath, so the
/// hand-back to the slew value is smooth.
///
/// Note for ESC chassis: the kick must stay disabled there — any residual
/// above neutral delays the ESC's neutral detection and breaks reverse (see the
/// plan's boundary conditions).
pub struct StartKick {
    /// Deadline of the active burst; `None` when no burst is running.
    kick_until: Option<Instant>,
    prev_was_zero: bool,
}

impl StartKick {
    pub(crate) const fn new() -> Self {
        Self {
            kick_until: None,
            prev_was_zero: true,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Returns `Some(speed)` while the kick burst is active, `None` otherwise.
    /// `limit` is the current speed ceiling (mix already applied).
    pub fn tick(
        &mut self,
        target: i32,
        limit: i32,
        now: Instant,
        cfg: &ControlConfig,
    ) -> Option<i32> {
        if cfg.kick_duration_ms == 0 {
            return None; // disabled
        }
        let big_enough = target.abs() * cfg.kick_min_den >= limit * cfg.kick_min_num;
        if target == 0 || !big_enough {
            self.kick_until = None;
            self.prev_was_zero = true;
            return None;
        }
        if self.prev_was_zero {
            self.prev_was_zero = false;
            self.kick_until = Some(now + Duration::from_millis(cfg.kick_duration_ms));
        }
        match self.kick_until {
            Some(until) if now < until => Some(limit * target.signum()),
            _ => None,
        }
    }
}
