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
    /// Maximum steering deflection on the **positive** side, in degrees: the
    /// longer servo pulse, which is the side [`crate::steering`] and the
    /// `servo-test` sweeps label LEFT.
    ///
    /// Separate from the negative limit because steering linkages are not
    /// symmetric: the knuckle's own stops can differ by several degrees, and the
    /// only software lever is to cap each side separately. Measured on the
    /// 1/10 car: both ends sit 3–5° short of their stop at ±75° of servo travel,
    /// i.e. the asymmetry is in the stops, not in the servo's range.
    pub steer_max_left_deg: i32,
    /// Maximum steering deflection on the **negative** side, in degrees (the
    /// shorter pulse).
    ///
    /// Evening the two sides out by *reducing* this one is the safe direction;
    /// raising `steer_max_left_deg` eats that side's margin to its stop.
    pub steer_max_right_deg: i32,
    /// Flip the sign of the stick→angle mapping: the composite of the receiver's
    /// axis polarity and the car's steering mounting, which together decide
    /// whether stick-left steers left (see [`rx_to_deg`] for why the two are
    /// separable only with a bench check, not with the code).
    ///
    /// It came from a measurement, not from a convention: the 1/10 car sent the
    /// wheels the wrong way on the default sign (stick left → wheels right). The
    /// other car keeps the default — it has never been reported as reversed, and
    /// this change must not silently flip a car that already drives; if it ever
    /// turns out reversed too, it is this same flag. A standard analog servo has
    /// no reverse bit, and re-fitting the horn 180° would move the mechanical
    /// centre with it, so this is also the only place it can be fixed.
    ///
    /// It does **not** swap the two limits to the other stick end: they are
    /// picked by the sign of the angle actually sent, so `steer_max_left_deg`
    /// stays the longer-pulse side on both settings.
    pub steer_invert: bool,
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
    /// Command held during the braking phase of a reversal, in abstract speed
    /// (negative = the direction being entered). `0` disables the phase.
    ///
    /// Measured, not guessed: this ESC will not accept reverse after a forward
    /// demand unless a sub-neutral pulse arrives *first* and is followed by
    /// neutral — with the brake in place every reverse depth from 1450 µs down
    /// to 1050 µs engaged; without it, a 600 ms neutral window alone did not.
    /// The value has to land just past the actuator's neutral deadband and only
    /// shallowly (the bin derives it from a µs figure).
    pub reversal_brake_speed: i32,
    /// How long the braking pulse is held, in milliseconds.
    pub reversal_brake_ms: u64,
    /// How long the reversal then holds *exactly* zero before the new direction
    /// is allowed to ramp up, in milliseconds.
    ///
    /// This is the actuator's direction-change requirement expressed in time (it
    /// used to be "one call" — the last per-tick quantity in the control path):
    ///
    /// - TB6612: a few ms, so the H-bridge is not reversed instantaneously.
    /// - latching brushed ESC: **the unit's latch-release window**. Too short and
    ///   reverse silently never engages — the plan's boundary condition ①.
    ///
    /// `0` disables the hold.
    pub reversal_neutral_ms: u64,
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
/// `rx` rising (0 → 255) maps toward the **positive** angle — the longer pulse —
/// and the magnitude is scaled by **that side's** limit, so full stick is exactly
/// that side's cap. Scaling both sides by one of them (all a single symmetric
/// limit could do) would have the smaller side reach its cap early and leave a
/// dead band above it.
///
/// Which *stick* end that is, and which *physical* side it steers, are two facts
/// this function cannot separate: the receiver's axis polarity (the notes in
/// `ps2.rs` and the bins say 0 = stick left, and the control loop prints `rx=`
/// every 100 ms, so it is checkable) times the car's steering mounting. Their
/// product is what actually decides whether stick-left steers left, and it is
/// per-car — [`ControlConfig::steer_invert`].
pub fn rx_to_deg(rx: u8, cfg: &ControlConfig) -> i32 {
    let centered = rx as i32 - 128;
    if centered.abs() <= cfg.rx_deadzone {
        return 0;
    }
    // The per-car sign. Note it does not move the two limits to the other
    // stick end: they are picked by the sign of the angle that is actually
    // sent, so `steer_max_left_deg` is always the longer-pulse side.
    let cmd = if cfg.steer_invert { -centered } else { centered };
    let limit = if cmd > 0 {
        cfg.steer_max_left_deg
    } else {
        cfg.steer_max_right_deg
    };
    cmd * limit / 128
}

/// Speed ceiling for a given steering angle: full `motor_max_speed` at centre,
/// `motor_max_speed * (1 - mix_num/mix_den)` at full lock.
///
/// Without this, the motor fights the front-wheel scrub during turns on a
/// chassis with no differential, which is the main source of stall current and
/// motor heat there. Normalised by *that side's* limit, so an asymmetric
/// steering limit does not skew the mix.
pub fn speed_limit(steer_deg: i32, cfg: &ControlConfig) -> i32 {
    if cfg.steer_mix_num == 0 {
        return cfg.motor_max_speed;
    }
    let limit = if steer_deg >= 0 {
        cfg.steer_max_left_deg
    } else {
        cfg.steer_max_right_deg
    };
    let cut = steer_deg.abs() * cfg.motor_max_speed * cfg.steer_mix_num
        / (limit * cfg.steer_mix_den);
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

/// Stateful, time-based slew-rate limiter with a timed **reversal protocol**.
///
/// Three layers:
/// 1. **Slew-rate**: the change per call is capped at `rate_per_s × dt`, so the
///    limit is expressed in physical time and is independent of how often the
///    control loop runs (or how fast reports arrive). The previous per-tick
///    formulation silently scaled with the report rate. The sub-unit remainder
///    is carried across calls: a rate that covers less than one unit per call
///    must still advance, or it truncates to zero and the motor freezes.
/// 2. **Reversal protocol**: when the *demanded* direction reverses, the
///    actuator is not handed the new direction immediately. Entering reverse
///    runs [`ControlConfig::reversal_brake_speed`] for
///    [`ControlConfig::reversal_brake_ms`] (a latching ESC needs a sub-neutral
///    pulse there before it will accept reverse at all), then holds *exactly*
///    zero for [`ControlConfig::reversal_neutral_ms`] (the latch-release
///    window; also the H-bridge coast for a brushed driver), and only then ramps
///    the new direction up **from zero**. The ramp is frozen for the whole
///    protocol: letting it advance would hand the actuator the already-completed
///    target the moment the window expires, i.e. a step instead of a ramp.
///
///    It keys on the demand rather than on the output because the output lags the
///    stick: a driver that tried to detect this from the value it is handed sees
///    the ramp crossing zero and cannot tell "still reversing" from "operator
///    already asked for forward", which is how a reverse burst ends up under a
///    stick held at full forward. The protocol is abandoned if the demand
///    changes direction again mid-way.
/// 3. **Direction memory**: the last non-zero direction survives both a stop and
///    a passage through zero, so a reversal can still be recognised after the
///    throttle has been at rest. A driver that has never driven anything has no
///    latch to release, which is why a cold start goes straight to reverse.
pub struct MotorSlew {
    current: i32,
    /// Direction of the last non-zero output (-1 / 0 / +1); 0 = nothing driven
    /// yet this power cycle.
    last_dir: i32,
    /// Maximum speed change per second (abstract speed units/s).
    rate_per_s: i32,
    /// Sub-unit slew credit, in 1e-6 speed units.
    residual: i64,
    /// Reversal protocol parameters (see the struct docs).
    brake_speed: i32,
    brake_us: u64,
    neutral_us: u64,
    /// Time left in the braking phase (0 = not braking).
    brake_left_us: u64,
    /// Time left in the exact-zero window (0 = not holding).
    hold_left_us: u64,
    /// Direction the in-progress protocol is denying.
    into_dir: i32,
}

impl MotorSlew {
    pub fn new(cfg: &ControlConfig) -> Self {
        Self {
            current: 0,
            last_dir: 0,
            rate_per_s: cfg.motor_slew_rate_speed_s,
            residual: 0,
            brake_speed: cfg.reversal_brake_speed,
            brake_us: cfg.reversal_brake_ms * 1_000,
            neutral_us: cfg.reversal_neutral_ms * 1_000,
            brake_left_us: 0,
            hold_left_us: 0,
            into_dir: 0,
        }
    }

    /// Whether a reversal protocol is running.
    ///
    /// Callers must not override it with anything else — it is an actuator
    /// requirement, and the start-kick in particular would otherwise jump
    /// straight to full speed in the direction being denied.
    pub fn in_reversal(&self) -> bool {
        self.brake_left_us > 0 || self.hold_left_us > 0
    }

    /// Take the target speed plus the elapsed time since the previous call
    /// (`dt_us`, microseconds), apply the layers above, and return the value
    /// that should actually be written to the motor driver.
    pub fn update(&mut self, target: i32, dt_us: u64) -> i32 {
        let target_dir = target.signum();

        // ── Run (or abandon) an in-progress reversal protocol ────────────
        if self.brake_left_us > 0 || self.hold_left_us > 0 {
            if target_dir != 0 && target_dir != self.into_dir {
                // The operator changed their mind again: the protocol exists to
                // deny one specific direction, so drop it and resume the ramp
                // from wherever `current` sits.
                self.brake_left_us = 0;
                self.hold_left_us = 0;
            } else if self.brake_left_us > 0 {
                self.brake_left_us = self.brake_left_us.saturating_sub(dt_us);
                if self.brake_left_us > 0 {
                    self.current = self.brake_speed;
                    self.residual = 0;
                    return self.brake_speed;
                }
                // Brake done: exact neutral, and the ramp restarts from zero.
                self.hold_left_us = self.neutral_us;
                self.current = 0;
                self.residual = 0;
                return 0;
            } else {
                self.hold_left_us = self.hold_left_us.saturating_sub(dt_us);
                if self.hold_left_us > 0 {
                    return 0;
                }
                // Window done; fall through and ramp normally from zero.
            }
        }

        // ── Start one on a change of *demanded* direction ────────────────
        if target_dir != 0
            && self.last_dir != 0
            && target_dir != self.last_dir
            && (self.brake_us > 0 || self.neutral_us > 0)
        {
            self.into_dir = target_dir;
            self.last_dir = 0;
            // The brake phase is only meaningful entering *reverse* (the
            // direction a latching ESC gates). Entering forward gets the neutral
            // window alone.
            if target_dir < 0 && self.brake_us > 0 && self.brake_speed != 0 {
                self.brake_left_us = self.brake_us;
                self.current = self.brake_speed;
                self.residual = 0;
                return self.brake_speed;
            }
            self.hold_left_us = self.neutral_us;
            self.current = 0;
            self.residual = 0;
            return 0;
        }

        // ── Normal time-based slew step ─────────────────────────────────
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

        if self.current != 0 {
            self.last_dir = self.current.signum();
        }
        self.current
    }

    /// Reset the ramp to zero (call on stop / device disconnect).
    ///
    /// The direction memory is deliberately **kept**: after a stop the
    /// actuator's own direction latch still reflects what we last drove, so a
    /// reverse command that follows a halt still runs the protocol.
    pub fn reset(&mut self) {
        self.current = 0;
        self.residual = 0;
        self.brake_left_us = 0;
        self.hold_left_us = 0;
    }
}

// ── Start kick ────────────────────────────────────────────────────────

/// One-shot start-kick state, timed in physical time.
///
/// Private to the crate: the kick is part of the chassis' policy, and the only
/// thing a bin would do with it is construct one — which `Chassis` already does.
///
/// The kick briefly outputs the full allowed speed when the throttle jumps from
/// rest, so the motor can overcome static friction immediately instead of
/// waiting for the slew ramp. The slew state still advances underneath, so the
/// hand-back to the slew value is smooth.
///
/// Note for ESC chassis: the kick must stay disabled there — any residual
/// above neutral delays the ESC's neutral detection and breaks reverse (see the
/// plan's boundary conditions).
pub(crate) struct StartKick {
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

    pub(crate) fn reset(&mut self) {
        *self = Self::new();
    }

    /// Returns `Some(speed)` while the kick burst is active, `None` otherwise.
    /// `limit` is the current speed ceiling (mix already applied).
    pub(crate) fn tick(
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
