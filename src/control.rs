//! Control policy (steering + throttle mapping).
//!
//! Pure mapping functions translate stick bytes into steering and motor
//! commands. [`MotorSlew`] adds stateful, time-based slew-rate limiting and
//! coast-before-reverse protection.

// ── Configuration ─────────────────────────────────────────────────────

/// Control parameters initialised once in `main` and passed to mapping
/// functions.
pub struct ControlConfig {
    /// Maximum steering deflection per side, in degrees.
    pub steer_max_deg: i32,
    /// Maximum motor speed, in abstract units (±4095). The actuator maps them
    /// into its own domain — duty for a TB6612 channel, pulse width for an ESC.
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
}

// ── Drive mode (legacy dual-mode era) ─────────────────────────────────

/// Steering strategy for the car.
///
/// Only the archived dual-motor chassis used `Diff`; the current cars drive a
/// single motor. Kept until the dual-mode remnants are removed.
#[derive(Clone, Copy, PartialEq, defmt::Format)]
pub enum DriveMode {
    /// Servo axle steers; both motors drive symmetrically.
    Servo,
    /// Servo centred; motors drive differentially for steering.
    Diff,
}

impl DriveMode {
    pub fn flip(self) -> Self {
        match self {
            Self::Servo => Self::Diff,
            Self::Diff => Self::Servo,
        }
    }
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

/// Servo-mode motor command: both motors at the same speed.
///
/// Legacy: only the archived dual-motor firmware calls this.
pub fn motor_servo(ly: u8, deadzone: i32, max_duty: i32) -> (i32, i32) {
    let s = ly_to_speed(ly, deadzone, max_duty);
    (s, s)
}

/// Diff-mode motor command: left/right speed split by right-stick X
/// position for differential steering.
///
/// Legacy: only the archived dual-motor firmware calls this.
pub fn motor_diff(ly: u8, rx: u8, deadzone: i32, max_duty: i32) -> (i32, i32) {
    let base = ly_to_speed(ly, deadzone, max_duty);

    let centered = rx as i32 - 128;
    let diff = if centered.abs() <= deadzone {
        0
    } else {
        centered * max_duty / 128
    };

    let l = (base + diff).clamp(-max_duty, max_duty);
    let r = (base - diff).clamp(-max_duty, max_duty);
    (l, r)
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
    current_l: i32,
    current_r: i32,
    last_l: i32,
    last_r: i32,
    /// Maximum speed change per second (abstract speed units/s).
    rate_per_s: i32,
    /// Sub-unit slew credit per channel, in 1e-6 speed units.
    residual_l: i64,
    residual_r: i64,
}

impl MotorSlew {
    pub fn new(rate_per_s: i32) -> Self {
        Self {
            current_l: 0,
            current_r: 0,
            last_l: 0,
            last_r: 0,
            rate_per_s,
            residual_l: 0,
            residual_r: 0,
        }
    }

    /// Take target speeds plus the elapsed time since the previous call
    /// (`dt_us`, microseconds), apply slew + reverse protection, and return the
    /// values that should actually be written to the motor driver.
    pub fn update(&mut self, target_l: i32, target_r: i32, dt_us: u64) -> (i32, i32) {
        // Layer 1: time-based slew limit — the most we may move is rate × dt,
        // with any sub-unit remainder carried over to the next call.
        let rate = self.rate_per_s;
        let slew = |target: i32, current: &mut i32, residual: &mut i64| {
            let err = target - *current;
            if err == 0 {
                *residual = 0;
                return *current;
            }
            *residual += rate as i64 * dt_us as i64;
            let budget = (*residual / 1_000_000).clamp(0, i32::MAX as i64) as i32;
            if budget == 0 {
                return *current; // not enough accumulated time for one unit yet
            }
            let delta = err.clamp(-budget, budget);
            *residual -= delta.abs() as i64 * 1_000_000;
            *current += delta;
            if *current == target {
                *residual = 0; // reached the goal; drop stale credit
            }
            *current
        };
        let l = slew(target_l, &mut self.current_l, &mut self.residual_l);
        let r = slew(target_r, &mut self.current_r, &mut self.residual_r);

        // Layer 2: coast-before-reverse
        let protect = |cmd: i32, last: i32| {
            if cmd.signum() * last.signum() < 0 && cmd != 0 {
                0
            } else {
                cmd
            }
        };
        let l = protect(l, self.last_l);
        let r = protect(r, self.last_r);

        self.last_l = l;
        self.last_r = r;
        (l, r)
    }

    /// Reset all internal state to zero (call on stop / device disconnect).
    pub fn reset(&mut self) {
        self.current_l = 0;
        self.current_r = 0;
        self.last_l = 0;
        self.last_r = 0;
        self.residual_l = 0;
        self.residual_r = 0;
    }
}
