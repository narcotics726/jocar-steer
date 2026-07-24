//! Control policy for dual-mode car (rear-drive / front-drive differential).
//!
//! Pure mapping functions translate PS2 stick/button bytes into steering
//! and motor commands. [`MotorSlew`] adds stateful slew-rate limiting and
//! coast-before-reverse protection.

// ── Configuration ─────────────────────────────────────────────────────

/// Control parameters initialised once in `main` and passed to mapping
/// functions.
pub struct ControlConfig {
    /// Maximum steering deflection per side, in degrees.
    pub steer_max_deg: i32,
    /// Maximum motor duty (12-bit: 0..4095).
    pub motor_max_duty: i32,
    /// Maximum motor duty change per tick (~33 ms).
    pub motor_slew_step: i32,
    /// Deadzone around right-stick X centre (±counts).
    pub rx_deadzone: i32,
    /// Deadzone around left-stick Y centre (±counts).
    pub ly_deadzone: i32,
}

// ── Drive mode ────────────────────────────────────────────────────────

/// Which end of the car is the "front".
#[derive(Clone, Copy, PartialEq, defmt::Format)]
pub enum DriveMode {
    /// Servo axle is front; motors push symmetrically from the rear.
    Rear,
    /// Motor axle is front; differential steering, servo centred.
    Front,
}

impl DriveMode {
    pub fn flip(self) -> Self {
        match self {
            Self::Rear => Self::Front,
            Self::Front => Self::Rear,
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
pub fn ly_to_speed(ly: u8, deadzone: i32, max_duty: i32) -> i32 {
    let centered = 128 - ly as i32;
    if centered.abs() <= deadzone {
        return 0;
    }
    centered * max_duty / 128
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

/// Rear-drive motor command: both motors at the same speed.
pub fn motor_rear(ly: u8, deadzone: i32, max_duty: i32) -> (i32, i32) {
    let s = ly_to_speed(ly, deadzone, max_duty);
    (s, s)
}

/// Front-drive differential motor command: left/right speed split by
/// right-stick X position.
pub fn motor_front(ly: u8, rx: u8, deadzone: i32, max_duty: i32) -> (i32, i32) {
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

/// Stateful slew-rate limiter with coast-before-reverse protection.
///
/// Two-layer protection:
/// 1. **Slew-rate**: per-tick duty change is clamped to `max_step`.
/// 2. **Coast-before-reverse**: when the sign flips the output is forced
///    to 0 for one tick so the TB6612 coasts (IN1=IN2=0) rather than
///    reversing abruptly.
pub struct MotorSlew {
    current_l: i32,
    current_r: i32,
    last_l: i32,
    last_r: i32,
    max_step: i32,
}

impl MotorSlew {
    pub fn new(max_step: i32) -> Self {
        Self {
            current_l: 0,
            current_r: 0,
            last_l: 0,
            last_r: 0,
            max_step,
        }
    }

    /// Take target duties, apply slew + reverse protection, return the
    /// values that should actually be written to the motor driver.
    pub fn update(&mut self, target_l: i32, target_r: i32) -> (i32, i32) {
        // Layer 1: slew-rate limit
        let slew = |target: i32, current: &mut i32| {
            let delta = (target - *current).clamp(-self.max_step, self.max_step);
            *current += delta;
            *current
        };
        let l = slew(target_l, &mut self.current_l);
        let r = slew(target_r, &mut self.current_r);

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

    /// Reset all internal state to zero (call on mode switch or PS2
    /// recovery).
    pub fn reset(&mut self) {
        self.current_l = 0;
        self.current_r = 0;
        self.last_l = 0;
        self.last_r = 0;
    }
}
