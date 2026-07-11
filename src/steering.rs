//! Steering servo abstraction.
//!
//! `Steering` encapsulates the conversion from logical angle to PWM duty, and
//! hides the LEDC channel details from the input layer. The input layer only
//! passes a desired angle in degrees — it does not need to know whether the
//! angle came from a button pad, a PS2 analog stick, or anything else.
//!
//! # Future: slew-rate limiting (not yet implemented)
//!
//! If smooth ramping is ever needed (e.g. for throttle or a softer digital-steer
//! feel), add:
//!
//! - `set_target(deg)` — store the goal but do **not** write the channel.
//! - `update(max_step)` — called from a fixed-rate tick; moves an internal
//!   `current_deg` toward `target_deg` by at most `max_step` each call, then
//!   writes the channel.
//!
//! With a large enough `max_step`, `update` behaves identically to the current
//! immediate `set_angle`. The steering servo itself is fast enough that slew-rate
//! control is generally unnecessary — the servo responds instantly and the
//! operator expects direct 1:1 tracking.

use esp_hal::ledc::channel::ChannelHW;

// --- SG90 servo timing (50 Hz PWM, 12-bit LEDC duty) ---
// SG90 datasheet: 20 ms period; 1.0 ms = -90°, 1.5 ms = 0° (center), 2.0 ms = +90°.
// Stay within 1000..=2000 µs — the theoretical 500/2400 µs limits slam the
// mechanical stops (buzzing / overheating / gear damage).
const PERIOD_US: u32 = 20_000; // 50 Hz
const DUTY_MAX: u32 = 1 << 12; // 12-bit timer → 4096 counts per period
const CENTER_US: i32 = 1500; // 0°
const US_PER_90DEG: i32 = 500; // 90° swing = 500 µs from center

/// Convert a servo angle in degrees (-90..=90) to a raw 12-bit LEDC duty count.
///
/// Reference values (for self-check):
/// -   0° → 307
/// - +90° → 410
/// - -90° → 205
/// - +60° → 375
/// -  +3° → 310
fn angle_to_counts(deg: i32) -> u32 {
    let deg = deg.clamp(-90, 90);
    let pulse_us = (CENTER_US + deg * US_PER_90DEG / 90) as u32;
    (DUTY_MAX * pulse_us) / PERIOD_US
}

/// Steering servo abstraction: independent of input source.
///
/// Hides the LEDC channel, the static center-offset compensation, run-time trim,
/// and the angle→PWM-duty mapping behind a small, stable API.
pub struct Steering<Ch> {
    channel: Ch,
    /// Static center-offset compensation (e.g. residual horn-mounting error).
    center_offset_deg: i32,
    /// Maximum deflection angle on each side, in degrees.
    max_deg: i32,
    /// Run-time fine calibration (accumulated by [`adjust_trim`]).
    trim_deg: i32,
    /// Most recent target angle passed to [`set_angle`].
    target_deg: i32,
}

impl<Ch: ChannelHW> Steering<Ch> {
    /// Create a new `Steering` and immediately drive the servo to its
    /// compensated center position.
    ///
    /// `center_offset_deg` is a static offset that compensates for mechanical
    /// mounting error (e.g. the horn cannot be fitted exactly at 0°).
    ///
    /// `max_deg` is the maximum commanded deflection per side. The effective
    /// servo angle is always clamped to ±90° by `angle_to_counts`, so values
    /// larger than ~87° (when combined with the offset) will hit the hardware
    /// safety ceiling.
    pub fn new(channel: Ch, center_offset_deg: i32, max_deg: i32) -> Self {
        let mut this = Self {
            channel,
            center_offset_deg,
            max_deg,
            trim_deg: 0,
            target_deg: 0,
        };
        this.apply();
        this
    }

    /// Command a steering angle relative to the trimmed center.
    ///
    /// `deg` is clamped to `[-max_deg, max_deg]` before being written to the
    /// servo. Positive = left (longer pulse), negative = right (shorter pulse).
    pub fn set_angle(&mut self, deg: i32) {
        self.target_deg = deg.clamp(-self.max_deg, self.max_deg);
        self.apply();
    }

    /// Return the servo to the trimmed center position (equivalent to
    /// `set_angle(0)`).
    pub fn center(&mut self) {
        self.set_angle(0);
    }

    /// Adjust the run-time trim by `delta` degrees.
    ///
    /// The accumulated trim is clamped to ±30°. The change takes effect
    /// immediately (the channel is re-written).
    pub fn adjust_trim(&mut self, delta: i32) {
        self.trim_deg = (self.trim_deg + delta).clamp(-30, 30);
        self.apply();
    }

    /// Return the current run-time trim value in degrees.
    pub fn trim(&self) -> i32 {
        self.trim_deg
    }

    // ── private ──────────────────────────────────────────────────────

    /// Compute the effective angle and write it to the LEDC channel.
    fn apply(&mut self) {
        // effective_deg = static offset + run-time trim + target
        let effective_deg = self.center_offset_deg + self.trim_deg + self.target_deg;
        self.channel.set_duty_hw(angle_to_counts(effective_deg));
    }
}
