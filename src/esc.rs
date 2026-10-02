//! Brushed ESC driver — the "servo that controls speed".
//!
//! A brushed ESC is driven exactly like a servo (a 50 Hz pulse train whose
//! width is the command), and this module exists for the two ways it is *not*
//! just a servo:
//!
//! 1. **Neutral is a command, not silence.** Duty 0 means "no signal" to an
//!    ESC: it stops driving and may disarm. [`stop`](MotorDriver::stop) keeps
//!    emitting the neutral pulse instead of going quiet, and a command inside
//!    the deadzone becomes an *exact* neutral pulse rather than a near-neutral
//!    one.
//! 2. **Arming.** After power-up (and after any reset) the unit wants to see a
//!    continuous neutral pulse for a while before it accepts throttle, so
//!    [`enable`](MotorDriver::enable) (re)starts that hold window.
//!
//! # Two-stage reverse deliberately does *not* live here
//!
//! A cheap forward/reverse ESC latches its direction and only releases the latch
//! after a neutral dwell, so it is tempting to implement that dwell in this
//! driver. It was implemented here first, and it was wrong: a driver only sees
//! the value the slew hands it, and that value **lags the stick**. When the
//! operator returns the stick to forward in the middle of the dwell, the value
//! is still deep in reverse, so the driver cannot tell "still reversing" from
//! "operator changed their mind" — and on the dwell's expiry it emitted a
//! reverse pulse under a stick held at full forward.
//!
//! The dwell therefore belongs to [`MotorSlew`](crate::control::MotorSlew),
//! which sees the *demanded* direction (`ControlConfig::reverse_coast_ms`) and
//! which freezes its ramp during the hold, so the new direction also starts as a
//! ramp instead of a step. This driver's only obligation is that a zero command
//! is exactly neutral — which is what the deadzone below guarantees.
//!
//! # Resolution
//!
//! This channel shares the steering servo's 50 Hz LEDC timer, so the pulse
//! resolution is the timer's: 12-bit → 4.88 µs per step ≈ 1 % of a 500 µs span.
//! If the throttle feels coarse on the car, raise it in [`crate::rc_pwm`] — the
//! timer configuration and this conversion live there together so they cannot
//! drift apart.

use embassy_time::{Duration, Instant};
use esp_hal::ledc::channel::ChannelHW;

use crate::chassis::MotorDriver;
use crate::rc_pwm::pulse_to_counts;

/// Pulse range the RC domain uses. Kept as a hard clamp (same reasoning as the
/// servo's ±90° clamp): a config typo must not be able to hand the ESC a pulse
/// it will interpret as garbage. Finding behaviour *outside* this range is what
/// `servo-test` is for.
const PULSE_MIN_US: u32 = 1000;
const PULSE_MAX_US: u32 = 2000;

/// Nominal RC neutral, used when a configured neutral is out of range.
const NEUTRAL_US: u32 = 1500;

/// ESC calibration and limits. Varies per unit — every field here is a
/// calibration input, not a guess.
#[derive(Clone, Copy)]
pub struct EscConfig {
    /// Abstract speed that maps to the configured full pulse span. **Must equal
    /// [`ControlConfig::motor_max_speed`](crate::control::ControlConfig::motor_max_speed)**
    /// — take both from one constant in the bin, or the full-throttle point
    /// silently moves.
    pub speed_full: i32,
    /// Pulse width that means neutral/stop, in µs. Must lie inside
    /// [`PULSE_MIN_US`]..=[`PULSE_MAX_US`]; a value outside is refused at
    /// construction (see [`Esc::new`]).
    pub neutral_us: u32,
    /// Travel from neutral to full forward, in µs.
    pub forward_span_us: u32,
    /// Travel from neutral to full reverse, in µs (often shorter than forward
    /// on this class of ESC).
    pub reverse_span_us: u32,
    /// `|speed|` at or below this is commanded as an *exact* neutral pulse.
    ///
    /// This is boundary condition ① of the plan and it is a wide value on
    /// purpose: it has to swallow what the stick still reports at rest *after*
    /// `ControlConfig::ly_deadzone` has been applied, otherwise the ESC never
    /// sees neutral, never releases its reverse latch, and the car has no
    /// reverse for reasons that look exactly like hardware. The resting sticks
    /// are already printed every 100 ms (`ly=…`), so this is measurable rather
    /// than guessable.
    pub deadzone: i32,
    /// Neutral hold after power-up/reset before throttle is accepted, in ms.
    pub arm_ms: u64,
}

impl EscConfig {
    /// Pulse for an abstract speed, with the deadzone treated as the bottom of
    /// the range rather than a hole in it: the mapping is continuous at the
    /// threshold, so a stick just past the deadzone asks for a pulse just past
    /// neutral instead of stepping a few µs.
    pub fn pulse_for(&self, speed: i32) -> u32 {
        let full = self.speed_full.max(1);
        // A negative deadzone would make `|s| <= deadzone` false at *centre*,
        // i.e. a centred stick would select the reverse span. Config errors must
        // fail towards "no drive", never towards "drive".
        let deadzone = self.deadzone.max(0);
        let s = speed.clamp(-full, full);
        if s.abs() <= deadzone {
            return self.neutral_us;
        }
        let over = s.abs() - deadzone;
        let span = if s > 0 {
            self.forward_span_us
        } else {
            self.reverse_span_us
        };
        let travel = (span as i64 * over as i64 / (full - deadzone).max(1) as i64) as u32;
        // Saturating, not `-`: `travel` is bounded by `span`, which a config
        // typo can make larger than `neutral_us`. A wrapping subtraction would
        // turn "reverse" into a huge value that the clamp then pins at
        // PULSE_MAX_US — i.e. the misconfiguration would command *full forward*
        // instead of going to the reverse end.
        let pulse = if s > 0 {
            self.neutral_us.saturating_add(travel)
        } else {
            self.neutral_us.saturating_sub(travel)
        };
        pulse.clamp(PULSE_MIN_US, PULSE_MAX_US)
    }

    /// The abstract speed that lands on `pulse_us`: the inverse of
    /// [`pulse_for`](Self::pulse_for).
    ///
    /// Exists so a bin can express a *pulse* the driver will emit as a control-
    /// layer speed — the shallow braking pulse a latching ESC needs before it
    /// accepts reverse is measured in µs, and hand-computing the speed would rot
    /// the moment a span changes.
    pub fn speed_for_pulse(&self, pulse_us: u32) -> i32 {
        let full = self.speed_full.max(1);
        let deadzone = self.deadzone.max(0);
        let range = (full - deadzone).max(1) as i64;
        let (span, travel) = if pulse_us >= self.neutral_us {
            (self.forward_span_us, pulse_us - self.neutral_us)
        } else {
            (self.reverse_span_us, self.neutral_us - pulse_us)
        };
        let over = (travel as i64 * range / span.max(1) as i64) as i32;
        let magnitude = (over + deadzone).clamp(0, full);
        if pulse_us >= self.neutral_us {
            magnitude
        } else {
            -magnitude
        }
    }
}

/// Brushed ESC on one LEDC channel.
pub struct Esc<Ch> {
    channel: Ch,
    cfg: EscConfig,
    /// End of the arming hold.
    armed_at: Instant,
}

impl<Ch: ChannelHW> Esc<Ch> {
    /// Create the driver and start emitting neutral immediately.
    ///
    /// Emitting starts here rather than at first use because the pulse train is
    /// the arming signal: the earlier it is continuous, the earlier the unit is
    /// ready. Neutral is validated here — see [`EscConfig::neutral_us`].
    pub fn new(channel: Ch, cfg: EscConfig) -> Self {
        let cfg = if (PULSE_MIN_US..=PULSE_MAX_US).contains(&cfg.neutral_us) {
            cfg
        } else {
            // Clamping instead would silently relocate neutral: a typo of
            // `neutral_us: 900` would clamp to 1000 µs = *full reverse*, held
            // through the whole arming window and every stop.
            defmt::error!(
                "Esc: neutral_us={} outside {}-{} µs — falling back to {}",
                cfg.neutral_us,
                PULSE_MIN_US,
                PULSE_MAX_US,
                NEUTRAL_US
            );
            EscConfig {
                neutral_us: NEUTRAL_US,
                ..cfg
            }
        };

        let mut this = Self {
            channel,
            cfg,
            armed_at: Instant::now(),
        };
        this.write_neutral();
        this
    }

    /// Pulse for an abstract speed — see [`EscConfig::pulse_for`].
    fn write_neutral(&mut self) {
        self.write_pulse(self.cfg.neutral_us);
    }

    fn write_pulse(&mut self, pulse_us: u32) {
        let counts = pulse_to_counts(pulse_us.clamp(PULSE_MIN_US, PULSE_MAX_US));
        self.channel.set_duty_hw(counts);
    }
}

impl<Ch: ChannelHW> MotorDriver for Esc<Ch> {
    fn set_speed(&mut self, speed: i32) {
        // Arming hold: no throttle until the unit has seen neutral long enough.
        // Any reset re-runs this (the bin re-constructs the driver).
        if Instant::now() < self.armed_at + Duration::from_millis(self.cfg.arm_ms) {
            self.write_neutral();
            return;
        }
        self.write_pulse(self.cfg.pulse_for(speed));
    }

    /// Continuous neutral — *not* silence (see the module docs).
    fn stop(&mut self) {
        self.write_neutral();
    }

    /// (Re)start the neutral hold and emit it now.
    fn enable(&mut self) {
        self.armed_at = Instant::now();
        self.write_neutral();
    }
}
