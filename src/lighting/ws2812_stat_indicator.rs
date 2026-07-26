//! WS2812 state indicator — single-LED status light for debug/telemetry.
//!
//! # Design
//!
//! Pure state machine: receives [`StatusInput`] each tick, returns an [`Rgb`]
//! value. Does **not** own the WS2812 hardware — the caller decides when and
//! how to push the resulting colour to the LED.
//!
//! ## State priority (highest first)
//!
//! 1. PS2 disconnected → yellow fast blink
//! 2. Low battery      → red slow blink
//! 3. Boot fade        → white ramp-up (ignores overlays)
//! 4. Running          → green (Servo) or blue (Diff)

use crate::control::DriveMode;

// ── Input / Output ───────────────────────────────────────────────────

/// Signals fed from the main loop every tick.
pub struct StatusInput {
    /// Current drive mode.
    pub mode: DriveMode,
    /// Battery voltage is below the low threshold.
    pub battery_is_low: bool,
    /// PS2 controller is connected (sending valid analog frames).
    pub ps2_connected: bool,
    /// Monotonic tick counter — starts at 0 on boot and increments each
    /// main-loop iteration.  Used for boot-fade progress and blink phase.
    pub ticks_since_boot: u32,
}

/// RGB colour with WS2812 byte order (GRB — Green sent first).
#[derive(Clone, Copy, PartialEq, defmt::Format)]
pub struct Rgb {
    pub g: u8,
    pub r: u8,
    pub b: u8,
}

// ── Pre-defined colours ──────────────────────────────────────────────

impl Rgb {
    pub const OFF: Self = Self { g: 0, r: 0, b: 0 };
}

// ── State machine ────────────────────────────────────────────────────

/// Blink pattern parameters.
struct Blink {
    /// How many ticks the LED stays on.
    on_ticks: u32,
    /// How many ticks the LED stays off.
    off_ticks: u32,
}

/// Tick-based phase counter.
///
/// Wraps around at `period` to avoid unbounded growth.  Using modulo
/// over a local counter means the blink is unaffected by `ticks_since_boot`
/// rolling over (which would take ~4 years at 30 Hz anyway).
struct Phase {
    count: u32,
    period: u32,
}

impl Phase {
    fn new(period: u32) -> Self {
        Self { count: 0, period }
    }

    /// Advance one tick, return `true` during the "on" half.
    fn tick(&mut self, on_ticks: u32) -> bool {
        let on = self.count < on_ticks;
        self.count += 1;
        if self.count >= self.period {
            self.count = 0;
        }
        on
    }
}

pub struct Ws2812StatIndicator {
    /// Number of ticks the boot fade takes.
    boot_fade_ticks: u32,
    /// Tracks slow-blink phase (low battery).
    slow: Phase,
    /// Tracks fast-blink phase (disconnected).
    fast: Phase,
}

impl Ws2812StatIndicator {
    /// Maximum brightness for solid / blink colours (0-255).
    const MAX: u8 = 8;

    /// Fast blink: ~165 ms on, 165 ms off  (5 ticks × 33 ms).
    const FAST_BLINK: Blink = Blink {
        on_ticks: 5,
        off_ticks: 10, // period = 5 + (10-5)… see tick logic
    };
    /// Slow blink: ~500 ms on, 500 ms off (15 ticks × 33 ms).
    const SLOW_BLINK: Blink = Blink {
        on_ticks: 15,
        off_ticks: 30,
    };

    /// Create a new indicator.
    ///
    /// `boot_fade_ticks` controls how long the white fade-in lasts
    /// (e.g. 15 ticks ≈ 500 ms at 33 ms/tick).
    pub fn new(boot_fade_ticks: u32) -> Self {
        Self {
            boot_fade_ticks,
            slow: Phase::new(Self::SLOW_BLINK.off_ticks),
            fast: Phase::new(Self::FAST_BLINK.off_ticks),
        }
    }

    /// Evaluate state and return the colour to display this tick.
    pub fn update(&mut self, input: &StatusInput) -> Rgb {
        // Layer 2: PS2 disconnected (fast yellow blink)
        if !input.ps2_connected {
            let on = self.fast.tick(Self::FAST_BLINK.on_ticks);
            return if on {
                Self::yellow(Self::MAX)
            } else {
                Rgb::OFF
            };
        }

        // Layer 1: Low battery (slow red blink)
        if input.battery_is_low {
            let on = self.slow.tick(Self::SLOW_BLINK.on_ticks);
            return if on { Self::red(Self::MAX) } else { Rgb::OFF };
        }

        // Layer 0a: Boot fade
        if input.ticks_since_boot < self.boot_fade_ticks {
            let progress = input.ticks_since_boot as u16;
            let max = self.boot_fade_ticks as u16;
            let b = ((progress as u32 * Self::MAX as u32) / max as u32) as u8;
            return Self::white(b);
        }

        // Layer 0b: Running — drive-mode colour
        match input.mode {
            DriveMode::Servo => Self::green(Self::MAX),
            DriveMode::Diff => Self::blue(Self::MAX),
        }
    }

    // ── colour helpers ────────────────────────────────────────────

    fn red(b: u8) -> Rgb {
        Rgb { g: 0, r: b, b: 0 }
    }
    fn green(b: u8) -> Rgb {
        Rgb { g: b, r: 0, b: 0 }
    }
    fn blue(b: u8) -> Rgb {
        Rgb { g: 0, r: 0, b }
    }
    fn yellow(b: u8) -> Rgb {
        Rgb { g: b, r: b, b: 0 }
    }
    fn white(b: u8) -> Rgb {
        Rgb { g: b, r: b, b }
    }
}
