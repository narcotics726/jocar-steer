//! Shared 50 Hz RC pulse domain — the steering servo and the ESC throttle.
//!
//! Both outputs are RC pulses on the **same** LEDC timer, so three facts have to
//! agree: the timer's frequency, its duty resolution, and the pulse→counts
//! conversion the drivers use. Keeping them in one place is not tidiness — if
//! the resolution in a bin's timer config and in the conversion ever disagree,
//! the servo receives a pulse several times too short and slams into its
//! mechanical stop (buzzing, stall current, gear damage), which is both
//! dangerous and hard to attribute.
//!
//! So the split is by kind, not by module:
//! - bins call [`init`] once and configure their timer with [`timer_config`],
//! - drivers convert with [`pulse_to_counts`],
//! - raising the resolution (if the ESC's throttle ever feels coarse) is a
//!   change to this file rather than a hunt through bins and drivers.

use esp_hal::ledc::{LSGlobalClkSource, Ledc, timer};
use esp_hal::time::Rate;

/// RC pulse period, as a frequency.
pub const PWM_HZ: u32 = 50;

/// The same period in microseconds — the unit pulse widths are expressed in.
pub const PERIOD_US: u32 = 20_000;

/// LEDC duty resolution in bits. 12-bit → 4096 counts per period → 4.88 µs per
/// step, about 1 % of a 500 µs throttle span. Must match [`DUTY`].
pub const DUTY_BITS: u32 = 12;

/// Counts per period: `1 << DUTY_BITS`.
pub const DUTY_MAX: u32 = 1 << DUTY_BITS;

/// The same resolution in the form the timer wants. Must match [`DUTY_BITS`].
pub const DUTY: timer::config::Duty = timer::config::Duty::Duty12Bit;

// Compile-time proof that the redundant representations above agree: changing
// one without the others is a build error rather than a pulse that is silently
// half or twice as long as intended.
const _: () = assert!(PERIOD_US == 1_000_000 / PWM_HZ);
const _: () = assert!(DUTY_MAX == 1 << DUTY_BITS);
const _: () = assert!(DUTY as u32 == DUTY_BITS);

/// Select the LEDC clock every RC output on these cars depends on.
///
/// This is the group's one *global* fact, and it is easy to miss: on this chip
/// esp-hal ignores the per-timer `clock_source` field when it computes the
/// divider (it always uses the APB clock) — what actually steers the source is
/// this call. A bin that configures a timer from [`timer_config`] but skips
/// `init` gets a period and a pulse width both scaled away from 50 Hz and
/// 1500 µs, which is the failure this module exists to prevent.
pub fn init(ledc: &mut Ledc<'_>) {
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
}

/// Timer configuration every RC output on these cars must use.
pub fn timer_config() -> timer::config::Config<timer::LSClockSource> {
    timer::config::Config {
        duty: DUTY,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_hz(PWM_HZ),
    }
}

/// Pulse width in µs → raw LEDC duty count.
///
/// Callers stay inside the RC range (≤ 2000 µs — `esc` and `steering` both clamp
/// there), so the product cannot overflow, and the LEDC hardware itself caps the
/// resolution at 14 bits on this chip.
pub fn pulse_to_counts(pulse_us: u32) -> u32 {
    DUTY_MAX * pulse_us / PERIOD_US
}
