//! Battery voltage monitor.
//!
//! Processes calibrated ADC readings from a resistor-divider network
//! to compute battery voltage, auto-detect battery type, and signal
//! low-voltage alarms.
//!
//! This module does **not** own the ADC hardware — the caller configures
//! the ADC and feeds calibrated mV readings into [`BatteryMonitor::sample`].
//!
//! ## Typical usage
//!
//! ```ignore
//! let mut monitor = BatteryMonitor::new(147, 47); // R1=100k, R2=47k
//!
//! loop {
//!     let adc_mv = adc.read_blocking(&mut bat_pin);
//!     monitor.sample(adc_mv);
//!
//!     if monitor.filled() && monitor.battery_type().is_none() {
//!         let t = monitor.lock_type();
//!         info!("battery type locked: {:?}", t);
//!     }
//!
//!     if monitor.is_low() {
//!         // sound alarm, flash LED, etc.
//!     }
//! }
//! ```

/// Number of samples in the moving-average window.
const AVG_WINDOW: usize = 32;

/// If a new sample differs from the current window average by more than
/// this (mV at ADC pin), the window is reset — handles battery plug/unplug.
const JUMP_THRESHOLD_MV: u16 = 400;

// ── Battery type ──────────────────────────────────────────────────────

/// Detected battery configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BatteryType {
    /// 1S LiPo: 3.7 V nominal, 3.3–4.2 V working range.
    LiPo1S,
    /// 4×AA alkaline or NiMH: 4.4–6.4 V working range.
    Aax4,
    /// 2S LiPo: 7.4 V nominal, 7.0–8.4 V working range.
    LiPo2S,
}

impl BatteryType {
    /// Low-voltage alarm threshold, in mV.
    pub fn low_threshold_mv(self) -> u32 {
        match self {
            Self::LiPo1S => 3500,
            Self::Aax4 => 4400,
            Self::LiPo2S => 7000,
        }
    }
}

// ── Reading snapshot ──────────────────────────────────────────────────

/// One-shot snapshot of the current battery state.
#[derive(Debug, Clone, Copy)]
pub struct BatteryReading {
    /// Estimated battery voltage, in mV.
    pub bat_mv: u32,
    /// Averaged ADC pin voltage, in mV.
    pub adc_mv: u16,
    /// Locked battery type, if any.
    pub battery_type: Option<BatteryType>,
    /// `true` if voltage is below the alarm threshold for the locked type.
    pub is_low: bool,
}

// ── Monitor ───────────────────────────────────────────────────────────

/// Stateful battery monitor with moving-average smoothing.
///
/// Stores a configurable voltage-divider ratio so that the same monitor
/// works with any resistor pair:
///
/// ```text
/// V_BAT = V_ADC × divider_num / divider_den
/// ```
///
/// For R₁ = 100 kΩ, R₂ = 47 kΩ:
///
/// ```text
/// V_ADC = V_BAT × 47 / 147   →   V_BAT = V_ADC × 147 / 47
/// ```
///
/// so `new(147, 47)`.
pub struct BatteryMonitor {
    /// `V_BAT = V_ADC × divider_num / divider_den`
    divider_num: u32,
    divider_den: u32,

    battery_type: Option<BatteryType>,

    /// Ring buffer of calibrated ADC pin voltages (mV).
    samples: [u16; AVG_WINDOW],
    idx: usize,
    filled: bool,
}

impl BatteryMonitor {
    /// Create a new monitor with the given voltage-divider ratio.
    ///
    /// `divider_num` / `divider_den` = (R₁ + R₂) / R₂.
    pub fn new(divider_num: u32, divider_den: u32) -> Self {
        Self {
            divider_num,
            divider_den,
            battery_type: None,
            samples: [0; AVG_WINDOW],
            idx: 0,
            filled: false,
        }
    }

    // ── Data feed ──────────────────────────────────────────────────

    /// Feed one calibrated ADC pin reading (mV) into the moving average.
    ///
    /// If the new sample jumps more than `JUMP_THRESHOLD_MV` from the
    /// current average, the entire window is reset — this handles
    /// battery plug / unplug events transparently.
    pub fn sample(&mut self, adc_mv: u16) {
        // Jump detection: only when we have enough history to compare
        if self.idx > 0 || self.filled {
            let avg = self.avg_adc_mv();
            if adc_mv.abs_diff(avg) > JUMP_THRESHOLD_MV {
                self.samples.fill(adc_mv);
                self.idx = 1;
                self.filled = false;
                return;
            }
        }

        self.samples[self.idx] = adc_mv;
        self.idx = (self.idx + 1) % AVG_WINDOW;
        if self.idx == 0 {
            self.filled = true;
        }
    }

    /// Whether the moving-average window is full.
    pub fn filled(&self) -> bool {
        self.filled
    }

    // ── Voltage query ──────────────────────────────────────────────

    /// Averaged ADC pin voltage over the window, in mV.
    pub fn avg_adc_mv(&self) -> u16 {
        let count = if self.filled { AVG_WINDOW } else { self.idx };
        if count == 0 {
            return 0;
        }
        let sum: u32 = self.samples.iter().take(count).map(|&v| v as u32).sum();
        (sum / count as u32) as u16
    }

    /// Estimated battery voltage, in mV.
    pub fn bat_mv(&self) -> u32 {
        self.avg_adc_mv() as u32 * self.divider_num / self.divider_den
    }

    // ── Type detection ─────────────────────────────────────────────

    /// Locked battery type, if [`lock_type`](Self::lock_type) has been
    /// called.
    pub fn battery_type(&self) -> Option<BatteryType> {
        self.battery_type
    }

    /// Auto-detect the battery type from current voltage and lock it.
    ///
    /// Call once after the window has filled. The detection thresholds
    /// sit in the gap between battery type voltage ranges:
    ///
    /// | Threshold | Result |
    /// |-----------|--------|
    /// | ≥ 7000 mV | `LiPo2S` |
    /// | ≥ 4500 mV | `Aax4` |
    /// | < 4500 mV | `LiPo1S` |
    pub fn lock_type(&mut self) -> BatteryType {
        let mv = self.bat_mv();
        let t = if mv >= 7000 {
            BatteryType::LiPo2S
        } else if mv >= 4500 {
            BatteryType::Aax4
        } else {
            BatteryType::LiPo1S
        };
        self.battery_type = Some(t);
        t
    }

    // ── Alarm ──────────────────────────────────────────────────────

    /// Whether the battery is below its low-voltage alarm threshold.
    ///
    /// Returns `false` if the battery type has not been locked yet.
    pub fn is_low(&self) -> bool {
        match self.battery_type {
            Some(t) => self.bat_mv() < t.low_threshold_mv(),
            None => false,
        }
    }

    // ── Snapshot ───────────────────────────────────────────────────

    /// Full state snapshot for logging or telemetry.
    pub fn reading(&self) -> BatteryReading {
        BatteryReading {
            bat_mv: self.bat_mv(),
            adc_mv: self.avg_adc_mv(),
            battery_type: self.battery_type,
            is_low: self.is_low(),
        }
    }
}
