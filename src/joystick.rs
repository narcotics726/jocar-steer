//! Joystick abstraction: two-axis analog stick read via ESP32 ADC1.
//!
//! Wraps the ADC and pin types so the main loop only calls `read()` and
//! accesses `center_x` directly.

use esp_hal::analog::adc::{Adc, AdcCalScheme, AdcChannel, AdcPin, RegisterAccess};
use esp_hal::Blocking;

/// Two-axis analog joystick backed by an ESP32 ADC unit.
///
/// Type parameters are inferred at construction time; callers don't need to
/// write them out.
pub struct Joystick<'d, ADCX, PX, PY, CS> {
    adc: Adc<'d, ADCX, Blocking>,
    pin_x: AdcPin<PX, ADCX, CS>,
    pin_y: AdcPin<PY, ADCX, CS>,
    /// X-axis center (calibrated at startup).
    pub center_x: i32,
}

/// A single two-axis reading from the joystick.
pub struct Reading {
    pub x: i32,
    pub y: i32,
}

impl<'d, ADCX, PX, PY, CS> Joystick<'d, ADCX, PX, PY, CS> {
    /// Wrap an already-configured ADC and its pin pair.
    pub fn new(
        adc: Adc<'d, ADCX, Blocking>,
        pin_x: AdcPin<PX, ADCX, CS>,
        pin_y: AdcPin<PY, ADCX, CS>,
    ) -> Self {
        Self {
            adc,
            pin_x,
            pin_y,
            center_x: 0,
        }
    }
}

impl<'d, ADCX, PX, PY, CS> Joystick<'d, ADCX, PX, PY, CS>
where
    ADCX: RegisterAccess + 'd,
    PX: AdcChannel,
    PY: AdcChannel,
    CS: AdcCalScheme<ADCX>,
{
    /// Blocking single-shot read on X axis (raw ADC counts).
    pub fn read_x(&mut self) -> u16 {
        loop {
            if let Ok(v) = self.adc.read_oneshot(&mut self.pin_x) {
                return v;
            }
        }
    }

    /// Blocking single-shot read on Y axis (raw ADC counts).
    pub fn read_y(&mut self) -> u16 {
        loop {
            if let Ok(v) = self.adc.read_oneshot(&mut self.pin_y) {
                return v;
            }
        }
    }

    /// Read X axis, averaging `n` samples to suppress ADC noise.
    pub fn read_x_avg(&mut self, n: u32) -> i32 {
        let mut sum: i32 = 0;
        for _ in 0..n {
            sum += self.read_x() as i32;
        }
        sum / n as i32
    }

    /// Read Y axis, averaging `n` samples.
    pub fn read_y_avg(&mut self, n: u32) -> i32 {
        let mut sum: i32 = 0;
        for _ in 0..n {
            sum += self.read_y() as i32;
        }
        sum / n as i32
    }

    /// Read both axes (4-sample average each) and return X relative to center.
    pub fn read(&mut self) -> Reading {
        Reading {
            x: self.read_x_avg(4),
            y: self.read_y_avg(4),
        }
    }
}
