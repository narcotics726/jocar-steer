//! TB6612FNG dual motor driver module.
//!
//! Drives two DC motors (right and left) via direct GPIO for the direction
//! logic inputs and two LEDC PWM channels for speed control.
//!
//! Pin mapping:
//! | ESP32 GPIO | Signal | Motor       |
//! |------------|--------|-------------|
//! | G9         | AIN1   | Right IN1   |
//! | G10        | AIN2   | Right IN2   |
//! | G11        | BIN1   | Left IN1    |
//! | G12        | BIN2   | Left IN2    |
//! | G13        | STBY   | Shared      |
//! | G1         | PWMA   | Right PWM   |
//! | G2         | PWMB   | Left PWM    |

use defmt::info;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use esp_hal::ledc::channel::ChannelHW;

/// TB6612FNG dual motor driver abstraction.
///
/// Controls two DC motors (right / left) through five direct GPIO output pins
/// for the direction logic and the shared STDBY line, plus two LEDC PWM
/// channels for speed.
///
/// # Type parameters
///
/// * `ChA` — LEDC channel for the right motor (PWMA), must implement [`ChannelHW`].
/// * `ChB` — LEDC channel for the left motor (PWMB), must implement [`ChannelHW`].
pub struct Tb6612<'d, ChA, ChB> {
    ain1: Output<'d>,
    ain2: Output<'d>,
    bin1: Output<'d>,
    bin2: Output<'d>,
    stby: Output<'d>,
    pwm_a: ChA,
    pwm_b: ChB,
}

impl<'d, ChA, ChB> Tb6612<'d, ChA, ChB>
where
    ChA: ChannelHW,
    ChB: ChannelHW,
{
    /// Create a new `Tb6612` driver.
    ///
    /// All direction pins start low (coast) and STBY = 0 (motors disabled /
    /// standby). The PWM channels are not modified during construction.
    pub fn new<A1, A2, B1, B2, ST>(
        ain1: A1,
        ain2: A2,
        bin1: B1,
        bin2: B2,
        stby: ST,
        pwm_a: ChA,
        pwm_b: ChB,
    ) -> Self
    where
        A1: OutputPin + 'd,
        A2: OutputPin + 'd,
        B1: OutputPin + 'd,
        B2: OutputPin + 'd,
        ST: OutputPin + 'd,
    {
        info!("Tb6612: new() — direct GPIO, STBY=0 (standby)");
        Self {
            ain1: Output::new(ain1, Level::Low, OutputConfig::default()),
            ain2: Output::new(ain2, Level::Low, OutputConfig::default()),
            bin1: Output::new(bin1, Level::Low, OutputConfig::default()),
            bin2: Output::new(bin2, Level::Low, OutputConfig::default()),
            stby: Output::new(stby, Level::Low, OutputConfig::default()),
            pwm_a,
            pwm_b,
        }
    }

    // ── Public API ──────────────────────────────────────────────────────

    /// Enable both motors by driving the STDBY line high.
    ///
    /// After calling `enable()`, the motors respond to the last-set direction
    /// and PWM duty.
    pub fn enable(&mut self) {
        self.stby.set_high();
        info!("Tb6612: enable()");
    }

    /// Disable both motors by driving the STDBY line low.
    ///
    /// All motor outputs go to a high-impedance (coast) state regardless of
    /// the IN1/IN2 pins.
    pub fn standby(&mut self) {
        self.stby.set_low();
        info!("Tb6612: standby()");
    }

    /// Set the right motor speed and direction.
    ///
    /// `speed` range (12-bit signed):
    /// - Positive → forward at duty `speed` (clamped to 4095)
    /// - Negative → reverse at duty `-speed` (clamped to 4095)
    /// - 0 → coast (high-impedance freewheel via IN1=0, IN2=0, PWM=0).
    ///   Coasting (not short-braking) on neutral avoids the extra current
    ///   draw/heat of a short brake. Use [`brake_right`](Self::brake_right)
    ///   explicitly if an active stop is wanted.
    pub fn set_right(&mut self, speed: i32) {
        let speed = speed.clamp(-4095, 4095);
        if speed > 0 {
            self.ain1.set_high();
            self.ain2.set_low();
            self.pwm_a.set_duty_hw(speed as u32);
        } else if speed < 0 {
            self.ain1.set_low();
            self.ain2.set_high();
            self.pwm_a.set_duty_hw((-speed) as u32);
        } else {
            self.coast_right();
        }
    }

    /// Set the left motor speed and direction.
    ///
    /// Semantics match [`set_right`](Self::set_right) (0 → coast).
    pub fn set_left(&mut self, speed: i32) {
        let speed = speed.clamp(-4095, 4095);
        if speed > 0 {
            self.bin1.set_high();
            self.bin2.set_low();
            self.pwm_b.set_duty_hw(speed as u32);
        } else if speed < 0 {
            self.bin1.set_low();
            self.bin2.set_high();
            self.pwm_b.set_duty_hw((-speed) as u32);
        } else {
            self.coast_left();
        }
    }

    /// Short brake on the right motor (IN1=1, IN2=1, PWM=0).
    pub fn brake_right(&mut self) {
        self.ain1.set_high();
        self.ain2.set_high();
        self.pwm_a.set_duty_hw(0);
    }

    /// Short brake on the left motor (IN1=1, IN2=1, PWM=0).
    pub fn brake_left(&mut self) {
        self.bin1.set_high();
        self.bin2.set_high();
        self.pwm_b.set_duty_hw(0);
    }

    /// Coast the right motor (IN1=0, IN2=0, PWM=0).
    pub fn coast_right(&mut self) {
        self.ain1.set_low();
        self.ain2.set_low();
        self.pwm_a.set_duty_hw(0);
    }

    /// Coast the left motor (IN1=0, IN2=0, PWM=0).
    pub fn coast_left(&mut self) {
        self.bin1.set_low();
        self.bin2.set_low();
        self.pwm_b.set_duty_hw(0);
    }
}

/// TB6612FNG single-motor driver (single motor + driveshaft chassis).
///
/// Uses only channel A: AIN1/AIN2 for direction, one LEDC PWM channel for
/// speed and the shared STBY line. Direction/speed semantics match
/// [`Tb6612::set_right`] (positive → forward, negative → reverse, 0 → coast).
///
/// # Type parameters
///
/// * `Ch` — LEDC channel for the motor PWM (PWMA), must implement [`ChannelHW`].
pub struct Tb6612Single<'d, Ch> {
    ain1: Output<'d>,
    ain2: Output<'d>,
    stby: Output<'d>,
    pwm: Ch,
}

impl<'d, Ch> Tb6612Single<'d, Ch>
where
    Ch: ChannelHW,
{
    /// Create a new single-motor driver.
    ///
    /// Direction pins start low (coast) and STBY = 0 (motors disabled /
    /// standby). The PWM channel is not modified during construction.
    pub fn new<A1, A2, ST>(ain1: A1, ain2: A2, stby: ST, pwm: Ch) -> Self
    where
        A1: OutputPin + 'd,
        A2: OutputPin + 'd,
        ST: OutputPin + 'd,
    {
        info!("Tb6612Single: new() — direct GPIO, STBY=0 (standby)");
        Self {
            ain1: Output::new(ain1, Level::Low, OutputConfig::default()),
            ain2: Output::new(ain2, Level::Low, OutputConfig::default()),
            stby: Output::new(stby, Level::Low, OutputConfig::default()),
            pwm,
        }
    }

    /// Enable the motor by driving the STDBY line high.
    pub fn enable(&mut self) {
        self.stby.set_high();
        info!("Tb6612Single: enable()");
    }

    /// Disable the motor by driving the STDBY line low (coast).
    pub fn standby(&mut self) {
        self.stby.set_low();
        info!("Tb6612Single: standby()");
    }

    /// Set motor speed and direction.
    ///
    /// `speed` range (12-bit signed): positive → forward, negative → reverse,
    /// 0 → coast (high-impedance freewheel via IN1=0, IN2=0, PWM=0). Use
    /// [`brake`](Self::brake) for an active stop.
    pub fn set_motor(&mut self, speed: i32) {
        let speed = speed.clamp(-4095, 4095);
        if speed > 0 {
            self.ain1.set_high();
            self.ain2.set_low();
            self.pwm.set_duty_hw(speed as u32);
        } else if speed < 0 {
            self.ain1.set_low();
            self.ain2.set_high();
            self.pwm.set_duty_hw((-speed) as u32);
        } else {
            self.coast();
        }
    }

    /// Short brake (IN1=1, IN2=1, PWM=0).
    pub fn brake(&mut self) {
        self.ain1.set_high();
        self.ain2.set_high();
        self.pwm.set_duty_hw(0);
    }

    /// Coast (IN1=0, IN2=0, PWM=0).
    pub fn coast(&mut self) {
        self.ain1.set_low();
        self.ain2.set_low();
        self.pwm.set_duty_hw(0);
    }
}

impl<Ch> crate::chassis::MotorDriver for Tb6612Single<'_, Ch>
where
    Ch: ChannelHW,
{
    fn set_speed(&mut self, speed: i32) {
        self.set_motor(speed);
    }

    /// Coast. A brushed driver's "no drive" is a zero command; the ESC
    /// implementation of this trait does very different work (it must keep
    /// pulsing neutral instead of going quiet).
    fn stop(&mut self) {
        self.coast();
    }

    /// STBY high. [`Tb6612Single::new`] deliberately leaves STBY low, so this
    /// is the call that actually arms the driver — idempotent, but not
    /// redundant.
    fn enable(&mut self) {
        Tb6612Single::enable(self);
    }
}
