//! PlayStation 2 controller driver via bit-bang protocol.
//!
//! The PS2 protocol uses SPI Mode 3 (CPOL=1, CPHA=1) but LSB-first,
//! which is not supported by the ESP32-S3 hardware SPI. This driver
//! bit-bangs the protocol using GPIO.
//!
//! Pin assignment (default for this project):
//!   DAT  = GPIO4   (MISO, input)
//!   CMD  = GPIO5   (MOSI, output)
//!   CLK  = GPIO7   (SCLK, output, idle-high)
//!   ATT  = GPIO6   (CS, output, active-low)

use esp_hal::gpio::{Input, InputConfig, InputPin, Level, Output, OutputConfig, OutputPin, Pull};

/// Event returned by [`Ps2Controller::read`] so callers can react to
/// analog/digital transitions without tracking state themselves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Ps2Event {
    /// Normal analog-mode poll with live stick data.
    Analog(Ps2State),
    /// The controller just dropped from analog to digital mode.
    LostAnalog,
    /// The controller just recovered analog mode.
    RecoveredAnalog,
}

/// PS2 controller state decoded from a 9-byte response packet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ps2State {
    /// Raw 9-byte packet.
    pub raw: [u8; 9],
}

/// Button bit positions in the PS2 response.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    Select,
    L3,
    R3,
    Start,
    Up,
    Right,
    Down,
    Left,
    L2,
    R2,
    L1,
    R1,
    Triangle,
    Circle,
    Cross,
    Square,
}

impl Ps2State {
    /// Check if a specific button is pressed (active-low in the bitmask).
    ///
    /// This wireless receiver inserts a fixed ID byte (0x5A) at raw[2],
    /// shifting all subsequent data by +1 vs standard PS2 layout:
    ///   raw[3] = first button group  (standard raw[2])
    ///   raw[4] = second button group (standard raw[3])
    pub fn pressed(&self, btn: Button) -> bool {
        match btn {
            // First group: raw[3]
            Button::Select => self.raw[3] & 0x01 == 0,
            Button::L3 => self.raw[3] & 0x02 == 0,
            Button::R3 => self.raw[3] & 0x04 == 0,
            Button::Start => self.raw[3] & 0x08 == 0,
            Button::Up => self.raw[3] & 0x10 == 0,
            Button::Right => self.raw[3] & 0x20 == 0,
            Button::Down => self.raw[3] & 0x40 == 0,
            Button::Left => self.raw[3] & 0x80 == 0,
            // Second group: raw[4] (shifted by +1 due to extra ID byte)
            Button::L2 => self.raw[4] & 0x01 == 0,
            Button::R2 => self.raw[4] & 0x02 == 0,
            Button::L1 => self.raw[4] & 0x04 == 0,
            Button::R1 => self.raw[4] & 0x08 == 0,
            Button::Triangle => self.raw[4] & 0x10 == 0,
            Button::Circle => self.raw[4] & 0x20 == 0,
            Button::Cross => self.raw[4] & 0x40 == 0,
            Button::Square => self.raw[4] & 0x80 == 0,
        }
    }

    /// Whether the poll carries live analog stick data.
    ///
    /// Byte 1 is `(mode << 4) | words`, where `words` is the number of
    /// 16-bit data words following the 0x5A header. Analog packets carry
    /// >=3 words (6 bytes: 2 button + 4 axis); digital (0x41) carries 1.
    ///
    /// This clone receiver reports `0xf3` (mode nibble 0xF = config mode)
    /// rather than the textbook `0x73`, but still streams valid stick data,
    /// so we accept both the analog (0x7_) and config (0xF_) mode nibbles
    /// and simply require enough data words.
    pub fn is_analog(&self) -> bool {
        let mode = self.raw[1] & 0xF0;
        (mode == 0x70 || mode == 0xF0) && (self.raw[1] & 0x0F) >= 3
    }

    /// Check if the response indicates digital mode (mode nibble 0x4, e.g. 0x41).
    pub fn is_digital(&self) -> bool {
        self.raw[1] & 0xF0 == 0x40
    }

    /// Right joystick X axis (0 = left, 128 = center, 255 = right).
    pub fn rx(&self) -> u8 {
        self.raw[5]
    }
    /// Right joystick Y axis (0 = up, 128 = center, 255 = down).
    pub fn ry(&self) -> u8 {
        self.raw[6]
    }
    /// Left joystick X axis (0 = left, 128 = center, 255 = right).
    pub fn lx(&self) -> u8 {
        self.raw[7]
    }
    /// Left joystick Y axis (0 = up, 128 = center, 255 = down).
    pub fn ly(&self) -> u8 {
        self.raw[8]
    }
}

/// PS2 controller driver using bit-bang SPI Mode 3, LSB-first.
pub struct Ps2Controller<'d> {
    clk: Output<'d>,
    cmd: Output<'d>,
    dat: Input<'d>,
    att: Output<'d>,
    was_analog: bool,
}

impl<'d> Ps2Controller<'d> {
    /// Create a new PS2 controller driver.
    ///
    /// Initializes all pins to idle state (CLK high, CMD high, ATT high).
    ///
    /// Pin order matches the physical wiring convention:
    ///   dat_pin  = DATA  (MISO, from controller) — GPIO4
    ///   cmd_pin  = CMD   (MOSI, to controller)   — GPIO5
    ///   clk_pin  = CLK   (SCLK, output)           — GPIO7
    ///   att_pin  = ATT   (CS,   active-low)        — GPIO6
    pub fn new<DAT, CMD, CLK, ATT>(dat_pin: DAT, cmd_pin: CMD, clk_pin: CLK, att_pin: ATT) -> Self
    where
        DAT: InputPin + 'd,
        CMD: OutputPin + 'd,
        CLK: OutputPin + 'd,
        ATT: OutputPin + 'd,
    {
        let dat = Input::new(dat_pin, InputConfig::default().with_pull(Pull::Up));
        let cmd = Output::new(cmd_pin, Level::High, OutputConfig::default());
        let clk = Output::new(clk_pin, Level::High, OutputConfig::default());
        let att = Output::new(att_pin, Level::High, OutputConfig::default());
        Self {
            clk,
            cmd,
            dat,
            att,
            was_analog: false,
        }
    }

    /// Send a configuration command sequence and return the raw 9-byte response.
    fn command(&mut self, cmd_seq: &[u8; 9]) -> [u8; 9] {
        let mut buf = [0u8; 9];

        self.att.set_low();
        delay_us(10);

        for i in 0..9 {
            buf[i] = self.transfer_byte(cmd_seq[i]);
            // Wireless receivers need ~10-20µs between bytes
            delay_byte_gap();
        }

        self.att.set_high();
        buf
    }

    /// Force the controller into analog mode and lock it there.
    ///
    /// After this call the analog LED lights up and the sticks report
    /// real values (poll byte 1 == 0x73). The mode is *locked*, so the
    /// physical MODE button and the controller's power-saving logic can't
    /// silently switch back to digital. Wireless pads still lose this on a
    /// full disconnect/sleep, so the caller should also re-run this whenever
    /// a poll comes back in digital mode (see `Ps2State::is_analog`).
    pub fn enter_analog_mode(&mut self) {
        // Step 1: enter config mode.
        //   0x01 = start, 0x43 = enter/exit config, 0x01 = enter.
        let _ = self.command(&[0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
        // Step 2: set major mode (0x44). byte[3]=0x01 => analog,
        //   byte[4]=0x03 => LOCK (0x00 would leave it user-switchable).
        let _ = self.command(&[0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00]);
        // Step 3: exit config mode (0x43 with byte[3]=0x00). The trailing
        //   0x5A bytes are the conventional filler for this command.
        let _ = self.command(&[0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A]);
    }

    /// Poll the controller and return the raw state.
    fn read_raw(&mut self) -> Ps2State {
        let raw = self.command(&[0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
        Ps2State { raw }
    }

    /// Poll the controller and return a state-change event.
    pub fn read(&mut self) -> Ps2Event {
        let state = self.read_raw();

        if state.is_analog() {
            if !self.was_analog {
                self.was_analog = true;
                return Ps2Event::RecoveredAnalog;
            }
            Ps2Event::Analog(state)
        } else {
            if self.was_analog {
                self.was_analog = false;
                return Ps2Event::LostAnalog;
            }
            Ps2Event::LostAnalog
        }
    }

    /// Transfer a single byte LSB-first.
    ///
    /// SPI Mode 3: CLK idle high. Data changes on falling edge,
    /// sampled on rising edge.
    fn transfer_byte(&mut self, mut byte_to_send: u8) -> u8 {
        let mut received: u8 = 0;

        for i in 0..8 {
            // Set CMD bit (LSB first)
            if byte_to_send & 0x01 != 0 {
                self.cmd.set_high();
            } else {
                self.cmd.set_low();
            }
            byte_to_send >>= 1;

            // Falling edge: controller updates DAT
            self.clk.set_low();
            delay_half_cycle();

            // Rising edge: sample DAT
            self.clk.set_high();
            if self.dat.is_high() {
                received |= 1 << i;
            }
            delay_half_cycle();
        }

        received
    }
}

/// Busy-wait delay for half a PS2 clock cycle (~4 µs → ~125 kHz).
///
/// Uses the Xtensa CCOUNT (cycle counter) register for deterministic timing.
/// 240 MHz × 4 µs = 960 cycles.
fn delay_half_cycle() {
    delay_cycles(960);
}

/// Delay between bytes to give the controller/receiver time to process (~15 µs).
fn delay_byte_gap() {
    delay_cycles(3600); // 240 MHz × 15 µs
}

/// Microsecond-level delay.
fn delay_us(us: u32) {
    delay_cycles(us * 240);
}

/// Block for exactly `cycles` CPU clock cycles.
#[inline]
fn delay_cycles(cycles: u32) {
    let start = xtensa_lx::timer::get_cycle_count();
    while xtensa_lx::timer::get_cycle_count().wrapping_sub(start) < cycles {
        // spin
    }
}
