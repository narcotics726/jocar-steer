//! ZD Controller 2.4G receiver — USB host driver (native OTG port).
//!
//! Mirror of [`crate::ps2`]: this module owns the USB host pipe binding and
//! the report decoding, returning **typed state**; the caller owns
//! enumeration orchestration, hub fallback, status LEDs and whatever it
//! wants to *do* with the state.
//!
//! The ZD receiver exposes two interfaces:
//!   - **interface 0** (Vendor Specific, EP 0x81) — the full Xinput-style
//!     64-byte report with analog triggers, 8-bit sticks and the 16-bit
//!     button map. `embassy-usb-host`'s `HidHost` only binds HID-class
//!     interfaces, so [`GamepadHost`] allocates the interrupt-IN pipe itself.
//!   - **interface 1** (HID class) — a slim 32-byte view, not used here.
//!
//! Interface 0 report (64 B):
//!   [0]=0x00 [1]=0x14      header
//!   [2] D-pad+system: ↑01 ↓02 ←04 →08 START10 BACK20 L3=40 R3=80
//!   [3] LB=01 RB=02 HOME=04 ?=08 A=10 B=20 X=40 Y=80
//!   [4]=LT [5]=RT          8-bit analog triggers
//!   [6..13]                2nd stick copy (16-bit region, xpad-only, unused)
//!   [14]=04 [15]=0a [16]=8a constant
//!   [17..19]=00             constant
//!   [20]=LX [21]=LY [22]=RX [23]=RY   8-bit sticks (~128 centre)
//!   [24..63]=00
//!
//! Button map (u16, LE: low byte = [3], high byte = [2]):
//!   LB=0x01 RB=0x02 HOME=0x04 A=0x10 B=0x20 X=0x40 Y=0x80
//!   D_U=0x100 D_D=0x200 D_L=0x400 D_R=0x800 START=0x1000 BACK=0x2000
//!   L3=0x4000 R3=0x8000

use core::marker::PhantomData;

use embassy_usb_driver::{
    host::{pipe, UsbHostAllocator, UsbPipe},
    Direction as UsbDirection, EndpointAddress, EndpointInfo, EndpointType,
};
use embassy_usb_host::{
    descriptor::ConfigurationDescriptor,
    handler::EnumerationInfo,
};

/// Vendor ID of the ZD Controller 2.4G receiver.
pub const ZD_VID: u16 = 0x2345;

/// Errors from binding/reading the gamepad's vendor interface.
#[derive(Debug, defmt::Format)]
pub enum GamepadError {
    /// Device is not a ZD receiver, or has no vendor interface 0 with an
    /// interrupt-IN endpoint. The caller should fall back to hub handling.
    NoInterface,
    /// Interrupt-IN pipe allocation failed.
    NoPipe,
    /// A transfer on the pipe failed (device unplugged, babble, …).
    Transfer,
}

/// Typed buttons of the ZD Controller.
///
/// Mirrors [`crate::ps2::Button`] so both input sources expose the same
/// `pressed()` call pattern to the caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Button {
    Lb,
    Rb,
    Home,
    A,
    B,
    X,
    Y,
    DUp,
    DDown,
    DLeft,
    DRight,
    Start,
    Back,
    L3,
    R3,
}

/// Decoded interface-0 report: the full Xinput-style frame.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct GamepadState {
    /// [20] Left stick X: 0 = left, 128 ≈ centre, 255 = right.
    pub lx: u8,
    /// [21] Left stick Y: 0 = up, 128 ≈ centre, 255 = down.
    pub ly: u8,
    /// [22] Right stick X.
    pub rx: u8,
    /// [23] Right stick Y.
    pub ry: u8,
    /// [3]|[2] 16-bit button map (LE), see [`Button`].
    pub buttons: u16,
    /// [4] LT analog trigger: 0 = released, 255 = full.
    pub accel: u8,
    /// [5] RT analog trigger.
    pub brake: u8,
}

impl GamepadState {
    /// Parse a 64-byte interface-0 report.
    ///
    /// Returns `None` on a wrong-length or mis-headed frame (e.g. the slim
    /// 32-byte HID view of interface 1).
    pub fn parse(report: &[u8]) -> Option<Self> {
        if report.len() >= 64 && report[0] == 0x00 && report[1] == 0x14 {
            Some(Self {
                lx: report[20],
                ly: report[21],
                rx: report[22],
                ry: report[23],
                buttons: u16::from_le_bytes([report[3], report[2]]),
                accel: report[4],
                brake: report[5],
            })
        } else {
            None
        }
    }

    /// Check if a button is pressed (bit set in the 16-bit map).
    pub fn pressed(&self, btn: Button) -> bool {
        let bit = match btn {
            Button::Lb => 0x0001,
            Button::Rb => 0x0002,
            Button::Home => 0x0004,
            Button::A => 0x0010,
            Button::B => 0x0020,
            Button::X => 0x0040,
            Button::Y => 0x0080,
            Button::DUp => 0x0100,
            Button::DDown => 0x0200,
            Button::DLeft => 0x0400,
            Button::DRight => 0x0800,
            Button::Start => 0x1000,
            Button::Back => 0x2000,
            Button::L3 => 0x4000,
            Button::R3 => 0x8000,
        };
        self.buttons & bit != 0
    }
}

/// Host pipe bound to the ZD receiver's vendor interface 0.
///
/// `embassy-usb-host`'s `HidHost` only binds HID-class interfaces (iface 1,
/// the slim 32-byte view), so this allocates the interrupt-IN pipe of
/// interface 0 directly — the one carrying the analog triggers and the full
/// button map.
///
/// The caller drives the read loop: call [`read`](Self::read) into a buffer
/// and feed it to [`GamepadState::parse`]. Raw access is kept so the caller
/// can also inspect the first bytes of a frame for diagnostics.
pub struct GamepadHost<'d, A: UsbHostAllocator<'d>> {
    in_ch: A::Pipe<pipe::Interrupt, pipe::In>,
    _phantom: PhantomData<&'d ()>,
}

impl<'d, A: UsbHostAllocator<'d>> GamepadHost<'d, A> {
    /// Bind the interrupt-IN pipe of the device's vendor interface 0.
    ///
    /// Only binds the ZD receiver (VID [`ZD_VID`]). Hubs and other devices
    /// can also expose vendor-class interrupt-IN endpoints (e.g. a hub's
    /// status EP) — binding those would steal the read loop.
    pub fn new(
        alloc: &A,
        config_desc: &[u8],
        enum_info: &EnumerationInfo,
    ) -> Result<Self, GamepadError> {
        if enum_info.device_desc.vendor_id != ZD_VID {
            return Err(GamepadError::NoInterface);
        }
        let cfg = ConfigurationDescriptor::try_from_slice(config_desc)
            .map_err(|_| GamepadError::NoInterface)?;
        let mut ep = None;
        for iface in cfg.iter_interface() {
            if iface.interface_number == 0 {
                for e in iface.iter_endpoints() {
                    if e.ep_type() == EndpointType::Interrupt && e.is_in() {
                        ep = Some(e);
                        break;
                    }
                }
                break;
            }
        }
        let ep = ep.ok_or(GamepadError::NoInterface)?;

        let in_ep_info = EndpointInfo {
            addr: EndpointAddress::from_parts(ep.ep_number() as usize, UsbDirection::In),
            ep_type: EndpointType::Interrupt,
            max_packet_size: ep.max_packet_size,
            interval_ms: 0,
        };
        let in_ch = alloc
            .alloc_pipe::<pipe::Interrupt, pipe::In>(
                enum_info.device_address,
                &in_ep_info,
                enum_info.split(),
            )
            .map_err(|_| GamepadError::NoPipe)?;
        Ok(Self { in_ch, _phantom: PhantomData })
    }

    /// Read one report into `buf`; returns the number of bytes received.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, GamepadError> {
        self.in_ch
            .request_in(buf)
            .await
            .map_err(|_| GamepadError::Transfer)
    }
}
