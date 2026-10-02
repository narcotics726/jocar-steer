#![no_std]
// The control path (chassis, usb_session, control) lives in this crate, so the
// stack-frame budget the bins enforce has to be enforced here too.
#![deny(clippy::large_stack_frames)]

pub mod battery;
pub mod chassis;
pub mod control;
pub mod esc;
pub mod flash_log;
pub mod ps2;
pub mod rc_pwm;
pub mod steering;
pub mod tb6612;
pub mod usb_gamepad;
pub mod usb_session;
