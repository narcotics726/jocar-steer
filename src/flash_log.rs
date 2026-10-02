//! Tiny persistent event log in an unused flash sector.
//!
//! Why it exists: the console is not always attached when something goes
//! wrong — the OTG port may be occupied by the receiver/adapter, or the car is
//! driving untethered. Records survive resets and power cycles, and
//! [`FlashLog::dump`] replays them, so attaching the console *later* still
//! shows what happened.
//!
//! Flash writes disable the cache and interrupts and erasing a sector takes
//! tens of milliseconds, so this is strictly for **rare** events — never a
//! general defmt sink (that would disturb USB timing).

use core::fmt::{self, Write};

use esp_storage::FlashStorage;

/// Sector reserved for the log. The partition table ends at 0x00FB_0000, so
/// this space is unallocated (per the boot loader's partition table).
pub const LOG_BASE: u32 = 0x00FB_0000;
const SECTOR_SIZE: u32 = 4096;
const REC_SIZE: usize = 64;
const HEADER: usize = 16;
const TEXT_CAP: usize = REC_SIZE - HEADER;
const REC_COUNT: u32 = SECTOR_SIZE / REC_SIZE as u32;
/// Marks a written record; erased flash reads back as 0xFFFF_FFFF.
const MAGIC: u32 = 0x474C_434A; // "JCLG"

/// Event kinds. Stable — they are persisted.
pub const EV_BOOT: u8 = 1;
pub const EV_CONNECTED: u8 = 2;
pub const EV_ENUM_FAIL: u8 = 3;
pub const EV_LOST: u8 = 4;
pub const EV_RESET: u8 = 5;
/// Enumeration succeeded (device identified).
pub const EV_ENUM_OK: u8 = 6;
/// `GamepadHost::new` result — decides gamepad path vs hub fallback.
pub const EV_IFACE: u8 = 7;
/// Hub fallback milestones (registering / registered / failed / waiting).
pub const EV_HUB: u8 = 8;
/// A gamepad read session started (direct or behind a hub).
pub const EV_SESSION: u8 = 9;
/// A HID read failed.
pub const EV_READ_ERR: u8 = 10;
/// No reports for long enough that the session was abandoned.
pub const EV_STALE: u8 = 11;
/// First report of a session that parsed (proves reports are arriving).
pub const EV_FIRST_REPORT: u8 = 12;
/// First report of a session that did *not* parse, with len + header bytes.
pub const EV_PARSE_FAIL: u8 = 13;

/// Copies `fmt` output into a fixed buffer, truncating on a char boundary.
struct TextSink<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl Write for TextSink<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for ch in s.chars() {
            let mut tmp = [0u8; 4];
            let enc = ch.encode_utf8(&mut tmp).as_bytes();
            if self.len + enc.len() > self.buf.len() {
                return Ok(());
            }
            self.buf[self.len..self.len + enc.len()].copy_from_slice(enc);
            self.len += enc.len();
        }
        Ok(())
    }
}

/// Append-only event log living in one flash sector.
pub struct FlashLog<'d> {
    storage: FlashStorage<'d>,
    /// Next free record slot.
    next: u32,
    /// Monotonic record counter, continued from the stored records.
    seq: u32,
}

impl<'d> FlashLog<'d> {
    /// Open the log, locating the first free record slot.
    pub fn new(mut storage: FlashStorage<'d>) -> Self {
        let mut word = [0u8; 4];
        let mut next = REC_COUNT;
        let mut seq = 0;
        for slot in 0..REC_COUNT {
            let at = LOG_BASE + slot * REC_SIZE as u32;
            if storage.read_nor(at, &mut word).is_err() {
                break;
            }
            if u32::from_le_bytes(word) != MAGIC {
                next = slot;
                break;
            }
            if storage.read_nor(at + 4, &mut word).is_ok() {
                seq = u32::from_le_bytes(word);
            }
        }
        Self { storage, next, seq }
    }

    /// Append one record, erasing the sector when the log wraps.
    pub fn record(&mut self, kind: u8, args: fmt::Arguments<'_>) {
        if self.next >= REC_COUNT {
            if self.storage.erase(LOG_BASE, LOG_BASE + SECTOR_SIZE).is_err() {
                return;
            }
            self.next = 0;
        }
        let mut text = [0u8; TEXT_CAP];
        let mut sink = TextSink {
            buf: &mut text,
            len: 0,
        };
        let _ = sink.write_fmt(args);
        let tlen = sink.len;

        self.seq = self.seq.wrapping_add(1);
        let mut rec = [0u8; REC_SIZE];
        rec[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        rec[4..8].copy_from_slice(&self.seq.to_le_bytes());
        rec[8..12].copy_from_slice(&self.next.to_le_bytes());
        rec[12] = kind;
        rec[HEADER..HEADER + tlen].copy_from_slice(&text[..tlen]);

        let at = LOG_BASE + self.next * REC_SIZE as u32;
        if self.storage.write_nor(at, &rec).is_ok() {
            self.next += 1;
        } else {
            // Wrong offset, flash too small, ... — surface it instead of
            // silently losing the history.
            defmt::warn!("flashlog: write failed at {}", at);
        }
    }

    /// Replay every stored record to the console.
    pub fn dump(&mut self) {
        let mut rec = [0u8; REC_SIZE];
        for slot in 0..REC_COUNT {
            let at = LOG_BASE + slot * REC_SIZE as u32;
            if self.storage.read_nor(at, &mut rec).is_err() {
                return;
            }
            if u32::from_le_bytes([rec[0], rec[1], rec[2], rec[3]]) != MAGIC {
                continue;
            }
            let seq = u32::from_le_bytes([rec[4], rec[5], rec[6], rec[7]]);
            let kind = rec[12];
            let raw = &rec[HEADER..];
            let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
            let text = core::str::from_utf8(&raw[..end]).unwrap_or("?");
            defmt::info!("flashlog seq={} kind={} {}", seq, kind, text);
        }
    }
}
