//! USB HID gamepad report mapper — capture the raw report bytes of the
//! ZD Controller receiver so the vendor layout can be reverse-engineered.
//!
//! Works in ANY receiver mode (blue/Xinput e023, white/DInput e024, ...):
//! whenever any byte of the interrupt report changes, the full raw report is
//! persisted to flash (hex) and echoed to UART. The LED shows the usual
//! status (waiting / connected / running / errors).
//!
//! Usage: flash via TTL, unplug TTL, plug the hub + receiver, run the button
//! test sequence below, then plug TTL back and read the flash log.
//!
//! ## Button test sequence (each item: press-hold ~0.5 s, release, pause
//! ~0.5 s; repeat ×5 unless noted) — for the mapping session:
//!   1.  stay idle 3 s                      (baseline)
//!   2.  A ×5
//!   3.  B ×5
//!   4.  X ×5
//!   5.  Y ×5
//!   6.  LB ×5
//!   7.  RB ×5
//!   8.  L3 (press left stick) ×5
//!   9.  R3 (press right stick) ×5
//!  10.  BACK / Select ×5
//!  11.  START ×5
//!  12.  HOME / Guide ×5
//!  13.  D-pad: Up ×5, Down ×5, Left ×5, Right ×5
//!  14.  Left stick: hold full Left ×3, Right ×3, Up ×3, Down ×3 (1 s each)
//!  15.  Right stick: same as 14
//!  16.  LT: half-press ×3, full-press ×3 (hold 1 s)
//!  17.  RT: half-press ×3, full-press ×3 (hold 1 s)
//!  18.  stay idle 3 s                      (closing baseline)
//!
//! Over/under-pressing is fine: every press is a state change and gets
//! captured; the analysis only looks at which byte changed.

#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    fmt::Write as _,
    sync::atomic::{AtomicU8, Ordering},
};

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb_host::{
    BusRoute, BusState, class::hid::HidHost, class::hub::{HubEvent, HubHandler}, handler::HandlerEvent,
};
use esp_bootloader_esp_idf::partitions::{self, DataPartitionSubType, PartitionType};
use esp_hal::{
    clock::CpuClock,
    gpio::Level,
    interrupt::software::SoftwareInterruptControl,
    peripherals::GPIO48,
    rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator},
    time::Rate,
    timer::timg::TimerGroup,
    usb::otg::{Usb, embassy_usb_host::Driver},
};
use esp_println as _;
use esp_storage::FlashStorage;
use jocar_steer::lighting::ws2812_stat_indicator::Rgb;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// ── Flash log (plain text, NVS partition area) ──────────────────────

const LOG_MAGIC: [u8; 4] = *b"JCLG";
const LOG_START: u32 = 4; // records start after the magic
const LOG_MAX: u32 = 12000; // usable record area (12 KB of the NVS partition)
const LOG_SECTOR: u32 = 12288; // erase granularity (3 × 4 KB sectors)

/// Minimal `fmt::Write` over a byte slice (no alloc).
struct Buf<'a>(&'a mut [u8], usize);

impl core::fmt::Write for Buf<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len();
        if self.1 + n > self.0.len() {
            return Err(core::fmt::Error);
        }
        self.0[self.1..self.1 + n].copy_from_slice(s.as_bytes());
        self.1 += n;
        Ok(())
    }
}

/// Fresh `FlashStorage` over the whole flash (unstable `steal`).
fn flash_storage() -> FlashStorage<'static> {
    let p = unsafe { esp_hal::peripherals::Peripherals::steal() };
    FlashStorage::new(p.FLASH)
}

// One FlashStorage instance for the program lifetime: creating a new one
// each call re-sends an RDID command via SPI1, which breaks subsequent
// ROM flash reads on this setup.
static FLASH_CELL: static_cell::StaticCell<FlashStorage<'static>> = static_cell::StaticCell::new();
static FLASH_MUTEX: critical_section::Mutex<RefCell<Option<&'static mut FlashStorage<'static>>>> =
    critical_section::Mutex::new(RefCell::new(None));

/// Run `f` with a view of the NVS partition (our log lives in its first sector).
fn with_log_region<R>(
    f: impl FnOnce(&mut partitions::FlashRegion<'_, '_>) -> Result<R, ()>,
) -> Result<R, ()> {
    critical_section::with(|cs| {
        let cell = FLASH_MUTEX.borrow(cs);
        let mut slot = cell.try_borrow_mut().map_err(|_| ())?;
        if slot.is_none() {
            *slot = Some(FLASH_CELL.init(flash_storage()));
        }
        let flash: &mut FlashStorage<'static> = slot.as_mut().unwrap();

        let mut pt_buf = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(flash, &mut pt_buf).map_err(|_| ())?;
        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .map_err(|_| ())?
            .ok_or(())?;
        let mut region = nvs.as_flash_region(flash);
        f(&mut region)
    })
}

/// Append one record `[len u8][text]` to the log.
fn flash_log_append(text: &[u8]) -> Result<(), ()> {
    let len = text.len();
    if len == 0 || len > 255 {
        return Err(());
    }

    with_log_region(|region| {
        let mut hdr = [0u8; 4];
        region.read(0, &mut hdr).map_err(|_| ())?;
        if hdr != LOG_MAGIC {
            region.erase(0, LOG_SECTOR).map_err(|_| ())?;
            region.write(0, &LOG_MAGIC).map_err(|_| ())?;
        }

        // Walk to the end of valid records (survives a partial tail write).
        let mut p = LOG_START;
        loop {
            if p >= LOG_MAX {
                break;
            }
            let mut b = [0u8; 1];
            region.read(p, &mut b).map_err(|_| ())?;
            if b[0] == 0xFF || b[0] == 0 || p + 1 + b[0] as u32 > LOG_MAX {
                break;
            }
            p += 1 + b[0] as u32;
        }

        if p + 1 + len as u32 > LOG_MAX {
            // Full: restart the log.
            region.erase(0, LOG_SECTOR).map_err(|_| ())?;
            region.write(0, &LOG_MAGIC).map_err(|_| ())?;
            p = LOG_START;
        }

        let mut rec = [0u8; 256];
        rec[0] = len as u8;
        rec[1..1 + len].copy_from_slice(text);
        region.write(p, &rec[..1 + len]).map_err(|_| ())
    })
}

/// Print all records to UART, then erase the log area.
fn flash_log_dump() {
    esp_println::println!("[flog] ── dump start ──");
    with_log_region(|region| {
        let mut buf = [0u8; LOG_MAX as usize];
        region.read(0, &mut buf).map_err(|_| ())?;
        if buf[0..4] != LOG_MAGIC {
            esp_println::println!("[flog] no log found (magic absent)");
            return Ok(());
        }
        let mut p = LOG_START as usize;
        let mut n = 0u32;
        loop {
            if p >= LOG_MAX as usize {
                break;
            }
            let len = buf[p];
            if len == 0xFF || len == 0 || p + 1 + len as usize > LOG_MAX as usize {
                break;
            }
            let s = core::str::from_utf8(&buf[p + 1..p + 1 + len as usize]).unwrap_or("<bad utf8>");
            esp_println::println!("  [{}] {}", n, s);
            n += 1;
            p += 1 + len as usize;
        }
        esp_println::println!("[flog] {} records — retained (not cleared)", n);
        Ok(())
    })
    .ok();
    esp_println::println!("[flog] ── dump end ──");
}

/// Echo to UART + persist to flash.
fn flog_raw(text: &[u8]) {
    let s = core::str::from_utf8(text).unwrap_or("<bad utf8>");
    esp_println::println!("[flog] {}", s);
    if flash_log_append(text).is_err() {
        esp_println::println!("[flog] ⚠ flash append failed");
    }
}

/// Log a formatted message.
fn flog(args: core::fmt::Arguments) {
    let mut text = [0u8; 220];
    let len = {
        let mut w = Buf(&mut text, 0);
        let _ = core::fmt::write(&mut w, args);
        w.1
    };
    flog_raw(&text[..len]);
}

macro_rules! flog {
    ($($t:tt)*) => {
        $crate::flog(core::format_args!($($t)*))
    };
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Best-effort persist to flash so an untethered panic is not lost.
    flog!("PANIC: {}", info);
    defmt::error!("Panic: {}", defmt::Display2Format(info));
    loop {}
}

// ── LED status ──────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq)]
enum Status {
    Boot = 0,
    Waiting = 1,
    Connected = 2,
    Running = 3,
    ErrEnum = 4,
    ErrHid = 5,
}

impl Status {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Waiting,
            2 => Self::Connected,
            3 => Self::Running,
            4 => Self::ErrEnum,
            5 => Self::ErrHid,
            _ => Self::Boot,
        }
    }
}

static STATUS: AtomicU8 = AtomicU8::new(Status::Boot as u8);

fn set_status(s: Status) {
    STATUS.store(s as u8, Ordering::Relaxed);
}

const CODE_0: PulseCode = PulseCode::new(Level::High, 32, Level::Low, 68);
const CODE_1: PulseCode = PulseCode::new(Level::High, 64, Level::Low, 36);
const CODE_RESET: PulseCode = PulseCode::new(Level::Low, 4800, Level::Low, 0);

const BITS_PER_LED: usize = 24;
const MAX_LEDS: usize = 1;

type WsBuf = [PulseCode; BITS_PER_LED * MAX_LEDS + 1];

fn encode_led(buf: &mut WsBuf, offset: usize, rgb: Rgb) {
    let bits = ((rgb.g as u32) << 16) | ((rgb.r as u32) << 8) | (rgb.b as u32);
    for i in 0..BITS_PER_LED {
        let code = if (bits & (1 << (23 - i))) != 0 {
            CODE_1
        } else {
            CODE_0
        };
        buf[offset + i] = code;
    }
}

fn encode_frame(buf: &mut WsBuf, colors: &[Rgb], count: usize) {
    let count = count.min(MAX_LEDS);
    for i in 0..count {
        encode_led(buf, i * BITS_PER_LED, colors[i]);
    }
    for i in count..MAX_LEDS {
        encode_led(buf, i * BITS_PER_LED, Rgb::OFF);
    }
    buf[MAX_LEDS * BITS_PER_LED] = CODE_RESET;
}

const BRIGHT: u8 = 12;

fn rgb(g: u8, r: u8, b: u8) -> Rgb {
    Rgb { g, r, b }
}

fn led_green() -> Rgb {
    rgb(BRIGHT, 0, 0)
}
fn led_red() -> Rgb {
    rgb(0, BRIGHT, 0)
}
fn led_yellow() -> Rgb {
    rgb(BRIGHT, BRIGHT, 0)
}
fn led_white() -> Rgb {
    rgb(BRIGHT, BRIGHT, BRIGHT)
}

#[embassy_executor::task]
async fn led_task(rmt: Rmt<'static, esp_hal::Blocking>, led_pin: GPIO48<'static>) {
    let tx_config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_idle_output(true);
    let mut channel = rmt
        .channel0
        .configure_tx(&tx_config)
        .expect("TX config")
        .with_pin(led_pin);
    let mut ws_buf: WsBuf = [PulseCode::default(); BITS_PER_LED * MAX_LEDS + 1];

    loop {
        let status = Status::from_u8(STATUS.load(Ordering::Relaxed));
        let now = Instant::now().as_millis();
        let color = match status {
            Status::Boot => led_red(),
            Status::Waiting => {
                if now % 1000 < 500 {
                    led_yellow()
                } else {
                    Rgb::OFF
                }
            }
            Status::Connected => led_white(),
            Status::Running => {
                if now % 1000 < 500 {
                    led_green()
                } else {
                    Rgb::OFF
                }
            }
            Status::ErrEnum => led_red(),
            Status::ErrHid => {
                if now % 400 < 200 {
                    led_red()
                } else {
                    Rgb::OFF
                }
            }
        };

        encode_frame(&mut ws_buf, &[color], 1);
        match channel.transmit(&ws_buf) {
            Ok(tx) => channel = tx.wait().expect("LED TX wait"),
            Err((e, ch)) => {
                warn!("LED TX error: {:?}", e);
                channel = ch;
            }
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}

// ── Entry point ─────────────────────────────────────────────────────

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Dump any log left by a previous untethered run, then clear it.
    flash_log_dump();

    // Strapping
    let _ = peripherals.GPIO0;
    let _ = peripherals.GPIO3;
    let _ = peripherals.GPIO45;
    let _ = peripherals.GPIO46;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("usb-gamepad-map-test: starting");
    flog!("map boot");

    // ── WS2812 LED task (GPIO48) ──
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("RMT init");
    spawner.spawn(led_task(rmt, peripherals.GPIO48).expect("spawn led_task"));

    Timer::after(Duration::from_millis(500)).await;
    set_status(Status::Waiting);
    info!("USB host ready — plug the receiver, then run the button sequence");
    flog!("map host ready");

    // ── USB OTG host on GPIO19 (D-) / GPIO20 (D+) ──
    let usb = Usb::new_fs(peripherals.USB_FS, peripherals.GPIO20, peripherals.GPIO19);
    static BUS_STATE: BusState = BusState::new();
    let (mut bus_ctrl, bus) = embassy_usb_host::bus(Driver::new(usb), &BUS_STATE);

    loop {
        let speed = bus_ctrl.wait_for_connection().await;
        set_status(Status::Connected);
        flog!("connected speed={:?}", speed);

        let mut config_buf = [0u8; 256];
        let enum_result = embassy_time::with_timeout(
            Duration::from_secs(5),
            bus.enumerate(BusRoute::Direct(speed), &mut config_buf),
        )
        .await;
        let enum_result = match enum_result {
            Ok(r) => r,
            Err(_) => {
                flog!("enum timeout 5s");
                set_status(Status::ErrEnum);
                Timer::after(Duration::from_millis(1000)).await;
                set_status(Status::Waiting);
                Timer::after(Duration::from_millis(2000)).await;
                continue;
            }
        };
        match enum_result {
            Ok((enum_info, config_len)) => {
                flog!(
                    "enumerated vid={:04x} pid={:04x} cfg_len={}",
                    enum_info.device_desc.vendor_id,
                    enum_info.device_desc.product_id,
                    config_len
                );

                // Direct HID device?
                match HidHost::new(&bus, &config_buf[..config_len], &enum_info) {
                    Ok(h) => {
                        let mut hid = h;

                        // Fetch the HID report descriptor (its size hints at the layout).
                        let mut desc_buf = [0u8; 256];
                        match hid.fetch_report_descriptor(&mut desc_buf).await {
                            Ok(desc) => flog!("report desc {} bytes", desc.len()),
                            Err(e) => warn!("fetch_report_descriptor failed: {:?}", e),
                        }
                        match hid.set_idle(0, 0).await {
                            Ok(()) => {}
                            Err(e) => warn!("SET_IDLE failed (continuing): {:?}", e),
                        }

                        set_status(Status::Running);
                        flog!("hid ready (direct) — run the button sequence");

                        run_capture_session(&mut hid, 0).await;
                        flog!("capture session ended");
                    }
                    Err(_) => {
                        // Not a direct HID device — try hub.
                        flog!("no direct HID — trying hub");
                        match HubHandler::<_, 8>::try_register(&bus, &enum_info).await {
                            Ok(mut hub) => {
                                set_status(Status::Running);
                                flog!("hub registered");

                                loop {
                                    match hub.wait_for_event().await {
                                        Ok(HandlerEvent::HandlerEvent(HubEvent::DeviceDetected {
                                            port,
                                            speed,
                                        })) => {
                                            flog!("hub port {} connected speed={:?}", port, speed);
                                            let mut cfg = [0u8; 256];
                                            let r = embassy_time::with_timeout(
                                                Duration::from_secs(5),
                                                hub.enumerate_port(&mut cfg, port, speed),
                                            )
                                            .await;
                                            match r {
                                                Ok(Ok((ei, len))) => {
                                                    flog!(
                                                        "hub port {} enumerated vid={:04x} pid={:04x}",
                                                        port,
                                                        ei.device_desc.vendor_id,
                                                        ei.device_desc.product_id
                                                    );
                                                    match HidHost::new(&bus, &cfg[..len], &ei) {
                                                        Ok(h) => {
                                                            let mut hid = h;
                                                            let mut desc_buf = [0u8; 256];
                                                            match hid
                                                                .fetch_report_descriptor(&mut desc_buf)
                                                                .await
                                                            {
                                                                Ok(desc) => flog!(
                                                                    "hub port {} report desc {} bytes",
                                                                    port,
                                                                    desc.len()
                                                                ),
                                                                Err(e) => warn!(
                                                                    "fetch_report_descriptor failed: {:?}",
                                                                    e
                                                                ),
                                                            }
                                                            match hid.set_idle(0, 0).await {
                                                                Ok(()) => {}
                                                                Err(e) => warn!(
                                                                    "SET_IDLE failed (continuing): {:?}",
                                                                    e
                                                                ),
                                                            }
                                                            set_status(Status::Running);
                                                            flog!(
                                                                "hid ready on hub port {} — run the button sequence",
                                                                port
                                                            );
                                                            run_capture_session(&mut hid, port)
                                                                .await;
                                                            flog!("capture session ended");
                                                        }
                                                        Err(e) => {
                                                            flog!("hub port {} hid error {:?}", port, e);
                                                        }
                                                    }
                                                }
                                                Ok(Err(e)) => {
                                                    flog!("hub port {} enum error {:?}", port, e);
                                                }
                                                Err(_) => {
                                                    flog!("hub port {} enum timeout", port);
                                                }
                                            }
                                        }
                                        Ok(HandlerEvent::HandlerEvent(HubEvent::DeviceRemoved {
                                            port,
                                            ..
                                        })) => {
                                            flog!("hub port {} device removed", port);
                                        }
                                        Ok(HandlerEvent::HandlerDisconnected) => {
                                            flog!("hub disconnected");
                                            break;
                                        }
                                        Ok(HandlerEvent::NoChange) => {}
                                        Err(e) => {
                                            flog!("hub event error {:?}", e);
                                            break;
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                flog!("hub register error {:?}", e);
                                set_status(Status::ErrEnum);
                                Timer::after(Duration::from_millis(1000)).await;
                            }
                        }
                    }
                }
            }
            Err(e) => {
                flog!("enum error {:?}", e);
                set_status(Status::ErrEnum);
                Timer::after(Duration::from_millis(1000)).await;
            }
        }
        set_status(Status::Waiting);
        Timer::after(Duration::from_millis(2000)).await;
    }
}

/// Read reports and persist every change of the raw report bytes to flash.
/// `port` is 0 for a direct device, otherwise the hub port number.
async fn run_capture_session<A: embassy_usb_driver::host::UsbHostAllocator<'static>>(
    hid: &mut HidHost<'static, A>,
    port: u8,
) {
    let mut buf = [0u8; 64];
    let mut last = [0u8; 64];
    let mut have_last = false;
    let mut last_cap = Instant::now();

    loop {
        match hid.read(&mut buf).await {
            Ok(n) if n > 0 => {
                let changed = !have_last || buf[..n] != last[..n];
                if changed && last_cap.elapsed() >= Duration::from_millis(50) {
                    flog_hex_tag(port, &buf[..n]);
                    last_cap = Instant::now();
                }
                last[..n].copy_from_slice(&buf[..n]);
                have_last = true;
            }
            Ok(_) => {}
            Err(e) => {
                flog!("capture read error {:?}", e);
                break;
            }
        }
    }
}

/// Persist raw report bytes with a port tag prefix (`r<port> <hex>`).
fn flog_hex_tag(port: u8, bytes: &[u8]) {
    let mut text = [0u8; 240];
    let len = {
        let mut w = Buf(&mut text, 0);
        let _ = w.write_str("r");
        if port != 0 {
            let _ = core::fmt::write(&mut w, core::format_args!("{}", port));
        }
        let _ = w.write_str(" ");
        for b in bytes {
            let _ = core::fmt::write(&mut w, core::format_args!("{:02x}", b));
        }
        w.1
    };
    flog_raw(&text[..len]);
}
