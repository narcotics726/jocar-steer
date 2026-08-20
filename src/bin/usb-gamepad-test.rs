//! USB gamepad test — read the ZD Controller (2.4G receiver) over the native
//! USB OTG port (GPIO19/20) and drive the steering servo from the analog
//! triggers. Driver + report decoding live in [`jocar_steer::usb_gamepad`]
//! (mirror of `ps2.rs`); this bin keeps the USB session orchestration
//! (enumerate → bind → hub fallback), the status LED and the flash log.
//!
//! Steering: LT → left, RT → right; trigger depth sets the magnitude
//! (0-255 → 0..±90°). Throttle stays on the LY stick (integrated in main.rs).
//!
//! Onboard WS2812 LED (GPIO48) feedback:
//!   status   Boot → red solid ~0.5 s (self-test), Waiting → yellow 1 Hz blink,
//!            Connected → white solid, Running → green 1 Hz heartbeat,
//!            ErrEnum → red solid (enumerate failed), ErrHid → fast red blink
//!   input    (overrides status while held) joystick → blue, A/B/X/Y → yellow,
//!            LB/RB/others → magenta, D-pad → white, triggers → red
//!
//! Flash log: key events are appended in plain text to the first 2 KB of the
//! NVS partition (unused by this firmware), surviving unplug/power loss.
//! On every boot the log is dumped to UART and cleared — plug the TTL cable
//! back in and reset to read what happened during an untethered test.

#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU8, Ordering},
};

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb_host::{
    BusRoute, BusState, class::hub::{HubEvent, HubHandler}, handler::HandlerEvent,
};
use esp_bootloader_esp_idf::partitions::{self, DataPartitionSubType, PartitionType};
use esp_hal::{
    clock::CpuClock,
    gpio::{DriveMode, Level},
    interrupt::software::SoftwareInterruptControl,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelHW, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::GPIO48,
    rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator},
    time::Rate,
    timer::timg::TimerGroup,
    usb::otg::{Usb, embassy_usb_host::Driver},
};
use esp_println as _;
use esp_storage::FlashStorage;
use jocar_steer::lighting::ws2812_stat_indicator::Rgb;
use jocar_steer::usb_gamepad::{GamepadHost, GamepadState};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// ── Flash log (plain text, NVS partition area) ──────────────────────

const LOG_MAGIC: [u8; 4] = *b"JCLG";
const LOG_START: u32 = 4; // records start after the magic
const LOG_MAX: u32 = 2048; // usable record area within the sector
const LOG_SECTOR: u32 = 4096; // erase granularity

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
        let pt = match partitions::read_partition_table(flash, &mut pt_buf) {
            Ok(pt) => pt,
            Err(e) => {
                esp_println::println!("[flog] read_partition_table err: {:?}", e);
                return Err(());
            }
        };
        let nvs = match pt.find_partition(PartitionType::Data(DataPartitionSubType::Nvs)) {
            Ok(Some(nvs)) => nvs,
            Ok(None) => {
                esp_println::println!("[flog] NVS partition not found");
                return Err(());
            }
            Err(e) => {
                esp_println::println!("[flog] find_partition err: {:?}", e);
                return Err(());
            }
        };
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
        region.read(0, &mut hdr).map_err(|e| {
            esp_println::println!("[flog] read hdr err: {:?}", e);
        })?;
        if hdr != LOG_MAGIC {
            region.erase(0, LOG_SECTOR).map_err(|e| {
                esp_println::println!("[flog] erase err: {:?}", e);
            })?;
            region.write(0, &LOG_MAGIC).map_err(|e| {
                esp_println::println!("[flog] write magic err: {:?}", e);
            })?;
        }

        // Walk to the end of valid records (survives a partial tail write).
        let mut p = LOG_START;
        loop {
            if p >= LOG_MAX {
                break;
            }
            let mut b = [0u8; 1];
            region.read(p, &mut b).map_err(|e| {
                esp_println::println!("[flog] read pos err: {:?}", e);
            })?;
            if b[0] == 0xFF || b[0] == 0 || p + 1 + b[0] as u32 > LOG_MAX {
                break;
            }
            p += 1 + b[0] as u32;
        }

        if p + 1 + len as u32 > LOG_MAX {
            // Full: restart the log.
            region.erase(0, LOG_SECTOR).map_err(|e| {
                esp_println::println!("[flog] erase-full err: {:?}", e);
            })?;
            region.write(0, &LOG_MAGIC).map_err(|e| {
                esp_println::println!("[flog] write magic-full err: {:?}", e);
            })?;
            p = LOG_START;
        }

        let mut rec = [0u8; 256];
        rec[0] = len as u8;
        rec[1..1 + len].copy_from_slice(text);
        region.write(p, &rec[..1 + len]).map_err(|e| {
            esp_println::println!("[flog] write rec err: {:?}", e);
        })
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
            let s =
                core::str::from_utf8(&buf[p + 1..p + 1 + len as usize]).unwrap_or("<bad utf8>");
            esp_println::println!("  [{}] {}", n, s);
            n += 1;
            p += 1 + len as usize;
        }
        esp_println::println!("[flog] {} records — clearing", n);
        region.erase(0, LOG_SECTOR).map_err(|_| ())
    })
    .ok();
    esp_println::println!("[flog] ── dump end ──");
}

/// Log a formatted message: echo to UART + persist to flash.
fn flog(args: core::fmt::Arguments) {
    let mut text = [0u8; 220];
    let len = {
        let mut w = Buf(&mut text, 0);
        let _ = core::fmt::write(&mut w, args);
        w.1
    };
    let text = &text[..len];
    let s = core::str::from_utf8(text).unwrap_or("<bad utf8>");
    esp_println::println!("[flog] {}", s);
    if flash_log_append(text).is_err() {
        esp_println::println!("[flog] ⚠ flash append failed");
    }
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
    // The LED freezes on its last colour — a visible "hung" marker.
    defmt::error!("Panic: {}", defmt::Display2Format(info));
    loop {}
}

// ── Shared state between `main` and the LED task ────────────────────

#[derive(Clone, Copy, PartialEq)]
enum Status {
    Boot = 0,
    Waiting = 1,
    Connected = 2,
    Running = 3,
    /// Device detected but enumeration failed (red solid).
    ErrEnum = 4,
    /// Enumerated OK but HID init failed (fast red blink).
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

/// Input class shared with the LED task; `Idle` means "show status pattern".
#[derive(Clone, Copy, PartialEq, defmt::Format)]
enum InputClass {
    Idle = 0,
    Joystick = 1,
    Dpad = 2,
    Xyab = 3,
    Bumpers = 4,
    Trigger = 5,
}

impl InputClass {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Joystick,
            2 => Self::Dpad,
            3 => Self::Xyab,
            4 => Self::Bumpers,
            5 => Self::Trigger,
            _ => Self::Idle,
        }
    }
}

static INPUT: AtomicU8 = AtomicU8::new(InputClass::Idle as u8);

fn set_input(c: InputClass) {
    INPUT.store(c as u8, Ordering::Relaxed);
}

// ── WS2812 LED (GPIO48, RMT ch0 @ 80 MHz) ──────────────────────────

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

/// WS2812 brightness (0-255).
const BRIGHT: u8 = 12;

/// Build an `Rgb` (GRB byte order) from green/red/blue.
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
fn led_blue() -> Rgb {
    rgb(0, 0, BRIGHT)
}
fn led_magenta() -> Rgb {
    rgb(BRIGHT, 0, BRIGHT)
}

fn class_color(c: InputClass) -> Rgb {
    match c {
        InputClass::Idle => Rgb::OFF,
        InputClass::Joystick => led_blue(),
        InputClass::Xyab => led_yellow(),
        InputClass::Bumpers => led_magenta(),
        InputClass::Dpad => led_white(),
        InputClass::Trigger => led_red(),
    }
}

/// Background task: drives the WS2812 from the shared [`Status`] /
/// [`InputClass`] state. Samples at 10 Hz; blink phases anchored to
/// `Instant::now()` so the sampling rate does not distort them.
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

    let mut last_input = InputClass::Idle;
    let mut input_since = Instant::now();

    loop {
        let status = Status::from_u8(STATUS.load(Ordering::Relaxed));
        let input = InputClass::from_u8(INPUT.load(Ordering::Relaxed));
        if input != last_input {
            last_input = input;
            input_since = Instant::now();
        }

        let now = Instant::now().as_millis();
        let color = if input != InputClass::Idle {
            // Input class wins over the status pattern.
            let target = class_color(input);
            let since = input_since.elapsed();
            if since < Duration::from_millis(150) {
                // Flash on change: 50 ms on / 50 ms off / 50 ms on.
                if since.as_millis() % 100 < 50 {
                    target
                } else {
                    Rgb::OFF
                }
            } else {
                target
            }
        } else {
            match status {
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
                    // Fast red blink: ~200 ms on / 200 ms off.
                    if now % 400 < 200 {
                        led_red()
                    } else {
                        Rgb::OFF
                    }
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

// ── Driver: jocar_steer::usb_gamepad ─────────────────────────────────
// GamepadHost (vendor iface 0 pipe) + GamepadState (report decode) live in
// the library module, mirroring ps2.rs. This bin keeps only the USB session
// orchestration and the diagnostics below.

/// Stick deadzone around the 127 centre.
const DEADZONE: i16 = 20;
/// Trigger threshold.
const TRIGGER_ON: u8 = 16;

// ── MG90S servo on GPIO13 (50 Hz PWM, 12-bit LEDC duty) ─────────────
// Same timing as servo-test.rs: 1000..=2000 µs pulse = -90°..+90°.

const PERIOD_US: u32 = 20_000; // 50 Hz
const DUTY_MAX: u32 = 1 << 12; // 12-bit → 4096 counts/period
const CENTER_US: i32 = 1500; // 0° centre
const US_PER_90DEG: i32 = 500; // 500 µs per 90°

fn angle_to_counts(deg: i32) -> u32 {
    let deg = deg.clamp(-90, 90);
    let pulse_us = (CENTER_US + deg * US_PER_90DEG / 90) as u32;
    (DUTY_MAX * pulse_us) / PERIOD_US
}

/// LT → left, RT → right; trigger depth sets steering magnitude.
fn triggers_to_angle(lt: u8, rt: u8, max_deg: i32) -> i32 {
    let l = (lt as i32) * max_deg / 255;
    let r = (rt as i32) * max_deg / 255;
    (r - l).clamp(-max_deg, max_deg)
}

fn classify(gp: &GamepadState) -> InputClass {
    let stick_active = (gp.lx as i16 - 127).abs() > DEADZONE
        || (gp.ly as i16 - 127).abs() > DEADZONE
        || (gp.rx as i16 - 127).abs() > DEADZONE
        || (gp.ry as i16 - 127).abs() > DEADZONE;

    // Priority: triggers > ABXY > other buttons > D-pad > sticks.
    if gp.accel > TRIGGER_ON || gp.brake > TRIGGER_ON {
        InputClass::Trigger
    } else if gp.buttons & 0x000F != 0 {
        InputClass::Xyab
    } else if gp.buttons & 0xFFF0 != 0 {
        InputClass::Bumpers
    } else if gp.buttons & 0xF000 != 0 {
        // D-pad: bitmap bits 0x1000..0x8000 (e024 pads send a hat instead,
        // but iface 0 never reports one — hat is always 8 here).
        InputClass::Dpad
    } else if stick_active {
        InputClass::Joystick
    } else {
        InputClass::Idle
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

    info!("usb-gamepad-test: starting");
    flog!("boot");

    // ── WS2812 LED task (GPIO48) — spawned before anything else so the
    // boot state is always visible on the LED. ──
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("RMT init");
    spawner.spawn(led_task(rmt, peripherals.GPIO48).expect("spawn led_task"));

    // Boot self-test: red for 0.5 s, then waiting.
    Timer::after(Duration::from_millis(500)).await;
    set_status(Status::Waiting);
    info!("USB host ready — plug the receiver (blue-LED/e023 or white-LED/e024)");
    flog!("host ready, waiting for device");

    // ── MG90S servo on GPIO13 (LEDC Timer0 Ch0 @ 50 Hz) ──
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();
    let mut servo_ch = ledc.channel(channel::Number::Channel0, peripherals.GPIO13);
    servo_ch
        .configure(channel::config::Config {
            timer: &lstimer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();
    servo_ch.set_duty_hw(angle_to_counts(0));
    info!("Servo ready: LEDC Timer0 Ch0 → GPIO13 (MG90S), left-stick X → angle");

    // ── USB OTG host on GPIO19 (D-) / GPIO20 (D+) ──
    let usb = Usb::new_fs(peripherals.USB_FS, peripherals.GPIO20, peripherals.GPIO19);
    static BUS_STATE: BusState = BusState::new();
    let (mut bus_ctrl, bus) = embassy_usb_host::bus(Driver::new(usb), &BUS_STATE);

    loop {
        set_input(InputClass::Idle);
        let speed = bus_ctrl.wait_for_connection().await;
        set_status(Status::Connected);
        info!("Device connected at {:?}", speed);
        flog!("connected speed={:?}", speed);

        let mut config_buf = [0u8; 256];
        // Bound the enumeration so a non-responding device cannot hang the
        // loop forever (seen as a stuck white LED on a flaky power rail).
        let enum_result = embassy_time::with_timeout(
            Duration::from_secs(5),
            bus.enumerate(BusRoute::Direct(speed), &mut config_buf),
        )
        .await;
        let enum_result = match enum_result {
            Ok(r) => r,
            Err(_) => {
                error!("Enumeration timed out after 5s");
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
                info!(
                    "Enumerated: VID={:04x} PID={:04x}",
                    enum_info.device_desc.vendor_id,
                    enum_info.device_desc.product_id
                );
                flog!(
                    "enumerated vid={:04x} pid={:04x} cfg_len={}",
                    enum_info.device_desc.vendor_id,
                    enum_info.device_desc.product_id,
                    config_len
                );

                // Vendor interface 0: full Xinput-style report (analog triggers).
                match GamepadHost::new(&bus, &config_buf[..config_len], &enum_info) {
                    Ok(h) => {
                        let mut hid = h;

                        set_status(Status::Running);
                        info!("iface0 ready — reading gamepad reports");
                        flog!("iface0 ready");

                        let mut buf = [0u8; 64];
                        let mut last_log = Instant::now();
                        let mut first_report = true;
                        let mut last_btns = 0u16;
                        let _last_btn_log = Instant::now();
                        let mut last_servo_angle = 0i32;
                        let mut last_report = Instant::now();

                        loop {
                            match embassy_time::with_timeout(
                                Duration::from_millis(2000),
                                hid.read(&mut buf),
                            )
                            .await
                            {
                                Ok(Ok(n)) if n > 0 => {
                                    last_report = Instant::now();
                                    if let Some(gp) = GamepadState::parse(&buf[..n]) {
                                        let class = classify(&gp);
                                        set_input(class);

                                        // Servo: LT → left, RT → right, depth = magnitude.
                                        let angle = triggers_to_angle(gp.accel, gp.brake, 90);
                                        if angle != last_servo_angle {
                                            servo_ch.set_duty_hw(angle_to_counts(angle));
                                            last_servo_angle = angle;
                                        }

                                        if first_report {
                                            first_report = false;
                                            flog!(
                                                "first report n={} bytes={:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                                                n, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                                                buf[6], buf[7], buf[8]
                                            );
                                        }

                                        // Persist button state changes to flash so
                                        // unknown buttons can be identified later.
                                        if gp.buttons != last_btns {
                                            flog!(
                                                "btns={:04x} lx={} ly={} rx={} ry={} accel={} brake={}",
                                                gp.buttons, gp.lx, gp.ly, gp.rx, gp.ry,
                                                gp.accel, gp.brake
                                            );
                                            last_btns = gp.buttons;
                                        }

                                        // Console log (throttled to 10 Hz).
                                        if last_log.elapsed() >= Duration::from_millis(100) {
                                            info!(
                                                "lx={} ly={} rx={} ry={} btns={:04x} accel={} brake={} class={:?}",
                                                gp.lx, gp.ly, gp.rx, gp.ry, gp.buttons,
                                                gp.accel, gp.brake, class
                                            );
                                            last_log = Instant::now();
                                        }
                                    } else {
                                        flog!(
                                            "bad report n={} head={:02x} {:02x} {:02x} {:02x}",
                                            n, buf[0], buf[1], buf[2], buf[3]
                                        );
                                    }
                                }
                                Ok(Ok(_)) => {}
                                Ok(Err(e)) => {
                                    error!("HID read failed: {:?}", e);
                                    flog!("read error {:?}", e);
                                    break;
                                }
                                Err(_) => {
                                    if last_report.elapsed() >= Duration::from_secs(5) {
                                        error!("no reports for 5s — device gone?");
                                        flog!("no reports for 5s");
                                        break;
                                    }
                                }
                            }
                        }
                        info!("Device disconnected, waiting for next");
                        flog!("device gone");
                    }
                    Err(_) => {
                        // Not a direct HID device — try registering it as a hub and
                        // service downstream ports.
                        flog!("no direct HID — trying hub");
                        match HubHandler::<_, 8>::try_register(&bus, &enum_info).await {
                            Ok(mut hub) => {
                                set_status(Status::Running);
                                flog!("hub registered");

                                // Hub event loop: waits for port events, enumerates
                                // downstream devices and reads HID reports from them.
                                loop {
                                    match hub.wait_for_event().await {
                                        Ok(HandlerEvent::HandlerEvent(HubEvent::DeviceDetected {
                                            port,
                                            speed,
                                        })) => {
                                            flog!(
                                                "hub port {} connected speed={:?}",
                                                port, speed
                                            );
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
                                                    match GamepadHost::new(&bus, &cfg[..len], &ei) {
                                                        Ok(h) => {
                                                            let mut hid = h;

                                                            set_status(Status::Running);
                                                            flog!("iface0 ready on hub port {}", port);

                                                            let mut buf = [0u8; 64];
                                                            let mut last_log = Instant::now();
                                                            let mut first_report = true;
                                                            let mut last_btns = 0u16;
                                                            let mut last_btn_log = Instant::now();
                                                            let mut last_servo_angle = 0i32;
                                                            let mut last_report = Instant::now();

                                                            loop {
                                                                match embassy_time::with_timeout(
                                                                    Duration::from_millis(2000),
                                                                    hid.read(&mut buf),
                                                                )
                                                                .await
                                                                {
                                                                    Ok(Ok(n)) if n > 0 => {
                                                                        last_report = Instant::now();
                                                                        if let Some(gp) =
                                                                            GamepadState::parse(
                                                                                &buf[..n],
                                                                            )
                                                                        {
                                                                            let class = classify(&gp);
                                                                            set_input(class);

                                                                    // Servo: LT → left, RT → right, depth = magnitude.
                                                                    let angle =
                                                                        triggers_to_angle(
                                                                            gp.accel,
                                                                            gp.brake,
                                                                            90,
                                                                        );
                                                                            if angle != last_servo_angle
                                                                            {
                                                                                servo_ch.set_duty_hw(
                                                                                    angle_to_counts(
                                                                                        angle,
                                                                                    ),
                                                                                );
                                                                                last_servo_angle = angle;
                                                                            }

                                                                            if first_report {
                                                                                first_report = false;
                                                                                flog!(
                                                                                    "first report n={} bytes={:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                                                                                    n, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8]
                                                                                );
                                                                            }

                                                                            // Persist button state changes to flash so
                                                                            // unknown buttons can be identified later.
                                                                            if gp.buttons != last_btns {
                                                                                if last_btn_log.elapsed()
                                                                                    >= Duration::from_millis(150)
                                                                                {
                                                                                    flog!(
                                                                                        "btns={:04x} lx={} ly={} rx={} ry={} accel={} brake={}",
                                                                                        gp.buttons, gp.lx, gp.ly, gp.rx, gp.ry, gp.accel, gp.brake
                                                                                    );
                                                                                    last_btn_log = Instant::now();
                                                                                }
                                                                                last_btns = gp.buttons;
                                                                            }

                                                                            if last_log.elapsed()
                                                                                >= Duration::from_millis(100)
                                                                            {
                                                                                info!(
                                                                                    "lx={} ly={} rx={} ry={} btns={:04x} accel={} brake={} class={:?}",
                                                                                    gp.lx, gp.ly, gp.rx, gp.ry, gp.buttons, gp.accel, gp.brake, class
                                                                                );
                                                                                last_log = Instant::now();
                                                                            }
                                                                    } else {
                                                                        flog!(
                                                                            "hub port {} bad report n={} head={:02x} {:02x} {:02x} {:02x}",
                                                                            port, n, buf[0], buf[1], buf[2], buf[3]
                                                                        );
                                                                    }
                                                                }
                                                                Ok(Ok(_)) => {}
                                                                    Ok(Err(e)) => {
                                                                        error!("HID read failed: {:?}", e);
                                                                        flog!("hub port {} read error {:?}", port, e);
                                                                        break;
                                                                    }
                                                                    Err(_) => {
                                                                        if last_report.elapsed()
                                                                            >= Duration::from_secs(5)
                                                                        {
                                                                            error!("no reports for 5s — device gone?");
                                                                            flog!("hub port {} no reports 5s", port);
                                                                            break;
                                                                        }
                                                                    }
                                                                }
                                                            }
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
                                error!("Hub register failed: {:?}", e);
                                set_status(Status::ErrEnum);
                                flog!("hub register error {:?}", e);
                                Timer::after(Duration::from_millis(1000)).await;
                            }
                        }
                    }
                }
            }
            Err(e) => {
                error!("Enumeration failed: {:?}", e);
                set_status(Status::ErrEnum);
                flog!("enum error {:?}", e);
                Timer::after(Duration::from_millis(1000)).await;
            }
        }
        set_status(Status::Waiting);
        // Give the bus a beat before re-attempting so the LED shows yellow
        // between retries instead of looking stuck.
        Timer::after(Duration::from_millis(2000)).await;
    }
}
