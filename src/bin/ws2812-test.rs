#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Level;
use esp_hal::rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;
use jocar_steer::control::DriveMode;
use jocar_steer::lighting::ws2812_stat_indicator::{
    Rgb, StatusInput, Ws2812StatIndicator,
};
use panic_rtt_target as _;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// ── WS2812 timing (80 MHz RMT clock = 12.5 ns / tick) ──────────────

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

// ── Simulation helpers ───────────────────────────────────────────────

/// Simulated state for the test sequence.
struct Sim {
    tick: u32,
    phase: u8,  // 0=boot, 1=servo, 2=diff, 3=batt-low, 4=disconnected, 5=recover
    phase_start_tick: u32,
}

impl Sim {
    fn new() -> Self {
        Self {
            tick: 0,
            phase: 0,
            phase_start_tick: 0,
        }
    }

    fn advance(&mut self) -> StatusInput {
        self.tick += 1;

        // Auto-advance phases after fixed durations.
        let in_phase = self.tick - self.phase_start_tick;
        match self.phase {
            0 if in_phase >= 30 => {
                // Boot fade done (~1 s) → go to Servo
                self.phase = 1;
                self.phase_start_tick = self.tick;
                info!("→ PHASE 1: Servo mode (green)");
            }
            1 if in_phase >= 60 => {
                // 2 s of Servo → switch to Diff
                self.phase = 2;
                self.phase_start_tick = self.tick;
                info!("→ PHASE 2: Diff mode (blue)");
            }
            2 if in_phase >= 60 => {
                // 2 s of Diff → simulate low battery
                self.phase = 3;
                self.phase_start_tick = self.tick;
                info!("→ PHASE 3: Low battery (slow red blink)");
            }
            3 if in_phase >= 90 => {
                // 3 s of low battery → simulate PS2 disconnect
                self.phase = 4;
                self.phase_start_tick = self.tick;
                info!("→ PHASE 4: PS2 disconnected (fast yellow blink)");
            }
            4 if in_phase >= 60 => {
                // 2 s of disconnect → recover
                self.phase = 5;
                self.phase_start_tick = self.tick;
                info!("→ PHASE 5: Recovered (back to Diff mode)");
            }
            5 if in_phase >= 60 => {
                // Loop back to Servo
                self.phase = 1;
                self.phase_start_tick = self.tick;
                info!("→ PHASE 1: Servo mode (green)  [loop]");
            }
            _ => {}
        }

        StatusInput {
            mode: match self.phase {
                0 | 1 => DriveMode::Servo,
                _ => DriveMode::Diff,
            },
            battery_is_low: self.phase == 3,
            ps2_connected: self.phase != 4,
            ticks_since_boot: self.tick,
        }
    }
}

// ── Entry point ──────────────────────────────────────────────────────

#[allow(clippy::large_stack_frames, reason = "main is the entry point")]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("ws2812-test: Embassy initialized");

    // --- RMT setup ---
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("RMT init");

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_idle_output(true);

    let mut channel = rmt
        .channel0
        .configure_tx(&tx_config)
        .expect("TX config")
        .with_pin(peripherals.GPIO48);

    info!("RMT ready: ch0 → GPIO48");

    // --- Indicator ---
    let mut indicator = Ws2812StatIndicator::new(15); // 15 ticks ≈ 500 ms fade
    let mut sim = Sim::new();
    let mut buf: WsBuf = [PulseCode::default(); BITS_PER_LED * MAX_LEDS + 1];

    info!("Starting state simulation loop...");

    loop {
        let input = sim.advance();
        let rgb = indicator.update(&input);

        encode_frame(&mut buf, &[rgb], 1);
        match channel.transmit(&buf) {
            Ok(tx) => channel = tx.wait().expect("TX wait"),
            Err((e, ch)) => {
                info!("TX error: {:?}", e);
                channel = ch;
            }
        }

        Timer::after(Duration::from_millis(33)).await;
    }
}
