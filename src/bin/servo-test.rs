#![no_std]
#![no_main]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    LSGlobalClkSource, Ledc, LowSpeed,
    channel::{self, ChannelIFace, ChannelHW},
    timer::{self, TimerIFace},
};
use esp_hal::time::Rate;
use esp_println as _;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("Panic: {}", defmt::Display2Format(info));
    loop {}
}

// ── MG90S servo timing (50 Hz PWM, 12-bit LEDC duty) ─────────────────
//
// MG90S datasheet: 20 ms period; ~1.0 ms = -90°, 1.5 ms = 0°, ~2.0 ms = +90°.
// Stay within 1000..=2000 µs to avoid hitting mechanical stops.

const PERIOD_US: u32 = 20_000;           // 50 Hz
const DUTY_MAX: u32 = 1 << 12;           // 12-bit → 4096 counts/period
const CENTER_US: i32 = 1500;             // 0° centre
const US_PER_90DEG: i32 = 500;           // 500 µs per 90°

fn angle_to_counts(deg: i32) -> u32 {
    let deg = deg.clamp(-90, 90);
    let pulse_us = (CENTER_US + deg * US_PER_90DEG / 90) as u32;
    (DUTY_MAX * pulse_us) / PERIOD_US
}

// ── Sweep step size & delay ───────────────────────────────────────────
const SWEEP_STEP_DEG: i32 = 2;           // degrees per tick
const TICK_MS: u64 = 15;                 // ~66 Hz update rate
const PAUSE_MS: u64 = 1000;              // pause at each endpoint

// ── Entry point ──────────────────────────────────────────────────────

#[allow(
    clippy::large_stack_frames,
    reason = "main is the entry point; stack budget is generous here"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Reserve bootstrapping / flash pins (never touch these).
    let _ = peripherals.GPIO0;
    let _ = peripherals.GPIO3;
    let _ = peripherals.GPIO45;
    let _ = peripherals.GPIO46;

    // Flash / PSRAM (octal WROOM-1: G26–G37)
    let _ = peripherals.GPIO26;
    let _ = peripherals.GPIO27;
    let _ = peripherals.GPIO28;
    let _ = peripherals.GPIO29;
    let _ = peripherals.GPIO30;
    let _ = peripherals.GPIO31;
    let _ = peripherals.GPIO32;
    let _ = peripherals.GPIO33;
    let _ = peripherals.GPIO34;
    let _ = peripherals.GPIO35;
    let _ = peripherals.GPIO36;
    let _ = peripherals.GPIO37;

    // USB/JTAG
    let _ = peripherals.GPIO19;
    let _ = peripherals.GPIO20;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("servo-test: Embassy initialized");

    // --- LEDC setup (50 Hz for MG90S) ---
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

    let servo_pin = peripherals.GPIO13;
    let mut ch = ledc.channel(channel::Number::Channel0, servo_pin);
    ch.configure(channel::config::Config {
        timer: &lstimer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    info!("LEDC ready: Timer0 Ch0 → GPIO13 (MG90S servo)");

    info!("Starting MG90S sweep test (GPIO13) — 2 cycles, then park at centre");

    let mut current: i32 = 0;

    for cycle in 0..2 {
        info!("cycle {} / 2", cycle + 1);

        // ── Centre → Left limit ──
        info!("→ sweeping to LEFT  (-90°)");
        while current > -90 {
            current = (current - SWEEP_STEP_DEG).max(-90);
            ch.set_duty_hw(angle_to_counts(current));
            Timer::after(Duration::from_millis(TICK_MS)).await;
        }
        Timer::after(Duration::from_millis(PAUSE_MS)).await;

        // ── Left → Right limit ──
        info!("→ sweeping to RIGHT (+90°)");
        while current < 90 {
            current = (current + SWEEP_STEP_DEG).min(90);
            ch.set_duty_hw(angle_to_counts(current));
            Timer::after(Duration::from_millis(TICK_MS)).await;
        }
        Timer::after(Duration::from_millis(PAUSE_MS)).await;

        // ── Right → Centre ──
        info!("→ returning to CENTRE (0°)");
        while current > 0 {
            current = (current - SWEEP_STEP_DEG).max(0);
            ch.set_duty_hw(angle_to_counts(current));
            Timer::after(Duration::from_millis(TICK_MS)).await;
        }
        Timer::after(Duration::from_millis(PAUSE_MS)).await;
    }

    info!("done — parked at centre");

    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
