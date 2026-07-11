#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    Ledc, LowSpeed, LSGlobalClkSource,
    channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::time::Rate;

use jocar_steer::ps2::Ps2Controller;
use jocar_steer::steering::Steering;
use panic_rtt_target as _;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

/// Map PS2 right stick X axis (0..=255, center=128) to steering degrees.
/// 0 = full left, 128 = center, 255 = full right.
fn rx_to_deg(rx: u8, max_deg: i32) -> i32 {
    if rx == 128 {
        return 0;
    }
    let centered = rx as i32 - 128;
    // Scale: ±127 maps to ±max_deg
    (centered * max_deg) / 127
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

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

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // --- PS2 controller on GPIO10/11/12/46 ---
    let mut ps2 = Ps2Controller::new(
        peripherals.GPIO10, // DAT (input)
        peripherals.GPIO11, // CMD (output)
        peripherals.GPIO12, // CLK (output)
        peripherals.GPIO46, // ATT / CS (output, active-low)
    );
    info!("PS2 driver ready: DAT=G10 CMD=G11 CLK=G12 CS=G46");

    // Enter analog mode so the sticks are active.
    Timer::after(Duration::from_millis(200)).await;
    ps2.enter_analog_mode();
    info!("PS2 analog mode entered");

    // --- SG90 servo on GPIO4 via LEDC ---
    // Calibrate with X/A buttons to find center, left, right duty % values.
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

    let servo_pin = peripherals.GPIO4;
    let mut ch = ledc.channel(channel::Number::Channel0, servo_pin);
    ch.configure(channel::config::Config {
        timer: &lstimer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    // How far to swing the steering left/right from center, in degrees.
    const STEER_DEG: i32 = 60;
    // Static center offset in degrees to cancel residual mounting error.
    const CENTER_TRIM_DEG: i32 = 3;

    let mut steering = Steering::new(ch, CENTER_TRIM_DEG, STEER_DEG);

    info!(
        "Steering: offset={}°  max={}°  PS2 right stick → steer",
        CENTER_TRIM_DEG, STEER_DEG
    );

    // Deadzone around stick center (±3 counts out of 127).
    const RX_DEADZONE: i32 = 3;
    let mut last_deg: i32 = i32::MIN;

    loop {
        let state = ps2.read();
        let rx = state.rx() as i32;

        let centered = rx - 128;
        let cmd = if centered.abs() <= RX_DEADZONE {
            0
        } else {
            rx_to_deg(state.rx(), STEER_DEG)
        };

        if cmd != last_deg {
            info!("angle: {}°  rx: {}", cmd, state.rx());
            last_deg = cmd;
        }

        steering.set_angle(cmd);
        Timer::after(Duration::from_millis(33)).await; // ~30 Hz
    }
}
