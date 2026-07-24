#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use jocar_steer::ps2::{Ps2Controller, Ps2Event};
use panic_rtt_target as _;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "main is the entry point"
)]
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

    info!("Embassy initialized!");

    let mut ps2 = Ps2Controller::new(
        peripherals.GPIO4,  // DAT (input from controller)
        peripherals.GPIO5,  // CMD (output to controller)
        peripherals.GPIO7,  // CLK (output clock)
        peripherals.GPIO6,  // ATT / CS (output, active-low)
    );

    info!("PS2 driver ready: DAT=G4 CMD=G5 CLK=G7 CS=G6");

    Timer::after(Duration::from_millis(500)).await;
    info!("--- PS2 Test ---");

    // NOTE: intentionally does NOT call enter_analog_mode(). This is a raw
    // probe: press MODE by hand and watch how the full packet changes so we
    // can learn what THIS clone receiver actually reports for analog vs digital.
    loop {
        match ps2.read() {
            Ps2Event::Analog(state) => {
                info!(
                    "b1={:02x} analog={} | RAW {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                    state.raw[1],
                    state.is_analog(),
                    state.raw[0], state.raw[1], state.raw[2],
                    state.raw[3], state.raw[4], state.raw[5],
                    state.raw[6], state.raw[7], state.raw[8],
                );
            }
            Ps2Event::LostAnalog => {
                info!("Lost analog");
            }
            Ps2Event::RecoveredAnalog => {
                info!("Recovered analog");
            }
        }

        Timer::after(Duration::from_millis(200)).await;
    }
}
