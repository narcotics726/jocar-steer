/*

 */
#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config, I2c},
    time::Rate,
};
use panic_rtt_target as _;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

/// PCF8575 I2C address (A0=A1=A2=GND).
const PCF8575_ADDR: u8 = 0x20;

/// Low byte: P06=0 (亮), P07=1 (灭), 其余高阻.
const STATE_P06_ON: u8 = 0xBF; // 0b1011_1111
/// Low byte: P06=1 (灭), P07=0 (亮), 其余高阻.
const STATE_P07_ON: u8 = 0x7F; // 0b0111_1111
/// Low byte: 全部高阻 (全灭).
const STATE_ALL_OFF: u8 = 0xFF; // 0b1111_1111
/// High byte: P10–P17 全部高阻.
const HIGH_BYTE: u8 = 0xFF;

/// Write 16-bit value to PCF8575.
fn pcf8575_write(i2c: &mut I2c<'_, esp_hal::Blocking>, low: u8, high: u8) {
    if let Err(e) = i2c.write(PCF8575_ADDR, &[low, high]) {
        info!("I2C write error: {:?}", e);
    }
}

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

    // Initialize I2C0 with 100kHz for PCF8575.
    let i2c_config = Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("Failed to initialize I2C0")
        .with_sda(peripherals.GPIO17)
        .with_scl(peripherals.GPIO18);

    info!("PCF8575 I2C ready: SDA=G17 SCL=G18 addr=0x{:02X}", PCF8575_ADDR);

    // Start with all pins off.
    pcf8575_write(&mut i2c, STATE_ALL_OFF, HIGH_BYTE);
    info!("--- PCF8575 LED Test ---");
    info!("P06=LED1  P07=LED2  cycle: P06→P07→off");

    loop {
        // P06 on, P07 off
        pcf8575_write(&mut i2c, STATE_P06_ON, HIGH_BYTE);
        info!("P06 ON");
        Timer::after(Duration::from_millis(500)).await;

        // P07 on, P06 off
        pcf8575_write(&mut i2c, STATE_P07_ON, HIGH_BYTE);
        info!("P07 ON");
        Timer::after(Duration::from_millis(500)).await;

        // Both off
        pcf8575_write(&mut i2c, STATE_ALL_OFF, HIGH_BYTE);
        info!("ALL OFF");
        Timer::after(Duration::from_millis(500)).await;
    }
}
