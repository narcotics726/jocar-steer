#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use jocar_steer::battery::BatteryMonitor;
use panic_rtt_target as _;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

/// Update interval in milliseconds.
const UPDATE_MS: u64 = 500;

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

    info!("battery-mon-test: Embassy initialized");

    // --- ADC setup: GPIO1 = ADC1_CH0, 11dB attenuation, curve calibration ---
    let mut adc1_config = AdcConfig::new();
    let mut bat_pin = adc1_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(
        peripherals.GPIO1,
        Attenuation::_11dB,
    );
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    // R1 = 100kΩ, R2 = 47kΩ → V_BAT = V_ADC × 147/47
    let mut monitor = BatteryMonitor::new(147, 47);

    info!("ADC ready: GPIO1 (ADC1_CH0), 11dB atten, curve cal");

    // Warm up
    while !monitor.filled() {
        let adc_mv = adc1.read_blocking(&mut bat_pin);
        monitor.sample(adc_mv);
        Timer::after(Duration::from_millis(10)).await;
    }

    let t = monitor.lock_type();
    info!("Warm-up complete, battery type locked: {:?}", t);

    loop {
        let adc_mv = adc1.read_blocking(&mut bat_pin);
        monitor.sample(adc_mv);

        let r = monitor.reading();
        let low = if r.is_low { " LOW!" } else { "" };

        info!(
            "BAT: adc={=u16}mV bat={=u32}mV type={:?}{}",
            r.adc_mv,
            r.bat_mv,
            r.battery_type,
            low,
        );

        Timer::after(Duration::from_millis(UPDATE_MS)).await;
    }
}
