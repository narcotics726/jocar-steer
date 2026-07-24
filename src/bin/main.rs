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

use jocar_steer::control;
use jocar_steer::ps2::{Button, Ps2Controller, Ps2Event};
use jocar_steer::steering::Steering;
use jocar_steer::tb6612::Tb6612;
use panic_rtt_target as _;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
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

    // --- PS2 controller on GPIO4/5/6/7 ---
    let mut ps2 = Ps2Controller::new(
        peripherals.GPIO4, // DAT (input)
        peripherals.GPIO5, // CMD (output)
        peripherals.GPIO7, // CLK (output)
        peripherals.GPIO6, // ATT / CS (output, active-low)
    );
    info!("PS2 driver ready: DAT=G4 CMD=G5 CLK=G7 CS=G6");

    // Enter analog mode so the sticks are active.
    Timer::after(Duration::from_millis(200)).await;
    ps2.enter_analog_mode();
    info!("PS2 analog mode entered");

    // --- SG90 servo on GPIO14 via LEDC ---
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

    let servo_pin = peripherals.GPIO14;
    let mut ch = ledc.channel(channel::Number::Channel0, servo_pin);
    ch.configure(channel::config::Config {
        timer: &lstimer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    // --- TB6612 motor PWM channels (G1=right, G2=left) ---
    // NOTE: Timer1 and Channel2 were both found to produce no output on this
    // setup (see diagnostics), so the motors use Timer2 + Channel1/Channel3.
    let mut motor_timer = ledc.timer::<LowSpeed>(timer::Number::Timer2);
    motor_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(10),
        })
        .unwrap();

    let right_pwm = peripherals.GPIO1;
    let mut ch1 = ledc.channel(channel::Number::Channel1, right_pwm);
    ch1.configure(channel::config::Config {
        timer: &motor_timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    let left_pwm = peripherals.GPIO2;
    let mut ch2 = ledc.channel(channel::Number::Channel3, left_pwm);
    ch2.configure(channel::config::Config {
        timer: &motor_timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    // ── Control configuration ──────────────────────────────────────────
    let cfg = control::ControlConfig {
        steer_max_deg: 60,
        motor_max_duty: 2048,
        motor_slew_step: 512,
        rx_deadzone: 3,
        ly_deadzone: 3,
    };

    // Static center offset in degrees to cancel residual mounting error.
    const CENTER_TRIM_DEG: i32 = 3;

    let mut steering = Steering::new(ch, CENTER_TRIM_DEG, cfg.steer_max_deg);

    info!(
        "Steering: offset={}°  max={}°  PS2 right stick → steer",
        CENTER_TRIM_DEG, cfg.steer_max_deg
    );

    // --- TB6612FNG motor driver (direct GPIO) ---
    // Direction pins: AIN1=G9, AIN2=G10, BIN1=G11, BIN2=G12, STBY=G13
    let mut motors = Tb6612::new(
        peripherals.GPIO9,
        peripherals.GPIO10,
        peripherals.GPIO11,
        peripherals.GPIO12,
        peripherals.GPIO13,
        ch1,
        ch2,
    );
    motors.enable();
    info!("Motors enabled");

    let mut mode = control::DriveMode::Rear;
    let mut motor_slew = control::MotorSlew::new(cfg.motor_slew_step);
    let mut mode_switch_held: bool = false;

    loop {
        match ps2.read() {
            Ps2Event::LostAnalog => {
                info!("PS2 lost analog — held; press MODE");
                Timer::after(Duration::from_millis(50)).await;
            }
            Ps2Event::RecoveredAnalog => {
                ps2.enter_analog_mode();
                info!("PS2 analog restored and re-locked");
                motor_slew.reset();
            }
            Ps2Event::Analog(state) => {
                // ── Mode switch: L3 + R3 (edge-triggered) ──────────────
                let switch_pressed = state.pressed(Button::L3)
                    && state.pressed(Button::R3);
                if switch_pressed && !mode_switch_held {
                    motors.set_left(0);
                    motors.set_right(0);
                    steering.center();
                    mode = mode.flip();
                    motor_slew.reset();
                    info!("mode → {:?}", mode);
                    mode_switch_held = true;
                    Timer::after(Duration::from_millis(33)).await;
                    continue;
                }
                mode_switch_held = switch_pressed;

                // ── Steering + Motors per mode ─────────────────────────
                match mode {
                    control::DriveMode::Rear => {
                        // Steering: right stick → servo
                        steering.set_target(control::rx_to_deg(
                            state.rx(),
                            cfg.rx_deadzone,
                            cfg.steer_max_deg,
                        ));
                        steering.update(8);

                        // Motors: symmetric
                        let (l_target, r_target) = control::motor_rear(
                            state.ly(),
                            cfg.ly_deadzone,
                            cfg.motor_max_duty,
                        );
                        let (l, r) = motor_slew.update(l_target, r_target);
                        motors.set_left(l);
                        motors.set_right(r);
                    }
                    control::DriveMode::Front => {
                        // Steering: hold centre
                        steering.set_target(0);
                        steering.update(8);

                        // Motors: differential
                        let (l_target, r_target) = control::motor_front(
                            state.ly(),
                            state.rx(),
                            cfg.ly_deadzone,
                            cfg.motor_max_duty,
                        );
                        let (l, r) = motor_slew.update(l_target, r_target);
                        motors.set_left(l);
                        motors.set_right(r);
                    }
                }

                Timer::after(Duration::from_millis(33)).await;
            }
        }
    }
}
