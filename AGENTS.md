/*

 */

# jocar-steer — ESP32-S3 no_std Rust Firmware

## Build & Flash

```bash
cargo build                    # Debug build
cargo build --release          # Release build
cargo run                      # Build + flash via espflash over TTL port (auto-detected)
cargo test                     # Alias to `cargo build` (no test harness; all verification is on-device)
```

## Toolchain

- **Rust toolchain:** `esp` (Xtensa fork, see `rust-toolchain.toml`)
- **Target:** `xtensa-esp32s3-none-elf`
- **Runner (bins):** `espflash flash --monitor -L defmt` — port auto-detected, no `--port` needed (configured in `.cargo/config.toml`)
- **No test harness:** `[alias] test = "build"` in `.cargo/config.toml`; verification is manual (flash + monitor)
- **Direnv:** source `.envrc` to set Xtensa toolchain PATH

## USB port usage (TTL vs OTG)

- **TTL port** (USB-UART bridge on UART0, GPIO43/44 — currently a WCH CH9102, enumerated as `/dev/ttyACM0`) → flashing + console via `espflash`, defmt decoded by monitor (`-L defmt`); the runner auto-detects the port (no `--port`), so swapping adapters/boards needs no config change
- **OTG port** (GPIO19/20, native USB) → reserved for the USB host gamepad receiver; USB-Serial/JTAG is unavailable while OTG is active
- The ESP32-S3 has a single USB PHY shared between USB-Serial/JTAG and USB OTG — they cannot be active at the same time. Hold BOOT at reset to enter download mode if auto-reset wiring is missing.

## Current car hardware (2026-08)

- **Board:** HW678 (white-label ESP32-S3 DevKitC-1 clone, WROOM-1 octal module); powered by a 5 V buck from the battery into 5VIN
- **Battery:** 2S 7.4 V LiPo → TB6612 VM direct
- **Motor:** 12 V-rated N30 4000 RPM, single rear drive (no differential); TB6612 channel A
- **Servo:** SG90 · **Gamepad:** ZD receiver on the native USB port (OTG)
- **Pin map:** `docs/wiring.md` (per chassis — the 1/10 car differs)
- **Control tuning:** steer ±30° + steer-throttle mix (50 % cut at full lock); full duty (4095) is safe — the motor is 12 V-rated, so 2S is under-voltage, and heat comes from stall current, not voltage

## Architecture

- `#![no_std]` with `esp-alloc` heap (72 KiB in reclaimed RAM)
- Async runtime: `esp-rtos` — a preemptive RTOS in front, with the embassy executor integrated on top of it (embassy-executor 0.10 has no per-task stacks)
- Control path: USB session (connect → enumerate → read → read-timeout/failsafe/watchdog) → chassis policy (steering, mix, slew, kick) → motor driver (TB6612 today, ESC for the 1/10 car). Both cars share it through the library; a bin keeps only its pin map, chassis parameters and peripherals.
- Logging: `defmt` over UART0 via `esp-println` (`defmt-espflash` framing), decoded by `espflash ... -L defmt`
- Persistent event log: one unused flash sector, for what happened while the console was not attached (see `src/flash_log.rs`); replay with `espflash read-flash` + `tools/flashlog_decode.py`
- Panic handler: custom `#[panic_handler]` printing via defmt (`defmt::Display2Format`), then a chip reset — a panicking motor loop must not keep its last command
- Bootloader: `esp-bootloader-esp-idf` with `esp_app_desc!()`
- Stack smashing protection enabled (`-Z stack-protector=all`)

## Constraints

- `#[deny(clippy::mem_forget)]` — forbidden on esp-hal DMA buffer types
- `#[deny(clippy::large_stack_frames)]` — stack threshold 1024 bytes (`clippy.toml`)
- No `std`. Use `esp-alloc` for heap, `static_cell` for statics.
- GPIOs 0,3,45,46 reserved for bootstrap; GPIOs 27-37 used by PSRAM/flash on WROOM-1 octal module.
