/*

 */
# jocar-steer — ESP32-S3 no_std Rust Firmware

## Build & Flash

```bash
cargo build                    # Debug build
cargo build --release          # Release build
cargo run                      # Build + flash via probe-rs
cargo test                     # Run embedded-tests (semihosting)
```

## Toolchain

- **Rust toolchain:** `esp` (Xtensa fork, see `rust-toolchain.toml`)
- **Target:** `xtensa-esp32s3-none-elf`
- **Runner:** `probe-rs run --chip=esp32s3` (configured in `.cargo/config.toml`)
- **Direnv:** source `.envrc` to set Xtensa toolchain PATH

## Architecture

- `#![no_std]` with `esp-alloc` heap (72 KiB in reclaimed RAM)
- Async runtime: `esp-rtos` (based on `embassy-executor`)
- Logging: `defmt` over RTT (`panic-rtt-target`)
- Bootloader: `esp-bootloader-esp-idf` with `esp_app_desc!()`
- Stack smashing protection enabled (`-Z stack-protector=all`)

## File Layout

```
src/bin/main.rs    — Firmware entry point (#![no_main])
src/lib.rs         — Library root (#![no_std])
tests/             — embedded-test suites (semihosting)
build.rs           — Linker scripts & error hints
.cargo/config.toml — Target, runner, rustflags
```

## Constraints

- `#[deny(clippy::mem_forget)]` — forbidden on esp-hal DMA buffer types
- `#[deny(clippy::large_stack_frames)]` — stack threshold 1024 bytes (`clippy.toml`)
- No `std`. Use `esp-alloc` for heap, `static_cell` for statics.
- GPIOs 0,3,45,46 reserved for bootstrap; GPIOs 27-37 used by PSRAM/flash on WROOM-1 octal module.
