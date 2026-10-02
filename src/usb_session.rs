//! USB host session: find the receiver, enumerate it, run the control loop.
//!
//! Extracted from the firmware bins because both cars share the whole input
//! path: the same receiver, the same enumeration fallbacks, the same safety
//! envelope. A bin supplies a USB driver and a [`Chassis`] (which carries its
//! own control configuration); everything between the root port and the
//! actuator writes lives here.
//!
//! # Safety envelope (this is the code that owns it)
//!
//! - **Read timeout** ([`READ_TIMEOUT_MS`]): bounds every await in the control
//!   phase, so the loop always gets back to feeding the watchdog and to the
//!   failsafe check.
//! - **Control-phase watchdog** ([`WDT_TIMEOUT_MS`]): enabled only while the
//!   control loop runs, fed from inside that loop. It is deliberately *not*
//!   enabled during connection/enumeration/hub waits — a car booted with no
//!   receiver must be allowed to wait indefinitely. Those phases have nothing
//!   *commanded* (the chassis is halted before entering them); the drivers
//!   themselves are enabled from construction, which is their own safe state
//!   (TB6612: STBY high, duty 0, direction pins low).
//! - **Failsafe** (`ControlConfig::failsafe_timeout_ms`): reports stopping is
//!   not an error the USB stack reports — the link can look perfectly healthy
//!   while nothing arrives, or while reports arrive in a layout the parser
//!   rejects. [`Chassis::failsafe_expired`] is therefore evaluated on every
//!   loop iteration, not only when a read times out, and it keys on the last
//!   *accepted* report.
//!
//! **The watchdog timeout must stay above the longest gap between two `feed`
//! calls in the control loop.** Today that gap is one read timeout
//! ([`READ_TIMEOUT_MS`]) plus the iteration's own work, hence WDT = 2× it. The
//! known consumer of the remaining margin is [`FlashLog::record`], which can
//! erase a 4 KiB sector (tens of milliseconds) from inside the loop; anything
//! longer added here must raise the WDT timeout with it.

use defmt::{error, info};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb_driver::host::{UsbHostAllocator, UsbHostController};
use embassy_usb_host::{
    BusController, BusHandle, BusRoute, BusState,
    class::hub::{HubEvent, HubHandler},
    handler::{EnumerationInfo, HandlerEvent},
};
use esp_hal::ledc::channel::ChannelHW;
use esp_hal::timer::timg::{MwdtStage, TimerGroupInstance, Wdt};

use crate::chassis::{Chassis, MotorDriver};
use crate::flash_log::{self, FlashLog};
use crate::usb_gamepad::{GamepadHost, GamepadState};

/// Timeout on a single report read.
pub const READ_TIMEOUT_MS: u64 = 500;

/// Control-phase watchdog timeout (see the module docs for the constraint).
pub const WDT_TIMEOUT_MS: u64 = 1000;

/// Timeout on a single enumeration attempt.
const ENUM_TIMEOUT_MS: u64 = 5000;
/// How many times to retry enumerating a directly-attached device before
/// giving up on it and waiting for a fresh connection event.
const ENUM_ATTEMPTS: u8 = 3;
/// Pause between enumeration attempts.
const ENUM_RETRY_SPACING_MS: u64 = 200;

/// Let the device finish its own power-up before the first control transfer. A
/// hub port inserts this delay implicitly; a direct connection does not, which
/// is one reason direct enumeration can stall on this receiver.
const SETTLE_MS: u64 = 200;

/// How long to wait for a device after one was already seen, before resetting
/// the chip to rebuild the USB host stack (see [`run`]).
const RECONNECT_TIMEOUT_MS: u64 = 5000;

/// Downstream ports the hub fallback handles.
const HUB_PORTS: usize = 8;

/// Size of the configuration-descriptor buffer (USB allows up to 256 bytes for
/// the first configuration).
const CONFIG_BUF_LEN: usize = 256;

/// One USB host instance: root-port controller + shareable bus handle.
pub struct UsbSession<'d, D: UsbHostController<'d>> {
    ctrl: BusController<'d, D>,
    bus: BusHandle<'d, D::Allocator>,
}

/// Bus-wide state (address table, enumeration lock). One instance per chip;
/// `BusState` is a `static` because the bus outlives any single session.
static BUS_STATE: BusState = BusState::new();

impl<'d, D: UsbHostController<'d>> UsbSession<'d, D> {
    /// Split `driver` into the controller/handle pair this session drives.
    pub fn new(driver: D) -> Self {
        let (ctrl, bus) = embassy_usb_host::bus(driver, &BUS_STATE);
        Self { ctrl, bus }
    }

    /// Connect, enumerate and run control reports until the chip resets.
    ///
    /// Never returns: every exit path either loops back to waiting for a
    /// connection or resets the chip, because a half-dead USB stack on a moving
    /// car is not a recoverable state from inside the firmware.
    #[allow(
        clippy::large_stack_frames,
        reason = "one 256 B configuration-descriptor buffer (the protocol maximum for the \
        first configuration) lives in this frame; the bin's `main` carries the same allowance"
    )]
    pub async fn run<S, M, TG>(
        &mut self,
        chassis: &mut Chassis<S, M>,
        log: &mut FlashLog<'_>,
        wdt: &mut Wdt<TG>,
    ) -> !
    where
        S: ChannelHW,
        M: MotorDriver,
        TG: TimerGroupInstance,
    {
        // Configured here rather than in the bin: the timeout is only correct
        // relative to this module's own read timeout.
        wdt.set_timeout(
            MwdtStage::Stage0,
            esp_hal::time::Duration::from_millis(WDT_TIMEOUT_MS),
        );

        // Set once a device has been seen: from then on, losing it and not
        // getting it back is treated as a wedged USB host.
        let mut had_session = false;

        loop {
            // ── Wait for a device on the root port ───────────────────────
            let speed = if had_session {
                // After a device is removed the root port is not always
                // re-armed for a re-attach (observed: no further connect
                // events), and the hub path can wait on its event queue
                // forever too. A chip reset rebuilds the whole USB host stack,
                // so it is the only reliable way back without a physical
                // replug. Guarded by `had_session` so a car booted with
                // nothing plugged in waits instead of reboot-looping.
                match embassy_time::with_timeout(
                    Duration::from_millis(RECONNECT_TIMEOUT_MS),
                    self.ctrl.wait_for_connection(),
                )
                .await
                {
                    Ok(speed) => speed,
                    Err(_) => {
                        error!(
                            "no device for {} ms after a loss — resetting to recover USB",
                            RECONNECT_TIMEOUT_MS
                        );
                        log.record(flash_log::EV_RESET, format_args!("usb lost"));
                        // Replay the persisted log now: while the direct
                        // adapter occupies the OTG port the console is
                        // unattached, so the history is read after the user
                        // swaps the cable back.
                        log.dump();
                        esp_hal::system::software_reset()
                    }
                }
            } else {
                // No device has been seen yet. Wait indefinitely — a car
                // booted with nothing attached must not reboot-loop — but keep
                // replaying the persisted log every period so that attaching
                // the console later still shows what a previous session
                // recorded.
                loop {
                    match embassy_time::with_timeout(
                        Duration::from_millis(RECONNECT_TIMEOUT_MS),
                        self.ctrl.wait_for_connection(),
                    )
                    .await
                    {
                        Ok(speed) => break speed,
                        Err(_) => log.dump(),
                    }
                }
            };
            had_session = true;
            info!("Device connected at {:?}", speed);
            log.record(flash_log::EV_CONNECTED, format_args!("{:?}", speed));

            Timer::after(Duration::from_millis(SETTLE_MS)).await;

            // ── Enumerate ────────────────────────────────────────────────
            let mut config_buf = [0u8; CONFIG_BUF_LEN];
            let Some((enum_info, config_len)) = enumerate_with_retries(
                &self.bus,
                BusRoute::Direct(speed),
                &mut config_buf,
                log,
            )
            .await
            else {
                Timer::after(Duration::from_millis(1000)).await;
                continue;
            };

            info!(
                "Enumerated: VID={:04x} PID={:04x}",
                enum_info.device_desc.vendor_id, enum_info.device_desc.product_id
            );
            log.record(
                flash_log::EV_ENUM_OK,
                format_args!(
                    "vid={:04x} pid={:04x}",
                    enum_info.device_desc.vendor_id, enum_info.device_desc.product_id
                ),
            );

            // ── Gamepad? Otherwise treat it as a hub ─────────────────────
            match GamepadHost::new(&self.bus, &config_buf[..config_len], &enum_info) {
                Ok(mut hid) => {
                    log.record(flash_log::EV_IFACE, format_args!("iface0 ok"));
                    log.record(flash_log::EV_SESSION, format_args!("direct reading"));
                    info!("iface0 ready — reading gamepad reports");

                    run_control(&mut hid, chassis, log, wdt, "direct").await;

                    chassis.halt();
                    // The library requires the application to release the
                    // device address once the device is gone.
                    self.bus.free_address(enum_info.device_address);
                    log.record(flash_log::EV_LOST, format_args!("direct device gone"));
                    info!("Device disconnected, waiting for next");
                }
                Err(e) => {
                    log.record(flash_log::EV_IFACE, format_args!("no iface: {:?}", e));
                    info!("no direct gamepad — trying hub");
                    run_hub(&self.bus, &enum_info, chassis, log, wdt).await;
                }
            }
        }
    }
}

/// Enumerate with in-place retries.
///
/// Falling back to `wait_for_connection` on failure wedges the loop: that call
/// waits for a *new* connection event, and a device that stays plugged in never
/// produces one — the car then needed a physical replug. Each attempt stays
/// bounded so a non-responding device cannot hang the loop forever (seen as a
/// stuck state on a flaky power rail).
async fn enumerate_with_retries<'d, A>(
    bus: &BusHandle<'d, A>,
    route: BusRoute,
    config_buf: &mut [u8],
    log: &mut FlashLog<'_>,
) -> Option<(EnumerationInfo, usize)>
where
    A: UsbHostAllocator<'d>,
{
    for attempt in 1..=ENUM_ATTEMPTS {
        let result = embassy_time::with_timeout(
            Duration::from_millis(ENUM_TIMEOUT_MS),
            bus.enumerate(route, config_buf),
        )
        .await;

        match &result {
            Ok(Ok(_)) => {}
            Ok(Err(e)) => {
                error!("Enumeration attempt {} failed: {:?}", attempt, e);
                log.record(
                    flash_log::EV_ENUM_FAIL,
                    format_args!("{:?} try{}", e, attempt),
                );
            }
            Err(_) => {
                error!("Enumeration attempt {} timed out", attempt);
                log.record(
                    flash_log::EV_ENUM_FAIL,
                    format_args!("timeout after 5s try{}", attempt),
                );
            }
        }

        match result {
            Ok(Ok(found)) => return Some(found),
            // Retry on a timeout as well as on a transfer error (e.g. STALL).
            _ => Timer::after(Duration::from_millis(ENUM_RETRY_SPACING_MS)).await,
        }
    }

    error!(
        "Enumeration failed {} times — waiting for a reconnect",
        ENUM_ATTEMPTS
    );
    None
}

/// Service a device that is not the gamepad: register it as a hub and read
/// gamepads plugged into its downstream ports.
///
/// Returns when the hub itself errors (the caller then goes back to waiting for
/// a connection). The hub's own `wait_for_event` runs with the watchdog **off**
/// — an idle hub may legitimately block forever — while a downstream port
/// session goes through [`run_control`] and therefore re-enables it.
#[allow(
    clippy::large_stack_frames,
    reason = "same configuration-descriptor buffer as `run`, plus `HubHandler`'s port state"
)]
async fn run_hub<'d, A, S, M, TG>(
    bus: &BusHandle<'d, A>,
    hub_enum: &EnumerationInfo,
    chassis: &mut Chassis<S, M>,
    log: &mut FlashLog<'_>,
    wdt: &mut Wdt<TG>,
) where
    A: UsbHostAllocator<'d>,
    S: ChannelHW,
    M: MotorDriver,
    TG: TimerGroupInstance,
{
    log.record(flash_log::EV_HUB, format_args!("registering"));

    let mut hub = match HubHandler::<_, HUB_PORTS>::try_register(bus, hub_enum).await {
        Ok(hub) => hub,
        Err(e) => {
            error!("hub register failed: {:?}", e);
            log.record(flash_log::EV_HUB, format_args!("reg failed: {:?}", e));
            Timer::after(Duration::from_millis(500)).await;
            return;
        }
    };
    log.record(
        flash_log::EV_HUB,
        format_args!("registered, waiting for events"),
    );
    info!("hub registered");

    loop {
        match hub.wait_for_event().await {
            Ok(HandlerEvent::HandlerEvent(HubEvent::DeviceDetected { port, speed })) => {
                info!("hub port {} connected speed={:?}", port, speed);

                let mut port_cfg_buf = [0u8; CONFIG_BUF_LEN];
                let result = embassy_time::with_timeout(
                    Duration::from_millis(ENUM_TIMEOUT_MS),
                    hub.enumerate_port(&mut port_cfg_buf, port, speed),
                )
                .await;

                let (port_enum, len) = match result {
                    Ok(Ok(found)) => found,
                    Err(_) => {
                        error!("hub port {} enumeration timed out", port);
                        continue;
                    }
                    Ok(Err(e)) => {
                        error!("hub port {} enumeration failed: {:?}", port, e);
                        continue;
                    }
                };

                info!(
                    "hub port {} enumerated vid={:04x} pid={:04x}",
                    port, port_enum.device_desc.vendor_id, port_enum.device_desc.product_id
                );

                match GamepadHost::new(bus, &port_cfg_buf[..len], &port_enum) {
                    Ok(mut hid) => {
                        log.record(flash_log::EV_SESSION, format_args!("hub{} reading", port));
                        info!("iface0 ready on hub port {}", port);

                        run_control(&mut hid, chassis, log, wdt, "hub").await;

                        chassis.halt();
                        bus.free_address(port_enum.device_address);
                        log.record(flash_log::EV_LOST, format_args!("hub port {} gone", port));
                        info!("Device on hub port {} gone", port);
                    }
                    Err(_) => {
                        info!("hub port {} not a gamepad", port);
                        // We enumerated this device but bind nothing to it:
                        // hand the address back, or the address table fills up.
                        bus.free_address(port_enum.device_address);
                    }
                }
            }
            Ok(_) => {}
            Err(e) => {
                error!("hub event error: {:?}", e);
                log.record(flash_log::EV_HUB, format_args!("event err: {:?}", e));
                return;
            }
        }
    }
}

/// The control phase: feed reports to the chassis, feed the watchdog, stop on
/// silence.
///
/// Returns when the device stops answering or the failsafe window expires; the
/// caller owns the teardown. The watchdog is enabled for the duration (this is
/// the only phase that can drive the motors) and disabled again on the way out,
/// before the caller blocks on anything unbounded.
///
/// The enable/disable pair has no RAII guard because this function is awaited
/// directly and has no early return: do not wrap it in `select`/`with_timeout`,
/// or a dropped future would leave the watchdog running into the caller's
/// unbounded waits.
#[allow(
    clippy::large_stack_frames,
    reason = "read buffer plus the diagnostic formatting in the loop"
)]
async fn run_control<'d, A, S, M, TG>(
    hid: &mut GamepadHost<'d, A>,
    chassis: &mut Chassis<S, M>,
    log: &mut FlashLog<'_>,
    wdt: &mut Wdt<TG>,
    label: &str,
) where
    A: UsbHostAllocator<'d>,
    S: ChannelHW,
    M: MotorDriver,
    TG: TimerGroupInstance,
{
    let mut buf = [0u8; 64];
    let mut last_info = Instant::now();
    // Diagnostics: prove whether reports arrive at all, and whether they parse
    // (each logged once per session). Without this, "no control" cannot be told
    // apart from "reports arrive in an unknown layout".
    let mut first_report_logged = false;
    let mut parse_fail_logged = false;

    wdt.enable();
    // Feed immediately: an enabled watchdog counts from the moment it was
    // enabled, and the first read may legitimately take the whole read timeout.
    wdt.feed();

    loop {
        match embassy_time::with_timeout(Duration::from_millis(READ_TIMEOUT_MS), hid.read(&mut buf))
            .await
        {
            Ok(Ok(n)) if n > 0 => {
                let now = Instant::now();
                if let Some(gp) = GamepadState::parse(&buf[..n]) {
                    if !first_report_logged {
                        first_report_logged = true;
                        log.record(flash_log::EV_FIRST_REPORT, format_args!("n={}", n));
                    }

                    chassis.on_report(gp.rx, gp.ly, now);

                    if last_info.elapsed() >= Duration::from_millis(100) {
                        info!(
                            "{} lx={} ly={} rx={} ry={} btns={:04x}",
                            label, gp.lx, gp.ly, gp.rx, gp.ry, gp.buttons
                        );
                        last_info = Instant::now();
                    }
                } else if !parse_fail_logged {
                    // Reports arrive but the layout is not the one this
                    // firmware knows (which mode is the receiver in?).
                    parse_fail_logged = true;
                    let b0 = *buf.first().unwrap_or(&0);
                    let b1 = *buf.get(1).unwrap_or(&0);
                    log.record(
                        flash_log::EV_PARSE_FAIL,
                        format_args!("n={} b0={:02x} b1={:02x}", n, b0, b1),
                    );
                }
            }
            Ok(Ok(_)) => {}
            Ok(Err(e)) => {
                error!("{} HID read failed: {:?}", label, e);
                log.record(flash_log::EV_READ_ERR, format_args!("{:?}", e));
                break;
            }
            // A timed-out read is not itself the end of the session — the
            // failsafe below is the single place that decides when silence
            // becomes a stop.
            Err(_) => {}
        }

        // Evaluated on *every* iteration, not only when a read times out:
        // frames that keep arriving but never parse hold the link looking
        // perfectly healthy while nothing reaches the chassis, and the last
        // commanded speed would otherwise stay live forever. The clock is the
        // last *accepted* report, so both failure modes stop the car here.
        if chassis.failsafe_expired(Instant::now()) {
            let window_ms = chassis.failsafe_timeout_ms();
            error!(
                "{}: no accepted report for {} ms — stopping",
                label, window_ms
            );
            // Stop the car *before* touching flash: recording the event may
            // erase a whole sector, and that is the one window where a
            // commanded throttle would outlive the decision to stop.
            chassis.halt();
            log.record(
                flash_log::EV_STALE,
                format_args!("no accepted report {}ms", window_ms),
            );
            break;
        }

        // Fed from inside the control loop on purpose: a task that fed the
        // watchdog for us would keep the chip alive through exactly the
        // hang we are guarding against.
        wdt.feed();
    }

    wdt.disable();
}
