//! Software-in-the-loop: run the host-compiled pump-controller firmware
//! against the Embrig virtual bus, driven by the exact same YAML suites used
//! for the built-in virtual ECUs.
//!
//! ## The embedded code
//!
//! `ControllerFirmware` below is the code under test — a stand-in for firmware
//! you would compile for a target board. It implements the [`NetEcu`] trait:
//!
//! * `on_message` — receives the temperature reading on `0x100` from the bus.
//! * `update` — runs the control law (valve open while 10..=90 °C, closed
//!   otherwise, fail-safe) and transmits `valve_open` on `0x200` every 50 ms.
//!
//! This is the only part that is "the device"; everything else in this file is
//! host test harness.
//!
//! ## The host harness
//!
//! `main` is test infrastructure, not embedded code: it loads
//! `fixtures/vehicle.yaml` + `fixtures/controller.dbc`, binds the `controller`
//! node to a fresh `ControllerFirmware` through a [`SilRegistry`], and runs the
//! YAML suites with [`sil_run`]. The firmware factory is re-invoked before
//! every test, so firmware state never leaks between tests.
//!
//! ## Supporting files
//!
//! * `fixtures/vehicle.yaml` — declares the `sensor` config node (the stimulus
//!   source, overridden by `set_signal`) and the `controller` node with
//!   `type: sil` (firmware is code, not config).
//! * `fixtures/controller.dbc` — the message map: `SensorInput` (`0x100`) and
//!   `ValveCommand` (`0x200`).
//! * `suites/nominal.yaml` — a valid temperature keeps the valve open.
//! * `suites/overrange.yaml` — an over-range temperature closes it.
//!
//! ## How to run
//!
//! ```text
//! cargo run --example sil_firmware --package embrig-sil
//! ```
//!
//! → `2 passed, 0 failed`. Each simulated firmware step runs under a wall-clock
//! budget (default 100 ms, `step_budget_us` in YAML); an overrun fails the test
//! instead of hanging it, and `set_signal` on the firmware itself is rejected.

use std::path::Path;
use std::sync::OnceLock;

use embrig_core::frame::CanFrame;
use embrig_core::time::Timestamp;
use embrig_core::{NetEcu, NetEcuError};
use embrig_dbc::Network;
use embrig_models::load_vehicle_config;
use embrig_sil::{sil_run, SilRegistry};

const DBC: &str = include_str!("fixtures/controller.dbc");

fn network() -> &'static Network {
    static NETWORK: OnceLock<Network> = OnceLock::new();
    NETWORK.get_or_init(|| embrig_dbc::parse(DBC).expect("valid controller DBC"))
}

/// The system under test: opens a valve while the temperature is within
/// 10..90 °C, closes it otherwise (fail-safe).
struct ControllerFirmware {
    name: String,
    temperature: f64,
    open: bool,
    next_tx: Timestamp,
    period_us: u64,
}

impl ControllerFirmware {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            temperature: 0.0,
            open: false,
            next_tx: 0,
            period_us: 50_000,
        }
    }
}

impl NetEcu<CanFrame> for ControllerFirmware {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, frame: &CanFrame, _time: Timestamp) {
        if frame.id == 0x100 {
            if let Ok(temperature) = network()
                .message(0x100)
                .unwrap()
                .decode_signal(&frame.data, "temperature")
            {
                self.temperature = temperature;
                self.open = (10.0..=90.0).contains(&self.temperature);
            }
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if time >= self.next_tx {
            let data = network()
                .message(0x200)
                .unwrap()
                .encode_signals(&[("valve_open", if self.open { 1.0 } else { 0.0 })])
                .expect("valve_open encodes");
            out.push(CanFrame::new(0x200, data).expect("8-byte frame"));
            self.next_tx = time + self.period_us;
        }
    }
}

fn main() -> anyhow::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let (config, _) = load_vehicle_config(&root.join("fixtures/vehicle.yaml"))?;
    let dbc = root.join("fixtures/controller.dbc");

    let mut registry = SilRegistry::new();
    registry.register(
        "controller",
        |name: &str, _budget: u64| -> Result<Box<dyn NetEcu<CanFrame>>, NetEcuError> {
            Ok(Box::new(ControllerFirmware::new(name)))
        },
    );

    let suites = embrig_test::collect_suites(&root.join("suites"))?;
    let result = sil_run(&config, &dbc, registry, &suites)?;

    println!("SIL suite: {}", result.file);
    embrig_test::print_suite(&result);
    if result.failed() > 0 {
        std::process::exit(1);
    }
    Ok(())
}
