//! Software-in-the-loop against a hand-rolled CANopen node: run the
//! host-compiled [`CanOpenControllerFirmware`] on the Embrig virtual bus,
//! driven by the exact same YAML suites used for the DBC firmware — the
//! message map comes from a [`CanOpenCodec`] built from `eds.yaml` instead of
//! a DBC file.
//!
//! ## The embedded code
//!
//! The system under test is the CANopen node in
//! `firmware/canopen-controller` (`fw-canopen-controller`), a stand-in for
//! firmware you would compile for a target board. It implements the
//! [`NetEcu`] trait and speaks just enough CiA 301 for the demo:
//!
//! * **NMT** — boots PRE_OPERATIONAL; an NMT START (`0x000`, node 1) moves it
//!   to OPERATIONAL, NMT STOP back to STOPPED.
//! * **RPDO1** (`0x201`) — receives the process temperature.
//! * **TPDO1** (`0x181`) — transmits the fail-safe `valve_open` command.
//! * **Heartbeat** (`0x701`) — producer frame carrying the NMT state.
//!
//! ## The host harness
//!
//! `main` loads `vehicle.yaml` + `eds.yaml`, binds the `controller` node to a
//! fresh [`CanOpenControllerFirmware`] through a [`SilRegistry`], and runs the
//! YAML suites with [`sil_run_codec`]. The firmware factory is re-invoked
//! before every test, so firmware state never leaks between tests.
//!
//! ## Supporting files
//!
//! * `canopen/eds.yaml` — the node description (TPDO1/RPDO1 mapping, heartbeat
//!   period), the CANopen analogue of a DBC.
//! * `canopen/vehicle.yaml` — the `master_rpdo` + `master_nmt` config nodes
//!   (CANopen masters) and the `controller` node with `type: sil`.
//! * `canopen/suites/nominal.yaml` — NMT START + a valid temperature keeps the
//!   valve open and the node OPERATIONAL.
//! * `canopen/suites/overrange.yaml` — an over-range temperature closes it.
//!
//! ## How to run
//!
//! ```text
//! cargo run --example sil_canopen --package embrig-sil
//! ```
//!
//! → `2 passed, 0 failed`.

use std::path::Path;

use embrig_canopen::codec::CanOpenCodec;
use embrig_canopen::spec::EcuSpec;
use embrig_core::frame::CanFrame;
use embrig_core::{NetEcu, NetEcuError};
use embrig_models::load_vehicle_config;
use embrig_sil::{sil_run_codec, SilRegistry};
use fw_canopen_controller::CanOpenControllerFirmware;

fn main() -> anyhow::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let (config, _) = load_vehicle_config(&root.join("canopen/vehicle.yaml"))?;
    let eds = root.join("canopen/eds.yaml");
    let codec = CanOpenCodec::new(&EcuSpec::load(&eds)?)?;

    let mut registry = SilRegistry::new();
    registry.register(
        "controller",
        |name: &str, _budget: u64| -> Result<Box<dyn NetEcu<CanFrame>>, NetEcuError> {
            Ok(Box::new(CanOpenControllerFirmware::new(name)))
        },
    );

    let suites = embrig_test::collect_suites(&root.join("canopen/suites"))?;
    let result = sil_run_codec(&config, Box::new(codec), registry, &suites)?;

    println!("SIL suite: {}", result.file);
    embrig_test::print_suite(&result);
    if result.failed() > 0 {
        std::process::exit(1);
    }
    Ok(())
}
