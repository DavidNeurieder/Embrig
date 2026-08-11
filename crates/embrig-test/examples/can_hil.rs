//! Drive the EV powertrain over a real SocketCAN interface (hardware-in-the-loop).
//!
//! This is the hardware twin of the `virtual` CAN target: the same YAML suites
//! run unchanged, but frames go out on a real (or `vcan`) interface and
//! expectations are matched against what comes back off the bus. `set_signal`
//! and faults are rejected — there is no software router in the loop.
//!
//! ```text
//! # vcan0, no arguments needed (runs the loopback smoke test)
//! cargo run --example can_hil --package embrig-test --features socketcan
//!
//! # a specific interface / vehicle / tests
//! cargo run --example can_hil --package embrig-test --features socketcan \
//!     -- vcan0 path/to/vehicle.yaml path/to/tests/
//! ```
//!
//! Arguments (all optional): `INTERFACE VEHICLE [TEST...]`. The interface is
//! the *name* declared in `vehicle.yaml` (defaults to its first `socketcan`
//! interface). Without a vehicle it uses the `ev-powertrain` fixture; without
//! tests it uses the bundled loopback spec — send `0x7FF`, expect it back —
//! because virtual-only suites (`set_signal`, faults, config nodes) do not run
//! on hardware. Pass your own send-based suites to exercise a real ECU.
//!
//! Bring up a loopback-capable virtual bus first:
//!
//! ```text
//! sudo scripts/vcan-up.sh
//! ```

#[cfg(feature = "socketcan")]
use std::path::{Path, PathBuf};

#[cfg(feature = "socketcan")]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    use embrig_models::load_vehicle_config;
    use embrig_test::{ProtocolInput, ProtocolRegistry};

    let args: Vec<String> = std::env::args().skip(1).collect();
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/ev-powertrain");

    let vehicle = args
        .get(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("vehicle.yaml"));
    let (config, dbc_path) = load_vehicle_config(&vehicle)?;

    let interface = args.first().cloned().unwrap_or_else(|| {
        config
            .interfaces
            .iter()
            .find(|i| i.kind == "socketcan")
            .map(|i| i.name.clone())
            .unwrap_or_else(|| "vcan0".to_string())
    });

    let vehicle_dir = vehicle
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let test_inputs: Vec<String> = args.iter().skip(2).cloned().collect();
    let loopback = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scripts/loopback.yaml");
    let tests = if test_inputs.is_empty() {
        vec![loopback]
    } else {
        let mut out = Vec::new();
        for input in &test_inputs {
            let path = if Path::new(input).exists() {
                PathBuf::from(input)
            } else {
                vehicle_dir.join(input)
            };
            if path.is_dir() {
                out.extend(embrig_test::collect_suites(&path)?);
            } else {
                out.push(path);
            }
        }
        out
    };
    if tests.is_empty() {
        return Err("no YAML test files found".into());
    }

    println!(
        "HIL (socketcan) interface `{interface}`, {} test file(s) against {}",
        tests.len(),
        vehicle.display()
    );

    let input = ProtocolInput {
        config: &config,
        dbc_path: &dbc_path,
        vehicle_dir,
        interface: Some(&interface),
    };
    let registry = ProtocolRegistry::default();
    let mut target = match registry.build("socketcan", &input) {
        Ok(target) => target,
        Err(e) => {
            return Err(format!(
                "cannot open CAN interface `{interface}`: {e}\n\
                 create a virtual bus with `sudo scripts/vcan-up.sh`, or pass a \
                 real interface name (`can_hil <iface> [vehicle] [tests...]`)"
            )
            .into())
        }
    };

    let suite =
        embrig_test::run_suite(&mut *target, &tests, &vehicle.display().to_string()).await?;
    embrig_test::print_suite(&suite);
    if suite.failed() > 0 {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(not(feature = "socketcan"))]
fn main() {
    eprintln!("this example needs the `socketcan` feature:");
    eprintln!("  cargo run --example can_hil --package embrig-test --features socketcan");
    std::process::exit(2);
}
