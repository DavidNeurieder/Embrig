//! Drive the Ethernet (UDP) rover over the virtual network.
//!
//! The rover's Ethernet nodes (`joystick`, `motion`) are plain `udp-config`
//! ECUs: they transmit their netmap message on a fixed period, exactly like
//! the built-in virtual CAN ECUs. The same YAML suites that run against the
//! CAN virtual simulation run here unchanged — only the transport differs.
//!
//! ```text
//! cargo run --example udp_rover --package embrig-test
//! ```
//!
//! The fixture lives in `examples/rover/`:
//!
//! * `netmap.yaml` maps message names to fields, byte offsets and the
//!   destination endpoint each message is delivered to.
//! * `vehicle.yaml` declares the Ethernet nodes, the host endpoint and the
//!   network's netmap.
//! * `suites/*.yaml` are the test definitions, keyed by netmap message name.

use std::path::Path;

use embrig_models::load_vehicle_config;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/rover");
    let (config, _) = load_vehicle_config(&root.join("vehicle.yaml"))?;
    let net_config = config
        .networks
        .iter()
        .find(|n| n.kind == "udp")
        .ok_or_else(|| {
            format!(
                "`{}` has no `udp` network",
                root.join("vehicle.yaml").display()
            )
        })?;
    let netmap = root.join(&net_config.netmap);

    let suites = vec![
        root.join("suites/telemetry_reports_stopped.yaml"),
        root.join("suites/speed_override.yaml"),
        root.join("suites/drop_recovers.yaml"),
        root.join("suites/corrupt_recovers.yaml"),
    ];
    let result = embrig_test::udp_run(&config, net_config, &netmap, &suites)?;

    println!("UDP suite: {}", result.file);
    for test in &result.tests {
        let status = if test.passed { "PASS" } else { "FAIL" };
        println!("  [{status}] {} ({} steps)", test.name, test.steps);
        for failure in &test.failures {
            println!("      {failure}");
        }
    }
    let failed = result.failed();
    println!("{} passed, {failed} failed", result.tests.len() - failed);
    if failed > 0 {
        std::process::exit(1);
    }
    Ok(())
}
