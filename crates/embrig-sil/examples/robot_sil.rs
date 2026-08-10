//! Software-in-the-loop robotics: run the host-compiled motion-controller
//! firmware of a differential-drive rover against the Embrig virtual bus,
//! driven by the exact same YAML suites used for virtual ECUs and hardware.
//!
//! ```text
//! cargo run --example robot_sil --package embrig-sil
//! ```
//!
//! `robot/vehicle.yaml` declares two config nodes — `joystick`
//! (`DriveCommand`) and `e-stop` (`EStop`) — plus a `motion` node with
//! `type: sil`. The firmware for `motion` is a plain [`Ecu`] implementation
//! in this file, registered by name. The suites exercise the rover's
//! fail-safes: drive on command, e-stop halts, e-stop release resumes, and
//! commands above the speed limit are refused.

use std::path::Path;
use std::sync::OnceLock;

use embrig_core::ecu::{Ecu, EcuError};
use embrig_core::frame::CanFrame;
use embrig_core::time::Timestamp;
use embrig_dbc::Network;
use embrig_models::load_vehicle_config;
use embrig_sil::{sil_run, SilRegistry};

const DBC: &str = include_str!("robot/robot.dbc");

fn network() -> &'static Network {
    static NETWORK: OnceLock<Network> = OnceLock::new();
    NETWORK.get_or_init(|| embrig_dbc::parse(DBC).expect("valid robot DBC"))
}

const MAX_SPEED: f64 = 1.5;
const WHEEL_MAX: f64 = 3.0;
const TRACK: f64 = 0.5;

fn clamp_speed(v: f64) -> f64 {
    v.clamp(-WHEEL_MAX, WHEEL_MAX)
}

/// The system under test: maps joystick commands to wheel speeds with an
/// e-stop override and an over-speed fail-safe.
struct RobotFirmware {
    name: String,
    speed_cmd: f64,
    steer_cmd: f64,
    estop: bool,
    next_tx: Timestamp,
    period_us: u64,
}

impl RobotFirmware {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            speed_cmd: 0.0,
            steer_cmd: 0.0,
            estop: false,
            next_tx: 0,
            period_us: 50_000,
        }
    }

    /// Priority order: e-stop, then over-speed fault, then idle, then drive.
    fn behaviour(&self) -> (&'static str, f64, f64) {
        if self.estop {
            ("ESTOP", 0.0, 0.0)
        } else if self.speed_cmd.abs() > MAX_SPEED {
            ("FAULT", 0.0, 0.0)
        } else if self.speed_cmd.abs() < 1e-9 && self.steer_cmd.abs() < 1e-9 {
            ("READY", 0.0, 0.0)
        } else {
            (
                "DRIVING",
                clamp_speed(self.speed_cmd - self.steer_cmd * TRACK),
                clamp_speed(self.speed_cmd + self.steer_cmd * TRACK),
            )
        }
    }
}

impl Ecu for RobotFirmware {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, frame: &CanFrame, _time: Timestamp) {
        match frame.id {
            0x100 => {
                if let Ok(speed) = network()
                    .message(0x100)
                    .unwrap()
                    .decode_signal(&frame.data, "speed")
                {
                    self.speed_cmd = speed;
                }
                if let Ok(steer) = network()
                    .message(0x100)
                    .unwrap()
                    .decode_signal(&frame.data, "steer")
                {
                    self.steer_cmd = steer;
                }
            }
            0x110 => {
                if let Ok(estop) = network()
                    .message(0x110)
                    .unwrap()
                    .decode_signal(&frame.data, "estop")
                {
                    self.estop = estop > 0.5;
                }
            }
            _ => {}
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if time < self.next_tx {
            return;
        }
        let (state, left, right) = self.behaviour();
        let motor = network()
            .message(0x200)
            .unwrap()
            .encode_signals(&[("left_speed", left), ("right_speed", right)])
            .expect("wheel speeds encode");
        let status = network()
            .message(0x300)
            .unwrap()
            .encode_signals(&[(
                "state",
                network()
                    .message(0x300)
                    .unwrap()
                    .physical_for_symbol("state", state)
                    .expect("state symbol exists"),
            )])
            .expect("state encodes");
        out.push(CanFrame::new(0x200, motor).expect("8-byte frame"));
        out.push(CanFrame::new(0x300, status).expect("8-byte frame"));
        self.next_tx = time + self.period_us;
    }
}

fn main() -> anyhow::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/robot");
    let (config, _) = load_vehicle_config(&root.join("vehicle.yaml"))?;
    let dbc = root.join("robot.dbc");

    let mut registry = SilRegistry::new();
    registry.register(
        "motion",
        |name: &str, _budget: u64| -> Result<Box<dyn Ecu>, EcuError> {
            Ok(Box::new(RobotFirmware::new(name)))
        },
    );

    let mut suites = Vec::new();
    for suite in ["drive", "estop", "resume", "overspeed"] {
        suites.push(root.join("suites").join(format!("{suite}.yaml")));
    }
    let result = sil_run(&config, &dbc, registry, &suites)?;

    println!("SIL robotics suite: {}", result.file);
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
