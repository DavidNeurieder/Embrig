//! Software-in-the-loop robotics: run the host-compiled motion-controller
//! firmware of a differential-drive rover against the Embrig virtual bus,
//! driven by the exact same YAML suites used for virtual ECUs and hardware.
//!
//! ## The embedded code
//!
//! `RobotFirmware` below is the code under test — a stand-in for the rover's
//! motion-controller firmware. It implements the [`NetEcu`] trait:
//!
//! * `on_message` — decodes the joystick speed/steer command on `0x100` and
//!   the e-stop state on `0x110`.
//! * `update` — picks a state with `behaviour()` (priority: e-stop →
//!   over-speed fault → idle → drive), clamps wheel speeds, and transmits the
//!   wheel-speed command on `0x200` and the status word on `0x300` every 50 ms.
//!
//! This is the only part that is "the device"; everything else in this file is
//! host test harness.
//!
//! ## The host harness
//!
//! `main` is test infrastructure, not embedded code: it loads
//! `robot/vehicle.yaml` + `robot/robot.dbc`, binds the `motion` node to a
//! fresh `RobotFirmware` through a [`SilRegistry`], and runs the YAML suites in
//! `robot/suites` with [`sil_run`]. The firmware factory is re-invoked before
//! every test, so firmware state never leaks between tests.
//!
//! ## Supporting files
//!
//! * `robot/vehicle.yaml` — the `joystick` and `e-stop` config nodes (stimulus
//!   sources, overridden by `set_signal`) plus the `motion` node with
//!   `type: sil` (firmware is code, not config).
//! * `robot/robot.dbc` — the message map: `DriveCommand` (`0x100`), `EStop`
//!   (`0x110`), `MotorCommand` (`0x200`) and `RobotStatus` (`0x300`).
//! * `robot/suites/` — the four tests: drive on joystick command, e-stop
//!   halt, e-stop-release resume, over-speed refusal.
//! * `robot/suites_hil/` — send-based HIL twins of the same scenarios, for a
//!   real rover on a real CAN bus.
//!
//! ## How to run
//!
//! ```text
//! cargo run --example robot_sil --package embrig-sil
//! ```
//!
//! → `4 passed, 0 failed`. Each simulated firmware step runs under a wall-clock
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

impl NetEcu<CanFrame> for RobotFirmware {
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
        |name: &str, _budget: u64| -> Result<Box<dyn NetEcu<CanFrame>>, NetEcuError> {
            Ok(Box::new(RobotFirmware::new(name)))
        },
    );

    let suites = embrig_test::collect_suites(&root.join("suites"))?;
    let result = sil_run(&config, &dbc, registry, &suites)?;

    println!("SIL robotics suite: {}", result.file);
    embrig_test::print_suite(&result);
    if result.failed() > 0 {
        std::process::exit(1);
    }
    Ok(())
}
