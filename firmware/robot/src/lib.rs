//! Motion-controller firmware for the software-in-the-loop robotics example.
//!
//! This is the code under test — a stand-in for the rover's motion-controller
//! firmware. It is the **only** embedded code in the SIL example: it lives in
//! its own crate so it compiles separately from the host test harness
//! (`cargo build -p fw-robot`), and the harness in
//! `embrig-sil/examples/robot_sil.rs` only registers it by name.
//!
//! It implements the [`NetEcu`] trait against the virtual bus:
//!
//! * `on_message` — decodes the joystick speed/steer command on `0x100` and
//!   the e-stop state on `0x110`.
//! * `update` — picks a state with `behaviour()` (priority: e-stop →
//!   over-speed fault → idle → drive), clamps wheel speeds, and transmits the
//!   wheel-speed command on `0x200` and the status word on `0x300` every 50 ms.

use std::sync::OnceLock;

use embrig_core::frame::CanFrame;
use embrig_core::time::Timestamp;
use embrig_core::NetEcu;
use embrig_dbc::Network;

const DBC: &str = include_str!("../../../crates/embrig-sil/examples/robot/robot.dbc");

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
pub struct RobotFirmware {
    name: String,
    speed_cmd: f64,
    steer_cmd: f64,
    estop: bool,
    next_tx: Timestamp,
    period_us: u64,
}

impl RobotFirmware {
    pub fn new(name: &str) -> Self {
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
