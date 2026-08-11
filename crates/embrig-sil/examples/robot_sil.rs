//! Software-in-the-loop robotics: run the host-compiled motion-controller
//! firmware of a differential-drive rover against the Embrig virtual bus,
//! driven by the exact same YAML suites used for virtual ECUs and hardware.
//!
//! ## The embedded code
//!
//! The system under test is [`RobotFirmware`] — a stand-in for the rover's
//! motion-controller firmware. It is **not** in this file: it lives in its own
//! crate, `firmware/robot` (`fw-robot`), so it compiles separately from the
//! test harness (`cargo build -p fw-robot`) and can be replaced by your real
//! firmware. It implements the [`NetEcu`] trait:
//!
//! * `on_message` — decodes the joystick speed/steer command on `0x100` and
//!   the e-stop state on `0x110`.
//! * `update` — picks a state with `behaviour()` (priority: e-stop →
//!   over-speed fault → idle → drive), clamps wheel speeds, and transmits the
//!   wheel-speed command on `0x200` and the status word on `0x300` every 50 ms.
//!
//! This file is host test harness only.
//!
//! ## The host harness
//!
//! `main` loads `robot/vehicle.yaml` + `robot/robot.dbc`, binds the `motion`
//! node to a fresh [`RobotFirmware`] through a [`SilRegistry`], and runs the
//! YAML suites in `robot/suites` with [`sil_run`]. The firmware factory is
//! re-invoked before every test, so firmware state never leaks between tests.
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

use embrig_core::frame::CanFrame;
use embrig_core::{NetEcu, NetEcuError};
use embrig_models::load_vehicle_config;
use embrig_sil::{sil_run, SilRegistry};
use fw_robot::RobotFirmware;

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
