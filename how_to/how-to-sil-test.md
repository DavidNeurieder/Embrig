# How to run a SIL test on your existing firmware

Software-in-the-loop (SIL) runs your **actual firmware, compiled for the host**,
against the deterministic Embrig virtual bus. The same YAML suites you use for
virtual ECUs drive your real control logic — without a CAN adapter or a target
board. Once the firmware is wrapped, the identical suites can later run against
real hardware ([HIL](how-to-hil-test.md)).

This guide assumes your firmware already exists in **Rust**. The only "porting"
required is wrapping it in the `Ecu` trait so the simulator can step it.

---

## 1. Prerequisites

- The Embrig workspace checked out.
- A Rust toolchain (stable).
- `embrig-sil` is a workspace member — no extra install.

The reference pattern lives in the bundled examples and is worth reading
alongside this guide:

- `crates/embrig-sil/examples/robot_sil.rs` + `examples/robot/` (full walkthrough)
- `crates/embrig-sil/examples/sil_firmware.rs` + `examples/fixtures/` (minimal)

## 2. Describe the network: DBC + `vehicle.yaml`

### 2a. The DBC file

Embrig needs the message map of the bus your firmware talks on: every message
and signal you want to *inject* or *assert* must be in the DBC. Your DBC is
typically the one used to generate the firmware's CAN stack — reuse it directly:

```text
BO_ 256 DriveCommand: 8 Vector__XXX
 SG_ speed : 0|16@1- (0.01,0) [-5|5] "m/s"  Vector__XXX
 SG_ steer : 16|16@1- (0.01,0) [-5|5] ""  Vector__XXX

BO_ 512 MotorCommand: 8 Vector__XXX
 SG_ left_speed : 0|16@1- (0.01,0) [-3|3] "m/s"  Vector__XXX
 SG_ right_speed : 16|16@1- (0.01,0) [-3|3] "m/s"  Vector__XXX

BO_ 768 RobotStatus: 8 Vector__XXX
 SG_ state : 0|8@1+ (1,0) [0|4] ""  Vector__XXX
 VAL_ 768 state 0 "OFF" 1 "READY" 2 "DRIVING" 3 "ESTOP" 4 "FAULT" ;
```

### 2b. `vehicle.yaml`

Declare each node on the bus. Two kinds matter for SIL:

- **Config nodes** — stimulus sources you drive from the tests. No code needed;
  the test runner overrides their signals.
- **The firmware node** — `type: sil`. Its behaviour is your Rust firmware,
  registered in code (step 4).

```yaml
name: diff-drive-rover
dbc: robot.dbc
step_us: 1000

ecus:
  # Stimulus: the tests override these signals.
  - name: joystick
    type: config
    message: DriveCommand
    period_us: 100000
    signals:
      speed: 0.0
      steer: 0.0

  # The ECU under test: firmware compiled for the host.
  - name: motion
    type: sil
    period_us: 50000          # how often the firmware's `update` runs
    listen: [0x100, 0x110]    # ids delivered to its `on_message`
    step_budget_us: 100000    # wall-clock budget per step (default 100 ms)

interfaces:
  - name: virtual
    type: virtual
  - name: sil
    type: sil
```

The `name:` of the `type: sil` node is the key you register firmware under —
keep it in sync with the registry in step 4.

## 3. Wrap your firmware in the `Ecu` trait

`Ecu` is deliberately small (`crates/embrig-core/src/ecu.rs`): only `name()` is
mandatory — `update`, `on_message` and `set_signal` all have defaults. You
implement two callbacks:

- `on_message(&mut self, frame, time)` — handle a frame whose id is in `listen`.
- `update(&mut self, time, out)` — advance state, push outgoing frames onto `out`.

Decode inputs and encode outputs through the parsed DBC network (cache it in a
`OnceLock` — this is the pattern the examples use):

```rust
use std::sync::OnceLock;
use embrig_core::ecu::{Ecu, EcuError};
use embrig_core::frame::CanFrame;
use embrig_core::time::Timestamp;
use embrig_dbc::Network;

const DBC: &str = include_str!("robot/robot.dbc");

fn network() -> &'static Network {
    static NETWORK: OnceLock<Network> = OnceLock::new();
    NETWORK.get_or_init(|| embrig_dbc::parse(DBC).expect("valid DBC"))
}

/// Wrap your existing motion-controller logic.
struct MotionFirmware {
    name: String,
    speed_cmd: f64,
    next_tx: Timestamp,
}

impl MotionFirmware {
    fn new(name: &str) -> Self {
        Self { name: name.to_string(), speed_cmd: 0.0, next_tx: 0 }
    }

    /// Your real control law — keep this identical to the firmware on the target.
    fn behaviour(&self) -> (f64, f64) {
        let left = self.speed_cmd.clamp(-3.0, 3.0);
        (left, left)
    }
}

impl Ecu for MotionFirmware {
    fn name(&self) -> &str {
        &self.name
    }

    fn on_message(&mut self, frame: &CanFrame, _time: Timestamp) {
        if frame.id == 0x100 {
            // decode_signal applies factor/offset; returns the physical value.
            self.speed_cmd = network()
                .message(0x100)
                .unwrap()
                .decode_signal(&frame.data, "speed")
                .unwrap_or(self.speed_cmd);
        }
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if time < self.next_tx {
            return;
        }
        let (left, right) = self.behaviour();
        let data = network()
            .message(0x200)
            .unwrap()
            .encode_signals(&[("left_speed", left), ("right_speed", right)])
            .expect("wheel speeds encode");
        out.push(CanFrame::new(0x200, data).expect("8-byte frame"));
        self.next_tx = time + 50_000;
    }
}
```

Notes on matching your real firmware:

- **`on_message`/`update` timing** — `update` runs every `period_us`; the
  simulator calls it in order each tick. Model your real CAN-priority / polling
  behaviour here so the host port behaves like the target.
- **Symbols** — for enumerated signals use `physical_for_symbol` (as
  `robot_sil.rs` does for `state`), so `encode_signals` accepts `"DRIVING"`
  rather than the raw integer.
- **`set_signal`** — you do *not* implement it. Tests are not allowed to poke
  the firmware's state; they drive it through the bus. `set_signal` on a SIL
  node fails with `UnsupportedOnSut` at the target level.

## 4. Write the test suites

Suites are plain YAML, one file per test, run in sorted order. Steps:
`wait`, `send`, `set_signal`, `fault`, `expect`. Assertions decode real signals
through the DBC — never raw bytes:

```yaml
name: joystick_command_drives_the_rover
timeout: 5s
steps:
  - wait: { time: 300ms }
  - set_signal: { ecu: joystick, id: 0x100, signal: speed, value: 0.8 }
  - expect: { id: 0x200, signal: left_speed, equals: 0.8, within: 1s }
  - expect: { id: 0x200, signal: right_speed, equals: 0.8, within: 1s }
  - expect: { id: 0x300, signal: state, equals: "DRIVING", within: 1s }
```

Rules specific to SIL:

- `set_signal` targets **config nodes only**. Overriding the firmware itself is
  rejected — drive it via the bus instead (this is the point).
- `send` injects a raw frame onto the virtual bus; `fault` (drop/delay/corrupt)
  works exactly as in virtual mode.
- `expect` polls for up to `within`; without `within` it checks the current bus
  state once.

## 5. Register the firmware and run

Bind the vehicle.yaml node name to a factory and call `sil_run` — it builds the
simulation and a tokio runtime for you:

```rust
use std::path::Path;
use embrig_core::ecu::EcuError;
use embrig_models::load_vehicle_config;
use embrig_sil::{sil_run, SilRegistry};

fn main() -> anyhow::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/robot");
    let (config, _) = load_vehicle_config(&root.join("vehicle.yaml"))?;
    let dbc = root.join("robot.dbc");

    let mut registry = SilRegistry::new();
    registry.register(
        "motion", // must match the `type: sil` node name in vehicle.yaml
        |name: &str, _budget: u64| -> Result<Box<dyn Ecu>, EcuError> {
            Ok(Box::new(MotionFirmware::new(name)))
        },
    );

    let mut suites = Vec::new();
    for suite in ["drive", "estop", "resume", "overspeed"] {
        suites.push(root.join("suites").join(format!("{suite}.yaml")));
    }

    let result = sil_run(&config, &dbc, registry, &suites)?;
    for test in &result.tests {
        println!("{}  {}  ({} steps)", if test.passed { "PASS" } else { "FAIL" }, test.name, test.steps);
        for failure in &test.failures {
            println!("       {failure}");
        }
    }
    if result.failed() > 0 {
        std::process::exit(1);
    }
    Ok(())
}
```

The factory closure receives `(name, step_budget_us)` and returns a fresh
firmware instance — it is re-invoked **before every test**, so firmware state
never leaks between tests.

Run it (as a Cargo example or your own binary that depends on `embrig-sil`):

```sh
cargo run --example robot_sil --package embrig-sil
```

→ `4 passed, 0 failed`

## 6. Interpret the results and tune

- **Output** — one `PASS`/`FAIL` line per test with its failure messages, then a
  total. Exit code is non-zero on any failure (use it in CI).
- **`SutTimeout`** — a step exceeded its budget (default 100 ms wall-clock). The
  message shows how long it actually took. If the failure message names your
  firmware, raise `step_budget_us` on the node (slow CI machines) or speed the
  firmware up. A budget overrun fails the test cleanly instead of hanging it.
- **`UnsupportedOnSut`** — a test tried to `set_signal` the firmware. Drive it
  through the bus.
- **`NotRegistered`** — a `type: sil` node has no matching factory. Check the
  registry key matches the vehicle.yaml name (the error lists what is
  registered).

## Caveats

- **Host-compiled Rust only.** Embrig SIL does not run your target binary or
  cross-compiled C/C++ firmware. If your firmware is C/C++, port the control
  logic into the `Ecu` implementation and review it against the real source —
  the suites then *prove* the port, and HIL verifies the port against the real
  firmware.
- **Wall-clock budget, simulated time.** The bus clock is deterministic
  integer microseconds, but the budget is measured in real time. Perfectly
  deterministic behaviour, with a time ceiling to catch runaway steps.
- **Deterministic.** ECUs step in config order on the integer-µs clock — a
  PASS→FAIL flip is caused by your change, never by the runner.

## Next: hardware

The same YAML suites run against a real bus once your tests use `send` steps
instead of `set_signal` — see [how-to-hil-test.md](how-to-hil-test.md).
