//! YAML test definitions and the Embrig test runner.
//!
//! A test file describes a sequence of steps: sending frames, overriding
//! signals, waiting, injecting faults and asserting on received signals.
//! The same definition runs against either the deterministic virtual
//! simulation or a real SocketCAN interface (`--features socketcan`).
//!
//! ```yaml
//! name: overvoltage_disables_motor
//! timeout: 5s
//! steps:
//!   - wait: { time: 100ms }
//!   - set_signal: { ecu: battery, id: 0x100, signal: voltage, value: 460.0 }
//!   - expect: { id: 0x220, signal: motor_enable, equals: false, within: 1s }
//!   - fault: { type: drop, id: 0x100, duration: 100ms }
//!   - expect: { id: 0x230, signal: state, equals: "SAFE", within: 1s }
//! ```

pub mod dsl;
pub mod report;
pub mod runner;
pub mod target;

pub use dsl::{load_spec, parse_duration, ExpectStep, ExpectedValue, FaultKind, Step, TestSpec};
pub use report::{html, json, load_json, write_report, SuiteResult, TestResult};
pub use runner::{run_spec, run_suite, TestError};
pub use target::{TargetError, TestTarget, VirtualTarget};
