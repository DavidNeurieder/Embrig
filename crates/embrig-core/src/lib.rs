//! Deterministic simulation core for Embrig.
//!
//! This crate deliberately has **no async runtime and no external
//! dependencies**. It provides the types and engine used by both the virtual
//! simulator and, indirectly, the hardware backends: frames, a monotonic
//! microsecond clock, the [`Ecu`] trait, fault injection and the
//! [`Simulation`] loop.
//!
//! Determinism is a hard requirement: ECUs are stepped in insertion order,
//! timestamps are integers, and there is no wall-clock dependence anywhere in
//! this crate.

pub mod ecu;
pub mod fault;
pub mod frame;
pub mod recorder;
pub mod signal;
pub mod simulation;
pub mod time;

pub use ecu::{Ecu, EcuError};
pub use fault::{Fault, FaultRule};
pub use frame::{CanFrame, FrameError};
pub use recorder::{Record, Recorder};
pub use signal::SignalValue;
pub use simulation::Simulation;
pub use time::Timestamp;
