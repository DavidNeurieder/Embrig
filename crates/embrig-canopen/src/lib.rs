//! Minimal CANopen (CiA 301) subset behind the [`embrig-core`] codec seam.
//!
//! This crate speaks a small, hand-rolled slice of CANopen — enough for a
//! deterministic SIL demo without pulling in a third-party protocol stack:
//!
//! * **COB-IDs**: TPDO1 `0x180+n`, RPDO1 `0x200+n`, heartbeat `0x700+n` and
//!   NMT `0x000` (see [`tpdo1`], [`rpdo1`], [`heartbeat`], [`NMT`]).
//! * **PDOs** are packed with the existing DBC bit-packer: a PDO mapping is
//!   just a DBC-style message whose signals are the mapped object-dictionary
//!   entries ([`CanOpenCodec`] builds them from an [`EcuSpec`]).
//! * **Heartbeat** (1 byte: boot-up / stopped / operational / pre-operational)
//!   and **NMT** (2 bytes: node-id + command) are small bespoke codecs.
//!
//! [`CanOpenCodec`] implements [`SignalCodec`], so the same `vehicle.yaml`
//! shape and YAML suites drive a CANopen firmware exactly like a DBC one: the
//! config nodes send RPDO/NMT frames, `set_signal` overrides an RPDO signal,
//! and `expect` decodes TPDO/heartbeat frames by COB-ID and signal name.

pub mod codec;
pub mod spec;

pub use codec::{CanOpenCodec, CanOpenError, HeartbeatMessage, NmtMessage};
pub use spec::{EcuSpec, SignalSpec};

pub use embrig_core::codec::SignalCodec;

/// NMT service COB-ID (all nodes).
pub const NMT: u32 = 0x000;
/// PDO1 process data — transmit base (`0x180 + node`).
pub const TPDO1_COB: u32 = 0x180;
/// PDO1 process data — receive base (`0x200 + node`).
pub const RPDO1_COB: u32 = 0x200;
/// Heartbeat producer COB-ID base (`0x700 + node`).
pub const HEARTBEAT_COB: u32 = 0x700;

/// TPDO1 COB-ID for `node` (`0x180 + node`).
pub fn tpdo1(node: u8) -> u32 {
    TPDO1_COB + node as u32
}

/// RPDO1 COB-ID for `node` (`0x200 + node`).
pub fn rpdo1(node: u8) -> u32 {
    RPDO1_COB + node as u32
}

/// Heartbeat COB-ID for `node` (`0x700 + node`).
pub fn heartbeat(node: u8) -> u32 {
    HEARTBEAT_COB + node as u32
}

/// NMT COB-ID (always `0x000`, all nodes).
pub fn nmt() -> u32 {
    NMT
}

/// NMT command codes (CiA 301 §NMT master commands).
pub const NMT_START: u8 = 0x01;
pub const NMT_STOP: u8 = 0x02;
pub const NMT_PRE_OPERATIONAL: u8 = 0x80;

/// Heartbeat / NMT state bytes (CiA 301 §NMT states).
pub const HB_BOOT_UP: u8 = 0x00;
pub const HB_STOPPED: u8 = 0x04;
pub const HB_OPERATIONAL: u8 = 0x05;
pub const HB_PRE_OPERATIONAL: u8 = 0x7F;
