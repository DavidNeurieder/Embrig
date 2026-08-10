//! DBC parsing and signal codec for Embrig.
//!
//! Supports the subset of the Vector DBC format needed for simulation:
//! `BO_` messages, `SG_` signals (Intel and Motorola byte order, signed and
//! unsigned, factor/offset scaling) and `VAL_` value tables (used to render
//! readable state names in reports).

mod codec;
mod parser;
mod types;

pub use codec::CodecError;
pub use parser::{parse, ParseError};
pub use types::{ByteOrder, MessageDef, Network, Signal, SignalDef};
