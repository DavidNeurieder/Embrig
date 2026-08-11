//! Signal codec seam.
//!
//! A codec converts between CAN data bytes and named signal values. The DBC
//! crate implements these traits for its `MessageDef`/`Network` types; other
//! protocols (e.g. the minimal CANopen subset in `embrig-canopen`) provide
//! their own implementations. Keeping the seam in this dependency-free crate
//! lets ECUs, the simulator and the test runner stay protocol-agnostic.

/// A decoded signal value.
#[derive(Debug, Clone, PartialEq)]
pub struct DecodedSignal {
    pub name: String,
    /// Physical value (already scaled by the codec's factor/offset).
    pub value: f64,
    /// Symbolic name if the raw value maps to one (e.g. a `VAL_` table).
    pub symbol: Option<String>,
}

/// Message-level signal codec: the seam between wire bytes and signal names.
///
/// Implementations are stateless views over a message definition, so every
/// method takes `&self`. `Send + Sync` lets codecs be stored inside [`NetEcu`]
/// implementations, which must be `Send`.
///
/// [`NetEcu`]: crate::network::NetEcu
pub trait MessageCodec: Send + Sync {
    /// The CAN id of the message.
    fn id(&self) -> u32;

    /// Encode physical values for a set of named signals into a CAN data field.
    fn encode_signals(&self, values: &[(&str, f64)]) -> Result<Vec<u8>, String>;

    /// Validate that `value` can be encoded for `name` without encoding.
    fn check_value(&self, name: &str, value: f64) -> Result<(), String>;

    /// Resolve a symbolic value (e.g. `OPEN`) to its physical value.
    fn physical_for_symbol(&self, signal: &str, symbol: &str) -> Option<f64>;

    /// Resolve a raw value to its symbolic name, if it has one.
    fn symbol_for(&self, signal: &str, raw: i64) -> Option<String>;

    /// Decode all signals in a CAN data field into physical values.
    fn decode_signals(&self, data: &[u8]) -> Result<Vec<DecodedSignal>, String>;

    /// Decode a single signal.
    fn decode_signal(&self, data: &[u8], name: &str) -> Result<f64, String>;

    /// Clone this codec into an owned box (for ECUs that retain their message).
    fn boxed(&self) -> Box<dyn MessageCodec>;
}

/// Bus-level codec: resolves messages by CAN id (for expectations) or by the
/// name used in `vehicle.yaml` `config` ECU entries.
pub trait SignalCodec: Send + Sync {
    /// Look up a message by CAN id. Borrowed, so it suits the poll loop of an
    /// `expect` step.
    fn message_by_id(&self, id: u32) -> Option<&dyn MessageCodec>;

    /// Look up a message by the name used in `vehicle.yaml` `config` entries.
    fn message_by_name(&self, name: &str) -> Option<&dyn MessageCodec>;

    /// Owned variant of [`message_by_id`](Self::message_by_id), used when
    /// constructing ECUs that retain their message.
    fn owned_message_by_id(&self, id: u32) -> Option<Box<dyn MessageCodec>> {
        self.message_by_id(id).map(|message| message.boxed())
    }

    /// Owned variant of [`message_by_name`](Self::message_by_name).
    fn owned_message_by_name(&self, name: &str) -> Option<Box<dyn MessageCodec>> {
        self.message_by_name(name).map(|message| message.boxed())
    }
}
