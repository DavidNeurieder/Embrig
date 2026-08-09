use std::collections::BTreeMap;

/// Byte order of a DBC signal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ByteOrder {
    /// Little endian (`@1` in DBC).
    Intel,
    /// Big endian (`@0` in DBC).
    Motorola,
}

/// A decoded signal value together with its definition.
#[derive(Debug, Clone, PartialEq)]
pub struct Signal {
    pub name: String,
    /// Physical value (already scaled by factor/offset).
    pub value: f64,
    pub unit: String,
    /// Symbolic name from a `VAL_` table, if the raw value maps to one.
    pub symbol: Option<String>,
}

/// One signal in a message.
#[derive(Debug, Clone, PartialEq)]
pub struct SignalDef {
    pub name: String,
    /// Start bit (Intel: LSB position; Motorola: MSB saw-tooth position).
    pub start_bit: u16,
    pub length: u16,
    pub byte_order: ByteOrder,
    pub is_signed: bool,
    pub factor: f64,
    pub offset: f64,
    pub unit: String,
    pub min: Option<f64>,
    pub max: Option<f64>,
    /// `VAL_` value table: raw value -> symbolic name.
    pub value_table: BTreeMap<i64, String>,
}

/// One CAN message.
#[derive(Debug, Clone, PartialEq)]
pub struct MessageDef {
    pub id: u32,
    pub name: String,
    pub dlc: u8,
    pub signals: Vec<SignalDef>,
}

impl MessageDef {
    pub fn signal(&self, name: &str) -> Option<&SignalDef> {
        self.signals.iter().find(|s| s.name == name)
    }

    /// Resolve a symbolic value against a signal's `VAL_` table.
    pub fn resolve_symbol(&self, signal: &str, symbol: &str) -> Option<i64> {
        self.signal(signal)
            .and_then(|s| s.value_table.iter().find(|(_, n)| n.as_str() == symbol))
            .map(|(v, _)| *v)
    }

    /// Resolve a symbolic value to its physical (scaled) value.
    pub fn physical_for_symbol(&self, signal: &str, symbol: &str) -> Option<f64> {
        let s = self.signal(signal)?;
        let raw = self.resolve_symbol(signal, symbol)?;
        Some(raw as f64 * s.factor + s.offset)
    }

    /// Format a raw value via the signal's `VAL_` table, if present.
    pub fn symbol_for(&self, signal: &str, raw: i64) -> Option<String> {
        self.signal(signal)
            .and_then(|s| s.value_table.get(&raw))
            .cloned()
    }
}

/// A parsed DBC network: a set of messages indexed by CAN id.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Network {
    pub messages: BTreeMap<u32, MessageDef>,
}

impl Network {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn message(&self, id: u32) -> Option<&MessageDef> {
        self.messages.get(&id)
    }
}
