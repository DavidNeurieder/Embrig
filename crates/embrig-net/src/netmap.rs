//! The Ethernet message map ("netmap") and field codec.
//!
//! A netmap describes the UDP traffic of a network: every message is
//! identified by its destination endpoint (`dst`) and carries named fields at
//! fixed byte offsets. This mirrors how a DBC describes CAN messages, but for
//! arbitrary-length payloads with scalar field types instead of bit-packed
//! signals.

use std::collections::BTreeMap;
use std::net::SocketAddr;

use embrig_core::signal::SignalValue;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Errors produced while encoding or decoding fields.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum FieldError {
    #[error("field `{0}` at offset {1} does not fit in a {2}-byte payload")]
    OutOfBounds(String, usize, usize),
    #[error("unknown field `{0}` on this message")]
    UnknownField(String),
    #[error("field `{name}` expects a numeric value, got `{value}`")]
    SymbolicValue { name: String, value: String },
    #[error("unknown symbol `{symbol}` for field `{field}`")]
    UnknownSymbol { field: String, symbol: String },
    #[error("value {0} is out of range for field type {1}")]
    ValueOutOfRange(f64, FieldType),
}

/// A fixed-size scalar field type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FieldType {
    U8,
    U16le,
    U16be,
    U32le,
    U32be,
    I16le,
    I32le,
    F32le,
    F64le,
    Bool,
}

impl FieldType {
    /// Size of this field in bytes.
    pub fn size(&self) -> usize {
        match self {
            FieldType::U8 | FieldType::Bool => 1,
            FieldType::U16le | FieldType::U16be | FieldType::I16le => 2,
            FieldType::U32le | FieldType::U32be | FieldType::I32le | FieldType::F32le => 4,
            FieldType::F64le => 8,
        }
    }
}

impl std::fmt::Display for FieldType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            FieldType::U8 => "u8",
            FieldType::U16le => "u16le",
            FieldType::U16be => "u16be",
            FieldType::U32le => "u32le",
            FieldType::U32be => "u32be",
            FieldType::I16le => "i16le",
            FieldType::I32le => "i32le",
            FieldType::F32le => "f32le",
            FieldType::F64le => "f64le",
            FieldType::Bool => "bool",
        };
        write!(f, "{name}")
    }
}

/// One field in a message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FieldDef {
    /// Byte offset of the field within the payload.
    pub offset: usize,
    #[serde(rename = "type")]
    pub ty: FieldType,
    /// Scaling factor: physical = raw × factor + shift.
    #[serde(default = "default_factor")]
    pub factor: f64,
    /// Scaling shift: physical = raw × factor + shift.
    #[serde(default)]
    pub shift: f64,
    /// Symbol table: raw integer -> symbolic name.
    #[serde(default)]
    pub values: BTreeMap<i64, String>,
}

fn default_factor() -> f64 {
    1.0
}

/// One UDP message: a fixed payload shape delivered to `dst`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MessageDef {
    /// Destination endpoint that identifies this message.
    pub dst: SocketAddr,
    /// Full payload length in bytes (payloads are zero-padded to this).
    pub length: usize,
    #[serde(default)]
    pub fields: BTreeMap<String, FieldDef>,
}

/// A decoded field value together with its definition.
#[derive(Debug, Clone, PartialEq)]
pub struct DecodedField {
    pub name: String,
    /// Physical value (already scaled by factor/shift).
    pub value: f64,
    /// Symbolic name from the field's `values` table, if the raw value maps to one.
    pub symbol: Option<String>,
}

/// The message map of one Ethernet network.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Netmap {
    #[serde(default)]
    pub messages: BTreeMap<String, MessageDef>,
}

impl Netmap {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn message(&self, name: &str) -> Option<&MessageDef> {
        self.messages.get(name)
    }

    /// The destination endpoint of a message, if any.
    pub fn message_dst(&self, name: &str) -> Option<SocketAddr> {
        self.messages.get(name).map(|m| m.dst)
    }
}

impl MessageDef {
    pub fn field(&self, name: &str) -> Option<&FieldDef> {
        self.fields.get(name)
    }

    /// Resolve a symbolic value against a field's `values` table.
    pub fn resolve_symbol(&self, field: &str, symbol: &str) -> Option<i64> {
        self.field(field)
            .and_then(|f| f.values.iter().find(|(_, n)| n.as_str() == symbol))
            .map(|(v, _)| *v)
    }

    /// Resolve a symbolic value to its physical (scaled) value.
    pub fn physical_for_symbol(&self, field: &str, symbol: &str) -> Option<f64> {
        let f = self.field(field)?;
        let raw = self.resolve_symbol(field, symbol)?;
        Some(raw as f64 * f.factor + f.shift)
    }

    /// Encode physical/symbolic values for a set of named fields into a payload.
    ///
    /// Values may be numeric (physical, scaled by factor/shift) or symbolic
    /// (matched against the field's `values` table). A field may be omitted
    /// (encoded as zero) or overridden by a later entry.
    pub fn encode_fields(&self, values: &[(&str, SignalValue)]) -> Result<Vec<u8>, FieldError> {
        let mut payload = vec![0u8; self.length];
        let resolved = self.resolve_all(values)?;
        for (name, raw) in resolved {
            let field = self
                .field(&name)
                .expect("resolve_all only returns known fields");
            self.write_field(field, raw, &mut payload)?;
        }
        Ok(payload)
    }

    /// Decode a single field from a payload.
    pub fn decode_field(&self, payload: &[u8], name: &str) -> Result<DecodedField, FieldError> {
        let Some(field) = self.field(name) else {
            return Err(FieldError::UnknownField(name.to_string()));
        };
        let raw = self.read_field(field, payload)?;
        let symbol = field.values.get(&raw).cloned();
        Ok(DecodedField {
            name: name.to_string(),
            value: raw_to_physical(field, raw),
            symbol,
        })
    }

    fn resolve_all(
        &self,
        values: &[(&str, SignalValue)],
    ) -> Result<BTreeMap<String, i64>, FieldError> {
        let mut map = BTreeMap::new();
        for (name, value) in values {
            let Some(field) = self.field(name) else {
                continue;
            };
            let raw = match value {
                SignalValue::Num(v) => physical_to_raw(field, *v)?,
                SignalValue::Str(s) => {
                    self.resolve_symbol(name, s)
                        .ok_or_else(|| FieldError::UnknownSymbol {
                            field: name.to_string(),
                            symbol: s.clone(),
                        })?
                }
            };
            map.insert(name.to_string(), raw);
        }
        Ok(map)
    }

    fn write_field(
        &self,
        field: &FieldDef,
        raw: i64,
        payload: &mut [u8],
    ) -> Result<(), FieldError> {
        let size = field.ty.size();
        let end = field.offset.saturating_add(size);
        if end > payload.len() {
            return Err(FieldError::OutOfBounds(
                field_placeholder_name(field),
                field.offset,
                payload.len(),
            ));
        }
        let bytes = raw_bytes(field.ty, raw);
        for (i, b) in bytes.iter().enumerate() {
            payload[field.offset + i] = *b;
        }
        Ok(())
    }

    fn read_field(&self, field: &FieldDef, payload: &[u8]) -> Result<i64, FieldError> {
        let size = field.ty.size();
        let end = field.offset.saturating_add(size);
        if end > payload.len() {
            return Err(FieldError::OutOfBounds(
                field_placeholder_name(field),
                field.offset,
                payload.len(),
            ));
        }
        Ok(raw_int(field.ty, &payload[field.offset..end]))
    }
}

fn field_placeholder_name(_field: &FieldDef) -> String {
    String::new()
}

/// Convert a raw integer to its bytes for `ty` (little- or big-endian). For
/// float fields `raw` holds the IEEE-754 bit pattern, not a rounded value.
fn raw_bytes(ty: FieldType, raw: i64) -> Vec<u8> {
    match ty {
        FieldType::U8 => vec![raw as u8],
        FieldType::Bool => vec![if raw != 0 { 1 } else { 0 }],
        FieldType::U16le => (raw as u16).to_le_bytes().to_vec(),
        FieldType::U16be => (raw as u16).to_be_bytes().to_vec(),
        FieldType::U32le => (raw as u32).to_le_bytes().to_vec(),
        FieldType::U32be => (raw as u32).to_be_bytes().to_vec(),
        FieldType::I16le => (raw as i16).to_le_bytes().to_vec(),
        FieldType::I32le => (raw as i32).to_le_bytes().to_vec(),
        FieldType::F32le => (raw as u32).to_le_bytes().to_vec(),
        FieldType::F64le => (raw as u64).to_le_bytes().to_vec(),
    }
}

/// Read a raw integer from bytes for `ty`. For float fields this is the
/// IEEE-754 bit pattern.
fn raw_int(ty: FieldType, bytes: &[u8]) -> i64 {
    match ty {
        FieldType::U8 => bytes[0] as i64,
        FieldType::Bool => (bytes[0] != 0) as i64,
        FieldType::U16le => i64::from(u16::from_le_bytes([bytes[0], bytes[1]])),
        FieldType::U16be => i64::from(u16::from_be_bytes([bytes[0], bytes[1]])),
        FieldType::U32le => i64::from(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])),
        FieldType::U32be => i64::from(u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])),
        FieldType::I16le => i64::from(i16::from_le_bytes([bytes[0], bytes[1]])),
        FieldType::I32le => i64::from(i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])),
        FieldType::F32le => i64::from(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])),
        FieldType::F64le => {
            let bits = u64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ]);
            bits as i64
        }
    }
}

/// Convert a physical value to the raw integer stored in the payload.
fn physical_to_raw(field: &FieldDef, value: f64) -> Result<i64, FieldError> {
    let scaled = (value - field.shift) / field.factor;
    if scaled.is_nan() || scaled.is_infinite() {
        return Err(FieldError::ValueOutOfRange(value, field.ty));
    }
    match field.ty {
        FieldType::F32le => {
            let f = scaled as f32;
            if f.is_nan() || f.is_infinite() {
                return Err(FieldError::ValueOutOfRange(value, field.ty));
            }
            Ok(f.to_bits() as i64)
        }
        FieldType::F64le => Ok(scaled.to_bits() as i64),
        _ => {
            let raw = scaled.round() as i64;
            if !fits_type(field.ty, raw) {
                return Err(FieldError::ValueOutOfRange(value, field.ty));
            }
            Ok(raw)
        }
    }
}

/// Convert a raw integer back to a physical value.
fn raw_to_physical(field: &FieldDef, raw: i64) -> f64 {
    let raw_value = match field.ty {
        FieldType::F32le => f32::from_bits(raw as u32) as f64,
        FieldType::F64le => f64::from_bits(raw as u64),
        _ => raw as f64,
    };
    raw_value * field.factor + field.shift
}

fn fits_type(ty: FieldType, value: i64) -> bool {
    match ty {
        FieldType::U8 => (0..=u8::MAX as i64).contains(&value),
        FieldType::Bool => value == 0 || value == 1,
        FieldType::U16le | FieldType::U16be => (0..=u16::MAX as i64).contains(&value),
        FieldType::U32le | FieldType::U32be => (0..=u32::MAX as i64).contains(&value),
        FieldType::I16le => (i16::MIN as i64..=i16::MAX as i64).contains(&value),
        FieldType::I32le => (i32::MIN as i64..=i32::MAX as i64).contains(&value),
        FieldType::F32le | FieldType::F64le => true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message() -> MessageDef {
        MessageDef {
            dst: "192.168.1.10:5000".parse().unwrap(),
            length: 12,
            fields: BTreeMap::from([
                (
                    "state".to_string(),
                    FieldDef {
                        offset: 0,
                        ty: FieldType::U8,
                        factor: 1.0,
                        shift: 0.0,
                        values: BTreeMap::from([(0, "OFF".into()), (1, "READY".into())]),
                    },
                ),
                (
                    "speed".to_string(),
                    FieldDef {
                        offset: 4,
                        ty: FieldType::F32le,
                        factor: 1.0,
                        shift: 0.0,
                        values: BTreeMap::new(),
                    },
                ),
                (
                    "counter".to_string(),
                    FieldDef {
                        offset: 8,
                        ty: FieldType::U16le,
                        factor: 1.0,
                        shift: 0.0,
                        values: BTreeMap::new(),
                    },
                ),
            ]),
        }
    }

    #[test]
    fn field_type_sizes() {
        assert_eq!(FieldType::Bool.size(), 1);
        assert_eq!(FieldType::U16be.size(), 2);
        assert_eq!(FieldType::F32le.size(), 4);
        assert_eq!(FieldType::F64le.size(), 8);
    }

    #[test]
    fn encode_decode_round_trip() {
        let m = message();
        let payload = m
            .encode_fields(&[
                ("state", SignalValue::Str("READY".into())),
                ("speed", 1.5f64.into()),
                ("counter", 42f64.into()),
            ])
            .unwrap();
        assert_eq!(payload[0], 1);
        assert_eq!(
            f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            1.5
        );
        assert_eq!(u16::from_le_bytes([payload[8], payload[9]]), 42);

        let state = m.decode_field(&payload, "state").unwrap();
        assert_eq!(state.value, 1.0);
        assert_eq!(state.symbol.as_deref(), Some("READY"));
        let speed = m.decode_field(&payload, "speed").unwrap();
        assert!((speed.value - 1.5).abs() < 1e-6);
    }

    #[test]
    fn scaled_fields() {
        let mut m = message();
        m.fields.insert(
            "temp".to_string(),
            FieldDef {
                offset: 10,
                ty: FieldType::U16le,
                factor: 0.1,
                shift: -40.0,
                values: BTreeMap::new(),
            },
        );
        let payload = m.encode_fields(&[("temp", 45.6.into())]).unwrap();
        let decoded = m.decode_field(&payload, "temp").unwrap();
        assert!((decoded.value - 45.6).abs() < 1e-9);
    }

    #[test]
    fn big_endian_encoding() {
        let m = MessageDef {
            dst: "127.0.0.1:9".parse().unwrap(),
            length: 2,
            fields: BTreeMap::from([(
                "v".to_string(),
                FieldDef {
                    offset: 0,
                    ty: FieldType::U16be,
                    factor: 1.0,
                    shift: 0.0,
                    values: BTreeMap::new(),
                },
            )]),
        };
        let payload = m.encode_fields(&[("v", 4660f64.into())]).unwrap();
        assert_eq!(payload[0], 0x12);
        assert_eq!(payload[1], 0x34);
        assert_eq!(m.decode_field(&payload, "v").unwrap().value, 4660.0);
    }

    #[test]
    fn unknown_field_errors() {
        let m = message();
        assert!(matches!(
            m.decode_field(&[0u8; 12], "nope"),
            Err(FieldError::UnknownField(_))
        ));
    }

    #[test]
    fn unknown_symbol_errors() {
        let m = message();
        let err = m
            .encode_fields(&[("state", SignalValue::Str("FAULT".into()))])
            .unwrap_err();
        assert!(matches!(err, FieldError::UnknownSymbol { .. }));
    }

    #[test]
    fn out_of_range_errors() {
        let m = message();
        let err = m
            .encode_fields(&[("counter", 70_000.0.into())])
            .unwrap_err();
        assert!(matches!(err, FieldError::ValueOutOfRange(..)));
    }

    #[test]
    fn payload_too_short_errors() {
        let m = message();
        assert!(m.decode_field(&[0u8; 2], "speed").is_err());
    }

    #[test]
    fn signed_field_round_trip() {
        let m = MessageDef {
            dst: "127.0.0.1:9".parse().unwrap(),
            length: 2,
            fields: BTreeMap::from([(
                "v".to_string(),
                FieldDef {
                    offset: 0,
                    ty: FieldType::I16le,
                    factor: 1.0,
                    shift: 0.0,
                    values: BTreeMap::new(),
                },
            )]),
        };
        let payload = m.encode_fields(&[("v", (-123.0).into())]).unwrap();
        assert_eq!(payload, vec![0x85, 0xFF]);
        assert_eq!(m.decode_field(&payload, "v").unwrap().value, -123.0);
    }

    #[test]
    fn bool_field_round_trip() {
        let m = MessageDef {
            dst: "127.0.0.1:9".parse().unwrap(),
            length: 1,
            fields: BTreeMap::from([(
                "on".to_string(),
                FieldDef {
                    offset: 0,
                    ty: FieldType::Bool,
                    factor: 1.0,
                    shift: 0.0,
                    values: BTreeMap::new(),
                },
            )]),
        };
        let on = m.encode_fields(&[("on", 1.0.into())]).unwrap();
        let off = m.encode_fields(&[("on", 0.0.into())]).unwrap();
        assert_eq!(m.decode_field(&on, "on").unwrap().value, 1.0);
        assert_eq!(m.decode_field(&off, "on").unwrap().value, 0.0);
    }
}
