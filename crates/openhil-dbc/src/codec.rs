use crate::types::{ByteOrder, MessageDef, Signal, SignalDef};
use thiserror::Error;

/// Error produced while encoding or decoding signals.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum CodecError {
    #[error("message {0} defines more signals than fit in {1} bytes")]
    Overflows(u32, usize),
    #[error("signal `{0}` (bit {1} + {2}) does not fit in {3} bytes")]
    SignalOutOfBounds(String, u16, u16, usize),
    #[error("signal `{name}` expects a numeric value, got `{value}`")]
    SymbolicValue { name: String, value: String },
    #[error("unknown symbol `{symbol}` for signal `{signal}`")]
    UnknownSymbol { signal: String, symbol: String },
    #[error("data length {0} is shorter than DLC {1}")]
    ShortData(usize, u8),
    #[error("value {0} is out of range for a {1}-bit signal")]
    ValueOutOfRange(i64, u16),
}

/// Compute the (byte, bit-in-byte) positions covered by a signal.
///
/// `bit_in_byte` counts from the LSB (0) of each byte. The returned vector is
/// indexed by the signal's bit number (`0` = LSB of the value).
///
/// - **Intel**: bits are numbered 0, 1, 2, … from the LSB of byte 0, so the
///   positions are simply `start_bit` onward.
/// - **Motorola**: the start bit is the position of the signal's MSB. The
///   bits run downward through each byte (`7, 6, …, 0` in LSB numbering) and
///   wrap to the next byte's bit 7 when a byte boundary is crossed.
fn positions(signal: &SignalDef) -> Vec<(usize, u8)> {
    match signal.byte_order {
        ByteOrder::Intel => {
            let mut out = Vec::with_capacity(signal.length as usize);
            for i in 0..signal.length {
                let bit = signal.start_bit + i;
                out.push((bit as usize / 8, (bit % 8) as u8));
            }
            out
        }
        ByteOrder::Motorola => {
            let mut out = vec![(0usize, 0u8); signal.length as usize];
            let mut p: i64 = signal.start_bit as i64;
            for j in (0..signal.length).rev() {
                out[j as usize] = ((p / 8) as usize, (p % 8) as u8);
                p = if p % 8 == 0 { p + 15 } else { p - 1 };
            }
            out
        }
    }
}

impl MessageDef {
    /// Encode physical values for a set of named signals into a CAN data field.
    ///
    /// Values are scaled using each signal's factor/offset. A signal may be
    /// omitted (encoded as zero) or overridden by a later entry.
    pub fn encode_signals(&self, values: &[(&str, f64)]) -> Result<Vec<u8>, CodecError> {
        let mut data = vec![0u8; self.dlc.max(1) as usize];
        let resolved = self.resolve_all(values);
        for signal in &self.signals {
            let Some(&raw) = resolved.get(&signal.name) else {
                continue;
            };
            self.write_signal(signal, raw, &mut data)?;
        }
        Ok(data)
    }

    /// Encode a single signal into a zero-initialized data field.
    pub fn encode_signal(&self, name: &str, value: f64) -> Result<Vec<u8>, CodecError> {
        let Some(signal) = self.signal(name) else {
            return Err(CodecError::UnknownSymbol {
                signal: name.to_string(),
                symbol: format!("{value}"),
            });
        };
        let raw = physical_to_raw(signal, value)?;
        let mut data = vec![0u8; self.dlc.max(1) as usize];
        self.write_signal(signal, raw, &mut data)?;
        Ok(data)
    }

    /// Decode all signals in a CAN data field into physical values.
    pub fn decode_signals(&self, data: &[u8]) -> Result<Vec<Signal>, CodecError> {
        if data.len() < self.dlc as usize {
            return Err(CodecError::ShortData(data.len(), self.dlc));
        }
        let mut out = Vec::with_capacity(self.signals.len());
        for signal in &self.signals {
            let raw = self.read_signal(signal, data)?;
            let value = raw_to_physical(signal, raw);
            let symbol = signal.value_table.get(&raw).cloned();
            out.push(Signal {
                name: signal.name.clone(),
                value,
                unit: signal.unit.clone(),
                symbol,
            });
        }
        Ok(out)
    }

    /// Decode a single signal.
    pub fn decode_signal(&self, data: &[u8], name: &str) -> Result<f64, CodecError> {
        let Some(signal) = self.signal(name) else {
            return Err(CodecError::UnknownSymbol {
                signal: name.to_string(),
                symbol: String::new(),
            });
        };
        let raw = self.read_signal(signal, data)?;
        Ok(raw_to_physical(signal, raw))
    }

    /// Validate that `value` can be encoded for `name` without encoding.
    pub fn check_value(&self, name: &str, value: f64) -> Result<(), CodecError> {
        let Some(signal) = self.signal(name) else {
            return Err(CodecError::UnknownSymbol {
                signal: name.to_string(),
                symbol: format!("{value}"),
            });
        };
        physical_to_raw(signal, value).map(|_| ())
    }

    fn resolve_all(&self, values: &[(&str, f64)]) -> std::collections::BTreeMap<String, i64> {
        let mut map = std::collections::BTreeMap::new();
        for (name, value) in values {
            if let Some(signal) = self.signal(name) {
                if let Ok(raw) = physical_to_raw(signal, *value) {
                    map.insert(name.to_string(), raw);
                }
            }
        }
        map
    }

    fn write_signal(
        &self,
        signal: &SignalDef,
        raw: i64,
        data: &mut [u8],
    ) -> Result<(), CodecError> {
        let positions = positions(signal);
        if let Some(&(byte, _)) = positions.last() {
            if byte >= data.len() {
                return Err(CodecError::SignalOutOfBounds(
                    signal.name.clone(),
                    signal.start_bit,
                    signal.length,
                    data.len(),
                ));
            }
        }
        let raw = raw as u64;
        for (i, (byte, bit)) in positions.into_iter().enumerate() {
            let mask = 1u64 << i;
            if raw & mask != 0 {
                data[byte] |= 1 << bit;
            } else {
                data[byte] &= !(1 << bit);
            }
        }
        Ok(())
    }

    fn read_signal(&self, signal: &SignalDef, data: &[u8]) -> Result<i64, CodecError> {
        let positions = positions(signal);
        if let Some(&(byte, _)) = positions.last() {
            if byte >= data.len() {
                return Err(CodecError::SignalOutOfBounds(
                    signal.name.clone(),
                    signal.start_bit,
                    signal.length,
                    data.len(),
                ));
            }
        }
        let mut raw: u64 = 0;
        for (i, (byte, bit)) in positions.into_iter().enumerate() {
            if data[byte] & (1 << bit) != 0 {
                raw |= 1u64 << i;
            }
        }
        let raw = if signal.is_signed {
            sign_extend(raw, signal.length)
        } else {
            raw as i64
        };
        Ok(raw)
    }
}

/// Convert a physical value to the raw integer stored in the frame.
fn physical_to_raw(signal: &SignalDef, value: f64) -> Result<i64, CodecError> {
    let raw_f = (value - signal.offset) / signal.factor;
    let raw = raw_f.round() as i64;
    if raw_f.is_nan() || raw_f.is_infinite() {
        return Err(CodecError::ValueOutOfRange(raw, signal.length));
    }
    if !fits_bits(raw, signal.length, signal.is_signed) {
        return Err(CodecError::ValueOutOfRange(raw, signal.length));
    }
    Ok(raw)
}

/// Convert a raw integer back to a physical value.
fn raw_to_physical(signal: &SignalDef, raw: i64) -> f64 {
    raw as f64 * signal.factor + signal.offset
}

/// Sign-extend a raw value to i64.
fn sign_extend(raw: u64, length: u16) -> i64 {
    if length == 64 {
        return raw as i64;
    }
    if length > 0 && raw & (1u64 << (length - 1)) != 0 {
        let mask = (1u64 << length) - 1;
        (raw | !mask) as i64
    } else {
        raw as i64
    }
}

fn fits_bits(value: i64, length: u16, is_signed: bool) -> bool {
    if length >= 64 {
        return true;
    }
    let max_unsigned = (1u64 << length) - 1;
    if is_signed {
        let min = -(1i64 << (length - 1));
        let max = (1i64 << (length - 1)) - 1;
        value >= min && value <= max
    } else {
        value >= 0 && (value as u64) <= max_unsigned
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn signal(
        name: &str,
        start_bit: u16,
        length: u16,
        order: ByteOrder,
        is_signed: bool,
        factor: f64,
        offset: f64,
    ) -> SignalDef {
        SignalDef {
            name: name.to_string(),
            start_bit,
            length,
            byte_order: order,
            is_signed,
            factor,
            offset,
            unit: String::new(),
            min: None,
            max: None,
            value_table: Default::default(),
        }
    }

    fn msg(signals: Vec<SignalDef>) -> MessageDef {
        MessageDef {
            id: 0x100,
            name: "Test".into(),
            dlc: 8,
            signals,
        }
    }

    #[test]
    fn intel_16_bit_little_endian() {
        let m = msg(vec![signal("v", 0, 16, ByteOrder::Intel, false, 1.0, 0.0)]);
        let data = m.encode_signal("v", 0x1234u32 as f64).unwrap();
        assert_eq!(data[0], 0x34);
        assert_eq!(data[1], 0x12);
        assert_eq!(m.decode_signal(&data, "v").unwrap(), 0x1234u32 as f64);
    }

    #[test]
    fn intel_signal_straddling_bytes() {
        // start bit 12 = byte1 bit4, 8 bits wide: value bits 0..3 at
        // byte1 bits 4..7, value bits 4..7 at byte2 bits 0..3.
        let m = msg(vec![signal("v", 12, 8, ByteOrder::Intel, false, 1.0, 0.0)]);
        let data = m.encode_signal("v", 0x2Au8 as f64).unwrap();
        assert_eq!(data[1], 0xA0);
        assert_eq!(data[2], 0x02);
        assert_eq!(m.decode_signal(&data, "v").unwrap(), 0x2Au8 as f64);
    }

    #[test]
    fn motorola_8_bit_plain() {
        // A single-byte Motorola signal in byte 0 has its MSB (start bit) at
        // bit 7, so the byte reads out unshifted.
        let m = msg(vec![signal(
            "v",
            7,
            8,
            ByteOrder::Motorola,
            false,
            1.0,
            0.0,
        )]);
        let data = m.encode_signal("v", 0x12u8 as f64).unwrap();
        assert_eq!(data[0], 0x12);
        assert_eq!(m.decode_signal(&data, "v").unwrap(), 0x12u8 as f64);
    }

    #[test]
    fn motorola_16_bit_big_endian() {
        // start bit 7 spans bytes 0..1 with the high byte in byte 0.
        let m = msg(vec![signal(
            "v",
            7,
            16,
            ByteOrder::Motorola,
            false,
            1.0,
            0.0,
        )]);
        let data = m.encode_signal("v", 0x1234u32 as f64).unwrap();
        assert_eq!(data[0], 0x12);
        assert_eq!(data[1], 0x34);
        assert_eq!(data[2], 0x00);
        assert_eq!(data[3], 0x00);
        assert_eq!(m.decode_signal(&data, "v").unwrap(), 0x1234u32 as f64);
    }

    #[test]
    fn signed_two_complement() {
        let m = msg(vec![signal("v", 0, 8, ByteOrder::Intel, true, 1.0, 0.0)]);
        let data = m.encode_signal("v", -5.0).unwrap();
        assert_eq!(data[0], 0xFB);
        assert_eq!(m.decode_signal(&data, "v").unwrap(), -5.0);
    }

    #[test]
    fn factor_and_offset_scaling() {
        let m = msg(vec![signal(
            "v",
            0,
            16,
            ByteOrder::Intel,
            false,
            0.1,
            -40.0,
        )]);
        let data = m.encode_signal("v", 45.6).unwrap();
        // raw = (45.6 - -40) / 0.1 = 856
        assert_eq!(data[0], (856u16 & 0xFF) as u8);
        assert_eq!(data[1], (856u16 >> 8) as u8);
        assert!((m.decode_signal(&data, "v").unwrap() - 45.6).abs() < 1e-9);
    }

    #[test]
    fn single_bit_signal() {
        let m = msg(vec![signal("on", 3, 1, ByteOrder::Intel, false, 1.0, 0.0)]);
        let data = m.encode_signal("on", 1.0).unwrap();
        assert_eq!(data[0], 0x08);
        assert_eq!(m.decode_signal(&data, "on").unwrap(), 1.0);
    }

    #[test]
    fn value_out_of_range_errors() {
        let m = msg(vec![signal("v", 0, 8, ByteOrder::Intel, false, 1.0, 0.0)]);
        assert!(m.encode_signal("v", 300.0).is_err());
        assert!(m.encode_signal("v", -1.0).is_err());
    }

    #[test]
    fn round_trip_across_layouts() {
        let layouts: Vec<(u16, u16, ByteOrder, bool)> = vec![
            (0, 8, ByteOrder::Intel, false),
            (4, 8, ByteOrder::Intel, false),
            (12, 16, ByteOrder::Intel, true),
            (0, 16, ByteOrder::Motorola, true),
            (7, 16, ByteOrder::Motorola, false),
            (3, 8, ByteOrder::Motorola, true),
            (0, 32, ByteOrder::Intel, false),
            (4, 12, ByteOrder::Intel, true),
        ];
        let values: Vec<f64> = vec![0.0, 1.0, 2.0, 127.0, 128.0, 1023.0, 4095.0, 12345.0];
        for (start, len, order, signed) in layouts {
            let m = msg(vec![signal("v", start, len, order, signed, 1.0, 0.0)]);
            for v in &values {
                if m.encode_signal("v", *v).is_err() {
                    continue;
                }
                let data = m.encode_signal("v", *v).unwrap();
                let decoded = m.decode_signal(&data, "v").unwrap();
                assert_eq!(
                    decoded, *v,
                    "layout {start}|{len}@{order:?} signed={signed}"
                );
            }
        }
    }

    #[test]
    fn decode_symbols_from_value_table() {
        let mut s = signal("state", 0, 8, ByteOrder::Intel, false, 1.0, 0.0);
        s.value_table.insert(0, "OFF".into());
        s.value_table.insert(2, "READY".into());
        let m = msg(vec![s]);
        let data = m.encode_signal("state", 2.0).unwrap();
        let decoded = m.decode_signals(&data).unwrap();
        assert_eq!(decoded[0].symbol.as_deref(), Some("READY"));
        assert_eq!(decoded[0].value, 2.0);
        assert_eq!(m.resolve_symbol("state", "OFF"), Some(0));
    }

    #[test]
    fn short_data_errors() {
        let m = msg(vec![signal("v", 0, 8, ByteOrder::Intel, false, 1.0, 0.0)]);
        assert_eq!(
            m.decode_signals(&[0u8; 2]),
            Err(CodecError::ShortData(2, 8))
        );
    }
}
