use std::collections::BTreeMap;

use crate::types::{ByteOrder, MessageDef, Network, SignalDef};
use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq)]
pub enum ParseError {
    #[error("unexpected line {line}: {text}")]
    Unexpected { line: usize, text: String },
    #[error("malformed signal spec `{spec}`: {reason}")]
    MalformedSignal { spec: String, reason: String },
    #[error("malformed factor/offset `{text}`: {reason}")]
    MalformedFactor { text: String, reason: String },
    #[error("malformed value table on line {line}: {reason}")]
    MalformedValueTable { line: usize, reason: String },
}

/// Parse a DBC document into a [`Network`].
pub fn parse(input: &str) -> Result<Network, ParseError> {
    let mut network = Network::new();
    // Message currently being filled in by SG_ lines.
    let mut current: Option<u32> = None;

    for (idx, raw_line) in input.lines().enumerate() {
        let line_no = idx + 1;
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with("//") || line.starts_with("VERSION") {
            continue;
        }

        if let Some(rest) = line.strip_prefix("BO_ ") {
            current = Some(parse_bo(rest, line_no, &mut network)?);
        } else if let Some(rest) = line.strip_prefix("SG_ ") {
            let id = current.ok_or_else(|| ParseError::Unexpected {
                line: line_no,
                text: line.to_string(),
            })?;
            let signal = parse_sg(rest, line_no)?;
            network
                .messages
                .get_mut(&id)
                .expect("BO_ inserted current")
                .signals
                .push(signal);
        } else if let Some(rest) = line.strip_prefix("VAL_ ") {
            parse_val(rest, line_no, &mut network)?;
        } else if line.starts_with("BS_:")
            || line.starts_with("NS_")
            || line.starts_with("BU_:")
            || line.starts_with("BA_")
            || line.starts_with("CM_")
        {
            // Attributes, node tables, comments: not needed for simulation.
        } else {
            // Unknown line: be tolerant of comment-like content, error otherwise.
            return Err(ParseError::Unexpected {
                line: line_no,
                text: line.to_string(),
            });
        }
    }

    Ok(network)
}

/// `BO_ 256 BatteryStatus: 8 Vector__XXX` — registers the message.
fn parse_bo(rest: &str, line_no: usize, network: &mut Network) -> Result<u32, ParseError> {
    let unexp = || ParseError::Unexpected {
        line: line_no,
        text: format!("BO_ {rest}"),
    };
    let mut tokens = rest.split_whitespace();
    let id: u32 = tokens
        .next()
        .ok_or_else(unexp)?
        .parse()
        .map_err(|_| unexp())?;
    let name_token = tokens.next().ok_or_else(unexp)?;
    let name = name_token.trim_end_matches(':').to_string();
    let dlc: u8 = tokens
        .next()
        .ok_or_else(unexp)?
        .parse()
        .map_err(|_| unexp())?;
    network.messages.insert(
        id,
        MessageDef {
            id,
            name,
            dlc,
            signals: Vec::new(),
        },
    );
    Ok(id)
}

/// `SG_ state : 24|8@1+ (1,0) [0|4] "State"  Vector__XXX`
fn parse_sg(rest: &str, line_no: usize) -> Result<SignalDef, ParseError> {
    let unexp = || ParseError::Unexpected {
        line: line_no,
        text: format!("SG_ {rest}"),
    };
    let tokens: Vec<&str> = rest.split_whitespace().collect();
    if tokens.len() < 4 {
        return Err(unexp());
    }
    let name = tokens[0].to_string();

    // Find the spec token (contains '@').
    let spec_idx = tokens
        .iter()
        .position(|t| t.contains('@'))
        .ok_or_else(unexp)?;

    let spec = tokens[spec_idx];
    let (start, len, order, is_signed) = parse_spec(spec, line_no)?;

    let factor_text = tokens.get(spec_idx + 1).ok_or_else(unexp)?;
    let (factor, offset) = parse_factor(factor_text, line_no)?;
    let mut unit = String::new();
    if let Some(t) = tokens.get(spec_idx + 3) {
        unit = t.trim_matches('"').to_string();
    }

    let min = tokens
        .get(spec_idx + 2)
        .and_then(|t| parse_range(t))
        .map(|(lo, _)| lo);

    Ok(SignalDef {
        name,
        start_bit: start,
        length: len,
        byte_order: order,
        is_signed,
        factor,
        offset,
        unit,
        min,
        max: tokens
            .get(spec_idx + 2)
            .and_then(|t| parse_range(t))
            .map(|(_, hi)| hi),
        value_table: BTreeMap::new(),
    })
}

/// `0|16@1+` -> (0, 16, Intel, unsigned)
fn parse_spec(spec: &str, line_no: usize) -> Result<(u16, u16, ByteOrder, bool), ParseError> {
    let err = || ParseError::MalformedSignal {
        spec: spec.to_string(),
        reason: format!("line {line_no}"),
    };
    let (layout, order_sign) = spec.split_once('@').ok_or_else(err)?;
    let (start_s, len_s) = layout.split_once('|').ok_or_else(err)?;
    let start: u16 = start_s.parse().map_err(|_| err())?;
    let len: u16 = len_s.parse().map_err(|_| err())?;

    let mut chars = order_sign.chars();
    let order = match chars.next() {
        Some('1') => ByteOrder::Intel,
        Some('0') => ByteOrder::Motorola,
        _ => return Err(err()),
    };
    let is_signed = match chars.next() {
        Some('-') => true,
        Some('+') => false,
        _ => return Err(err()),
    };
    Ok((start, len, order, is_signed))
}

/// `(0.1,0)` -> (0.1, 0)
fn parse_factor(text: &str, line_no: usize) -> Result<(f64, f64), ParseError> {
    let inner = text.trim().trim_start_matches('(').trim_end_matches(')');
    let mut parts = inner.split(',');
    let factor = parts
        .next()
        .and_then(|s| s.trim().parse::<f64>().ok())
        .ok_or_else(|| ParseError::MalformedFactor {
            text: text.to_string(),
            reason: format!("line {line_no}"),
        })?;
    let offset = parts
        .next()
        .and_then(|s| s.trim().parse::<f64>().ok())
        .unwrap_or(0.0);
    Ok((factor, offset))
}

/// `[0|1000]` -> (0.0, 1000.0)
fn parse_range(text: &str) -> Option<(f64, f64)> {
    let inner = text.trim().trim_start_matches('[').trim_end_matches(']');
    let mut parts = inner.split('|');
    let lo = parts.next()?.trim().parse::<f64>().ok()?;
    let hi = parts.next()?.trim().parse::<f64>().ok()?;
    Some((lo, hi))
}

/// `VAL_ 256 state 0 "OFF" 1 "READY" ;`
fn parse_val(rest: &str, line: usize, network: &mut Network) -> Result<(), ParseError> {
    let tokens: Vec<&str> = rest.split_whitespace().collect();
    if tokens.len() < 3 {
        return Err(ParseError::MalformedValueTable {
            line,
            reason: "expected <id> <signal> <pairs>".into(),
        });
    }
    let id: u32 = tokens[0]
        .parse()
        .map_err(|_| ParseError::MalformedValueTable {
            line,
            reason: format!("bad message id `{}`", tokens[0]),
        })?;
    let signal_name = tokens[1].to_string();

    let Some(message) = network.messages.get_mut(&id) else {
        // VAL_ for a message we have not parsed yet; ignore.
        return Ok(());
    };
    let Some(signal) = message.signals.iter_mut().find(|s| s.name == signal_name) else {
        return Ok(());
    };

    let mut pairs = tokens[2..].iter();
    while let Some(value_s) = pairs.next() {
        let value_s = value_s.trim_end_matches(';');
        if value_s.is_empty() {
            break;
        }
        let value: i64 = value_s
            .parse()
            .map_err(|_| ParseError::MalformedValueTable {
                line,
                reason: format!("bad value `{value_s}`"),
            })?;
        let Some(symbol) = pairs.next() else {
            return Err(ParseError::MalformedValueTable {
                line,
                reason: "symbol missing for value".into(),
            });
        };
        signal
            .value_table
            .insert(value, symbol.trim_matches('"').to_string());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
VERSION ""

NS_ :

BS_:

BU_: Vector__XXX

BO_ 256 BatteryStatus: 8 Vector__XXX
 SG_ voltage : 0|16@1+ (0.1,0) [0|1000] "V"  Vector__XXX
 SG_ state : 24|8@1+ (1,0) [0|4] ""  Vector__XXX

BO_ 512 MotorStatus: 8 Vector__XXX
 SG_ rpm : 0|16@0+ (1,0) [0|10000] "rpm"  Vector__XXX

VAL_ 256 state 0 "OFF" 1 "INIT" 2 "READY" ;
"#;

    #[test]
    fn parses_network() {
        let net = parse(SAMPLE).unwrap();
        assert_eq!(net.messages.len(), 2);
        let b = net.message(0x100).unwrap();
        assert_eq!(b.name, "BatteryStatus");
        assert_eq!(b.dlc, 8);
        assert_eq!(b.signals.len(), 2);
        let v = b.signal("voltage").unwrap();
        assert_eq!(v.start_bit, 0);
        assert_eq!(v.length, 16);
        assert_eq!(v.byte_order, ByteOrder::Intel);
        assert!((v.factor - 0.1).abs() < 1e-9);
        assert_eq!(v.unit, "V");
    }

    #[test]
    fn parses_motorola() {
        let net = parse(SAMPLE).unwrap();
        let m = net.message(0x200).unwrap();
        let rpm = m.signal("rpm").unwrap();
        assert_eq!(rpm.byte_order, ByteOrder::Motorola);
        assert!(!rpm.is_signed);
    }

    #[test]
    fn parses_value_table() {
        let net = parse(SAMPLE).unwrap();
        let b = net.message(0x100).unwrap();
        assert_eq!(b.symbol_for("state", 2).as_deref(), Some("READY"));
        assert_eq!(b.resolve_symbol("state", "OFF"), Some(0));
    }

    #[test]
    fn tolerates_unknown_attribute_lines() {
        let net = parse("BS_:\nBA_ \"GenMsgCycleTime\" BO_ 256 100;\nBO_ 1 A: 8 X\n SG_ a : 0|8@1+ (1,0) [0|255] \"\" X\n").unwrap();
        assert_eq!(net.messages.len(), 1);
    }

    #[test]
    fn rejects_malformed_spec() {
        let input = "BO_ 1 A: 8 X\n SG_ a : bad@1+ (1,0) [0|255] \"\" X\n";
        assert!(matches!(
            parse(input),
            Err(ParseError::MalformedSignal { .. })
        ));
    }
}
