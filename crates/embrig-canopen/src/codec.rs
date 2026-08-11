//! The CANopen [`SignalCodec`]: PDOs packed with the DBC bit-packer plus the
//! bespoke heartbeat and NMT message codecs.

use std::collections::BTreeMap;

use embrig_core::codec::{DecodedSignal, MessageCodec, SignalCodec};
use embrig_dbc::{ByteOrder, MessageDef, SignalDef};
use thiserror::Error;

use crate::spec::{EcuSpec, SignalSpec};
use crate::{
    heartbeat, nmt, rpdo1, tpdo1, HB_BOOT_UP, HB_OPERATIONAL, HB_PRE_OPERATIONAL, HB_STOPPED,
    NMT_PRE_OPERATIONAL, NMT_START, NMT_STOP,
};

/// Errors while building the CANopen codec.
#[derive(Debug, Error)]
pub enum CanOpenError {
    #[error("node id {0} is outside the CANopen range 1..=127")]
    InvalidNodeId(u8),
}

const HB_BOOT_UP_I64: i64 = HB_BOOT_UP as i64;
const HB_STOPPED_I64: i64 = HB_STOPPED as i64;
const HB_OPERATIONAL_I64: i64 = HB_OPERATIONAL as i64;
const HB_PRE_OPERATIONAL_I64: i64 = HB_PRE_OPERATIONAL as i64;
const NMT_START_I64: i64 = NMT_START as i64;
const NMT_STOP_I64: i64 = NMT_STOP as i64;
const NMT_PRE_OPERATIONAL_I64: i64 = NMT_PRE_OPERATIONAL as i64;

/// Build a DBC-style [`MessageDef`] from a PDO signal mapping.
///
/// The PDO payload is little-endian packed fields, which is exactly what the
/// DBC bit-packer already does — so a CANopen PDO mapping is just a message.
fn pdo(id: u32, name: &str, signals: &[SignalSpec]) -> MessageDef {
    MessageDef {
        id,
        name: name.to_string(),
        dlc: 8,
        signals: signals
            .iter()
            .map(|s| SignalDef {
                name: s.name.clone(),
                start_bit: s.bit,
                length: s.length,
                byte_order: ByteOrder::Intel,
                is_signed: s.is_signed,
                factor: s.factor,
                offset: s.offset,
                unit: String::new(),
                min: None,
                max: None,
                value_table: BTreeMap::new(),
            })
            .collect(),
    }
}

/// Heartbeat producer frame: one byte, the NMT state.
#[derive(Debug, Clone, PartialEq)]
pub struct HeartbeatMessage {
    /// COB-ID, `0x700 + node`.
    pub id: u32,
}

impl MessageCodec for HeartbeatMessage {
    fn id(&self) -> u32 {
        self.id
    }

    fn encode_signals(&self, values: &[(&str, f64)]) -> Result<Vec<u8>, String> {
        let state = values
            .iter()
            .find(|(name, _)| *name == "state")
            .map(|(_, v)| *v)
            .ok_or_else(|| "heartbeat needs a `state` signal".to_string())?;
        self.check_value("state", state)?;
        Ok(vec![state.round() as u8])
    }

    fn check_value(&self, name: &str, value: f64) -> Result<(), String> {
        match name {
            "state" if (0.0..=255.0).contains(&value) => Ok(()),
            "state" => Err(format!("heartbeat state {value} is out of range")),
            _ => Err(format!("no signal `{name}` on heartbeat")),
        }
    }

    fn physical_for_symbol(&self, signal: &str, symbol: &str) -> Option<f64> {
        if signal != "state" {
            return None;
        }
        match symbol {
            "BOOT_UP" => Some(HB_BOOT_UP as f64),
            "STOPPED" => Some(HB_STOPPED as f64),
            "OPERATIONAL" => Some(HB_OPERATIONAL as f64),
            "PRE_OPERATIONAL" => Some(HB_PRE_OPERATIONAL as f64),
            _ => None,
        }
    }

    fn symbol_for(&self, signal: &str, raw: i64) -> Option<String> {
        if signal != "state" {
            return None;
        }
        match raw {
            HB_BOOT_UP_I64 => Some("BOOT_UP".to_string()),
            HB_STOPPED_I64 => Some("STOPPED".to_string()),
            HB_OPERATIONAL_I64 => Some("OPERATIONAL".to_string()),
            HB_PRE_OPERATIONAL_I64 => Some("PRE_OPERATIONAL".to_string()),
            _ => None,
        }
    }

    fn decode_signals(&self, data: &[u8]) -> Result<Vec<DecodedSignal>, String> {
        let state = data
            .first()
            .copied()
            .ok_or_else(|| "heartbeat frame is empty".to_string())?;
        Ok(vec![DecodedSignal {
            name: "state".into(),
            value: state as f64,
            symbol: self.symbol_for("state", state as i64),
        }])
    }

    fn decode_signal(&self, data: &[u8], name: &str) -> Result<f64, String> {
        if name != "state" {
            return Err(format!("no signal `{name}` on heartbeat"));
        }
        Ok(*data
            .first()
            .ok_or_else(|| "heartbeat frame is empty".to_string())? as f64)
    }

    fn boxed(&self) -> Box<dyn MessageCodec> {
        Box::new(self.clone())
    }
}

/// NMT frame: two bytes, `[node-id, command]`.
#[derive(Debug, Clone, PartialEq)]
pub struct NmtMessage {
    /// COB-ID, always `0x000`.
    pub id: u32,
}

impl MessageCodec for NmtMessage {
    fn id(&self) -> u32 {
        self.id
    }

    fn encode_signals(&self, values: &[(&str, f64)]) -> Result<Vec<u8>, String> {
        let node = values
            .iter()
            .find(|(name, _)| *name == "node")
            .map(|(_, v)| *v)
            .ok_or_else(|| "NMT needs a `node` signal".to_string())?;
        let command = values
            .iter()
            .find(|(name, _)| *name == "command")
            .map(|(_, v)| *v)
            .ok_or_else(|| "NMT needs a `command` signal".to_string())?;
        self.check_value("node", node)?;
        self.check_value("command", command)?;
        Ok(vec![node.round() as u8, command.round() as u8])
    }

    fn check_value(&self, name: &str, value: f64) -> Result<(), String> {
        match name {
            "node" if (0.0..=127.0).contains(&value) => Ok(()),
            "node" => Err(format!("NMT node id {value} is out of range")),
            "command" if (0.0..=255.0).contains(&value) => Ok(()),
            "command" => Err(format!("NMT command {value} is out of range")),
            _ => Err(format!("no signal `{name}` on NMT")),
        }
    }

    fn physical_for_symbol(&self, signal: &str, symbol: &str) -> Option<f64> {
        if signal != "command" {
            return None;
        }
        match symbol {
            "START" => Some(NMT_START as f64),
            "STOP" => Some(NMT_STOP as f64),
            "PRE_OPERATIONAL" => Some(NMT_PRE_OPERATIONAL as f64),
            _ => None,
        }
    }

    fn symbol_for(&self, signal: &str, raw: i64) -> Option<String> {
        if signal != "command" {
            return None;
        }
        match raw {
            NMT_START_I64 => Some("START".to_string()),
            NMT_STOP_I64 => Some("STOP".to_string()),
            NMT_PRE_OPERATIONAL_I64 => Some("PRE_OPERATIONAL".to_string()),
            _ => None,
        }
    }

    fn decode_signals(&self, data: &[u8]) -> Result<Vec<DecodedSignal>, String> {
        if data.len() < 2 {
            return Err(format!("NMT frame needs 2 bytes, got {}", data.len()));
        }
        Ok(vec![
            DecodedSignal {
                name: "node".into(),
                value: data[0] as f64,
                symbol: None,
            },
            DecodedSignal {
                name: "command".into(),
                value: data[1] as f64,
                symbol: self.symbol_for("command", data[1] as i64),
            },
        ])
    }

    fn decode_signal(&self, data: &[u8], name: &str) -> Result<f64, String> {
        if data.len() < 2 {
            return Err(format!("NMT frame needs 2 bytes, got {}", data.len()));
        }
        match name {
            "node" => Ok(data[0] as f64),
            "command" => Ok(data[1] as f64),
            _ => Err(format!("no signal `{name}` on NMT")),
        }
    }

    fn boxed(&self) -> Box<dyn MessageCodec> {
        Box::new(self.clone())
    }
}

/// A CANopen node as a [`SignalCodec`].
///
/// Resolves the node's TPDO1, RPDO1, heartbeat and NMT messages by COB-ID
/// (for `expect`) and by name (for `vehicle.yaml` `config` ECU entries).
#[derive(Debug, Clone)]
pub struct CanOpenCodec {
    node: u8,
    tpdo: MessageDef,
    rpdo: MessageDef,
    heartbeat: HeartbeatMessage,
    nmt: NmtMessage,
}

impl CanOpenCodec {
    /// Build the codec for the node described by `spec`.
    pub fn new(spec: &EcuSpec) -> Result<Self, CanOpenError> {
        let node = spec.node_id;
        if node == 0 || node > 127 {
            return Err(CanOpenError::InvalidNodeId(node));
        }
        Ok(Self {
            node,
            tpdo: pdo(tpdo1(node), "TPDO1", &spec.tpdo1),
            rpdo: pdo(rpdo1(node), "RPDO1", &spec.rpdo1),
            heartbeat: HeartbeatMessage {
                id: heartbeat(node),
            },
            nmt: NmtMessage { id: nmt() },
        })
    }

    /// The node id the codec was built for.
    pub fn node_id(&self) -> u8 {
        self.node
    }

    /// The TPDO1 message definition.
    pub fn tpdo(&self) -> &MessageDef {
        &self.tpdo
    }

    /// The RPDO1 message definition.
    pub fn rpdo(&self) -> &MessageDef {
        &self.rpdo
    }
}

impl SignalCodec for CanOpenCodec {
    fn message_by_id(&self, id: u32) -> Option<&dyn MessageCodec> {
        match id {
            id if id == self.tpdo.id => Some(&self.tpdo),
            id if id == self.rpdo.id => Some(&self.rpdo),
            id if id == self.heartbeat.id => Some(&self.heartbeat),
            id if id == self.nmt.id => Some(&self.nmt),
            _ => None,
        }
    }

    fn message_by_name(&self, name: &str) -> Option<&dyn MessageCodec> {
        match name {
            "TPDO1" => Some(&self.tpdo),
            "RPDO1" => Some(&self.rpdo),
            "Heartbeat" => Some(&self.heartbeat),
            "NMT" => Some(&self.nmt),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::NMT;

    fn spec() -> EcuSpec {
        EcuSpec {
            node_id: 1,
            heartbeat_period_us: 100_000,
            tpdo1: vec![SignalSpec {
                name: "valve_open".into(),
                bit: 0,
                length: 1,
                is_signed: false,
                factor: 1.0,
                offset: 0.0,
            }],
            rpdo1: vec![SignalSpec {
                name: "temperature".into(),
                bit: 0,
                length: 16,
                is_signed: false,
                factor: 0.1,
                offset: 0.0,
            }],
        }
    }

    #[test]
    fn resolves_by_cob_id_and_name() {
        let codec = CanOpenCodec::new(&spec()).unwrap();
        assert_eq!(codec.node_id(), 1);
        assert!(codec.message_by_id(tpdo1(1)).is_some());
        assert!(codec.message_by_id(rpdo1(1)).is_some());
        assert!(codec.message_by_id(heartbeat(1)).is_some());
        assert!(codec.message_by_id(NMT).is_some());
        assert!(codec.message_by_id(0x500).is_none());
        assert_eq!(codec.message_by_name("TPDO1").unwrap().id(), tpdo1(1));
        assert_eq!(codec.message_by_name("RPDO1").unwrap().id(), rpdo1(1));
        assert!(codec.message_by_name("SDO").is_none());
    }

    #[test]
    fn rpdo_packs_temperature_with_scaling() {
        let codec = CanOpenCodec::new(&spec()).unwrap();
        let data = codec
            .rpdo()
            .encode_signals(&[("temperature", 45.6)])
            .unwrap();
        assert_eq!(data[0], 0xC8); // raw 456 = 0x1C8
        assert_eq!(data[1], 0x01);
        assert!((codec.rpdo().decode_signal(&data, "temperature").unwrap() - 45.6).abs() < 1e-9);
    }

    #[test]
    fn tpdo_packs_valve_bit() {
        let codec = CanOpenCodec::new(&spec()).unwrap();
        let data = codec.tpdo().encode_signals(&[("valve_open", 1.0)]).unwrap();
        assert_eq!(data[0], 0x01);
        assert_eq!(
            codec.tpdo().decode_signal(&data, "valve_open").unwrap(),
            1.0
        );
    }

    #[test]
    fn heartbeat_encodes_and_decodes_states() {
        let hb = HeartbeatMessage { id: heartbeat(1) };
        let data = hb
            .encode_signals(&[("state", HB_OPERATIONAL as f64)])
            .unwrap();
        assert_eq!(data, vec![HB_OPERATIONAL]);
        let decoded = hb.decode_signals(&data).unwrap();
        assert_eq!(decoded[0].value, HB_OPERATIONAL as f64);
        assert_eq!(decoded[0].symbol.as_deref(), Some("OPERATIONAL"));
        assert_eq!(hb.physical_for_symbol("state", "STOPPED"), Some(4.0));
        assert!(hb.check_value("state", 300.0).is_err());
    }

    #[test]
    fn nmt_encodes_and_decodes() {
        let nmt_msg = NmtMessage { id: nmt() };
        let data = nmt_msg
            .encode_signals(&[("node", 1.0), ("command", NMT_START as f64)])
            .unwrap();
        assert_eq!(data, vec![1, NMT_START]);
        let decoded = nmt_msg.decode_signals(&data).unwrap();
        assert_eq!(decoded[0].value, 1.0);
        assert_eq!(decoded[1].symbol.as_deref(), Some("START"));
        assert_eq!(
            nmt_msg.physical_for_symbol("command", "STOP"),
            Some(NMT_STOP as f64)
        );
        assert!(nmt_msg.check_value("node", 200.0).is_err());
    }

    #[test]
    fn rejects_invalid_node_id() {
        let mut bad = spec();
        bad.node_id = 0;
        assert!(matches!(
            CanOpenCodec::new(&bad),
            Err(CanOpenError::InvalidNodeId(0))
        ));
    }
}
