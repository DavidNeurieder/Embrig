//! UDP virtual ECUs: the config-driven stimulus node and re-exports of the
//! unified [`NetEcu`] trait, [`NetEcuError`], [`NetEcuFactory`] and
//! [`NetRegistry`] specialized for [`UdpDatagram`].

use std::collections::BTreeMap;
use std::net::SocketAddr;

use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;

use crate::datagram::UdpDatagram;
use crate::netmap::MessageDef;

pub use embrig_core::network::{NetEcu, NetEcuError, NetEcuFactory, NetRegistry};

/// A config-driven stimulus node: transmits one message on a fixed period with
/// field values that can be overridden at runtime.
pub struct UdpConfigEcu {
    name: String,
    src: SocketAddr,
    message_name: String,
    message: MessageDef,
    period_us: u64,
    base: BTreeMap<String, SignalValue>,
    overrides: BTreeMap<String, SignalValue>,
    next: Timestamp,
}

impl UdpConfigEcu {
    /// Create a stimulus node. Fields not present in `base` default to zero.
    pub fn new(
        name: String,
        src: SocketAddr,
        message_name: &str,
        message: MessageDef,
        period_us: u64,
        base: BTreeMap<String, SignalValue>,
    ) -> Self {
        let mut base = base;
        for field in message.fields.keys() {
            base.entry(field.clone()).or_insert(SignalValue::Num(0.0));
        }
        Self {
            name,
            src,
            message_name: message_name.to_string(),
            message,
            period_us,
            base,
            overrides: BTreeMap::new(),
            next: 0,
        }
    }

    fn encode(&self) -> Result<Vec<u8>, NetEcuError> {
        let values: Vec<(&str, SignalValue)> = self
            .message
            .fields
            .keys()
            .map(|name| {
                let value = self
                    .overrides
                    .get(name)
                    .or_else(|| self.base.get(name))
                    .cloned()
                    .unwrap_or(SignalValue::Num(0.0));
                (name.as_str(), value)
            })
            .collect();
        self.message
            .encode_fields(&values)
            .map_err(|e| NetEcuError::InvalidValue(e.to_string()))
    }
}

impl NetEcu<UdpDatagram> for UdpConfigEcu {
    fn name(&self) -> &str {
        &self.name
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<UdpDatagram>) {
        if time < self.next {
            return;
        }
        if let Ok(payload) = self.encode() {
            out.push(UdpDatagram::new(self.src, self.message.dst, payload));
        }
        self.next = time + self.period_us;
    }

    fn set_field(
        &mut self,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), NetEcuError> {
        if message != self.message_name {
            return Err(NetEcuError::UnknownMessage(message.to_string()));
        }
        if !self.message.fields.contains_key(field) {
            return Err(NetEcuError::UnknownField(field.to_string()));
        }
        self.overrides.insert(field.to_string(), value);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    use crate::netmap::{FieldDef, FieldType};

    fn message() -> MessageDef {
        MessageDef {
            dst: "192.168.1.50:5000".parse().unwrap(),
            length: 8,
            fields: BTreeMap::from([(
                "speed".to_string(),
                FieldDef {
                    offset: 0,
                    ty: FieldType::F32le,
                    factor: 1.0,
                    shift: 0.0,
                    values: BTreeMap::new(),
                },
            )]),
        }
    }

    #[test]
    fn config_ecu_emits_and_overrides() {
        let src: SocketAddr = "192.168.1.10:5000".parse().unwrap();
        let mut ecu = UdpConfigEcu::new(
            "joystick".into(),
            src,
            "DriveCommand",
            message(),
            100_000,
            BTreeMap::new(),
        );
        let mut out = Vec::new();
        ecu.update(0, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].dst, message().dst);
        assert_eq!(out[0].src, src);

        ecu.set_field("DriveCommand", "speed", SignalValue::Num(1.5))
            .unwrap();
        let mut out = Vec::new();
        ecu.update(100_000, &mut out);
        let speed = f32::from_le_bytes([
            out[0].payload[0],
            out[0].payload[1],
            out[0].payload[2],
            out[0].payload[3],
        ]);
        assert!((speed - 1.5).abs() < 1e-6);
    }

    #[test]
    fn config_ecu_rejects_unknown_message_or_field() {
        let mut ecu = UdpConfigEcu::new(
            "joystick".into(),
            "192.168.1.10:5000".parse().unwrap(),
            "DriveCommand",
            message(),
            100_000,
            BTreeMap::new(),
        );
        assert!(matches!(
            ecu.set_field("Bogus", "speed", SignalValue::Num(1.0)),
            Err(NetEcuError::UnknownMessage(_))
        ));
        assert!(matches!(
            ecu.set_field("DriveCommand", "bogus", SignalValue::Num(1.0)),
            Err(NetEcuError::UnknownField(_))
        ));
    }

    #[test]
    fn registry_unknown_ecu_fails_clearly() {
        let registry = NetRegistry::<UdpDatagram>::new();
        let err = match registry.create("motion", 100_000) {
            Ok(_) => panic!("expected an error"),
            Err(e) => e,
        };
        let message = err.to_string();
        assert!(
            message.contains("no firmware registered for SIL ECU `motion`"),
            "got: {message}"
        );
    }

    #[test]
    fn closure_factory_is_a_factory() {
        struct Dummy;
        impl NetEcu<UdpDatagram> for Dummy {
            fn name(&self) -> &str {
                "dummy"
            }
        }
        let mut registry = NetRegistry::<UdpDatagram>::new();
        registry.register(
            "motion",
            |_name: &str, _budget: u64| -> Result<Box<dyn NetEcu<UdpDatagram>>, NetEcuError> {
                Ok(Box::new(Dummy))
            },
        );
        let ecu = registry.create("motion", 100_000).unwrap();
        assert_eq!(ecu.name(), "dummy");
    }
}
