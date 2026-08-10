//! UDP virtual ECUs: the `UdpEcu` trait, the config-driven stimulus node, the
//! firmware factory registry and wall-clock budget enforcement.

use std::collections::{BTreeMap, HashMap};
use std::net::SocketAddr;
use std::time::Instant;

use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;

use crate::datagram::UdpDatagram;
use crate::netmap::MessageDef;

/// A virtual UDP ECU.
///
/// Implementations are stepped in insertion order every simulation tick.
/// [`UdpEcu::update`] is called each tick to advance behaviour and produce
/// outgoing datagrams; [`UdpEcu::on_datagram`] is called when a datagram
/// addressed to this ECU's endpoint is delivered.
pub trait UdpEcu: Send {
    /// Stable name used in reports and error messages.
    fn name(&self) -> &str;

    /// Advance the ECU's internal state to `time`.
    fn update(&mut self, _time: Timestamp, _out: &mut Vec<UdpDatagram>) {}

    /// Handle a received datagram.
    fn on_datagram(&mut self, _dg: &UdpDatagram, _time: Timestamp) {}

    /// Override a field value (used by tests to inject stimulus).
    fn set_field(
        &mut self,
        _message: &str,
        _field: &str,
        _value: SignalValue,
    ) -> Result<(), UdpEcuError> {
        Err(UdpEcuError::SignalNotSupported)
    }
}

/// Errors produced by UDP ECUs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UdpEcuError {
    /// This ECU does not support runtime field overrides.
    SignalNotSupported,
    /// No such message on this ECU.
    UnknownMessage(String),
    /// No such field on this message.
    UnknownField(String),
    /// The value (or symbol) cannot be encoded for this field.
    InvalidValue(String),
    /// No firmware implementation is registered for this SIL ECU.
    NotRegistered(String),
}

impl std::fmt::Display for UdpEcuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UdpEcuError::SignalNotSupported => {
                write!(f, "field override not supported by this ECU")
            }
            UdpEcuError::UnknownMessage(name) => write!(f, "no message `{name}` on this ECU"),
            UdpEcuError::UnknownField(name) => write!(f, "no field `{name}` on this message"),
            UdpEcuError::InvalidValue(v) => write!(f, "cannot encode value for field: {v}"),
            UdpEcuError::NotRegistered(name) => {
                write!(f, "no firmware registered for SIL ECU {name}")
            }
        }
    }
}

impl std::error::Error for UdpEcuError {}

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

    fn encode(&self) -> Result<Vec<u8>, UdpEcuError> {
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
            .map_err(|e| UdpEcuError::InvalidValue(e.to_string()))
    }
}

impl UdpEcu for UdpConfigEcu {
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
    ) -> Result<(), UdpEcuError> {
        if message != self.message_name {
            return Err(UdpEcuError::UnknownMessage(message.to_string()));
        }
        if !self.message.fields.contains_key(field) {
            return Err(UdpEcuError::UnknownField(field.to_string()));
        }
        self.overrides.insert(field.to_string(), value);
        Ok(())
    }
}

/// A firmware factory keyed by the `udp-sil` ECU name.
///
/// Factories are looked up (and re-invoked) every time the simulation is
/// built, i.e. once at startup and again on each test reset — so firmware
/// state never leaks between tests.
pub trait UdpEcuFactory: Send + Sync {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn UdpEcu>, UdpEcuError>;
}

/// A factory registry with no firmware: instantiating any `udp-sil` ECU fails.
#[derive(Default)]
pub struct NoFirmware;

impl UdpEcuFactory for NoFirmware {
    fn create(&self, name: &str, _step_budget_us: u64) -> Result<Box<dyn UdpEcu>, UdpEcuError> {
        Err(UdpEcuError::NotRegistered(name.to_string()))
    }
}

/// Lets callers register a firmware factory as a plain closure, e.g.
/// `registry.register("motion", |name, _budget| Ok(Box::new(Firmware::new(name))))`.
impl<F> UdpEcuFactory for F
where
    F: Fn(&str, u64) -> Result<Box<dyn UdpEcu>, UdpEcuError> + Send + Sync + 'static,
{
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn UdpEcu>, UdpEcuError> {
        self(name, step_budget_us)
    }
}

/// A firmware factory registry keyed by the `udp-sil` ECU name.
#[derive(Default)]
pub struct UdpRegistry {
    factories: HashMap<String, Box<dyn UdpEcuFactory>>,
}

impl UdpRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register the firmware for an ECU.
    pub fn register(
        &mut self,
        name: impl Into<String>,
        factory: impl UdpEcuFactory + 'static,
    ) -> &mut Self {
        self.factories.insert(name.into(), Box::new(factory));
        self
    }

    /// The registered ECU names, sorted (for diagnostics).
    pub fn names(&self) -> Vec<String> {
        let mut names: Vec<String> = self.factories.keys().cloned().collect();
        names.sort();
        names
    }
}

impl UdpEcuFactory for UdpRegistry {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn UdpEcu>, UdpEcuError> {
        let factory = self.factories.get(name).ok_or_else(|| {
            UdpEcuError::NotRegistered(format!(
                "`{name}` (registered: {})",
                self.names().join(", ")
            ))
        })?;
        let inner = factory.create(name, step_budget_us)?;
        Ok(Box::new(BudgetedUdpEcu::new(inner, step_budget_us)))
    }
}

impl UdpEcuFactory for &UdpRegistry {
    fn create(&self, name: &str, step_budget_us: u64) -> Result<Box<dyn UdpEcu>, UdpEcuError> {
        UdpRegistry::create(self, name, step_budget_us)
    }
}

/// Wall-clock budget enforcement around one firmware ECU.
///
/// Panics if a single `update`/`on_datagram` call takes longer than
/// `budget_us`; the target converts that panic into a test failure.
struct BudgetedUdpEcu {
    inner: Box<dyn UdpEcu>,
    budget_us: u64,
    step_start: Instant,
}

impl BudgetedUdpEcu {
    fn new(inner: Box<dyn UdpEcu>, budget_us: u64) -> Self {
        Self {
            inner,
            budget_us,
            step_start: Instant::now(),
        }
    }
}

impl UdpEcu for BudgetedUdpEcu {
    fn name(&self) -> &str {
        self.inner.name()
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<UdpDatagram>) {
        self.step_start = Instant::now();
        self.inner.update(time, out);
        check_budget(self.inner.name(), self.budget_us, self.step_start);
    }

    fn on_datagram(&mut self, dg: &UdpDatagram, time: Timestamp) {
        self.step_start = Instant::now();
        self.inner.on_datagram(dg, time);
        check_budget(self.inner.name(), self.budget_us, self.step_start);
    }

    fn set_field(
        &mut self,
        message: &str,
        field: &str,
        value: SignalValue,
    ) -> Result<(), UdpEcuError> {
        self.inner.set_field(message, field, value)
    }
}

fn check_budget(name: &str, budget_us: u64, start: Instant) {
    let took_us = start.elapsed().as_micros() as u64;
    if took_us > budget_us {
        panic!("firmware `{name}` exceeded its {budget_us}µs step budget (took {took_us}µs)");
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
            Err(UdpEcuError::UnknownMessage(_))
        ));
        assert!(matches!(
            ecu.set_field("DriveCommand", "bogus", SignalValue::Num(1.0)),
            Err(UdpEcuError::UnknownField(_))
        ));
    }

    #[test]
    fn registry_unknown_ecu_fails_clearly() {
        let err = match UdpRegistry::new().create("motion", 100_000) {
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
        impl UdpEcu for Dummy {
            fn name(&self) -> &str {
                "dummy"
            }
        }
        let mut registry = UdpRegistry::new();
        registry.register(
            "motion",
            |_name: &str, _budget: u64| -> Result<Box<dyn UdpEcu>, UdpEcuError> {
                Ok(Box::new(Dummy))
            },
        );
        let ecu = registry.create("motion", 100_000).unwrap();
        assert_eq!(ecu.name(), "dummy");
    }
}
