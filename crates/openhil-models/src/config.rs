//! YAML vehicle configuration and the config-driven ECU.
//!
//! A `VehicleConfig` describes the DBC file to load, the simulation step and
//! the list of vECUs. The "config" ECU kind transmits a fixed message on a
//! fixed period, with signal values that can be overridden at runtime (used
//! by the battery, brake pedal and driver-request nodes).

use std::collections::BTreeMap;

use openhil_core::frame::CanFrame;
use openhil_core::signal::SignalValue;
use openhil_core::time::Timestamp;
use openhil_core::{Ecu, EcuError};
use openhil_dbc::MessageDef;
use serde::{Deserialize, Serialize};

const DEFAULT_STEP_US: u64 = 1_000;
const DEFAULT_PERIOD_US: u64 = 100_000;

/// Top-level vehicle definition (the `vehicle.yaml` file).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleConfig {
    pub name: String,
    /// Path to the DBC file (relative to the vehicle.yaml location).
    pub dbc: String,
    /// Simulation step in microseconds.
    #[serde(default = "default_step_us")]
    pub step_us: u64,
    #[serde(default)]
    pub ecus: Vec<EcuConfig>,
    #[serde(default)]
    pub interfaces: Vec<InterfaceConfig>,
}

/// A single vECU entry in `vehicle.yaml`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcuConfig {
    pub name: String,
    #[serde(rename = "type")]
    pub kind: EcuKind,
    /// For `config` ECUs: name of the DBC message to transmit.
    #[serde(default)]
    pub message: Option<String>,
    /// Transmit period in microseconds.
    #[serde(default = "default_period_us")]
    pub period_us: u64,
    /// Initial physical signal values (numeric or symbolic).
    #[serde(default)]
    pub signals: BTreeMap<String, SignalLiteral>,
    /// Frame ids this ECU receives.
    #[serde(default)]
    pub listen: Vec<u32>,
}

/// Which built-in behaviour an ECU implements.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EcuKind {
    /// Fixed messages with runtime-overridable signals.
    Config,
    /// Charger state machine (uses 0x200 / 0x100 / 0x210).
    Charger,
    /// The ECU under test: computes `motor_enable` from its inputs.
    Vcu,
    /// Motor: RUNNING when enabled, otherwise SAFE.
    Motor,
}

/// A YAML scalar signal value: number, boolean, or symbolic string.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum SignalLiteral {
    Num(f64),
    Bool(bool),
    Str(String),
}

/// A target the simulation can be connected to.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterfaceConfig {
    pub name: String,
    /// `virtual` or `socketcan`.
    #[serde(rename = "type")]
    pub kind: String,
    /// SocketCAN device, e.g. `vcan0`.
    #[serde(default)]
    pub interface: Option<String>,
}

fn default_step_us() -> u64 {
    DEFAULT_STEP_US
}
fn default_period_us() -> u64 {
    DEFAULT_PERIOD_US
}

impl VehicleConfig {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            dbc: String::new(),
            step_us: DEFAULT_STEP_US,
            ecus: Vec::new(),
            interfaces: Vec::new(),
        }
    }
}

/// Convert a YAML literal to a physical value for a signal.
pub fn literal_to_physical(
    message: &MessageDef,
    name: &str,
    lit: &SignalLiteral,
) -> Result<f64, EcuError> {
    match lit {
        SignalLiteral::Num(v) => Ok(*v),
        SignalLiteral::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        SignalLiteral::Str(s) => message
            .physical_for_symbol(name, s)
            .ok_or_else(|| EcuError::InvalidValue(format!("unknown symbol `{s}` for `{name}`"))),
    }
}

/// A config-driven vECU: periodically transmits a fixed DBC message with the
/// configured signal values.
///
/// [`Ecu::set_signal`] overrides a signal for the next transmission, which is
/// how tests inject stimulus (e.g. an over-voltage on the battery bus).
pub struct ConfigEcu {
    name: String,
    message: MessageDef,
    values: BTreeMap<String, f64>,
    period: Timestamp,
    next: Timestamp,
}

impl ConfigEcu {
    pub fn new(
        name: impl Into<String>,
        message: MessageDef,
        period: Timestamp,
        initial: &BTreeMap<String, SignalLiteral>,
    ) -> Result<Self, EcuError> {
        let name = name.into();
        let mut values = BTreeMap::new();
        for (sig, lit) in initial {
            let v = literal_to_physical(&message, sig, lit)?;
            message
                .check_value(sig, v)
                .map_err(|e| EcuError::InvalidValue(e.to_string()))?;
            values.insert(sig.clone(), v);
        }
        Ok(Self {
            name,
            message,
            values,
            period,
            next: 0,
        })
    }
}

impl Ecu for ConfigEcu {
    fn name(&self) -> &str {
        &self.name
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if time < self.next {
            return;
        }
        let values: Vec<(&str, f64)> = self.values.iter().map(|(k, v)| (k.as_str(), *v)).collect();
        if let Ok(data) = self.message.encode_signals(&values) {
            if let Ok(frame) = CanFrame::new(self.message.id, data) {
                out.push(frame);
            }
        }
        self.next = time + self.period;
    }

    fn set_signal(&mut self, id: u32, signal: &str, value: SignalValue) -> Result<(), EcuError> {
        if id != self.message.id {
            return Err(EcuError::UnknownMessage(id));
        }
        let physical = match value {
            SignalValue::Num(v) => v,
            SignalValue::Str(s) => {
                self.message
                    .physical_for_symbol(signal, &s)
                    .ok_or_else(|| {
                        EcuError::InvalidValue(format!("unknown symbol `{s}` for `{signal}`"))
                    })?
            }
        };
        self.message
            .check_value(signal, physical)
            .map_err(|e| EcuError::InvalidValue(e.to_string()))?;
        self.values.insert(signal.to_string(), physical);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openhil_dbc::ByteOrder;

    fn message() -> MessageDef {
        MessageDef {
            id: 0x100,
            name: "BatteryStatus".into(),
            dlc: 8,
            signals: vec![
                openhil_dbc::SignalDef {
                    name: "voltage".into(),
                    start_bit: 0,
                    length: 16,
                    byte_order: ByteOrder::Intel,
                    is_signed: false,
                    factor: 0.1,
                    offset: 0.0,
                    unit: "V".into(),
                    min: Some(0.0),
                    max: Some(600.0),
                    value_table: BTreeMap::new(),
                },
                openhil_dbc::SignalDef {
                    name: "state".into(),
                    start_bit: 16,
                    length: 4,
                    byte_order: ByteOrder::Intel,
                    is_signed: false,
                    factor: 1.0,
                    offset: 0.0,
                    unit: String::new(),
                    min: Some(0.0),
                    max: Some(4.0),
                    value_table: BTreeMap::from([(2, "READY".into()), (3, "CHARGING".into())]),
                },
            ],
        }
    }

    #[test]
    fn transmits_on_period() {
        let mut initial = BTreeMap::new();
        initial.insert("voltage".into(), SignalLiteral::Num(400.0));
        initial.insert("state".into(), SignalLiteral::Str("READY".into()));
        let ecu =
            ConfigEcu::new("battery", message(), openhil_core::time::ms(100), &initial).unwrap();
        let mut sim = openhil_core::Simulation::new(openhil_core::time::US_PER_MS);
        sim.attach(Box::new(ecu), &[]);
        sim.run_ms(100);
        assert_eq!(sim.frame_counts(), vec![(0x100, 1)]);
        let frame = sim.recorder().last_frame(0x100).unwrap();
        let decoded = message().decode_signals(&frame.data).unwrap();
        assert_eq!(decoded[0].value, 400.0);
        assert_eq!(decoded[1].symbol.as_deref(), Some("READY"));
    }

    #[test]
    fn set_signal_overrides_next_frame() {
        let mut initial = BTreeMap::new();
        initial.insert("voltage".into(), SignalLiteral::Num(400.0));
        let ecu =
            ConfigEcu::new("battery", message(), openhil_core::time::ms(100), &initial).unwrap();
        let mut sim = openhil_core::Simulation::new(openhil_core::time::US_PER_MS);
        sim.attach(Box::new(ecu), &[]);
        sim.set_signal(0, 0x100, "voltage", SignalValue::Num(500.0))
            .unwrap();
        sim.run_ms(100);
        let frame = sim.recorder().last_frame(0x100).unwrap();
        assert_eq!(
            message().decode_signal(&frame.data, "voltage").unwrap(),
            500.0
        );
    }

    #[test]
    fn set_signal_errors_on_bad_symbol() {
        let mut initial = BTreeMap::new();
        initial.insert("state".into(), SignalLiteral::Str("READY".into()));
        let mut ecu =
            ConfigEcu::new("battery", message(), openhil_core::time::ms(100), &initial).unwrap();
        let err = ecu.set_signal(0x100, "state", SignalValue::Str("NOPE".into()));
        assert_eq!(
            err,
            Err(EcuError::InvalidValue(
                "unknown symbol `NOPE` for `state`".into()
            ))
        );
    }
}
