//! YAML vehicle configuration and the config-driven ECU.
//!
//! A `VehicleConfig` describes the DBC file to load, the simulation step and
//! the list of vECUs. The "config" ECU kind transmits a fixed message on a
//! fixed period, with signal values that can be overridden at runtime (used
//! by the battery, brake pedal and driver-request nodes).
//!
//! Ethernet-only vehicles omit `dbc` entirely and instead list UDP config
//! ECUs under `eth_ecus` and their networks under `networks`.

use std::collections::BTreeMap;
use std::net::SocketAddr;

use embrig_core::codec::MessageCodec;
use embrig_core::frame::CanFrame;
use embrig_core::signal::SignalValue;
use embrig_core::time::Timestamp;
use embrig_core::{EcuError, NetEcu};
use serde::{Deserialize, Serialize};

const DEFAULT_STEP_US: u64 = 1_000;
const DEFAULT_PERIOD_US: u64 = 100_000;
const DEFAULT_STEP_BUDGET_US: u64 = 100_000;

/// Top-level vehicle definition (the `vehicle.yaml` file).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleConfig {
    pub name: String,
    /// Path to the DBC file (relative to the vehicle.yaml location).
    ///
    /// Ethernet-only projects (no CAN bus) omit this field entirely.
    #[serde(default)]
    pub dbc: String,
    /// Simulation step in microseconds.
    #[serde(default = "default_step_us")]
    pub step_us: u64,
    #[serde(default)]
    pub ecus: Vec<EcuConfig>,
    /// Ethernet vECUs (UDP networks).
    #[serde(default)]
    pub eth_ecus: Vec<EthEcuConfig>,
    /// Ethernet networks (UDP) referenced by `interfaces`.
    #[serde(default)]
    pub networks: Vec<NetworkConfig>,
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
    /// Wall-clock budget (µs) allowed for one simulated step of this ECU
    /// (`sil` nodes only). Exceeding it fails the test.
    #[serde(default = "default_step_budget_us")]
    pub step_budget_us: u64,
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
    /// The system under test for software-in-the-loop: firmware compiled for
    /// the host. The implementation is bound in code via a
    /// [`NetEcuFactory`](embrig_core::network::NetEcuFactory).
    Sil,
}

/// A YAML scalar signal value: number, boolean, or symbolic string.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum SignalLiteral {
    Num(f64),
    Bool(bool),
    Str(String),
}

/// A single Ethernet (UDP) vECU entry in `vehicle.yaml`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EthEcuConfig {
    pub name: String,
    #[serde(rename = "type")]
    pub kind: EthEcuKind,
    /// This ECU's socket address; datagrams destined to it are delivered here.
    pub address: SocketAddr,
    /// For `udp-config` ECUs: name of the netmap message to transmit.
    #[serde(default)]
    pub message: Option<String>,
    /// Transmit period in microseconds.
    #[serde(default = "default_period_us")]
    pub period_us: u64,
    /// Initial physical field values (numeric or symbolic).
    #[serde(default)]
    pub fields: BTreeMap<String, SignalLiteral>,
    /// Wall-clock budget (µs) allowed for one simulated step of this ECU
    /// (`udp-sil` nodes only). Exceeding it fails the test.
    #[serde(default = "default_step_budget_us")]
    pub step_budget_us: u64,
}

/// Which built-in behaviour an Ethernet vECU implements.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EthEcuKind {
    /// Fixed netmap messages with runtime-overridable fields.
    #[serde(rename = "udp-config")]
    UdpConfig,
    /// The system under test for software-in-the-loop: firmware compiled for
    /// the host, bound in code via a
    /// [`NetEcuFactory`](embrig_core::network::NetEcuFactory).
    #[serde(rename = "udp-sil")]
    UdpSil,
}

/// An Ethernet network of a vehicle.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetworkConfig {
    pub name: String,
    /// `udp` (the only kind today).
    #[serde(rename = "type")]
    pub kind: String,
    /// The host (test rig) endpoint. Tests bind here; received telemetry is
    /// destined to this address.
    pub host: SocketAddr,
    /// Path to the netmap file, relative to the vehicle.yaml location.
    pub netmap: String,
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
fn default_step_budget_us() -> u64 {
    DEFAULT_STEP_BUDGET_US
}

impl VehicleConfig {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            dbc: String::new(),
            step_us: DEFAULT_STEP_US,
            ecus: Vec::new(),
            eth_ecus: Vec::new(),
            networks: Vec::new(),
            interfaces: Vec::new(),
        }
    }
}

/// Convert a YAML literal to a physical value for a signal.
pub fn literal_to_physical(
    message: &dyn MessageCodec,
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

/// A config-driven vECU: periodically transmits a fixed message with the
/// configured signal values.
///
/// [`Ecu::set_signal`] overrides a signal for the next transmission, which is
/// how tests inject stimulus (e.g. an over-voltage on the battery bus).
///
/// The transmitted message is a boxed [`MessageCodec`], so a config node can
/// speak any protocol (DBC today, CANopen for the SIL demo).
pub struct ConfigEcu {
    name: String,
    codec: Box<dyn MessageCodec>,
    values: BTreeMap<String, f64>,
    period: Timestamp,
    next: Timestamp,
}

impl ConfigEcu {
    pub fn new(
        name: impl Into<String>,
        codec: Box<dyn MessageCodec>,
        period: Timestamp,
        initial: &BTreeMap<String, SignalLiteral>,
    ) -> Result<Self, EcuError> {
        let name = name.into();
        let mut values = BTreeMap::new();
        for (sig, lit) in initial {
            let v = literal_to_physical(&*codec, sig, lit)?;
            codec
                .check_value(sig, v)
                .map_err(|e| EcuError::InvalidValue(e.to_string()))?;
            values.insert(sig.clone(), v);
        }
        Ok(Self {
            name,
            codec,
            values,
            period,
            next: 0,
        })
    }
}

impl NetEcu<CanFrame> for ConfigEcu {
    fn name(&self) -> &str {
        &self.name
    }

    fn update(&mut self, time: Timestamp, out: &mut Vec<CanFrame>) {
        if time < self.next {
            return;
        }
        let values: Vec<(&str, f64)> = self.values.iter().map(|(k, v)| (k.as_str(), *v)).collect();
        if let Ok(data) = self.codec.encode_signals(&values) {
            if let Ok(frame) = CanFrame::new(self.codec.id(), data) {
                out.push(frame);
            }
        }
        self.next = time + self.period;
    }

    fn set_signal(&mut self, id: u32, signal: &str, value: SignalValue) -> Result<(), EcuError> {
        if id != self.codec.id() {
            return Err(EcuError::UnknownMessage(format!("0x{id:03X}")));
        }
        let physical = match value {
            SignalValue::Num(v) => v,
            SignalValue::Str(s) => self.codec.physical_for_symbol(signal, &s).ok_or_else(|| {
                EcuError::InvalidValue(format!("unknown symbol `{s}` for `{signal}`"))
            })?,
        };
        self.codec
            .check_value(signal, physical)
            .map_err(|e| EcuError::InvalidValue(e.to_string()))?;
        self.values.insert(signal.to_string(), physical);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embrig_core::network::CanSimExt;
    use embrig_dbc::{ByteOrder, MessageDef};

    fn message() -> MessageDef {
        MessageDef {
            id: 0x100,
            name: "BatteryStatus".into(),
            dlc: 8,
            signals: vec![
                embrig_dbc::SignalDef {
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
                embrig_dbc::SignalDef {
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
        let ecu = ConfigEcu::new(
            "battery",
            Box::new(message()),
            embrig_core::time::ms(100),
            &initial,
        )
        .unwrap();
        let mut sim = embrig_core::Simulation::new(embrig_core::time::US_PER_MS);
        sim.attach(Box::new(ecu), &[]);
        sim.run_ms(100);
        assert_eq!(sim.frame_counts(), vec![(0x100, 1)]);
        let frame = sim.recorder().last_message(&0x100).unwrap();
        let decoded = message().decode_signals(&frame.data).unwrap();
        assert_eq!(decoded[0].value, 400.0);
        assert_eq!(decoded[1].symbol.as_deref(), Some("READY"));
    }

    #[test]
    fn set_signal_overrides_next_frame() {
        let mut initial = BTreeMap::new();
        initial.insert("voltage".into(), SignalLiteral::Num(400.0));
        let ecu = ConfigEcu::new(
            "battery",
            Box::new(message()),
            embrig_core::time::ms(100),
            &initial,
        )
        .unwrap();
        let mut sim = embrig_core::Simulation::new(embrig_core::time::US_PER_MS);
        sim.attach(Box::new(ecu), &[]);
        sim.set_signal(0, 0x100, "voltage", SignalValue::Num(500.0))
            .unwrap();
        sim.run_ms(100);
        let frame = sim.recorder().last_message(&0x100).unwrap();
        assert_eq!(
            message().decode_signal(&frame.data, "voltage").unwrap(),
            500.0
        );
    }

    #[test]
    fn set_signal_errors_on_bad_symbol() {
        let mut initial = BTreeMap::new();
        initial.insert("state".into(), SignalLiteral::Str("READY".into()));
        let mut ecu = ConfigEcu::new(
            "battery",
            Box::new(message()),
            embrig_core::time::ms(100),
            &initial,
        )
        .unwrap();
        let err = ecu.set_signal(0x100, "state", SignalValue::Str("NOPE".into()));
        assert_eq!(
            err,
            Err(EcuError::InvalidValue(
                "unknown symbol `NOPE` for `state`".into()
            ))
        );
    }

    #[test]
    fn parses_ethernet_only_vehicle() {
        let cfg: VehicleConfig = serde_saphyr::from_str(
            r#"
name: rover
step_us: 1000
eth_ecus:
  - name: joystick
    type: udp-config
    address: 192.168.1.20:6000
    message: Joystick
    period_us: 20000
  - name: motion
    type: udp-sil
    address: 192.168.1.30:5000
    step_budget_us: 100000
networks:
  - name: eth
    type: udp
    host: 192.168.1.10:5000
    netmap: netmap.yaml
interfaces:
  - name: virtual
    type: virtual
  - name: udp
    type: udp
"#,
        )
        .unwrap();
        assert!(cfg.dbc.is_empty());
        assert_eq!(cfg.eth_ecus.len(), 2);
        assert_eq!(
            cfg.eth_ecus[0].address,
            "192.168.1.20:6000".parse().unwrap()
        );
        assert_eq!(cfg.eth_ecus[0].kind, EthEcuKind::UdpConfig);
        assert_eq!(cfg.eth_ecus[1].kind, EthEcuKind::UdpSil);
        assert_eq!(cfg.networks[0].kind, "udp");
        assert_eq!(cfg.networks[0].host, "192.168.1.10:5000".parse().unwrap());
        assert_eq!(cfg.eth_ecus[0].period_us, 20_000);
        assert_eq!(cfg.eth_ecus[1].step_budget_us, 100_000);
    }
}
