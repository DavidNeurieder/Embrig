//! EDS-style node description used to build the CANopen codec.
//!
//! This is the CANopen analogue of a DBC file: it declares the node id and the
//! signal mappings of the PDOs. It is intentionally tiny — just what the
//! hand-rolled subset needs — and loaded from a small YAML file in the SIL
//! example fixtures.

use std::path::Path;

use serde::{Deserialize, Serialize};

/// One signal mapped into a PDO, packed little-endian like a DBC signal.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SignalSpec {
    pub name: String,
    /// Start bit in Intel (LSB) numbering.
    #[serde(default)]
    pub bit: u16,
    /// Signal width in bits.
    pub length: u16,
    #[serde(default)]
    pub is_signed: bool,
    /// Physical scale: `physical = raw * factor + offset`.
    #[serde(default = "factor_default")]
    pub factor: f64,
    #[serde(default)]
    pub offset: f64,
}

fn factor_default() -> f64 {
    1.0
}

/// A minimal CANopen node description (the `eds.yaml` in the SIL fixtures).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EcuSpec {
    /// CANopen node id, `1..=127`.
    pub node_id: u8,
    /// Heartbeat producer period in microseconds.
    #[serde(default)]
    pub heartbeat_period_us: u64,
    /// TPDO1 (`0x180 + node`) signal mapping, produced by the node.
    #[serde(default)]
    pub tpdo1: Vec<SignalSpec>,
    /// RPDO1 (`0x200 + node`) signal mapping, consumed by the node.
    #[serde(default)]
    pub rpdo1: Vec<SignalSpec>,
}

impl EcuSpec {
    /// Load an EDS from a YAML file.
    pub fn load(path: &Path) -> Result<Self, EcuSpecError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| EcuSpecError::Read(path.display().to_string(), e))?;
        Self::parse(&text)
    }

    /// Parse an EDS from YAML text.
    pub fn parse(text: &str) -> Result<Self, EcuSpecError> {
        serde_saphyr::from_str(text).map_err(|e| EcuSpecError::Parse(e.to_string()))
    }
}

/// Errors while loading or parsing an [`EcuSpec`].
#[derive(Debug, thiserror::Error)]
pub enum EcuSpecError {
    #[error("failed to read EDS file `{0}`: {1}")]
    Read(String, std::io::Error),
    #[error("invalid EDS: {0}")]
    Parse(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn omitted_factor_defaults_to_one_not_zero() {
        let spec: EcuSpec = serde_saphyr::from_str(
            r#"
node_id: 1
tpdo1:
  - name: valve_open
    bit: 0
    length: 1
rpdo1:
  - name: temperature
    bit: 0
    length: 16
"#,
        )
        .unwrap();
        assert_eq!(spec.tpdo1[0].factor, 1.0);
        assert_eq!(spec.tpdo1[0].offset, 0.0);
        assert_eq!(spec.rpdo1[0].factor, 1.0);
    }
}
