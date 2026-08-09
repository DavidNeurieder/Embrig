//! The YAML test DSL: parsing and validation of test files.

use std::path::Path;

use openhil_core::time::{Timestamp, US_PER_MS, US_PER_S};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// A single test: a named sequence of steps with an overall time budget.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestSpec {
    pub name: String,
    /// Wall/simulation time budget for the whole test (e.g. `5s`).
    #[serde(default = "default_timeout")]
    pub timeout: String,
    #[serde(default)]
    pub steps: Vec<Step>,
}

fn default_timeout() -> String {
    "5s".to_string()
}

/// A step in a test.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Step {
    /// Transmit a raw frame on the bus (data is a byte list, max 8 bytes).
    Send { id: u32, data: Vec<u8> },
    /// Override a signal of a vECU for its next transmission.
    SetSignal {
        ecu: String,
        id: u32,
        signal: String,
        value: ExpectedValue,
    },
    /// Advance the simulation / sleep on hardware by a duration.
    Wait { time: String },
    /// Assert a condition on a received frame.
    Expect {
        #[serde(flatten)]
        spec: ExpectStep,
    },
    /// Inject a fault on a frame id, optionally windowed.
    Fault {
        #[serde(rename = "type")]
        kind: FaultKind,
        id: u32,
        /// Delay amount for `delay` faults (e.g. `5ms`).
        #[serde(default)]
        delay: Option<String>,
        /// Byte index for `corrupt` faults.
        #[serde(default)]
        byte: Option<usize>,
        /// Bit mask for `corrupt` faults.
        #[serde(default)]
        mask: Option<u8>,
        #[serde(default)]
        start: Option<String>,
        #[serde(default)]
        duration: Option<String>,
    },
}

/// A fault kind, matching the fault model of `openhil-core`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultKind {
    /// Suppress every frame with the id.
    Drop,
    /// Hold frames back by a fixed delay (see the `delay` field).
    Delay,
    /// Flip bits in one byte of every frame (see `byte`/`mask`).
    Corrupt,
}

/// A YAML scalar value used as an expected signal value.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ExpectedValue {
    Num(f64),
    Bool(bool),
    Str(String),
}

impl ExpectedValue {
    pub fn describe(&self) -> String {
        match self {
            ExpectedValue::Num(v) => format!("{v}"),
            ExpectedValue::Bool(b) => b.to_string(),
            ExpectedValue::Str(s) => format!("`{s}`"),
        }
    }
}

/// The fields of an `expect` step. Exactly one of
/// `equals`/`greater_than`/`less_than`/`present` must be set.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ExpectStep {
    pub id: u32,
    /// Signal name to assert on (required unless `present` is used).
    #[serde(default)]
    pub signal: Option<String>,
    #[serde(default)]
    pub equals: Option<ExpectedValue>,
    #[serde(default)]
    pub greater_than: Option<f64>,
    #[serde(default)]
    pub less_than: Option<f64>,
    /// `present: true` — a frame with the id must appear within `within`;
    /// `present: false` — it must not appear.
    #[serde(default)]
    pub present: Option<bool>,
    /// Time budget to poll within (e.g. `1s`). Without it the assertion is
    /// checked once against the current bus state.
    #[serde(default)]
    pub within: Option<String>,
}

impl ExpectStep {
    pub fn validate(&self) -> Result<(), DslError> {
        let ops = usize::from(self.equals.is_some())
            + usize::from(self.greater_than.is_some())
            + usize::from(self.less_than.is_some())
            + usize::from(self.present.is_some());
        if ops != 1 {
            return Err(DslError::BadExpect {
                id: self.id,
                message: "exactly one of equals/greater_than/less_than/present must be set".into(),
            });
        }
        if self.present.is_none() && self.signal.is_none() {
            return Err(DslError::BadExpect {
                id: self.id,
                message: "a signal name is required unless `present` is used".into(),
            });
        }
        if let Some(w) = &self.within {
            parse_duration(w)?;
        }
        Ok(())
    }
}

/// Parse a duration string like `250us`, `10ms` or `2s` into microseconds.
pub fn parse_duration(input: &str) -> Result<Timestamp, DslError> {
    let input = input.trim();
    let split = input
        .find(|c: char| !c.is_ascii_digit())
        .ok_or_else(|| DslError::BadDuration {
            input: input.to_string(),
            message: "missing unit (us, ms or s)".into(),
        })?;
    let (num, unit) = input.split_at(split);
    let value: Timestamp = num.parse().map_err(|_| DslError::BadDuration {
        input: input.to_string(),
        message: format!("`{num}` is not an integer"),
    })?;
    match unit {
        "us" | "µs" => Ok(value),
        "ms" => Ok(value * US_PER_MS),
        "s" => Ok(value * US_PER_S),
        other => Err(DslError::BadDuration {
            input: input.to_string(),
            message: format!("unknown unit `{other}` (use us, ms or s)"),
        }),
    }
}

/// Errors from loading or validating a test definition.
#[derive(Debug, Error)]
pub enum DslError {
    #[error("invalid duration `{input}`: {message}")]
    BadDuration { input: String, message: String },
    #[error("failed to read test file `{path}`: {message}")]
    Load { path: String, message: String },
    #[error("invalid expect step for id 0x{id:03X}: {message}")]
    BadExpect { id: u32, message: String },
    #[error("invalid `{kind}` fault: {message}")]
    BadFault { kind: String, message: String },
}

/// Load and validate a single test file.
pub fn load_spec(path: &Path) -> Result<TestSpec, DslError> {
    let text = std::fs::read_to_string(path).map_err(|e| DslError::Load {
        path: path.display().to_string(),
        message: e.to_string(),
    })?;
    let spec: TestSpec = serde_saphyr::from_str(&text).map_err(|e| DslError::Load {
        path: path.display().to_string(),
        message: e.to_string(),
    })?;
    for step in &spec.steps {
        if let Step::Expect { spec } = step {
            spec.validate()?;
        }
        if let Step::Wait { time } = step {
            parse_duration(time)?;
        }
        if let Step::Fault {
            kind,
            delay,
            byte,
            mask,
            start,
            duration,
            ..
        } = step
        {
            if let Some(d) = duration {
                parse_duration(d)?;
            }
            if let Some(s) = start {
                parse_duration(s)?;
            }
            match kind {
                FaultKind::Delay if delay.is_none() => {
                    return Err(DslError::BadFault {
                        kind: "delay".into(),
                        message: "a `delay` duration is required".into(),
                    })
                }
                FaultKind::Corrupt if byte.is_none() || mask.is_none() => {
                    return Err(DslError::BadFault {
                        kind: "corrupt".into(),
                        message: "both `byte` and `mask` are required".into(),
                    })
                }
                FaultKind::Delay => {
                    parse_duration(delay.as_deref().unwrap())?;
                }
                _ => {}
            }
        }
    }
    Ok(spec)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn write_tmp(name: &str, content: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("openhil-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(content.as_bytes()).unwrap();
        path
    }

    #[test]
    fn parses_full_spec() {
        let path = write_tmp(
            "full.yaml",
            r#"
name: overvoltage_disables_motor
timeout: 5s
steps:
  - wait: { time: 100ms }
  - set_signal: { ecu: battery, id: 0x100, signal: voltage, value: 460.0 }
  - expect: { id: 0x220, signal: motor_enable, equals: false, within: 1s }
  - expect: { id: 0x100, present: true, within: 500ms }
  - expect: { id: 0x230, signal: state, greater_than: 2.0, within: 1s }
  - expect: { id: 0x230, signal: state, less_than: 4.0 }
  - expect: { id: 0x230, signal: state, equals: "SAFE" }
  - expect: { id: 0x230, signal: enabled, equals: true }
  - fault: { type: drop, id: 0x100, duration: 100ms }
  - fault: { type: delay, id: 0x100, delay: 5ms, duration: 20ms }
  - fault: { type: corrupt, id: 0x100, byte: 0, mask: 0xFF }
  - send: { id: 0x200, data: [1, 0, 0, 0, 0, 0, 0, 0] }
"#,
        );
        let spec = load_spec(&path).unwrap();
        assert_eq!(spec.name, "overvoltage_disables_motor");
        assert_eq!(spec.steps.len(), 12);
    }

    #[test]
    fn rejects_ambiguous_expect() {
        let path = write_tmp(
            "bad.yaml",
            "name: bad\nsteps:\n  - expect: { id: 0x100, signal: a, equals: 1, present: true }\n",
        );
        assert!(load_spec(&path).is_err());
    }

    #[test]
    fn rejects_missing_signal() {
        let path = write_tmp(
            "bad2.yaml",
            "name: bad\nsteps:\n  - expect: { id: 0x100, equals: 1 }\n",
        );
        assert!(load_spec(&path).is_err());
    }

    #[test]
    fn duration_parsing() {
        assert_eq!(parse_duration("250us").unwrap(), 250);
        assert_eq!(parse_duration("10ms").unwrap(), 10 * US_PER_MS);
        assert_eq!(parse_duration("2s").unwrap(), 2 * US_PER_S);
        assert!(parse_duration("5").is_err());
        assert!(parse_duration("1m").is_err());
    }
}
