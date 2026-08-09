//! Signal value handling.
//!
//! A signal value can be numeric (the physical value, in DBC terms) or a
//! symbolic string matched against a DBC `VAL_` value table.

#[derive(Debug, Clone, PartialEq)]
pub enum SignalValue {
    /// A physical value (already scaled by factor/offset).
    Num(f64),
    /// A symbolic value, e.g. a state name from a `VAL_` table.
    Str(String),
}

impl SignalValue {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            SignalValue::Num(v) => Some(*v),
            SignalValue::Str(_) => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            SignalValue::Num(_) => None,
            SignalValue::Str(s) => Some(s),
        }
    }
}

impl From<f64> for SignalValue {
    fn from(v: f64) -> Self {
        SignalValue::Num(v)
    }
}

impl From<bool> for SignalValue {
    fn from(v: bool) -> Self {
        SignalValue::Num(if v { 1.0 } else { 0.0 })
    }
}

impl From<&str> for SignalValue {
    fn from(v: &str) -> Self {
        SignalValue::Str(v.to_string())
    }
}

impl From<String> for SignalValue {
    fn from(v: String) -> Self {
        SignalValue::Str(v)
    }
}
