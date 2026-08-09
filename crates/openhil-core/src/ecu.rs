use crate::frame::CanFrame;
use crate::signal::SignalValue;
use crate::time::Timestamp;

/// A virtual ECU.
///
/// Implementations are stepped in insertion order every simulation tick.
/// [`Ecu::update`] is called each tick to advance behaviour and produce
/// outgoing frames; [`Ecu::on_message`] is called when a subscribed frame is
/// delivered to this ECU.
pub trait Ecu: Send {
    /// Stable name used in reports and error messages.
    fn name(&self) -> &str;

    /// Advance the ECU's internal state to `time`.
    fn update(&mut self, _time: Timestamp, _out: &mut Vec<CanFrame>) {}

    /// Handle a received frame.
    fn on_message(&mut self, _frame: &CanFrame, _time: Timestamp) {}

    /// Override a signal value (used by tests to inject stimulus).
    fn set_signal(&mut self, _id: u32, _signal: &str, _value: SignalValue) -> Result<(), EcuError> {
        Err(EcuError::SignalNotSupported)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EcuError {
    /// This ECU does not support runtime signal overrides.
    SignalNotSupported,
    /// No such message id on this ECU.
    UnknownMessage(u32),
    /// No such signal on this ECU/message.
    UnknownSignal(String),
    /// The value (or symbol) cannot be encoded for this signal.
    InvalidValue(String),
}

impl std::fmt::Display for EcuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EcuError::SignalNotSupported => {
                write!(f, "signal override not supported by this ECU")
            }
            EcuError::UnknownMessage(id) => write!(f, "no message 0x{id:03X} on this ECU"),
            EcuError::UnknownSignal(sig) => write!(f, "no signal `{sig}` on this message"),
            EcuError::InvalidValue(v) => write!(f, "cannot encode value for signal: {v}"),
        }
    }
}

impl std::error::Error for EcuError {}

#[cfg(test)]
mod tests {
    use super::*;

    struct NoopEcu;

    impl Ecu for NoopEcu {
        fn name(&self) -> &str {
            "noop"
        }
    }

    #[test]
    fn default_trait_methods_are_noops() {
        let mut ecu = NoopEcu;
        let mut out = Vec::new();
        ecu.update(1_000, &mut out);
        let f = CanFrame::new(0x100, vec![0; 8]).unwrap();
        ecu.on_message(&f, 1_000);
        assert!(out.is_empty());
        assert_eq!(
            ecu.set_signal(0x100, "x", SignalValue::Num(1.0)),
            Err(EcuError::SignalNotSupported)
        );
    }
}
