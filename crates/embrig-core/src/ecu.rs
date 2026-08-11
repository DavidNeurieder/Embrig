//! Virtual ECUs.
//!
//! The generic [`NetEcu`] trait is the single ECU interface for every
//! transport: implementations are stepped in insertion order every simulation
//! tick, produce outgoing messages in [`NetEcu::update`] and react to
//! subscribed messages in [`NetEcu::on_message`]. CAN firmware implements
//! `NetEcu<CanFrame>`; UDP and TCP firmware implement it against their own
//! message types.
//!
//! [`EcuError`] is a CAN-flavoured alias for the unified [`NetEcuError`],
//! kept so existing firmware written before the transports were unified keeps
//! compiling.

pub use crate::network::{NetEcu, NetEcuError};

/// CAN-flavoured alias for [`NetEcuError`].
pub type EcuError = NetEcuError;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::CanFrame;
    use crate::signal::SignalValue;

    struct NoopEcu;

    impl NetEcu<CanFrame> for NoopEcu {
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
