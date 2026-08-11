use crate::network::{NetAction, NetFault};
use crate::time::Timestamp;

/// A fault injected into the simulated bus.
///
/// Faults are time-windowed: they apply while
/// `start <= now < start + duration` (or forever when `duration` is `None`).
#[derive(Debug, Clone, PartialEq)]
pub enum Fault {
    /// Suppress all frames with the given id.
    DropFrame { id: u32 },
    /// Delay frames with the given id by a fixed amount.
    DelayFrame { id: u32, delay_us: Timestamp },
    /// Flip bits in one byte of frames with the given id.
    CorruptByte { id: u32, byte: usize, mask: u8 },
}

impl NetFault<u32> for Fault {
    fn matches(&self, key: &u32) -> bool {
        match self {
            Fault::DropFrame { id } => id == key,
            Fault::DelayFrame { id, .. } => id == key,
            Fault::CorruptByte { id, .. } => id == key,
        }
    }

    fn action(&self) -> NetAction {
        match self {
            Fault::DropFrame { .. } => NetAction::Drop,
            Fault::DelayFrame { delay_us, .. } => NetAction::Delay(*delay_us),
            Fault::CorruptByte { byte, mask, .. } => NetAction::Corrupt {
                byte: *byte,
                mask: *mask,
            },
        }
    }
}

/// A CAN fault rule bound to a time window.
pub type FaultRule = crate::network::NetFaultRule<Fault>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::ms;

    #[test]
    fn window_is_half_open() {
        let rule = FaultRule {
            fault: Fault::DropFrame { id: 0x100 },
            start: ms(10),
            duration: Some(ms(100)),
        };
        assert!(!rule.active_at(ms(9)));
        assert!(rule.active_at(ms(10)));
        assert!(rule.active_at(ms(109)));
        assert!(!rule.active_at(ms(110)));
    }

    #[test]
    fn unbounded_window() {
        let rule = FaultRule {
            fault: Fault::DelayFrame {
                id: 0x100,
                delay_us: ms(1),
            },
            start: 0,
            duration: None,
        };
        assert!(rule.active_at(ms(60_000)));
    }
}
