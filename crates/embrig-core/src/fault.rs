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

/// A fault rule bound to a time window.
#[derive(Debug, Clone, PartialEq)]
pub struct FaultRule {
    pub fault: Fault,
    pub start: Timestamp,
    pub duration: Option<Timestamp>,
}

impl FaultRule {
    /// Whether the rule is active at `now`.
    pub fn active_at(&self, now: Timestamp) -> bool {
        if now < self.start {
            return false;
        }
        match self.duration {
            None => true,
            Some(d) => now < self.start + d,
        }
    }
}

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
