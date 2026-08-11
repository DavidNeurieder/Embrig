use crate::fault::Fault;
use crate::frame::CanFrame;
use crate::network::NetworkSim;
use crate::time::Timestamp;

/// A CAN bus simulation.
///
/// The generic [`NetworkSim`] specialized for CAN: subscriptions are keyed by
/// frame id, faults are [`Fault`] and the recorder logs [`CanFrame`]s. The
/// public API (`attach`, `set_signal`, `run_ms`, `frame_counts`, `recorder`)
/// matches the interface firmware and the test layer relied on before the
/// transports were unified, so existing code keeps compiling unchanged.
pub type Simulation = NetworkSim<u32, CanFrame, Fault>;

/// Convenience mirror of the [`NetworkSim`] constructor used by existing code.
pub fn new_sim(step_us: Timestamp) -> Simulation {
    Simulation::new(step_us)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fault::FaultRule;
    use crate::network::{CanSimExt, NetEcu, NetMessage};
    use crate::signal::SignalValue;
    use crate::time::{ms, US_PER_MS};
    use std::sync::{Arc, Mutex};

    struct PeriodicSender {
        id: u32,
        every: Timestamp,
    }

    impl NetEcu<CanFrame> for PeriodicSender {
        fn name(&self) -> &str {
            "sender"
        }

        fn update(&mut self, now: Timestamp, out: &mut Vec<CanFrame>) {
            if now.is_multiple_of(self.every) {
                let data = ((now / self.every) as u8).wrapping_add(1);
                out.push(CanFrame::with_ts(self.id, vec![data; 8], now).unwrap());
            }
        }

        fn set_signal(
            &mut self,
            _id: u32,
            _signal: &str,
            _value: SignalValue,
        ) -> Result<(), crate::network::NetEcuError> {
            Err(crate::network::NetEcuError::SignalNotSupported)
        }
    }

    struct EchoOnUpdate {
        id: u32,
        every: Timestamp,
        seen: Arc<Mutex<Vec<u32>>>,
    }

    impl NetEcu<CanFrame> for EchoOnUpdate {
        fn name(&self) -> &str {
            "echo"
        }

        fn update(&mut self, now: Timestamp, out: &mut Vec<CanFrame>) {
            if now.is_multiple_of(self.every) {
                out.push(CanFrame::with_ts(self.id, vec![0xAA; 8], now).unwrap());
            }
        }

        fn on_message(&mut self, f: &CanFrame, _now: Timestamp) {
            self.seen.lock().unwrap().push(f.key());
        }

        fn set_signal(
            &mut self,
            _id: u32,
            _signal: &str,
            _value: SignalValue,
        ) -> Result<(), crate::network::NetEcuError> {
            Err(crate::network::NetEcuError::SignalNotSupported)
        }
    }

    fn sim() -> Simulation {
        let mut s = Simulation::new(US_PER_MS);
        s.attach(
            Box::new(PeriodicSender {
                id: 0x100,
                every: 10 * US_PER_MS,
            }),
            &[0x100],
        );
        s.attach(
            Box::new(EchoOnUpdate {
                id: 0x200,
                every: 20 * US_PER_MS,
                seen: Arc::new(Mutex::new(Vec::new())),
            }),
            &[0x200],
        );
        s
    }

    #[test]
    fn route_delivers_to_subscribed_ecus_only() {
        let mut s = sim();
        s.attach(
            Box::new(EchoOnUpdate {
                id: 0x300,
                every: US_PER_MS,
                seen: Arc::new(Mutex::new(Vec::new())),
            }),
            &[0x300],
        );
        for _ in 0..20 {
            s.step();
        }
        let delivered: Vec<u32> = s.recorder().messages().iter().map(|f| f.key()).collect();
        assert!(delivered.contains(&0x100));
        assert!(delivered.contains(&0x200));
        assert!(delivered.contains(&0x300));
        assert!(!delivered.contains(&0x400));
    }

    #[test]
    fn delivery_is_to_subscribed_ecus_only() {
        let mut s = Simulation::new(US_PER_MS);
        s.attach(
            Box::new(PeriodicSender {
                id: 0x100,
                every: 10 * US_PER_MS,
            }),
            &[0x100],
        );
        let seen = Arc::new(Mutex::new(Vec::new()));
        s.attach(
            Box::new(EchoOnUpdate {
                id: 0x300,
                every: US_PER_MS,
                seen: Arc::clone(&seen),
            }),
            &[0x100, 0x300],
        );
        s.subscribe(0x200, 0);
        for _ in 0..20 {
            s.step();
        }
        let seen = seen.lock().unwrap();
        assert!(seen.contains(&0x100), "subscribed to 0x100");
        assert!(seen.contains(&0x300), "subscribed to 0x300");
        assert!(!seen.contains(&0x200), "0x200 only routed to ECU 0");
    }

    #[test]
    fn run_ms_advances_to_requested_time() {
        let mut s = sim();
        s.run_ms(50);
        assert_eq!(s.time(), 50 * US_PER_MS);
        assert_eq!(s.recorder().messages().len(), 5 + 3);
    }

    #[test]
    fn frame_counts_are_reported_per_id() {
        let mut s = sim();
        s.run_ms(100);
        assert_eq!(s.frame_counts(), vec![(0x100, 10), (0x200, 5)]);
    }

    #[test]
    fn last_frame_uses_recorder() {
        let mut s = sim();
        s.run_ms(100);
        assert_eq!(s.recorder().last_message(&0x100).unwrap().id, 0x100);
        assert_eq!(s.recorder().last_message(&0x200).unwrap().id, 0x200);
        assert_eq!(s.recorder().last_message(&0x999), None);
    }

    #[test]
    fn recorder_events_and_messages_mix() {
        let mut s = sim();
        s.run_ms(50);
        s.recorder_mut().event(100, "ecu", "fault");
        let events: Vec<_> = s
            .recorder()
            .records
            .iter()
            .filter(|r| matches!(r, crate::network::NetRecord::Event { .. }))
            .collect();
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn set_signal_forwards_to_ecu() {
        let mut s = Simulation::new(US_PER_MS);
        s.attach(
            Box::new(PeriodicSender {
                id: 0x100,
                every: 10 * US_PER_MS,
            }),
            &[0x100],
        );
        assert_eq!(
            s.set_signal(0, 0x100, "signal", SignalValue::Num(1.0)),
            Err(crate::network::NetEcuError::SignalNotSupported)
        );
    }

    #[test]
    fn faults_window_is_half_open() {
        let f = FaultRule {
            fault: Fault::DropFrame { id: 0x100 },
            start: 10,
            duration: Some(10),
        };
        assert!(!f.active_at(9));
        assert!(f.active_at(10));
        assert!(f.active_at(19));
        assert!(!f.active_at(20));
    }

    #[test]
    fn fault_rule_can_be_built_from_generic_type() {
        let rule = crate::network::NetFaultRule::<Fault> {
            fault: Fault::DropFrame { id: 0x100 },
            start: 0,
            duration: None,
        };
        assert!(rule.active_at(5));
    }

    #[test]
    fn set_signal_unknown_message() {
        let mut s = sim();
        let err = s.set_signal(0, 0xFFFF, "x", SignalValue::Num(1.0));
        assert_eq!(err, Err(crate::network::NetEcuError::SignalNotSupported));
    }

    #[test]
    fn inject_pushes_external_message() {
        let mut s = sim();
        s.inject(CanFrame::with_ts(0x400, vec![0; 8], 5).unwrap());
        assert_eq!(s.recorder().messages().len(), 1);
    }

    #[test]
    fn delay_fault_holds_message_until_delay_elapses() {
        let mut s = sim();
        s.add_fault(FaultRule {
            fault: Fault::DelayFrame {
                id: 0x100,
                delay_us: 50 * US_PER_MS,
            },
            start: 0,
            duration: None,
        });
        s.run_ms(20);
        let all = s.recorder().messages();
        let delayed = all.iter().filter(|f| f.key() == 0x100).count();
        assert_eq!(delayed, 0, "0x100 still held back");
        s.run_ms(60);
        let all = s.recorder().messages();
        let delayed = all.iter().filter(|f| f.key() == 0x100).count();
        assert!(delayed > 0);
    }

    #[test]
    fn drop_fault_suppresses_messages_then_recovers() {
        let mut s = sim();
        s.add_fault(FaultRule {
            fault: Fault::DropFrame { id: 0x100 },
            start: 10 * US_PER_MS,
            duration: Some(10 * US_PER_MS),
        });
        s.run_ms(40);
        assert_eq!(
            s.frame_counts(),
            vec![(0x100, 3), (0x200, 2)],
            "only the t=10ms frame is inside the [10ms, 20ms) window"
        );
    }

    #[test]
    fn corrupt_byte_flips_bits() {
        let mut s = sim();
        s.add_fault(FaultRule {
            fault: Fault::CorruptByte {
                id: 0x100,
                byte: 0,
                mask: 0x0F,
            },
            start: 0,
            duration: None,
        });
        s.run_ms(10);
        let last = s.recorder().last_message(&0x100).unwrap();
        assert_eq!(last.data[0], 0x0E, "1 XOR 0x0F = 0x0E");
    }

    #[test]
    fn ms_helper_is_milliseconds() {
        assert_eq!(ms(10), 10 * US_PER_MS);
    }
}
