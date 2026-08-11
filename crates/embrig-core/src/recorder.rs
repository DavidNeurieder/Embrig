use crate::frame::CanFrame;
use crate::network::{NetRecord, NetRecorder};

/// A recorded CAN item: a delivered frame or a marker event.
pub type Record = NetRecord<CanFrame>;

/// Ordered event log for a CAN simulation run.
///
/// The generic [`NetRecorder`] with [`CanFrame`] as the message type, so the
/// exact same recorder used by the CAN sim serves the UDP and TCP sims.
pub type Recorder = NetRecorder<CanFrame>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::NetMessage;

    fn frame(id: u32) -> CanFrame {
        CanFrame::with_ts(id, vec![0; 8], id as u64).unwrap()
    }

    #[test]
    fn messages_returns_most_recent_first() {
        let mut r = Recorder::new();
        r.event(0, "ecu", "boot");
        r.message(frame(0x100));
        r.message(frame(0x200));
        let ids: Vec<u32> = r.messages().iter().map(|f| f.key()).collect();
        assert_eq!(
            ids,
            vec![0x200, 0x100],
            "events do not appear in messages()"
        );
    }

    #[test]
    fn last_message_returns_newest_for_key() {
        let mut r = Recorder::new();
        assert_eq!(r.last_message(&0x100), None);
        r.message(frame(0x100));
        r.message(frame(0x200));
        r.message(frame(0x100));
        assert_eq!(r.last_message(&0x100).unwrap().id, 0x100);
        assert_eq!(r.last_message(&0x100).unwrap().ts, 0x100, "newest wins");
        assert_eq!(r.last_message(&0x999), None);
    }

    #[test]
    fn has_message_sees_messages_but_not_events() {
        let mut r = Recorder::new();
        r.event(1, "ecu", "fault");
        assert!(!r.has_message(&0x100));
        r.message(frame(0x100));
        assert!(r.has_message(&0x100));
        assert!(!r.has_message(&0x200));
    }

    #[test]
    fn clear_empties_records() {
        let mut r = Recorder::new();
        r.message(frame(0x100));
        r.event(1, "ecu", "fault");
        r.clear();
        assert!(r.records.is_empty());
        assert!(!r.has_message(&0x100));
        assert_eq!(r.last_message(&0x100), None);
    }

    #[test]
    fn record_variants_match_can_semantics() {
        let record: Record = Record::Message(frame(0x100));
        match &record {
            Record::Message(f) => assert_eq!(f.id, 0x100),
            Record::Event { .. } => panic!("not an event"),
        }
    }
}
