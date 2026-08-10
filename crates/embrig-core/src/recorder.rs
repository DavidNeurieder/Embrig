use crate::frame::CanFrame;
use crate::time::Timestamp;

/// A recorded item during a simulation run.
#[derive(Debug, Clone, PartialEq)]
pub enum Record {
    /// A frame that was actually delivered on the bus.
    Frame(CanFrame),
    /// A marker event, e.g. a fault being triggered.
    Event {
        ts: Timestamp,
        source: String,
        message: String,
    },
}

/// Ordered event log for a simulation run.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Recorder {
    pub records: Vec<Record>,
}

impl Recorder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, record: Record) {
        self.records.push(record);
    }

    pub fn frame(&mut self, frame: CanFrame) {
        self.push(Record::Frame(frame));
    }

    pub fn event(&mut self, ts: Timestamp, source: impl Into<String>, message: impl Into<String>) {
        self.push(Record::Event {
            ts,
            source: source.into(),
            message: message.into(),
        });
    }

    /// All frames, in order, from most recent to least recent.
    pub fn frames(&self) -> Vec<&CanFrame> {
        self.records
            .iter()
            .rev()
            .filter_map(|r| match r {
                Record::Frame(f) => Some(f),
                _ => None,
            })
            .collect()
    }

    /// The most recent frame with the given id, if any.
    pub fn last_frame(&self, id: u32) -> Option<&CanFrame> {
        self.records.iter().rev().find_map(|r| match r {
            Record::Frame(f) if f.id == id => Some(f),
            _ => None,
        })
    }

    /// Whether any frame with the given id appears in the log.
    pub fn has_frame(&self, id: u32) -> bool {
        self.records
            .iter()
            .any(|r| matches!(r, Record::Frame(f) if f.id == id))
    }

    pub fn clear(&mut self) {
        self.records.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(id: u32) -> CanFrame {
        CanFrame::with_ts(id, vec![0; 8], id as u64).unwrap()
    }

    #[test]
    fn frames_returns_most_recent_first() {
        let mut r = Recorder::new();
        r.event(0, "ecu", "boot");
        r.frame(frame(0x100));
        r.frame(frame(0x200));
        let ids: Vec<u32> = r.frames().iter().map(|f| f.id).collect();
        assert_eq!(ids, vec![0x200, 0x100], "events do not appear in frames()");
    }

    #[test]
    fn last_frame_returns_newest_for_id() {
        let mut r = Recorder::new();
        assert_eq!(r.last_frame(0x100), None);
        r.frame(frame(0x100));
        r.frame(frame(0x200));
        r.frame(frame(0x100));
        assert_eq!(r.last_frame(0x100).unwrap().id, 0x100);
        assert_eq!(r.last_frame(0x100).unwrap().ts, 0x100, "newest wins");
        assert_eq!(r.last_frame(0x999), None);
    }

    #[test]
    fn has_frame_sees_frames_but_not_events() {
        let mut r = Recorder::new();
        r.event(1, "ecu", "fault");
        assert!(!r.has_frame(0x100));
        r.frame(frame(0x100));
        assert!(r.has_frame(0x100));
        assert!(!r.has_frame(0x200));
    }

    #[test]
    fn clear_empties_records() {
        let mut r = Recorder::new();
        r.frame(frame(0x100));
        r.event(1, "ecu", "fault");
        r.clear();
        assert!(r.records.is_empty());
        assert!(!r.has_frame(0x100));
        assert_eq!(r.last_frame(0x100), None);
    }
}
