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
