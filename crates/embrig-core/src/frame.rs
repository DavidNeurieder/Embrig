use crate::time::Timestamp;

/// A CAN frame travelling on a simulated or real bus.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub data: Vec<u8>,
    /// Timestamp when the frame was produced (µs).
    pub ts: Timestamp,
    pub extended: bool,
}

impl CanFrame {
    /// Create a frame with timestamp 0.
    pub fn new(id: u32, data: Vec<u8>) -> Result<Self, FrameError> {
        Self::with_ts(id, data, 0)
    }

    /// Create a frame with an explicit timestamp.
    pub fn with_ts(id: u32, data: Vec<u8>, ts: Timestamp) -> Result<Self, FrameError> {
        if data.len() > 8 {
            return Err(FrameError::DataTooLong(data.len()));
        }
        Ok(Self {
            id,
            data,
            ts,
            extended: false,
        })
    }

    /// Decode the two-byte little-endian value starting at `offset`.
    pub fn u16_le(&self, offset: usize) -> u16 {
        u16::from_le_bytes([self.data[offset], self.data[offset + 1]])
    }

    /// Set a two-byte little-endian value starting at `offset`.
    pub fn set_u16_le(&mut self, offset: usize, value: u16) {
        let bytes = value.to_le_bytes();
        self.data[offset] = bytes[0];
        self.data[offset + 1] = bytes[1];
    }
}

impl std::fmt::Display for CanFrame {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let data: Vec<String> = self.data.iter().map(|b| format!("{b:02X}")).collect();
        write!(
            f,
            "{:>12} 0x{:03X}  [{}] {}",
            self.ts,
            self.id,
            self.data.len(),
            data.join(" ")
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameError {
    /// CAN 2.0 frames carry at most 8 data bytes.
    DataTooLong(usize),
    /// Offset/length does not fit inside the frame data.
    OutOfBounds,
}

impl std::fmt::Display for FrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FrameError::DataTooLong(len) => {
                write!(f, "CAN frame data length {len} exceeds 8 bytes")
            }
            FrameError::OutOfBounds => write!(f, "access outside frame data"),
        }
    }
}

impl std::error::Error for FrameError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn data_length_limit() {
        assert!(CanFrame::new(0x100, vec![0; 8]).is_ok());
        assert_eq!(
            CanFrame::new(0x100, vec![0; 9]),
            Err(FrameError::DataTooLong(9))
        );
    }

    #[test]
    fn u16_round_trip() {
        let mut f = CanFrame::new(0x100, vec![0; 8]).unwrap();
        f.set_u16_le(2, 0xBEEF);
        assert_eq!(f.u16_le(2), 0xBEEF);
        assert_eq!(f.data[2], 0xEF);
        assert_eq!(f.data[3], 0xBE);
    }
}
