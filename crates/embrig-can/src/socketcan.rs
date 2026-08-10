//! Async SocketCAN backend.
//!
//! This wraps `socketcan`'s native tokio socket (`socketcan::tokio::CanSocket`),
//! which uses an async task on the socket's readiness. Frame timestamps are
//! the number of microseconds since the bus was opened (a monotonic origin).
//!
//! The backend is strictly point-to-point: it cannot drop, delay or corrupt
//! frames, because there is no software router in the loop. Faults that
//! require a router are rejected by the test runner instead of silently
//! ignored.

use std::time::{Duration, Instant};

use embrig_core::frame::CanFrame;
use embrig_core::time::Timestamp;
use socketcan::tokio::CanSocket;
use socketcan::{CanId, EmbeddedFrame, SocketOptions};
use thiserror::Error;

/// Errors produced by the SocketCAN backend.
#[derive(Debug, Error)]
pub enum CanError {
    #[error("failed to open CAN interface `{interface}`: {source}")]
    Open {
        interface: String,
        source: std::io::Error,
    },
    #[error("invalid CAN id {id:03X} (extended={extended})")]
    InvalidId { id: u32, extended: bool },
    #[error("send on `{interface}` failed: {source}")]
    Send {
        interface: String,
        source: std::io::Error,
    },
    #[error("receive on `{interface}` failed: {source}")]
    Recv {
        interface: String,
        source: std::io::Error,
    },
}

/// An async CAN bus interface backed by a Linux SocketCAN device
/// (e.g. `vcan0` or `can0`).
pub struct SocketCanBus {
    interface: String,
    socket: CanSocket,
    origin: Instant,
}

impl SocketCanBus {
    /// Open `interface` (e.g. `vcan0`).
    pub fn open(interface: &str) -> Result<Self, CanError> {
        let socket = CanSocket::open(interface).map_err(|source| CanError::Open {
            interface: interface.to_string(),
            source,
        })?;
        // Receive our own transmissions so a single socket can verify the
        // send → receive path (used by the loopback smoke test).
        socket
            .set_recv_own_msgs(true)
            .map_err(|source| CanError::Open {
                interface: interface.to_string(),
                source,
            })?;
        Ok(Self {
            interface: interface.to_string(),
            socket,
            origin: Instant::now(),
        })
    }

    pub fn interface(&self) -> &str {
        &self.interface
    }

    /// Microseconds elapsed since the bus was opened.
    pub fn now_us(&self) -> Timestamp {
        self.origin.elapsed().as_micros() as u64
    }

    /// Transmit a frame on the bus.
    ///
    /// The frame's `ts` field is set to the current bus time before sending.
    pub async fn send(&self, frame: &CanFrame) -> Result<Timestamp, CanError> {
        let can_id = if frame.extended {
            CanId::extended(frame.id)
        } else {
            CanId::standard(frame.id as u16)
        }
        .ok_or(CanError::InvalidId {
            id: frame.id,
            extended: frame.extended,
        })?;
        let out =
            socketcan::CanFrame::new(can_id.as_id(), &frame.data).ok_or(CanError::InvalidId {
                id: frame.id,
                extended: frame.extended,
            })?;
        self.socket
            .write_frame(out)
            .await
            .map_err(|source| CanError::Send {
                interface: self.interface.clone(),
                source,
            })?;
        Ok(self.now_us())
    }

    /// Receive one frame, waiting up to `timeout`.
    ///
    /// Returns `Ok(None)` on timeout or when a non-data frame (remote/error)
    /// is received. Frame timestamps are set to the receive time.
    pub async fn recv(&self, timeout: Duration) -> Result<Option<CanFrame>, CanError> {
        match tokio::time::timeout(timeout, self.socket.read_frame()).await {
            Ok(Ok(frame)) => Ok(convert_in(frame, self.now_us())),
            Ok(Err(source)) => Err(CanError::Recv {
                interface: self.interface.clone(),
                source,
            }),
            Err(_elapsed) => Ok(None),
        }
    }
}

fn convert_in(frame: socketcan::CanFrame, ts: Timestamp) -> Option<CanFrame> {
    if !matches!(frame, socketcan::CanFrame::Data(_)) {
        // Remote/error frames are not part of the Embrig data model.
        return None;
    }
    let id = CanId::from(frame.id()).as_raw();
    Some(CanFrame {
        id,
        data: frame.data().to_vec(),
        ts,
        extended: frame.is_extended(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extended_flag_survives_id_lookup() {
        // CanId::from(Id).as_raw() round-trips the 29-bit space.
        for (id, ext) in [(0x100u32, false), (0x1C0FFEEu32, true)] {
            let can_id = if ext {
                CanId::extended(id).unwrap()
            } else {
                CanId::standard(id as u16).unwrap()
            };
            assert_eq!(can_id.as_raw(), id);
            assert_eq!(can_id.is_extended(), ext);
        }
    }

    #[tokio::test]
    async fn convert_in_ignores_remote_frames() {
        let remote =
            socketcan::CanFrame::new_remote(CanId::standard(0x100).unwrap().as_id(), 0).unwrap();
        assert!(convert_in(remote, 0).is_none());
    }
}
