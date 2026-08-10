//! UDP/IP datagrams and endpoints.

use std::net::SocketAddr;

use embrig_core::time::Timestamp;

/// A UDP datagram travelling on a simulated or real Ethernet network.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdpDatagram {
    pub src: SocketAddr,
    pub dst: SocketAddr,
    pub payload: Vec<u8>,
    /// Timestamp when the datagram was produced (µs).
    pub ts: Timestamp,
}

impl UdpDatagram {
    /// Create a datagram with timestamp 0.
    pub fn new(src: SocketAddr, dst: SocketAddr, payload: Vec<u8>) -> Self {
        Self {
            src,
            dst,
            payload,
            ts: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn datagram_construction() {
        let src: SocketAddr = "127.0.0.1:5001".parse().unwrap();
        let dst: SocketAddr = "127.0.0.1:5000".parse().unwrap();
        let dg = UdpDatagram::new(src, dst, vec![1, 2, 3]);
        assert_eq!(dg.src, src);
        assert_eq!(dg.dst, dst);
        assert_eq!(dg.ts, 0);
    }
}
