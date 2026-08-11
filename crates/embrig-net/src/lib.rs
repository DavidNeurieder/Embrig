//! Ethernet transports for Embrig: the netmap field codec, a deterministic
//! virtual UDP network simulation, a TCP connection simulation (the
//! third-transport proof) and the virtual ECU layer for Ethernet nodes.
//!
//! This crate is deliberately transport-only: it has no test runner and no
//! dependency on `embrig-test`. The UDP test targets (`UdpTarget`, SIL and
//! hardware) live in `embrig-test::udp`, which drives the simulation here.
//!
//! A netmap describes the Ethernet traffic of a network the way a DBC
//! describes CAN: each message is identified by its destination endpoint and
//! carries named fields at fixed byte offsets. Test steps key by message name,
//! so suites look the same whether they run against the virtual network, SIL
//! firmware or a real Ethernet link.
//!
//! All simulation engines, ECU traits, error types and firmware registries are
//! the unified ones from [`embrig_core::network`], specialized per transport:
//! `NetworkSim<SocketAddr, UdpDatagram, UdpFault>` and
//! `NetworkSim<SocketAddr, TcpSegment, TcpFault>`. This crate only adds the
//! message types, the netmap codec and the thin transport wrappers
//! ([`UdpSim`], [`TcpSim`], [`UdpConfigEcu`], [`TcpConfigEcu`]).

pub mod datagram;
pub mod ecu;
pub mod netmap;
pub mod sim;
pub mod tcp;

pub use datagram::UdpDatagram;
pub use ecu::UdpConfigEcu;
pub use netmap::{DecodedField, FieldDef, FieldError, FieldType, MessageDef, Netmap};
pub use sim::{UdpFault, UdpFaultRule, UdpRecord, UdpRecorder, UdpSim};
pub use tcp::{TcpConfigEcu, TcpFault, TcpFaultRule, TcpRecord, TcpRecorder, TcpSegment, TcpSim};
