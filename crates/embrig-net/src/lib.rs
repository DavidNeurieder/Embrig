//! Ethernet (UDP/IP) for Embrig: the netmap field codec, a deterministic
//! virtual network simulation, and the virtual ECU layer for Ethernet nodes.
//!
//! This crate is deliberately transport-only: it has no test runner and no
//! dependency on `embrig-test`. The UDP test targets (`UdpTarget`, SIL and
//! hardware) live in `embrig-test::udp`, which drives the simulation here.
//!
//! A netmap describes the UDP traffic of a network the way a DBC describes
//! CAN: each message is identified by its destination endpoint and carries
//! named fields at fixed byte offsets. Test steps key by message name, so
//! suites look the same whether they run against the virtual network, SIL
//! firmware or a real Ethernet link.

pub mod datagram;
pub mod ecu;
pub mod netmap;
pub mod sim;

pub use datagram::UdpDatagram;
pub use ecu::{NoFirmware, UdpConfigEcu, UdpEcu, UdpEcuError, UdpEcuFactory, UdpRegistry};
pub use netmap::{DecodedField, FieldDef, FieldError, FieldType, MessageDef, Netmap};
pub use sim::{UdpFault, UdpFaultRule, UdpRecord, UdpRecorder, UdpSim};
