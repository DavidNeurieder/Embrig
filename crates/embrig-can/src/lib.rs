//! Hardware CAN backends for Embrig.
//!
//! The only backend is SocketCAN, gated behind the `socketcan` feature because
//! it links against Linux socket APIs. Without the feature, this crate is an
//! empty library so the rest of the workspace still builds anywhere.

#[cfg(feature = "socketcan")]
pub mod socketcan;

#[cfg(feature = "socketcan")]
pub use socketcan::{CanError, SocketCanBus};
