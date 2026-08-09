//! Simulation time.
//!
//! Time is an integer count of microseconds. Using an integer avoids float
//! drift and keeps simulations reproducible.

/// A simulation timestamp, in microseconds since the simulation started.
pub type Timestamp = u64;

/// Microseconds per millisecond.
pub const US_PER_MS: Timestamp = 1_000;
/// Microseconds per second.
pub const US_PER_S: Timestamp = 1_000_000;

/// Convert a duration in milliseconds to microseconds.
pub fn ms(ms: u64) -> Timestamp {
    ms * US_PER_MS
}

/// Convert a duration in seconds to microseconds.
pub fn sec(s: u64) -> Timestamp {
    s * US_PER_S
}
