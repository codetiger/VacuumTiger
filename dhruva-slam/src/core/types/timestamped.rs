//! Generic timestamp wrapper.
//!
//! Note: Some utility methods are defined for future use.

use serde::{Deserialize, Serialize};

/// Generic timestamp wrapper for any data type.
///
/// Timestamps are in microseconds since epoch (matches SangamIO).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timestamped<T> {
    /// The wrapped data
    pub data: T,
    /// Timestamp in microseconds since epoch
    pub timestamp_us: u64,
}

impl<T> Timestamped<T> {
    /// Create a new timestamped value.
    #[inline]
    pub fn new(data: T, timestamp_us: u64) -> Self {
        Self { data, timestamp_us }
    }
}
