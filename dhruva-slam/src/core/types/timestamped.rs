//! Generic timestamp wrapper.

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

    /// Map the inner data while preserving timestamp.
    #[inline]
    pub fn map<U, F: FnOnce(T) -> U>(self, f: F) -> Timestamped<U> {
        Timestamped {
            data: f(self.data),
            timestamp_us: self.timestamp_us,
        }
    }

    /// Get a reference to the inner data.
    #[inline]
    pub fn as_ref(&self) -> Timestamped<&T> {
        Timestamped {
            data: &self.data,
            timestamp_us: self.timestamp_us,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timestamped_map() {
        let ts = Timestamped::new(42i32, 1000);
        let doubled = ts.map(|x| x * 2);

        assert_eq!(doubled.data, 84);
        assert_eq!(doubled.timestamp_us, 1000);
    }
}
