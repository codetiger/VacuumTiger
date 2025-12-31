//! Exploration error types.
//!
//! Provides structured error types for exploration failures,
//! replacing String-based error messages.

use std::fmt;

use crate::core::WorldPoint;

/// Errors that can occur during autonomous exploration.
#[derive(Debug, Clone)]
pub enum ExplorationError {
    /// Robot connection was lost during exploration.
    ConnectionLost,

    /// Battery level dropped below minimum threshold.
    LowBattery {
        /// Current battery percentage.
        percent: u8,
        /// Configured minimum threshold.
        threshold: u8,
    },

    /// Exploration exceeded the configured time limit.
    TimeoutExceeded {
        /// Elapsed time in seconds.
        elapsed_secs: f32,
        /// Configured limit in seconds.
        limit_secs: f32,
    },

    /// Too many consecutive planning/navigation failures.
    TooManyFailures {
        /// Number of consecutive failures.
        count: usize,
        /// Configured maximum.
        max: usize,
    },

    /// Robot appears to be stuck and recovery failed.
    StuckRecoveryFailed {
        /// Position where robot got stuck.
        position: WorldPoint,
        /// Number of recovery attempts made.
        attempts: usize,
    },

    /// Localization drift detected (robot position appears blocked).
    LocalizationDrift {
        /// Number of consecutive "start blocked" events.
        consecutive_events: usize,
    },

    /// No valid frontiers found with insufficient exploration.
    InsufficientExploration {
        /// Explored floor area in square meters.
        explored_area_m2: f32,
        /// Number of frontiers successfully explored.
        frontiers_explored: usize,
    },

    /// Custom error with message.
    Other(String),
}

impl ExplorationError {
    /// Create an error from a string message (for backwards compatibility).
    pub fn from_message(msg: impl Into<String>) -> Self {
        Self::Other(msg.into())
    }

    /// Get a short error code for logging/metrics.
    pub fn code(&self) -> &'static str {
        match self {
            Self::ConnectionLost => "CONNECTION_LOST",
            Self::LowBattery { .. } => "LOW_BATTERY",
            Self::TimeoutExceeded { .. } => "TIMEOUT",
            Self::TooManyFailures { .. } => "TOO_MANY_FAILURES",
            Self::StuckRecoveryFailed { .. } => "STUCK",
            Self::LocalizationDrift { .. } => "LOCALIZATION_DRIFT",
            Self::InsufficientExploration { .. } => "INSUFFICIENT_EXPLORATION",
            Self::Other(_) => "OTHER",
        }
    }

    /// Check if this error is recoverable (exploration could potentially continue).
    pub fn is_recoverable(&self) -> bool {
        matches!(
            self,
            Self::StuckRecoveryFailed { .. } | Self::LocalizationDrift { .. }
        )
    }
}

impl fmt::Display for ExplorationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ConnectionLost => write!(f, "Robot connection lost"),
            Self::LowBattery { percent, threshold } => {
                write!(f, "Battery low: {}% (minimum: {}%)", percent, threshold)
            }
            Self::TimeoutExceeded {
                elapsed_secs,
                limit_secs,
            } => {
                write!(
                    f,
                    "Time limit exceeded: {:.1}s elapsed (limit: {:.1}s)",
                    elapsed_secs, limit_secs
                )
            }
            Self::TooManyFailures { count, max } => {
                write!(f, "Too many consecutive failures: {} (max: {})", count, max)
            }
            Self::StuckRecoveryFailed { position, attempts } => {
                write!(
                    f,
                    "Stuck at ({:.2}, {:.2}) after {} recovery attempts",
                    position.x, position.y, attempts
                )
            }
            Self::LocalizationDrift { consecutive_events } => {
                write!(
                    f,
                    "Localization drift detected ({} consecutive blocked events)",
                    consecutive_events
                )
            }
            Self::InsufficientExploration {
                explored_area_m2,
                frontiers_explored,
            } => {
                write!(
                    f,
                    "Insufficient exploration: {:.2}m² explored, {} frontiers done",
                    explored_area_m2, frontiers_explored
                )
            }
            Self::Other(msg) => write!(f, "{}", msg),
        }
    }
}

impl std::error::Error for ExplorationError {}

/// Convert from String for backwards compatibility.
impl From<String> for ExplorationError {
    fn from(s: String) -> Self {
        Self::Other(s)
    }
}

impl From<&str> for ExplorationError {
    fn from(s: &str) -> Self {
        Self::Other(s.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = ExplorationError::LowBattery {
            percent: 15,
            threshold: 20,
        };
        assert_eq!(err.to_string(), "Battery low: 15% (minimum: 20%)");
    }

    #[test]
    fn test_error_code() {
        assert_eq!(ExplorationError::ConnectionLost.code(), "CONNECTION_LOST");
        assert_eq!(
            ExplorationError::TooManyFailures { count: 5, max: 5 }.code(),
            "TOO_MANY_FAILURES"
        );
    }

    #[test]
    fn test_from_string() {
        let err: ExplorationError = "Custom error".into();
        assert!(matches!(err, ExplorationError::Other(_)));
    }
}
