//! Motion command definitions

use std::time::{Duration, Instant};

/// Motion command types
#[derive(Debug, Clone)]
pub enum MotionCommand {
    /// Set continuous velocity
    Velocity {
        /// Linear velocity in m/s
        linear: f32,
        /// Angular velocity in rad/s
        angular: f32,
    },

    /// Move forward/backward a specific distance
    MoveDistance {
        /// Distance in meters (negative for backward)
        distance: f32,
        /// Maximum velocity in m/s
        max_velocity: f32,
    },

    /// Rotate in place
    Rotate {
        /// Angle in radians (positive = CCW)
        angle: f32,
        /// Maximum angular velocity in rad/s
        max_velocity: f32,
    },

    /// Stop motion
    Stop,

    /// Emergency stop
    EmergencyStop,
}

/// Status of active motion command
#[derive(Debug, Clone)]
pub struct MotionCommandStatus {
    /// Type of command
    pub command_type: String,

    /// Progress (0.0 to 1.0)
    pub progress: f32,

    /// Time since command started
    pub elapsed_time: Duration,

    /// Start time
    pub start_time: Instant,

    /// Estimated time remaining
    pub time_remaining: Option<Duration>,
}

impl MotionCommand {
    /// Get command type as string
    pub fn command_type(&self) -> String {
        match self {
            Self::Velocity { .. } => "velocity".to_string(),
            Self::MoveDistance { .. } => "move_distance".to_string(),
            Self::Rotate { .. } => "rotate".to_string(),
            Self::Stop => "stop".to_string(),
            Self::EmergencyStop => "emergency_stop".to_string(),
        }
    }

    /// Check if this is a position-based command
    pub fn is_position_based(&self) -> bool {
        matches!(self, Self::MoveDistance { .. } | Self::Rotate { .. })
    }
}
