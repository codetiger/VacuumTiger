//! Exploration state machine states.

use crate::core::WorldPoint;
use crate::query::Frontier;

// Note: Frontier and WorldPoint are still used by ExplorationState and ExplorationCommand

/// Exploration state
#[derive(Clone, Debug)]
pub enum ExplorationState {
    /// Idle - waiting to start exploration
    Idle,

    /// Searching for frontiers
    SearchingFrontiers,

    /// Planning path to selected frontier
    Planning {
        /// Target frontier
        target: Frontier,
    },

    /// Navigating to frontier
    Navigating {
        /// Current path as world points
        path: Vec<WorldPoint>,
        /// Current waypoint index
        current_waypoint: usize,
        /// Target frontier
        target: Frontier,
    },

    /// Reached frontier, scanning area by rotating
    Scanning {
        /// Position where we're scanning
        position: WorldPoint,
        /// Remaining rotation in radians (full rotation = 2π)
        rotation_remaining: f32,
    },

    /// Recovering from obstacle or stuck condition
    Recovering {
        /// Recovery action being performed
        action: RecoveryAction,
        /// Number of recovery attempts
        attempts: usize,
    },

    /// Exploration complete (no more frontiers)
    Complete,

    /// Failed (too many errors)
    Failed {
        /// Reason for failure
        reason: String,
    },
}

impl ExplorationState {
    /// Is this a terminal state?
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            ExplorationState::Complete | ExplorationState::Failed { .. }
        )
    }

    /// Is exploration active?
    pub fn is_active(&self) -> bool {
        !matches!(
            self,
            ExplorationState::Idle | ExplorationState::Complete | ExplorationState::Failed { .. }
        )
    }

    /// State name for logging
    pub fn name(&self) -> &'static str {
        match self {
            ExplorationState::Idle => "Idle",
            ExplorationState::SearchingFrontiers => "SearchingFrontiers",
            ExplorationState::Planning { .. } => "Planning",
            ExplorationState::Navigating { .. } => "Navigating",
            ExplorationState::Scanning { .. } => "Scanning",
            ExplorationState::Recovering { .. } => "Recovering",
            ExplorationState::Complete => "Complete",
            ExplorationState::Failed { .. } => "Failed",
        }
    }
}

/// Recovery action types
#[derive(Clone, Debug)]
pub enum RecoveryAction {
    /// Turn in place by specified angle (radians)
    TurnInPlace(f32),
    /// Back up by specified distance (meters)
    BackUp(f32),
    /// Wait for obstacle to clear
    Wait,
}

/// Command output from exploration controller
#[derive(Clone, Debug)]
pub enum ExplorationCommand {
    /// No action needed
    None,

    /// Move to target position
    MoveTo {
        /// Target position in world coordinates
        target: WorldPoint,
        /// Maximum speed (m/s)
        max_speed: f32,
    },

    /// Rotate in place
    Rotate {
        /// Target heading (radians)
        target_heading: f32,
        /// Maximum angular speed (rad/s)
        max_angular_speed: f32,
    },

    /// Stop immediately
    Stop,

    /// Exploration is complete
    ExplorationComplete,

    /// Exploration failed
    ExplorationFailed {
        /// Reason for failure
        reason: String,
    },
}

/// Event that can trigger state transitions.
///
/// These events are used to communicate external conditions to the exploration
/// controller, allowing it to respond appropriately (e.g., entering recovery
/// mode when an obstacle is detected).
#[derive(Clone, Debug)]
pub enum ExplorationEvent {
    /// Start exploration.
    Start,

    /// Stop exploration and return to Idle state.
    Stop,

    /// Obstacle detected by sensors (cliff, bumper, etc.).
    /// Triggers recovery behavior.
    ObstacleDetected,

    /// Path is blocked and cannot continue.
    /// Triggers recovery behavior.
    PathBlocked,

    /// Recovery maneuver completed successfully.
    /// Returns to frontier searching.
    RecoveryComplete,

    /// Scan rotation completed at frontier.
    /// Increments frontiers explored and searches for next.
    ScanComplete,
}
