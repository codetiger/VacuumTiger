//! Exploration controller configuration.

use serde::{Deserialize, Serialize};

use crate::query::RobotFootprint;

/// Exploration controller configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExplorationConfig {
    /// Robot footprint for collision checking
    pub footprint: RobotFootprint,

    /// Minimum frontier size to consider (cells)
    pub min_frontier_size: usize,

    /// Distance threshold to consider frontier reached (meters)
    pub frontier_reached_threshold: f32,

    /// Distance threshold for waypoint reached (meters)
    pub waypoint_threshold: f32,

    /// Maximum distance to plan at once (meters)
    pub max_plan_distance: f32,

    /// Replan interval when path is blocked (seconds)
    pub replan_interval: f32,

    /// Maximum consecutive failures before giving up
    pub max_consecutive_failures: usize,

    /// Clearance weight for path planning (higher = prefer more clearance)
    pub path_clearance_weight: f32,

    /// Enable path smoothing
    pub smooth_paths: bool,

    /// Recovery turn angle (radians) when stuck
    pub recovery_turn_angle: f32,

    /// Recovery backup distance (meters) when stuck
    pub recovery_backup_distance: f32,
}

impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            footprint: RobotFootprint::default(),
            min_frontier_size: 5,
            frontier_reached_threshold: 0.3, // 30cm
            waypoint_threshold: 0.1,         // 10cm
            max_plan_distance: 5.0,          // 5m
            replan_interval: 1.0,            // 1 second
            max_consecutive_failures: 5,
            path_clearance_weight: 0.0,
            smooth_paths: true,
            recovery_turn_angle: std::f32::consts::FRAC_PI_4, // 45 degrees
            recovery_backup_distance: 0.1,                    // 10cm
        }
    }
}

impl ExplorationConfig {
    /// Create with custom robot footprint
    pub fn with_footprint(footprint: RobotFootprint) -> Self {
        Self {
            footprint,
            ..Default::default()
        }
    }
}
