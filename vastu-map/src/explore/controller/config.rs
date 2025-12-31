//! Exploration controller configuration.

use serde::{Deserialize, Serialize};

use crate::query::RobotFootprint;

/// Exploration controller configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExplorationConfig {
    /// Robot footprint for collision checking.
    #[serde(default)]
    pub footprint: RobotFootprint,

    /// Minimum frontier size to consider (cells).
    #[serde(default = "default_min_frontier_size")]
    pub min_frontier_size: usize,

    /// Distance threshold to consider frontier reached (meters).
    #[serde(default = "default_frontier_reached_threshold")]
    pub frontier_reached_threshold: f32,

    /// Distance threshold for waypoint reached (meters).
    #[serde(default = "default_waypoint_threshold")]
    pub waypoint_threshold: f32,

    /// Maximum distance to plan at once (meters).
    #[serde(default = "default_max_plan_distance")]
    pub max_plan_distance: f32,

    /// Replan interval when path is blocked (seconds).
    #[serde(default = "default_replan_interval")]
    pub replan_interval: f32,

    /// Maximum consecutive failures before giving up.
    #[serde(default = "default_max_consecutive_failures")]
    pub max_consecutive_failures: usize,

    /// Clearance weight for path planning (higher = prefer more clearance).
    #[serde(default)]
    pub path_clearance_weight: f32,

    /// Enable path smoothing.
    #[serde(default = "default_smooth_paths")]
    pub smooth_paths: bool,

    /// Recovery turn angle (radians) when stuck.
    #[serde(default = "default_recovery_turn_angle")]
    pub recovery_turn_angle: f32,

    /// Recovery backup distance (meters) when stuck.
    #[serde(default = "default_recovery_backup_distance")]
    pub recovery_backup_distance: f32,

    /// Maximum linear speed during navigation (m/s).
    #[serde(default = "default_navigation_max_speed")]
    pub navigation_max_speed: f32,

    /// Maximum linear speed during recovery (m/s).
    #[serde(default = "default_recovery_max_speed")]
    pub recovery_max_speed: f32,

    /// Angular speed during scanning rotation (rad/s).
    #[serde(default = "default_scan_angular_speed")]
    pub scan_angular_speed: f32,

    /// Rotation step size during scanning (radians per update).
    #[serde(default = "default_scan_rotation_step")]
    pub scan_rotation_step: f32,

    /// Distance tolerance for matching recently failed frontiers (meters).
    #[serde(default = "default_frontier_blacklist_tolerance")]
    pub frontier_blacklist_tolerance: f32,

    /// Maximum number of steps without progress before declaring stuck.
    #[serde(default = "default_max_stuck_steps")]
    pub max_stuck_steps: usize,

    /// Minimum progress distance to reset stuck counter (meters).
    #[serde(default = "default_min_progress_distance")]
    pub min_progress_distance: f32,
}

fn default_min_frontier_size() -> usize {
    5
}

fn default_frontier_reached_threshold() -> f32 {
    0.3
}

fn default_waypoint_threshold() -> f32 {
    0.1
}

fn default_max_plan_distance() -> f32 {
    5.0
}

fn default_replan_interval() -> f32 {
    1.0
}

fn default_max_consecutive_failures() -> usize {
    5
}

fn default_smooth_paths() -> bool {
    true
}

fn default_recovery_turn_angle() -> f32 {
    std::f32::consts::FRAC_PI_4 // 45 degrees
}

fn default_recovery_backup_distance() -> f32 {
    0.1
}

fn default_navigation_max_speed() -> f32 {
    0.3 // 30 cm/s
}

fn default_recovery_max_speed() -> f32 {
    0.1 // 10 cm/s
}

fn default_scan_angular_speed() -> f32 {
    0.5 // rad/s
}

fn default_scan_rotation_step() -> f32 {
    0.1 // ~6 degrees per step
}

fn default_frontier_blacklist_tolerance() -> f32 {
    0.5 // 50 cm
}

fn default_max_stuck_steps() -> usize {
    50 // ~5 seconds at 10Hz
}

fn default_min_progress_distance() -> f32 {
    0.05 // 5 cm
}

impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            footprint: RobotFootprint::default(),
            min_frontier_size: default_min_frontier_size(),
            frontier_reached_threshold: default_frontier_reached_threshold(),
            waypoint_threshold: default_waypoint_threshold(),
            max_plan_distance: default_max_plan_distance(),
            replan_interval: default_replan_interval(),
            max_consecutive_failures: default_max_consecutive_failures(),
            path_clearance_weight: 0.0,
            smooth_paths: default_smooth_paths(),
            recovery_turn_angle: default_recovery_turn_angle(),
            recovery_backup_distance: default_recovery_backup_distance(),
            navigation_max_speed: default_navigation_max_speed(),
            recovery_max_speed: default_recovery_max_speed(),
            scan_angular_speed: default_scan_angular_speed(),
            scan_rotation_step: default_scan_rotation_step(),
            frontier_blacklist_tolerance: default_frontier_blacklist_tolerance(),
            max_stuck_steps: default_max_stuck_steps(),
            min_progress_distance: default_min_progress_distance(),
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
