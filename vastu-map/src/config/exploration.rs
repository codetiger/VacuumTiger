//! Exploration configuration section.

use serde::{Deserialize, Serialize};

use super::defaults;

/// Exploration settings section
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExplorationSection {
    /// Minimum cells for valid frontier
    #[serde(default = "defaults::min_frontier_size")]
    pub min_frontier_size: usize,

    /// Distance to consider frontier reached (meters)
    #[serde(default = "defaults::frontier_threshold")]
    pub frontier_reached_threshold: f32,

    /// Distance for waypoint reached (meters)
    #[serde(default = "defaults::waypoint_threshold")]
    pub waypoint_threshold: f32,

    /// Maximum planning distance (meters)
    #[serde(default = "defaults::max_plan_distance")]
    pub max_plan_distance: f32,

    /// Seconds between replans when blocked
    #[serde(default = "defaults::replan_interval")]
    pub replan_interval: f32,

    /// Max consecutive planning failures before giving up
    #[serde(default = "defaults::max_failures")]
    pub max_consecutive_failures: usize,

    /// Penalty for paths near obstacles
    #[serde(default)]
    pub path_clearance_weight: f32,

    /// Enable path smoothing
    #[serde(default = "defaults::enabled")]
    pub smooth_paths: bool,

    /// Recovery behavior settings
    #[serde(default)]
    pub recovery: RecoverySettings,
}

impl Default for ExplorationSection {
    fn default() -> Self {
        Self {
            min_frontier_size: 5,
            frontier_reached_threshold: 0.3,
            waypoint_threshold: 0.1,
            max_plan_distance: 5.0,
            replan_interval: 1.0,
            max_consecutive_failures: 5,
            path_clearance_weight: 0.0,
            smooth_paths: true,
            recovery: RecoverySettings::default(),
        }
    }
}

/// Recovery behavior settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecoverySettings {
    /// Turn angle for recovery (radians)
    #[serde(default = "defaults::turn_angle")]
    pub turn_angle: f32,

    /// Backup distance for recovery (meters)
    #[serde(default = "defaults::backup_distance")]
    pub backup_distance: f32,
}

impl Default for RecoverySettings {
    fn default() -> Self {
        Self {
            turn_angle: std::f32::consts::FRAC_PI_4,
            backup_distance: 0.1,
        }
    }
}
