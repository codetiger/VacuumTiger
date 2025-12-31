//! Pathfinding configuration section.

use serde::{Deserialize, Serialize};

use super::defaults;

/// Pathfinding settings section
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct PathfindingSection {
    /// A* algorithm settings
    #[serde(default)]
    pub astar: AStarSettings,

    /// Path smoothing settings
    #[serde(default)]
    pub smoothing: SmoothingSettings,
}

/// A* algorithm settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AStarSettings {
    /// Enable 8-directional movement
    #[serde(default = "defaults::enabled")]
    pub allow_diagonal: bool,

    /// Cost multiplier for diagonal moves (sqrt(2))
    #[serde(default = "defaults::diagonal_cost")]
    pub diagonal_cost: f32,

    /// Maximum nodes to expand
    #[serde(default = "defaults::max_iterations")]
    pub max_iterations: usize,

    /// Penalty for paths near obstacles (0=disabled)
    #[serde(default)]
    pub clearance_weight: f32,
}

impl Default for AStarSettings {
    fn default() -> Self {
        Self {
            allow_diagonal: true,
            diagonal_cost: std::f32::consts::SQRT_2,
            max_iterations: 100_000,
            clearance_weight: 0.0,
        }
    }
}

/// Path smoothing settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SmoothingSettings {
    /// Enable path smoothing
    #[serde(default = "defaults::enabled")]
    pub enabled: bool,

    /// Line-of-sight check step (meters)
    #[serde(default = "defaults::los_step")]
    pub los_step_size: f32,

    /// Midpoint smoothing iterations
    #[serde(default = "defaults::smooth_iterations")]
    pub max_iterations: usize,
}

impl Default for SmoothingSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            los_step_size: 0.05,
            max_iterations: 100,
        }
    }
}
