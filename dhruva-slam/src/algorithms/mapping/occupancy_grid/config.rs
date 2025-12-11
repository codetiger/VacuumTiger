//! Occupancy grid configuration.

use serde::{Deserialize, Serialize};

/// Cell state for visualization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellState {
    /// Unknown (never observed)
    Unknown,
    /// Free space (definitely empty)
    Free,
    /// Occupied (definitely contains obstacle)
    Occupied,
}

/// Configuration for occupancy grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OccupancyGridConfig {
    /// Cell size in meters.
    pub resolution: f32,

    /// Initial map width in meters.
    ///
    /// Map will grow automatically if needed.
    pub initial_width: f32,

    /// Initial map height in meters.
    pub initial_height: f32,

    /// Log-odds value for occupied observation.
    ///
    /// Higher = more confident. Typical: 0.9
    pub log_odds_occupied: f32,

    /// Log-odds value for free observation.
    ///
    /// Negative value. Typical: -0.7
    pub log_odds_free: f32,

    /// Maximum log-odds value (clamp).
    ///
    /// Prevents overconfidence. Typical: 50.0
    pub log_odds_max: f32,

    /// Minimum log-odds value (clamp).
    ///
    /// Prevents overconfidence. Typical: -50.0
    pub log_odds_min: f32,

    /// Log-odds threshold for considering a cell occupied.
    ///
    /// Cells above this are drawn as occupied.
    pub occupied_threshold: f32,

    /// Log-odds threshold for considering a cell free.
    ///
    /// Cells below this are drawn as free.
    pub free_threshold: f32,
}

impl Default for OccupancyGridConfig {
    fn default() -> Self {
        Self {
            resolution: 0.02,     // 2cm cells
            initial_width: 20.0,  // 20m
            initial_height: 20.0, // 20m
            log_odds_occupied: 0.9,
            log_odds_free: -0.7,
            log_odds_max: 50.0,
            log_odds_min: -50.0,
            occupied_threshold: 0.5,
            free_threshold: -0.5,
        }
    }
}
