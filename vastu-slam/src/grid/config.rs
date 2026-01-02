//! Configuration types for the occupancy grid.

use crate::core::WorldPoint;
use serde::{Deserialize, Serialize};

/// Grid configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridConfig {
    /// Meters per cell (e.g., 0.025 = 2.5cm cells)
    pub resolution: f32,

    /// Initial grid width in cells
    pub initial_width: usize,

    /// Initial grid height in cells
    pub initial_height: usize,

    /// World coordinates of cell (0,0) corner
    /// If None, grid will be centered at origin
    pub origin: Option<WorldPoint>,

    /// Enable dynamic grid expansion when robot moves beyond bounds
    pub auto_expand: bool,

    /// Maximum grid width in cells (to limit memory)
    pub max_width: usize,

    /// Maximum grid height in cells (to limit memory)
    pub max_height: usize,
}

impl Default for GridConfig {
    fn default() -> Self {
        Self {
            resolution: 0.025,   // 2.5cm cells
            initial_width: 800,  // 20m at 2.5cm
            initial_height: 800, // 20m at 2.5cm
            origin: None,        // Centered at origin
            auto_expand: true,
            max_width: 2000,  // 50m max
            max_height: 2000, // 50m max
        }
    }
}

impl GridConfig {
    /// Create a configuration for a specific area size (in meters)
    pub fn for_area(width_m: f32, height_m: f32, resolution: f32) -> Self {
        let width = (width_m / resolution).ceil() as usize;
        let height = (height_m / resolution).ceil() as usize;

        Self {
            resolution,
            initial_width: width,
            initial_height: height,
            origin: None,
            auto_expand: true,
            max_width: width * 2,
            max_height: height * 2,
        }
    }

    /// Calculate the origin for a centered grid
    pub fn centered_origin(&self) -> WorldPoint {
        let half_width = (self.initial_width as f32 * self.resolution) / 2.0;
        let half_height = (self.initial_height as f32 * self.resolution) / 2.0;
        WorldPoint::new(-half_width, -half_height)
    }

    /// Get the effective origin (uses centered_origin if origin is None)
    pub fn effective_origin(&self) -> WorldPoint {
        self.origin.unwrap_or_else(|| self.centered_origin())
    }

    /// Calculate memory usage in bytes for the initial grid
    pub fn initial_memory_bytes(&self) -> usize {
        // Each Cell is 4 bytes (CellType + confidence + observation_count + swept)
        self.initial_width * self.initial_height * 4
    }

    /// Calculate maximum memory usage in bytes
    pub fn max_memory_bytes(&self) -> usize {
        self.max_width * self.max_height * 4
    }
}

/// Sensor configuration for grid updates
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorConfig {
    /// Robot radius in meters (for obstacle inflation)
    pub robot_radius: f32,

    /// Lidar mounting offset from robot center (x, y) in robot frame
    pub lidar_offset: WorldPoint,

    /// How many observations needed for a cell to be "confident"
    pub confidence_threshold: u8,

    /// Maximum lidar range to trust (meters)
    pub max_lidar_range: f32,

    /// Minimum lidar range to trust (meters)
    pub min_lidar_range: f32,
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            robot_radius: 0.17,
            lidar_offset: WorldPoint::new(-0.110, 0.0), // From SangamIO CRL-200S config
            confidence_threshold: 3,
            max_lidar_range: 8.0,
            min_lidar_range: 0.15,
        }
    }
}

/// Configuration for log-odds occupancy updates.
///
/// Based on Google Cartographer's probability grid model:
/// - Log-odds: L(x) = log(P(x) / (1 - P(x)))
/// - Bayesian update: L_new = L_old + L_observation
/// - Stored as fixed-point i16: actual = value / 100
///
/// Lower values = more observations needed to establish walls.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LogOddsConfig {
    /// Log-odds increment for hit (lidar endpoint = occupied).
    /// Cartographer default: ~20 (hit_probability=0.55)
    /// VastuMap aggressive: 70 (P=0.67)
    #[serde(default = "default_l_hit")]
    pub l_hit: i16,

    /// Log-odds decrement for miss (ray passes through = free).
    /// Cartographer default: ~-4 (miss_probability=0.49)
    /// VastuMap aggressive: -28 (P=0.43)
    /// Asymmetric to make walls "stickier" than free space.
    #[serde(default = "default_l_miss")]
    pub l_miss: i16,

    /// Threshold for considering cell occupied. P > 0.62 → L > 50
    #[serde(default = "default_l_occupied")]
    pub l_occupied_threshold: i16,

    /// Threshold for considering cell free. P < 0.38 → L < -50
    #[serde(default = "default_l_free")]
    pub l_free_threshold: i16,

    /// Minimum log-odds value (clamping). P ≈ 0.12
    #[serde(default = "default_l_min")]
    pub l_min: i16,

    /// Maximum log-odds value (clamping). P ≈ 0.88
    #[serde(default = "default_l_max")]
    pub l_max: i16,
}

fn default_l_hit() -> i16 {
    20
} // P≈0.55 (Cartographer default)
fn default_l_miss() -> i16 {
    -4
} // P≈0.49 (Cartographer default)
fn default_l_occupied() -> i16 {
    50
}
fn default_l_free() -> i16 {
    -50
}
fn default_l_min() -> i16 {
    -200
}
fn default_l_max() -> i16 {
    200
}

impl Default for LogOddsConfig {
    fn default() -> Self {
        Self {
            l_hit: default_l_hit(),
            l_miss: default_l_miss(),
            l_occupied_threshold: default_l_occupied(),
            l_free_threshold: default_l_free(),
            l_min: default_l_min(),
            l_max: default_l_max(),
        }
    }
}

impl LogOddsConfig {
    /// Cartographer-like conservative updates (DEFAULT).
    /// Requires 3-5 observations to establish wall.
    /// This is the recommended default for robust mapping.
    pub fn cartographer() -> Self {
        Self::default()
    }

    /// Aggressive updates (original VastuMap behavior).
    /// Single observation can establish wall.
    /// Use for fast mapping in controlled environments.
    pub fn aggressive() -> Self {
        Self {
            l_hit: 70,   // P=0.67
            l_miss: -28, // P=0.43
            ..Default::default()
        }
    }

    /// Balanced - 2-3 observations for wall.
    /// Mid-ground between Cartographer and aggressive.
    pub fn balanced() -> Self {
        Self {
            l_hit: 30,   // P=0.57
            l_miss: -12, // P=0.47
            ..Default::default()
        }
    }

    /// Convert hit probability to log-odds.
    /// Formula: L = 100 * log(p / (1 - p))
    pub fn from_probability(hit_prob: f32, miss_prob: f32) -> Self {
        let l_hit = (100.0 * (hit_prob / (1.0 - hit_prob)).ln()) as i16;
        let l_miss = (100.0 * (miss_prob / (1.0 - miss_prob)).ln()) as i16;
        Self {
            l_hit,
            l_miss,
            ..Default::default()
        }
    }

    /// Convert log-odds to probability.
    /// Formula: P = exp(L/100) / (1 + exp(L/100))
    pub fn log_odds_to_probability(log_odds: i16) -> f32 {
        let l = log_odds as f32 / 100.0;
        let exp_l = l.exp();
        exp_l / (1.0 + exp_l)
    }
}

/// Full map configuration
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct MapConfig {
    /// Grid configuration (size, resolution, origin)
    pub grid: GridConfig,
    /// Sensor configuration (lidar, robot geometry)
    pub sensor: SensorConfig,
    /// Log-odds occupancy configuration
    #[serde(default)]
    pub log_odds: LogOddsConfig,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = GridConfig::default();
        assert_eq!(config.resolution, 0.025);
        assert_eq!(config.initial_width, 800);
    }

    #[test]
    fn test_for_area() {
        let config = GridConfig::for_area(10.0, 10.0, 0.05);
        assert_eq!(config.initial_width, 200);
        assert_eq!(config.initial_height, 200);
    }

    #[test]
    fn test_memory_calculation() {
        let config = GridConfig::default();
        let mem = config.initial_memory_bytes();
        // 800 * 800 * 4 = 2,560,000 bytes = 2.44 MB
        assert_eq!(mem, 2560000);
    }
}
