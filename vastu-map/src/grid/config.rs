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

/// Full map configuration
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct MapConfig {
    /// Grid configuration (size, resolution, origin)
    pub grid: GridConfig,
    /// Sensor configuration (lidar, robot geometry)
    pub sensor: SensorConfig,
}

impl MapConfig {
    /// Load configuration from a YAML file
    pub fn from_yaml_file(path: &std::path::Path) -> Result<Self, ConfigError> {
        let contents =
            std::fs::read_to_string(path).map_err(|e| ConfigError::IoError(e.to_string()))?;
        Self::from_yaml(&contents)
    }

    /// Load configuration from a YAML string
    pub fn from_yaml(yaml: &str) -> Result<Self, ConfigError> {
        serde_yaml::from_str(yaml).map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    /// Save configuration to a YAML file
    pub fn to_yaml_file(&self, path: &std::path::Path) -> Result<(), ConfigError> {
        let yaml = self.to_yaml()?;
        std::fs::write(path, yaml).map_err(|e| ConfigError::IoError(e.to_string()))
    }

    /// Serialize to YAML string
    pub fn to_yaml(&self) -> Result<String, ConfigError> {
        serde_yaml::to_string(self).map_err(|e| ConfigError::ParseError(e.to_string()))
    }
}

/// Configuration error type
#[derive(Debug, Clone)]
pub enum ConfigError {
    /// File I/O error
    IoError(String),
    /// YAML parsing error
    ParseError(String),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::IoError(msg) => write!(f, "IO error: {}", msg),
            ConfigError::ParseError(msg) => write!(f, "Parse error: {}", msg),
        }
    }
}

impl std::error::Error for ConfigError {}

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

    #[test]
    fn test_yaml_serialization() {
        let config = MapConfig::default();
        let yaml = config.to_yaml().unwrap();
        let parsed: MapConfig = MapConfig::from_yaml(&yaml).unwrap();
        assert_eq!(parsed.grid.resolution, config.grid.resolution);
    }
}
