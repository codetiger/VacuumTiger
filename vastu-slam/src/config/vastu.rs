//! Main VastuConfig and conversion methods.

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::grid::MapConfig;
use crate::matching::{
    BackgroundOptimizerConfig, CorrelativeMatcherConfig, LoopClosureConfig, PoseGraphConfig,
};

use super::error::ConfigLoadError;
use super::grid::GridSection;
use super::persistence::PersistenceSection;
use super::sensor::SensorSection;
use super::slam::SlamSection;

/// Full VastuSLAM configuration loaded from YAML
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct VastuConfig {
    /// Grid settings
    #[serde(default)]
    pub grid: GridSection,

    /// Sensor settings
    #[serde(default)]
    pub sensor: SensorSection,

    /// SLAM settings (Cartographer-style defaults)
    #[serde(default)]
    pub slam: SlamSection,

    /// Persistence settings
    #[serde(default)]
    pub persistence: PersistenceSection,
}

impl VastuConfig {
    /// Load configuration from a YAML file
    pub fn load(path: &Path) -> Result<Self, ConfigLoadError> {
        let contents =
            std::fs::read_to_string(path).map_err(|e| ConfigLoadError::Io(e.to_string()))?;
        Self::from_yaml(&contents)
    }

    /// Load from default config path (configs/config.yaml)
    pub fn load_default() -> Result<Self, ConfigLoadError> {
        let path = Path::new("configs/config.yaml");
        if path.exists() {
            Self::load(path)
        } else {
            Ok(Self::default())
        }
    }

    /// Parse from YAML string
    pub fn from_yaml(yaml: &str) -> Result<Self, ConfigLoadError> {
        serde_yaml::from_str(yaml).map_err(|e| ConfigLoadError::Parse(e.to_string()))
    }

    /// Convert to MapConfig for OccupancyGridMap
    pub fn to_map_config(&self) -> MapConfig {
        MapConfig {
            grid: self.grid.to_grid_config(),
            sensor: self.sensor.to_sensor_config(),
            log_odds: self.slam.log_odds.clone(),
        }
    }

    /// Get the correlative matcher config
    pub fn correlative_matcher_config(&self) -> CorrelativeMatcherConfig {
        self.slam.correlative.clone()
    }

    /// Get the loop closure config
    pub fn loop_closure_config(&self) -> LoopClosureConfig {
        self.slam.loop_closure.clone()
    }

    /// Get the pose graph config
    pub fn pose_graph_config(&self) -> PoseGraphConfig {
        self.slam.pose_graph.clone()
    }

    /// Get the background optimizer config
    pub fn background_optimizer_config(&self) -> BackgroundOptimizerConfig {
        self.slam.background_optimizer.clone()
    }

    /// Get robot radius
    pub fn robot_radius(&self) -> f32 {
        self.sensor.robot.radius
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = VastuConfig::default();
        assert_eq!(config.grid.resolution, 0.025);
        assert_eq!(config.sensor.robot.radius, 0.17);
    }

    #[test]
    fn test_yaml_roundtrip() {
        let config = VastuConfig::default();
        let yaml = serde_yaml::to_string(&config).unwrap();
        let parsed: VastuConfig = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(parsed.grid.resolution, config.grid.resolution);
    }

    #[test]
    fn test_to_map_config() {
        let config = VastuConfig::default();
        let map_config = config.to_map_config();
        assert_eq!(map_config.grid.resolution, 0.025);
        assert_eq!(map_config.sensor.robot_radius, 0.17);
    }
}
