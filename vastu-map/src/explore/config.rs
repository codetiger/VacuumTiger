//! Explorer configuration.

use serde::{Deserialize, Serialize};

use crate::exploration::ExplorationConfig;
use crate::grid::{GridConfig, MapConfig, SensorConfig};
use crate::slam::CorrelativeMatcherConfig;
use crate::slam::loop_closure::{LoopClosureConfig, PoseGraphConfig};

/// Configuration for the VastuExplorer.
///
/// This combines all the configuration needed for autonomous exploration:
/// - Grid/map settings
/// - Sensor integration
/// - Scan matching
/// - Loop closure
/// - Exploration behavior
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExplorerConfig {
    /// Grid storage configuration.
    #[serde(default)]
    pub grid: GridConfig,

    /// Sensor integration configuration.
    #[serde(default)]
    pub sensor: SensorConfig,

    /// Exploration controller configuration.
    #[serde(default)]
    pub exploration: ExplorationConfig,

    /// Scan-to-map matching configuration.
    #[serde(default)]
    pub matching: CorrelativeMatcherConfig,

    /// Loop closure detection configuration.
    #[serde(default)]
    pub loop_closure: LoopClosureConfig,

    /// Pose graph optimization configuration.
    #[serde(default)]
    pub pose_graph: PoseGraphConfig,

    /// Control loop update rate in Hz.
    #[serde(default = "default_update_rate")]
    pub update_rate_hz: f32,

    /// Maximum exploration time in seconds (0 = unlimited).
    #[serde(default)]
    pub max_time_secs: f32,

    /// Stop if battery falls below this percentage.
    #[serde(default = "default_min_battery")]
    pub min_battery_percent: u8,
}

fn default_update_rate() -> f32 {
    10.0 // 10 Hz
}

fn default_min_battery() -> u8 {
    20 // 20%
}

impl Default for ExplorerConfig {
    fn default() -> Self {
        Self {
            grid: GridConfig::default(),
            sensor: SensorConfig::default(),
            exploration: ExplorationConfig::default(),
            matching: CorrelativeMatcherConfig::default(),
            loop_closure: LoopClosureConfig::default(),
            pose_graph: PoseGraphConfig::default(),
            update_rate_hz: default_update_rate(),
            max_time_secs: 0.0,
            min_battery_percent: default_min_battery(),
        }
    }
}

impl ExplorerConfig {
    /// Create a fast configuration for quick testing.
    pub fn fast() -> Self {
        Self {
            matching: CorrelativeMatcherConfig::fast(),
            update_rate_hz: 5.0,
            ..Default::default()
        }
    }

    /// Create a thorough configuration for best quality maps.
    pub fn thorough() -> Self {
        Self {
            matching: CorrelativeMatcherConfig::thorough(),
            update_rate_hz: 10.0,
            ..Default::default()
        }
    }

    /// Convert to MapConfig for OccupancyGridMap creation.
    pub fn to_map_config(&self) -> MapConfig {
        MapConfig {
            grid: self.grid.clone(),
            sensor: self.sensor.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = ExplorerConfig::default();
        assert!((config.update_rate_hz - 10.0).abs() < 0.001);
        assert_eq!(config.min_battery_percent, 20);
    }

    #[test]
    fn test_fast_config() {
        let config = ExplorerConfig::fast();
        assert!((config.update_rate_hz - 5.0).abs() < 0.001);
    }
}
