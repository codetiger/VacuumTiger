//! Main VastuConfig and conversion methods.

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::explore::{ExplorationConfig, ExplorerConfig};
use crate::grid::MapConfig;
use crate::pathfinding::{AStarConfig, SmoothingConfig};
use crate::query::{FrontierDetector, RobotFootprint};
use crate::slam::{
    BackgroundOptimizerConfig, CorrelativeMatcherConfig, LoopClosureConfig, PoseGraphConfig,
};

use super::error::ConfigLoadError;
use super::exploration::ExplorationSection;
use super::grid::GridSection;
use super::motion_filter::MotionFilterSection;
use super::pathfinding::PathfindingSection;
use super::persistence::PersistenceSection;
use super::query::QuerySection;
use super::sensor::SensorSection;
use super::slam::SlamSection;

/// Full VastuMap configuration loaded from YAML
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

    /// Query settings
    #[serde(default)]
    pub query: QuerySection,

    /// Pathfinding settings
    #[serde(default)]
    pub pathfinding: PathfindingSection,

    /// Motion filter settings
    #[serde(default)]
    pub motion_filter: MotionFilterSection,

    /// Exploration settings
    #[serde(default)]
    pub exploration: ExplorationSection,

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

    /// Get robot footprint
    pub fn robot_footprint(&self) -> RobotFootprint {
        RobotFootprint::new(
            self.sensor.robot.radius,
            self.query.traversability.safety_margin,
        )
    }

    /// Convert to ExplorationConfig
    pub fn to_exploration_config(&self) -> ExplorationConfig {
        ExplorationConfig {
            footprint: self.robot_footprint(),
            min_frontier_size: self.exploration.min_frontier_size,
            frontier_reached_threshold: self.exploration.frontier_reached_threshold,
            waypoint_threshold: self.exploration.waypoint_threshold,
            max_plan_distance: self.exploration.max_plan_distance,
            replan_interval: self.exploration.replan_interval,
            max_consecutive_failures: self.exploration.max_consecutive_failures,
            path_clearance_weight: self.exploration.path_clearance_weight,
            smooth_paths: self.exploration.smooth_paths,
            recovery_turn_angle: self.exploration.recovery.turn_angle,
            recovery_backup_distance: self.exploration.recovery.backup_distance,
            // Use defaults for new fields
            ..Default::default()
        }
    }

    /// Convert to AStarConfig
    pub fn to_astar_config(&self) -> AStarConfig {
        AStarConfig {
            footprint: self.robot_footprint(),
            allow_diagonal: self.pathfinding.astar.allow_diagonal,
            diagonal_cost: self.pathfinding.astar.diagonal_cost,
            max_iterations: self.pathfinding.astar.max_iterations,
            clearance_weight: self.pathfinding.astar.clearance_weight,
            exploration_mode: false, // Default to safe mode
        }
    }

    /// Convert to SmoothingConfig
    pub fn to_smoothing_config(&self) -> SmoothingConfig {
        SmoothingConfig {
            footprint: self.robot_footprint(),
            los_step_size: self.pathfinding.smoothing.los_step_size,
            max_iterations: self.pathfinding.smoothing.max_iterations,
        }
    }

    /// Convert to ExplorerConfig for VastuExplorer
    pub fn to_explorer_config(&self) -> ExplorerConfig {
        ExplorerConfig {
            grid: self.grid.to_grid_config(),
            sensor: self.sensor.to_sensor_config(),
            exploration: self.to_exploration_config(),
            motion_filter: self.motion_filter.to_motion_filter_config(),
            ..ExplorerConfig::default()
        }
    }

    /// Create frontier detector with config settings
    pub fn frontier_detector(&self) -> FrontierDetector {
        FrontierDetector {
            min_frontier_size: self.query.frontier.min_frontier_size,
            use_8_connectivity: self.query.frontier.use_8_connectivity,
        }
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
