//! Unified configuration loading for VastuMap.
//!
//! Loads all configuration from a single YAML file.

use crate::core::WorldPoint;
use crate::exploration::ExplorationConfig;
use crate::explore::ExplorerConfig;
use crate::grid::{GridConfig, MapConfig, SensorConfig};
use crate::pathfinding::{AStarConfig, SmoothingConfig};
use crate::query::{FrontierDetector, RobotFootprint};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Full VastuMap configuration loaded from YAML
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct VastuConfig {
    /// Grid settings
    #[serde(default)]
    pub grid: GridSection,

    /// Sensor settings
    #[serde(default)]
    pub sensor: SensorSection,

    /// Query settings
    #[serde(default)]
    pub query: QuerySection,

    /// Pathfinding settings
    #[serde(default)]
    pub pathfinding: PathfindingSection,

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
        }
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

/// Grid configuration section
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridSection {
    /// Cell resolution (meters)
    #[serde(default = "default_resolution")]
    pub resolution: f32,

    /// Initial grid width (cells)
    #[serde(default = "default_grid_size")]
    pub initial_width: usize,

    /// Initial grid height (cells)
    #[serde(default = "default_grid_size")]
    pub initial_height: usize,

    /// Origin mode: "center" or "corner"
    #[serde(default = "default_origin_mode")]
    pub origin_mode: String,

    /// Origin X (for custom mode)
    #[serde(default)]
    pub origin_x: f32,

    /// Origin Y (for custom mode)
    #[serde(default)]
    pub origin_y: f32,

    /// Auto expand grid
    #[serde(default = "default_true")]
    pub auto_expand: bool,
}

impl Default for GridSection {
    fn default() -> Self {
        Self {
            resolution: 0.025,
            initial_width: 800,
            initial_height: 800,
            origin_mode: "center".to_string(),
            origin_x: 0.0,
            origin_y: 0.0,
            auto_expand: true,
        }
    }
}

impl GridSection {
    /// Convert to GridConfig
    pub fn to_grid_config(&self) -> GridConfig {
        let origin = if self.origin_mode == "corner" {
            Some(WorldPoint::new(self.origin_x, self.origin_y))
        } else {
            None // Centered
        };

        GridConfig {
            resolution: self.resolution,
            initial_width: self.initial_width,
            initial_height: self.initial_height,
            origin,
            auto_expand: self.auto_expand,
            max_width: self.initial_width * 2,
            max_height: self.initial_height * 2,
        }
    }
}

/// Sensor configuration section
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct SensorSection {
    /// LiDAR settings
    #[serde(default)]
    pub lidar: LidarSettings,

    /// Robot geometry
    #[serde(default)]
    pub robot: RobotSettings,

    /// Cliff sensor positions
    #[serde(default)]
    pub cliff_sensors: CliffSensorSettings,

    /// Bumper settings
    #[serde(default)]
    pub bumper: BumperSettings,
}

impl SensorSection {
    /// Convert to SensorConfig
    pub fn to_sensor_config(&self) -> SensorConfig {
        SensorConfig {
            robot_radius: self.robot.radius,
            lidar_offset: WorldPoint::new(self.lidar.offset_x, self.lidar.offset_y),
            confidence_threshold: 3,
            max_lidar_range: self.lidar.max_range,
            min_lidar_range: self.lidar.min_range,
        }
    }
}

/// LiDAR settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LidarSettings {
    /// Offset X from robot center
    #[serde(default = "default_lidar_offset_x")]
    pub offset_x: f32,

    /// Offset Y from robot center
    #[serde(default)]
    pub offset_y: f32,

    /// Minimum valid range
    #[serde(default = "default_min_range")]
    pub min_range: f32,

    /// Maximum valid range
    #[serde(default = "default_max_range")]
    pub max_range: f32,
}

impl Default for LidarSettings {
    fn default() -> Self {
        Self {
            offset_x: -0.110,
            offset_y: 0.0,
            min_range: 0.15,
            max_range: 8.0,
        }
    }
}

/// Robot geometry settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotSettings {
    /// Robot radius (meters)
    #[serde(default = "default_robot_radius")]
    pub radius: f32,
}

impl Default for RobotSettings {
    fn default() -> Self {
        Self { radius: 0.17 }
    }
}

/// Cliff sensor positions
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CliffSensorSettings {
    /// Left side sensor position [x, y]
    pub left_side: [f32; 2],
    /// Left front sensor position [x, y]
    pub left_front: [f32; 2],
    /// Right front sensor position [x, y]
    pub right_front: [f32; 2],
    /// Right side sensor position [x, y]
    pub right_side: [f32; 2],
}

impl Default for CliffSensorSettings {
    fn default() -> Self {
        Self {
            left_side: [0.12, 0.10],
            left_front: [0.15, 0.05],
            right_front: [0.15, -0.05],
            right_side: [0.12, -0.10],
        }
    }
}

/// Bumper settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BumperSettings {
    /// Left bumper angle (radians from forward)
    #[serde(default = "default_left_bumper_angle")]
    pub left_angle: f32,

    /// Right bumper angle (radians from forward)
    #[serde(default = "default_right_bumper_angle")]
    pub right_angle: f32,
}

impl Default for BumperSettings {
    fn default() -> Self {
        Self {
            left_angle: 0.5,
            right_angle: -0.5,
        }
    }
}

/// Query settings section
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct QuerySection {
    /// Frontier detection settings
    #[serde(default)]
    pub frontier: FrontierSettings,

    /// Traversability checking settings
    #[serde(default)]
    pub traversability: TraversabilitySettings,
}

/// Frontier detection settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FrontierSettings {
    /// Minimum cells for valid frontier
    #[serde(default = "default_min_frontier_size")]
    pub min_frontier_size: usize,

    /// Use 8-connected grid (vs 4-connected)
    #[serde(default = "default_true")]
    pub use_8_connectivity: bool,
}

impl Default for FrontierSettings {
    fn default() -> Self {
        Self {
            min_frontier_size: 5,
            use_8_connectivity: true,
        }
    }
}

/// Traversability settings
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TraversabilitySettings {
    /// Extra clearance around robot (meters)
    #[serde(default = "default_safety_margin")]
    pub safety_margin: f32,
}

impl Default for TraversabilitySettings {
    fn default() -> Self {
        Self {
            safety_margin: 0.05,
        }
    }
}

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
    #[serde(default = "default_true")]
    pub allow_diagonal: bool,

    /// Cost multiplier for diagonal moves (sqrt(2))
    #[serde(default = "default_diagonal_cost")]
    pub diagonal_cost: f32,

    /// Maximum nodes to expand
    #[serde(default = "default_max_iterations")]
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
    #[serde(default = "default_true")]
    pub enabled: bool,

    /// Line-of-sight check step (meters)
    #[serde(default = "default_los_step")]
    pub los_step_size: f32,

    /// Midpoint smoothing iterations
    #[serde(default = "default_smooth_iterations")]
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

/// Exploration settings section
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExplorationSection {
    /// Minimum cells for valid frontier
    #[serde(default = "default_min_frontier_size")]
    pub min_frontier_size: usize,

    /// Distance to consider frontier reached (meters)
    #[serde(default = "default_frontier_threshold")]
    pub frontier_reached_threshold: f32,

    /// Distance for waypoint reached (meters)
    #[serde(default = "default_waypoint_threshold")]
    pub waypoint_threshold: f32,

    /// Maximum planning distance (meters)
    #[serde(default = "default_max_plan_distance")]
    pub max_plan_distance: f32,

    /// Seconds between replans when blocked
    #[serde(default = "default_replan_interval")]
    pub replan_interval: f32,

    /// Max consecutive planning failures before giving up
    #[serde(default = "default_max_failures")]
    pub max_consecutive_failures: usize,

    /// Penalty for paths near obstacles
    #[serde(default)]
    pub path_clearance_weight: f32,

    /// Enable path smoothing
    #[serde(default = "default_true")]
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
    #[serde(default = "default_turn_angle")]
    pub turn_angle: f32,

    /// Backup distance for recovery (meters)
    #[serde(default = "default_backup_distance")]
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

/// Persistence settings section
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PersistenceSection {
    /// Output format: "vastu", "pgm", or "both"
    #[serde(default = "default_output_format")]
    pub output_format: String,

    /// Output directory path
    #[serde(default = "default_output_dir")]
    pub output_dir: String,

    /// Auto-save interval (seconds, 0=disabled)
    #[serde(default)]
    pub auto_save_interval: u32,
}

impl Default for PersistenceSection {
    fn default() -> Self {
        Self {
            output_format: "both".to_string(),
            output_dir: "./output".to_string(),
            auto_save_interval: 30,
        }
    }
}

/// Config load error
#[derive(Debug, Clone)]
pub enum ConfigLoadError {
    /// I/O error
    Io(String),
    /// Parse error
    Parse(String),
}

impl std::fmt::Display for ConfigLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigLoadError::Io(msg) => write!(f, "IO error: {}", msg),
            ConfigLoadError::Parse(msg) => write!(f, "Parse error: {}", msg),
        }
    }
}

impl std::error::Error for ConfigLoadError {}

// Default value functions for serde
fn default_resolution() -> f32 {
    0.025
}
fn default_grid_size() -> usize {
    800
}
fn default_origin_mode() -> String {
    "center".to_string()
}
fn default_true() -> bool {
    true
}
fn default_lidar_offset_x() -> f32 {
    -0.110
}
fn default_min_range() -> f32 {
    0.15
}
fn default_max_range() -> f32 {
    8.0
}
fn default_robot_radius() -> f32 {
    0.17
}
fn default_left_bumper_angle() -> f32 {
    0.5
}
fn default_right_bumper_angle() -> f32 {
    -0.5
}
fn default_min_frontier_size() -> usize {
    5
}
fn default_safety_margin() -> f32 {
    0.05
}
fn default_diagonal_cost() -> f32 {
    std::f32::consts::SQRT_2
}
fn default_max_iterations() -> usize {
    100_000
}
fn default_los_step() -> f32 {
    0.05
}
fn default_smooth_iterations() -> usize {
    100
}
fn default_frontier_threshold() -> f32 {
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
fn default_max_failures() -> usize {
    5
}
fn default_turn_angle() -> f32 {
    std::f32::consts::FRAC_PI_4
}
fn default_backup_distance() -> f32 {
    0.1
}
fn default_output_format() -> String {
    "both".to_string()
}
fn default_output_dir() -> String {
    "./output".to_string()
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
