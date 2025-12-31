//! Scenario YAML parsing for scan matching tests.
//!
//! A scenario defines a test sequence with:
//! - Map to load
//! - Starting pose
//! - Sequence of movement commands (move/stay)
//! - SVG output configuration

use serde::{Deserialize, Serialize};
use std::path::Path;

/// A test scenario loaded from YAML
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Scenario {
    /// Human-readable scenario name
    pub name: String,

    /// Optional description
    #[serde(default)]
    pub description: String,

    /// Map name (looked up in maps/ folder) or full path
    pub map: String,

    /// Initial pose (None = auto-detect free position)
    #[serde(default)]
    pub start_pose: Option<StartPose>,

    /// Seed configuration for initial map building
    #[serde(default)]
    pub seed: SeedConfig,

    /// Sequence of movement commands
    pub commands: Vec<Command>,

    /// SVG output configuration
    #[serde(default)]
    pub svg_output: SvgOutputConfig,
}

/// Starting pose specification
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StartPose {
    /// X position in meters
    pub x: f32,
    /// Y position in meters
    pub y: f32,
    /// Orientation in radians (CCW positive from +X)
    #[serde(default)]
    pub theta: f32,
}

/// Map seeding configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SeedConfig {
    /// Number of scans for seeding
    #[serde(default = "default_seed_scans")]
    pub scans: usize,

    /// Whether to rotate during seeding (360° coverage)
    #[serde(default = "default_true")]
    pub rotation: bool,
}

impl Default for SeedConfig {
    fn default() -> Self {
        Self {
            scans: 20,
            rotation: true,
        }
    }
}

/// A movement command
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Command {
    /// Move with given velocities for a duration
    Move {
        /// Linear velocity in m/s
        linear_velocity: f32,
        /// Angular velocity in rad/s
        angular_velocity: f32,
        /// Duration in milliseconds
        duration_ms: u64,
    },
    /// Stay still for a duration
    Stay {
        /// Duration in milliseconds
        duration_ms: u64,
    },
}

impl Command {
    /// Get duration in milliseconds
    pub fn duration_ms(&self) -> u64 {
        match self {
            Command::Move { duration_ms, .. } => *duration_ms,
            Command::Stay { duration_ms } => *duration_ms,
        }
    }

    /// Get duration in seconds
    pub fn duration_secs(&self) -> f32 {
        self.duration_ms() as f32 / 1000.0
    }

    /// Get velocities (linear, angular)
    pub fn velocities(&self) -> (f32, f32) {
        match self {
            Command::Move {
                linear_velocity,
                angular_velocity,
                ..
            } => (*linear_velocity, *angular_velocity),
            Command::Stay { .. } => (0.0, 0.0),
        }
    }
}

/// SVG output configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SvgOutputConfig {
    /// Output filename (default: scenario name + .svg)
    #[serde(default)]
    pub filename: Option<String>,

    /// Scale: pixels per meter
    #[serde(default = "default_svg_scale")]
    pub scale: f32,

    /// Distance interval for pose markers (meters)
    #[serde(default)]
    pub marker_interval_m: Option<f32>,

    /// Time interval for pose markers (seconds)
    #[serde(default)]
    pub marker_interval_s: Option<f32>,

    /// Whether to show individual lidar scan points
    #[serde(default)]
    pub show_scan_points: bool,

    /// Line width for trajectories
    #[serde(default = "default_trajectory_width")]
    pub trajectory_width: f32,
}

impl Default for SvgOutputConfig {
    fn default() -> Self {
        Self {
            filename: None,
            scale: 50.0,
            marker_interval_m: Some(0.5),
            marker_interval_s: None,
            show_scan_points: false,
            trajectory_width: 2.0,
        }
    }
}

// Default value functions for serde
fn default_seed_scans() -> usize {
    20
}
fn default_true() -> bool {
    true
}
fn default_svg_scale() -> f32 {
    50.0
}
fn default_trajectory_width() -> f32 {
    2.0
}

/// Error type for scenario loading
#[derive(Debug, Clone)]
pub enum ScenarioError {
    /// I/O error reading file
    Io(String),
    /// YAML parsing error
    Parse(String),
    /// Invalid map reference
    InvalidMap(String),
}

impl std::fmt::Display for ScenarioError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ScenarioError::Io(msg) => write!(f, "I/O error: {}", msg),
            ScenarioError::Parse(msg) => write!(f, "Parse error: {}", msg),
            ScenarioError::InvalidMap(msg) => write!(f, "Invalid map: {}", msg),
        }
    }
}

impl std::error::Error for ScenarioError {}

impl Scenario {
    /// Load scenario from YAML file
    pub fn load(path: &Path) -> Result<Self, ScenarioError> {
        let contents =
            std::fs::read_to_string(path).map_err(|e| ScenarioError::Io(e.to_string()))?;
        Self::from_yaml(&contents)
    }

    /// Parse from YAML string
    pub fn from_yaml(yaml: &str) -> Result<Self, ScenarioError> {
        serde_yaml::from_str(yaml).map_err(|e| ScenarioError::Parse(e.to_string()))
    }

    /// Total duration of all commands in milliseconds
    pub fn total_duration_ms(&self) -> u64 {
        self.commands.iter().map(|c| c.duration_ms()).sum()
    }

    /// Total duration of all commands in seconds
    pub fn total_duration_secs(&self) -> f32 {
        self.total_duration_ms() as f32 / 1000.0
    }

    /// Resolve map path (handles both name and full path)
    pub fn resolve_map_path(&self, maps_dir: &Path) -> std::path::PathBuf {
        let map_path = Path::new(&self.map);
        if map_path.is_absolute() || map_path.exists() {
            map_path.to_path_buf()
        } else {
            maps_dir.join(format!("{}.yaml", self.map))
        }
    }

    /// Get the SVG output filename
    pub fn svg_filename(&self) -> String {
        self.svg_output
            .filename
            .clone()
            .unwrap_or_else(|| format!("{}.svg", self.name.to_lowercase().replace(' ', "_")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_scenario() {
        let yaml = r#"
name: "Test Scenario"
map: "medium_room"
commands:
  - type: move
    linear_velocity: 0.2
    angular_velocity: 0.0
    duration_ms: 5000
  - type: stay
    duration_ms: 1000
"#;
        let scenario = Scenario::from_yaml(yaml).unwrap();
        assert_eq!(scenario.name, "Test Scenario");
        assert_eq!(scenario.map, "medium_room");
        assert_eq!(scenario.commands.len(), 2);
        assert_eq!(scenario.total_duration_ms(), 6000);
    }

    #[test]
    fn test_parse_with_start_pose() {
        let yaml = r#"
name: "With Start Pose"
map: "simple_room"
start_pose:
  x: 2.0
  y: 3.0
  theta: 1.57
commands:
  - type: move
    linear_velocity: 0.1
    angular_velocity: 0.5
    duration_ms: 2000
"#;
        let scenario = Scenario::from_yaml(yaml).unwrap();
        assert!(scenario.start_pose.is_some());
        let pose = scenario.start_pose.unwrap();
        assert!((pose.x - 2.0).abs() < 1e-6);
        assert!((pose.y - 3.0).abs() < 1e-6);
        assert!((pose.theta - 1.57).abs() < 1e-6);
    }

    #[test]
    fn test_command_velocities() {
        let move_cmd = Command::Move {
            linear_velocity: 0.2,
            angular_velocity: 0.5,
            duration_ms: 1000,
        };
        assert_eq!(move_cmd.velocities(), (0.2, 0.5));
        assert_eq!(move_cmd.duration_secs(), 1.0);

        let stay_cmd = Command::Stay { duration_ms: 500 };
        assert_eq!(stay_cmd.velocities(), (0.0, 0.0));
        assert_eq!(stay_cmd.duration_secs(), 0.5);
    }

    #[test]
    fn test_svg_filename_generation() {
        let yaml = r#"
name: "My Test Scenario"
map: "room"
commands: []
"#;
        let scenario = Scenario::from_yaml(yaml).unwrap();
        assert_eq!(scenario.svg_filename(), "my_test_scenario.svg");
    }
}
