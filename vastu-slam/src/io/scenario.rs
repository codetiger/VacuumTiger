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

/// Default maximum wheel velocity for auto-calculating duration (m/s)
const DEFAULT_MAX_WHEEL_VELOCITY: f32 = 0.3;

/// Default wheel base for differential drive kinematics (meters)
const DEFAULT_WHEEL_BASE: f32 = 0.233;

/// Default angular velocity for rotation commands (rad/s)
const DEFAULT_ANGULAR_VELOCITY: f32 = 0.5;

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
    /// Move by specifying wheel distances directly (more accurate for testing)
    MoveDistance {
        /// Left wheel distance in meters
        left_distance_m: f32,
        /// Right wheel distance in meters
        right_distance_m: f32,
        /// Optional duration in ms (auto-calculated if omitted based on max velocity)
        #[serde(default)]
        duration_ms: Option<u64>,
    },
    /// Rotate in place by a specified angle in degrees
    Rotate {
        /// Rotation angle in degrees (positive = CCW, negative = CW)
        degrees: f32,
        /// Optional duration in ms (auto-calculated if omitted based on angular velocity)
        #[serde(default)]
        duration_ms: Option<u64>,
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
            Command::MoveDistance {
                left_distance_m,
                right_distance_m,
                duration_ms,
            } => {
                // Use provided duration or auto-calculate from max wheel velocity
                duration_ms.unwrap_or_else(|| {
                    let max_distance = left_distance_m.abs().max(right_distance_m.abs());
                    let duration_secs = max_distance / DEFAULT_MAX_WHEEL_VELOCITY;
                    (duration_secs * 1000.0) as u64
                })
            }
            Command::Rotate { degrees, duration_ms } => {
                // Use provided duration or auto-calculate from angular velocity
                duration_ms.unwrap_or_else(|| {
                    let radians = degrees.to_radians();
                    let duration_secs = radians.abs() / DEFAULT_ANGULAR_VELOCITY;
                    (duration_secs * 1000.0) as u64
                })
            }
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
            Command::MoveDistance {
                left_distance_m,
                right_distance_m,
                ..
            } => {
                // Calculate velocities from wheel distances and duration
                let duration_secs = self.duration_secs();
                if duration_secs <= 0.0 {
                    return (0.0, 0.0);
                }

                // Wheel velocities
                let vel_left = left_distance_m / duration_secs;
                let vel_right = right_distance_m / duration_secs;

                // Differential drive kinematics:
                // v = (v_r + v_l) / 2
                // ω = (v_r - v_l) / wheel_base
                let linear_velocity = (vel_left + vel_right) / 2.0;
                let angular_velocity = (vel_right - vel_left) / DEFAULT_WHEEL_BASE;

                (linear_velocity, angular_velocity)
            }
            Command::Rotate { degrees, .. } => {
                // Pure rotation: no linear velocity
                let duration_secs = self.duration_secs();
                if duration_secs <= 0.0 {
                    return (0.0, 0.0);
                }

                let radians = degrees.to_radians();
                let angular_velocity = radians / duration_secs;

                (0.0, angular_velocity)
            }
            Command::Stay { .. } => (0.0, 0.0),
        }
    }

    /// Get wheel distances (left, right) in meters, if applicable
    pub fn wheel_distances(&self) -> Option<(f32, f32)> {
        match self {
            Command::MoveDistance {
                left_distance_m,
                right_distance_m,
                ..
            } => Some((*left_distance_m, *right_distance_m)),
            Command::Rotate { degrees, .. } => {
                // Calculate wheel arc lengths for rotation
                // Arc length = angle × (wheel_base / 2)
                // For CW rotation (negative degrees): left forward (+), right backward (-)
                // For CCW rotation (positive degrees): left backward (-), right forward (+)
                let radians = degrees.to_radians();
                let arc_length = radians * (DEFAULT_WHEEL_BASE / 2.0);
                // arc_length is negative for CW (negative degrees)
                // -arc_length flips to positive for left wheel (forward)
                // arc_length stays negative for right wheel (backward)
                Some((-arc_length, arc_length))
            }
            _ => None,
        }
    }

    /// Get rotation angle in degrees, if applicable
    pub fn rotation_degrees(&self) -> Option<f32> {
        match self {
            Command::Rotate { degrees, .. } => Some(*degrees),
            _ => None,
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

    #[test]
    fn test_move_distance_parsing() {
        let yaml = r#"
name: "Distance Test"
map: "room"
commands:
  - type: move_distance
    left_distance_m: 0.5
    right_distance_m: 0.5
  - type: move_distance
    left_distance_m: 1.0
    right_distance_m: 1.0
    duration_ms: 5000
"#;
        let scenario = Scenario::from_yaml(yaml).unwrap();
        assert_eq!(scenario.commands.len(), 2);

        // First command: auto-calculated duration (0.5m / 0.3m/s = 1.667s)
        let cmd1 = &scenario.commands[0];
        assert_eq!(cmd1.wheel_distances(), Some((0.5, 0.5)));
        let duration_ms = cmd1.duration_ms();
        assert!(duration_ms > 1600 && duration_ms < 1700, "Expected ~1667ms, got {}", duration_ms);

        // Second command: explicit duration
        let cmd2 = &scenario.commands[1];
        assert_eq!(cmd2.duration_ms(), 5000);
    }

    #[test]
    fn test_move_distance_velocities_forward() {
        // Straight forward: equal wheel distances
        let cmd = Command::MoveDistance {
            left_distance_m: 0.6,
            right_distance_m: 0.6,
            duration_ms: Some(2000), // 2 seconds
        };

        let (linear, angular) = cmd.velocities();
        // vel_left = vel_right = 0.6m / 2s = 0.3 m/s
        // linear = (0.3 + 0.3) / 2 = 0.3 m/s
        // angular = (0.3 - 0.3) / 0.233 = 0 rad/s
        assert!((linear - 0.3).abs() < 1e-6, "Expected linear=0.3, got {}", linear);
        assert!(angular.abs() < 1e-6, "Expected angular=0, got {}", angular);
    }

    #[test]
    fn test_move_distance_velocities_rotation() {
        // Rotation in place: opposite wheel distances
        let cmd = Command::MoveDistance {
            left_distance_m: -0.1165, // half wheel base
            right_distance_m: 0.1165,
            duration_ms: Some(1000), // 1 second
        };

        let (linear, angular) = cmd.velocities();
        // vel_left = -0.1165 m/s, vel_right = 0.1165 m/s
        // linear = (-0.1165 + 0.1165) / 2 = 0 m/s
        // angular = (0.1165 - (-0.1165)) / 0.233 = 0.233 / 0.233 = 1 rad/s
        assert!(linear.abs() < 1e-6, "Expected linear=0, got {}", linear);
        assert!((angular - 1.0).abs() < 0.01, "Expected angular=1.0, got {}", angular);
    }

    #[test]
    fn test_move_distance_auto_duration() {
        // Auto-calculated duration based on max wheel velocity (0.3 m/s)
        let cmd = Command::MoveDistance {
            left_distance_m: 0.9,
            right_distance_m: 0.6,
            duration_ms: None,
        };

        // Duration based on max distance: 0.9m / 0.3m/s = 3s = 3000ms
        // Allow for floating point rounding (2999 or 3000)
        let duration = cmd.duration_ms();
        assert!(
            (2999..=3001).contains(&duration),
            "Expected ~3000ms, got {}",
            duration
        );

        // Velocities with ~3s duration
        let (linear, angular) = cmd.velocities();
        // vel_left = 0.9/3 = 0.3, vel_right = 0.6/3 = 0.2
        // linear = (0.3 + 0.2) / 2 = 0.25
        // angular = (0.2 - 0.3) / 0.233 ≈ -0.429
        assert!((linear - 0.25).abs() < 0.01);
        assert!((angular + 0.429).abs() < 0.02);
    }

    #[test]
    fn test_rotate_parsing() {
        let yaml = r#"
name: "Rotate Test"
map: "room"
commands:
  - type: rotate
    degrees: 90
  - type: rotate
    degrees: -180
    duration_ms: 3000
"#;
        let scenario = Scenario::from_yaml(yaml).unwrap();
        assert_eq!(scenario.commands.len(), 2);

        // First command: 90° CCW, auto-calculated duration
        let cmd1 = &scenario.commands[0];
        assert_eq!(cmd1.rotation_degrees(), Some(90.0));

        // Second command: 180° CW with explicit duration
        let cmd2 = &scenario.commands[1];
        assert_eq!(cmd2.rotation_degrees(), Some(-180.0));
        assert_eq!(cmd2.duration_ms(), 3000);
    }

    #[test]
    fn test_rotate_velocities_ccw() {
        // 90° CCW rotation with explicit duration
        let cmd = Command::Rotate {
            degrees: 90.0,
            duration_ms: Some(1000), // 1 second
        };

        let (linear, angular) = cmd.velocities();
        // linear = 0 (pure rotation)
        // angular = π/2 / 1s ≈ 1.571 rad/s
        assert!(linear.abs() < 1e-6, "Expected linear=0, got {}", linear);
        assert!(
            (angular - std::f32::consts::FRAC_PI_2).abs() < 0.01,
            "Expected angular≈1.571, got {}",
            angular
        );
    }

    #[test]
    fn test_rotate_velocities_cw() {
        // 90° CW rotation (negative degrees)
        let cmd = Command::Rotate {
            degrees: -90.0,
            duration_ms: Some(1000),
        };

        let (linear, angular) = cmd.velocities();
        assert!(linear.abs() < 1e-6);
        assert!(
            (angular + std::f32::consts::FRAC_PI_2).abs() < 0.01,
            "Expected angular≈-1.571, got {}",
            angular
        );
    }

    #[test]
    fn test_rotate_auto_duration() {
        // 360° rotation with default angular velocity (0.5 rad/s)
        let cmd = Command::Rotate {
            degrees: 360.0,
            duration_ms: None,
        };

        // Duration = 2π / 0.5 rad/s ≈ 12.566s ≈ 12566ms
        let duration = cmd.duration_ms();
        assert!(
            (12500..=12600).contains(&duration),
            "Expected ~12566ms, got {}",
            duration
        );
    }

    #[test]
    fn test_rotate_wheel_distances() {
        // 360° CCW rotation
        let cmd = Command::Rotate {
            degrees: 360.0,
            duration_ms: None,
        };

        let (left, right) = cmd.wheel_distances().unwrap();
        // Arc length = 2π × (0.233/2) ≈ 0.732m
        // CCW: left backward (-), right forward (+)
        assert!(
            (left + 0.732).abs() < 0.01,
            "Expected left≈-0.732, got {}",
            left
        );
        assert!(
            (right - 0.732).abs() < 0.01,
            "Expected right≈0.732, got {}",
            right
        );
    }
}
