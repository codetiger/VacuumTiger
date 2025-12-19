//! YAML configuration structures for scenario-based tests
//!
//! Allows defining test scenarios in YAML files with wheel distance commands
//! for precise, encoder-based robot movement.

use serde::Deserialize;

/// Top-level scenario configuration loaded from YAML.
#[derive(Debug, Deserialize)]
pub struct ScenarioConfig {
    /// Scenario name (used for output files)
    pub name: String,
    /// Path to map YAML file (relative to scenario file)
    pub map: String,
    /// Starting pose
    #[serde(default)]
    pub start_pose: PoseConfig,
    /// Path as wheel distance commands
    pub path: Vec<WheelCommand>,
}

/// Robot pose configuration.
#[derive(Debug, Deserialize)]
pub struct PoseConfig {
    /// X position in meters
    #[serde(default = "default_2_5")]
    pub x: f32,
    /// Y position in meters
    #[serde(default = "default_2_5")]
    pub y: f32,
    /// Orientation in radians
    #[serde(default)]
    pub theta: f32,
}

impl Default for PoseConfig {
    fn default() -> Self {
        Self {
            x: 2.5,
            y: 2.5,
            theta: 0.0,
        }
    }
}

/// Wheel distance command - precise encoder-based movement.
///
/// For differential drive robot (wheel_base = 0.233m):
/// - Forward/Back: left = right = distance
/// - Rotate in place: left = -right = θ * wheel_base / 2 (θ in radians)
///   - 90° = π/2 * 0.233 / 2 = 0.183m per wheel
///   - 180° = π * 0.233 / 2 = 0.366m per wheel
///   - 360° = 2π * 0.233 / 2 = 0.732m per wheel
/// - Arc: Different distances create curves
#[derive(Debug, Deserialize)]
pub struct WheelCommand {
    /// Left wheel distance in meters (positive = forward)
    pub left: f32,
    /// Right wheel distance in meters (positive = forward)
    pub right: f32,
}

fn default_2_5() -> f32 {
    2.5
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deserialize_scenario() {
        let yaml = r#"
name: "test_scenario"
map: "../maps/simple_room.yaml"
start_pose:
  x: 2.5
  y: 2.5
  theta: 0.0
path:
  - {left: 0.5, right: 0.5}
  - {left: -0.183, right: 0.183}
"#;

        let config: ScenarioConfig = serde_yaml::from_str(yaml).expect("Failed to parse YAML");
        assert_eq!(config.name, "test_scenario");
        assert_eq!(config.path.len(), 2);
        assert_eq!(config.path[0].left, 0.5);
        assert_eq!(config.path[0].right, 0.5);
    }

    #[test]
    fn test_default_pose() {
        let yaml = r#"
name: "minimal"
map: "test.yaml"
path:
  - {left: 1.0, right: 1.0}
"#;

        let config: ScenarioConfig = serde_yaml::from_str(yaml).expect("Failed to parse YAML");
        assert_eq!(config.start_pose.x, 2.5);
        assert_eq!(config.start_pose.y, 2.5);
        assert_eq!(config.start_pose.theta, 0.0);
    }
}
