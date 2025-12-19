//! YAML scenario runner
//!
//! Loads and executes test scenarios defined in YAML files.

use crate::harness::{HarnessConfig, TestHarness, TestResult};
use crate::path_generator::PathSegment;
use crate::yaml_config::{ScenarioConfig, WheelCommand};
use std::path::Path;

/// Wheel base of the robot in meters (from RobotConfig default).
const WHEEL_BASE: f32 = 0.233;

/// Default wheel speed for simulation in m/s.
const DEFAULT_SPEED: f32 = 0.2;

/// Load a scenario configuration from a YAML file.
pub fn load_scenario<P: AsRef<Path>>(
    path: P,
) -> Result<ScenarioConfig, Box<dyn std::error::Error>> {
    let content = std::fs::read_to_string(path)?;
    Ok(serde_yaml::from_str(&content)?)
}

/// Run a scenario from a YAML file.
///
/// Returns the test result or an error if the scenario failed to load/run.
pub fn run_scenario<P: AsRef<Path>>(
    yaml_path: P,
) -> Result<TestResult, Box<dyn std::error::Error>> {
    let yaml_path = yaml_path.as_ref();
    let config = load_scenario(yaml_path)?;

    // Resolve map path relative to YAML file
    let yaml_dir = yaml_path.parent().unwrap_or(Path::new("."));
    let map_file = yaml_dir
        .join(&config.map)
        .canonicalize()
        .map(|p| p.to_string_lossy().to_string())
        .unwrap_or_else(|_| yaml_dir.join(&config.map).to_string_lossy().to_string());

    let harness_config = HarnessConfig {
        map_file,
        start_x: config.start_pose.x,
        start_y: config.start_pose.y,
        start_theta: config.start_pose.theta,
        ..Default::default()
    };

    let path = convert_wheel_commands(&config.path);

    // Ensure output directory exists
    std::fs::create_dir_all("results").ok();

    let svg_path = format!("results/{}.svg", config.name);
    let mut harness = TestHarness::new(harness_config)?.with_visualization(&svg_path);

    Ok(harness.run_path(&path))
}

/// Convert wheel distance commands to PathSegments.
///
/// Uses differential drive kinematics:
/// - linear_dist = (left + right) / 2
/// - angular_dist = (right - left) / wheel_base
fn convert_wheel_commands(commands: &[WheelCommand]) -> Vec<PathSegment> {
    commands
        .iter()
        .filter_map(|cmd| {
            // Calculate linear and angular distances
            let linear_dist = (cmd.left + cmd.right) / 2.0;
            let angular_dist = (cmd.right - cmd.left) / WHEEL_BASE; // radians

            // Calculate duration based on max wheel distance
            let max_wheel_dist = cmd.left.abs().max(cmd.right.abs());
            if max_wheel_dist < 0.001 {
                return None; // Skip zero-distance commands
            }

            let duration = max_wheel_dist / DEFAULT_SPEED;

            // Calculate velocities
            let linear_vel = linear_dist / duration;
            let angular_vel = angular_dist / duration;

            Some(PathSegment {
                linear_vel,
                angular_vel,
                duration,
            })
        })
        .collect()
}

/// Run all YAML scenarios in a directory.
///
/// Returns a vector of (scenario_name, result) tuples.
pub fn run_all_scenarios<P: AsRef<Path>>(dir: P) -> Vec<(String, Result<TestResult, String>)> {
    let mut results = Vec::new();

    let entries = match std::fs::read_dir(&dir) {
        Ok(entries) => entries,
        Err(e) => {
            log::warn!("Failed to read scenario directory: {}", e);
            return results;
        }
    };

    // Collect and sort entries for consistent ordering
    let mut yaml_files: Vec<_> = entries
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().map_or(false, |e| e == "yaml" || e == "yml"))
        .filter(|p| !p.to_string_lossy().contains("/maps/")) // Skip map yaml files
        .collect();

    yaml_files.sort();

    for path in yaml_files {
        let name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();

        println!("\n--- Running scenario: {} ---", name);
        let result = run_scenario(&path).map_err(|e| e.to_string());

        match &result {
            Ok(r) => println!("  {}", r.metrics.summary()),
            Err(e) => println!("  FAILED: {}", e),
        }

        results.push((name, result));
    }

    results
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wheel_command_forward() {
        let commands = vec![WheelCommand {
            left: 1.0,
            right: 1.0,
        }];
        let segments = convert_wheel_commands(&commands);

        assert_eq!(segments.len(), 1);
        let seg = &segments[0];

        // Forward: linear = 1.0, angular = 0
        assert!((seg.linear_vel - DEFAULT_SPEED).abs() < 0.01);
        assert!(seg.angular_vel.abs() < 0.01);
        assert!((seg.duration - 5.0).abs() < 0.01); // 1.0m / 0.2m/s = 5s
    }

    #[test]
    fn test_wheel_command_rotate() {
        // 90 degree turn: left = -0.183, right = 0.183
        let turn_dist = std::f32::consts::FRAC_PI_2 * WHEEL_BASE / 2.0;
        let commands = vec![WheelCommand {
            left: -turn_dist,
            right: turn_dist,
        }];
        let segments = convert_wheel_commands(&commands);

        assert_eq!(segments.len(), 1);
        let seg = &segments[0];

        // Rotation in place: linear = 0
        assert!(seg.linear_vel.abs() < 0.01);
        // Angular should be positive (CCW)
        assert!(seg.angular_vel > 0.0);
    }

    #[test]
    fn test_wheel_command_arc() {
        // Arc motion: different wheel distances
        let commands = vec![WheelCommand {
            left: 0.5,
            right: 0.7,
        }];
        let segments = convert_wheel_commands(&commands);

        assert_eq!(segments.len(), 1);
        let seg = &segments[0];

        // Both linear and angular should be non-zero
        assert!(seg.linear_vel > 0.0);
        assert!(seg.angular_vel > 0.0); // Turning left (outer wheel faster)
    }

    #[test]
    fn test_skip_zero_commands() {
        let commands = vec![
            WheelCommand {
                left: 0.0,
                right: 0.0,
            },
            WheelCommand {
                left: 1.0,
                right: 1.0,
            },
        ];
        let segments = convert_wheel_commands(&commands);

        // Zero command should be skipped
        assert_eq!(segments.len(), 1);
    }
}
