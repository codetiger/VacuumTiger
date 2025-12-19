//! Quantitative validation metrics
//!
//! Computes error metrics comparing SLAM output against ground truth:
//! - Position and orientation error
//! - Wall accuracy (how close detected lines are to actual walls in map)

use sangam_io::devices::mock::map_loader::SimulationMap;
use vastu_map::VectorMap;
use vastu_map::core::Pose2D;
use vastu_map::features::Line2D;

/// Test metrics results.
#[derive(Debug, Clone)]
pub struct TestMetrics {
    /// Position error in meters (Euclidean distance)
    pub position_error: f32,
    /// Orientation error in radians
    pub orientation_error: f32,
    /// Average distance from detected lines to actual walls in meters
    pub wall_distance: f32,
    /// Percentage of detected line points that are near actual walls (0.0-1.0)
    pub wall_accuracy: f32,
    /// Number of detected walls/features
    pub detected_walls: usize,
}

impl TestMetrics {
    /// Compute all metrics comparing SLAM output to ground truth map.
    pub fn compute(
        slam: &VectorMap,
        map: &SimulationMap,
        final_pose: Pose2D,
        truth_pose: Pose2D,
    ) -> Self {
        // Position error
        let position_error =
            ((final_pose.x - truth_pose.x).powi(2) + (final_pose.y - truth_pose.y).powi(2)).sqrt();

        // Orientation error (normalized to [-PI, PI])
        let orientation_error = angle_diff(final_pose.theta, truth_pose.theta).abs();

        // Get detected lines
        let detected_lines = slam.lines();
        let detected_walls = detected_lines.len();

        // Compute wall accuracy using the simulation map
        let (wall_distance, wall_accuracy) = compute_wall_accuracy(&detected_lines, map);

        Self {
            position_error,
            orientation_error,
            wall_distance,
            wall_accuracy,
            detected_walls,
        }
    }

    /// Check if metrics pass acceptance criteria.
    pub fn passes(&self, criteria: &AcceptanceCriteria) -> bool {
        self.position_error <= criteria.max_position_error
            && self.orientation_error <= criteria.max_orientation_error
            && self.wall_accuracy >= criteria.min_wall_accuracy
    }

    /// Format metrics as a human-readable string.
    pub fn summary(&self) -> String {
        format!(
            "Position error: {:.3}m, Orientation error: {:.1}°, \
             Wall distance: {:.3}m, Wall accuracy: {:.0}%, Lines: {}",
            self.position_error,
            self.orientation_error.to_degrees(),
            self.wall_distance,
            self.wall_accuracy * 100.0,
            self.detected_walls
        )
    }
}

/// Acceptance criteria for test pass/fail determination.
#[derive(Clone, Debug)]
pub struct AcceptanceCriteria {
    /// Maximum position error in meters
    pub max_position_error: f32,
    /// Maximum orientation error in radians
    pub max_orientation_error: f32,
    /// Minimum wall accuracy (0.0-1.0)
    pub min_wall_accuracy: f32,
}

impl Default for AcceptanceCriteria {
    fn default() -> Self {
        Self {
            max_position_error: 0.1,      // 10cm
            max_orientation_error: 0.087, // ~5 degrees
            min_wall_accuracy: 0.7,       // 70% of line points near walls
        }
    }
}

impl AcceptanceCriteria {
    /// Strict criteria for high-precision tests.
    pub fn strict() -> Self {
        Self {
            max_position_error: 0.05,     // 5cm
            max_orientation_error: 0.035, // ~2 degrees
            min_wall_accuracy: 0.85,      // 85%
        }
    }

    /// Lenient criteria for noisy/challenging tests.
    pub fn lenient() -> Self {
        Self {
            max_position_error: 0.2,      // 20cm
            max_orientation_error: 0.175, // ~10 degrees
            min_wall_accuracy: 0.5,       // 50%
        }
    }
}

/// Compute angle difference normalized to [-PI, PI].
fn angle_diff(a: f32, b: f32) -> f32 {
    let diff = a - b;
    if diff > std::f32::consts::PI {
        diff - 2.0 * std::f32::consts::PI
    } else if diff < -std::f32::consts::PI {
        diff + 2.0 * std::f32::consts::PI
    } else {
        diff
    }
}

/// Maximum distance (meters) from line point to wall to count as valid.
const WALL_DISTANCE_THRESHOLD: f32 = 0.15; // 15cm

/// Number of sample points along each line.
const SAMPLES_PER_LINE: usize = 20;

/// Compute wall accuracy by checking if detected lines lie on occupied pixels.
///
/// Returns (average_distance, accuracy) where:
/// - average_distance: mean distance from line sample points to nearest wall
/// - accuracy: fraction of sample points that are within threshold of a wall
fn compute_wall_accuracy(detected: &[Line2D], map: &SimulationMap) -> (f32, f32) {
    if detected.is_empty() {
        return (0.0, 0.0);
    }

    let mut total_distance = 0.0;
    let mut near_wall_count = 0;
    let mut total_samples = 0;

    let resolution = map.resolution();

    for line in detected {
        // Sample points along the line
        for i in 0..SAMPLES_PER_LINE {
            let t = i as f32 / (SAMPLES_PER_LINE - 1) as f32;
            let x = line.start.x + t * (line.end.x - line.start.x);
            let y = line.start.y + t * (line.end.y - line.start.y);

            // Find distance to nearest occupied cell
            let dist = distance_to_nearest_wall(x, y, map, resolution);
            total_distance += dist;
            total_samples += 1;

            if dist <= WALL_DISTANCE_THRESHOLD {
                near_wall_count += 1;
            }
        }
    }

    let avg_distance = total_distance / total_samples as f32;
    let accuracy = near_wall_count as f32 / total_samples as f32;

    (avg_distance, accuracy)
}

/// Find distance from point to nearest occupied cell in map.
///
/// Searches in a spiral pattern outward from the point.
fn distance_to_nearest_wall(x: f32, y: f32, map: &SimulationMap, resolution: f32) -> f32 {
    // Check if point itself is on a wall
    if map.is_occupied(x, y) {
        return 0.0;
    }

    // Search outward in a grid pattern
    let max_search = (WALL_DISTANCE_THRESHOLD * 2.0 / resolution) as i32;

    for radius in 1..=max_search {
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                // Only check cells at this radius (perimeter)
                if dx.abs() != radius && dy.abs() != radius {
                    continue;
                }

                let check_x = x + dx as f32 * resolution;
                let check_y = y + dy as f32 * resolution;

                if map.is_occupied(check_x, check_y) {
                    let dist = ((check_x - x).powi(2) + (check_y - y).powi(2)).sqrt();
                    return dist;
                }
            }
        }
    }

    // No wall found within search radius
    WALL_DISTANCE_THRESHOLD * 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_diff() {
        assert!((angle_diff(0.0, 0.0)).abs() < 0.001);
        assert!((angle_diff(0.1, 0.0) - 0.1).abs() < 0.001);
        assert!((angle_diff(std::f32::consts::PI, -std::f32::consts::PI)).abs() < 0.001);
    }

    #[test]
    fn test_acceptance_criteria() {
        let metrics = TestMetrics {
            position_error: 0.05,
            orientation_error: 0.05,
            wall_distance: 0.05,
            wall_accuracy: 0.9,
            detected_walls: 4,
        };

        assert!(metrics.passes(&AcceptanceCriteria::default()));
        assert!(metrics.passes(&AcceptanceCriteria::lenient()));
    }
}
