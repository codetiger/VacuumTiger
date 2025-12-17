//! Clearance analysis and dock detection for safe robot operation.
//!
//! This module provides utilities for:
//! - Analyzing lidar scans to determine clearance in all directions
//! - Detecting if robot is in a dock (using charging state + lidar pattern)
//! - Calculating escape routes from confined spaces
//!
//! Note: Some types are defined for planned exploration features.

use crate::core::types::{LaserScan, Point2D, Pose2D};

/// Minimum clearance needed for 360° rotation (robot radius + safety margin).
/// Robot is ~18cm radius, so 25cm gives 7cm margin.
pub const MIN_ROTATION_CLEARANCE: f32 = 0.25;

/// Confidence level for dock detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DockConfidence {
    /// Definitely in dock (charging OR strong lidar pattern).
    Certain,
    /// Probably in dock (moderate lidar pattern).
    Likely,
    /// Probably not in dock.
    Unlikely,
}

/// Dock detection result combining charging state and lidar analysis.
#[derive(Debug, Clone)]
pub struct DockDetection {
    /// Robot is currently charging (definitive dock indicator).
    pub is_charging: bool,
    /// Lidar pattern matches dock (close on 3 sides, open front).
    pub lidar_pattern_matches: bool,
    /// Confidence level.
    pub confidence: DockConfidence,
}

/// Clearance analysis from lidar scan.
///
/// Analyzes the minimum distances in each direction sector to determine
/// if the robot has enough space to rotate safely.
#[derive(Debug, Clone)]
pub struct ClearanceAnalysis {
    /// Minimum distance in front sector (-30° to +30°).
    pub front_min: f32,
    /// Minimum distance in back sector (150° to -150°).
    pub back_min: f32,
    /// Minimum distance in left sector (60° to 120°).
    pub left_min: f32,
    /// Minimum distance in right sector (-60° to -120°).
    pub right_min: f32,
    /// Overall minimum distance in any direction.
    pub overall_min: f32,
    /// Direction with maximum clearance (radians, 0 = front).
    pub clearest_direction: f32,
    /// Distance in clearest direction.
    pub clearest_distance: f32,
}

impl ClearanceAnalysis {
    /// Create from lidar scan.
    ///
    /// Analyzes the scan to determine clearance in all directions
    /// and finds the clearest escape route.
    pub fn from_laser_scan(scan: &LaserScan) -> Self {
        let mut front_min = f32::MAX;
        let mut back_min = f32::MAX;
        let mut left_min = f32::MAX;
        let mut right_min = f32::MAX;
        let mut clearest_direction = 0.0f32;
        let mut clearest_distance = 0.0f32;

        for (angle, dist, _intensity) in scan.iter_valid() {
            // Skip invalid readings (already filtered by iter_valid, but extra safety)
            if !(0.02..=10.0).contains(&dist) {
                continue;
            }

            // Normalize angle to [-π, π] for consistent sector detection
            let normalized_angle = normalize_angle(angle);
            let angle_deg = normalized_angle.to_degrees();

            // Categorize by sector (angle is in radians, 0 = front)
            if angle_deg > -30.0 && angle_deg < 30.0 {
                front_min = front_min.min(dist);
            } else if !(-150.0..=150.0).contains(&angle_deg) {
                back_min = back_min.min(dist);
            } else if angle_deg > 60.0 && angle_deg < 120.0 {
                left_min = left_min.min(dist);
            } else if angle_deg > -120.0 && angle_deg < -60.0 {
                right_min = right_min.min(dist);
            }

            // Track clearest direction (for escape route)
            if dist > clearest_distance {
                clearest_distance = dist;
                clearest_direction = angle;
            }
        }

        // Handle case where no valid readings in a sector
        if front_min == f32::MAX {
            front_min = 0.0;
        }
        if back_min == f32::MAX {
            back_min = 0.0;
        }
        if left_min == f32::MAX {
            left_min = 0.0;
        }
        if right_min == f32::MAX {
            right_min = 0.0;
        }

        let overall_min = front_min.min(back_min).min(left_min).min(right_min);

        Self {
            front_min,
            back_min,
            left_min,
            right_min,
            overall_min,
            clearest_direction,
            clearest_distance,
        }
    }

    /// Check if robot has enough clearance to rotate in place.
    pub fn can_rotate_safely(&self) -> bool {
        self.overall_min >= MIN_ROTATION_CLEARANCE
    }

    /// Detect if robot is in dock using charging state + lidar pattern.
    ///
    /// Uses two indicators:
    /// - `is_charging`: 100% reliable hardware confirmation
    /// - Lidar U-pattern: back<15cm AND both sides<20cm AND front>40cm
    pub fn detect_dock(&self, is_charging: bool) -> DockDetection {
        // Check lidar U-pattern: close on back and sides, open in front
        let back_close = self.back_min < 0.15;
        let sides_close = self.left_min < 0.20 && self.right_min < 0.20;
        let front_open = self.front_min > 0.40;
        let lidar_pattern_matches = back_close && sides_close && front_open;

        let confidence = if is_charging {
            // Charging = definitely in dock, regardless of lidar
            DockConfidence::Certain
        } else if lidar_pattern_matches && back_close {
            // Strong lidar U-pattern without charging
            // (maybe just placed in dock, or battery full)
            DockConfidence::Certain
        } else if back_close && (self.left_min < 0.25 || self.right_min < 0.25) {
            // Partial pattern - might be docked or in corner
            DockConfidence::Likely
        } else {
            DockConfidence::Unlikely
        };

        DockDetection {
            is_charging,
            lidar_pattern_matches,
            confidence,
        }
    }

    /// Check if robot appears to be in dock.
    pub fn is_likely_in_dock(&self, is_charging: bool) -> bool {
        let detection = self.detect_dock(is_charging);
        matches!(
            detection.confidence,
            DockConfidence::Certain | DockConfidence::Likely
        )
    }

    /// Calculate escape target position to get clearance for rotation.
    ///
    /// Returns a point in the clearest direction, far enough to provide
    /// rotation clearance.
    pub fn calculate_escape_target(
        &self,
        pose: &Pose2D,
        min_rotation_clearance: f32,
        min_escape_distance: f32,
    ) -> Point2D {
        // Calculate how far to move to get rotation clearance
        let needed_distance = (min_rotation_clearance - self.overall_min + 0.10)
            .max(min_escape_distance) // At least minimum escape distance
            .min(1.0); // Cap at 1m to avoid overshooting

        // Prefer forward if it's reasonably clear, otherwise use clearest direction
        let escape_direction = if self.front_min > min_rotation_clearance + 0.10 {
            pose.theta // Forward is clear enough
        } else {
            // Use clearest direction (absolute angle from lidar)
            normalize_angle(pose.theta + self.clearest_direction)
        };

        Point2D::new(
            pose.x + needed_distance * escape_direction.cos(),
            pose.y + needed_distance * escape_direction.sin(),
        )
    }
}

/// Normalize angle to [-π, π].
fn normalize_angle(angle: f32) -> f32 {
    let mut result = angle;
    while result > std::f32::consts::PI {
        result -= std::f32::consts::TAU;
    }
    while result < -std::f32::consts::PI {
        result += std::f32::consts::TAU;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_scan(ranges: Vec<f32>) -> LaserScan {
        let n = ranges.len();
        let angle_increment = std::f32::consts::TAU / n as f32;
        LaserScan::new(
            0.0,
            std::f32::consts::TAU - angle_increment,
            angle_increment,
            0.1,
            10.0,
            ranges,
        )
    }

    #[test]
    fn test_clearance_analysis_open_space() {
        // All directions have 2m clearance
        let ranges = vec![2.0; 360];
        let scan = create_test_scan(ranges);
        let analysis = ClearanceAnalysis::from_laser_scan(&scan);

        assert!(analysis.can_rotate_safely());
        assert!(analysis.overall_min > 1.5);
    }

    #[test]
    fn test_clearance_analysis_confined() {
        // All directions have 10cm clearance
        let ranges = vec![0.10; 360];
        let scan = create_test_scan(ranges);
        let analysis = ClearanceAnalysis::from_laser_scan(&scan);

        assert!(!analysis.can_rotate_safely());
        assert!(analysis.overall_min < MIN_ROTATION_CLEARANCE);
    }

    #[test]
    fn test_dock_detection_charging() {
        let ranges = vec![0.5; 360];
        let scan = create_test_scan(ranges);
        let analysis = ClearanceAnalysis::from_laser_scan(&scan);

        let detection = analysis.detect_dock(true);
        assert!(detection.is_charging);
        assert_eq!(detection.confidence, DockConfidence::Certain);
    }

    #[test]
    fn test_dock_detection_not_charging_open() {
        let ranges = vec![2.0; 360];
        let scan = create_test_scan(ranges);
        let analysis = ClearanceAnalysis::from_laser_scan(&scan);

        let detection = analysis.detect_dock(false);
        assert!(!detection.is_charging);
        assert_eq!(detection.confidence, DockConfidence::Unlikely);
    }

    #[test]
    fn test_escape_target_calculation() {
        let ranges = vec![2.0; 360];
        let scan = create_test_scan(ranges);
        let analysis = ClearanceAnalysis::from_laser_scan(&scan);

        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let target = analysis.calculate_escape_target(&pose, 0.25, 0.15);

        // Should escape forward (clearest direction)
        assert!(target.x > 0.0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(std::f32::consts::PI) - std::f32::consts::PI).abs() < 0.001);
        assert!(
            (normalize_angle(std::f32::consts::TAU) - 0.0).abs() < 0.001
                || (normalize_angle(std::f32::consts::TAU) + std::f32::consts::TAU).abs() < 0.001
        );
        assert!(
            (normalize_angle(-std::f32::consts::TAU) - 0.0).abs() < 0.001
                || (normalize_angle(-std::f32::consts::TAU) + std::f32::consts::TAU).abs() < 0.001
        );
    }
}
