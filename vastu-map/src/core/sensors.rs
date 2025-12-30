//! Sensor input types for the occupancy grid.

use super::point::WorldPoint;
use super::pose::Pose2D;
use serde::{Deserialize, Serialize};

/// Lidar scan data in polar coordinates
#[derive(Clone, Debug, Default)]
pub struct LidarScan {
    /// Range measurements in meters (f32::INFINITY or > max_range = no return)
    pub ranges: Vec<f32>,
    /// Angle for each range in radians (0 = forward, CCW positive)
    pub angles: Vec<f32>,
    /// Minimum valid range (ignore closer measurements)
    pub range_min: f32,
    /// Maximum valid range (treat further as "no hit")
    pub range_max: f32,
}

impl LidarScan {
    /// Create a new lidar scan
    pub fn new(ranges: Vec<f32>, angles: Vec<f32>, range_min: f32, range_max: f32) -> Self {
        assert_eq!(
            ranges.len(),
            angles.len(),
            "ranges and angles must have same length"
        );
        Self {
            ranges,
            angles,
            range_min,
            range_max,
        }
    }

    /// Create from SangamIO PointCloud2D message format
    /// Input: Vec of (angle_rad, distance_m, quality)
    pub fn from_sangamio(points: &[(f32, f32, u8)]) -> Self {
        let mut ranges = Vec::with_capacity(points.len());
        let mut angles = Vec::with_capacity(points.len());

        for (angle, distance, _quality) in points {
            angles.push(*angle);
            ranges.push(*distance);
        }

        Self {
            ranges,
            angles,
            range_min: 0.15, // Delta-2D lidar spec
            range_max: 8.0,  // Delta-2D lidar spec
        }
    }

    /// Number of rays in the scan
    #[inline]
    pub fn len(&self) -> usize {
        self.ranges.len()
    }

    /// Is the scan empty?
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.ranges.is_empty()
    }

    /// Check if a range measurement is valid
    #[inline]
    pub fn is_valid_range(&self, range: f32) -> bool {
        range >= self.range_min && range <= self.range_max && range.is_finite()
    }

    /// Get valid points as (angle, range) pairs
    pub fn valid_points(&self) -> impl Iterator<Item = (f32, f32)> + '_ {
        self.angles
            .iter()
            .zip(self.ranges.iter())
            .filter(|&(_, r)| self.is_valid_range(*r))
            .map(|(&a, &r)| (a, r))
    }

    /// Convert to cartesian points in sensor frame
    pub fn to_cartesian(&self) -> Vec<WorldPoint> {
        self.valid_points()
            .map(|(angle, range)| WorldPoint::new(range * angle.cos(), range * angle.sin()))
            .collect()
    }

    /// Convert to cartesian points in world frame given sensor pose
    pub fn to_world_points(&self, sensor_pose: Pose2D) -> Vec<WorldPoint> {
        self.valid_points()
            .map(|(angle, range)| {
                let local = WorldPoint::new(range * angle.cos(), range * angle.sin());
                sensor_pose.transform_point(local)
            })
            .collect()
    }
}

/// Cliff sensor readings
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CliffSensors {
    /// Left side cliff sensor triggered
    pub left_side: bool,
    /// Left front cliff sensor triggered
    pub left_front: bool,
    /// Right front cliff sensor triggered
    pub right_front: bool,
    /// Right side cliff sensor triggered
    pub right_side: bool,
}

impl CliffSensors {
    /// All sensors safe (no cliffs)
    pub const SAFE: CliffSensors = CliffSensors {
        left_side: false,
        left_front: false,
        right_front: false,
        right_side: false,
    };

    /// Any cliff sensor triggered?
    #[inline]
    pub fn any_triggered(&self) -> bool {
        self.left_side || self.left_front || self.right_front || self.right_side
    }

    /// Count of triggered sensors
    #[inline]
    pub fn count_triggered(&self) -> u8 {
        self.left_side as u8
            + self.left_front as u8
            + self.right_front as u8
            + self.right_side as u8
    }

    /// Get positions of triggered sensors in robot frame
    /// Positions based on CRL-200S robot configuration from SangamIO
    pub fn triggered_positions(&self) -> Vec<WorldPoint> {
        let mut positions = Vec::with_capacity(4);

        // Sensor positions in robot frame (from SangamIO config)
        if self.left_side {
            positions.push(WorldPoint::new(0.12, 0.10));
        }
        if self.left_front {
            positions.push(WorldPoint::new(0.15, 0.05));
        }
        if self.right_front {
            positions.push(WorldPoint::new(0.15, -0.05));
        }
        if self.right_side {
            positions.push(WorldPoint::new(0.12, -0.10));
        }

        positions
    }

    /// Get positions of triggered sensors in world frame
    pub fn triggered_world_positions(&self, robot_pose: Pose2D) -> Vec<WorldPoint> {
        self.triggered_positions()
            .into_iter()
            .map(|p| robot_pose.transform_point(p))
            .collect()
    }
}

/// Bumper sensor readings
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BumperSensors {
    /// Left bumper triggered
    pub left: bool,
    /// Right bumper triggered
    pub right: bool,
}

impl BumperSensors {
    /// No bumpers triggered
    pub const CLEAR: BumperSensors = BumperSensors {
        left: false,
        right: false,
    };

    /// Any bumper triggered?
    #[inline]
    pub fn any_triggered(&self) -> bool {
        self.left || self.right
    }

    /// Both bumpers triggered?
    #[inline]
    pub fn both_triggered(&self) -> bool {
        self.left && self.right
    }

    /// Estimate collision angle in robot frame (radians)
    /// Returns None if no collision
    /// - 0.0 = center collision
    /// - positive = left side
    /// - negative = right side
    pub fn collision_angle(&self) -> Option<f32> {
        match (self.left, self.right) {
            (true, true) => Some(0.0),   // Center collision
            (true, false) => Some(0.5),  // Left collision (~30°)
            (false, true) => Some(-0.5), // Right collision (~-30°)
            (false, false) => None,      // No collision
        }
    }

    /// Estimate collision point in robot frame
    /// Uses robot radius to compute approximate impact location
    pub fn collision_point(&self, robot_radius: f32) -> Option<WorldPoint> {
        self.collision_angle()
            .map(|angle| WorldPoint::new(robot_radius * angle.cos(), robot_radius * angle.sin()))
    }

    /// Estimate collision point in world frame
    pub fn collision_world_point(
        &self,
        robot_pose: Pose2D,
        robot_radius: f32,
    ) -> Option<WorldPoint> {
        self.collision_point(robot_radius)
            .map(|p| robot_pose.transform_point(p))
    }
}

/// Combined sensor observation for a single timestep
#[derive(Clone, Debug)]
pub struct SensorObservation {
    /// Robot pose when observation was taken
    pub pose: Pose2D,
    /// Lidar scan (None if not available this tick)
    pub lidar: Option<LidarScan>,
    /// Cliff sensor states
    pub cliffs: CliffSensors,
    /// Bumper sensor states
    pub bumpers: BumperSensors,
    /// Timestamp in microseconds since epoch
    pub timestamp_us: u64,
}

impl SensorObservation {
    /// Create an observation with only pose
    pub fn pose_only(pose: Pose2D, timestamp_us: u64) -> Self {
        Self {
            pose,
            lidar: None,
            cliffs: CliffSensors::SAFE,
            bumpers: BumperSensors::CLEAR,
            timestamp_us,
        }
    }

    /// Create an observation with lidar
    pub fn with_lidar(pose: Pose2D, lidar: LidarScan, timestamp_us: u64) -> Self {
        Self {
            pose,
            lidar: Some(lidar),
            cliffs: CliffSensors::SAFE,
            bumpers: BumperSensors::CLEAR,
            timestamp_us,
        }
    }

    /// Any sensor triggered (cliff or bumper)?
    pub fn any_triggered(&self) -> bool {
        self.cliffs.any_triggered() || self.bumpers.any_triggered()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn test_lidar_scan_valid_points() {
        let scan = LidarScan::new(
            vec![1.0, 0.1, 5.0, 10.0, f32::INFINITY],
            vec![0.0, 0.1, 0.2, 0.3, 0.4],
            0.15,
            8.0,
        );

        let valid: Vec<_> = scan.valid_points().collect();
        assert_eq!(valid.len(), 2); // Only 1.0 and 5.0 are valid
    }

    #[test]
    fn test_lidar_to_cartesian() {
        let scan = LidarScan::new(
            vec![1.0],
            vec![0.0], // Forward
            0.15,
            8.0,
        );

        let points = scan.to_cartesian();
        assert_eq!(points.len(), 1);
        assert!((points[0].x - 1.0).abs() < 1e-6);
        assert!((points[0].y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_cliff_sensors_triggered() {
        let cliffs = CliffSensors {
            left_side: true,
            left_front: false,
            right_front: true,
            right_side: false,
        };

        assert!(cliffs.any_triggered());
        assert_eq!(cliffs.count_triggered(), 2);
        assert_eq!(cliffs.triggered_positions().len(), 2);
    }

    #[test]
    fn test_bumper_collision_angle() {
        assert_eq!(BumperSensors::CLEAR.collision_angle(), None);
        assert_eq!(
            BumperSensors {
                left: true,
                right: true
            }
            .collision_angle(),
            Some(0.0)
        );
        assert!(
            BumperSensors {
                left: true,
                right: false
            }
            .collision_angle()
            .unwrap()
                > 0.0
        );
        assert!(
            BumperSensors {
                left: false,
                right: true
            }
            .collision_angle()
            .unwrap()
                < 0.0
        );
    }

    #[test]
    fn test_lidar_to_world_points() {
        let scan = LidarScan::new(
            vec![1.0],
            vec![0.0], // Forward in sensor frame
            0.15,
            8.0,
        );

        // Robot at origin facing +Y (90 degrees)
        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let world_points = scan.to_world_points(pose);

        assert_eq!(world_points.len(), 1);
        // Forward in robot frame (which is +Y in world) should be at (0, 1)
        assert!((world_points[0].x - 0.0).abs() < 1e-5);
        assert!((world_points[0].y - 1.0).abs() < 1e-5);
    }
}
