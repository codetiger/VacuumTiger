//! Sensor input types for the occupancy grid.
//!
//! This module defines the sensor data structures used by VastuSLAM for
//! multi-sensor occupancy grid mapping.
//!
//! ## Sensor Types
//!
//! | Sensor | Rate | Purpose |
//! |--------|------|---------|
//! | [`LidarScan`] | ~5 Hz | 360° obstacle detection |
//! | [`CliffSensors`] | ~110 Hz | Floor drop-off detection |
//! | [`BumperSensors`] | Event | Collision detection |
//!
//! ## CRL-200S Robot Sensor Layout
//!
//! ```text
//!              Front (0°)
//!          ┌───────────────┐
//!          │  LF       RF  │  Cliff sensors (IR)
//!          │ ╲    ↑    ╱   │
//!          │  ╲   │   ╱    │
//!          │LS ╲  │  ╱ RS  │  Cliff sensors (side)
//!          │    ╲ │ ╱      │
//!          │  L ──┴── R    │  Bumpers (mechanical)
//!          │     LIDAR     │  360° lidar (center)
//!          └───────────────┘
//!               Back
//!
//! LF/RF = Left/Right Front cliff (15cm fwd, ±5cm lateral)
//! LS/RS = Left/Right Side cliff (12cm fwd, ±10cm lateral)
//! L/R   = Left/Right bumper (front arc coverage)
//! ```
//!
//! ## Sensor Coordinate Frame
//!
//! All sensor positions are in the robot frame:
//! - Origin: Robot center
//! - +X: Forward
//! - +Y: Left

use super::point::WorldPoint;
use super::pose::Pose2D;
use serde::{Deserialize, Serialize};

/// Lidar scan data in polar coordinates.
///
/// Represents a complete 360° lidar scan as a collection of range measurements
/// at known angles. Invalid measurements (too close, too far, or no return)
/// are filtered using [`is_valid_range`](Self::is_valid_range).
///
/// # Angular Convention
///
/// ```text
///          0° (+X, Forward)
///           ↑
///           │
///  90° (+Y) ├───────→ -90° (-Y)
///   Left    │          Right
///           ↓
///        ±180° (-X, Back)
/// ```
///
/// Angles are in radians, counter-clockwise from +X axis.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_slam::core::{LidarScan, Pose2D};
///
/// // Create scan from raw measurements
/// let scan = LidarScan::new(ranges, angles, 0.15, 8.0);
///
/// // Filter valid points and transform to world frame
/// let robot_pose = Pose2D::new(x, y, theta);
/// let world_points = scan.to_world_points(robot_pose);
/// ```
#[derive(Clone, Debug, Default)]
pub struct LidarScan {
    /// Range measurements in meters.
    /// Values > max_range or f32::INFINITY indicate no return (ray passed through).
    pub ranges: Vec<f32>,
    /// Angle for each range in radians (0 = forward, counter-clockwise positive).
    pub angles: Vec<f32>,
    /// Minimum valid range in meters (reject closer measurements, typically 0.15m).
    pub range_min: f32,
    /// Maximum valid range in meters (treat farther as "no hit", typically 8.0m).
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

/// Cliff sensor readings from IR floor-detection sensors.
///
/// Cliff sensors detect floor drop-offs (stairs, ledges) by measuring
/// the distance to the floor surface. When a cliff is detected, the
/// sensor reports `true`.
///
/// # Sensor Positions (CRL-200S)
///
/// ```text
///        ┌─────────┐
///     LF │    ↑    │ RF   (Front: ±5cm, 15cm forward)
///        │    │    │
///     LS │    │    │ RS   (Side: ±10cm, 12cm forward)
///        └─────────┘
/// ```
///
/// # Usage
///
/// ```rust,ignore
/// use vastu_slam::core::{CliffSensors, Pose2D};
///
/// let cliffs = CliffSensors {
///     left_front: true,
///     right_front: false,
///     left_side: false,
///     right_side: false,
/// };
///
/// if cliffs.any_triggered() {
///     // Get world positions of triggered sensors
///     let robot_pose = Pose2D::new(x, y, theta);
///     let cliff_points = cliffs.triggered_world_positions(robot_pose);
///     // Mark these positions as Cliff in the grid
/// }
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CliffSensors {
    /// Left side cliff sensor triggered (12cm fwd, 10cm left)
    pub left_side: bool,
    /// Left front cliff sensor triggered (15cm fwd, 5cm left)
    pub left_front: bool,
    /// Right front cliff sensor triggered (15cm fwd, 5cm right)
    pub right_front: bool,
    /// Right side cliff sensor triggered (12cm fwd, 10cm right)
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

/// Bumper sensor readings from mechanical collision sensors.
///
/// Bumpers detect physical collisions with obstacles. This is crucial for
/// detecting "invisible" obstacles that lidar cannot see, such as:
/// - Glass doors and windows
/// - Mirrors
/// - Thin table/chair legs
/// - Dark-colored objects that absorb lidar
///
/// # Collision Angle Estimation
///
/// ```text
///          ↑ Forward
///          │
///    L ────┼──── R
///          │
///
/// Both = center collision (0°)
/// Left only = left collision (~30° left)
/// Right only = right collision (~30° right)
/// ```
///
/// # Priority
///
/// Bumper collisions have the highest priority in the cell type hierarchy.
/// Once a cell is marked as `Bump`, it cannot be downgraded by lidar or
/// cliff observations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BumperSensors {
    /// Left bumper triggered (covers left front arc)
    pub left: bool,
    /// Right bumper triggered (covers right front arc)
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

/// IMU measurement from inertial measurement unit (6-axis).
///
/// All values are in SI units (m/s², rad/s). The coordinate frame follows
/// ROS REP-103: X forward, Y left, Z up.
///
/// # Conversion from SangamIO raw i16 values
///
/// SangamIO streams raw i16 values that need conversion:
/// - gyro: `raw_i16 / 1000.0` → rad/s
/// - accel: `raw_i16 / 16384.0 * 9.81` → m/s²
///
/// Use [`ImuMeasurement::from_raw`] for automatic conversion.
///
/// # Usage for 2D SLAM
///
/// For 2D ground robots, primarily use:
/// - `gyro_z`: Yaw rate (rotation around vertical axis)
/// - `accel_x`, `accel_y`: For gravity estimation (tilt detection)
///
/// # Example
///
/// ```rust,ignore
/// use vastu_slam::core::ImuMeasurement;
///
/// // From SangamIO raw values
/// let imu = ImuMeasurement::from_raw(
///     timestamp_us,
///     [gyro_x_raw, gyro_y_raw, gyro_z_raw],
///     [accel_x_raw, accel_y_raw, accel_z_raw],
/// );
///
/// // Yaw rate for rotation estimation
/// let yaw_rate = imu.gyro_z;  // rad/s, CCW positive
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuMeasurement {
    /// Timestamp in microseconds since epoch
    pub timestamp_us: u64,
    /// Linear acceleration X in m/s² (forward)
    pub accel_x: f32,
    /// Linear acceleration Y in m/s² (left)
    pub accel_y: f32,
    /// Linear acceleration Z in m/s² (up, includes gravity: ~+9.81 when level)
    pub accel_z: f32,
    /// Angular velocity X in rad/s (roll rate, rotation around forward axis)
    pub gyro_x: f32,
    /// Angular velocity Y in rad/s (pitch rate, rotation around left axis)
    pub gyro_y: f32,
    /// Angular velocity Z in rad/s (yaw rate, rotation around up axis, CCW positive)
    /// This is the PRIMARY value for 2D SLAM rotation estimation.
    pub gyro_z: f32,
}

/// Scale factor for converting raw gyroscope i16 to rad/s
const GYRO_SCALE: f32 = 1000.0;

/// Scale factor for converting raw accelerometer i16 to g units
const ACCEL_SCALE: f32 = 16384.0;

/// Standard gravity in m/s²
const G: f32 = 9.81;

impl ImuMeasurement {
    /// Create a new IMU measurement with SI units
    pub fn new(timestamp_us: u64, accel: [f32; 3], gyro: [f32; 3]) -> Self {
        Self {
            timestamp_us,
            accel_x: accel[0],
            accel_y: accel[1],
            accel_z: accel[2],
            gyro_x: gyro[0],
            gyro_y: gyro[1],
            gyro_z: gyro[2],
        }
    }

    /// Create from SangamIO raw i16 values
    ///
    /// Converts raw sensor values to SI units:
    /// - gyro: raw / 1000.0 → rad/s
    /// - accel: raw / 16384.0 * 9.81 → m/s²
    pub fn from_raw(timestamp_us: u64, gyro: [i16; 3], accel: [i16; 3]) -> Self {
        Self {
            timestamp_us,
            gyro_x: gyro[0] as f32 / GYRO_SCALE,
            gyro_y: gyro[1] as f32 / GYRO_SCALE,
            gyro_z: gyro[2] as f32 / GYRO_SCALE,
            accel_x: accel[0] as f32 / ACCEL_SCALE * G,
            accel_y: accel[1] as f32 / ACCEL_SCALE * G,
            accel_z: accel[2] as f32 / ACCEL_SCALE * G,
        }
    }

    /// Get acceleration as array [x, y, z] in m/s²
    #[inline]
    pub fn accel(&self) -> [f32; 3] {
        [self.accel_x, self.accel_y, self.accel_z]
    }

    /// Get angular velocity as array [x, y, z] in rad/s
    #[inline]
    pub fn gyro(&self) -> [f32; 3] {
        [self.gyro_x, self.gyro_y, self.gyro_z]
    }

    /// Get acceleration magnitude in m/s²
    #[inline]
    pub fn accel_magnitude(&self) -> f32 {
        (self.accel_x * self.accel_x + self.accel_y * self.accel_y + self.accel_z * self.accel_z)
            .sqrt()
    }

    /// Get angular velocity magnitude in rad/s
    #[inline]
    pub fn gyro_magnitude(&self) -> f32 {
        (self.gyro_x * self.gyro_x + self.gyro_y * self.gyro_y + self.gyro_z * self.gyro_z).sqrt()
    }
}

/// Combined sensor observation for a single timestep.
///
/// Bundles all sensor readings taken at approximately the same time,
/// along with the robot's pose at that moment. This is the primary
/// input type for [`crate::OccupancyGridMap::observe`].
///
/// # Sensor Availability
///
/// Not all sensors report at the same rate:
/// - **Lidar**: ~5 Hz (every ~200ms) - may be `None` on most ticks
/// - **IMU**: ~110 Hz - optional, for motion filtering
/// - **Cliff/Bumper**: ~110 Hz - always available
///
/// # Processing Order
///
/// When processed by the map, sensors are handled by priority:
/// 1. Lidar (lowest priority) - marks Floor and Wall
/// 2. Cliff - marks Cliff (overrides Floor/Wall)
/// 3. Bumper (highest priority) - marks Bump (overrides everything)
#[derive(Clone, Debug)]
pub struct SensorObservation {
    /// Robot pose when observation was taken (in world frame)
    pub pose: Pose2D,
    /// Lidar scan (None if lidar didn't report this tick, ~5 Hz)
    pub lidar: Option<LidarScan>,
    /// Cliff sensor states (always available, ~110 Hz)
    pub cliffs: CliffSensors,
    /// Bumper sensor states (event-driven, only true on collision)
    pub bumpers: BumperSensors,
    /// IMU measurement (optional, ~110 Hz, for motion filtering)
    pub imu: Option<ImuMeasurement>,
    /// Timestamp in microseconds since epoch (for temporal ordering)
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
            imu: None,
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
            imu: None,
            timestamp_us,
        }
    }

    /// Create an observation with lidar and IMU
    pub fn with_lidar_and_imu(
        pose: Pose2D,
        lidar: LidarScan,
        imu: ImuMeasurement,
        timestamp_us: u64,
    ) -> Self {
        Self {
            pose,
            lidar: Some(lidar),
            cliffs: CliffSensors::SAFE,
            bumpers: BumperSensors::CLEAR,
            imu: Some(imu),
            timestamp_us,
        }
    }

    /// Create an observation with IMU only (no lidar)
    pub fn with_imu(pose: Pose2D, imu: ImuMeasurement, timestamp_us: u64) -> Self {
        Self {
            pose,
            lidar: None,
            cliffs: CliffSensors::SAFE,
            bumpers: BumperSensors::CLEAR,
            imu: Some(imu),
            timestamp_us,
        }
    }

    /// Any sensor triggered (cliff or bumper)?
    pub fn any_triggered(&self) -> bool {
        self.cliffs.any_triggered() || self.bumpers.any_triggered()
    }

    /// Check if IMU data is available
    pub fn has_imu(&self) -> bool {
        self.imu.is_some()
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

    #[test]
    fn test_imu_measurement_from_raw() {
        // Test conversion from raw i16 values
        let imu = ImuMeasurement::from_raw(
            1000000,         // 1 second timestamp
            [1000, 0, -500], // gyro raw: 1 rad/s X, 0 Y, -0.5 rad/s Z
            [0, 0, 16384],   // accel raw: 0, 0, 1g (level)
        );

        // Check gyro conversion: raw / 1000.0 = rad/s
        assert!((imu.gyro_x - 1.0).abs() < 1e-6);
        assert!((imu.gyro_y - 0.0).abs() < 1e-6);
        assert!((imu.gyro_z - (-0.5)).abs() < 1e-6);

        // Check accel conversion: raw / 16384.0 * 9.81 = m/s²
        assert!((imu.accel_x - 0.0).abs() < 1e-6);
        assert!((imu.accel_y - 0.0).abs() < 1e-6);
        assert!((imu.accel_z - 9.81).abs() < 0.01);
    }

    #[test]
    fn test_imu_measurement_new() {
        let imu = ImuMeasurement::new(
            500000,
            [0.0, 0.0, 9.81], // accel
            [0.1, 0.0, -0.2], // gyro
        );

        assert_eq!(imu.timestamp_us, 500000);
        assert_eq!(imu.accel(), [0.0, 0.0, 9.81]);
        assert_eq!(imu.gyro(), [0.1, 0.0, -0.2]);
    }

    #[test]
    fn test_imu_measurement_magnitudes() {
        let imu = ImuMeasurement::new(
            0,
            [3.0, 4.0, 0.0], // accel: magnitude = 5.0
            [0.0, 0.0, 1.0], // gyro: magnitude = 1.0
        );

        assert!((imu.accel_magnitude() - 5.0).abs() < 1e-6);
        assert!((imu.gyro_magnitude() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_sensor_observation_with_imu() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let imu = ImuMeasurement::default();

        let obs = SensorObservation::with_imu(pose, imu, 123456);

        assert!(obs.has_imu());
        assert!(obs.lidar.is_none());
        assert_eq!(obs.timestamp_us, 123456);
    }
}
