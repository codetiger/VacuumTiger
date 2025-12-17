//! Dynamic odometry selection for runtime algorithm switching.
//!
//! Provides runtime-selectable odometry filters through the [`DynOdometry`] enum,
//! allowing applications to switch between odometry algorithms based on configuration.
//!
//! # Odometry Types
//!
//! - **Wheel**: Pure wheel encoder odometry (no sensor fusion)
//! - **Complementary**: Wheel encoders + gyroscope with complementary filter
//! - **ESKF**: Error-State Kalman Filter for full IMU fusion
//! - **Mahony**: Mahony AHRS filter with automatic gyro bias calibration
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::sensors::odometry::{DynOdometry, DynOdometryConfig, OdometryType};
//!
//! // Create odometry with runtime-selected type
//! let config = DynOdometryConfig::default();
//! let mut odom = DynOdometry::new(OdometryType::Complementary, config);
//!
//! // Process sensor data
//! if let Some(pose) = odom.update(left_ticks, right_ticks, gyro_z, timestamp_us) {
//!     println!("Pose: {:?}", pose);
//! }
//! ```
//!
//! Note: Some utility methods are defined for future use.

use clap::ValueEnum;
use serde::{Deserialize, Serialize};

use crate::core::types::{Pose2D, PoseTracker};
use crate::sensors::odometry::{
    ComplementaryConfig, ComplementaryFilter, Eskf, EskfConfig, MahonyAhrs, MahonyConfig,
    RawImuData, WheelOdometry, WheelOdometryConfig,
};

/// Available odometry algorithm types.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, ValueEnum, Serialize, Deserialize)]
pub enum OdometryType {
    /// Wheel odometry only (encoder ticks).
    ///
    /// Uses differential drive kinematics to compute pose from encoder ticks.
    /// No sensor fusion - heading comes entirely from wheel encoders.
    Wheel,

    /// Complementary filter (encoder + gyro fusion).
    ///
    /// Fuses wheel encoder translation with gyroscope-derived heading.
    /// Simple and effective for most indoor applications.
    Complementary,

    /// Error-State Kalman Filter (full IMU fusion).
    ///
    /// More sophisticated sensor fusion with proper uncertainty propagation.
    /// Better handling of noisy sensors but more computationally expensive.
    Eskf,

    /// Mahony AHRS filter (gyro + gravity fusion).
    ///
    /// Uses Mahony filter for yaw estimation with automatic gyro bias calibration.
    /// Best for applications requiring stable heading estimation.
    Mahony,
}

impl std::fmt::Display for OdometryType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OdometryType::Wheel => write!(f, "Wheel"),
            OdometryType::Complementary => write!(f, "Complementary"),
            OdometryType::Eskf => write!(f, "ESKF"),
            OdometryType::Mahony => write!(f, "Mahony"),
        }
    }
}

/// Configuration for all odometry types.
///
/// Each field configures its respective odometry algorithm.
/// Only the relevant config is used based on the selected [`OdometryType`].
#[derive(Debug, Clone, Default)]
pub struct DynOdometryConfig {
    /// Configuration for wheel odometry (used by all types).
    pub wheel: WheelOdometryConfig,

    /// Configuration for complementary filter.
    pub complementary: ComplementaryConfig,

    /// Configuration for ESKF.
    pub eskf: EskfConfig,

    /// Configuration for Mahony AHRS.
    pub mahony: MahonyConfig,
}

// ============================================================================
// Internal Wrapper Types
// ============================================================================

/// Wheel-only odometry wrapper.
#[derive(Debug)]
struct WheelOdometryWrapper {
    wheel: WheelOdometry,
    tracker: PoseTracker,
}

impl WheelOdometryWrapper {
    fn new(config: WheelOdometryConfig) -> Self {
        Self {
            wheel: WheelOdometry::new(config),
            tracker: PoseTracker::new(),
        }
    }

    fn update(&mut self, left: u16, right: u16) -> Option<Pose2D> {
        if let Some(delta) = self.wheel.update(left, right) {
            self.tracker.update(&delta);
            Some(self.tracker.pose())
        } else {
            None
        }
    }
}

/// Complementary filter odometry wrapper.
#[derive(Debug)]
struct ComplementaryOdometry {
    wheel: WheelOdometry,
    filter: ComplementaryFilter,
}

impl ComplementaryOdometry {
    fn new(wheel_config: WheelOdometryConfig, filter_config: ComplementaryConfig) -> Self {
        Self {
            wheel: WheelOdometry::new(wheel_config),
            filter: ComplementaryFilter::new(filter_config),
        }
    }

    fn update(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        if let Some(encoder_delta) = self.wheel.update(left, right) {
            let global_pose = self.filter.update(encoder_delta, gyro_yaw, timestamp_us);
            Some(global_pose)
        } else {
            None
        }
    }
}

/// ESKF odometry wrapper.
#[derive(Debug)]
struct EskfOdometry {
    wheel: WheelOdometry,
    eskf: Eskf,
}

impl EskfOdometry {
    fn new(wheel_config: WheelOdometryConfig, eskf_config: EskfConfig) -> Self {
        Self {
            wheel: WheelOdometry::new(wheel_config),
            eskf: Eskf::new(eskf_config),
        }
    }

    fn update(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        if let Some(encoder_delta) = self.wheel.update(left, right) {
            let global_pose = self.eskf.update(encoder_delta, gyro_yaw, timestamp_us);
            Some(global_pose)
        } else {
            None
        }
    }
}

/// Mahony AHRS odometry wrapper.
///
/// Uses Mahony filter for yaw estimation with automatic gyro bias calibration.
/// Combines wheel encoder translation with Mahony-derived heading.
#[derive(Debug)]
struct MahonyOdometry {
    wheel: WheelOdometry,
    ahrs: MahonyAhrs,
    tracker: PoseTracker,
    last_yaw: f32,
}

impl MahonyOdometry {
    fn new(wheel_config: WheelOdometryConfig, mahony_config: MahonyConfig) -> Self {
        Self {
            wheel: WheelOdometry::new(wheel_config),
            ahrs: MahonyAhrs::new(mahony_config),
            tracker: PoseTracker::new(),
            last_yaw: 0.0,
        }
    }

    fn update(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        // Update Mahony AHRS (uses gyro_yaw as Z-axis rotation)
        // For 2D odometry, we only need yaw from the AHRS
        // Pass zeros for X/Y axes and gravity (simplified 2D case)
        let imu = RawImuData::new(
            [0, 0, gyro_yaw], // gyro: [roll, pitch, yaw] - yaw on Z axis
            [0, 0, 1000],     // tilt: assume level (gravity pointing down)
        );
        let (_roll, _pitch, yaw) = self.ahrs.update(&imu, timestamp_us);

        // Get wheel odometry delta
        if let Some(encoder_delta) = self.wheel.update(left, right) {
            if self.ahrs.is_calibrated() {
                // Use Mahony yaw for heading
                let yaw_delta = yaw - self.last_yaw;
                self.last_yaw = yaw;

                // Create delta with encoder translation but Mahony rotation
                let delta = Pose2D::new(encoder_delta.x, encoder_delta.y, yaw_delta);
                self.tracker.update(&delta);
            } else {
                // During calibration, use encoder-only odometry
                self.tracker.update(&encoder_delta);
            }
            Some(self.tracker.pose())
        } else {
            None
        }
    }
}

// ============================================================================
// Dynamic Odometry Enum
// ============================================================================

/// Runtime-selectable odometry implementation.
///
/// Wraps all available odometry algorithms behind a unified interface,
/// enabling runtime algorithm selection based on configuration.
///
/// # Example
///
/// ```ignore
/// use dhruva_slam::sensors::odometry::{DynOdometry, DynOdometryConfig, OdometryType};
///
/// // Create with default config
/// let mut odom = DynOdometry::new(OdometryType::Complementary, DynOdometryConfig::default());
///
/// // Process updates
/// if let Some(pose) = odom.update(1000, 1000, 10, 0) {
///     println!("x={:.3}, y={:.3}, θ={:.2}°", pose.x, pose.y, pose.theta.to_degrees());
/// }
/// ```
#[derive(Debug)]
#[non_exhaustive]
#[allow(private_interfaces)]
pub enum DynOdometry {
    /// Wheel encoder odometry only.
    Wheel(Box<WheelOdometryWrapper>),
    /// Complementary filter (encoder + gyro).
    Complementary(Box<ComplementaryOdometry>),
    /// Error-State Kalman Filter.
    Eskf(Box<EskfOdometry>),
    /// Mahony AHRS filter.
    Mahony(Box<MahonyOdometry>),
}

impl DynOdometry {
    /// Create a new dynamic odometry instance.
    ///
    /// # Arguments
    ///
    /// * `odom_type` - The type of odometry algorithm to use
    /// * `config` - Configuration containing settings for all algorithm types
    pub fn new(odom_type: OdometryType, config: DynOdometryConfig) -> Self {
        match odom_type {
            OdometryType::Wheel => {
                DynOdometry::Wheel(Box::new(WheelOdometryWrapper::new(config.wheel)))
            }
            OdometryType::Complementary => DynOdometry::Complementary(Box::new(
                ComplementaryOdometry::new(config.wheel, config.complementary),
            )),
            OdometryType::Eskf => {
                DynOdometry::Eskf(Box::new(EskfOdometry::new(config.wheel, config.eskf)))
            }
            OdometryType::Mahony => {
                DynOdometry::Mahony(Box::new(MahonyOdometry::new(config.wheel, config.mahony)))
            }
        }
    }

    /// Process new sensor data and return updated pose.
    ///
    /// # Arguments
    ///
    /// * `left` - Left wheel encoder ticks (absolute)
    /// * `right` - Right wheel encoder ticks (absolute)
    /// * `gyro_yaw` - Raw gyroscope yaw reading
    /// * `timestamp_us` - Timestamp in microseconds
    ///
    /// # Returns
    ///
    /// `Some(Pose2D)` with updated global pose, or `None` on first call
    /// (initialization) or during calibration.
    pub fn update(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        match self {
            DynOdometry::Wheel(o) => o.update(left, right),
            DynOdometry::Complementary(o) => o.update(left, right, gyro_yaw, timestamp_us),
            DynOdometry::Eskf(o) => o.update(left, right, gyro_yaw, timestamp_us),
            DynOdometry::Mahony(o) => o.update(left, right, gyro_yaw, timestamp_us),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn default_config() -> DynOdometryConfig {
        DynOdometryConfig {
            wheel: WheelOdometryConfig {
                ticks_per_meter: 1000.0,
                wheel_base: 0.17,
            },
            ..Default::default()
        }
    }

    #[test]
    fn test_odometry_type_display() {
        assert_eq!(format!("{}", OdometryType::Wheel), "Wheel");
        assert_eq!(format!("{}", OdometryType::Complementary), "Complementary");
        assert_eq!(format!("{}", OdometryType::Eskf), "ESKF");
        assert_eq!(format!("{}", OdometryType::Mahony), "Mahony");
    }

    #[test]
    fn test_dyn_odometry_wheel() {
        let config = default_config();
        let mut odom = DynOdometry::new(OdometryType::Wheel, config);

        // First call initializes
        assert!(odom.update(0, 0, 0, 0).is_none());

        // Move forward 100 ticks on each wheel = 0.1m
        let pose = odom.update(100, 100, 0, 10_000).unwrap();
        assert_relative_eq!(pose.x, 0.1, epsilon = 0.01);
        assert_relative_eq!(pose.y, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_dyn_odometry_complementary() {
        let config = default_config();
        let mut odom = DynOdometry::new(OdometryType::Complementary, config);

        // First call initializes
        assert!(odom.update(0, 0, 0, 0).is_none());

        // Move forward
        let pose = odom.update(100, 100, 0, 10_000).unwrap();
        assert_relative_eq!(pose.x, 0.1, epsilon = 0.01);
    }

    #[test]
    fn test_dyn_odometry_eskf() {
        let config = default_config();
        let mut odom = DynOdometry::new(OdometryType::Eskf, config);

        // First call initializes
        assert!(odom.update(0, 0, 0, 0).is_none());

        // Move forward
        let pose = odom.update(100, 100, 0, 10_000).unwrap();
        assert_relative_eq!(pose.x, 0.1, epsilon = 0.01);
    }

    #[test]
    fn test_dyn_odometry_mahony() {
        let config = DynOdometryConfig {
            wheel: WheelOdometryConfig {
                ticks_per_meter: 1000.0,
                wheel_base: 0.17,
            },
            mahony: MahonyConfig {
                calibration_samples: 5, // Short calibration for test
                ..MahonyConfig::default()
            },
            ..Default::default()
        };
        let mut odom = DynOdometry::new(OdometryType::Mahony, config);

        // First call initializes
        odom.update(0, 0, 0, 0);

        // Complete calibration
        for i in 1..=10 {
            odom.update(0, 0, 0, i * 10_000);
        }
    }
}
