//! Odometry estimation module (Phase 2).
//!
//! Provides wheel odometry and sensor fusion for dead-reckoning.
//!
//! # Components
//!
//! - [`WheelOdometry`]: Convert encoder ticks to pose deltas
//! - [`ComplementaryFilter`]: Simple complementary filter for encoder + gyro fusion
//! - [`Eskf`]: Error-State Kalman Filter for production-quality fusion
//! - [`MahonyAhrs`]: Mahony AHRS filter with automatic gyro bias calibration
//! - [`FusedPoseTracker`]: Fused pose tracking for SLAM + odometry integration
//! - [`OdometryEvaluator`]: Compute drift metrics for calibration and validation
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::odometry::{WheelOdometry, WheelOdometryConfig, MahonyAhrs, MahonyConfig};
//!
//! let config = WheelOdometryConfig {
//!     ticks_per_meter: 1000.0,
//!     wheel_base: 0.17,
//! };
//! let mut odom = WheelOdometry::new(config);
//!
//! // First update initializes state
//! odom.update(0, 0);
//!
//! // Subsequent updates return pose deltas
//! if let Some(delta) = odom.update(100, 100) {
//!     println!("Moved forward: {} m", delta.x);
//! }
//!
//! // Use Mahony AHRS for orientation with automatic bias calibration
//! let mut ahrs = MahonyAhrs::new(MahonyConfig::default());
//! let imu = RawImuData::new(gyro_raw, tilt_raw);
//! let (roll, pitch, yaw) = ahrs.update(&imu, timestamp_us);
//! ```

mod calibration;
mod complementary;
mod dynamic;
mod eskf;
mod evaluator;
mod fused_tracker;
mod imu_analysis;
mod mahony;
mod wheel_odometry;

pub use calibration::{CRL200S_GYRO_SCALE, GyroBiasEstimator};
pub use complementary::{ComplementaryConfig, ComplementaryFilter};
pub use dynamic::{DynOdometry, DynOdometryConfig, OdometryType};
pub use eskf::{Eskf, EskfConfig, MeasurementNoise, ProcessNoise};
pub use evaluator::{EvaluationResult, OdometryEvaluator, ScenarioBounds, Stats, TestScenario};
pub use fused_tracker::FusedPoseTracker;
pub use imu_analysis::{GyroRotationResult, ImuAnalysisResult, ImuAnalyzer, ImuQuality};
pub use mahony::{
    CalibratedImuData, EulerAngles, MahonyAhrs, MahonyConfig, Quaternion, RawImuData,
};
pub use wheel_odometry::{WheelOdometry, WheelOdometryConfig};
