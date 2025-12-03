//! Odometry estimation module (Phase 2).
//!
//! Provides wheel odometry and sensor fusion for dead-reckoning.
//!
//! # Components
//!
//! - [`WheelOdometry`]: Convert encoder ticks to pose deltas
//! - [`ComplementaryFilter`]: Simple complementary filter for encoder + gyro fusion
//! - [`Eskf`]: Error-State Kalman Filter for production-quality fusion
//! - [`OdometryEvaluator`]: Compute drift metrics for calibration and validation
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::odometry::{WheelOdometry, WheelOdometryConfig, Eskf, EskfConfig};
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
//! // Use ESKF for better accuracy
//! let mut eskf = Eskf::new(EskfConfig::default());
//! eskf.update(delta, gyro_z, timestamp_us);
//! ```

mod complementary;
mod eskf;
mod evaluator;
mod wheel_odometry;

pub use complementary::{ComplementaryConfig, ComplementaryFilter, CRL200S_GYRO_SCALE};
pub use eskf::{Eskf, EskfConfig, MeasurementNoise, ProcessNoise};
pub use evaluator::{
    EvaluationResult, OdometryEvaluator, ScenarioBounds, Stats, TestScenario,
};
pub use wheel_odometry::{WheelOdometry, WheelOdometryConfig};
