//! Odometry processing pipeline combining wheel odometry with complementary filter.
//!
//! This module provides a unified interface for processing raw sensor data
//! (encoder ticks and gyroscope readings) and producing fused pose estimates.
//!
//! # Usage
//!
//! ```ignore
//! use dhruva_slam::OdometryPipeline;
//!
//! let config = OdometryPipelineConfig::default();
//! let mut pipeline = OdometryPipeline::new(config);
//!
//! // In sensor loop (500Hz from SangamIO)
//! if let Some(pose) = pipeline.process(left_ticks, right_ticks, gyro_z, timestamp_us) {
//!     // New pose available at output rate (e.g., 50Hz)
//!     publisher.publish_pose(&pose, timestamp_us);
//! }
//! ```

use crate::core::types::Pose2D;
use crate::io::streaming::OdometryDiagnostics;
use crate::sensors::odometry::{
    ComplementaryConfig, ComplementaryFilter, WheelOdometry, WheelOdometryConfig,
};

/// Configuration for the odometry pipeline.
#[derive(Debug, Clone)]
pub struct OdometryPipelineConfig {
    /// Wheel odometry configuration
    pub wheel_odom: WheelOdometryConfig,

    /// Complementary filter configuration
    pub filter: ComplementaryConfig,

    /// Output rate in Hz (decimation from input rate)
    ///
    /// Input is typically 500Hz from SangamIO, output should be
    /// lower (e.g., 50Hz) for visualization.
    pub output_rate_hz: f32,

    /// Number of samples to collect for gyro bias calibration at startup.
    ///
    /// At 500Hz, 1500 samples = 3 seconds of calibration.
    /// Set to 0 to disable auto-calibration.
    pub calibration_samples: u32,
}

impl Default for OdometryPipelineConfig {
    fn default() -> Self {
        Self {
            wheel_odom: WheelOdometryConfig::default(),
            filter: ComplementaryConfig::default(),
            output_rate_hz: 50.0,
            calibration_samples: 1500, // ~3 seconds at 500Hz
        }
    }
}

/// Calibration state for gyro bias auto-detection.
#[derive(Debug)]
enum CalibrationState {
    /// Collecting samples for calibration
    Calibrating {
        samples: Vec<i32>,
        target_count: u32,
    },
    /// Calibration complete
    Calibrated,
}

/// Odometry processing pipeline.
///
/// Combines wheel odometry and complementary filter to produce fused
/// pose estimates at a configurable output rate.
///
/// # Gyro Calibration
///
/// On startup, the pipeline collects gyro samples while the robot should be
/// stationary. After collecting `calibration_samples` readings, it computes
/// the average as the gyro bias. During calibration, `process()` returns `None`.
#[derive(Debug)]
pub struct OdometryPipeline {
    wheel_odom: WheelOdometry,
    filter: ComplementaryFilter,
    output_interval_us: u64,
    last_output_us: u64,
    // Gyro calibration
    calibration: CalibrationState,
    calibrated_bias: f32,
    // Diagnostics tracking
    diagnostics: PipelineDiagnostics,
}

#[derive(Debug, Default)]
struct PipelineDiagnostics {
    /// Total distance traveled
    total_distance: f32,
    /// Accumulated left wheel ticks for rate calculation
    left_tick_accumulator: i32,
    /// Accumulated right wheel ticks for rate calculation
    right_tick_accumulator: i32,
    /// Last diagnostics calculation timestamp
    last_diag_us: u64,
    /// Diagnostics calculation interval (1 second)
    diag_interval_us: u64,
    /// Last calculated tick rates
    tick_rate_left: f32,
    tick_rate_right: f32,
}

impl OdometryPipeline {
    /// Create a new odometry pipeline with the given configuration.
    pub fn new(config: OdometryPipelineConfig) -> Self {
        let output_interval_us = (1_000_000.0 / config.output_rate_hz) as u64;

        let calibration = if config.calibration_samples > 0 {
            CalibrationState::Calibrating {
                samples: Vec::with_capacity(config.calibration_samples as usize),
                target_count: config.calibration_samples,
            }
        } else {
            CalibrationState::Calibrated
        };

        Self {
            wheel_odom: WheelOdometry::new(config.wheel_odom),
            filter: ComplementaryFilter::new(config.filter),
            output_interval_us,
            last_output_us: 0,
            calibration,
            calibrated_bias: config.filter.gyro_bias_z,
            diagnostics: PipelineDiagnostics {
                diag_interval_us: 1_000_000, // 1 second
                ..Default::default()
            },
        }
    }

    /// Check if gyro calibration is still in progress.
    pub fn is_calibrating(&self) -> bool {
        matches!(self.calibration, CalibrationState::Calibrating { .. })
    }

    /// Get the calibrated gyro bias (only valid after calibration completes).
    pub fn gyro_bias(&self) -> f32 {
        self.calibrated_bias
    }

    /// Process new sensor data.
    ///
    /// # Arguments
    ///
    /// * `left` - Left wheel encoder ticks (u16, wrapping)
    /// * `right` - Right wheel encoder ticks (u16, wrapping)
    /// * `gyro_yaw` - Raw gyroscope yaw reading (i16, from gyro_x on CRL-200S)
    /// * `timestamp_us` - Timestamp in microseconds
    ///
    /// # Returns
    ///
    /// `Some(Pose2D)` if enough time has passed since last output (rate limiting),
    /// `None` otherwise. Returns `None` during calibration phase.
    pub fn process(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        // Handle calibration phase
        match &mut self.calibration {
            CalibrationState::Calibrating {
                samples,
                target_count,
            } => {
                samples.push(gyro_yaw as i32);

                if samples.len() >= *target_count as usize {
                    // Calculate average bias
                    let sum: i64 = samples.iter().map(|&x| x as i64).sum();
                    self.calibrated_bias = sum as f32 / samples.len() as f32;

                    log::info!(
                        "Gyro calibration complete: bias = {:.2} (raw units)",
                        self.calibrated_bias
                    );

                    // Update filter config with calibrated bias
                    let mut config = *self.filter.config();
                    config.gyro_bias_z = self.calibrated_bias;
                    self.filter = ComplementaryFilter::new(config);

                    // Transition to calibrated state
                    self.calibration = CalibrationState::Calibrated;
                }

                // During calibration, also initialize wheel odometry state
                self.wheel_odom.update(left, right);
                return None;
            }
            CalibrationState::Calibrated => {}
        }

        // Update wheel odometry
        let encoder_delta = match self.wheel_odom.update(left, right) {
            Some(delta) => {
                // Track distance for diagnostics
                self.diagnostics.total_distance += (delta.x * delta.x + delta.y * delta.y).sqrt();
                delta
            }
            None => return None, // First update after calibration, initializing state
        };

        // Update complementary filter with calibrated bias
        let pose = self.filter.update(encoder_delta, gyro_yaw, timestamp_us);

        // Track tick deltas for rate calculation
        self.diagnostics.left_tick_accumulator += 1;
        self.diagnostics.right_tick_accumulator += 1;

        // Rate limiting: only output at configured rate
        if timestamp_us >= self.last_output_us + self.output_interval_us {
            self.last_output_us = timestamp_us;
            Some(pose)
        } else {
            None
        }
    }

    /// Get the current pose estimate.
    pub fn pose(&self) -> Pose2D {
        self.filter.pose()
    }

    /// Reset the pipeline to initial state.
    pub fn reset(&mut self) {
        self.wheel_odom.reset();
        self.filter.reset();
        self.last_output_us = 0;
        self.diagnostics = PipelineDiagnostics {
            diag_interval_us: 1_000_000,
            ..Default::default()
        };
    }

    /// Reset to a specific pose.
    pub fn reset_to(&mut self, pose: Pose2D) {
        self.wheel_odom.reset();
        self.filter.reset_to(pose);
        self.last_output_us = 0;
    }

    /// Get diagnostics for the pipeline.
    ///
    /// # Arguments
    ///
    /// * `timestamp_us` - Current timestamp for rate calculations
    ///
    /// # Returns
    ///
    /// `Some(OdometryDiagnostics)` if diagnostics interval has passed,
    /// `None` otherwise.
    pub fn diagnostics(&mut self, timestamp_us: u64) -> Option<OdometryDiagnostics> {
        if timestamp_us >= self.diagnostics.last_diag_us + self.diagnostics.diag_interval_us {
            let dt = if self.diagnostics.last_diag_us > 0 {
                (timestamp_us - self.diagnostics.last_diag_us) as f32 / 1_000_000.0
            } else {
                1.0
            };

            // Calculate tick rates
            self.diagnostics.tick_rate_left = self.diagnostics.left_tick_accumulator as f32 / dt;
            self.diagnostics.tick_rate_right = self.diagnostics.right_tick_accumulator as f32 / dt;

            // Reset accumulators
            self.diagnostics.left_tick_accumulator = 0;
            self.diagnostics.right_tick_accumulator = 0;
            self.diagnostics.last_diag_us = timestamp_us;

            Some(OdometryDiagnostics {
                drift_rate: 0.0, // TODO: Calculate actual drift
                tick_rate_left: self.diagnostics.tick_rate_left,
                tick_rate_right: self.diagnostics.tick_rate_right,
                distance_traveled: self.diagnostics.total_distance,
                gyro_bias: self.filter.config().gyro_bias_z,
            })
        } else {
            None
        }
    }

    /// Get the wheel odometry configuration.
    pub fn wheel_config(&self) -> &WheelOdometryConfig {
        self.wheel_odom.config()
    }

    /// Get the filter configuration.
    pub fn filter_config(&self) -> &ComplementaryConfig {
        self.filter.config()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn test_config() -> OdometryPipelineConfig {
        OdometryPipelineConfig {
            wheel_odom: WheelOdometryConfig {
                ticks_per_meter: 1000.0,
                wheel_base: 0.2,
            },
            filter: ComplementaryConfig {
                alpha: 0.98,
                gyro_scale: 0.001,
                gyro_bias_z: 0.0,
            },
            output_rate_hz: 50.0,   // 20ms interval
            calibration_samples: 0, // Disable calibration for tests
        }
    }

    #[test]
    fn test_first_update_returns_none() {
        let mut pipeline = OdometryPipeline::new(test_config());
        assert!(pipeline.process(0, 0, 0, 0).is_none());
    }

    #[test]
    fn test_rate_limiting() {
        let mut pipeline = OdometryPipeline::new(test_config());

        // Initialize at time 0
        pipeline.process(0, 0, 0, 0);

        // Second update at 20ms (50Hz interval) should output first valid data
        let result = pipeline.process(100, 100, 0, 20000);
        assert!(
            result.is_some(),
            "First valid data should output at rate interval"
        );

        // Third update 5ms later - within rate limit, should not output
        let result = pipeline.process(110, 110, 0, 25000);
        assert!(result.is_none(), "Should not output within rate interval");

        // Fourth update at 40ms (20ms after last output) - should output again
        let result = pipeline.process(120, 120, 0, 40000);
        assert!(
            result.is_some(),
            "Should output after rate interval elapsed"
        );
    }

    #[test]
    fn test_straight_motion() {
        let mut pipeline = OdometryPipeline::new(test_config());

        // Initialize
        pipeline.process(0, 0, 0, 0);

        // Move forward 1 meter
        let pose = pipeline.process(1000, 1000, 0, 20000).unwrap();

        assert_relative_eq!(pose.x, 1.0, epsilon = 0.02);
        assert_relative_eq!(pose.y, 0.0, epsilon = 0.01);
        assert_relative_eq!(pose.theta, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_pose_accessor() {
        let mut pipeline = OdometryPipeline::new(test_config());

        pipeline.process(0, 0, 0, 0);
        pipeline.process(500, 500, 0, 20000);

        let pose = pipeline.pose();
        assert!(pose.x > 0.0);
    }

    #[test]
    fn test_reset() {
        let mut pipeline = OdometryPipeline::new(test_config());

        pipeline.process(0, 0, 0, 0);
        pipeline.process(1000, 1000, 0, 20000);

        assert!(pipeline.pose().x > 0.5);

        pipeline.reset();

        let pose = pipeline.pose();
        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.y, 0.0);
        assert_eq!(pose.theta, 0.0);
    }

    #[test]
    fn test_reset_to() {
        let mut pipeline = OdometryPipeline::new(test_config());

        let new_pose = Pose2D::new(5.0, 3.0, 1.0);
        pipeline.reset_to(new_pose);

        let pose = pipeline.pose();
        assert_relative_eq!(pose.x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(pose.y, 3.0, epsilon = 1e-6);
        assert_relative_eq!(pose.theta, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_diagnostics_interval() {
        let mut pipeline = OdometryPipeline::new(test_config());

        pipeline.process(0, 0, 0, 0);
        pipeline.process(100, 100, 0, 20000);

        // Before 1 second, no diagnostics
        assert!(pipeline.diagnostics(500_000).is_none());

        // After 1 second, get diagnostics
        let diag = pipeline.diagnostics(1_000_001).unwrap();
        assert!(diag.distance_traveled > 0.0);
    }

    #[test]
    fn test_config_accessors() {
        let config = test_config();
        let pipeline = OdometryPipeline::new(config);

        assert_eq!(pipeline.wheel_config().ticks_per_meter, 1000.0);
        assert_eq!(pipeline.filter_config().alpha, 0.98);
    }
}
