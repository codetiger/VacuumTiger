//! IMU simulator for mock device
//!
//! Generates gyroscope, accelerometer, and tilt readings with configurable noise.
//!
//! ## Coordinate System (ROS REP-103)
//!
//! All outputs follow ROS REP-103 convention:
//! - **X = forward** (direction robot drives)
//! - **Y = left** (port side)
//! - **Z = up**
//! - **Rotations = counter-clockwise (CCW) positive**
//!
//! ## Sensor Outputs
//!
//! - **Gyroscope**: Angular velocity in raw i16 units
//!   - X (roll rate): rotation around forward axis
//!   - Y (pitch rate): rotation around left axis
//!   - Z (yaw rate): rotation around up axis (CCW positive when viewed from above)
//!
//! - **Accelerometer**: Linear acceleration including gravity, in raw i16 units
//!   - When level and stationary: X≈0, Y≈0, Z≈+16384 (1g upward)
//!
//! - **Tilt**: Low-pass filtered gravity direction (normalized), in raw i16 units
//!   - When level: X≈0, Y≈0, Z≈+16384 (gravity points down, sensor reads up)

use super::config::ImuConfig;
use super::noise::NoiseGenerator;

/// IMU reading in raw i16 units (matching CRL-200S hardware format)
///
/// Coordinate frame follows ROS REP-103:
/// - X = forward (roll axis)
/// - Y = left (pitch axis)
/// - Z = up (yaw axis)
pub struct ImuReading {
    /// Gyroscope readings [x, y, z] - angular velocity in raw units
    /// Z is yaw rate: positive = CCW rotation when viewed from above
    pub gyro: [i16; 3],
    /// Accelerometer readings [x, y, z] - linear acceleration in raw units
    /// When level: Z ≈ +16384 (1g pointing up, opposing gravity)
    pub accel: [i16; 3],
    /// Tilt sensor readings [x, y, z] - filtered gravity direction
    /// When level: Z ≈ +16384 (normalized "up" vector)
    pub tilt: [i16; 3],
}

/// IMU simulator with configurable noise
pub struct ImuSimulator {
    config: ImuConfig,
    noise: NoiseGenerator,
    /// Accumulated gyro bias drift
    gyro_drift: [f32; 3],
}

/// Scale factor: rad/s to raw gyro units
/// Typical MPU6050 at ±2000°/s range: ~16.4 LSB/(°/s) = ~939 LSB/(rad/s)
/// We use 1000 for round numbers
const GYRO_SCALE: f32 = 1000.0;

/// Scale factor: g to raw accel units
/// Typical MPU6050 at ±2g range: 16384 LSB/g
const ACCEL_SCALE: f32 = 16384.0;

/// Scale factor for tilt (same as accel, representing normalized gravity)
const TILT_SCALE: f32 = 16384.0;

impl ImuSimulator {
    /// Create new IMU simulator
    pub fn new(config: &ImuConfig, noise: NoiseGenerator) -> Self {
        Self {
            config: config.clone(),
            noise,
            gyro_drift: [0.0, 0.0, 0.0],
        }
    }

    /// Generate IMU reading based on robot motion
    ///
    /// # Arguments
    /// * `angular_vel` - Robot angular velocity (rad/s, CCW positive around Z)
    /// * `linear_vel` - Robot linear velocity (m/s, forward positive)
    /// * `dt` - Time step (seconds)
    ///
    /// # Returns
    /// IMU reading with gyro, accel, and tilt in ROS REP-103 frame
    pub fn generate(&mut self, angular_vel: f32, linear_vel: f32, dt: f32) -> ImuReading {
        // Copy config values to avoid borrow conflicts
        let gyro_drift_rate = self.config.gyro_noise.drift_rate;
        let gyro_bias = self.config.gyro_noise.bias;
        let gyro_stddev = self.config.gyro_noise.stddev;
        let accel_bias = self.config.accel_noise.bias;
        let accel_stddev = self.config.accel_noise.stddev;
        let tilt_bias = self.config.tilt_noise.bias;
        let tilt_stddev = self.config.tilt_noise.stddev;

        // Update gyro drift (random walk)
        if gyro_drift_rate > 0.0 {
            for i in 0..3 {
                self.gyro_drift[i] += self.noise.gaussian(gyro_drift_rate * dt);
            }
        }

        // =====================================================================
        // Gyroscope (angular velocity)
        // =====================================================================
        // For a 2D ground robot moving on flat surface:
        // - X (roll rate): ~0 (no rolling)
        // - Y (pitch rate): ~0 (no pitching)
        // - Z (yaw rate): = angular_vel (CCW positive)
        let gyro_z_ideal = angular_vel * GYRO_SCALE;

        let gyro_x_noise = self.noise.gaussian(gyro_stddev[0]);
        let gyro_y_noise = self.noise.gaussian(gyro_stddev[1]);
        let gyro_z_noise = self.noise.gaussian(gyro_stddev[2]);

        let gyro = [
            clamp_i16(self.gyro_drift[0] + gyro_bias[0] + gyro_x_noise),
            clamp_i16(self.gyro_drift[1] + gyro_bias[1] + gyro_y_noise),
            clamp_i16(gyro_z_ideal + self.gyro_drift[2] + gyro_bias[2] + gyro_z_noise),
        ];

        // =====================================================================
        // Accelerometer (linear acceleration + gravity)
        // =====================================================================
        // For a 2D ground robot on flat surface:
        // - X: forward acceleration (could add from velocity derivative)
        // - Y: lateral acceleration (~0 for differential drive)
        // - Z: +1g (sensor measures force opposing gravity = upward = positive Z)
        //
        // Note: Centripetal acceleration during rotation is small for typical robot
        // speeds and is neglected here.
        let accel_x_noise = self.noise.gaussian(accel_stddev[0]);
        let accel_y_noise = self.noise.gaussian(accel_stddev[1]);
        let accel_z_noise = self.noise.gaussian(accel_stddev[2]);

        let accel = [
            clamp_i16(accel_bias[0] + accel_x_noise),
            clamp_i16(accel_bias[1] + accel_y_noise),
            clamp_i16(ACCEL_SCALE + accel_bias[2] + accel_z_noise), // +1g in Z
        ];

        // =====================================================================
        // Tilt sensor (low-pass filtered gravity direction)
        // =====================================================================
        // Represents normalized "up" vector (opposite of gravity):
        // - When robot is level: (0, 0, +TILT_SCALE)
        // - When tilted forward: X becomes negative
        // - When tilted left: Y becomes positive
        let tilt_x_noise = self.noise.gaussian(tilt_stddev[0]);
        let tilt_y_noise = self.noise.gaussian(tilt_stddev[1]);
        let tilt_z_noise = self.noise.gaussian(tilt_stddev[2]);

        let tilt = [
            clamp_i16(tilt_bias[0] + tilt_x_noise), // ~0 when level
            clamp_i16(tilt_bias[1] + tilt_y_noise), // ~0 when level
            clamp_i16(TILT_SCALE + tilt_bias[2] + tilt_z_noise), // +1 (up) when level
        ];

        // Suppress unused parameter warning
        let _ = linear_vel;

        ImuReading { gyro, accel, tilt }
    }
}

/// Clamp f32 to i16 range
#[inline]
fn clamp_i16(value: f32) -> i16 {
    value.clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_stationary_level() {
        let config = ImuConfig::default();
        let noise = NoiseGenerator::new(42);
        let mut imu = ImuSimulator::new(&config, noise);

        // Generate reading with zero motion (stationary, level robot)
        let reading = imu.generate(0.0, 0.0, 0.01);

        // Gyro should be near zero (just noise, no rotation)
        assert!(reading.gyro[0].abs() < 100, "gyro_x={}", reading.gyro[0]);
        assert!(reading.gyro[1].abs() < 100, "gyro_y={}", reading.gyro[1]);
        assert!(reading.gyro[2].abs() < 100, "gyro_z={}", reading.gyro[2]);

        // Accel Z should be near +1g (16384) - robot sitting level
        assert!(
            reading.accel[2] > 15000 && reading.accel[2] < 18000,
            "accel_z={} (expected ~16384)",
            reading.accel[2]
        );
        // Accel X/Y should be near zero
        assert!(reading.accel[0].abs() < 100, "accel_x={}", reading.accel[0]);
        assert!(reading.accel[1].abs() < 100, "accel_y={}", reading.accel[1]);

        // Tilt Z should be near +1g (16384) - "up" direction
        assert!(
            reading.tilt[2] > 15000 && reading.tilt[2] < 18000,
            "tilt_z={} (expected ~16384)",
            reading.tilt[2]
        );
    }

    #[test]
    fn test_gyro_ccw_rotation() {
        let config = ImuConfig::default();
        let noise = NoiseGenerator::new(42);
        let mut imu = ImuSimulator::new(&config, noise);

        // Generate with positive (CCW) angular velocity
        let reading = imu.generate(1.0, 0.0, 0.01); // 1 rad/s CCW rotation

        // Z gyro should be positive for CCW rotation (1 rad/s * 1000 scale = 1000 raw units)
        assert!(
            reading.gyro[2] > 500,
            "gyro_z={} (expected positive ~1000 for CCW)",
            reading.gyro[2]
        );
    }

    #[test]
    fn test_gyro_cw_rotation() {
        let config = ImuConfig::default();
        let noise = NoiseGenerator::new(42);
        let mut imu = ImuSimulator::new(&config, noise);

        // Generate with negative (CW) angular velocity
        let reading = imu.generate(-1.0, 0.0, 0.01); // 1 rad/s CW rotation

        // Z gyro should be negative for CW rotation
        assert!(
            reading.gyro[2] < -500,
            "gyro_z={} (expected negative ~-1000 for CW)",
            reading.gyro[2]
        );
    }
}
