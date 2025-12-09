//! Odometry-related types.

use serde::{Deserialize, Serialize};

/// 2D velocity (linear and angular).
///
/// Represents velocities for a differential drive robot.
/// Linear velocities are in m/s, angular in rad/s.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Twist2D {
    /// Forward velocity in m/s
    pub linear_x: f32,
    /// Lateral velocity in m/s (usually 0 for differential drive)
    pub linear_y: f32,
    /// Rotational velocity in rad/s (positive = counter-clockwise)
    pub angular_z: f32,
}

impl Twist2D {
    /// Create a new twist.
    #[inline]
    pub fn new(linear_x: f32, linear_y: f32, angular_z: f32) -> Self {
        Self {
            linear_x,
            linear_y,
            angular_z,
        }
    }

    /// Create a twist with only forward motion.
    #[inline]
    pub fn forward(linear_x: f32) -> Self {
        Self {
            linear_x,
            linear_y: 0.0,
            angular_z: 0.0,
        }
    }

    /// Create a twist with only rotation.
    #[inline]
    pub fn rotation(angular_z: f32) -> Self {
        Self {
            linear_x: 0.0,
            linear_y: 0.0,
            angular_z,
        }
    }

    /// Epsilon for floating point near-zero comparisons.
    const EPSILON: f32 = 1e-9;

    /// Check if this twist represents no motion (within epsilon tolerance).
    #[inline]
    pub fn is_zero(&self) -> bool {
        self.linear_x.abs() < Self::EPSILON
            && self.linear_y.abs() < Self::EPSILON
            && self.angular_z.abs() < Self::EPSILON
    }

    /// Check if this twist is exactly zero (no epsilon tolerance).
    ///
    /// Use `is_zero()` for numerical comparisons. This method is only
    /// useful when you need to check for exact zero values.
    #[inline]
    pub fn is_exactly_zero(&self) -> bool {
        self.linear_x == 0.0 && self.linear_y == 0.0 && self.angular_z == 0.0
    }
}

/// Raw IMU sensor reading with accelerometer and gyroscope data.
///
/// All values should be calibrated to SI units before use.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct ImuReading {
    /// Linear acceleration [x, y, z] in m/s²
    /// x = forward, y = left, z = up (robot frame)
    pub accel: [f32; 3],
    /// Angular velocity [x, y, z] in rad/s
    /// x = roll rate, y = pitch rate, z = yaw rate
    pub gyro: [f32; 3],
}

impl ImuReading {
    /// Create a new IMU reading.
    #[inline]
    pub fn new(accel: [f32; 3], gyro: [f32; 3]) -> Self {
        Self { accel, gyro }
    }

    /// Get gyroscope Z (yaw rate) - the primary value for 2D odometry.
    #[inline]
    pub fn gyro_z(&self) -> f32 {
        self.gyro[2]
    }

    /// Create from raw sensor values with calibration.
    ///
    /// Applies scale factors and bias corrections:
    /// - `accel_scale`: Multiply raw accel by this to get m/s²
    /// - `gyro_scale`: Multiply raw gyro by this to get rad/s
    /// - `gyro_bias`: Subtract this from raw gyro before scaling
    pub fn from_raw(
        accel_raw: [i16; 3],
        gyro_raw: [i16; 3],
        accel_scale: f32,
        gyro_scale: f32,
        gyro_bias: [f32; 3],
    ) -> Self {
        Self {
            accel: [
                accel_raw[0] as f32 * accel_scale,
                accel_raw[1] as f32 * accel_scale,
                accel_raw[2] as f32 * accel_scale,
            ],
            gyro: [
                (gyro_raw[0] as f32 - gyro_bias[0]) * gyro_scale,
                (gyro_raw[1] as f32 - gyro_bias[1]) * gyro_scale,
                (gyro_raw[2] as f32 - gyro_bias[2]) * gyro_scale,
            ],
        }
    }
}

/// 3x3 covariance matrix for 2D pose uncertainty (x, y, theta).
///
/// Stored as row-major array: [xx, xy, xt, yx, yy, yt, tx, ty, tt]
/// where t = theta.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Covariance2D {
    /// Row-major 3x3 matrix data
    data: [f32; 9],
}

impl Covariance2D {
    /// Create a zero covariance matrix.
    #[inline]
    pub fn zero() -> Self {
        Self { data: [0.0; 9] }
    }

    /// Create a diagonal covariance matrix.
    ///
    /// Parameters are variances: xx = σ²_x, yy = σ²_y, tt = σ²_θ
    #[inline]
    pub fn diagonal(xx: f32, yy: f32, tt: f32) -> Self {
        Self {
            data: [xx, 0.0, 0.0, 0.0, yy, 0.0, 0.0, 0.0, tt],
        }
    }

    /// Create from row-major array.
    #[inline]
    pub fn from_array(data: [f32; 9]) -> Self {
        Self { data }
    }

    /// Get element at (row, col). Panics if indices out of bounds.
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> f32 {
        debug_assert!(row < 3 && col < 3, "Index out of bounds");
        self.data[row * 3 + col]
    }

    /// Set element at (row, col). Panics if indices out of bounds.
    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: f32) {
        debug_assert!(row < 3 && col < 3, "Index out of bounds");
        self.data[row * 3 + col] = val;
    }

    /// Get variance of x (element [0,0]).
    #[inline]
    pub fn var_x(&self) -> f32 {
        self.data[0]
    }

    /// Get variance of y (element [1,1]).
    #[inline]
    pub fn var_y(&self) -> f32 {
        self.data[4]
    }

    /// Get variance of theta (element [2,2]).
    #[inline]
    pub fn var_theta(&self) -> f32 {
        self.data[8]
    }

    /// Matrix addition.
    #[inline]
    pub fn add(&self, other: &Self) -> Self {
        let mut result = [0.0; 9];
        for (i, val) in result.iter_mut().enumerate() {
            *val = self.data[i] + other.data[i];
        }
        Self { data: result }
    }

    /// Get raw data as slice.
    #[inline]
    pub fn as_slice(&self) -> &[f32; 9] {
        &self.data
    }
}

impl Default for Covariance2D {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_twist2d_creation() {
        let t = Twist2D::new(1.0, 0.5, 0.1);
        assert_eq!(t.linear_x, 1.0);
        assert_eq!(t.linear_y, 0.5);
        assert_eq!(t.angular_z, 0.1);

        let forward = Twist2D::forward(2.0);
        assert_eq!(forward.linear_x, 2.0);
        assert_eq!(forward.linear_y, 0.0);
        assert_eq!(forward.angular_z, 0.0);

        let rotation = Twist2D::rotation(0.5);
        assert_eq!(rotation.linear_x, 0.0);
        assert_eq!(rotation.angular_z, 0.5);

        assert!(Twist2D::default().is_zero());
        assert!(!forward.is_zero());
    }

    #[test]
    fn test_imu_reading() {
        let imu = ImuReading::new([0.0, 0.0, 9.81], [0.0, 0.0, 0.1]);
        assert_eq!(imu.accel[2], 9.81);
        assert_eq!(imu.gyro_z(), 0.1);
    }

    #[test]
    fn test_imu_from_raw() {
        let accel_raw = [0i16, 0, 16384];
        let gyro_raw = [0i16, 0, 1000];
        let accel_scale = 9.81 / 16384.0;
        let gyro_scale = 0.001;
        let gyro_bias = [0.0, 0.0, 100.0];

        let imu = ImuReading::from_raw(accel_raw, gyro_raw, accel_scale, gyro_scale, gyro_bias);

        assert_relative_eq!(imu.accel[2], 9.81, epsilon = 0.01);
        assert_relative_eq!(imu.gyro_z(), 0.9, epsilon = 0.001);
    }

    #[test]
    fn test_covariance2d() {
        let zero = Covariance2D::zero();
        assert_eq!(zero.var_x(), 0.0);
        assert_eq!(zero.var_y(), 0.0);
        assert_eq!(zero.var_theta(), 0.0);

        let diag = Covariance2D::diagonal(0.1, 0.2, 0.05);
        assert_eq!(diag.var_x(), 0.1);
        assert_eq!(diag.var_y(), 0.2);
        assert_eq!(diag.var_theta(), 0.05);
        assert_eq!(diag.get(0, 1), 0.0);

        let mut cov = Covariance2D::zero();
        cov.set(0, 1, 0.01);
        assert_eq!(cov.get(0, 1), 0.01);
    }

    #[test]
    fn test_covariance2d_add() {
        let a = Covariance2D::diagonal(0.1, 0.2, 0.05);
        let b = Covariance2D::diagonal(0.05, 0.1, 0.02);
        let c = a.add(&b);

        assert_relative_eq!(c.var_x(), 0.15, epsilon = 1e-6);
        assert_relative_eq!(c.var_y(), 0.3, epsilon = 1e-6);
        assert_relative_eq!(c.var_theta(), 0.07, epsilon = 1e-6);
    }

    #[test]
    fn test_twist2d_is_zero() {
        assert!(Twist2D::default().is_zero());
        assert!(Twist2D::new(0.0, 0.0, 0.0).is_zero());
        // Values above epsilon should not be zero
        assert!(!Twist2D::new(0.001, 0.0, 0.0).is_zero());
        assert!(!Twist2D::rotation(0.001).is_zero());
        // Very small values below epsilon should be considered zero
        assert!(Twist2D::new(1e-10, 0.0, 0.0).is_zero());
        assert!(Twist2D::new(0.0, 1e-10, 1e-10).is_zero());
    }

    #[test]
    fn test_twist2d_is_exactly_zero() {
        assert!(Twist2D::default().is_exactly_zero());
        assert!(Twist2D::new(0.0, 0.0, 0.0).is_exactly_zero());
        // Even tiny values are not exactly zero
        assert!(!Twist2D::new(1e-10, 0.0, 0.0).is_exactly_zero());
        assert!(!Twist2D::new(0.0, 1e-10, 0.0).is_exactly_zero());
    }

    #[test]
    fn test_covariance_symmetry() {
        let mut cov = Covariance2D::zero();
        cov.set(0, 1, 0.5);
        cov.set(1, 0, 0.5);

        assert_eq!(cov.get(0, 1), cov.get(1, 0));
    }
}
