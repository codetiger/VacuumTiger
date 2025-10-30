//! IMU data types

/// IMU sensor data
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuData {
    /// Accelerometer data (m/s²)
    pub accel: [f32; 3], // x, y, z
    /// Gyroscope data (rad/s)
    pub gyro: [f32; 3], // x, y, z
    /// Magnetometer data (μT), if available
    pub mag: Option<[f32; 3]>, // x, y, z
}

impl ImuData {
    /// Create new IMU data
    pub fn new(accel: [f32; 3], gyro: [f32; 3], mag: Option<[f32; 3]>) -> Self {
        Self { accel, gyro, mag }
    }

    /// Create zero IMU data
    pub fn zero() -> Self {
        Self {
            accel: [0.0, 0.0, 0.0],
            gyro: [0.0, 0.0, 0.0],
            mag: None,
        }
    }

    /// Get accelerometer magnitude
    pub fn accel_magnitude(&self) -> f32 {
        (self.accel[0].powi(2) + self.accel[1].powi(2) + self.accel[2].powi(2)).sqrt()
    }

    /// Get gyroscope magnitude
    pub fn gyro_magnitude(&self) -> f32 {
        (self.gyro[0].powi(2) + self.gyro[1].powi(2) + self.gyro[2].powi(2)).sqrt()
    }
}

impl Default for ImuData {
    fn default() -> Self {
        Self::zero()
    }
}
