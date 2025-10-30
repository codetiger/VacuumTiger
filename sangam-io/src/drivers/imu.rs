//! IMU driver trait

use crate::error::Result;
use crate::types::ImuData;

/// IMU sensor driver trait
pub trait ImuDriver: Send {
    /// Read IMU data
    fn read(&mut self) -> Result<ImuData>;

    /// Calibrate IMU (if supported)
    fn calibrate(&mut self) -> Result<()> {
        Err(crate::Error::NotSupported(
            "IMU calibration not supported".to_string(),
        ))
    }
}
