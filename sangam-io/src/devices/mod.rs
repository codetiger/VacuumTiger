//! Device implementations

pub mod crl200s;

use crate::config::Config;
use crate::core::driver::DeviceDriver;
use crate::error::{Error, Result};
use crl200s::CRL200SDriver;

/// Create a device driver based on configuration
pub fn create_device(config: &Config) -> Result<Box<dyn DeviceDriver>> {
    match config.device.device_type.as_str() {
        "crl200s" => {
            let driver = CRL200SDriver::new(config.device.clone())?;
            Ok(Box::new(driver))
        }
        _ => Err(Error::UnknownDevice(config.device.device_type.clone())),
    }
}
