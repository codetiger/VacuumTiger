//! Device driver implementations and factory.
//!
//! # Adding a New Device
//!
//! 1. **Create module**: `src/devices/my_device/mod.rs`
//! 2. **Implement trait**: `impl DeviceDriver for MyDeviceDriver`
//! 3. **Register here**: Add match arm in [`create_device`]
//!
//! ## Module Structure
//! ```text
//! devices/my_device/
//! ├── mod.rs          # MyDeviceDriver struct + DeviceDriver impl
//! ├── protocol.rs     # Packet parsing (if serial)
//! └── commands.rs     # Command handlers (optional)
//! ```
//!
//! ## Key Patterns (from CRL200S)
//! - Use `Arc<Mutex<>>` for serial port shared across threads
//! - Use `Arc<AtomicBool>` for shutdown signal
//! - Implement `Drop` to join spawned threads
//! - Return sensor groups from `initialize()` for TCP streaming
//!
//! See [`crl200s::CRL200SDriver`] for a complete example.

pub mod crl200s;

use crate::config::Config;
use crate::core::driver::DeviceDriver;
use crate::error::{Error, Result};
use crl200s::CRL200SDriver;

/// Device factory - creates driver based on `device.type` in config.
///
/// To add a new device type, add a match arm here:
/// ```ignore
/// "my_device" => Ok(Box::new(MyDeviceDriver::new(config.device.clone())?)),
/// ```
pub fn create_device(config: &Config) -> Result<Box<dyn DeviceDriver>> {
    match config.device.device_type.as_str() {
        "crl200s" => {
            let driver = CRL200SDriver::new(config.device.clone())?;
            Ok(Box::new(driver))
        }
        _ => Err(Error::UnknownDevice(config.device.device_type.clone())),
    }
}
