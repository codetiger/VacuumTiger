//! Battery monitor driver trait

use crate::error::Result;
use crate::types::BatteryStatus;

/// Battery monitor driver trait
pub trait BatteryDriver: Send {
    /// Read battery status
    fn read(&mut self) -> Result<BatteryStatus>;
}
