//! Lidar driver trait

use crate::error::Result;
use crate::types::LidarScan;

/// Lidar scanner driver trait
pub trait LidarDriver: Send {
    /// Start scanning
    fn start(&mut self) -> Result<()>;

    /// Get the latest scan (non-blocking)
    fn get_scan(&mut self) -> Result<Option<LidarScan>>;

    /// Stop scanning
    fn stop(&mut self) -> Result<()>;

    /// Check if lidar is scanning
    fn is_scanning(&self) -> bool;
}
