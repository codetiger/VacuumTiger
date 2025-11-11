//! Lidar driver trait

use crate::error::Result;
use crate::types::LidarScan;

/// Lidar scanner driver trait with callback-based API
pub trait LidarDriver: Send {
    /// Start the lidar scanning thread with callback
    ///
    /// The callback will be invoked for each scan received from the lidar.
    fn start<F>(&mut self, callback: F) -> Result<()>
    where
        F: Fn(&LidarScan) + Send + 'static;

    /// Stop the lidar scanning thread
    fn stop(&mut self) -> Result<()>;

    /// Check if lidar is actively scanning
    fn is_active(&self) -> bool;

    /// Get scanning statistics (scan_count, error_count)
    fn get_stats(&self) -> (u64, u64);
}
