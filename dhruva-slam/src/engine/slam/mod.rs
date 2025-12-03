//! Online SLAM module (Phase 7).
//!
//! Provides simultaneous localization and mapping capability by combining
//! odometry, scan matching, and mapping into a unified system.
//!
//! # Architecture
//!
//! The SLAM system uses a submap-based architecture inspired by Cartographer:
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      LOCAL SLAM                              │
//! │                                                              │
//! │  Scan → Preprocess → Match to Submap → Update Submap        │
//! │                           │                                  │
//! │                           ▼                                  │
//! │                   Keyframe Decision                          │
//! │                           │                                  │
//! │              ┌────────────┴────────────┐                     │
//! │              │                         │                     │
//! │              ▼                         ▼                     │
//! │      Create Keyframe           Continue in Submap            │
//! │              │                                               │
//! │              ▼                                               │
//! │      Submap Full? ──Yes──▶ Finish Submap, Create New        │
//! └─────────────────────────────────────────────────────────────┘
//!                              │
//!                              ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │                     GLOBAL SLAM                              │
//! │                  (Background Thread)                         │
//! │                                                              │
//! │  Finished Submaps → Loop Detection → Graph Optimization      │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Components
//!
//! - [`Keyframe`]: A pose-stamped scan used for loop closure
//! - [`KeyframeManager`]: Decides when to create keyframes
//! - [`Submap`]: A local occupancy grid with associated poses
//! - [`SubmapManager`]: Manages submap lifecycle
//! - [`OnlineSlam`]: Main SLAM engine combining all components
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::slam::{OnlineSlam, OnlineSlamConfig};
//! use dhruva_slam::types::{Pose2D, PointCloud2D};
//!
//! let config = OnlineSlamConfig::default();
//! let mut slam = OnlineSlam::new(config);
//!
//! // In sensor loop
//! let result = slam.process_scan(&scan, &odom_delta, timestamp_us);
//! println!("Current pose: {:?}", result.pose);
//!
//! // Get the map
//! let map = slam.global_map();
//! ```

pub mod keyframe;
mod kidnapped_detector;
mod recovery_state;
mod submap;
mod online_slam;

pub use keyframe::{Keyframe, KeyframeManager, KeyframeManagerConfig as KeyframeConfig, ScanContext};
pub use kidnapped_detector::{
    KidnappedDetector, KidnappedDetectorConfig, KidnappedDetection, KidnappedReason,
};
pub use recovery_state::{
    RecoveryAction, RecoveryConfig, RecoveryState, RecoveryStateMachine, RecoveryStats,
    RecoveryStrategy,
};
pub use submap::{Submap, SubmapManager, SubmapManagerConfig as SubmapConfig, SubmapState};
pub use online_slam::{
    OnlineSlam, OnlineSlamConfig, SlamMode, SlamResult, SlamStatus,
};

use crate::core::types::{Pose2D, PointCloud2D};
use crate::algorithms::mapping::OccupancyGrid;

/// Trait for SLAM engines.
///
/// Defines the interface for any SLAM implementation.
pub trait SlamEngine {
    /// Process a new laser scan with odometry.
    ///
    /// # Arguments
    ///
    /// * `scan` - Preprocessed point cloud in robot frame
    /// * `odom_delta` - Odometry-derived motion since last scan
    /// * `timestamp_us` - Scan timestamp in microseconds
    ///
    /// # Returns
    ///
    /// SLAM result containing corrected pose and status.
    fn process_scan(
        &mut self,
        scan: &PointCloud2D,
        odom_delta: &Pose2D,
        timestamp_us: u64,
    ) -> SlamResult;

    /// Get the current pose estimate.
    fn current_pose(&self) -> Pose2D;

    /// Get the current map.
    fn map(&self) -> &OccupancyGrid;

    /// Get SLAM status.
    fn status(&self) -> SlamStatus;

    /// Reset SLAM to initial state.
    fn reset(&mut self);

    /// Reset SLAM to a specific pose.
    fn reset_to(&mut self, pose: Pose2D);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_module_compiles() {
        // Basic smoke test that the module structure is valid
        let _ = OnlineSlamConfig::default();
    }
}
