//! Motion filter for scan insertion throttling.
//!
//! This module implements Cartographer-style motion filtering that controls
//! when lidar scans are inserted into the map. By requiring minimum motion
//! between insertions, we reduce computational load and prevent redundant
//! map updates.
//!
//! # Insertion Criteria
//!
//! A scan is inserted when ANY of these conditions are met:
//!
//! 1. **Time**: Maximum time since last insertion exceeded
//! 2. **Distance**: Robot has moved beyond distance threshold
//! 3. **Rotation**: Robot has rotated beyond angle threshold
//!
//! # Cartographer Defaults
//!
//! The default values are based on Cartographer's 2D SLAM configuration:
//! - Max time: 5.0 seconds (insert at least every 5s)
//! - Max distance: 0.2 meters (20cm)
//! - Max angle: 0.035 radians (~2°)
//!
//! # Example
//!
//! ```rust,ignore
//! use vastu_slam::core::{MotionFilter, MotionFilterConfig, Pose2D};
//!
//! let config = MotionFilterConfig::default();
//! let mut filter = MotionFilter::new(config);
//!
//! // Check if scan should be inserted
//! if filter.should_insert(current_pose, timestamp_us) {
//!     // Process and insert the scan
//!     map.insert_scan(&scan);
//!
//!     // Mark as accepted
//!     filter.accept(current_pose, timestamp_us);
//! }
//! ```

use serde::{Deserialize, Serialize};

use super::Pose2D;

/// Configuration for the motion filter.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotionFilterConfig {
    /// Maximum time between scan insertions (seconds).
    ///
    /// If this much time has passed since the last insertion,
    /// insert a new scan regardless of motion.
    ///
    /// Default: 5.0 seconds
    #[serde(default = "default_max_time_seconds")]
    pub max_time_seconds: f32,

    /// Minimum distance to trigger scan insertion (meters).
    ///
    /// If the robot has moved this far since the last insertion,
    /// insert a new scan.
    ///
    /// Default: 0.2 meters (20cm)
    #[serde(default = "default_max_distance_meters")]
    pub max_distance_meters: f32,

    /// Minimum rotation to trigger scan insertion (radians).
    ///
    /// If the robot has rotated this much since the last insertion,
    /// insert a new scan.
    ///
    /// Default: 0.035 radians (~2°)
    #[serde(default = "default_max_angle_radians")]
    pub max_angle_radians: f32,
}

fn default_max_time_seconds() -> f32 {
    5.0
}
fn default_max_distance_meters() -> f32 {
    0.2
}
fn default_max_angle_radians() -> f32 {
    0.035
}

impl Default for MotionFilterConfig {
    fn default() -> Self {
        Self {
            max_time_seconds: 5.0,
            max_distance_meters: 0.2,
            max_angle_radians: 0.035, // ~2 degrees
        }
    }
}

impl MotionFilterConfig {
    /// Create a config for high-frequency insertion (more scans).
    ///
    /// Useful for detailed mapping or when computational resources allow.
    pub fn high_frequency() -> Self {
        Self {
            max_time_seconds: 2.0,
            max_distance_meters: 0.1,
            max_angle_radians: 0.02,
        }
    }

    /// Create a config for low-frequency insertion (fewer scans).
    ///
    /// Useful for resource-constrained systems or large environments.
    pub fn low_frequency() -> Self {
        Self {
            max_time_seconds: 10.0,
            max_distance_meters: 0.5,
            max_angle_radians: 0.1,
        }
    }
}

/// Motion filter for controlling scan insertion rate.
///
/// Tracks robot motion and decides when to insert scans based on
/// distance, rotation, and time thresholds.
#[derive(Clone, Debug)]
pub struct MotionFilter {
    /// Configuration
    config: MotionFilterConfig,

    /// Last accepted pose
    last_pose: Option<Pose2D>,

    /// Last accepted timestamp (microseconds)
    last_timestamp_us: Option<u64>,

    /// Number of scans accepted
    accepted_count: u64,

    /// Number of scans rejected
    rejected_count: u64,
}

impl MotionFilter {
    /// Create a new motion filter with the given configuration.
    pub fn new(config: MotionFilterConfig) -> Self {
        Self {
            config,
            last_pose: None,
            last_timestamp_us: None,
            accepted_count: 0,
            rejected_count: 0,
        }
    }

    /// Check if a scan should be inserted.
    ///
    /// Returns `true` if any of the following conditions are met:
    /// - This is the first scan
    /// - Time since last insertion exceeds `max_time_seconds`
    /// - Distance moved exceeds `max_distance_meters`
    /// - Rotation exceeds `max_angle_radians`
    pub fn should_insert(&self, pose: Pose2D, timestamp_us: u64) -> bool {
        // Always accept first scan
        let (last_pose, last_ts) = match (self.last_pose, self.last_timestamp_us) {
            (Some(p), Some(t)) => (p, t),
            _ => return true,
        };

        // Check time threshold
        let dt_seconds = (timestamp_us.saturating_sub(last_ts)) as f32 / 1_000_000.0;
        if dt_seconds >= self.config.max_time_seconds {
            return true;
        }

        // Check distance threshold
        let dx = pose.x - last_pose.x;
        let dy = pose.y - last_pose.y;
        let distance = (dx * dx + dy * dy).sqrt();
        if distance >= self.config.max_distance_meters {
            return true;
        }

        // Check angle threshold
        let dtheta = normalize_angle(pose.theta - last_pose.theta).abs();
        if dtheta >= self.config.max_angle_radians {
            return true;
        }

        false
    }

    /// Mark a pose as accepted (scan was inserted).
    ///
    /// Call this after successfully inserting a scan to update
    /// the filter's state.
    pub fn accept(&mut self, pose: Pose2D, timestamp_us: u64) {
        self.last_pose = Some(pose);
        self.last_timestamp_us = Some(timestamp_us);
        self.accepted_count += 1;
    }

    /// Mark a scan as rejected (not inserted).
    ///
    /// Call this when `should_insert` returns false to track statistics.
    pub fn reject(&mut self) {
        self.rejected_count += 1;
    }

    /// Get the number of accepted scans.
    #[inline]
    pub fn accepted_count(&self) -> u64 {
        self.accepted_count
    }

    /// Get the number of rejected scans.
    #[inline]
    pub fn rejected_count(&self) -> u64 {
        self.rejected_count
    }

    /// Get the total number of scans considered.
    #[inline]
    pub fn total_count(&self) -> u64 {
        self.accepted_count + self.rejected_count
    }

    /// Get the acceptance rate (0.0 - 1.0).
    pub fn acceptance_rate(&self) -> f32 {
        let total = self.total_count();
        if total == 0 {
            1.0
        } else {
            self.accepted_count as f32 / total as f32
        }
    }

    /// Get the last accepted pose.
    #[inline]
    pub fn last_pose(&self) -> Option<Pose2D> {
        self.last_pose
    }

    /// Get the last accepted timestamp.
    #[inline]
    pub fn last_timestamp_us(&self) -> Option<u64> {
        self.last_timestamp_us
    }

    /// Get time since last insertion (seconds).
    ///
    /// Returns `None` if no scan has been accepted yet.
    pub fn time_since_last(&self, current_timestamp_us: u64) -> Option<f32> {
        self.last_timestamp_us
            .map(|last_ts| (current_timestamp_us.saturating_sub(last_ts)) as f32 / 1_000_000.0)
    }

    /// Get distance since last insertion (meters).
    ///
    /// Returns `None` if no scan has been accepted yet.
    pub fn distance_since_last(&self, current_pose: Pose2D) -> Option<f32> {
        self.last_pose.map(|last_pose| {
            let dx = current_pose.x - last_pose.x;
            let dy = current_pose.y - last_pose.y;
            (dx * dx + dy * dy).sqrt()
        })
    }

    /// Get rotation since last insertion (radians, absolute).
    ///
    /// Returns `None` if no scan has been accepted yet.
    pub fn rotation_since_last(&self, current_pose: Pose2D) -> Option<f32> {
        self.last_pose
            .map(|last_pose| normalize_angle(current_pose.theta - last_pose.theta).abs())
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.last_pose = None;
        self.last_timestamp_us = None;
        self.accepted_count = 0;
        self.rejected_count = 0;
    }

    /// Reset only the statistics, keeping the last pose/timestamp.
    pub fn reset_stats(&mut self) {
        self.accepted_count = 0;
        self.rejected_count = 0;
    }
}

/// Normalize angle to [-PI, PI] range.
#[inline]
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle;
    while a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    }
    while a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motion_filter_creation() {
        let filter = MotionFilter::new(MotionFilterConfig::default());
        assert_eq!(filter.accepted_count(), 0);
        assert_eq!(filter.rejected_count(), 0);
        assert!(filter.last_pose().is_none());
    }

    #[test]
    fn test_first_scan_always_accepted() {
        let filter = MotionFilter::new(MotionFilterConfig::default());
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        assert!(filter.should_insert(pose, 0));
    }

    #[test]
    fn test_no_motion_rejected() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        // Accept first scan
        filter.accept(pose, 0);

        // Same pose, short time later - should be rejected
        assert!(!filter.should_insert(pose, 100000)); // 0.1 seconds later
    }

    #[test]
    fn test_distance_threshold() {
        let config = MotionFilterConfig {
            max_distance_meters: 0.2,
            max_time_seconds: 1000.0,  // Disable time trigger
            max_angle_radians: 1000.0, // Disable angle trigger
        };
        let mut filter = MotionFilter::new(config);

        let pose1 = Pose2D::new(0.0, 0.0, 0.0);
        filter.accept(pose1, 0);

        // Small movement - rejected
        let pose2 = Pose2D::new(0.1, 0.0, 0.0);
        assert!(!filter.should_insert(pose2, 100000));

        // Large movement - accepted
        let pose3 = Pose2D::new(0.25, 0.0, 0.0);
        assert!(filter.should_insert(pose3, 200000));
    }

    #[test]
    fn test_angle_threshold() {
        let config = MotionFilterConfig {
            max_distance_meters: 1000.0, // Disable distance trigger
            max_time_seconds: 1000.0,    // Disable time trigger
            max_angle_radians: 0.1,
        };
        let mut filter = MotionFilter::new(config);

        let pose1 = Pose2D::new(0.0, 0.0, 0.0);
        filter.accept(pose1, 0);

        // Small rotation - rejected
        let pose2 = Pose2D::new(0.0, 0.0, 0.05);
        assert!(!filter.should_insert(pose2, 100000));

        // Large rotation - accepted
        let pose3 = Pose2D::new(0.0, 0.0, 0.15);
        assert!(filter.should_insert(pose3, 200000));
    }

    #[test]
    fn test_time_threshold() {
        let config = MotionFilterConfig {
            max_distance_meters: 1000.0, // Disable distance trigger
            max_time_seconds: 1.0,
            max_angle_radians: 1000.0, // Disable angle trigger
        };
        let mut filter = MotionFilter::new(config);

        let pose = Pose2D::new(0.0, 0.0, 0.0);
        filter.accept(pose, 0);

        // Short time - rejected
        assert!(!filter.should_insert(pose, 500000)); // 0.5 seconds

        // Long time - accepted
        assert!(filter.should_insert(pose, 1500000)); // 1.5 seconds
    }

    #[test]
    fn test_statistics() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        filter.accept(pose, 0);
        filter.reject();
        filter.reject();
        filter.accept(Pose2D::new(1.0, 0.0, 0.0), 1000000);

        assert_eq!(filter.accepted_count(), 2);
        assert_eq!(filter.rejected_count(), 2);
        assert_eq!(filter.total_count(), 4);
        assert!((filter.acceptance_rate() - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_distance_since_last() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());

        let pose1 = Pose2D::new(0.0, 0.0, 0.0);
        filter.accept(pose1, 0);

        let pose2 = Pose2D::new(3.0, 4.0, 0.0);
        let dist = filter.distance_since_last(pose2).unwrap();
        assert!((dist - 5.0).abs() < 0.01); // 3-4-5 triangle
    }

    #[test]
    fn test_reset() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());

        filter.accept(Pose2D::new(1.0, 2.0, 0.5), 1000000);
        filter.reject();

        filter.reset();

        assert!(filter.last_pose().is_none());
        assert_eq!(filter.accepted_count(), 0);
        assert_eq!(filter.rejected_count(), 0);
    }

    #[test]
    fn test_high_frequency_config() {
        let config = MotionFilterConfig::high_frequency();
        assert!(config.max_distance_meters < 0.2);
        assert!(config.max_time_seconds < 5.0);
    }

    #[test]
    fn test_low_frequency_config() {
        let config = MotionFilterConfig::low_frequency();
        assert!(config.max_distance_meters > 0.2);
        assert!(config.max_time_seconds > 5.0);
    }
}
