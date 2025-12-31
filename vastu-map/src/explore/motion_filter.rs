//! Motion filter for gating scan processing based on robot movement.
//!
//! The motion filter prevents redundant scan processing when the robot is stationary
//! or moving slowly. A scan is only processed if sufficient time, distance, or rotation
//! has occurred since the last processed scan.
//!
//! This improves CPU efficiency and ensures consistent submap sizes regardless of
//! robot speed.

use std::time::Instant;

use serde::{Deserialize, Serialize};

use crate::core::{Pose2D, normalize_angle};

/// Configuration for the motion filter.
///
/// A scan passes the filter (should be processed) if ANY threshold is exceeded:
/// - Time since last scan > max_time_secs
/// - Distance traveled > max_distance
/// - Rotation > max_angle
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotionFilterConfig {
    /// Maximum time between scan insertions (seconds).
    /// Ensures at least some updates even when stationary.
    /// Default: 5.0s (from Google Cartographer)
    #[serde(default = "default_max_time")]
    pub max_time_secs: f32,

    /// Maximum distance before forcing scan insertion (meters).
    /// Default: 0.2m (20cm, from Google Cartographer)
    #[serde(default = "default_max_distance")]
    pub max_distance: f32,

    /// Maximum rotation before forcing scan insertion (radians).
    /// Default: 0.17 rad (~10°, from Google Cartographer)
    #[serde(default = "default_max_angle")]
    pub max_angle: f32,

    /// Whether the motion filter is enabled.
    /// When disabled, all scans pass through.
    /// Default: true
    #[serde(default = "default_enabled")]
    pub enabled: bool,
}

fn default_max_time() -> f32 {
    5.0
}

fn default_max_distance() -> f32 {
    0.2
}

fn default_max_angle() -> f32 {
    0.17
}

fn default_enabled() -> bool {
    true
}

impl Default for MotionFilterConfig {
    fn default() -> Self {
        Self {
            max_time_secs: default_max_time(),
            max_distance: default_max_distance(),
            max_angle: default_max_angle(),
            enabled: default_enabled(),
        }
    }
}

impl MotionFilterConfig {
    /// Create a permissive filter that processes more scans.
    /// Useful for high-quality mapping at the cost of CPU.
    pub fn permissive() -> Self {
        Self {
            max_time_secs: 2.0,
            max_distance: 0.1,
            max_angle: 0.08,
            enabled: true,
        }
    }

    /// Create a strict filter that processes fewer scans.
    /// Useful for CPU-constrained systems.
    pub fn strict() -> Self {
        Self {
            max_time_secs: 10.0,
            max_distance: 0.3,
            max_angle: 0.25,
            enabled: true,
        }
    }

    /// Create a disabled filter (all scans pass through).
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..Default::default()
        }
    }
}

/// Motion filter for gating scan processing.
///
/// The filter tracks the last processed pose and time, and determines
/// whether a new scan should be processed based on the configured thresholds.
pub struct MotionFilter {
    /// Pose when last scan was processed
    last_pose: Option<Pose2D>,
    /// Time when last scan was processed
    last_time: Option<Instant>,
    /// Configuration
    config: MotionFilterConfig,
    /// Count of scans that passed the filter
    passed_count: u64,
    /// Count of scans that were filtered out
    filtered_count: u64,
}

impl MotionFilter {
    /// Create a new motion filter with the given configuration.
    pub fn new(config: MotionFilterConfig) -> Self {
        Self {
            last_pose: None,
            last_time: None,
            config,
            passed_count: 0,
            filtered_count: 0,
        }
    }

    /// Check if a scan at the given pose should be processed.
    ///
    /// Returns true if the scan should be processed, false if it should be skipped.
    /// Updates internal state when returning true.
    pub fn should_process(&mut self, pose: Pose2D) -> bool {
        // If disabled, always process
        if !self.config.enabled {
            self.passed_count += 1;
            return true;
        }

        let now = Instant::now();

        // First scan always passes
        let (Some(last_pose), Some(last_time)) = (self.last_pose, self.last_time) else {
            self.last_pose = Some(pose);
            self.last_time = Some(now);
            self.passed_count += 1;
            return true;
        };

        // Check time threshold
        let dt = now.duration_since(last_time).as_secs_f32();
        if dt > self.config.max_time_secs {
            self.last_pose = Some(pose);
            self.last_time = Some(now);
            self.passed_count += 1;
            return true;
        }

        // Check distance threshold
        let dx = pose.x - last_pose.x;
        let dy = pose.y - last_pose.y;
        let dist = (dx * dx + dy * dy).sqrt();
        if dist > self.config.max_distance {
            self.last_pose = Some(pose);
            self.last_time = Some(now);
            self.passed_count += 1;
            return true;
        }

        // Check angle threshold
        let dtheta = normalize_angle(pose.theta - last_pose.theta).abs();
        if dtheta > self.config.max_angle {
            self.last_pose = Some(pose);
            self.last_time = Some(now);
            self.passed_count += 1;
            return true;
        }

        // All thresholds within limits, filter this scan
        self.filtered_count += 1;
        false
    }

    /// Reset the filter state.
    ///
    /// Call this when starting a new exploration session.
    pub fn reset(&mut self) {
        self.last_pose = None;
        self.last_time = None;
        self.passed_count = 0;
        self.filtered_count = 0;
    }

    /// Get the number of scans that passed the filter.
    pub fn passed_count(&self) -> u64 {
        self.passed_count
    }

    /// Get the number of scans that were filtered out.
    pub fn filtered_count(&self) -> u64 {
        self.filtered_count
    }

    /// Get the filter efficiency (fraction of scans filtered out).
    pub fn filter_efficiency(&self) -> f32 {
        let total = self.passed_count + self.filtered_count;
        if total == 0 {
            0.0
        } else {
            self.filtered_count as f32 / total as f32
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &MotionFilterConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread::sleep;
    use std::time::Duration;

    #[test]
    fn test_first_scan_always_passes() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        assert!(filter.should_process(pose));
        assert_eq!(filter.passed_count(), 1);
        assert_eq!(filter.filtered_count(), 0);
    }

    #[test]
    fn test_stationary_robot_filtered() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());
        let pose = Pose2D::new(1.0, 2.0, 0.5);

        // First scan passes
        assert!(filter.should_process(pose));

        // Same pose should be filtered
        assert!(!filter.should_process(pose));
        assert!(!filter.should_process(pose));

        assert_eq!(filter.passed_count(), 1);
        assert_eq!(filter.filtered_count(), 2);
    }

    #[test]
    fn test_distance_threshold() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());

        // First scan
        assert!(filter.should_process(Pose2D::new(0.0, 0.0, 0.0)));

        // Small movement (< 0.2m) - filtered
        assert!(!filter.should_process(Pose2D::new(0.1, 0.1, 0.0)));

        // Large movement (> 0.2m) - passes
        assert!(filter.should_process(Pose2D::new(0.5, 0.0, 0.0)));
    }

    #[test]
    fn test_angle_threshold() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());

        // First scan
        assert!(filter.should_process(Pose2D::new(0.0, 0.0, 0.0)));

        // Small rotation (< 0.17 rad) - filtered
        assert!(!filter.should_process(Pose2D::new(0.0, 0.0, 0.1)));

        // Large rotation (> 0.17 rad) - passes
        assert!(filter.should_process(Pose2D::new(0.0, 0.0, 0.5)));
    }

    #[test]
    fn test_time_threshold() {
        let config = MotionFilterConfig {
            max_time_secs: 0.05, // 50ms for fast test
            ..Default::default()
        };
        let mut filter = MotionFilter::new(config);

        // First scan
        assert!(filter.should_process(Pose2D::new(0.0, 0.0, 0.0)));

        // Same pose immediately - filtered
        assert!(!filter.should_process(Pose2D::new(0.0, 0.0, 0.0)));

        // Wait for time threshold
        sleep(Duration::from_millis(60));

        // Same pose after timeout - passes
        assert!(filter.should_process(Pose2D::new(0.0, 0.0, 0.0)));
    }

    #[test]
    fn test_disabled_filter() {
        let mut filter = MotionFilter::new(MotionFilterConfig::disabled());
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        // All scans should pass when disabled
        assert!(filter.should_process(pose));
        assert!(filter.should_process(pose));
        assert!(filter.should_process(pose));

        assert_eq!(filter.passed_count(), 3);
        assert_eq!(filter.filtered_count(), 0);
    }

    #[test]
    fn test_reset() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());

        filter.should_process(Pose2D::new(0.0, 0.0, 0.0));
        filter.should_process(Pose2D::new(0.0, 0.0, 0.0));

        filter.reset();

        assert_eq!(filter.passed_count(), 0);
        assert_eq!(filter.filtered_count(), 0);

        // After reset, first scan passes again
        assert!(filter.should_process(Pose2D::new(0.0, 0.0, 0.0)));
    }

    #[test]
    fn test_filter_efficiency() {
        let mut filter = MotionFilter::new(MotionFilterConfig::default());

        filter.should_process(Pose2D::new(0.0, 0.0, 0.0)); // pass
        filter.should_process(Pose2D::new(0.0, 0.0, 0.0)); // filter
        filter.should_process(Pose2D::new(0.0, 0.0, 0.0)); // filter
        filter.should_process(Pose2D::new(1.0, 0.0, 0.0)); // pass (distance)

        assert_eq!(filter.passed_count(), 2);
        assert_eq!(filter.filtered_count(), 2);
        assert!((filter.filter_efficiency() - 0.5).abs() < 0.001);
    }
}
