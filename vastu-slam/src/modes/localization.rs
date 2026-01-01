//! Localization-only mode for known maps.
//!
//! This mode uses scan matching to localize on a pre-built map
//! without modifying the map itself. Ideal for navigation in
//! previously mapped environments.
//!
//! # Pipeline
//!
//! ```text
//! LidarScan + Odometry
//!       │
//!       ▼
//! ┌─────────────────┐
//! │ Odom Prediction │  predicted = current ⊕ delta
//! └────────┬────────┘
//!          │
//!          ▼
//! ┌─────────────────┐
//! │  Scan Matching  │  Match against known map
//! └────────┬────────┘
//!          │
//!          ▼
//! ┌─────────────────┐
//! │ Deviation Check │  Is result within bounds?
//! └────────┬────────┘
//!          │
//!    ┌─────┴─────┐
//!    ▼           ▼
//! Accept     Fallback
//! (matched)  (odometry)
//! ```
//!
//! # Deviation Bounds
//!
//! The localizer rejects matches that deviate too far from odometry prediction:
//! - Linear: ≤ 0.5m (default)
//! - Angular: ≤ 0.5 rad (default)
//!
//! This prevents jumping to incorrect positions when the scan matcher finds
//! a local minimum far from the true robot position.

use serde::{Deserialize, Serialize};

use crate::core::{
    ImuMeasurement, LidarScan, MotionFilter, MotionFilterConfig, Pose2D, PoseExtrapolator,
    PoseExtrapolatorConfig,
};
use crate::grid::GridStorage;
use crate::matching::{CorrelativeMatcher, CorrelativeMatcherConfig, ScanMatchResult};

/// Configuration for the localizer.
///
/// Controls scan matching behavior and acceptance thresholds.
///
/// # Example
///
/// ```rust
/// use vastu_slam::modes::LocalizerConfig;
///
/// let config = LocalizerConfig {
///     min_score: 0.6,                // Require higher confidence
///     max_linear_deviation: 0.3,     // Tighter position bound
///     max_angular_deviation: 0.3,    // Tighter angle bound
///     ..Default::default()
/// };
/// ```
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LocalizerConfig {
    /// Scan matcher configuration.
    ///
    /// For localization, smaller search windows are typically sufficient
    /// since we have good odometry prediction.
    pub matcher: CorrelativeMatcherConfig,

    /// Minimum score to accept a match (0.0 - 1.0).
    ///
    /// Lower values accept more matches but may include false positives.
    /// Default: 0.5
    #[serde(default = "default_min_score")]
    pub min_score: f32,

    /// Maximum linear distance from prediction to accept match (meters).
    ///
    /// If the matched pose is further than this from odometry prediction,
    /// the match is rejected. Prevents jumping to incorrect positions.
    /// Default: 0.5m
    #[serde(default = "default_max_linear_deviation")]
    pub max_linear_deviation: f32,

    /// Maximum angular distance from prediction to accept match (radians).
    ///
    /// If the matched rotation differs by more than this from odometry
    /// prediction, the match is rejected.
    /// Default: 0.5 rad (~28.6°)
    #[serde(default = "default_max_angular_deviation")]
    pub max_angular_deviation: f32,

    /// Optional pose extrapolator configuration for IMU fusion.
    ///
    /// When set, enables Cartographer-style pose extrapolation with
    /// odometry (primary) and IMU (secondary) fusion.
    #[serde(default)]
    pub pose_extrapolator: Option<PoseExtrapolatorConfig>,

    /// Optional motion filter configuration for scan insertion throttling.
    ///
    /// When set, only processes scans when the robot has moved enough
    /// (distance, rotation, or time since last scan).
    #[serde(default)]
    pub motion_filter: Option<MotionFilterConfig>,
}

fn default_min_score() -> f32 {
    0.5
}
fn default_max_linear_deviation() -> f32 {
    0.5
}
fn default_max_angular_deviation() -> f32 {
    0.5
}

impl Default for LocalizerConfig {
    fn default() -> Self {
        Self {
            matcher: CorrelativeMatcherConfig::default(),
            min_score: default_min_score(),
            max_linear_deviation: default_max_linear_deviation(),
            max_angular_deviation: default_max_angular_deviation(),
            pose_extrapolator: None,
            motion_filter: None,
        }
    }
}

impl LocalizerConfig {
    /// Create a config with motion filtering enabled (Cartographer-style).
    ///
    /// Uses default values for pose extrapolation and motion filtering:
    /// - IMU rotation weight: 0.3 (70% odometry, 30% IMU)
    /// - Motion filter: insert when moved 20cm, rotated 2°, or 5s elapsed
    pub fn with_motion_filtering() -> Self {
        Self {
            pose_extrapolator: Some(PoseExtrapolatorConfig::default()),
            motion_filter: Some(MotionFilterConfig::default()),
            ..Default::default()
        }
    }
}

/// Result of a localization attempt.
///
/// Contains the estimated pose and diagnostic information about match quality.
///
/// # Interpreting Results
///
/// ```text
/// converged = match_result.converged && score >= min_score && within_bounds
/// ```
///
/// - `converged=true`: Match accepted, `pose` is scan-corrected
/// - `converged=false`: Match rejected, `pose` is odometry fallback
/// - `skipped=true`: Scan was skipped by motion filter (not enough motion)
#[derive(Clone, Debug)]
pub struct LocalizationResult {
    /// Estimated robot pose in world frame.
    ///
    /// If converged, this is the scan-matched pose.
    /// If not converged, this is the odometry-predicted pose.
    pub pose: Pose2D,

    /// Match score from scan matcher (0.0 - 1.0).
    ///
    /// Higher values indicate better alignment with the map.
    /// Typical good matches: 0.6 - 0.9
    /// Returns 0.0 if skipped.
    pub score: f32,

    /// Whether the localization converged successfully.
    ///
    /// True only if:
    /// - Scan matcher converged
    /// - Score >= min_score threshold
    /// - Deviation from prediction within bounds
    pub converged: bool,

    /// Whether the match passed deviation checks.
    ///
    /// False if matched pose is too far from odometry prediction,
    /// which may indicate a false match or robot kidnapping.
    pub within_bounds: bool,

    /// Whether the scan was skipped by the motion filter.
    ///
    /// True if the robot hasn't moved enough since the last scan
    /// to warrant processing this scan.
    pub skipped: bool,

    /// Raw scan match result for detailed diagnostics.
    /// Contains a default value if skipped.
    pub match_result: ScanMatchResult,
}

impl LocalizationResult {
    /// Create a result indicating the scan was skipped due to motion filter.
    pub fn skipped(pose: Pose2D) -> Self {
        Self {
            pose,
            score: 0.0,
            converged: false,
            within_bounds: true,
            skipped: true,
            match_result: ScanMatchResult::failed(pose),
        }
    }
}

/// Localizer for known maps.
///
/// Uses scan matching to estimate robot pose without modifying the map.
/// The map remains fixed throughout operation.
///
/// # Usage
///
/// ```rust,ignore
/// use vastu_slam::modes::{Localizer, LocalizerConfig};
/// use vastu_slam::core::Pose2D;
///
/// let mut localizer = Localizer::new(map, LocalizerConfig::default());
///
/// // Must set initial pose before first localization
/// localizer.set_initial_pose(Pose2D::new(0.0, 0.0, 0.0));
///
/// // Main loop
/// let result = localizer.localize(&scan, Some(odom_delta));
/// ```
///
/// # State Machine
///
/// ```text
/// ┌──────────────┐  set_initial_pose()  ┌─────────────┐
/// │ Uninitialized │ ─────────────────▶  │ Initialized │
/// └──────────────┘                      └──────┬──────┘
///                                              │
///                                              │ localize()
///                                              ▼
///                                       ┌─────────────┐
///                                       │   Running   │◀─┐
///                                       └──────┬──────┘  │
///                                              │         │
///                                              └─────────┘
/// ```
pub struct Localizer {
    /// The known map to localize against (immutable after creation).
    map: GridStorage,

    /// Scan matcher for pose estimation.
    matcher: CorrelativeMatcher,

    /// Configuration parameters.
    config: LocalizerConfig,

    /// Current pose estimate in world frame.
    pose: Pose2D,

    /// Whether we have a valid initial pose.
    initialized: bool,

    /// Optional pose extrapolator for IMU fusion (Cartographer-style).
    pose_extrapolator: Option<PoseExtrapolator>,

    /// Optional motion filter for scan insertion throttling.
    motion_filter: Option<MotionFilter>,
}

impl Localizer {
    /// Create a new localizer with a known map
    pub fn new(map: GridStorage, config: LocalizerConfig) -> Self {
        let matcher = CorrelativeMatcher::new(config.matcher.clone());

        // Create optional motion filtering components
        let pose_extrapolator = config
            .pose_extrapolator
            .as_ref()
            .map(|c| PoseExtrapolator::new(c.clone()));

        let motion_filter = config
            .motion_filter
            .as_ref()
            .map(|c| MotionFilter::new(c.clone()));

        Self {
            map,
            matcher,
            config,
            pose: Pose2D::default(),
            initialized: false,
            pose_extrapolator,
            motion_filter,
        }
    }

    /// Set the initial pose estimate
    pub fn set_initial_pose(&mut self, pose: Pose2D) {
        self.pose = pose;
        self.initialized = true;

        // Initialize pose extrapolator if present
        if let Some(ref mut extrapolator) = self.pose_extrapolator {
            extrapolator.set_initial_pose(pose, 0);
        }
    }

    /// Set the initial pose estimate with timestamp
    pub fn set_initial_pose_with_timestamp(&mut self, pose: Pose2D, timestamp_us: u64) {
        self.pose = pose;
        self.initialized = true;

        // Initialize pose extrapolator if present
        if let Some(ref mut extrapolator) = self.pose_extrapolator {
            extrapolator.set_initial_pose(pose, timestamp_us);
        }
    }

    /// Get current pose estimate
    pub fn pose(&self) -> Pose2D {
        self.pose
    }

    /// Check if localizer has been initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get reference to the map
    pub fn map(&self) -> &GridStorage {
        &self.map
    }

    /// Check if motion filtering is enabled
    pub fn has_motion_filtering(&self) -> bool {
        self.pose_extrapolator.is_some() || self.motion_filter.is_some()
    }

    /// Get a reference to the pose extrapolator (if enabled)
    pub fn pose_extrapolator(&self) -> Option<&PoseExtrapolator> {
        self.pose_extrapolator.as_ref()
    }

    /// Get a mutable reference to the pose extrapolator (if enabled)
    pub fn pose_extrapolator_mut(&mut self) -> Option<&mut PoseExtrapolator> {
        self.pose_extrapolator.as_mut()
    }

    /// Get a reference to the motion filter (if enabled)
    pub fn motion_filter(&self) -> Option<&MotionFilter> {
        self.motion_filter.as_ref()
    }

    /// Feed an IMU measurement to the pose extrapolator.
    ///
    /// Call this at high rate (~110 Hz) to keep the IMU tracker updated.
    /// Does nothing if pose extrapolator is not configured.
    pub fn add_imu(&mut self, imu: &ImuMeasurement) {
        if let Some(ref mut extrapolator) = self.pose_extrapolator {
            extrapolator.add_imu(imu);
        }
    }

    /// Localize using a lidar scan (basic version, backward compatible)
    ///
    /// # Arguments
    /// * `scan` - The lidar scan to match
    /// * `odom_delta` - Optional odometry delta since last localization
    ///
    /// # Returns
    /// Localization result with pose estimate and confidence
    pub fn localize(&mut self, scan: &LidarScan, odom_delta: Option<Pose2D>) -> LocalizationResult {
        // Apply odometry prediction if available
        let predicted_pose = if let Some(delta) = odom_delta {
            self.pose.compose(&delta)
        } else {
            self.pose
        };

        // Match scan against map
        let match_result = self.matcher.match_scan(scan, predicted_pose, &self.map);

        // Check if match is within acceptable bounds
        let within_bounds = self.check_deviation(&predicted_pose, &match_result.pose);

        // Determine if we accept this match
        let converged =
            match_result.converged && match_result.score >= self.config.min_score && within_bounds;

        // Update pose if converged
        if converged {
            self.pose = match_result.pose;
            self.initialized = true;
        } else if odom_delta.is_some() {
            // Fall back to odometry prediction
            self.pose = predicted_pose;
        }

        LocalizationResult {
            pose: self.pose,
            score: match_result.score,
            converged,
            within_bounds,
            skipped: false,
            match_result,
        }
    }

    /// Localize using a lidar scan with IMU fusion and motion filtering.
    ///
    /// This is the full-featured version that uses:
    /// - Pose extrapolation with IMU fusion for better prediction
    /// - Motion filtering to skip redundant scans
    ///
    /// # Arguments
    /// * `scan` - The lidar scan to match
    /// * `odom_delta` - Optional odometry delta since last localization
    /// * `timestamp_us` - Timestamp in microseconds
    ///
    /// # Returns
    /// Localization result with pose estimate and confidence.
    /// Returns `skipped=true` if motion filter rejected the scan.
    ///
    /// # Note
    /// Call `add_imu()` at high rate (~110 Hz) before calling this method
    /// to keep the IMU tracker updated.
    pub fn localize_with_timestamp(
        &mut self,
        scan: &LidarScan,
        odom_delta: Option<Pose2D>,
        timestamp_us: u64,
    ) -> LocalizationResult {
        // Feed odometry to pose extrapolator if available
        if let Some(ref mut extrapolator) = self.pose_extrapolator
            && let Some(delta) = odom_delta
        {
            extrapolator.add_odometry(delta, timestamp_us);
        }

        // Get predicted pose (uses filtered motion if extrapolator is available)
        let predicted_pose = if let Some(ref extrapolator) = self.pose_extrapolator {
            extrapolator.extrapolate_pose(timestamp_us)
        } else if let Some(delta) = odom_delta {
            self.pose.compose(&delta)
        } else {
            self.pose
        };

        // Check motion filter (should we process this scan?)
        if let Some(ref filter) = self.motion_filter
            && !filter.should_insert(predicted_pose, timestamp_us)
        {
            // Update pose with prediction even if skipped
            self.pose = predicted_pose;
            return LocalizationResult::skipped(predicted_pose);
        }

        // Match scan against map
        let match_result = self.matcher.match_scan(scan, predicted_pose, &self.map);

        // Check if match is within acceptable bounds
        let within_bounds = self.check_deviation(&predicted_pose, &match_result.pose);

        // Determine if we accept this match
        let converged =
            match_result.converged && match_result.score >= self.config.min_score && within_bounds;

        // Update pose if converged
        if converged {
            self.pose = match_result.pose;
            self.initialized = true;

            // Update pose extrapolator with matched pose
            if let Some(ref mut extrapolator) = self.pose_extrapolator {
                extrapolator.add_pose(match_result.pose, timestamp_us);
            }
        } else if odom_delta.is_some() {
            // Fall back to prediction
            self.pose = predicted_pose;
        }

        // Update motion filter
        if let Some(ref mut filter) = self.motion_filter {
            filter.accept(self.pose, timestamp_us);
        }

        LocalizationResult {
            pose: self.pose,
            score: match_result.score,
            converged,
            within_bounds,
            skipped: false,
            match_result,
        }
    }

    /// Check if matched pose is within acceptable deviation from prediction
    fn check_deviation(&self, predicted: &Pose2D, matched: &Pose2D) -> bool {
        let dx = matched.x - predicted.x;
        let dy = matched.y - predicted.y;
        let linear_dist = (dx * dx + dy * dy).sqrt();

        let angular_dist = (matched.theta - predicted.theta).abs();
        let angular_dist = if angular_dist > std::f32::consts::PI {
            2.0 * std::f32::consts::PI - angular_dist
        } else {
            angular_dist
        };

        linear_dist <= self.config.max_linear_deviation
            && angular_dist <= self.config.max_angular_deviation
    }

    /// Reset the localizer state
    pub fn reset(&mut self) {
        self.pose = Pose2D::default();
        self.initialized = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_localizer_config_default() {
        let config = LocalizerConfig::default();
        assert!(config.min_score > 0.0);
        assert!(config.max_linear_deviation > 0.0);
    }

    #[test]
    fn test_localizer_creation() {
        let map = GridStorage::new(100, 100, 0.05, crate::core::WorldPoint::ZERO);
        let config = LocalizerConfig::default();
        let localizer = Localizer::new(map, config);

        assert!(!localizer.is_initialized());
    }

    #[test]
    fn test_set_initial_pose() {
        let map = GridStorage::new(100, 100, 0.05, crate::core::WorldPoint::ZERO);
        let config = LocalizerConfig::default();
        let mut localizer = Localizer::new(map, config);

        let pose = Pose2D::new(1.0, 2.0, 0.5);
        localizer.set_initial_pose(pose);

        assert!(localizer.is_initialized());
        assert!((localizer.pose().x - 1.0).abs() < 1e-6);
    }
}
