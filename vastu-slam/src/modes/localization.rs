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

use crate::core::{LidarScan, Pose2D};
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

    /// Raw scan match result for detailed diagnostics.
    pub match_result: ScanMatchResult,
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
}

impl Localizer {
    /// Create a new localizer with a known map
    pub fn new(map: GridStorage, config: LocalizerConfig) -> Self {
        let matcher = CorrelativeMatcher::new(config.matcher.clone());

        Self {
            map,
            matcher,
            config,
            pose: Pose2D::default(),
            initialized: false,
        }
    }

    /// Set the initial pose estimate
    pub fn set_initial_pose(&mut self, pose: Pose2D) {
        self.pose = pose;
        self.initialized = true;
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

    /// Localize using a lidar scan
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
