//! Loop closure validation using geometric verification.
//!
//! Validates loop closure candidates by checking if the proposed
//! transformation produces a good scan-to-map alignment.

use serde::{Deserialize, Serialize};

use crate::core::{LidarScan, Pose2D};
use crate::grid::GridStorage;
use crate::matching::{CorrelativeMatcher, CorrelativeMatcherConfig};

use super::LoopClosure;

/// Configuration for loop closure validation
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoopValidatorConfig {
    /// Minimum match score to accept a loop closure
    #[serde(default = "default_min_score")]
    pub min_score: f32,

    /// Maximum position error between predicted and matched pose (meters)
    #[serde(default = "default_max_position_error")]
    pub max_position_error: f32,

    /// Maximum angular error between predicted and matched pose (radians)
    #[serde(default = "default_max_angular_error")]
    pub max_angular_error: f32,

    /// Minimum inlier ratio for RANSAC-style validation
    #[serde(default = "default_min_inlier_ratio")]
    pub min_inlier_ratio: f32,

    /// Scan matcher configuration for validation
    #[serde(default)]
    pub matcher: CorrelativeMatcherConfig,
}

fn default_min_score() -> f32 {
    0.6
}
fn default_max_position_error() -> f32 {
    0.3
}
fn default_max_angular_error() -> f32 {
    0.2
}
fn default_min_inlier_ratio() -> f32 {
    0.7
}

impl Default for LoopValidatorConfig {
    fn default() -> Self {
        Self {
            min_score: default_min_score(),
            max_position_error: default_max_position_error(),
            max_angular_error: default_max_angular_error(),
            min_inlier_ratio: default_min_inlier_ratio(),
            matcher: CorrelativeMatcherConfig::default(),
        }
    }
}

/// Result of loop closure validation
#[derive(Clone, Debug)]
pub struct ValidationResult {
    /// Whether the loop closure is valid
    pub valid: bool,

    /// Match score from scan matching
    pub score: f32,

    /// Position error between candidate and verified pose
    pub position_error: f32,

    /// Angular error between candidate and verified pose
    pub angular_error: f32,

    /// Refined pose from scan matching
    pub refined_pose: Pose2D,

    /// Reason for rejection (if not valid)
    pub rejection_reason: Option<RejectionReason>,
}

/// Reasons a loop closure candidate may be rejected
#[derive(Clone, Debug)]
pub enum RejectionReason {
    /// Match score too low
    LowScore(f32),
    /// Position error too large
    PositionError(f32),
    /// Angular error too large
    AngularError(f32),
    /// Not enough scan points matched
    LowInlierRatio(f32),
}

/// Validates loop closure candidates using geometric verification.
///
/// Performs scan matching at the proposed loop closure pose and checks
/// if the alignment quality meets the configured thresholds.
pub struct LoopValidator {
    /// Configuration
    config: LoopValidatorConfig,

    /// Scan matcher for verification
    matcher: CorrelativeMatcher,
}

impl LoopValidator {
    /// Create a new loop closure validator
    pub fn new(config: LoopValidatorConfig) -> Self {
        let matcher = CorrelativeMatcher::new(config.matcher.clone());

        Self { config, matcher }
    }

    /// Validate a loop closure candidate.
    ///
    /// # Arguments
    /// * `candidate` - The loop closure candidate to validate
    /// * `scan` - The lidar scan at the current pose
    /// * `map` - The occupancy grid map
    ///
    /// # Returns
    /// Validation result with refined pose if successful
    pub fn validate(
        &self,
        candidate: &LoopClosure,
        scan: &LidarScan,
        map: &GridStorage,
    ) -> ValidationResult {
        // Use the candidate's relative pose as initial guess
        let initial_pose = candidate.relative_pose;

        // Perform scan matching at the proposed pose
        let match_result = self.matcher.match_scan(scan, initial_pose, map);

        // Check score threshold
        if match_result.score < self.config.min_score {
            return ValidationResult {
                valid: false,
                score: match_result.score,
                position_error: 0.0,
                angular_error: 0.0,
                refined_pose: initial_pose,
                rejection_reason: Some(RejectionReason::LowScore(match_result.score)),
            };
        }

        // Compute position error
        let dx = match_result.pose.x - initial_pose.x;
        let dy = match_result.pose.y - initial_pose.y;
        let position_error = (dx * dx + dy * dy).sqrt();

        if position_error > self.config.max_position_error {
            return ValidationResult {
                valid: false,
                score: match_result.score,
                position_error,
                angular_error: 0.0,
                refined_pose: match_result.pose,
                rejection_reason: Some(RejectionReason::PositionError(position_error)),
            };
        }

        // Compute angular error
        let angular_error = (match_result.pose.theta - initial_pose.theta).abs();
        let angular_error = if angular_error > std::f32::consts::PI {
            2.0 * std::f32::consts::PI - angular_error
        } else {
            angular_error
        };

        if angular_error > self.config.max_angular_error {
            return ValidationResult {
                valid: false,
                score: match_result.score,
                position_error,
                angular_error,
                refined_pose: match_result.pose,
                rejection_reason: Some(RejectionReason::AngularError(angular_error)),
            };
        }

        // All checks passed
        ValidationResult {
            valid: true,
            score: match_result.score,
            position_error,
            angular_error,
            refined_pose: match_result.pose,
            rejection_reason: None,
        }
    }

    /// Quick validation using only score threshold.
    ///
    /// Faster than full validation, useful for filtering candidates.
    pub fn quick_validate(&self, scan: &LidarScan, pose: Pose2D, map: &GridStorage) -> bool {
        let match_result = self.matcher.match_scan(scan, pose, map);
        match_result.converged && match_result.score >= self.config.min_score
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validator_config_default() {
        let config = LoopValidatorConfig::default();
        assert!(config.min_score > 0.0);
        assert!(config.max_position_error > 0.0);
    }

    #[test]
    fn test_validator_creation() {
        let config = LoopValidatorConfig::default();
        let _validator = LoopValidator::new(config);
    }
}
