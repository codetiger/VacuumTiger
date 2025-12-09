//! Scan match result validation.
//!
//! Provides utilities for validating scan match results against expected
//! odometry deltas. This helps reject bad matches that would corrupt the map.

use super::ScanMatchResult;
use crate::core::types::Pose2D;

/// Configuration for scan match validation.
#[derive(Debug, Clone)]
pub struct ScanMatchValidatorConfig {
    /// Minimum acceptable match score (0.0-1.0).
    pub min_score: f32,

    /// Maximum acceptable correction distance (meters).
    ///
    /// The correction is the difference between the scan match result
    /// and the expected odometry delta.
    pub max_correction_distance: f32,

    /// Maximum acceptable correction angle (radians).
    pub max_correction_angle: f32,
}

impl Default for ScanMatchValidatorConfig {
    fn default() -> Self {
        Self {
            min_score: 0.3,
            max_correction_distance: 0.15, // 15cm
            max_correction_angle: 0.35,    // ~20 degrees
        }
    }
}

impl ScanMatchValidatorConfig {
    /// Create a strict configuration for high-quality matching.
    pub fn strict() -> Self {
        Self {
            min_score: 0.5,
            max_correction_distance: 0.08, // 8cm
            max_correction_angle: 0.15,    // ~8.5 degrees
        }
    }

    /// Create a lenient configuration for challenging environments.
    pub fn lenient() -> Self {
        Self {
            min_score: 0.2,
            max_correction_distance: 0.25, // 25cm
            max_correction_angle: 0.5,     // ~28 degrees
        }
    }
}

/// Reason for rejecting a scan match.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectionReason {
    /// Match score too low.
    LowScore,
    /// Translation correction exceeds threshold.
    ExcessiveTranslation,
    /// Rotation correction exceeds threshold.
    ExcessiveRotation,
    /// Matcher did not converge.
    NotConverged,
}

impl std::fmt::Display for RejectionReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LowScore => write!(f, "low score"),
            Self::ExcessiveTranslation => write!(f, "excessive translation correction"),
            Self::ExcessiveRotation => write!(f, "excessive rotation correction"),
            Self::NotConverged => write!(f, "not converged"),
        }
    }
}

/// Result of scan match validation.
#[derive(Debug, Clone)]
pub struct ValidationResult {
    /// Whether the match passes validation.
    pub is_valid: bool,

    /// Distance correction applied (meters).
    ///
    /// This is the Euclidean distance between the expected delta
    /// and the actual match result.
    pub correction_distance: f32,

    /// Angular correction applied (radians).
    ///
    /// This is the absolute difference between expected and actual rotation.
    pub correction_angle: f32,

    /// The match score from the scan matcher.
    pub score: f32,

    /// Reason for rejection (if invalid).
    pub rejection_reason: Option<RejectionReason>,
}

impl ValidationResult {
    /// Create a valid result.
    pub fn valid(correction_distance: f32, correction_angle: f32, score: f32) -> Self {
        Self {
            is_valid: true,
            correction_distance,
            correction_angle,
            score,
            rejection_reason: None,
        }
    }

    /// Create an invalid result with the given rejection reason.
    pub fn invalid(
        reason: RejectionReason,
        correction_distance: f32,
        correction_angle: f32,
        score: f32,
    ) -> Self {
        Self {
            is_valid: false,
            correction_distance,
            correction_angle,
            score,
            rejection_reason: Some(reason),
        }
    }
}

/// Validates scan match results against expected odometry.
///
/// # Example
///
/// ```
/// use dhruva_slam::algorithms::matching::{ScanMatchValidator, ScanMatchValidatorConfig, ScanMatchResult};
/// use dhruva_slam::Pose2D;
///
/// let validator = ScanMatchValidator::default();
///
/// // Validate a match result against expected odometry
/// let match_result = ScanMatchResult::success(
///     Pose2D::new(0.1, 0.08, 0.05),
///     0.8,
///     15,
///     0.001,
/// );
/// let expected_delta = Pose2D::new(0.1, 0.08, 0.05);
///
/// let validation = validator.validate(&match_result, &expected_delta);
/// assert!(validation.is_valid);
/// ```
#[derive(Debug, Clone)]
pub struct ScanMatchValidator {
    config: ScanMatchValidatorConfig,
}

impl ScanMatchValidator {
    /// Create a new validator with the given configuration.
    pub fn new(config: ScanMatchValidatorConfig) -> Self {
        Self { config }
    }

    /// Get the current configuration.
    pub fn config(&self) -> &ScanMatchValidatorConfig {
        &self.config
    }

    /// Validate a scan match result against expected odometry delta.
    ///
    /// # Arguments
    ///
    /// * `result` - The scan match result to validate
    /// * `expected_delta` - The expected pose change from odometry
    ///
    /// # Returns
    ///
    /// A `ValidationResult` indicating whether the match is acceptable
    /// and providing details about the correction applied.
    pub fn validate(&self, result: &ScanMatchResult, expected_delta: &Pose2D) -> ValidationResult {
        // Check convergence first
        if !result.converged {
            return ValidationResult::invalid(
                RejectionReason::NotConverged,
                0.0,
                0.0,
                result.score,
            );
        }

        // Compute corrections
        let dx = result.transform.x - expected_delta.x;
        let dy = result.transform.y - expected_delta.y;
        let correction_distance = (dx * dx + dy * dy).sqrt();

        // Use angle_diff for proper wraparound handling
        let correction_angle =
            crate::core::math::angle_diff(result.transform.theta, expected_delta.theta).abs();

        // Check rejection reasons in order of severity
        if result.score < self.config.min_score {
            return ValidationResult::invalid(
                RejectionReason::LowScore,
                correction_distance,
                correction_angle,
                result.score,
            );
        }

        if correction_distance > self.config.max_correction_distance {
            return ValidationResult::invalid(
                RejectionReason::ExcessiveTranslation,
                correction_distance,
                correction_angle,
                result.score,
            );
        }

        if correction_angle > self.config.max_correction_angle {
            return ValidationResult::invalid(
                RejectionReason::ExcessiveRotation,
                correction_distance,
                correction_angle,
                result.score,
            );
        }

        ValidationResult::valid(correction_distance, correction_angle, result.score)
    }

    /// Quick check if a match result is acceptable.
    ///
    /// This is a convenience method that returns only a boolean.
    /// Use `validate()` for detailed rejection information.
    pub fn is_valid(&self, result: &ScanMatchResult, expected_delta: &Pose2D) -> bool {
        self.validate(result, expected_delta).is_valid
    }
}

impl Default for ScanMatchValidator {
    fn default() -> Self {
        Self::new(ScanMatchValidatorConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Covariance2D;
    use std::f32::consts::PI;

    fn make_result(transform: Pose2D, score: f32, converged: bool) -> ScanMatchResult {
        ScanMatchResult {
            transform,
            covariance: Covariance2D::zero(),
            score,
            converged,
            iterations: 10,
            mse: 0.001,
        }
    }

    #[test]
    fn test_validator_accepts_good_match() {
        let validator = ScanMatchValidator::default();
        let result = make_result(Pose2D::new(0.1, 0.08, 0.05), 0.8, true);
        let expected = Pose2D::new(0.1, 0.08, 0.05);

        let validation = validator.validate(&result, &expected);
        assert!(validation.is_valid);
        assert!(validation.rejection_reason.is_none());
        assert!(validation.correction_distance < 0.001);
        assert!(validation.correction_angle < 0.001);
    }

    #[test]
    fn test_validator_accepts_small_correction() {
        let validator = ScanMatchValidator::default();
        let result = make_result(Pose2D::new(0.12, 0.10, 0.07), 0.7, true);
        let expected = Pose2D::new(0.10, 0.08, 0.05);

        let validation = validator.validate(&result, &expected);
        assert!(validation.is_valid);
        // Correction: sqrt(0.02² + 0.02²) ≈ 0.028m
        assert!(validation.correction_distance < 0.05);
    }

    #[test]
    fn test_validator_rejects_low_score() {
        let validator = ScanMatchValidator::default();
        let result = make_result(Pose2D::new(0.1, 0.08, 0.05), 0.1, true);
        let expected = Pose2D::new(0.1, 0.08, 0.05);

        let validation = validator.validate(&result, &expected);
        assert!(!validation.is_valid);
        assert_eq!(validation.rejection_reason, Some(RejectionReason::LowScore));
    }

    #[test]
    fn test_validator_rejects_excessive_translation() {
        let validator = ScanMatchValidator::default();
        let result = make_result(Pose2D::new(0.5, 0.5, 0.05), 0.8, true);
        let expected = Pose2D::new(0.1, 0.1, 0.05);

        let validation = validator.validate(&result, &expected);
        assert!(!validation.is_valid);
        assert_eq!(
            validation.rejection_reason,
            Some(RejectionReason::ExcessiveTranslation)
        );
    }

    #[test]
    fn test_validator_rejects_excessive_rotation() {
        let validator = ScanMatchValidator::default();
        let result = make_result(Pose2D::new(0.1, 0.08, 0.8), 0.8, true);
        let expected = Pose2D::new(0.1, 0.08, 0.05);

        let validation = validator.validate(&result, &expected);
        assert!(!validation.is_valid);
        assert_eq!(
            validation.rejection_reason,
            Some(RejectionReason::ExcessiveRotation)
        );
    }

    #[test]
    fn test_validator_rejects_not_converged() {
        let validator = ScanMatchValidator::default();
        let result = make_result(Pose2D::new(0.1, 0.08, 0.05), 0.8, false);
        let expected = Pose2D::new(0.1, 0.08, 0.05);

        let validation = validator.validate(&result, &expected);
        assert!(!validation.is_valid);
        assert_eq!(
            validation.rejection_reason,
            Some(RejectionReason::NotConverged)
        );
    }

    #[test]
    fn test_validator_strict_config() {
        let validator = ScanMatchValidator::new(ScanMatchValidatorConfig::strict());

        // This would pass default but fail strict
        let result = make_result(Pose2D::new(0.2, 0.1, 0.1), 0.4, true);
        let expected = Pose2D::new(0.1, 0.08, 0.05);

        let validation = validator.validate(&result, &expected);
        assert!(!validation.is_valid);
    }

    #[test]
    fn test_validator_lenient_config() {
        let validator = ScanMatchValidator::new(ScanMatchValidatorConfig::lenient());

        // This would fail default but pass lenient
        let result = make_result(Pose2D::new(0.3, 0.2, 0.15), 0.25, true);
        let expected = Pose2D::new(0.1, 0.08, 0.05);

        let validation = validator.validate(&result, &expected);
        assert!(validation.is_valid);
    }

    #[test]
    fn test_validator_angle_wraparound() {
        let validator = ScanMatchValidator::default();

        // Test near PI boundary
        let result = make_result(Pose2D::new(0.1, 0.0, PI - 0.05), 0.8, true);
        let expected = Pose2D::new(0.1, 0.0, -PI + 0.05);

        let validation = validator.validate(&result, &expected);
        // The angle difference should be ~0.1 rad, not ~2*PI
        assert!(validation.correction_angle < 0.2);
        assert!(validation.is_valid);
    }

    #[test]
    fn test_is_valid_shortcut() {
        let validator = ScanMatchValidator::default();
        let result = make_result(Pose2D::new(0.1, 0.08, 0.05), 0.8, true);
        let expected = Pose2D::new(0.1, 0.08, 0.05);

        assert!(validator.is_valid(&result, &expected));
    }

    #[test]
    fn test_rejection_reason_display() {
        assert_eq!(format!("{}", RejectionReason::LowScore), "low score");
        assert_eq!(
            format!("{}", RejectionReason::ExcessiveTranslation),
            "excessive translation correction"
        );
        assert_eq!(
            format!("{}", RejectionReason::ExcessiveRotation),
            "excessive rotation correction"
        );
        assert_eq!(
            format!("{}", RejectionReason::NotConverged),
            "not converged"
        );
    }
}
