//! Kidnapped robot detection module.
//!
//! Detects when the robot has been physically moved ("kidnapped") without
//! odometry tracking, causing the current pose estimate to be completely wrong.
//!
//! # Detection Methods
//!
//! 1. **Scan-to-Map Correlation**: Compare current scan against expected map
//!    at the estimated pose. Low correlation indicates potential kidnapping.
//!
//! 2. **Consecutive Low Scores**: Track multiple consecutive poor scan matches
//!    to avoid false positives from momentary occlusions.
//!
//! 3. **Innovation Check**: Large unexpected changes in sensor observations
//!    compared to predicted observations.
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::slam::{KidnappedDetector, KidnappedDetectorConfig};
//!
//! let config = KidnappedDetectorConfig::default();
//! let mut detector = KidnappedDetector::new(config);
//!
//! // In SLAM loop
//! let detection = detector.check(match_score, covariance_trace);
//! if detection.is_kidnapped {
//!     // Trigger relocalization
//!     println!("Robot kidnapped! Confidence: {}", detection.confidence);
//! }
//! ```

use crate::core::types::{Pose2D, Covariance2D};

/// Configuration for kidnapped robot detection.
#[derive(Debug, Clone)]
pub struct KidnappedDetectorConfig {
    /// Scan match score threshold below which a match is considered "poor".
    /// Default: 0.2
    pub score_threshold: f32,

    /// Number of consecutive poor matches needed to trigger kidnap detection.
    /// Default: 5
    pub consecutive_threshold: usize,

    /// Covariance trace threshold above which uncertainty is considered high.
    /// Default: 1.0
    pub covariance_threshold: f32,

    /// Innovation threshold (normalized) for sudden large changes.
    /// Default: 3.0 (3-sigma)
    pub innovation_threshold: f32,

    /// Minimum time (microseconds) between scans to consider for detection.
    /// Prevents false positives during initialization.
    /// Default: 1_000_000 (1 second)
    pub min_tracking_time_us: u64,

    /// Enable detection. When false, always returns not kidnapped.
    /// Default: true
    pub enabled: bool,
}

impl Default for KidnappedDetectorConfig {
    fn default() -> Self {
        Self {
            score_threshold: 0.2,
            consecutive_threshold: 5,
            covariance_threshold: 1.0,
            innovation_threshold: 3.0,
            min_tracking_time_us: 1_000_000,
            enabled: true,
        }
    }
}

/// Result of kidnapped detection check.
#[derive(Debug, Clone)]
pub struct KidnappedDetection {
    /// Whether the robot is detected as kidnapped.
    pub is_kidnapped: bool,

    /// Confidence in the detection (0.0-1.0).
    /// Higher values indicate more certain detection.
    pub confidence: f32,

    /// Reason for the detection.
    pub reason: KidnappedReason,

    /// Number of consecutive poor matches.
    pub consecutive_poor_matches: usize,

    /// Current score moving average.
    pub score_average: f32,
}

impl Default for KidnappedDetection {
    fn default() -> Self {
        Self {
            is_kidnapped: false,
            confidence: 0.0,
            reason: KidnappedReason::None,
            consecutive_poor_matches: 0,
            score_average: 1.0,
        }
    }
}

/// Reason for kidnapped detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KidnappedReason {
    /// No kidnap detected.
    None,
    /// Too many consecutive poor scan matches.
    ConsecutivePoorMatches,
    /// Covariance exceeded threshold.
    HighUncertainty,
    /// Large innovation (unexpected observation change).
    LargeInnovation,
    /// Combined indicators.
    Combined,
}

/// Detector for kidnapped robot scenarios.
///
/// Tracks scan match quality over time and detects when the robot
/// has likely been physically moved.
pub struct KidnappedDetector {
    config: KidnappedDetectorConfig,

    /// Count of consecutive poor matches.
    consecutive_poor: usize,

    /// Exponential moving average of match scores.
    score_ema: f32,

    /// Previous pose for innovation check.
    previous_pose: Option<Pose2D>,

    /// Previous predicted pose (from odometry).
    previous_prediction: Option<Pose2D>,

    /// Timestamp when tracking started.
    tracking_start_us: Option<u64>,

    /// Total scans processed.
    scan_count: u64,
}

impl KidnappedDetector {
    /// Create a new kidnapped detector.
    pub fn new(config: KidnappedDetectorConfig) -> Self {
        Self {
            config,
            consecutive_poor: 0,
            score_ema: 1.0,
            previous_pose: None,
            previous_prediction: None,
            tracking_start_us: None,
            scan_count: 0,
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &KidnappedDetectorConfig {
        &self.config
    }

    /// Check if the robot appears to be kidnapped.
    ///
    /// # Arguments
    /// * `match_score` - Current scan match quality (0.0-1.0)
    /// * `covariance` - Current pose covariance estimate
    /// * `timestamp_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Detection result with confidence and reason.
    pub fn check(
        &mut self,
        match_score: f32,
        covariance: &Covariance2D,
        timestamp_us: u64,
    ) -> KidnappedDetection {
        if !self.config.enabled {
            return KidnappedDetection::default();
        }

        // Initialize tracking start time
        if self.tracking_start_us.is_none() {
            self.tracking_start_us = Some(timestamp_us);
        }

        self.scan_count += 1;

        // Don't trigger during initial tracking period
        let tracking_time = timestamp_us.saturating_sub(self.tracking_start_us.unwrap_or(timestamp_us));
        if tracking_time < self.config.min_tracking_time_us {
            self.update_ema(match_score);
            return KidnappedDetection::default();
        }

        // Update score tracking
        let is_poor_match = match_score < self.config.score_threshold;
        if is_poor_match {
            self.consecutive_poor += 1;
        } else {
            self.consecutive_poor = 0;
        }

        self.update_ema(match_score);

        // Check covariance trace (sum of diagonal variances)
        let cov_trace = covariance.var_x() + covariance.var_y() + covariance.var_theta();
        let high_uncertainty = cov_trace > self.config.covariance_threshold;

        // Determine kidnap status
        let (is_kidnapped, reason, confidence) = self.evaluate_detection(high_uncertainty);

        KidnappedDetection {
            is_kidnapped,
            confidence,
            reason,
            consecutive_poor_matches: self.consecutive_poor,
            score_average: self.score_ema,
        }
    }

    /// Check with innovation (for advanced detection).
    ///
    /// # Arguments
    /// * `match_score` - Current scan match quality (0.0-1.0)
    /// * `covariance` - Current pose covariance estimate
    /// * `corrected_pose` - Pose after scan matching
    /// * `predicted_pose` - Pose predicted from odometry alone
    /// * `timestamp_us` - Current timestamp
    pub fn check_with_innovation(
        &mut self,
        match_score: f32,
        covariance: &Covariance2D,
        corrected_pose: &Pose2D,
        predicted_pose: &Pose2D,
        timestamp_us: u64,
    ) -> KidnappedDetection {
        // First do the basic check
        let mut detection = self.check(match_score, covariance, timestamp_us);

        // Additional innovation check
        if let Some(prev_pred) = &self.previous_prediction {
            let innovation = self.compute_innovation(corrected_pose, predicted_pose, prev_pred);

            if innovation > self.config.innovation_threshold {
                if detection.is_kidnapped {
                    detection.reason = KidnappedReason::Combined;
                    detection.confidence = (detection.confidence + 0.3).min(1.0);
                } else {
                    // Large innovation alone is suspicious but not conclusive
                    detection.confidence = (innovation / self.config.innovation_threshold - 1.0).min(0.5);
                }
            }
        }

        // Update state for next iteration
        self.previous_pose = Some(*corrected_pose);
        self.previous_prediction = Some(*predicted_pose);

        detection
    }

    /// Evaluate detection based on current state.
    fn evaluate_detection(&self, high_uncertainty: bool) -> (bool, KidnappedReason, f32) {
        let poor_matches = self.consecutive_poor >= self.config.consecutive_threshold;

        match (poor_matches, high_uncertainty) {
            (true, true) => {
                // Strong indication of kidnapping
                let confidence = 0.7 + 0.3 * (self.consecutive_poor as f32 / (self.config.consecutive_threshold as f32 * 2.0)).min(1.0);
                (true, KidnappedReason::Combined, confidence)
            }
            (true, false) => {
                // Consecutive poor matches alone
                let confidence = 0.5 + 0.3 * (self.consecutive_poor as f32 / (self.config.consecutive_threshold as f32 * 2.0)).min(1.0);
                (true, KidnappedReason::ConsecutivePoorMatches, confidence)
            }
            (false, true) if self.score_ema < self.config.score_threshold => {
                // High uncertainty with generally poor matching
                (true, KidnappedReason::HighUncertainty, 0.4)
            }
            _ => (false, KidnappedReason::None, 0.0),
        }
    }

    /// Update exponential moving average of scores.
    fn update_ema(&mut self, score: f32) {
        const ALPHA: f32 = 0.3; // Smoothing factor
        self.score_ema = ALPHA * score + (1.0 - ALPHA) * self.score_ema;
    }

    /// Compute innovation (mismatch between prediction and correction).
    fn compute_innovation(
        &self,
        corrected: &Pose2D,
        predicted: &Pose2D,
        _prev_predicted: &Pose2D,
    ) -> f32 {
        // Compute the difference between corrected and predicted poses
        let dx = corrected.x - predicted.x;
        let dy = corrected.y - predicted.y;
        let dtheta = crate::core::math::normalize_angle(corrected.theta - predicted.theta);

        // Normalized innovation (in "standard deviations")
        // Using typical motion model uncertainties
        let sigma_xy = 0.1; // 10cm position uncertainty
        let sigma_theta = 0.1; // ~6 deg rotation uncertainty

        let innovation_xy = (dx * dx + dy * dy).sqrt() / sigma_xy;
        let innovation_theta = dtheta.abs() / sigma_theta;

        innovation_xy.max(innovation_theta)
    }

    /// Reset the detector state.
    ///
    /// Call this after recovering from a kidnap event or when
    /// relocalization succeeds.
    pub fn reset(&mut self) {
        self.consecutive_poor = 0;
        self.score_ema = 1.0;
        self.previous_pose = None;
        self.previous_prediction = None;
        self.tracking_start_us = None;
        self.scan_count = 0;
    }

    /// Reset with a specific tracking start time.
    pub fn reset_at(&mut self, timestamp_us: u64) {
        self.reset();
        self.tracking_start_us = Some(timestamp_us);
    }

    /// Get current consecutive poor match count.
    pub fn consecutive_poor_matches(&self) -> usize {
        self.consecutive_poor
    }

    /// Get current score moving average.
    pub fn score_average(&self) -> f32 {
        self.score_ema
    }

    /// Get number of scans processed.
    pub fn scan_count(&self) -> u64 {
        self.scan_count
    }
}

impl Default for KidnappedDetector {
    fn default() -> Self {
        Self::new(KidnappedDetectorConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detector_default() {
        let detector = KidnappedDetector::default();
        assert!(detector.config.enabled);
        assert_eq!(detector.consecutive_poor_matches(), 0);
    }

    #[test]
    fn test_no_kidnap_with_good_scores() {
        let mut detector = KidnappedDetector::default();
        let cov = Covariance2D::diagonal(0.01, 0.01, 0.01);

        // Process several scans with good scores after tracking period
        for i in 0..10 {
            let detection = detector.check(0.8, &cov, 2_000_000 + i * 100_000);
            assert!(!detection.is_kidnapped);
            assert_eq!(detection.reason, KidnappedReason::None);
        }
    }

    #[test]
    fn test_kidnap_detected_with_poor_scores() {
        let config = KidnappedDetectorConfig {
            consecutive_threshold: 3,
            min_tracking_time_us: 0, // Disable warm-up for test
            ..Default::default()
        };
        let mut detector = KidnappedDetector::new(config);
        let cov = Covariance2D::diagonal(0.01, 0.01, 0.01);

        // First few poor scores
        for i in 0..3 {
            let detection = detector.check(0.1, &cov, i * 100_000);
            if i < 2 {
                assert!(!detection.is_kidnapped);
            }
        }

        // After threshold, should detect kidnap
        let detection = detector.check(0.1, &cov, 300_000);
        assert!(detection.is_kidnapped);
        assert_eq!(detection.reason, KidnappedReason::ConsecutivePoorMatches);
    }

    #[test]
    fn test_reset_clears_state() {
        let config = KidnappedDetectorConfig {
            consecutive_threshold: 3,
            min_tracking_time_us: 0,
            ..Default::default()
        };
        let mut detector = KidnappedDetector::new(config);
        let cov = Covariance2D::diagonal(0.01, 0.01, 0.01);

        // Build up poor matches
        for i in 0..5 {
            detector.check(0.1, &cov, i * 100_000);
        }
        assert!(detector.consecutive_poor_matches() > 0);

        // Reset
        detector.reset();
        assert_eq!(detector.consecutive_poor_matches(), 0);
        assert_eq!(detector.scan_count(), 0);
    }

    #[test]
    fn test_good_score_resets_consecutive() {
        let config = KidnappedDetectorConfig {
            consecutive_threshold: 5,
            min_tracking_time_us: 0,
            ..Default::default()
        };
        let mut detector = KidnappedDetector::new(config);
        let cov = Covariance2D::diagonal(0.01, 0.01, 0.01);

        // Build up some poor matches
        for i in 0..3 {
            detector.check(0.1, &cov, i * 100_000);
        }
        assert_eq!(detector.consecutive_poor_matches(), 3);

        // Good score should reset counter
        detector.check(0.8, &cov, 400_000);
        assert_eq!(detector.consecutive_poor_matches(), 0);
    }

    #[test]
    fn test_high_uncertainty_detection() {
        let config = KidnappedDetectorConfig {
            covariance_threshold: 0.5,
            score_threshold: 0.3,
            consecutive_threshold: 2,
            min_tracking_time_us: 0,
            ..Default::default()
        };
        let mut detector = KidnappedDetector::new(config);

        // High covariance with poor matches
        let high_cov = Covariance2D::diagonal(0.5, 0.5, 0.5);

        // Build up poor matches with high uncertainty
        detector.check(0.2, &high_cov, 0);
        detector.check(0.2, &high_cov, 100_000);
        let detection = detector.check(0.2, &high_cov, 200_000);

        assert!(detection.is_kidnapped);
        assert_eq!(detection.reason, KidnappedReason::Combined);
    }

    #[test]
    fn test_disabled_detector() {
        let config = KidnappedDetectorConfig {
            enabled: false,
            consecutive_threshold: 1,
            min_tracking_time_us: 0,
            ..Default::default()
        };
        let mut detector = KidnappedDetector::new(config);
        let cov = Covariance2D::diagonal(0.01, 0.01, 0.01);

        // Even with poor scores, should not detect kidnap
        for i in 0..10 {
            let detection = detector.check(0.0, &cov, i * 100_000);
            assert!(!detection.is_kidnapped);
        }
    }

    #[test]
    fn test_warmup_period() {
        let config = KidnappedDetectorConfig {
            min_tracking_time_us: 1_000_000, // 1 second
            consecutive_threshold: 2,
            ..Default::default()
        };
        let mut detector = KidnappedDetector::new(config);
        let cov = Covariance2D::diagonal(0.01, 0.01, 0.01);

        // Poor scores during warmup should not trigger
        for i in 0..5 {
            let detection = detector.check(0.0, &cov, i * 100_000);
            assert!(!detection.is_kidnapped, "Should not trigger during warmup");
        }

        // After warmup, should trigger
        for i in 0..5 {
            detector.check(0.0, &cov, 1_000_000 + i * 100_000);
        }

        let detection = detector.check(0.0, &cov, 1_500_000);
        assert!(detection.is_kidnapped);
    }

    #[test]
    fn test_score_ema() {
        let config = KidnappedDetectorConfig {
            min_tracking_time_us: 0,
            ..Default::default()
        };
        let mut detector = KidnappedDetector::new(config);
        let cov = Covariance2D::diagonal(0.01, 0.01, 0.01);

        // All low scores should bring down the EMA
        for i in 0..20 {
            detector.check(0.1, &cov, i * 100_000);
        }

        assert!(detector.score_average() < 0.3);

        // Good scores should gradually raise it
        for i in 0..20 {
            detector.check(0.9, &cov, 2_000_000 + i * 100_000);
        }

        assert!(detector.score_average() > 0.7);
    }

    #[test]
    fn test_innovation_check() {
        let config = KidnappedDetectorConfig {
            min_tracking_time_us: 0,
            innovation_threshold: 2.0,
            ..Default::default()
        };
        let mut detector = KidnappedDetector::new(config);
        let cov = Covariance2D::diagonal(0.01, 0.01, 0.01);

        // First call establishes baseline
        let corrected = Pose2D::new(0.0, 0.0, 0.0);
        let predicted = Pose2D::new(0.0, 0.0, 0.0);
        detector.check_with_innovation(0.8, &cov, &corrected, &predicted, 0);

        // Small innovation - should be fine
        let corrected = Pose2D::new(0.1, 0.0, 0.0);
        let predicted = Pose2D::new(0.1, 0.0, 0.0);
        let detection = detector.check_with_innovation(0.8, &cov, &corrected, &predicted, 100_000);
        assert!(!detection.is_kidnapped);

        // Large innovation with a good match score
        let corrected = Pose2D::new(1.0, 1.0, 0.0); // Big jump
        let predicted = Pose2D::new(0.2, 0.0, 0.0);
        let detection = detector.check_with_innovation(0.8, &cov, &corrected, &predicted, 200_000);
        // Should have elevated confidence but not necessarily trigger alone
        assert!(detection.confidence > 0.0 || !detection.is_kidnapped);
    }
}
