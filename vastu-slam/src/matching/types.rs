//! SLAM type definitions.

use crate::core::Pose2D;

/// Result of a scan-to-map matching operation.
#[derive(Clone, Debug)]
pub struct ScanMatchResult {
    /// The refined pose after matching.
    pub pose: Pose2D,
    /// Match quality score (0.0 to 1.0).
    /// Higher is better alignment.
    pub score: f32,
    /// Whether the matching converged to a good solution.
    pub converged: bool,
    /// Quality classification of the match.
    pub quality: MatchQuality,
    /// Number of scan points that hit occupied cells.
    pub hits: usize,
    /// Total number of valid scan points used.
    pub total_points: usize,
}

impl ScanMatchResult {
    /// Create a new match result.
    pub fn new(pose: Pose2D, score: f32, hits: usize, total_points: usize) -> Self {
        let quality = MatchQuality::from_score(score);
        // Accept Marginal quality during exploration when map is sparse
        let converged = matches!(
            quality,
            MatchQuality::Good | MatchQuality::Excellent | MatchQuality::Marginal
        );

        Self {
            pose,
            score,
            converged,
            quality,
            hits,
            total_points,
        }
    }

    /// Create a failed match result (no valid match found).
    pub fn failed(prior_pose: Pose2D) -> Self {
        Self {
            pose: prior_pose,
            score: 0.0,
            converged: false,
            quality: MatchQuality::Failed,
            hits: 0,
            total_points: 0,
        }
    }

    /// Hit ratio (0.0 to 1.0).
    pub fn hit_ratio(&self) -> f32 {
        if self.total_points == 0 {
            0.0
        } else {
            self.hits as f32 / self.total_points as f32
        }
    }
}

impl Default for ScanMatchResult {
    fn default() -> Self {
        Self::failed(Pose2D::default())
    }
}

/// Quality classification of a scan match.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MatchQuality {
    /// Excellent match (score > 0.8).
    Excellent,
    /// Good match (score > 0.6).
    Good,
    /// Marginal match (score > 0.4).
    Marginal,
    /// Poor match (score > 0.2).
    Poor,
    /// Match failed (score <= 0.2).
    Failed,
}

impl MatchQuality {
    /// Classify a match score.
    pub fn from_score(score: f32) -> Self {
        if score > 0.8 {
            MatchQuality::Excellent
        } else if score > 0.6 {
            MatchQuality::Good
        } else if score > 0.4 {
            MatchQuality::Marginal
        } else if score > 0.2 {
            MatchQuality::Poor
        } else {
            MatchQuality::Failed
        }
    }

    /// Check if this quality is acceptable for pose correction.
    pub fn is_acceptable(&self) -> bool {
        matches!(
            self,
            MatchQuality::Excellent | MatchQuality::Good | MatchQuality::Marginal
        )
    }
}
