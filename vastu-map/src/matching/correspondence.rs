//! Correspondence types for scan matching.
//!
//! A correspondence associates a point from the current scan with
//! a line in the map.

use crate::core::Point2D;

/// A correspondence between a scan point and a map line.
#[derive(Clone, Copy, Debug)]
pub struct Correspondence {
    /// Index of the point in the scan.
    pub point_idx: usize,
    /// Index of the line in the map.
    pub line_idx: usize,
    /// The scan point position (in world frame after initial transform).
    pub point: Point2D,
    /// Perpendicular distance from point to line.
    pub distance: f32,
    /// Projection parameter t along the line (0 = start, 1 = end).
    pub projection_t: f32,
}

impl Correspondence {
    /// Create a new correspondence.
    #[inline]
    pub fn new(
        point_idx: usize,
        line_idx: usize,
        point: Point2D,
        distance: f32,
        projection_t: f32,
    ) -> Self {
        Self {
            point_idx,
            line_idx,
            point,
            distance,
            projection_t,
        }
    }

    /// Check if the projection falls within the line segment.
    #[inline]
    pub fn is_within_segment(&self) -> bool {
        self.projection_t >= 0.0 && self.projection_t <= 1.0
    }

    /// Check if this is a valid correspondence (within distance threshold).
    #[inline]
    pub fn is_valid(&self, max_distance: f32) -> bool {
        self.distance <= max_distance
    }
}

/// Collection of correspondences for a scan match.
#[derive(Clone, Debug, Default)]
pub struct CorrespondenceSet {
    /// All correspondences found.
    pub correspondences: Vec<Correspondence>,
}

impl CorrespondenceSet {
    /// Create a new empty correspondence set.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            correspondences: Vec::with_capacity(capacity),
        }
    }

    /// Add a correspondence.
    #[inline]
    pub fn push(&mut self, corr: Correspondence) {
        self.correspondences.push(corr);
    }

    /// Number of correspondences.
    #[inline]
    pub fn len(&self) -> usize {
        self.correspondences.len()
    }

    /// Check if empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.correspondences.is_empty()
    }

    /// Filter correspondences by maximum distance.
    pub fn filter_by_distance(&self, max_distance: f32) -> Self {
        Self {
            correspondences: self
                .correspondences
                .iter()
                .filter(|c| c.distance <= max_distance)
                .copied()
                .collect(),
        }
    }

    /// Filter correspondences to only those within line segments.
    pub fn filter_within_segment(&self) -> Self {
        Self {
            correspondences: self
                .correspondences
                .iter()
                .filter(|c| c.is_within_segment())
                .copied()
                .collect(),
        }
    }

    /// Get the mean correspondence distance.
    pub fn mean_distance(&self) -> f32 {
        if self.is_empty() {
            return 0.0;
        }
        let sum: f32 = self.correspondences.iter().map(|c| c.distance).sum();
        sum / self.len() as f32
    }

    /// Get the RMS (root mean square) correspondence distance.
    pub fn rms_distance(&self) -> f32 {
        if self.is_empty() {
            return 0.0;
        }
        let sum_sq: f32 = self
            .correspondences
            .iter()
            .map(|c| c.distance * c.distance)
            .sum();
        (sum_sq / self.len() as f32).sqrt()
    }

    /// Get the maximum correspondence distance.
    pub fn max_distance(&self) -> f32 {
        self.correspondences
            .iter()
            .map(|c| c.distance)
            .fold(0.0, f32::max)
    }

    /// Compute inlier ratio (correspondences within threshold).
    pub fn inlier_ratio(&self, threshold: f32) -> f32 {
        if self.is_empty() {
            return 0.0;
        }
        let inliers = self
            .correspondences
            .iter()
            .filter(|c| c.distance <= threshold)
            .count();
        inliers as f32 / self.len() as f32
    }

    /// Clear all correspondences.
    pub fn clear(&mut self) {
        self.correspondences.clear();
    }

    /// Iterate over correspondences.
    pub fn iter(&self) -> impl Iterator<Item = &Correspondence> {
        self.correspondences.iter()
    }
}

/// Result of a scan matching operation.
#[derive(Clone, Debug)]
pub struct MatchResult {
    /// Estimated pose transformation.
    pub pose: crate::core::Pose2D,
    /// Correspondences used in the final match.
    pub correspondences: CorrespondenceSet,
    /// Number of iterations performed.
    pub iterations: usize,
    /// Whether the matching converged.
    pub converged: bool,
    /// Final RMS error.
    pub rms_error: f32,
    /// Match confidence (0.0 to 1.0).
    pub confidence: f32,
}

impl MatchResult {
    /// Create a new match result.
    pub fn new(
        pose: crate::core::Pose2D,
        correspondences: CorrespondenceSet,
        iterations: usize,
        converged: bool,
    ) -> Self {
        let rms_error = correspondences.rms_distance();
        let confidence = Self::compute_confidence(&correspondences, converged);

        Self {
            pose,
            correspondences,
            iterations,
            converged,
            rms_error,
            confidence,
        }
    }

    /// Compute match confidence based on various factors.
    fn compute_confidence(correspondences: &CorrespondenceSet, converged: bool) -> f32 {
        if !converged || correspondences.is_empty() {
            return 0.0;
        }

        // Factors affecting confidence:
        // 1. Number of correspondences (more is better)
        // 2. Mean distance (lower is better)
        // 3. Inlier ratio (higher is better)

        let n = correspondences.len() as f32;
        let count_factor = (n / 50.0).min(1.0); // Saturates at 50 correspondences

        let mean_dist = correspondences.mean_distance();
        let dist_factor = (-mean_dist / 0.1).exp(); // Exponential decay, 0.1m = ~37%

        let inlier_ratio = correspondences.inlier_ratio(0.1);

        // Combine factors
        (count_factor * dist_factor * inlier_ratio).clamp(0.0, 1.0)
    }

    /// Check if this is a good match.
    pub fn is_good_match(&self, min_confidence: f32) -> bool {
        self.converged && self.confidence >= min_confidence
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Pose2D;

    #[test]
    fn test_correspondence_new() {
        let corr = Correspondence::new(0, 1, Point2D::new(1.0, 2.0), 0.05, 0.5);

        assert_eq!(corr.point_idx, 0);
        assert_eq!(corr.line_idx, 1);
        assert_eq!(corr.distance, 0.05);
        assert!(corr.is_within_segment());
    }

    #[test]
    fn test_correspondence_within_segment() {
        let inside = Correspondence::new(0, 0, Point2D::zero(), 0.0, 0.5);
        assert!(inside.is_within_segment());

        let at_start = Correspondence::new(0, 0, Point2D::zero(), 0.0, 0.0);
        assert!(at_start.is_within_segment());

        let at_end = Correspondence::new(0, 0, Point2D::zero(), 0.0, 1.0);
        assert!(at_end.is_within_segment());

        let outside = Correspondence::new(0, 0, Point2D::zero(), 0.0, 1.5);
        assert!(!outside.is_within_segment());
    }

    #[test]
    fn test_correspondence_set_statistics() {
        let mut set = CorrespondenceSet::new();
        set.push(Correspondence::new(0, 0, Point2D::zero(), 0.1, 0.5));
        set.push(Correspondence::new(1, 0, Point2D::zero(), 0.2, 0.5));
        set.push(Correspondence::new(2, 0, Point2D::zero(), 0.3, 0.5));

        assert_eq!(set.len(), 3);
        assert!((set.mean_distance() - 0.2).abs() < 1e-6);
        assert_eq!(set.max_distance(), 0.3);

        // RMS = sqrt((0.01 + 0.04 + 0.09) / 3) = sqrt(0.14/3) ≈ 0.216
        assert!((set.rms_distance() - 0.216).abs() < 0.01);
    }

    #[test]
    fn test_correspondence_set_filter() {
        let mut set = CorrespondenceSet::new();
        set.push(Correspondence::new(0, 0, Point2D::zero(), 0.05, 0.5));
        set.push(Correspondence::new(1, 0, Point2D::zero(), 0.15, 0.5));
        set.push(Correspondence::new(2, 0, Point2D::zero(), 0.25, 0.5));

        let filtered = set.filter_by_distance(0.1);
        assert_eq!(filtered.len(), 1);
    }

    #[test]
    fn test_correspondence_set_inlier_ratio() {
        let mut set = CorrespondenceSet::new();
        set.push(Correspondence::new(0, 0, Point2D::zero(), 0.05, 0.5));
        set.push(Correspondence::new(1, 0, Point2D::zero(), 0.05, 0.5));
        set.push(Correspondence::new(2, 0, Point2D::zero(), 0.15, 0.5));
        set.push(Correspondence::new(3, 0, Point2D::zero(), 0.25, 0.5));

        let ratio = set.inlier_ratio(0.1);
        assert!((ratio - 0.5).abs() < 1e-6); // 2 out of 4
    }

    #[test]
    fn test_match_result() {
        let mut corr = CorrespondenceSet::new();
        for i in 0..50 {
            corr.push(Correspondence::new(i, 0, Point2D::zero(), 0.02, 0.5));
        }

        let result = MatchResult::new(Pose2D::identity(), corr, 10, true);

        assert!(result.converged);
        // With 50 correspondences at 0.02m: count_factor=1.0, dist_factor≈0.82, inlier_ratio=1.0
        // confidence ≈ 0.82
        assert!(result.confidence > 0.5);
        assert!(result.is_good_match(0.3));
    }
}
