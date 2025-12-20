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
    /// Weight for this correspondence (1/σ² based on measurement uncertainty).
    /// Higher weight = more reliable measurement. Default: 1.0
    pub weight: f32,
    /// Range from sensor to this point (meters). Used for weight computation.
    pub range: f32,
}

impl Correspondence {
    /// Create a new correspondence with default weight (1.0).
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
            weight: 1.0,
            range: 0.0,
        }
    }

    /// Create a new correspondence with explicit weight and range.
    #[inline]
    pub fn with_weight(
        point_idx: usize,
        line_idx: usize,
        point: Point2D,
        distance: f32,
        projection_t: f32,
        weight: f32,
        range: f32,
    ) -> Self {
        Self {
            point_idx,
            line_idx,
            point,
            distance,
            projection_t,
            weight,
            range,
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

    /// Get the weighted RMS correspondence distance.
    /// Uses correspondence weights for proper uncertainty-weighted averaging.
    pub fn weighted_rms_distance(&self) -> f32 {
        if self.is_empty() {
            return 0.0;
        }
        let mut sum_weighted_sq = 0.0f32;
        let mut sum_weights = 0.0f32;
        for c in &self.correspondences {
            sum_weighted_sq += c.weight * c.distance * c.distance;
            sum_weights += c.weight;
        }
        if sum_weights > 0.0 {
            (sum_weighted_sq / sum_weights).sqrt()
        } else {
            self.rms_distance() // fallback to unweighted
        }
    }

    /// Get the total weight of all correspondences.
    pub fn total_weight(&self) -> f32 {
        self.correspondences.iter().map(|c| c.weight).sum()
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

    /// Iterate over correspondences mutably.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Correspondence> {
        self.correspondences.iter_mut()
    }

    /// Apply weights to correspondences based on range using a noise model.
    ///
    /// The range for each correspondence is computed from the original robot-frame points
    /// (magnitude of the point vector). Weights are computed as 1/σ² where σ is the
    /// range-dependent measurement uncertainty.
    ///
    /// # Arguments
    /// * `robot_frame_points` - Original scan points in robot frame (for range computation)
    /// * `noise_model` - Lidar noise model for weight computation
    pub fn apply_weights(
        &mut self,
        robot_frame_points: &[crate::core::Point2D],
        noise_model: &crate::config::LidarNoiseModel,
    ) {
        for corr in &mut self.correspondences {
            if corr.point_idx < robot_frame_points.len() {
                let p = robot_frame_points[corr.point_idx];
                let range = (p.x * p.x + p.y * p.y).sqrt();
                corr.range = range;
                corr.weight = noise_model.weight(range);
            }
        }
    }

    /// Create a new set with weights applied based on range.
    pub fn with_weights(
        mut self,
        robot_frame_points: &[crate::core::Point2D],
        noise_model: &crate::config::LidarNoiseModel,
    ) -> Self {
        self.apply_weights(robot_frame_points, noise_model);
        self
    }
}

/// A correspondence between a scan point and a map corner.
///
/// Used for point-to-point constraints in ICP, which provide
/// better angular accuracy than point-to-line constraints alone.
#[derive(Clone, Copy, Debug)]
pub struct CornerCorrespondence {
    /// Index of the point in the scan.
    pub point_idx: usize,
    /// Index of the corner in the map.
    pub corner_idx: usize,
    /// The scan point position (in world frame after initial transform).
    pub point: Point2D,
    /// The map corner position.
    pub corner: Point2D,
    /// Distance from point to corner.
    pub distance: f32,
    /// Weight for this correspondence (1/σ² based on measurement uncertainty).
    pub weight: f32,
}

impl CornerCorrespondence {
    /// Create a new corner correspondence with default weight (1.0).
    #[inline]
    pub fn new(
        point_idx: usize,
        corner_idx: usize,
        point: Point2D,
        corner: Point2D,
        distance: f32,
    ) -> Self {
        Self {
            point_idx,
            corner_idx,
            point,
            corner,
            distance,
            weight: 1.0,
        }
    }

    /// Create a new corner correspondence with explicit weight.
    #[inline]
    pub fn with_weight(
        point_idx: usize,
        corner_idx: usize,
        point: Point2D,
        corner: Point2D,
        distance: f32,
        weight: f32,
    ) -> Self {
        Self {
            point_idx,
            corner_idx,
            point,
            corner,
            distance,
            weight,
        }
    }

    /// Check if this is a valid correspondence (within distance threshold).
    #[inline]
    pub fn is_valid(&self, max_distance: f32) -> bool {
        self.distance <= max_distance
    }
}

/// Collection of corner correspondences for a scan match.
#[derive(Clone, Debug, Default)]
pub struct CornerCorrespondenceSet {
    /// All corner correspondences found.
    pub correspondences: Vec<CornerCorrespondence>,
}

impl CornerCorrespondenceSet {
    /// Create a new empty corner correspondence set.
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
    pub fn push(&mut self, corr: CornerCorrespondence) {
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

    /// Get the total weight of all correspondences.
    pub fn total_weight(&self) -> f32 {
        self.correspondences.iter().map(|c| c.weight).sum()
    }

    /// Clear all correspondences.
    pub fn clear(&mut self) {
        self.correspondences.clear();
    }

    /// Iterate over correspondences.
    pub fn iter(&self) -> impl Iterator<Item = &CornerCorrespondence> {
        self.correspondences.iter()
    }

    /// Iterate over correspondences mutably.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut CornerCorrespondence> {
        self.correspondences.iter_mut()
    }
}

/// 3x3 covariance matrix for pose uncertainty [x, y, theta].
pub type PoseCovariance = [[f32; 3]; 3];

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
    /// Pose covariance matrix [x, y, theta].
    /// Computed from the Hessian of the Gauss-Newton optimization.
    pub covariance: PoseCovariance,
    /// Condition number of the Hessian matrix.
    /// High values (>100) indicate degenerate geometry (e.g., single wall).
    pub condition_number: f32,
    /// Whether coarse search was used for initialization.
    pub used_coarse_search: bool,
}

/// Identity covariance matrix (no uncertainty information).
pub const IDENTITY_COVARIANCE: PoseCovariance = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

impl MatchResult {
    /// Create a new match result with default covariance (backward compatible).
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
            covariance: IDENTITY_COVARIANCE,
            condition_number: 1.0,
            used_coarse_search: false,
        }
    }

    /// Create a new match result with full covariance information.
    pub fn with_covariance(
        pose: crate::core::Pose2D,
        correspondences: CorrespondenceSet,
        iterations: usize,
        converged: bool,
        covariance: PoseCovariance,
        condition_number: f32,
        used_coarse_search: bool,
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
            covariance,
            condition_number,
            used_coarse_search,
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

    /// Check if the geometry is degenerate (e.g., single wall).
    /// Returns true if condition number exceeds the threshold.
    pub fn is_degenerate(&self, threshold: f32) -> bool {
        self.condition_number > threshold
    }

    /// Get the position uncertainty (sqrt of x,y diagonal covariance elements).
    pub fn position_uncertainty(&self) -> f32 {
        (self.covariance[0][0] + self.covariance[1][1]).sqrt()
    }

    /// Get the orientation uncertainty (sqrt of theta diagonal covariance element).
    pub fn orientation_uncertainty(&self) -> f32 {
        self.covariance[2][2].sqrt()
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

    #[test]
    fn test_corner_correspondence_new() {
        let point = Point2D::new(1.0, 2.0);
        let corner = Point2D::new(1.1, 2.05);
        let corr = CornerCorrespondence::new(0, 1, point, corner, 0.1);

        assert_eq!(corr.point_idx, 0);
        assert_eq!(corr.corner_idx, 1);
        assert_eq!(corr.distance, 0.1);
        assert_eq!(corr.weight, 1.0);
        assert!(corr.is_valid(0.15));
        assert!(!corr.is_valid(0.05));
    }

    #[test]
    fn test_corner_correspondence_with_weight() {
        let point = Point2D::new(1.0, 2.0);
        let corner = Point2D::new(1.1, 2.05);
        let corr = CornerCorrespondence::with_weight(0, 1, point, corner, 0.1, 0.5);

        assert_eq!(corr.weight, 0.5);
    }

    #[test]
    fn test_corner_correspondence_set_statistics() {
        let mut set = CornerCorrespondenceSet::new();
        set.push(CornerCorrespondence::new(
            0,
            0,
            Point2D::zero(),
            Point2D::new(0.1, 0.0),
            0.1,
        ));
        set.push(CornerCorrespondence::new(
            1,
            1,
            Point2D::zero(),
            Point2D::new(0.2, 0.0),
            0.2,
        ));
        set.push(CornerCorrespondence::new(
            2,
            2,
            Point2D::zero(),
            Point2D::new(0.3, 0.0),
            0.3,
        ));

        assert_eq!(set.len(), 3);
        assert!(!set.is_empty());
        assert!((set.mean_distance() - 0.2).abs() < 1e-6);

        // RMS = sqrt((0.01 + 0.04 + 0.09) / 3) = sqrt(0.14/3) ≈ 0.216
        assert!((set.rms_distance() - 0.216).abs() < 0.01);
    }

    #[test]
    fn test_corner_correspondence_set_filter() {
        let mut set = CornerCorrespondenceSet::new();
        set.push(CornerCorrespondence::new(
            0,
            0,
            Point2D::zero(),
            Point2D::new(0.05, 0.0),
            0.05,
        ));
        set.push(CornerCorrespondence::new(
            1,
            1,
            Point2D::zero(),
            Point2D::new(0.15, 0.0),
            0.15,
        ));
        set.push(CornerCorrespondence::new(
            2,
            2,
            Point2D::zero(),
            Point2D::new(0.25, 0.0),
            0.25,
        ));

        let filtered = set.filter_by_distance(0.1);
        assert_eq!(filtered.len(), 1);
    }

    #[test]
    fn test_corner_correspondence_set_total_weight() {
        let mut set = CornerCorrespondenceSet::new();
        set.push(CornerCorrespondence::with_weight(
            0,
            0,
            Point2D::zero(),
            Point2D::new(0.1, 0.0),
            0.1,
            0.5,
        ));
        set.push(CornerCorrespondence::with_weight(
            1,
            1,
            Point2D::zero(),
            Point2D::new(0.2, 0.0),
            0.2,
            0.3,
        ));

        assert!((set.total_weight() - 0.8).abs() < 1e-6);
    }
}
