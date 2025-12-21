//! Correspondence types for scan matching.
//!
//! A correspondence associates a point from the current scan with
//! a line in the map.
//!
//! # Data Layouts
//!
//! Two layouts are provided:
//! - [`CorrespondenceSet`]: Array-of-Structs (AoS) layout for general use
//! - [`CorrespondenceSoA`]: Struct-of-Arrays (SoA) layout optimized for SIMD Gauss-Newton
//!
//! The SoA layout inlines line data (normals, endpoints) at correspondence creation time,
//! enabling contiguous SIMD loads in the hot loop instead of random indexed access.

use crate::core::Point2D;
use crate::features::LineCollection;

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

// ─────────────────────────────────────────────────────────────────────────────
// CorrespondenceSoA: SIMD-Optimized Struct-of-Arrays Layout
// ─────────────────────────────────────────────────────────────────────────────

/// SIMD-optimized correspondence collection with Struct-of-Arrays layout.
///
/// Unlike [`CorrespondenceSet`], this structure inlines line data (normals,
/// endpoints, inverse lengths) at correspondence creation time. This enables
/// contiguous SIMD loads in the Gauss-Newton hot loop instead of random
/// indexed access to line arrays.
///
/// # Memory Layout
///
/// **HOT data** (used in Gauss-Newton inner loop):
/// - Point positions (point_xs, point_ys)
/// - Line normals (normal_xs, normal_ys) - INLINED, not indexed
/// - Line endpoints (line_start_xs, line_start_ys, line_end_xs, line_end_ys)
/// - Inverse lengths (line_inv_lengths) - for residual computation
/// - Weights
///
/// **COLD data** (metadata, not used in optimization):
/// - Point indices, line indices, distances
///
/// # SIMD Usage
///
/// Arrays are padded to SIMD width (4) with zero-weight entries via `finalize()`.
/// Use `simd_chunks()` to get the number of 4-element chunks for iteration.
///
/// # Example
///
/// ```rust,ignore
/// use vastu_map::matching::CorrespondenceSoA;
///
/// let mut soa = CorrespondenceSoA::new();
/// // ... push correspondences ...
/// soa.finalize(); // Pad to SIMD width
///
/// for i in 0..soa.simd_chunks() {
///     let base = i * 4;
///     let px = f32x4::from_slice(&soa.point_xs[base..]);
///     let nx = f32x4::from_slice(&soa.normal_xs[base..]);
///     // ... SIMD operations ...
/// }
/// ```
#[derive(Clone, Debug, Default)]
pub struct CorrespondenceSoA {
    // === HOT: Contiguous arrays for SIMD (used in Gauss-Newton) ===
    /// X coordinates of scan points (world frame).
    pub point_xs: Vec<f32>,
    /// Y coordinates of scan points (world frame).
    pub point_ys: Vec<f32>,
    /// X components of line unit normals (INLINED from LineCollection).
    pub normal_xs: Vec<f32>,
    /// Y components of line unit normals (INLINED from LineCollection).
    pub normal_ys: Vec<f32>,
    /// X coordinates of line start points.
    pub line_start_xs: Vec<f32>,
    /// Y coordinates of line start points.
    pub line_start_ys: Vec<f32>,
    /// X coordinates of line end points.
    pub line_end_xs: Vec<f32>,
    /// Y coordinates of line end points.
    pub line_end_ys: Vec<f32>,
    /// Reciprocal of line lengths (1.0 / length).
    pub line_inv_lengths: Vec<f32>,
    /// Correspondence weights (1/σ² from noise model).
    pub weights: Vec<f32>,

    // === COLD: Metadata (not used in optimization) ===
    /// Indices of points in original scan.
    pub point_indices: Vec<usize>,
    /// Indices of lines in map.
    pub line_indices: Vec<usize>,
    /// Perpendicular distances from points to lines.
    pub distances: Vec<f32>,

    /// Number of real correspondences (before padding).
    len: usize,
}

/// SIMD lane width for f32x4.
const SIMD_WIDTH: usize = 4;

impl CorrespondenceSoA {
    /// Create a new empty SoA correspondence collection.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with capacity hint.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            point_xs: Vec::with_capacity(capacity),
            point_ys: Vec::with_capacity(capacity),
            normal_xs: Vec::with_capacity(capacity),
            normal_ys: Vec::with_capacity(capacity),
            line_start_xs: Vec::with_capacity(capacity),
            line_start_ys: Vec::with_capacity(capacity),
            line_end_xs: Vec::with_capacity(capacity),
            line_end_ys: Vec::with_capacity(capacity),
            line_inv_lengths: Vec::with_capacity(capacity),
            weights: Vec::with_capacity(capacity),
            point_indices: Vec::with_capacity(capacity),
            line_indices: Vec::with_capacity(capacity),
            distances: Vec::with_capacity(capacity),
            len: 0,
        }
    }

    /// Add a correspondence, inlining line data from the LineCollection.
    ///
    /// This copies the line's normal, endpoints, and inverse length into the
    /// SoA arrays, eliminating indexed access in the Gauss-Newton hot loop.
    #[inline]
    pub fn push(
        &mut self,
        point_idx: usize,
        line_idx: usize,
        point: Point2D,
        distance: f32,
        weight: f32,
        lines: &LineCollection,
    ) {
        self.point_xs.push(point.x);
        self.point_ys.push(point.y);

        // Inline line data (no indexed access needed in hot loop)
        let (normal_xs, normal_ys) = lines.normals();
        self.normal_xs.push(normal_xs[line_idx]);
        self.normal_ys.push(normal_ys[line_idx]);
        self.line_start_xs.push(lines.start_xs[line_idx]);
        self.line_start_ys.push(lines.start_ys[line_idx]);
        self.line_end_xs.push(lines.end_xs[line_idx]);
        self.line_end_ys.push(lines.end_ys[line_idx]);
        self.line_inv_lengths.push(lines.inv_lengths()[line_idx]);
        self.weights.push(weight);

        // Metadata
        self.point_indices.push(point_idx);
        self.line_indices.push(line_idx);
        self.distances.push(distance);

        self.len += 1;
    }

    /// Add a correspondence with explicit line data (for when LineCollection isn't available).
    #[inline]
    pub fn push_with_line_data(
        &mut self,
        point_idx: usize,
        line_idx: usize,
        point: Point2D,
        distance: f32,
        weight: f32,
        normal_x: f32,
        normal_y: f32,
        start_x: f32,
        start_y: f32,
        end_x: f32,
        end_y: f32,
        inv_length: f32,
    ) {
        self.point_xs.push(point.x);
        self.point_ys.push(point.y);
        self.normal_xs.push(normal_x);
        self.normal_ys.push(normal_y);
        self.line_start_xs.push(start_x);
        self.line_start_ys.push(start_y);
        self.line_end_xs.push(end_x);
        self.line_end_ys.push(end_y);
        self.line_inv_lengths.push(inv_length);
        self.weights.push(weight);
        self.point_indices.push(point_idx);
        self.line_indices.push(line_idx);
        self.distances.push(distance);
        self.len += 1;
    }

    /// Number of real correspondences (before SIMD padding).
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Pad arrays to SIMD width (4) with zero-weight entries.
    ///
    /// Call this after all correspondences have been added and before
    /// SIMD iteration. Zero-weight entries don't affect the optimization.
    pub fn finalize(&mut self) {
        let remainder = self.len % SIMD_WIDTH;
        if remainder == 0 {
            return;
        }

        let padding = SIMD_WIDTH - remainder;
        for _ in 0..padding {
            self.point_xs.push(0.0);
            self.point_ys.push(0.0);
            self.normal_xs.push(0.0);
            self.normal_ys.push(1.0); // Arbitrary valid normal
            self.line_start_xs.push(0.0);
            self.line_start_ys.push(0.0);
            self.line_end_xs.push(0.0);
            self.line_end_ys.push(0.0);
            self.line_inv_lengths.push(0.0);
            self.weights.push(0.0); // Zero weight = no contribution
            self.point_indices.push(0);
            self.line_indices.push(0);
            self.distances.push(0.0);
        }
    }

    /// Number of SIMD chunks (4-element groups) for iteration.
    ///
    /// Call after `finalize()` for correct padding.
    #[inline]
    pub fn simd_chunks(&self) -> usize {
        self.point_xs.len().div_ceil(SIMD_WIDTH)
    }

    /// Clear all correspondences (keeps capacity).
    pub fn clear(&mut self) {
        self.point_xs.clear();
        self.point_ys.clear();
        self.normal_xs.clear();
        self.normal_ys.clear();
        self.line_start_xs.clear();
        self.line_start_ys.clear();
        self.line_end_xs.clear();
        self.line_end_ys.clear();
        self.line_inv_lengths.clear();
        self.weights.clear();
        self.point_indices.clear();
        self.line_indices.clear();
        self.distances.clear();
        self.len = 0;
    }

    /// Get the mean correspondence distance.
    pub fn mean_distance(&self) -> f32 {
        if self.len == 0 {
            return 0.0;
        }
        let sum: f32 = self.distances[..self.len].iter().sum();
        sum / self.len as f32
    }

    /// Get the RMS (root mean square) correspondence distance.
    pub fn rms_distance(&self) -> f32 {
        if self.len == 0 {
            return 0.0;
        }
        let sum_sq: f32 = self.distances[..self.len].iter().map(|d| d * d).sum();
        (sum_sq / self.len as f32).sqrt()
    }

    /// Get the weighted RMS correspondence distance.
    pub fn weighted_rms_distance(&self) -> f32 {
        if self.len == 0 {
            return 0.0;
        }
        let mut sum_weighted_sq = 0.0f32;
        let mut sum_weights = 0.0f32;
        for i in 0..self.len {
            let w = self.weights[i];
            let d = self.distances[i];
            sum_weighted_sq += w * d * d;
            sum_weights += w;
        }
        if sum_weights > 0.0 {
            (sum_weighted_sq / sum_weights).sqrt()
        } else {
            self.rms_distance()
        }
    }

    /// Get the total weight of all correspondences.
    pub fn total_weight(&self) -> f32 {
        self.weights[..self.len].iter().sum()
    }

    /// Get the maximum correspondence distance.
    pub fn max_distance(&self) -> f32 {
        self.distances[..self.len]
            .iter()
            .copied()
            .fold(0.0, f32::max)
    }

    /// Compute inlier ratio (correspondences within threshold).
    pub fn inlier_ratio(&self, threshold: f32) -> f32 {
        if self.len == 0 {
            return 0.0;
        }
        let inliers = self.distances[..self.len]
            .iter()
            .filter(|&&d| d <= threshold)
            .count();
        inliers as f32 / self.len as f32
    }

    /// Convert to CorrespondenceSet (for compatibility with existing code).
    pub fn to_correspondence_set(&self) -> CorrespondenceSet {
        let mut set = CorrespondenceSet::with_capacity(self.len);
        for i in 0..self.len {
            set.push(Correspondence::with_weight(
                self.point_indices[i],
                self.line_indices[i],
                Point2D::new(self.point_xs[i], self.point_ys[i]),
                self.distances[i],
                0.0, // projection_t not stored in SoA
                self.weights[i],
                0.0, // range not stored in SoA
            ));
        }
        set
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
