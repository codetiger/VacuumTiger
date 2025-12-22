//! RANSAC-based line extraction.
//!
//! This module provides RANSAC (Random Sample Consensus) for extracting
//! dominant lines from point clouds. RANSAC is robust to outliers and
//! can find strong lines even in cluttered environments.
//!
//! # Usage
//!
//! ```rust,ignore
//! use vastu_map::extraction::{extract_lines_ransac, RansacLineConfig, RansacScratchSpace};
//!
//! let config = RansacLineConfig::default();
//!
//! // Zero-allocation version with scratch space
//! let mut scratch = RansacScratchSpace::with_capacity(400);
//! let (lines, remaining) = extract_lines_ransac_with_scratch(&points, &config, &mut scratch);
//!
//! // Reuse scratch for next scan - no allocations!
//! let (lines2, remaining2) = extract_lines_ransac_with_scratch(&points2, &config, &mut scratch);
//! ```

use serde::{Deserialize, Serialize};

use crate::core::Point2D;
use crate::core::math::{compute_centroid, compute_covariance};
use crate::features::Line2D;

use super::line_fitting::fit_line;

// ============================================================================
// Scratch Space for Zero-Allocation RANSAC
// ============================================================================

/// Pre-allocated scratch space for RANSAC operations.
///
/// Reuse this struct across multiple `extract_lines_ransac_with_scratch` calls
/// to eliminate per-iteration allocations. This is critical for real-time SLAM
/// where RANSAC may run 100+ iterations per scan.
///
/// # Performance Impact
///
/// Without scratch space, each RANSAC call allocates:
/// - `remaining_mask`: O(n) bools
/// - `remaining`: O(n) indices, rebuilt every outer iteration
/// - `current_inliers`: O(n) indices, rebuilt every RANSAC iteration (100x)
/// - `best_inliers`: O(n) indices
/// - `inlier_points`: O(n) Point2Ds
///
/// With scratch space, these allocations happen once and are reused.
///
/// # Example
/// ```rust,ignore
/// use vastu_map::extraction::{RansacScratchSpace, extract_lines_ransac_with_scratch};
///
/// // Create once, sized for typical lidar scans
/// let mut scratch = RansacScratchSpace::with_capacity(400);
///
/// for scan in scans {
///     let (lines, remaining) = extract_lines_ransac_with_scratch(&scan, &config, &mut scratch);
///     // scratch is reused, no allocations!
/// }
/// ```
#[derive(Clone, Debug)]
pub struct RansacScratchSpace {
    /// Bit mask for remaining (unconsumed) points.
    /// Avoids rebuilding Vec<usize> each iteration.
    remaining_mask: Vec<bool>,

    /// Buffer for current iteration's inlier indices.
    /// Reused across all RANSAC iterations.
    current_inliers: Vec<usize>,

    /// Buffer for best inliers found so far.
    best_inliers: Vec<usize>,

    /// Buffer for inlier points (for line fitting and gap splitting).
    inlier_points: Vec<Point2D>,

    /// Buffer for inlier indices corresponding to inlier_points.
    inlier_indices: Vec<usize>,

    /// Buffer for output lines.
    output_lines: Vec<Line2D>,

    /// Buffer for remaining indices output.
    remaining_output: Vec<usize>,

    /// Buffer for segment points in split_at_gaps.
    segment_points: Vec<Point2D>,

    /// Buffer for segment indices in split_at_gaps.
    segment_indices: Vec<usize>,

    /// Buffer for projection sorting in split_at_gaps.
    projection_buffer: Vec<(usize, f32)>,
}

impl RansacScratchSpace {
    /// Create new scratch space with capacity hints.
    ///
    /// # Arguments
    /// * `max_points` - Expected maximum scan points (e.g., 360-400 for typical lidar)
    pub fn with_capacity(max_points: usize) -> Self {
        Self {
            remaining_mask: Vec::with_capacity(max_points),
            current_inliers: Vec::with_capacity(max_points),
            best_inliers: Vec::with_capacity(max_points),
            inlier_points: Vec::with_capacity(max_points),
            inlier_indices: Vec::with_capacity(max_points),
            output_lines: Vec::with_capacity(32), // Typical max lines per scan
            remaining_output: Vec::with_capacity(max_points),
            segment_points: Vec::with_capacity(max_points),
            segment_indices: Vec::with_capacity(max_points),
            projection_buffer: Vec::with_capacity(max_points),
        }
    }

    /// Create with default capacity (400 points).
    pub fn default_capacity() -> Self {
        Self::with_capacity(400)
    }

    /// Reset scratch space for a new extraction run.
    /// Clears all buffers but retains allocated capacity.
    #[inline]
    pub fn reset(&mut self, num_points: usize) {
        self.remaining_mask.clear();
        self.remaining_mask.resize(num_points, true);
        self.current_inliers.clear();
        self.best_inliers.clear();
        self.inlier_points.clear();
        self.inlier_indices.clear();
        self.output_lines.clear();
        self.remaining_output.clear();
        self.segment_points.clear();
        self.segment_indices.clear();
        self.projection_buffer.clear();
    }

    /// Count remaining active points using the mask.
    #[inline]
    fn count_remaining(&self) -> usize {
        self.remaining_mask.iter().filter(|&&b| b).count()
    }
}

impl Default for RansacScratchSpace {
    fn default() -> Self {
        Self::default_capacity()
    }
}

/// Configuration for RANSAC line extraction.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RansacLineConfig {
    /// Number of RANSAC iterations per line.
    /// More iterations = better chance of finding optimal line.
    /// Default: 100
    pub iterations: usize,

    /// Inlier distance threshold (meters).
    /// Points within this distance of a line are considered inliers.
    /// Default: 0.02m (2cm)
    pub inlier_threshold: f32,

    /// Minimum inliers for a valid line.
    /// Lines with fewer inliers are rejected.
    /// Default: 5
    pub min_inliers: usize,

    /// Minimum line length (meters).
    /// Lines shorter than this are rejected.
    /// Default: 0.1m (10cm)
    pub min_line_length: f32,

    /// Maximum gap between consecutive inliers along a line (meters).
    /// If inliers have a gap larger than this, the line is split.
    /// This prevents connecting collinear walls separated by doorways.
    /// Default: 0.10m (10cm)
    pub max_point_gap: f32,

    /// Maximum lines to extract (0 = unlimited).
    /// Stops after extracting this many lines.
    /// Default: 0 (unlimited)
    pub max_lines: usize,

    /// Random seed for reproducibility.
    /// 0 = use entropy-based seed (non-deterministic).
    /// Default: 0
    pub seed: u64,
}

impl Default for RansacLineConfig {
    fn default() -> Self {
        Self {
            iterations: 100,
            inlier_threshold: 0.02,
            min_inliers: 5,
            min_line_length: 0.1,
            max_point_gap: 0.10,
            max_lines: 0,
            seed: 0,
        }
    }
}

impl RansacLineConfig {
    /// Create a new config with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for iterations.
    pub fn with_iterations(mut self, iterations: usize) -> Self {
        self.iterations = iterations;
        self
    }

    /// Builder-style setter for inlier threshold.
    pub fn with_inlier_threshold(mut self, threshold: f32) -> Self {
        self.inlier_threshold = threshold;
        self
    }

    /// Builder-style setter for minimum inliers.
    pub fn with_min_inliers(mut self, min_inliers: usize) -> Self {
        self.min_inliers = min_inliers;
        self
    }

    /// Builder-style setter for minimum line length.
    pub fn with_min_line_length(mut self, length: f32) -> Self {
        self.min_line_length = length;
        self
    }

    /// Builder-style setter for maximum lines.
    pub fn with_max_lines(mut self, max_lines: usize) -> Self {
        self.max_lines = max_lines;
        self
    }

    /// Builder-style setter for maximum point gap.
    pub fn with_max_point_gap(mut self, gap: f32) -> Self {
        self.max_point_gap = gap;
        self
    }

    /// Builder-style setter for random seed.
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }
}

/// Simple LCG random number generator for deterministic behavior.
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        // Use a non-zero seed
        Self {
            state: if seed == 0 {
                // Use a simple time-based seed
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos() as u64)
                    .unwrap_or(12345)
            } else {
                seed
            },
        }
    }

    fn next(&mut self) -> u64 {
        // LCG parameters from Numerical Recipes
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1);
        self.state
    }

    fn gen_range(&mut self, max: usize) -> usize {
        if max == 0 {
            return 0;
        }
        (self.next() % (max as u64)) as usize
    }
}

/// Split inliers at gaps to prevent connecting separate wall segments.
///
/// Points are sorted along the line direction and split wherever
/// consecutive points have a gap larger than `max_point_gap`.
fn split_at_gaps(
    inlier_points: &[Point2D],
    inlier_indices: &[usize],
    config: &RansacLineConfig,
) -> Vec<(Vec<Point2D>, Vec<usize>)> {
    if inlier_points.len() < 2 {
        return vec![(inlier_points.to_vec(), inlier_indices.to_vec())];
    }

    // Use shared utilities for centroid and covariance
    let centroid = compute_centroid(inlier_points);
    let cov = compute_covariance(inlier_points, centroid);

    // Principal direction from dominant eigenvector (using atan2 for angle)
    let theta = 0.5 * (2.0 * cov.cxy).atan2(cov.cxx - cov.cyy);
    let dir_x = theta.cos();
    let dir_y = theta.sin();

    // Project points onto principal direction and sort
    let mut indexed: Vec<(usize, f32)> = inlier_points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let proj = (p.x - centroid.x) * dir_x + (p.y - centroid.y) * dir_y;
            (i, proj)
        })
        .collect();

    indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    // Split at gaps
    let mut segments = Vec::new();
    let mut current_points = Vec::new();
    let mut current_indices = Vec::new();

    for (i, &(orig_idx, _proj)) in indexed.iter().enumerate() {
        current_points.push(inlier_points[orig_idx]);
        current_indices.push(inlier_indices[orig_idx]);

        // Check if there's a gap to the next point
        if i + 1 < indexed.len() {
            let next_orig_idx = indexed[i + 1].0;
            let dist = inlier_points[orig_idx].distance(inlier_points[next_orig_idx]);

            if dist > config.max_point_gap {
                // Gap found - save current segment and start new one
                if !current_points.is_empty() {
                    segments.push((
                        std::mem::take(&mut current_points),
                        std::mem::take(&mut current_indices),
                    ));
                }
            }
        }
    }

    // Add final segment
    if !current_points.is_empty() {
        segments.push((current_points, current_indices));
    }

    segments
}

/// Extract lines using RANSAC.
///
/// Returns extracted lines and indices of points not matched to any line.
///
/// # Arguments
/// * `points` - Input point cloud
/// * `config` - RANSAC configuration
///
/// # Returns
/// Tuple of (extracted_lines, remaining_point_indices)
pub fn extract_lines_ransac(
    points: &[Point2D],
    config: &RansacLineConfig,
) -> (Vec<Line2D>, Vec<usize>) {
    if points.len() < config.min_inliers {
        return (Vec::new(), (0..points.len()).collect());
    }

    let mut remaining_mask = vec![true; points.len()];
    let mut lines = Vec::new();
    let mut rng = SimpleRng::new(config.seed);

    loop {
        // Check max lines limit
        if config.max_lines > 0 && lines.len() >= config.max_lines {
            break;
        }

        // Collect remaining point indices
        let remaining: Vec<usize> = remaining_mask
            .iter()
            .enumerate()
            .filter_map(|(i, &active)| if active { Some(i) } else { None })
            .collect();

        if remaining.len() < config.min_inliers {
            break;
        }

        // Run RANSAC to find best line
        let mut best_inliers: Vec<usize> = Vec::new();

        for _ in 0..config.iterations {
            // Random sample of 2 distinct points
            let idx1 = remaining[rng.gen_range(remaining.len())];
            let idx2 = remaining[rng.gen_range(remaining.len())];
            if idx1 == idx2 {
                continue;
            }

            let p1 = points[idx1];
            let p2 = points[idx2];

            // Check minimum distance between sample points
            let sample_dist = p1.distance(p2);
            if sample_dist < config.min_line_length * 0.3 {
                continue;
            }

            // Create candidate line
            let line = Line2D::new(p1, p2);

            // Count inliers
            let inliers: Vec<usize> = remaining
                .iter()
                .copied()
                .filter(|&i| line.distance_to_point(points[i]) < config.inlier_threshold)
                .collect();

            if inliers.len() > best_inliers.len() {
                best_inliers = inliers;
            }
        }

        // Check if we found enough inliers
        if best_inliers.len() < config.min_inliers {
            break;
        }

        // Get inlier points
        let inlier_points: Vec<Point2D> = best_inliers.iter().map(|&i| points[i]).collect();

        // Split inliers at gaps to avoid connecting separate wall segments
        let segments = split_at_gaps(&inlier_points, &best_inliers, config);

        let mut added_any = false;
        for (segment_points, segment_indices) in segments {
            if segment_points.len() < config.min_inliers {
                continue;
            }

            if let Some(refined_line) = fit_line(&segment_points) {
                // Check minimum length
                if refined_line.length() >= config.min_line_length {
                    lines.push(refined_line);
                    added_any = true;

                    // Mark segment inliers as used
                    for &i in &segment_indices {
                        remaining_mask[i] = false;
                    }
                }
            }
        }

        // If we couldn't add any valid lines from this iteration, stop
        if !added_any {
            break;
        }
    }

    // Collect remaining point indices
    let remaining: Vec<usize> = remaining_mask
        .iter()
        .enumerate()
        .filter_map(|(i, &active)| if active { Some(i) } else { None })
        .collect();

    (lines, remaining)
}

/// Extract lines using RANSAC with pre-allocated scratch space.
///
/// This is the zero-allocation version for real-time applications.
/// Reuse the scratch space across multiple calls to eliminate allocations.
///
/// Returns extracted lines and indices of points not matched to any line.
/// The returned vectors are borrowed from scratch space and are valid until
/// the next call to this function.
///
/// # Arguments
/// * `points` - Input point cloud
/// * `config` - RANSAC configuration
/// * `scratch` - Pre-allocated scratch space (reused across calls)
///
/// # Returns
/// Tuple of (&[Line2D], &[usize]) - references into scratch space
///
/// # Example
/// ```rust,ignore
/// let mut scratch = RansacScratchSpace::with_capacity(400);
/// let (lines, remaining) = extract_lines_ransac_with_scratch(&points, &config, &mut scratch);
/// // Process lines and remaining before next call
/// ```
pub fn extract_lines_ransac_with_scratch<'a>(
    points: &[Point2D],
    config: &RansacLineConfig,
    scratch: &'a mut RansacScratchSpace,
) -> (&'a [Line2D], &'a [usize]) {
    // Reset scratch for this extraction
    scratch.reset(points.len());

    if points.len() < config.min_inliers {
        // Return remaining as all indices
        scratch.remaining_output.extend(0..points.len());
        return (&scratch.output_lines, &scratch.remaining_output);
    }

    let mut rng = SimpleRng::new(config.seed);

    loop {
        // Check max lines limit
        if config.max_lines > 0 && scratch.output_lines.len() >= config.max_lines {
            break;
        }

        // Count remaining points without allocating
        let remaining_count = scratch.count_remaining();
        if remaining_count < config.min_inliers {
            break;
        }

        // Run RANSAC to find best line
        scratch.best_inliers.clear();

        for _ in 0..config.iterations {
            // Sample random indices from remaining points
            // We need to pick the nth remaining point, not the nth point
            let sample_pos1 = rng.gen_range(remaining_count);
            let sample_pos2 = rng.gen_range(remaining_count);
            if sample_pos1 == sample_pos2 {
                continue;
            }

            // Convert sample positions to actual indices
            let idx1 = nth_remaining(&scratch.remaining_mask, sample_pos1);
            let idx2 = nth_remaining(&scratch.remaining_mask, sample_pos2);

            let p1 = points[idx1];
            let p2 = points[idx2];

            // Check minimum distance between sample points
            let sample_dist = p1.distance(p2);
            if sample_dist < config.min_line_length * 0.3 {
                continue;
            }

            // Create candidate line
            let line = Line2D::new(p1, p2);

            // Count inliers into scratch buffer (reused each iteration)
            scratch.current_inliers.clear();
            for (i, &is_remaining) in scratch.remaining_mask.iter().enumerate() {
                if is_remaining && line.distance_to_point(points[i]) < config.inlier_threshold {
                    scratch.current_inliers.push(i);
                }
            }

            // Update best if this is better
            if scratch.current_inliers.len() > scratch.best_inliers.len() {
                scratch.best_inliers.clear();
                scratch
                    .best_inliers
                    .extend_from_slice(&scratch.current_inliers);
            }
        }

        // Check if we found enough inliers
        if scratch.best_inliers.len() < config.min_inliers {
            break;
        }

        // Get inlier points into scratch buffer
        scratch.inlier_points.clear();
        scratch.inlier_indices.clear();
        for &i in &scratch.best_inliers {
            scratch.inlier_points.push(points[i]);
            scratch.inlier_indices.push(i);
        }

        // Split at gaps and process segments
        let added_any = split_at_gaps_with_scratch(config, scratch);

        // If we couldn't add any valid lines from this iteration, stop
        if !added_any {
            break;
        }
    }

    // Collect remaining point indices into output buffer
    scratch.remaining_output.clear();
    for (i, &is_remaining) in scratch.remaining_mask.iter().enumerate() {
        if is_remaining {
            scratch.remaining_output.push(i);
        }
    }

    (&scratch.output_lines, &scratch.remaining_output)
}

/// Find the nth remaining (active) index in the mask.
#[inline]
fn nth_remaining(mask: &[bool], mut n: usize) -> usize {
    for (i, &active) in mask.iter().enumerate() {
        if active {
            if n == 0 {
                return i;
            }
            n -= 1;
        }
    }
    // Fallback (shouldn't happen if n < count_remaining)
    0
}

/// Split inliers at gaps using scratch buffers, add valid lines to output.
/// Returns true if any lines were added.
fn split_at_gaps_with_scratch(config: &RansacLineConfig, scratch: &mut RansacScratchSpace) -> bool {
    let num_inliers = scratch.inlier_points.len();

    if num_inliers < 2 {
        // Single point segment - try to fit
        if num_inliers >= config.min_inliers
            && let Some(line) = fit_line(&scratch.inlier_points)
            && line.length() >= config.min_line_length
        {
            scratch.output_lines.push(line);
            for i in 0..scratch.inlier_indices.len() {
                let idx = scratch.inlier_indices[i];
                scratch.remaining_mask[idx] = false;
            }
            return true;
        }
        return false;
    }

    // Use shared utilities for centroid and covariance
    let centroid = compute_centroid(&scratch.inlier_points);
    let cov = compute_covariance(&scratch.inlier_points, centroid);

    // Principal direction from dominant eigenvector (using atan2 for angle)
    let theta = 0.5 * (2.0 * cov.cxy).atan2(cov.cxx - cov.cyy);
    let dir_x = theta.cos();
    let dir_y = theta.sin();

    // Project points onto principal direction and sort
    scratch.projection_buffer.clear();
    for (i, p) in scratch.inlier_points.iter().enumerate() {
        let proj = (p.x - centroid.x) * dir_x + (p.y - centroid.y) * dir_y;
        scratch.projection_buffer.push((i, proj));
    }
    scratch
        .projection_buffer
        .sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    // Process segments separated by gaps
    // We need to iterate through projection_buffer and build segments,
    // then mark consumed indices. To avoid borrow conflicts, we'll
    // collect segment boundaries first, then process them.
    let mut added_any = false;
    scratch.segment_points.clear();
    scratch.segment_indices.clear();

    let projection_len = scratch.projection_buffer.len();
    let mut segment_start = 0usize;

    for i in 0..projection_len {
        let (orig_idx, _proj) = scratch.projection_buffer[i];

        // Check if there's a gap to the next point
        let is_gap = if i + 1 < projection_len {
            let next_orig_idx = scratch.projection_buffer[i + 1].0;
            let p1 = scratch.inlier_points[orig_idx];
            let p2 = scratch.inlier_points[next_orig_idx];
            p1.distance(p2) > config.max_point_gap
        } else {
            true // End of points
        };

        if is_gap {
            // Process segment from segment_start to i (inclusive)
            let segment_len = i - segment_start + 1;

            if segment_len >= config.min_inliers {
                // Collect segment points
                scratch.segment_points.clear();
                scratch.segment_indices.clear();

                for j in segment_start..=i {
                    let idx = scratch.projection_buffer[j].0;
                    scratch.segment_points.push(scratch.inlier_points[idx]);
                    scratch.segment_indices.push(scratch.inlier_indices[idx]);
                }

                // Try to fit line
                if let Some(refined_line) = fit_line(&scratch.segment_points)
                    && refined_line.length() >= config.min_line_length
                {
                    scratch.output_lines.push(refined_line);
                    added_any = true;

                    // Mark segment inliers as consumed
                    for j in 0..scratch.segment_indices.len() {
                        let idx = scratch.segment_indices[j];
                        scratch.remaining_mask[idx] = false;
                    }
                }
            }

            segment_start = i + 1;
        }
    }

    added_any
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_config_default() {
        let config = RansacLineConfig::default();
        assert_eq!(config.iterations, 100);
        assert_relative_eq!(config.inlier_threshold, 0.02);
        assert_eq!(config.min_inliers, 5);
    }

    #[test]
    fn test_config_builder() {
        let config = RansacLineConfig::new()
            .with_iterations(50)
            .with_inlier_threshold(0.05)
            .with_min_inliers(3)
            .with_seed(42);

        assert_eq!(config.iterations, 50);
        assert_relative_eq!(config.inlier_threshold, 0.05);
        assert_eq!(config.min_inliers, 3);
        assert_eq!(config.seed, 42);
    }

    #[test]
    fn test_extract_empty_points() {
        let config = RansacLineConfig::default();
        let (lines, remaining) = extract_lines_ransac(&[], &config);

        assert!(lines.is_empty());
        assert!(remaining.is_empty());
    }

    #[test]
    fn test_extract_too_few_points() {
        let points = vec![Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)];
        let config = RansacLineConfig::default().with_min_inliers(5);
        let (lines, remaining) = extract_lines_ransac(&points, &config);

        assert!(lines.is_empty());
        assert_eq!(remaining.len(), 2);
    }

    #[test]
    fn test_extract_single_line() {
        // Points along a horizontal line (5cm spacing, under 10cm gap threshold)
        let points: Vec<Point2D> = (0..20)
            .map(|i| Point2D::new(i as f32 * 0.05, 0.0))
            .collect();

        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_min_inliers(5);

        let (lines, remaining) = extract_lines_ransac(&points, &config);

        assert_eq!(lines.len(), 1);
        // Most points should be consumed
        assert!(remaining.len() < 5);
        // Line should be roughly horizontal
        let angle = lines[0].angle().abs();
        assert!(angle < 0.1 || (std::f32::consts::PI - angle).abs() < 0.1);
    }

    #[test]
    fn test_extract_two_lines() {
        // Points along a horizontal line (5cm spacing)
        let mut points: Vec<Point2D> = (0..15)
            .map(|i| Point2D::new(i as f32 * 0.05, 0.0))
            .collect();

        // Points along a vertical line (5cm spacing)
        for i in 0..15 {
            points.push(Point2D::new(0.0, i as f32 * 0.05));
        }

        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_min_inliers(5)
            .with_max_lines(2);

        let (lines, _remaining) = extract_lines_ransac(&points, &config);

        assert_eq!(lines.len(), 2);
    }

    #[test]
    fn test_extract_with_noise() {
        // Points along a horizontal line with some noise (5cm spacing)
        let mut points: Vec<Point2D> = (0..20)
            .map(|i| Point2D::new(i as f32 * 0.05, (i as f32 * 0.05).sin() * 0.01))
            .collect();

        // Add some outliers
        points.push(Point2D::new(0.5, 1.0));
        points.push(Point2D::new(1.0, -1.0));

        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_inlier_threshold(0.03)
            .with_min_inliers(5);

        let (lines, _remaining) = extract_lines_ransac(&points, &config);

        // Should find the main line despite outliers
        assert!(!lines.is_empty());
    }

    #[test]
    fn test_max_lines_limit() {
        // Many points that could form multiple lines (5cm spacing)
        let mut points: Vec<Point2D> = Vec::new();
        for j in 0..4 {
            for i in 0..10 {
                points.push(Point2D::new(i as f32 * 0.05, j as f32 * 0.5));
            }
        }

        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_min_inliers(5)
            .with_max_lines(2);

        let (lines, _remaining) = extract_lines_ransac(&points, &config);

        assert!(lines.len() <= 2);
    }

    #[test]
    fn test_gap_splitting_no_gap() {
        // Continuous line with no gaps (3cm spacing, well under 10cm threshold)
        let points: Vec<Point2D> = (0..20)
            .map(|i| Point2D::new(i as f32 * 0.03, 0.0)) // 3cm spacing
            .collect();
        let indices: Vec<usize> = (0..20).collect();

        let config = RansacLineConfig::default().with_max_point_gap(0.10);

        let segments = split_at_gaps(&points, &indices, &config);

        // Should result in a single segment (all points within gap threshold)
        assert_eq!(segments.len(), 1);
        assert_eq!(segments[0].0.len(), 20);
    }

    #[test]
    fn test_gap_splitting_with_gap() {
        // Two wall segments separated by a doorway (gap > 10cm)
        let mut points = Vec::new();
        // First wall segment: 5 points with 3cm spacing
        for i in 0..5 {
            points.push(Point2D::new(i as f32 * 0.03, 0.0));
        }
        // Second wall segment: starts at 0.35m (gap of ~23cm from last point)
        for i in 0..5 {
            points.push(Point2D::new(0.35 + i as f32 * 0.03, 0.0));
        }
        let indices: Vec<usize> = (0..10).collect();

        let config = RansacLineConfig::default().with_max_point_gap(0.10);

        let segments = split_at_gaps(&points, &indices, &config);

        // Should split into two segments due to gap
        assert_eq!(segments.len(), 2);
        assert_eq!(segments[0].0.len(), 5);
        assert_eq!(segments[1].0.len(), 5);
    }

    #[test]
    fn test_gap_splitting_multiple_gaps() {
        // Three wall segments with gaps
        let mut points = Vec::new();
        // Segment 1: 3 points with 3cm spacing (0.0 to 0.06m)
        for i in 0..3 {
            points.push(Point2D::new(i as f32 * 0.03, 0.0));
        }
        // Gap of ~19cm
        // Segment 2: 3 points starting at 0.25m
        for i in 0..3 {
            points.push(Point2D::new(0.25 + i as f32 * 0.03, 0.0));
        }
        // Gap of ~19cm
        // Segment 3: 3 points starting at 0.50m
        for i in 0..3 {
            points.push(Point2D::new(0.50 + i as f32 * 0.03, 0.0));
        }
        let indices: Vec<usize> = (0..9).collect();

        let config = RansacLineConfig::default().with_max_point_gap(0.10);

        let segments = split_at_gaps(&points, &indices, &config);

        // Should split into three segments
        assert_eq!(segments.len(), 3);
        for segment in &segments {
            assert_eq!(segment.0.len(), 3);
        }
    }

    #[test]
    fn test_extract_lines_respects_gap() {
        // Collinear points with a gap (simulating doorway between walls)
        let mut points = Vec::new();
        // Left wall segment: 10 points with 3cm spacing (0.27m length)
        for i in 0..10 {
            points.push(Point2D::new(i as f32 * 0.03, 0.0));
        }
        // Gap of ~25cm
        // Right wall segment: 10 points with 3cm spacing starting at 0.52m
        for i in 0..10 {
            points.push(Point2D::new(0.52 + i as f32 * 0.03, 0.0));
        }

        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_min_inliers(5)
            .with_max_point_gap(0.10);

        let (lines, _remaining) = extract_lines_ransac(&points, &config);

        // Should find two separate lines, not one long line
        assert_eq!(lines.len(), 2);

        // Check each line is approximately the expected length
        for line in &lines {
            // Each wall segment is ~0.27m (10 points * 0.03m spacing)
            assert!(line.length() < 0.35, "Line too long: {}", line.length());
            assert!(line.length() > 0.15, "Line too short: {}", line.length());
        }
    }

    // =========================================================================
    // Tests for scratch space version
    // =========================================================================

    #[test]
    fn test_scratch_space_creation() {
        let scratch = RansacScratchSpace::with_capacity(400);
        assert!(scratch.remaining_mask.capacity() >= 400);
        assert!(scratch.current_inliers.capacity() >= 400);
    }

    #[test]
    fn test_scratch_extract_single_line() {
        // Same test as non-scratch version
        let points: Vec<Point2D> = (0..20)
            .map(|i| Point2D::new(i as f32 * 0.05, 0.0))
            .collect();

        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_min_inliers(5);

        let mut scratch = RansacScratchSpace::with_capacity(400);
        let (lines, remaining) = extract_lines_ransac_with_scratch(&points, &config, &mut scratch);

        assert_eq!(lines.len(), 1);
        assert!(remaining.len() < 5);
        let angle = lines[0].angle().abs();
        assert!(angle < 0.1 || (std::f32::consts::PI - angle).abs() < 0.1);
    }

    #[test]
    fn test_scratch_extract_two_lines() {
        let mut points: Vec<Point2D> = (0..15)
            .map(|i| Point2D::new(i as f32 * 0.05, 0.0))
            .collect();

        for i in 0..15 {
            points.push(Point2D::new(0.0, i as f32 * 0.05));
        }

        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_min_inliers(5)
            .with_max_lines(2);

        let mut scratch = RansacScratchSpace::with_capacity(400);
        let (lines, _remaining) = extract_lines_ransac_with_scratch(&points, &config, &mut scratch);

        assert_eq!(lines.len(), 2);
    }

    #[test]
    fn test_scratch_reuse() {
        // Test that scratch space can be reused across multiple calls
        let mut scratch = RansacScratchSpace::with_capacity(400);

        // First extraction
        let points1: Vec<Point2D> = (0..20)
            .map(|i| Point2D::new(i as f32 * 0.05, 0.0))
            .collect();
        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_min_inliers(5);

        let (lines1, _) = extract_lines_ransac_with_scratch(&points1, &config, &mut scratch);
        assert_eq!(lines1.len(), 1);

        // Second extraction with different points - reusing same scratch
        let points2: Vec<Point2D> = (0..20)
            .map(|i| Point2D::new(0.0, i as f32 * 0.05))
            .collect();

        let (lines2, _) = extract_lines_ransac_with_scratch(&points2, &config, &mut scratch);
        assert_eq!(lines2.len(), 1);

        // Verify the second extraction is independent
        let angle2 = lines2[0].angle().abs();
        let expected_vertical = std::f32::consts::FRAC_PI_2;
        assert!((angle2 - expected_vertical).abs() < 0.1);
    }

    #[test]
    fn test_scratch_equivalence_to_original() {
        // Verify scratch version produces same results as original
        let points: Vec<Point2D> = (0..25)
            .map(|i| Point2D::new(i as f32 * 0.04, 0.0))
            .collect();

        let config = RansacLineConfig::default()
            .with_seed(123)
            .with_min_inliers(5)
            .with_iterations(50);

        // Original version
        let (lines_orig, remaining_orig) = extract_lines_ransac(&points, &config);

        // Scratch version
        let mut scratch = RansacScratchSpace::with_capacity(100);
        let (lines_scratch, remaining_scratch) =
            extract_lines_ransac_with_scratch(&points, &config, &mut scratch);

        // Same number of lines
        assert_eq!(lines_orig.len(), lines_scratch.len());
        // Same number of remaining points
        assert_eq!(remaining_orig.len(), remaining_scratch.len());
    }

    #[test]
    fn test_scratch_gap_splitting() {
        // Collinear points with a gap
        let mut points = Vec::new();
        for i in 0..10 {
            points.push(Point2D::new(i as f32 * 0.03, 0.0));
        }
        for i in 0..10 {
            points.push(Point2D::new(0.52 + i as f32 * 0.03, 0.0));
        }

        let config = RansacLineConfig::default()
            .with_seed(42)
            .with_min_inliers(5)
            .with_max_point_gap(0.10);

        let mut scratch = RansacScratchSpace::with_capacity(100);
        let (lines, _) = extract_lines_ransac_with_scratch(&points, &config, &mut scratch);

        assert_eq!(lines.len(), 2);
        for line in lines {
            assert!(line.length() < 0.35);
            assert!(line.length() > 0.15);
        }
    }

    #[test]
    fn test_scratch_empty_input() {
        let points: Vec<Point2D> = vec![];
        let config = RansacLineConfig::default();
        let mut scratch = RansacScratchSpace::with_capacity(100);

        let (lines, remaining) = extract_lines_ransac_with_scratch(&points, &config, &mut scratch);

        assert!(lines.is_empty());
        assert!(remaining.is_empty());
    }

    #[test]
    fn test_scratch_too_few_points() {
        let points = vec![Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)];
        let config = RansacLineConfig::default().with_min_inliers(5);
        let mut scratch = RansacScratchSpace::with_capacity(100);

        let (lines, remaining) = extract_lines_ransac_with_scratch(&points, &config, &mut scratch);

        assert!(lines.is_empty());
        assert_eq!(remaining.len(), 2);
    }
}
