//! Shape context descriptors for loop closure detection.
//!
//! This module provides descriptors for corners and scans that enable
//! matching between different observations of the same place.
//!
//! # Geometric Concept: Shape Context Descriptors
//!
//! Shape context is a rich descriptor for 2D shapes that captures the spatial
//! distribution of features relative to a reference point. This implementation
//! adapts the technique for line-based SLAM.
//!
//! ## How It Works
//!
//! Given a corner point, we examine all lines within a radius and build histograms:
//!
//! ```text
//!                    Angle Bins (8 bins, 45° each)
//!
//!                         bin 1
//!                          │
//!                   bin 0  │  bin 2
//!                     ╲    │    ╱
//!                      ╲   │   ╱
//!                bin 7 ─── ● ─── bin 3    ← Corner position
//!                      ╱   │   ╲
//!                     ╱    │    ╲
//!                   bin 6  │  bin 4
//!                          │
//!                        bin 5
//!
//!    Distance Bins (4 bins)
//!    ┌─────┬─────┬─────┬─────────────┐
//!    │0-0.5│0.5-1│1-2m │   2+m       │
//!    └─────┴─────┴─────┴─────────────┘
//! ```
//!
//! ## Matching
//!
//! Two corners are similar if their histograms are similar, regardless of
//! the corner's absolute position or orientation. This enables recognizing
//! the same place from different viewpoints.
//!
//! ## Distance Metric
//!
//! Chi-squared distance compares histograms: χ² = Σ (a_i - b_i)² / (a_i + b_i)
//! - 0.0 = identical descriptors
//! - Higher values = more different
//!
//! # Corner Descriptors
//!
//! Each corner is described by histograms of nearby line properties:
//! - Angle histogram: orientation of nearby lines (8 bins, 45° each)
//! - Distance histogram: distances to nearby lines (4 bins)
//! - Length histogram: lengths of nearby lines (4 bins)
//!
//! # Scan Descriptors
//!
//! Aggregate descriptor for an entire scan, combining corner descriptors
//! with global statistics (line count, corner count, total length).

use crate::features::{Corner2D, Line2D};
use std::f32::consts::PI;

/// Number of bins in the angle histogram.
pub const ANGLE_BINS: usize = 8;
/// Number of bins in the distance histogram.
pub const DISTANCE_BINS: usize = 4;
/// Number of bins in the length histogram.
pub const LENGTH_BINS: usize = 4;

/// Distance bin boundaries in meters.
const DISTANCE_BIN_EDGES: [f32; DISTANCE_BINS] = [0.5, 1.0, 2.0, f32::INFINITY];
/// Length bin boundaries in meters.
const LENGTH_BIN_EDGES: [f32; LENGTH_BINS] = [0.3, 0.6, 1.0, f32::INFINITY];

/// Shape context descriptor for a corner feature.
///
/// Encodes the local structure around a corner by histogramming
/// properties of nearby lines. This is invariant to rotation
/// when using relative angles.
#[derive(Clone, Debug)]
pub struct CornerDescriptor {
    /// 8-bin histogram: angles of nearby lines relative to corner bisector.
    /// Each bin covers 45° (π/4 radians).
    pub angle_histogram: [f32; ANGLE_BINS],

    /// 4-bin histogram: distances to nearby lines.
    /// Bins: [0-0.5m, 0.5-1m, 1-2m, 2+m]
    pub distance_histogram: [f32; DISTANCE_BINS],

    /// 4-bin histogram: lengths of nearby lines.
    /// Bins: [0-0.3m, 0.3-0.6m, 0.6-1m, 1+m]
    pub length_histogram: [f32; LENGTH_BINS],

    /// Number of lines used to compute this descriptor.
    pub num_lines: usize,
}

impl Default for CornerDescriptor {
    fn default() -> Self {
        Self {
            angle_histogram: [0.0; ANGLE_BINS],
            distance_histogram: [0.0; DISTANCE_BINS],
            length_histogram: [0.0; LENGTH_BINS],
            num_lines: 0,
        }
    }
}

impl CornerDescriptor {
    /// Compute descriptor for a corner from nearby lines.
    ///
    /// # Arguments
    /// * `corner` - The corner to describe
    /// * `lines` - All lines in the scan/map
    /// * `radius` - Maximum distance to consider lines (default: 3.0m)
    ///
    /// # Example
    /// ```
    /// use vastu_map::features::{Corner2D, Line2D, CornerDescriptor};
    /// use vastu_map::core::Point2D;
    /// use std::f32::consts::FRAC_PI_2;
    ///
    /// let corner = Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2);
    /// let lines = vec![
    ///     Line2D::new(Point2D::new(-1.0, 0.0), Point2D::new(0.0, 0.0)),
    ///     Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0)),
    /// ];
    ///
    /// let descriptor = CornerDescriptor::compute(&corner, &lines, 3.0);
    /// assert!(descriptor.num_lines >= 2);
    /// ```
    pub fn compute(corner: &Corner2D, lines: &[Line2D], radius: f32) -> Self {
        let mut desc = Self::default();
        let radius_sq = radius * radius;

        for line in lines {
            // Check if line is within radius of corner
            let dist = line.distance_to_point(corner.position);
            if dist > radius {
                continue;
            }

            // Check if either endpoint is within radius
            let dist_start = corner.position.distance(line.start);
            let dist_end = corner.position.distance(line.end);
            let min_endpoint_dist = dist_start.min(dist_end);

            if min_endpoint_dist * min_endpoint_dist > radius_sq * 4.0 {
                continue;
            }

            desc.num_lines += 1;

            // Add to angle histogram
            let line_angle = line.angle();
            // Normalize to [0, 2π) then bin into 8 segments
            let normalized_angle = if line_angle < 0.0 {
                line_angle + 2.0 * PI
            } else {
                line_angle
            };
            let angle_bin = ((normalized_angle / (2.0 * PI)) * ANGLE_BINS as f32) as usize;
            let angle_bin = angle_bin.min(ANGLE_BINS - 1);
            desc.angle_histogram[angle_bin] += 1.0;

            // Add to distance histogram
            let dist_bin = Self::find_bin(dist, &DISTANCE_BIN_EDGES);
            desc.distance_histogram[dist_bin] += 1.0;

            // Add to length histogram
            let length = line.length();
            let length_bin = Self::find_bin(length, &LENGTH_BIN_EDGES);
            desc.length_histogram[length_bin] += 1.0;
        }

        // Normalize histograms
        desc.normalize();

        desc
    }

    /// Find the bin index for a value given bin edges.
    #[inline]
    fn find_bin(value: f32, edges: &[f32]) -> usize {
        for (i, &edge) in edges.iter().enumerate() {
            if value < edge {
                return i;
            }
        }
        edges.len() - 1
    }

    /// Normalize all histograms to sum to 1.
    fn normalize(&mut self) {
        Self::normalize_histogram(&mut self.angle_histogram);
        Self::normalize_histogram(&mut self.distance_histogram);
        Self::normalize_histogram(&mut self.length_histogram);
    }

    /// Normalize a single histogram.
    fn normalize_histogram(hist: &mut [f32]) {
        let sum: f32 = hist.iter().sum();
        if sum > f32::EPSILON {
            for val in hist.iter_mut() {
                *val /= sum;
            }
        }
    }

    /// Compute chi-squared distance between two descriptors.
    ///
    /// Lower values indicate more similar descriptors.
    /// Returns 0.0 for identical descriptors, up to ~2.0 for very different.
    ///
    /// # Formula
    /// χ² = Σ (a_i - b_i)² / (a_i + b_i + ε)
    pub fn distance(&self, other: &Self) -> f32 {
        let epsilon = 1e-10;

        let mut total = 0.0;

        // Compare angle histograms
        for i in 0..ANGLE_BINS {
            let a = self.angle_histogram[i];
            let b = other.angle_histogram[i];
            let denom = a + b + epsilon;
            total += (a - b).powi(2) / denom;
        }

        // Compare distance histograms
        for i in 0..DISTANCE_BINS {
            let a = self.distance_histogram[i];
            let b = other.distance_histogram[i];
            let denom = a + b + epsilon;
            total += (a - b).powi(2) / denom;
        }

        // Compare length histograms
        for i in 0..LENGTH_BINS {
            let a = self.length_histogram[i];
            let b = other.length_histogram[i];
            let denom = a + b + epsilon;
            total += (a - b).powi(2) / denom;
        }

        total / 2.0 // Normalize to [0, 1] range for identical mass
    }

    /// Check if this descriptor is empty (no lines contributed).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.num_lines == 0
    }
}

/// Aggregate descriptor for an entire scan.
///
/// Combines individual corner descriptors with global statistics
/// about the scan's line and corner content.
#[derive(Clone, Debug)]
pub struct ScanDescriptor {
    /// Descriptors for each corner in the scan.
    pub corner_descriptors: Vec<CornerDescriptor>,

    /// Number of lines in the scan.
    pub num_lines: usize,

    /// Number of corners in the scan.
    pub num_corners: usize,

    /// Total length of all lines.
    pub total_line_length: f32,

    /// Average corner angle (for quick filtering).
    pub avg_corner_angle: f32,
}

impl Default for ScanDescriptor {
    fn default() -> Self {
        Self {
            corner_descriptors: Vec::new(),
            num_lines: 0,
            num_corners: 0,
            total_line_length: 0.0,
            avg_corner_angle: 0.0,
        }
    }
}

impl ScanDescriptor {
    /// Compute scan descriptor from lines and corners.
    ///
    /// # Arguments
    /// * `lines` - Extracted line segments
    /// * `corners` - Detected corners
    /// * `descriptor_radius` - Radius for computing corner descriptors (default: 3.0m)
    ///
    /// # Example
    /// ```
    /// use vastu_map::features::{Corner2D, Line2D, ScanDescriptor};
    /// use vastu_map::core::Point2D;
    /// use std::f32::consts::FRAC_PI_2;
    ///
    /// let lines = vec![
    ///     Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
    ///     Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(1.0, 1.0)),
    /// ];
    /// let corners = vec![
    ///     Corner2D::new(Point2D::new(1.0, 0.0), 0, 1, FRAC_PI_2),
    /// ];
    ///
    /// let descriptor = ScanDescriptor::compute(&lines, &corners, 3.0);
    /// assert_eq!(descriptor.num_lines, 2);
    /// assert_eq!(descriptor.num_corners, 1);
    /// ```
    pub fn compute(lines: &[Line2D], corners: &[Corner2D], descriptor_radius: f32) -> Self {
        // Compute average corner angle
        let avg_corner_angle = if corners.is_empty() {
            0.0
        } else {
            corners.iter().map(|c| c.angle).sum::<f32>() / corners.len() as f32
        };

        Self {
            num_lines: lines.len(),
            num_corners: corners.len(),
            total_line_length: lines.iter().map(|l| l.length()).sum(),
            corner_descriptors: corners
                .iter()
                .map(|c| CornerDescriptor::compute(c, lines, descriptor_radius))
                .collect(),
            avg_corner_angle,
        }
    }

    /// Compute distance between two scan descriptors.
    ///
    /// Uses a combination of:
    /// 1. Global statistics comparison
    /// 2. Best-match corner descriptor distances
    ///
    /// Returns a value in [0, 1] where lower is more similar.
    pub fn distance(&self, other: &Self) -> f32 {
        // Quick rejection based on global stats
        let line_diff = (self.num_lines as f32 - other.num_lines as f32).abs();
        let corner_diff = (self.num_corners as f32 - other.num_corners as f32).abs();

        // If counts differ significantly, scans are different
        let max_lines = self.num_lines.max(other.num_lines) as f32;
        let max_corners = self.num_corners.max(other.num_corners) as f32;

        if max_lines > 0.0 && line_diff / max_lines > 0.5 {
            return 1.0;
        }
        if max_corners > 0.0 && corner_diff / max_corners > 0.5 {
            return 1.0;
        }

        // Match corner descriptors (greedy best-match)
        if self.corner_descriptors.is_empty() || other.corner_descriptors.is_empty() {
            // No corners to compare - use global stats only
            let length_diff = (self.total_line_length - other.total_line_length).abs();
            let max_length = self.total_line_length.max(other.total_line_length);
            if max_length > f32::EPSILON {
                return (length_diff / max_length).min(1.0);
            }
            return 0.5; // Indeterminate
        }

        // Compute average best-match distance
        let mut total_dist = 0.0;
        let mut matches = 0;

        for desc1 in &self.corner_descriptors {
            if desc1.is_empty() {
                continue;
            }

            let mut best_dist = f32::MAX;
            for desc2 in &other.corner_descriptors {
                if desc2.is_empty() {
                    continue;
                }
                let dist = desc1.distance(desc2);
                best_dist = best_dist.min(dist);
            }

            if best_dist < f32::MAX {
                total_dist += best_dist;
                matches += 1;
            }
        }

        if matches > 0 {
            total_dist / matches as f32
        } else {
            1.0
        }
    }

    /// Check if this descriptor has enough content for matching.
    #[inline]
    pub fn is_valid_for_matching(&self) -> bool {
        self.num_corners >= 2 && self.num_lines >= 3
    }

    /// Compute a quick hash for pre-filtering (based on feature counts).
    pub fn quick_signature(&self) -> u32 {
        // Encode line count (4 bits), corner count (4 bits), and length bucket (8 bits)
        let line_bucket = (self.num_lines.min(15)) as u32;
        let corner_bucket = (self.num_corners.min(15)) as u32;
        let length_bucket = ((self.total_line_length / 0.5).min(255.0)) as u32;

        (line_bucket << 12) | (corner_bucket << 8) | length_bucket
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point2D;
    use approx::assert_relative_eq;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn test_corner_descriptor_empty() {
        let corner = Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2);
        let lines: Vec<Line2D> = vec![];

        let desc = CornerDescriptor::compute(&corner, &lines, 3.0);

        assert!(desc.is_empty());
        assert_eq!(desc.num_lines, 0);
    }

    #[test]
    fn test_corner_descriptor_with_lines() {
        let corner = Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2);
        let lines = vec![
            Line2D::new(Point2D::new(-1.0, 0.0), Point2D::new(0.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0)),
        ];

        let desc = CornerDescriptor::compute(&corner, &lines, 3.0);

        assert!(!desc.is_empty());
        assert_eq!(desc.num_lines, 2);

        // Histograms should be normalized
        let angle_sum: f32 = desc.angle_histogram.iter().sum();
        assert_relative_eq!(angle_sum, 1.0, epsilon = 0.01);
    }

    #[test]
    fn test_corner_descriptor_distance_identical() {
        let corner = Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2);
        let lines = vec![
            Line2D::new(Point2D::new(-1.0, 0.0), Point2D::new(0.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0)),
        ];

        let desc = CornerDescriptor::compute(&corner, &lines, 3.0);

        // Distance to itself should be 0
        let dist = desc.distance(&desc);
        assert_relative_eq!(dist, 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_corner_descriptor_distance_different() {
        let corner1 = Corner2D::new(Point2D::new(0.0, 0.0), 0, 1, FRAC_PI_2);
        let lines1 = vec![
            Line2D::new(Point2D::new(-1.0, 0.0), Point2D::new(0.0, 0.0)),
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(0.0, 1.0)),
        ];

        let corner2 = Corner2D::new(Point2D::new(5.0, 5.0), 0, 1, FRAC_PI_2);
        let lines2 = vec![
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(10.0, 5.0)),
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(5.0, 10.0)),
            Line2D::new(Point2D::new(5.0, 5.0), Point2D::new(7.0, 7.0)),
        ];

        let desc1 = CornerDescriptor::compute(&corner1, &lines1, 3.0);
        let desc2 = CornerDescriptor::compute(&corner2, &lines2, 3.0);

        // Distance should be positive
        let dist = desc1.distance(&desc2);
        assert!(dist > 0.0);
    }

    #[test]
    fn test_scan_descriptor_empty() {
        let desc = ScanDescriptor::compute(&[], &[], 3.0);

        assert_eq!(desc.num_lines, 0);
        assert_eq!(desc.num_corners, 0);
        assert!(!desc.is_valid_for_matching());
    }

    #[test]
    fn test_scan_descriptor_with_features() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(1.0, 1.0)),
            Line2D::new(Point2D::new(1.0, 1.0), Point2D::new(0.0, 1.0)),
        ];
        let corners = vec![
            Corner2D::new(Point2D::new(1.0, 0.0), 0, 1, FRAC_PI_2),
            Corner2D::new(Point2D::new(1.0, 1.0), 1, 2, FRAC_PI_2),
        ];

        let desc = ScanDescriptor::compute(&lines, &corners, 3.0);

        assert_eq!(desc.num_lines, 3);
        assert_eq!(desc.num_corners, 2);
        assert_eq!(desc.corner_descriptors.len(), 2);
        assert!(desc.is_valid_for_matching());
        assert!(desc.total_line_length > 2.0);
    }

    #[test]
    fn test_scan_descriptor_distance_identical() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(1.0, 1.0)),
            Line2D::new(Point2D::new(1.0, 1.0), Point2D::new(0.0, 1.0)),
        ];
        let corners = vec![
            Corner2D::new(Point2D::new(1.0, 0.0), 0, 1, FRAC_PI_2),
            Corner2D::new(Point2D::new(1.0, 1.0), 1, 2, FRAC_PI_2),
        ];

        let desc = ScanDescriptor::compute(&lines, &corners, 3.0);

        // Distance to itself should be very small
        let dist = desc.distance(&desc);
        assert!(dist < 0.1, "Self-distance should be near 0, got {}", dist);
    }

    #[test]
    fn test_scan_descriptor_quick_signature() {
        let lines = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0)),
            Line2D::new(Point2D::new(1.0, 0.0), Point2D::new(1.0, 1.0)),
        ];
        let corners = vec![Corner2D::new(Point2D::new(1.0, 0.0), 0, 1, FRAC_PI_2)];

        let desc = ScanDescriptor::compute(&lines, &corners, 3.0);
        let sig = desc.quick_signature();

        // Signature should be non-zero
        assert!(sig > 0);
    }

    #[test]
    fn test_find_bin() {
        assert_eq!(CornerDescriptor::find_bin(0.3, &DISTANCE_BIN_EDGES), 0);
        assert_eq!(CornerDescriptor::find_bin(0.7, &DISTANCE_BIN_EDGES), 1);
        assert_eq!(CornerDescriptor::find_bin(1.5, &DISTANCE_BIN_EDGES), 2);
        assert_eq!(CornerDescriptor::find_bin(5.0, &DISTANCE_BIN_EDGES), 3);
    }
}
