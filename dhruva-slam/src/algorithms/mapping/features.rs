//! Feature extraction from occupancy grids.
//!
//! Extracts geometric features (lines, corners) from occupancy grid maps
//! for use in scan-to-feature matching, map simplification, and loop closure.

use rand::Rng;
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::core::simd::Float4;
use crate::core::types::Point2D;

use super::{MapRegion, OccupancyGrid};

// ============================================================================
// Data Structures
// ============================================================================

/// A line segment in 2D space representing a wall.
#[derive(Debug, Clone)]
pub struct LineSegment {
    /// Start point of the segment
    pub start: Point2D,
    /// End point of the segment
    pub end: Point2D,
    /// Normal vector x component (from ax + by + c = 0)
    pub normal_x: f32,
    /// Normal vector y component
    pub normal_y: f32,
    /// Distance from origin (c in ax + by + c = 0)
    pub c: f32,
    /// Segment length in meters
    pub length: f32,
    /// Fit quality (R² value, 0-1)
    pub quality: f32,
    /// Number of points used to fit this line
    pub point_count: usize,
}

impl LineSegment {
    /// Compute signed distance from a point to the infinite line.
    #[inline]
    pub fn distance_to_line(&self, p: &Point2D) -> f32 {
        self.normal_x * p.x + self.normal_y * p.y + self.c
    }

    /// Check if a point projects onto this segment (between start and end).
    pub fn point_projects_onto_segment(&self, p: &Point2D) -> bool {
        let dx = self.end.x - self.start.x;
        let dy = self.end.y - self.start.y;
        let len_sq = dx * dx + dy * dy;
        if len_sq < 1e-10 {
            return false;
        }

        let t = ((p.x - self.start.x) * dx + (p.y - self.start.y) * dy) / len_sq;
        (0.0..=1.0).contains(&t)
    }

    /// Get the direction vector of the line (unit length).
    pub fn direction(&self) -> (f32, f32) {
        // Direction is perpendicular to normal
        (-self.normal_y, self.normal_x)
    }

    /// Get the midpoint of the segment.
    pub fn midpoint(&self) -> Point2D {
        Point2D::new(
            (self.start.x + self.end.x) / 2.0,
            (self.start.y + self.end.y) / 2.0,
        )
    }
}

/// Type of corner junction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CornerType {
    /// L-junction: ~90° corner (80°-100°)
    LJunction,
    /// T-junction: one line ends at middle of another
    TJunction,
    /// X-junction: two lines cross
    XJunction,
    /// Acute corner: < 80°
    Acute,
    /// Obtuse corner: > 100°
    Obtuse,
}

/// A corner where two walls meet.
#[derive(Debug, Clone)]
pub struct Corner {
    /// Corner position (intersection point)
    pub position: Point2D,
    /// Angle between walls in radians (0 to π)
    pub angle: f32,
    /// Index of first line in MapFeatures.lines
    pub line1_idx: usize,
    /// Index of second line in MapFeatures.lines
    pub line2_idx: usize,
    /// Corner classification
    pub corner_type: CornerType,
}

/// Collection of extracted map features.
#[derive(Debug, Clone, Default)]
pub struct MapFeatures {
    /// Extracted line segments (walls)
    pub lines: Vec<LineSegment>,
    /// Detected corners (line intersections)
    pub corners: Vec<Corner>,
}

impl MapFeatures {
    /// Create empty features.
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if features are empty.
    pub fn is_empty(&self) -> bool {
        self.lines.is_empty() && self.corners.is_empty()
    }

    /// Total number of features.
    pub fn len(&self) -> usize {
        self.lines.len() + self.corners.len()
    }
}

// ============================================================================
// Configuration
// ============================================================================

/// Configuration for feature extraction.
#[derive(Debug, Clone)]
pub struct FeatureExtractorConfig {
    /// Minimum line segment length in meters (default: 0.3m)
    pub min_line_length: f32,
    /// Minimum R² quality for accepting a line fit (default: 0.85)
    pub min_line_quality: f32,
    /// Number of RANSAC iterations (default: 100)
    pub ransac_iterations: usize,
    /// Maximum distance from line for inliers in meters (default: 0.03m)
    pub ransac_threshold: f32,
    /// Minimum inliers to form a line (default: 10)
    pub min_inliers: usize,
    /// Maximum endpoint gap for corner detection in meters (default: 0.15m)
    pub corner_distance_threshold: f32,
    /// Angle tolerance for L-junction classification in radians (default: 10°)
    pub corner_angle_tolerance: f32,
}

impl Default for FeatureExtractorConfig {
    fn default() -> Self {
        Self {
            min_line_length: 0.3,
            min_line_quality: 0.85,
            ransac_iterations: 100,
            ransac_threshold: 0.03,
            min_inliers: 10,
            corner_distance_threshold: 0.15,
            corner_angle_tolerance: 10.0_f32.to_radians(),
        }
    }
}

// ============================================================================
// Feature Extractor
// ============================================================================

/// Extracts geometric features from occupancy grids.
pub struct FeatureExtractor {
    config: FeatureExtractorConfig,
    rng: StdRng,
}

impl FeatureExtractor {
    /// Create a new feature extractor with the given configuration.
    pub fn new(config: FeatureExtractorConfig) -> Self {
        Self {
            config,
            rng: StdRng::from_os_rng(),
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(FeatureExtractorConfig::default())
    }

    /// Extract all features from an occupancy grid.
    pub fn extract(&mut self, grid: &OccupancyGrid) -> MapFeatures {
        let points = grid.occupied_points();
        self.extract_from_points(&points)
    }

    /// Extract features from a specific region of the grid.
    pub fn extract_region(&mut self, grid: &OccupancyGrid, region: &MapRegion) -> MapFeatures {
        let points = grid.occupied_points_in_region(region);
        self.extract_from_points(&points)
    }

    /// Extract features from a set of points.
    pub fn extract_from_points(&mut self, points: &[Point2D]) -> MapFeatures {
        if points.len() < self.config.min_inliers {
            return MapFeatures::new();
        }

        // Extract lines using RANSAC
        let lines = self.extract_lines(points);

        // Detect corners from line intersections
        let corners = self.extract_corners(&lines);

        MapFeatures { lines, corners }
    }

    // ========================================================================
    // Line Extraction (RANSAC)
    // ========================================================================

    /// Extract line segments using RANSAC.
    fn extract_lines(&mut self, points: &[Point2D]) -> Vec<LineSegment> {
        let mut lines = Vec::new();
        let mut remaining: Vec<Point2D> = points.to_vec();

        // Keep extracting lines until not enough points remain
        while remaining.len() >= self.config.min_inliers {
            if let Some((line, inlier_indices)) = self.ransac_line(&remaining) {
                // Only keep lines that meet quality threshold
                if line.quality >= self.config.min_line_quality
                    && line.length >= self.config.min_line_length
                {
                    lines.push(line);
                }

                // Remove inliers from remaining points (in reverse order to preserve indices)
                let mut indices: Vec<_> = inlier_indices.into_iter().collect();
                indices.sort_unstable_by(|a, b| b.cmp(a));
                for idx in indices {
                    remaining.swap_remove(idx);
                }
            } else {
                break;
            }
        }

        // Merge collinear segments
        self.merge_collinear_lines(&mut lines);

        lines
    }

    /// RANSAC line fitting - returns best line and inlier indices.
    fn ransac_line(&mut self, points: &[Point2D]) -> Option<(LineSegment, Vec<usize>)> {
        if points.len() < 2 {
            return None;
        }

        let mut best_inliers: Vec<usize> = Vec::new();
        let mut best_score = 0usize;

        for _ in 0..self.config.ransac_iterations {
            // Randomly sample 2 points
            let idx1 = self.rng.random_range(0..points.len());
            let mut idx2 = self.rng.random_range(0..points.len());
            while idx2 == idx1 {
                idx2 = self.rng.random_range(0..points.len());
            }

            let p1 = &points[idx1];
            let p2 = &points[idx2];

            // Fit line through these two points
            if let Some((a, b, c)) = Self::line_from_two_points(p1, p2) {
                // Find inliers
                let mut inliers = Vec::new();
                for (i, p) in points.iter().enumerate() {
                    let dist = (a * p.x + b * p.y + c).abs();
                    if dist <= self.config.ransac_threshold {
                        inliers.push(i);
                    }
                }

                if inliers.len() > best_score && inliers.len() >= self.config.min_inliers {
                    best_score = inliers.len();
                    best_inliers = inliers;
                }
            }
        }

        if best_inliers.len() < self.config.min_inliers {
            return None;
        }

        // Refit line using all inliers with eigenvalue method
        let inlier_points: Vec<Point2D> = best_inliers.iter().map(|&i| points[i]).collect();

        self.fit_line_eigen(&inlier_points)
            .map(|line| (line, best_inliers))
    }

    /// Compute line parameters (a, b, c) from two points.
    /// Returns normalized (a, b, c) where a² + b² = 1.
    fn line_from_two_points(p1: &Point2D, p2: &Point2D) -> Option<(f32, f32, f32)> {
        let dx = p2.x - p1.x;
        let dy = p2.y - p1.y;
        let len = (dx * dx + dy * dy).sqrt();

        if len < 1e-10 {
            return None;
        }

        // Normal is perpendicular to direction
        let a = -dy / len;
        let b = dx / len;
        let c = -(a * p1.x + b * p1.y);

        Some((a, b, c))
    }

    /// Fit a line to points using eigenvalue decomposition.
    /// Returns a LineSegment with computed endpoints.
    fn fit_line_eigen(&self, points: &[Point2D]) -> Option<LineSegment> {
        if points.len() < 2 {
            return None;
        }

        let n = points.len();
        // Use reciprocal multiplication (faster than division on ARM)
        let inv_n = 1.0 / (n as f32);

        // Compute centroid
        let cx: f32 = points.iter().map(|p| p.x).sum::<f32>() * inv_n;
        let cy: f32 = points.iter().map(|p| p.y).sum::<f32>() * inv_n;

        // Compute covariance matrix using SIMD for chunks of 4 points
        let cx_v = Float4::splat(cx);
        let cy_v = Float4::splat(cy);
        let mut sxx_v = Float4::splat(0.0);
        let mut syy_v = Float4::splat(0.0);
        let mut sxy_v = Float4::splat(0.0);

        let chunks = points.len() / 4;
        for i in 0..chunks {
            let base = i * 4;
            let xs = Float4::new([
                points[base].x,
                points[base + 1].x,
                points[base + 2].x,
                points[base + 3].x,
            ]);
            let ys = Float4::new([
                points[base].y,
                points[base + 1].y,
                points[base + 2].y,
                points[base + 3].y,
            ]);

            let dxs = xs - cx_v;
            let dys = ys - cy_v;

            sxx_v += dxs * dxs;
            syy_v += dys * dys;
            sxy_v += dxs * dys;
        }

        // Reduce SIMD accumulators
        let mut sxx = sxx_v.reduce_add();
        let mut syy = syy_v.reduce_add();
        let mut sxy = sxy_v.reduce_add();

        // Handle remainder with scalar
        for p in points.iter().skip(chunks * 4) {
            let dx = p.x - cx;
            let dy = p.y - cy;
            sxx += dx * dx;
            syy += dy * dy;
            sxy += dx * dy;
        }

        // Eigenvalue decomposition
        let trace = sxx + syy;
        let det = sxx * syy - sxy * sxy;
        let discriminant = (trace * trace / 4.0 - det).max(0.0);
        let sqrt_disc = discriminant.sqrt();

        let lambda1 = trace / 2.0 + sqrt_disc; // Larger eigenvalue
        let lambda2 = trace / 2.0 - sqrt_disc; // Smaller eigenvalue

        // Quality is ratio of eigenvalues
        let quality = if lambda1 > 1e-10 {
            1.0 - (lambda2 / lambda1)
        } else {
            0.0
        };

        // Normal vector is eigenvector of smaller eigenvalue
        let (a, b) = if sxy.abs() > 1e-10 {
            (sxy, lambda2 - sxx)
        } else if sxx > syy {
            (0.0, 1.0) // Horizontal line
        } else {
            (1.0, 0.0) // Vertical line
        };

        // Normalize
        let norm = (a * a + b * b).sqrt();
        if norm < 1e-10 {
            return None;
        }

        let a = a / norm;
        let b = b / norm;
        let c = -(a * cx + b * cy);

        // Compute endpoints by projecting all points onto the line
        let (start, end) = self.compute_endpoints(points, a, b, c);
        let length = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();

        Some(LineSegment {
            start,
            end,
            normal_x: a,
            normal_y: b,
            c,
            length,
            quality,
            point_count: points.len(),
        })
    }

    /// Compute line segment endpoints by projecting points onto the line.
    fn compute_endpoints(&self, points: &[Point2D], a: f32, b: f32, c: f32) -> (Point2D, Point2D) {
        // Line direction is perpendicular to normal (a, b)
        let dir_x = -b;
        let dir_y = a;

        // Project all points onto the line and find extremes
        let mut min_t = f32::MAX;
        let mut max_t = f32::MIN;

        // Use centroid as reference point on the line
        let n = points.len() as f32;
        let cx: f32 = points.iter().map(|p| p.x).sum::<f32>() / n;
        let cy: f32 = points.iter().map(|p| p.y).sum::<f32>() / n;

        for p in points {
            // Project point onto line
            let dist = a * p.x + b * p.y + c;
            let proj_x = p.x - a * dist;
            let proj_y = p.y - b * dist;

            // Parameter t along line direction from centroid
            let t = (proj_x - cx) * dir_x + (proj_y - cy) * dir_y;

            min_t = min_t.min(t);
            max_t = max_t.max(t);
        }

        let start = Point2D::new(cx + dir_x * min_t, cy + dir_y * min_t);
        let end = Point2D::new(cx + dir_x * max_t, cy + dir_y * max_t);

        (start, end)
    }

    /// Merge collinear line segments that are close together.
    fn merge_collinear_lines(&self, lines: &mut Vec<LineSegment>) {
        if lines.len() < 2 {
            return;
        }

        let mut merged = true;
        while merged {
            merged = false;
            let mut i = 0;
            while i < lines.len() {
                let mut j = i + 1;
                while j < lines.len() {
                    if self.should_merge(&lines[i], &lines[j]) {
                        // Merge j into i
                        let merged_line = self.merge_lines(&lines[i], &lines[j]);
                        lines[i] = merged_line;
                        lines.swap_remove(j);
                        merged = true;
                    } else {
                        j += 1;
                    }
                }
                i += 1;
            }
        }
    }

    /// Check if two lines should be merged (collinear and close).
    fn should_merge(&self, l1: &LineSegment, l2: &LineSegment) -> bool {
        // Check if normals are parallel (or anti-parallel)
        let dot = l1.normal_x * l2.normal_x + l1.normal_y * l2.normal_y;
        if dot.abs() < 0.98 {
            // ~11 degree tolerance
            return false;
        }

        // Check if lines are close (distance between centroids projected to normal)
        let mid1 = l1.midpoint();
        let mid2 = l2.midpoint();
        let dist = (l1.distance_to_line(&mid2).abs() + l2.distance_to_line(&mid1).abs()) / 2.0;
        if dist > self.config.ransac_threshold * 2.0 {
            return false;
        }

        // Check if endpoints are close enough to merge
        let gap = self.endpoint_gap(l1, l2);
        gap < self.config.corner_distance_threshold
    }

    /// Compute minimum gap between endpoints of two line segments.
    fn endpoint_gap(&self, l1: &LineSegment, l2: &LineSegment) -> f32 {
        let distances = [
            Self::point_distance(&l1.end, &l2.start),
            Self::point_distance(&l1.end, &l2.end),
            Self::point_distance(&l1.start, &l2.start),
            Self::point_distance(&l1.start, &l2.end),
        ];
        distances.into_iter().fold(f32::MAX, f32::min)
    }

    /// Merge two line segments into one.
    fn merge_lines(&self, l1: &LineSegment, l2: &LineSegment) -> LineSegment {
        // Use weighted average for line parameters
        let w1 = l1.point_count as f32;
        let w2 = l2.point_count as f32;
        let total = w1 + w2;

        let a = (l1.normal_x * w1 + l2.normal_x * w2) / total;
        let b = (l1.normal_y * w1 + l2.normal_y * w2) / total;
        let norm = (a * a + b * b).sqrt();
        let a = a / norm;
        let b = b / norm;

        // Find new endpoints (extreme points from both segments)
        let points = [l1.start, l1.end, l2.start, l2.end];
        let mid_x = points.iter().map(|p| p.x).sum::<f32>() / 4.0;
        let mid_y = points.iter().map(|p| p.y).sum::<f32>() / 4.0;
        let c = -(a * mid_x + b * mid_y);

        let (start, end) = self.compute_endpoints(&points, a, b, c);
        let length = Self::point_distance(&start, &end);

        LineSegment {
            start,
            end,
            normal_x: a,
            normal_y: b,
            c,
            length,
            quality: (l1.quality * w1 + l2.quality * w2) / total,
            point_count: l1.point_count + l2.point_count,
        }
    }

    #[inline]
    fn point_distance(p1: &Point2D, p2: &Point2D) -> f32 {
        ((p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2)).sqrt()
    }

    // ========================================================================
    // Corner Detection
    // ========================================================================

    /// Detect corners from line intersections.
    fn extract_corners(&self, lines: &[LineSegment]) -> Vec<Corner> {
        let mut corners = Vec::new();

        for i in 0..lines.len() {
            for j in (i + 1)..lines.len() {
                if let Some(corner) = self.detect_corner(&lines[i], &lines[j], i, j) {
                    corners.push(corner);
                }
            }
        }

        // Remove duplicate corners (close positions)
        self.deduplicate_corners(&mut corners);

        corners
    }

    /// Detect a corner between two line segments.
    fn detect_corner(
        &self,
        l1: &LineSegment,
        l2: &LineSegment,
        idx1: usize,
        idx2: usize,
    ) -> Option<Corner> {
        // Check if any endpoints are close
        let endpoints1 = [&l1.start, &l1.end];
        let endpoints2 = [&l2.start, &l2.end];

        let mut closest_dist = f32::MAX;
        let mut closest_pair = (0, 0);

        for (i, e1) in endpoints1.iter().enumerate() {
            for (j, e2) in endpoints2.iter().enumerate() {
                let dist = Self::point_distance(e1, e2);
                if dist < closest_dist {
                    closest_dist = dist;
                    closest_pair = (i, j);
                }
            }
        }

        // Only detect corner if endpoints are close enough
        if closest_dist > self.config.corner_distance_threshold {
            return None;
        }

        // Compute intersection point
        let intersection = self.line_intersection(l1, l2)?;

        // Compute angle between lines
        let angle = self.angle_between_lines(l1, l2);

        // Classify corner type
        let corner_type = self.classify_corner(angle, l1, l2, &intersection, closest_pair);

        Some(Corner {
            position: intersection,
            angle,
            line1_idx: idx1,
            line2_idx: idx2,
            corner_type,
        })
    }

    /// Compute intersection point of two infinite lines.
    fn line_intersection(&self, l1: &LineSegment, l2: &LineSegment) -> Option<Point2D> {
        // Lines: a1*x + b1*y + c1 = 0 and a2*x + b2*y + c2 = 0
        // Using Cramer's rule
        let det = l1.normal_x * l2.normal_y - l2.normal_x * l1.normal_y;

        if det.abs() < 1e-10 {
            return None; // Parallel lines
        }

        let x = (l1.normal_y * l2.c - l2.normal_y * l1.c) / det;
        let y = (l2.normal_x * l1.c - l1.normal_x * l2.c) / det;

        Some(Point2D::new(x, y))
    }

    /// Compute angle between two lines (0 to π).
    fn angle_between_lines(&self, l1: &LineSegment, l2: &LineSegment) -> f32 {
        // Angle between normals
        let dot = l1.normal_x * l2.normal_x + l1.normal_y * l2.normal_y;
        let angle = dot.abs().acos();

        // Return the acute angle
        if angle > std::f32::consts::FRAC_PI_2 {
            std::f32::consts::PI - angle
        } else {
            angle
        }
    }

    /// Classify corner type based on angle and geometry.
    fn classify_corner(
        &self,
        angle: f32,
        l1: &LineSegment,
        l2: &LineSegment,
        intersection: &Point2D,
        _closest_endpoints: (usize, usize),
    ) -> CornerType {
        let right_angle = std::f32::consts::FRAC_PI_2;
        let tolerance = self.config.corner_angle_tolerance;

        // Check if intersection is near endpoints or in the middle of lines
        let on_l1 = l1.point_projects_onto_segment(intersection);
        let on_l2 = l2.point_projects_onto_segment(intersection);

        // X-junction: both lines continue through intersection
        if on_l1 && on_l2 {
            return CornerType::XJunction;
        }

        // T-junction: one line continues, other ends
        if on_l1 || on_l2 {
            return CornerType::TJunction;
        }

        // L-junction or angled corner
        if (angle - right_angle).abs() < tolerance {
            CornerType::LJunction
        } else if angle < right_angle - tolerance {
            CornerType::Acute
        } else {
            CornerType::Obtuse
        }
    }

    /// Remove duplicate corners that are too close together.
    fn deduplicate_corners(&self, corners: &mut Vec<Corner>) {
        let threshold = self.config.corner_distance_threshold;

        let mut i = 0;
        while i < corners.len() {
            let mut j = i + 1;
            while j < corners.len() {
                let dist = Self::point_distance(&corners[i].position, &corners[j].position);
                if dist < threshold {
                    // Keep the one with more right-angle (prefer L-junctions)
                    if corners[j].corner_type == CornerType::LJunction
                        && corners[i].corner_type != CornerType::LJunction
                    {
                        corners.swap(i, j);
                    }
                    corners.swap_remove(j);
                } else {
                    j += 1;
                }
            }
            i += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_from_two_points() {
        // Horizontal line y = 1
        let p1 = Point2D::new(0.0, 1.0);
        let p2 = Point2D::new(2.0, 1.0);
        let (a, b, c) = FeatureExtractor::line_from_two_points(&p1, &p2).unwrap();

        // Normal should be (0, 1) or (0, -1)
        assert!(a.abs() < 0.01);
        assert!((b.abs() - 1.0).abs() < 0.01);

        // Test distance from origin to line
        let dist = (a * 0.0 + b * 0.0 + c).abs();
        assert!((dist - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_line_segment_distance() {
        let line = LineSegment {
            start: Point2D::new(0.0, 0.0),
            end: Point2D::new(2.0, 0.0),
            normal_x: 0.0,
            normal_y: 1.0,
            c: 0.0,
            length: 2.0,
            quality: 1.0,
            point_count: 10,
        };

        // Point above line
        let p = Point2D::new(1.0, 0.5);
        let dist = line.distance_to_line(&p);
        assert!((dist - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_extract_simple_lines() {
        let mut extractor = FeatureExtractor::with_defaults();

        // Create points along two perpendicular walls
        let mut points = Vec::new();

        // Horizontal wall y = 0
        for i in 0..20 {
            points.push(Point2D::new(i as f32 * 0.05, 0.0));
        }

        // Vertical wall x = 1
        for i in 0..20 {
            points.push(Point2D::new(1.0, i as f32 * 0.05));
        }

        let features = extractor.extract_from_points(&points);

        // Should extract 2 lines
        assert!(
            features.lines.len() >= 2,
            "Expected 2 lines, got {}",
            features.lines.len()
        );

        // Should detect 1 corner (L-junction)
        assert!(!features.corners.is_empty(), "Expected at least 1 corner");
    }
}
