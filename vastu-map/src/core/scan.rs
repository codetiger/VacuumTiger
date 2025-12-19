//! Scan types for lidar data representation.
//!
//! This module provides:
//! - `PolarScan`: Raw lidar scan in polar coordinates
//! - `PointCloud2D`: Cartesian point cloud with SoA (Struct of Arrays) layout
//!
//! The SoA layout is optimized for SIMD operations on ARM NEON.

use super::point::Point2D;
use super::pose::Pose2D;
use crate::simd::Float4;

/// Raw lidar scan in polar coordinates.
///
/// Points are ordered by angle (sequential around scan).
/// Coordinate frame follows ROS REP-103:
/// - Angle 0 is forward (X-axis)
/// - Positive angles are counter-clockwise (toward Y-axis)
#[derive(Clone, Debug, Default)]
pub struct PolarScan {
    /// Points as (angle_rad, distance_m, quality).
    /// - angle_rad: Angle from X-axis in radians, CCW positive
    /// - distance_m: Range in meters
    /// - quality: Measurement quality (0-255, higher is better)
    pub points: Vec<(f32, f32, u8)>,
}

impl PolarScan {
    /// Create a new empty polar scan.
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Create a polar scan with capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
        }
    }

    /// Add a point to the scan.
    #[inline]
    pub fn push(&mut self, angle: f32, distance: f32, quality: u8) {
        self.points.push((angle, distance, quality));
    }

    /// Number of points in the scan.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if the scan is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Convert to cartesian PointCloud2D.
    ///
    /// Filters by quality threshold and valid range.
    ///
    /// # Arguments
    /// * `min_quality` - Minimum quality value to include
    /// * `min_range` - Minimum valid range in meters
    /// * `max_range` - Maximum valid range in meters
    pub fn to_cartesian(&self, min_quality: u8, min_range: f32, max_range: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(self.points.len());

        for &(angle, dist, quality) in &self.points {
            if quality >= min_quality && dist >= min_range && dist <= max_range {
                let (sin, cos) = angle.sin_cos();
                cloud.xs.push(dist * cos);
                cloud.ys.push(dist * sin);
            }
        }

        cloud
    }
}

/// Cartesian point cloud with SoA (Struct of Arrays) layout.
///
/// This layout enables efficient SIMD operations:
/// - Transform operations can process 4 points per NEON instruction
/// - Memory access patterns are cache-friendly for sequential processing
///
/// Coordinate frame follows ROS REP-103:
/// - X-forward, Y-left
#[derive(Clone, Debug, Default)]
pub struct PointCloud2D {
    /// X coordinates in meters (forward direction).
    pub xs: Vec<f32>,
    /// Y coordinates in meters (left direction).
    pub ys: Vec<f32>,
}

impl PointCloud2D {
    /// Create a new empty point cloud.
    pub fn new() -> Self {
        Self {
            xs: Vec::new(),
            ys: Vec::new(),
        }
    }

    /// Create a point cloud with capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            xs: Vec::with_capacity(capacity),
            ys: Vec::with_capacity(capacity),
        }
    }

    /// Create from a slice of Point2D.
    pub fn from_points(points: &[Point2D]) -> Self {
        let mut cloud = Self::with_capacity(points.len());
        for p in points {
            cloud.xs.push(p.x);
            cloud.ys.push(p.y);
        }
        cloud
    }

    /// Add a point to the cloud.
    #[inline]
    pub fn push(&mut self, x: f32, y: f32) {
        self.xs.push(x);
        self.ys.push(y);
    }

    /// Add a Point2D to the cloud.
    #[inline]
    pub fn push_point(&mut self, point: Point2D) {
        self.xs.push(point.x);
        self.ys.push(point.y);
    }

    /// Number of points in the cloud.
    #[inline]
    pub fn len(&self) -> usize {
        self.xs.len()
    }

    /// Check if the cloud is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.xs.is_empty()
    }

    /// Get a point by index.
    #[inline]
    pub fn get(&self, index: usize) -> Option<Point2D> {
        if index < self.len() {
            Some(Point2D::new(self.xs[index], self.ys[index]))
        } else {
            None
        }
    }

    /// Get a point by index (unchecked).
    ///
    /// # Safety
    /// Caller must ensure index is in bounds.
    #[inline]
    pub unsafe fn get_unchecked(&self, index: usize) -> Point2D {
        // SAFETY: Caller guarantees index is in bounds
        unsafe { Point2D::new(*self.xs.get_unchecked(index), *self.ys.get_unchecked(index)) }
    }

    /// Transform the point cloud by a pose.
    ///
    /// This is SIMD-optimized: processes 4 points per iteration using Float4.
    ///
    /// # Performance
    /// Uses fused multiply-add (FMA) operations which map directly to
    /// ARM NEON `vfmaq_f32` instructions on VFPv4 hardware.
    pub fn transform(&self, pose: &Pose2D) -> Self {
        let n = self.len();
        let mut result = Self::with_capacity(n);
        result.xs.resize(n, 0.0);
        result.ys.resize(n, 0.0);

        let (sin, cos) = pose.theta.sin_cos();
        let cos4 = Float4::splat(cos);
        let sin4 = Float4::splat(sin);
        let tx4 = Float4::splat(pose.x);
        let ty4 = Float4::splat(pose.y);

        // Process 4 points at a time (SIMD-friendly)
        let chunks = n / 4;
        for i in 0..chunks {
            let base = i * 4;

            // Load 4 points
            let xs = Float4::new([
                self.xs[base],
                self.xs[base + 1],
                self.xs[base + 2],
                self.xs[base + 3],
            ]);
            let ys = Float4::new([
                self.ys[base],
                self.ys[base + 1],
                self.ys[base + 2],
                self.ys[base + 3],
            ]);

            // Rotation + translation using FMA:
            // x' = x*cos - y*sin + tx
            // y' = x*sin + y*cos + ty
            let new_xs = ys.neg_mul_add(sin4, xs.mul_add(cos4, tx4));
            let new_ys = xs.mul_add(sin4, ys.mul_add(cos4, ty4));

            // Store results
            let new_xs_arr = new_xs.to_array();
            let new_ys_arr = new_ys.to_array();
            result.xs[base..base + 4].copy_from_slice(&new_xs_arr);
            result.ys[base..base + 4].copy_from_slice(&new_ys_arr);
        }

        // Handle remainder (scalar)
        for i in (chunks * 4)..n {
            let x = self.xs[i];
            let y = self.ys[i];
            result.xs[i] = x * cos - y * sin + pose.x;
            result.ys[i] = x * sin + y * cos + pose.y;
        }

        result
    }

    /// Compute bounding box of the point cloud.
    ///
    /// Returns (min, max) corners, or None if cloud is empty.
    /// SIMD-optimized using Float4 min/max operations.
    pub fn bounds(&self) -> Option<(Point2D, Point2D)> {
        if self.is_empty() {
            return None;
        }

        let n = self.len();

        // Initialize with first point
        let mut min_x = self.xs[0];
        let mut max_x = self.xs[0];
        let mut min_y = self.ys[0];
        let mut max_y = self.ys[0];

        // SIMD processing of remaining points
        let chunks = (n - 1) / 4;
        if chunks > 0 {
            let mut min_x4 = Float4::splat(min_x);
            let mut max_x4 = Float4::splat(max_x);
            let mut min_y4 = Float4::splat(min_y);
            let mut max_y4 = Float4::splat(max_y);

            for i in 0..chunks {
                let base = 1 + i * 4;

                let xs = Float4::new([
                    self.xs[base],
                    self.xs[base + 1],
                    self.xs[base + 2],
                    self.xs[base + 3],
                ]);
                let ys = Float4::new([
                    self.ys[base],
                    self.ys[base + 1],
                    self.ys[base + 2],
                    self.ys[base + 3],
                ]);

                min_x4 = min_x4.min(xs);
                max_x4 = max_x4.max(xs);
                min_y4 = min_y4.min(ys);
                max_y4 = max_y4.max(ys);
            }

            // Horizontal reduction
            min_x = min_x4.horizontal_min();
            max_x = max_x4.horizontal_max();
            min_y = min_y4.horizontal_min();
            max_y = max_y4.horizontal_max();
        }

        // Handle remainder (scalar)
        let remainder_start = 1 + chunks * 4;
        for i in remainder_start..n {
            min_x = min_x.min(self.xs[i]);
            max_x = max_x.max(self.xs[i]);
            min_y = min_y.min(self.ys[i]);
            max_y = max_y.max(self.ys[i]);
        }

        Some((Point2D::new(min_x, min_y), Point2D::new(max_x, max_y)))
    }

    /// Compute centroid (mean position) of the point cloud.
    ///
    /// Returns None if cloud is empty.
    pub fn centroid(&self) -> Option<Point2D> {
        if self.is_empty() {
            return None;
        }

        let n = self.len();
        let mut sum_x: f32 = 0.0;
        let mut sum_y: f32 = 0.0;

        // SIMD accumulation
        let chunks = n / 4;
        if chunks > 0 {
            let mut sum_x4 = Float4::zero();
            let mut sum_y4 = Float4::zero();

            for i in 0..chunks {
                let base = i * 4;
                let xs = Float4::new([
                    self.xs[base],
                    self.xs[base + 1],
                    self.xs[base + 2],
                    self.xs[base + 3],
                ]);
                let ys = Float4::new([
                    self.ys[base],
                    self.ys[base + 1],
                    self.ys[base + 2],
                    self.ys[base + 3],
                ]);

                sum_x4 += xs;
                sum_y4 += ys;
            }

            sum_x = sum_x4.sum();
            sum_y = sum_y4.sum();
        }

        // Handle remainder
        for i in (chunks * 4)..n {
            sum_x += self.xs[i];
            sum_y += self.ys[i];
        }

        Some(Point2D::new(sum_x / n as f32, sum_y / n as f32))
    }

    /// Convert to Vec<Point2D> (AoS layout).
    pub fn to_points(&self) -> Vec<Point2D> {
        self.xs
            .iter()
            .zip(&self.ys)
            .map(|(&x, &y)| Point2D::new(x, y))
            .collect()
    }

    /// Iterate over points.
    pub fn iter(&self) -> impl Iterator<Item = Point2D> + '_ {
        self.xs
            .iter()
            .zip(&self.ys)
            .map(|(&x, &y)| Point2D::new(x, y))
    }

    /// Clear all points.
    pub fn clear(&mut self) {
        self.xs.clear();
        self.ys.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn test_polar_scan_to_cartesian() {
        let mut scan = PolarScan::new();
        scan.push(0.0, 1.0, 100); // Forward, 1m
        scan.push(FRAC_PI_2, 2.0, 100); // Left, 2m
        scan.push(PI, 1.5, 50); // Behind, 1.5m, low quality

        let cloud = scan.to_cartesian(60, 0.0, 10.0);

        assert_eq!(cloud.len(), 2); // Third point filtered by quality

        // First point: (1, 0)
        assert_relative_eq!(cloud.xs[0], 1.0, epsilon = 1e-6);
        assert_relative_eq!(cloud.ys[0], 0.0, epsilon = 1e-6);

        // Second point: (0, 2)
        assert_relative_eq!(cloud.xs[1], 0.0, epsilon = 1e-6);
        assert_relative_eq!(cloud.ys[1], 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_polar_scan_range_filter() {
        let mut scan = PolarScan::new();
        scan.push(0.0, 0.05, 100); // Too close
        scan.push(0.0, 0.5, 100); // Good
        scan.push(0.0, 15.0, 100); // Too far

        let cloud = scan.to_cartesian(0, 0.1, 10.0);
        assert_eq!(cloud.len(), 1);
        assert_relative_eq!(cloud.xs[0], 0.5, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_from_points() {
        let points = vec![
            Point2D::new(1.0, 2.0),
            Point2D::new(3.0, 4.0),
            Point2D::new(5.0, 6.0),
        ];

        let cloud = PointCloud2D::from_points(&points);

        assert_eq!(cloud.len(), 3);
        assert_eq!(cloud.xs, vec![1.0, 3.0, 5.0]);
        assert_eq!(cloud.ys, vec![2.0, 4.0, 6.0]);
    }

    #[test]
    fn test_point_cloud_transform_identity() {
        let mut cloud = PointCloud2D::new();
        cloud.push(1.0, 2.0);
        cloud.push(3.0, 4.0);

        let pose = Pose2D::identity();
        let transformed = cloud.transform(&pose);

        assert_eq!(transformed.len(), 2);
        assert_relative_eq!(transformed.xs[0], 1.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.ys[0], 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_transform_translation() {
        let mut cloud = PointCloud2D::new();
        cloud.push(1.0, 2.0);

        let pose = Pose2D::new(10.0, 20.0, 0.0);
        let transformed = cloud.transform(&pose);

        assert_relative_eq!(transformed.xs[0], 11.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.ys[0], 22.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_transform_rotation() {
        let mut cloud = PointCloud2D::new();
        cloud.push(1.0, 0.0);

        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let transformed = cloud.transform(&pose);

        assert_relative_eq!(transformed.xs[0], 0.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.ys[0], 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_transform_combined() {
        let mut cloud = PointCloud2D::new();
        cloud.push(1.0, 0.0);

        // Rotate 90° then translate by (1, 0)
        let pose = Pose2D::new(1.0, 0.0, FRAC_PI_2);
        let transformed = cloud.transform(&pose);

        // Point rotates to (0, 1), then translates to (1, 1)
        assert_relative_eq!(transformed.xs[0], 1.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.ys[0], 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_transform_simd() {
        // Test with more than 4 points to exercise SIMD path
        let mut cloud = PointCloud2D::new();
        for i in 0..10 {
            cloud.push(i as f32, 0.0);
        }

        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let transformed = cloud.transform(&pose);

        for i in 0..10 {
            assert_relative_eq!(transformed.xs[i], 0.0, epsilon = 1e-5);
            assert_relative_eq!(transformed.ys[i], i as f32, epsilon = 1e-5);
        }
    }

    #[test]
    fn test_point_cloud_bounds() {
        let mut cloud = PointCloud2D::new();
        cloud.push(1.0, 2.0);
        cloud.push(-3.0, 4.0);
        cloud.push(5.0, -6.0);
        cloud.push(0.0, 0.0);

        let (min, max) = cloud.bounds().unwrap();

        assert_eq!(min.x, -3.0);
        assert_eq!(min.y, -6.0);
        assert_eq!(max.x, 5.0);
        assert_eq!(max.y, 4.0);
    }

    #[test]
    fn test_point_cloud_bounds_simd() {
        // Test with more than 4 points
        let mut cloud = PointCloud2D::new();
        for i in 0..10 {
            cloud.push(i as f32 - 5.0, i as f32 - 5.0);
        }

        let (min, max) = cloud.bounds().unwrap();

        assert_eq!(min.x, -5.0);
        assert_eq!(min.y, -5.0);
        assert_eq!(max.x, 4.0);
        assert_eq!(max.y, 4.0);
    }

    #[test]
    fn test_point_cloud_bounds_empty() {
        let cloud = PointCloud2D::new();
        assert!(cloud.bounds().is_none());
    }

    #[test]
    fn test_point_cloud_centroid() {
        let mut cloud = PointCloud2D::new();
        cloud.push(0.0, 0.0);
        cloud.push(2.0, 0.0);
        cloud.push(2.0, 2.0);
        cloud.push(0.0, 2.0);

        let centroid = cloud.centroid().unwrap();

        assert_relative_eq!(centroid.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(centroid.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_centroid_simd() {
        // Test with more than 4 points
        let mut cloud = PointCloud2D::new();
        for _ in 0..8 {
            cloud.push(4.0, 6.0);
        }

        let centroid = cloud.centroid().unwrap();

        assert_relative_eq!(centroid.x, 4.0, epsilon = 1e-6);
        assert_relative_eq!(centroid.y, 6.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_centroid_empty() {
        let cloud = PointCloud2D::new();
        assert!(cloud.centroid().is_none());
    }

    #[test]
    fn test_point_cloud_iter() {
        let mut cloud = PointCloud2D::new();
        cloud.push(1.0, 2.0);
        cloud.push(3.0, 4.0);

        let points: Vec<_> = cloud.iter().collect();

        assert_eq!(points.len(), 2);
        assert_eq!(points[0], Point2D::new(1.0, 2.0));
        assert_eq!(points[1], Point2D::new(3.0, 4.0));
    }

    #[test]
    fn test_point_cloud_get() {
        let mut cloud = PointCloud2D::new();
        cloud.push(1.0, 2.0);
        cloud.push(3.0, 4.0);

        assert_eq!(cloud.get(0), Some(Point2D::new(1.0, 2.0)));
        assert_eq!(cloud.get(1), Some(Point2D::new(3.0, 4.0)));
        assert_eq!(cloud.get(2), None);
    }
}
