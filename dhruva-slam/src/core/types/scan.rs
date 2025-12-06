//! LiDAR scan and point cloud types.

use super::pose::{Point2D, Pose2D};
use crate::core::simd::Float4;
use serde::{Deserialize, Serialize};

/// Raw LiDAR scan in polar coordinates.
///
/// Represents a single 360° (or partial) scan from a 2D LiDAR sensor.
/// Each measurement is a range value at a specific angle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LaserScan {
    /// Start angle in radians
    pub angle_min: f32,
    /// End angle in radians
    pub angle_max: f32,
    /// Angular resolution (radians between consecutive readings)
    /// Note: This is only used for scans with uniform spacing.
    /// For non-uniform scans (from raw lidar data), use the `angles` field.
    pub angle_increment: f32,
    /// Minimum valid range in meters
    pub range_min: f32,
    /// Maximum valid range in meters
    pub range_max: f32,
    /// Range measurements in meters (0 or NaN = invalid)
    pub ranges: Vec<f32>,
    /// Optional intensity values (0-255)
    pub intensities: Option<Vec<u8>>,
    /// Optional per-point angles (for non-uniform spacing).
    /// When present, these exact angles are used instead of computing from angle_increment.
    /// This preserves the original lidar measurement geometry for better matching accuracy.
    pub angles: Option<Vec<f32>>,
}

impl LaserScan {
    /// Create a new laser scan with the given parameters.
    pub fn new(
        angle_min: f32,
        angle_max: f32,
        angle_increment: f32,
        range_min: f32,
        range_max: f32,
        ranges: Vec<f32>,
    ) -> Self {
        Self {
            angle_min,
            angle_max,
            angle_increment,
            range_min,
            range_max,
            ranges,
            intensities: None,
            angles: None,
        }
    }

    /// Create a laser scan with intensities.
    pub fn with_intensities(mut self, intensities: Vec<u8>) -> Self {
        self.intensities = Some(intensities);
        self
    }

    /// Convert from SangamIO LidarScan format.
    ///
    /// SangamIO provides Vec<(angle_rad, distance_m, quality)>.
    /// This converts to our LaserScan format, preserving the original
    /// per-point angles for accurate scan matching.
    pub fn from_lidar_scan(scan: &crate::io::LidarScan) -> Self {
        if scan.is_empty() {
            return Self {
                angle_min: 0.0,
                angle_max: 0.0,
                angle_increment: 0.0,
                range_min: 0.0,
                range_max: 0.0,
                ranges: Vec::new(),
                intensities: None,
                angles: None,
            };
        }

        // Sort by angle to ensure proper ordering
        let mut sorted: Vec<_> = scan.to_vec();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let angle_min = sorted.first().map(|p| p.0).unwrap_or(0.0);
        let angle_max = sorted.last().map(|p| p.0).unwrap_or(0.0);

        // Estimate angle increment for legacy compatibility, but we'll use actual angles
        let angle_increment = if sorted.len() > 1 {
            (angle_max - angle_min) / (sorted.len() - 1) as f32
        } else {
            0.0
        };

        // Preserve the ACTUAL angles from the lidar - this is critical for accuracy!
        let angles: Vec<f32> = sorted.iter().map(|p| p.0).collect();
        let ranges: Vec<f32> = sorted.iter().map(|p| p.1).collect();
        let intensities: Vec<u8> = sorted.iter().map(|p| p.2).collect();

        Self {
            angle_min,
            angle_max,
            angle_increment,
            range_min: 0.15, // Delta-2D typical minimum
            range_max: 12.0, // Delta-2D typical maximum
            ranges,
            intensities: Some(intensities),
            angles: Some(angles), // Use original angles for better accuracy
        }
    }

    /// Number of range measurements.
    #[inline]
    pub fn len(&self) -> usize {
        self.ranges.len()
    }

    /// Check if scan is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.ranges.is_empty()
    }

    /// Get the angle for a given index (computes from angle_increment).
    ///
    /// For scans with non-uniform spacing, prefer `get_angle()` which
    /// uses actual per-point angles when available.
    #[inline]
    pub fn angle_at(&self, index: usize) -> f32 {
        self.angle_min + index as f32 * self.angle_increment
    }

    /// Get the actual angle for a given index.
    ///
    /// Uses the per-point angles if available (from `angles` field),
    /// otherwise falls back to computing from `angle_increment`.
    /// This is the preferred method for accessing angles as it preserves
    /// the original lidar measurement geometry.
    #[inline]
    pub fn get_angle(&self, index: usize) -> f32 {
        self.angles
            .as_ref()
            .and_then(|a| a.get(index).copied())
            .unwrap_or_else(|| self.angle_at(index))
    }

    /// Check if a range value is valid.
    #[inline]
    pub fn is_valid_range(&self, range: f32) -> bool {
        range.is_finite() && range > 0.0 && range >= self.range_min && range <= self.range_max
    }

    /// Iterate over (angle, range, intensity) tuples.
    ///
    /// When `angles` is present, uses the actual per-point angles.
    /// Otherwise, computes angles from `angle_min` and `angle_increment`.
    pub fn iter(&self) -> impl Iterator<Item = (f32, f32, u8)> + '_ {
        let intensities = &self.intensities;
        self.ranges.iter().enumerate().map(move |(i, &range)| {
            let angle = self.get_angle(i);
            let intensity = intensities
                .as_ref()
                .and_then(|v| v.get(i).copied())
                .unwrap_or(0);
            (angle, range, intensity)
        })
    }

    /// Iterate over valid points only (filters out invalid ranges).
    pub fn iter_valid(&self) -> impl Iterator<Item = (f32, f32, u8)> + '_ {
        self.iter()
            .filter(|(_, range, _)| self.is_valid_range(*range))
    }

    /// Count valid points.
    pub fn valid_count(&self) -> usize {
        self.ranges
            .iter()
            .filter(|&&r| self.is_valid_range(r))
            .count()
    }
}

impl Default for LaserScan {
    fn default() -> Self {
        Self {
            angle_min: 0.0,
            angle_max: std::f32::consts::TAU, // 2π for full 360°
            angle_increment: std::f32::consts::TAU / 360.0, // 1° resolution
            range_min: 0.15,
            range_max: 12.0,
            ranges: Vec::new(),
            intensities: None,
            angles: None,
        }
    }
}

/// SIMD-optimized collection of 2D points using Struct of Arrays (SoA) layout.
///
/// Instead of `Vec<Point2D>` (x,y,x,y,x,y...), stores:
/// - `xs: Vec<f32>` (x,x,x,x...)
/// - `ys: Vec<f32>` (y,y,y,y...)
///
/// This enables processing 4 points per SIMD instruction for significant
/// performance gains in transform, centroid, and bounds operations.
///
/// Used as the output of scan preprocessing and input to scan matching.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct PointCloud2D {
    /// X coordinates in meters (SoA layout for SIMD)
    pub xs: Vec<f32>,
    /// Y coordinates in meters (SoA layout for SIMD)
    pub ys: Vec<f32>,
    /// Optional intensity values (same length as xs/ys)
    pub intensities: Option<Vec<u8>>,
}

impl PointCloud2D {
    /// Create an empty point cloud.
    pub fn new() -> Self {
        Self {
            xs: Vec::new(),
            ys: Vec::new(),
            intensities: None,
        }
    }

    /// Create a point cloud with pre-allocated capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            xs: Vec::with_capacity(capacity),
            ys: Vec::with_capacity(capacity),
            intensities: None,
        }
    }

    /// Create from a vector of points (converts AoS to SoA).
    pub fn from_points(points: Vec<Point2D>) -> Self {
        let n = points.len();
        let mut xs = Vec::with_capacity(n);
        let mut ys = Vec::with_capacity(n);
        for p in points {
            xs.push(p.x);
            ys.push(p.y);
        }
        Self {
            xs,
            ys,
            intensities: None,
        }
    }

    /// Add a point without intensity.
    #[inline]
    pub fn push(&mut self, point: Point2D) {
        self.xs.push(point.x);
        self.ys.push(point.y);
        if let Some(ref mut intensities) = self.intensities {
            intensities.push(0);
        }
    }

    /// Add a point by x, y coordinates directly (faster than push).
    #[inline]
    pub fn push_xy(&mut self, x: f32, y: f32) {
        self.xs.push(x);
        self.ys.push(y);
        if let Some(ref mut intensities) = self.intensities {
            intensities.push(0);
        }
    }

    /// Add a point with intensity.
    #[inline]
    pub fn push_with_intensity(&mut self, point: Point2D, intensity: u8) {
        self.xs.push(point.x);
        self.ys.push(point.y);
        if self.intensities.is_none() {
            // Initialize intensities vector if this is first intensity
            self.intensities = Some(vec![0; self.xs.len() - 1]);
        }
        if let Some(ref mut intensities) = self.intensities {
            intensities.push(intensity);
        }
    }

    /// Number of points.
    #[inline]
    pub fn len(&self) -> usize {
        self.xs.len()
    }

    /// Check if empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.xs.is_empty()
    }

    /// Clear all points.
    pub fn clear(&mut self) {
        self.xs.clear();
        self.ys.clear();
        if let Some(ref mut intensities) = self.intensities {
            intensities.clear();
        }
    }

    /// Get point at index (compatibility layer for code expecting Point2D).
    #[inline]
    pub fn point_at(&self, i: usize) -> Point2D {
        Point2D::new(self.xs[i], self.ys[i])
    }

    /// Iterate over points (creates Point2D on the fly for compatibility).
    pub fn iter(&self) -> impl Iterator<Item = Point2D> + '_ {
        self.xs
            .iter()
            .zip(self.ys.iter())
            .map(|(&x, &y)| Point2D::new(x, y))
    }

    /// Iterate over points with intensities.
    pub fn iter_with_intensity(&self) -> impl Iterator<Item = (Point2D, u8)> + '_ {
        let intensities = &self.intensities;
        self.xs
            .iter()
            .zip(self.ys.iter())
            .enumerate()
            .map(move |(i, (&x, &y))| {
                let intensity = intensities
                    .as_ref()
                    .and_then(|v| v.get(i).copied())
                    .unwrap_or(0);
                (Point2D::new(x, y), intensity)
            })
    }

    /// SIMD-accelerated bounding box computation.
    #[inline]
    pub fn bounds(&self) -> Option<(Point2D, Point2D)> {
        if self.xs.is_empty() {
            return None;
        }

        let n = self.len();
        let chunks = n / 4;

        let mut min_x = Float4::splat(f32::MAX);
        let mut min_y = Float4::splat(f32::MAX);
        let mut max_x = Float4::splat(f32::MIN);
        let mut max_y = Float4::splat(f32::MIN);

        // Process 4 points at a time with SIMD
        for i in 0..chunks {
            let base = i * 4;
            let xs = Float4::new(self.xs[base..base + 4].try_into().unwrap());
            let ys = Float4::new(self.ys[base..base + 4].try_into().unwrap());
            min_x = min_x.min(xs);
            min_y = min_y.min(ys);
            max_x = max_x.max(xs);
            max_y = max_y.max(ys);
        }

        // Reduce SIMD vectors to scalar
        let min_x_arr = min_x.to_array();
        let min_y_arr = min_y.to_array();
        let max_x_arr = max_x.to_array();
        let max_y_arr = max_y.to_array();

        let mut final_min_x = min_x_arr[0]
            .min(min_x_arr[1])
            .min(min_x_arr[2])
            .min(min_x_arr[3]);
        let mut final_min_y = min_y_arr[0]
            .min(min_y_arr[1])
            .min(min_y_arr[2])
            .min(min_y_arr[3]);
        let mut final_max_x = max_x_arr[0]
            .max(max_x_arr[1])
            .max(max_x_arr[2])
            .max(max_x_arr[3]);
        let mut final_max_y = max_y_arr[0]
            .max(max_y_arr[1])
            .max(max_y_arr[2])
            .max(max_y_arr[3]);

        // Handle remainder (0-3 points) with scalar
        for i in (chunks * 4)..n {
            final_min_x = final_min_x.min(self.xs[i]);
            final_min_y = final_min_y.min(self.ys[i]);
            final_max_x = final_max_x.max(self.xs[i]);
            final_max_y = final_max_y.max(self.ys[i]);
        }

        Some((
            Point2D::new(final_min_x, final_min_y),
            Point2D::new(final_max_x, final_max_y),
        ))
    }

    /// SIMD-accelerated centroid (center of mass) computation.
    #[inline]
    pub fn centroid(&self) -> Option<Point2D> {
        if self.xs.is_empty() {
            return None;
        }

        let n = self.len();
        let chunks = n / 4;

        let mut sum_x = Float4::splat(0.0);
        let mut sum_y = Float4::splat(0.0);

        // Process 4 points at a time with SIMD
        for i in 0..chunks {
            let base = i * 4;
            let xs = Float4::new(self.xs[base..base + 4].try_into().unwrap());
            let ys = Float4::new(self.ys[base..base + 4].try_into().unwrap());
            sum_x += xs;
            sum_y += ys;
        }

        // Reduce SIMD to scalar
        let mut total_x = sum_x.reduce_add();
        let mut total_y = sum_y.reduce_add();

        // Handle remainder with scalar
        for i in (chunks * 4)..n {
            total_x += self.xs[i];
            total_y += self.ys[i];
        }

        Some(Point2D::new(total_x / n as f32, total_y / n as f32))
    }

    /// SIMD-accelerated transform of all points by a pose.
    ///
    /// Applies rotation and translation: p' = R(theta) * p + t
    #[inline]
    pub fn transform(&self, pose: &Pose2D) -> PointCloud2D {
        let n = self.len();
        let mut result = PointCloud2D::with_capacity(n);
        result.xs.resize(n, 0.0);
        result.ys.resize(n, 0.0);

        let (sin_t, cos_t) = pose.theta.sin_cos();
        let sin_v = Float4::splat(sin_t);
        let cos_v = Float4::splat(cos_t);
        let tx_v = Float4::splat(pose.x);
        let ty_v = Float4::splat(pose.y);

        let chunks = n / 4;

        // Process 4 points at a time with SIMD
        for i in 0..chunks {
            let base = i * 4;
            let xs = Float4::new(self.xs[base..base + 4].try_into().unwrap());
            let ys = Float4::new(self.ys[base..base + 4].try_into().unwrap());

            // x' = tx + x*cos - y*sin
            // y' = ty + x*sin + y*cos
            let new_xs = tx_v + xs * cos_v - ys * sin_v;
            let new_ys = ty_v + xs * sin_v + ys * cos_v;

            result.xs[base..base + 4].copy_from_slice(&new_xs.to_array());
            result.ys[base..base + 4].copy_from_slice(&new_ys.to_array());
        }

        // Handle remainder with scalar
        for i in (chunks * 4)..n {
            result.xs[i] = pose.x + self.xs[i] * cos_t - self.ys[i] * sin_t;
            result.ys[i] = pose.y + self.xs[i] * sin_t + self.ys[i] * cos_t;
        }

        result.intensities = self.intensities.clone();
        result
    }

    /// SIMD-accelerated in-place transform.
    #[inline]
    pub fn transform_mut(&mut self, pose: &Pose2D) {
        let n = self.len();
        let (sin_t, cos_t) = pose.theta.sin_cos();
        let sin_v = Float4::splat(sin_t);
        let cos_v = Float4::splat(cos_t);
        let tx_v = Float4::splat(pose.x);
        let ty_v = Float4::splat(pose.y);

        let chunks = n / 4;

        // Process 4 points at a time with SIMD
        for i in 0..chunks {
            let base = i * 4;
            let xs = Float4::new(self.xs[base..base + 4].try_into().unwrap());
            let ys = Float4::new(self.ys[base..base + 4].try_into().unwrap());

            let new_xs = tx_v + xs * cos_v - ys * sin_v;
            let new_ys = ty_v + xs * sin_v + ys * cos_v;

            self.xs[base..base + 4].copy_from_slice(&new_xs.to_array());
            self.ys[base..base + 4].copy_from_slice(&new_ys.to_array());
        }

        // Handle remainder with scalar
        for i in (chunks * 4)..n {
            let new_x = pose.x + self.xs[i] * cos_t - self.ys[i] * sin_t;
            let new_y = pose.y + self.xs[i] * sin_t + self.ys[i] * cos_t;
            self.xs[i] = new_x;
            self.ys[i] = new_y;
        }
    }

    /// SIMD-accelerated inverse transform (global to local frame).
    #[inline]
    pub fn inverse_transform(&self, pose: &Pose2D) -> PointCloud2D {
        let n = self.len();
        let mut result = PointCloud2D::with_capacity(n);
        result.xs.resize(n, 0.0);
        result.ys.resize(n, 0.0);

        let (sin_t, cos_t) = pose.theta.sin_cos();
        let sin_v = Float4::splat(sin_t);
        let cos_v = Float4::splat(cos_t);
        let px_v = Float4::splat(pose.x);
        let py_v = Float4::splat(pose.y);

        let chunks = n / 4;

        // Process 4 points at a time with SIMD
        for i in 0..chunks {
            let base = i * 4;
            let xs = Float4::new(self.xs[base..base + 4].try_into().unwrap());
            let ys = Float4::new(self.ys[base..base + 4].try_into().unwrap());

            let dx = xs - px_v;
            let dy = ys - py_v;

            // x' = dx*cos + dy*sin
            // y' = -dx*sin + dy*cos
            let new_xs = dx * cos_v + dy * sin_v;
            let new_ys = dy * cos_v - dx * sin_v;

            result.xs[base..base + 4].copy_from_slice(&new_xs.to_array());
            result.ys[base..base + 4].copy_from_slice(&new_ys.to_array());
        }

        // Handle remainder with scalar
        for i in (chunks * 4)..n {
            let dx = self.xs[i] - pose.x;
            let dy = self.ys[i] - pose.y;
            result.xs[i] = dx * cos_t + dy * sin_t;
            result.ys[i] = -dx * sin_t + dy * cos_t;
        }

        result.intensities = self.intensities.clone();
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, TAU};

    #[test]
    fn test_laser_scan_creation() {
        let ranges = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let scan = LaserScan::new(0.0, TAU, TAU / 5.0, 0.1, 10.0, ranges);

        assert_eq!(scan.len(), 5);
        assert!(!scan.is_empty());
        assert_relative_eq!(scan.angle_at(0), 0.0, epsilon = 1e-6);
        assert_relative_eq!(scan.angle_at(1), TAU / 5.0, epsilon = 1e-6);
    }

    #[test]
    fn test_laser_scan_valid_range() {
        let scan = LaserScan {
            range_min: 0.1,
            range_max: 10.0,
            ..LaserScan::default()
        };

        assert!(scan.is_valid_range(1.0));
        assert!(scan.is_valid_range(0.1));
        assert!(scan.is_valid_range(10.0));
        assert!(!scan.is_valid_range(0.05));
        assert!(!scan.is_valid_range(15.0));
        assert!(!scan.is_valid_range(0.0));
        assert!(!scan.is_valid_range(-1.0));
        assert!(!scan.is_valid_range(f32::NAN));
        assert!(!scan.is_valid_range(f32::INFINITY));
    }

    #[test]
    fn test_laser_scan_iter() {
        let ranges = vec![1.0, 2.0, 3.0];
        let intensities = vec![100, 150, 200];
        let scan =
            LaserScan::new(0.0, TAU, TAU / 3.0, 0.1, 10.0, ranges).with_intensities(intensities);

        let points: Vec<_> = scan.iter().collect();
        assert_eq!(points.len(), 3);
        assert_relative_eq!(points[0].0, 0.0, epsilon = 1e-6);
        assert_relative_eq!(points[0].1, 1.0, epsilon = 1e-6);
        assert_eq!(points[0].2, 100);
    }

    #[test]
    fn test_point_cloud_2d_basic() {
        let mut cloud = PointCloud2D::new();
        assert!(cloud.is_empty());

        cloud.push(Point2D::new(1.0, 2.0));
        cloud.push(Point2D::new(3.0, 4.0));

        assert_eq!(cloud.len(), 2);
        assert!(!cloud.is_empty());
    }

    #[test]
    fn test_point_cloud_2d_bounds() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(-1.0, -2.0));
        cloud.push(Point2D::new(3.0, 4.0));
        cloud.push(Point2D::new(0.0, 0.0));

        let (min, max) = cloud.bounds().unwrap();
        assert_relative_eq!(min.x, -1.0, epsilon = 1e-6);
        assert_relative_eq!(min.y, -2.0, epsilon = 1e-6);
        assert_relative_eq!(max.x, 3.0, epsilon = 1e-6);
        assert_relative_eq!(max.y, 4.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_2d_centroid() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(0.0, 0.0));
        cloud.push(Point2D::new(2.0, 0.0));
        cloud.push(Point2D::new(1.0, 3.0));

        let centroid = cloud.centroid().unwrap();
        assert_relative_eq!(centroid.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(centroid.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_2d_transform() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 0.0));

        let pose = Pose2D::new(0.0, 0.0, FRAC_PI_2);
        let transformed = cloud.transform(&pose);

        assert_relative_eq!(transformed.xs[0], 0.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.ys[0], 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_2d_transform_with_translation() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 0.0));

        let pose = Pose2D::new(2.0, 3.0, 0.0);
        let transformed = cloud.transform(&pose);

        assert_relative_eq!(transformed.xs[0], 3.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.ys[0], 3.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_2d_inverse_transform() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 1.0));

        let pose = Pose2D::new(1.0, 0.0, FRAC_PI_2);
        let local = cloud.inverse_transform(&pose);

        assert_relative_eq!(local.xs[0], 1.0, epsilon = 1e-6);
        assert_relative_eq!(local.ys[0], 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_transform_roundtrip() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 2.0));
        cloud.push(Point2D::new(-1.0, 3.0));

        let pose = Pose2D::new(5.0, -3.0, 1.2);

        let global = cloud.transform(&pose);
        let back = global.inverse_transform(&pose);

        for i in 0..cloud.len() {
            assert_relative_eq!(cloud.xs[i], back.xs[i], epsilon = 1e-5);
            assert_relative_eq!(cloud.ys[i], back.ys[i], epsilon = 1e-5);
        }
    }

    #[test]
    fn test_point_cloud_transform_mut_matches_transform() {
        let mut cloud1 = PointCloud2D::new();
        cloud1.push(Point2D::new(1.0, 2.0));
        cloud1.push(Point2D::new(-1.0, 3.0));
        cloud1.push(Point2D::new(0.0, 0.0));

        let mut cloud2 = cloud1.clone();
        let pose = Pose2D::new(5.0, -3.0, 1.2);

        let transformed = cloud1.transform(&pose);
        cloud2.transform_mut(&pose);

        for i in 0..transformed.len() {
            assert_relative_eq!(transformed.xs[i], cloud2.xs[i], epsilon = 1e-6);
            assert_relative_eq!(transformed.ys[i], cloud2.ys[i], epsilon = 1e-6);
        }
    }

    #[test]
    fn test_empty_point_cloud_operations() {
        let cloud = PointCloud2D::new();

        assert!(cloud.bounds().is_none());
        assert!(cloud.centroid().is_none());

        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let transformed = cloud.transform(&pose);
        assert!(transformed.is_empty());
    }

    #[test]
    fn test_single_point_cloud_operations() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(3.0, 4.0));

        let (min, max) = cloud.bounds().unwrap();
        assert_eq!(min.x, 3.0);
        assert_eq!(min.y, 4.0);
        assert_eq!(max.x, 3.0);
        assert_eq!(max.y, 4.0);

        let centroid = cloud.centroid().unwrap();
        assert_eq!(centroid.x, 3.0);
        assert_eq!(centroid.y, 4.0);
    }

    #[test]
    fn test_laser_scan_empty() {
        let scan = LaserScan::default();
        assert!(scan.is_empty());
        assert_eq!(scan.len(), 0);
        assert_eq!(scan.valid_count(), 0);
    }

    #[test]
    fn test_laser_scan_all_invalid() {
        let ranges = vec![0.0, -1.0, f32::NAN, f32::INFINITY, 0.05];
        let scan = LaserScan::new(0.0, 1.0, 0.25, 0.1, 10.0, ranges);

        assert_eq!(scan.len(), 5);
        assert_eq!(scan.valid_count(), 0);
    }

    #[test]
    fn test_point_cloud_clear() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 2.0));
        cloud.push(Point2D::new(3.0, 4.0));

        assert_eq!(cloud.len(), 2);

        cloud.clear();

        assert!(cloud.is_empty());
        assert_eq!(cloud.len(), 0);
    }
}
