//! LiDAR scan and point cloud types.

use super::pose::{Point2D, Pose2D};
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
    pub angle_increment: f32,
    /// Minimum valid range in meters
    pub range_min: f32,
    /// Maximum valid range in meters
    pub range_max: f32,
    /// Range measurements in meters (0 or NaN = invalid)
    pub ranges: Vec<f32>,
    /// Optional intensity values (0-255)
    pub intensities: Option<Vec<u8>>,
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
    /// This converts to our LaserScan format.
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
            };
        }

        // Sort by angle to ensure proper ordering
        let mut sorted: Vec<_> = scan.to_vec();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let angle_min = sorted.first().map(|p| p.0).unwrap_or(0.0);
        let angle_max = sorted.last().map(|p| p.0).unwrap_or(0.0);

        // Estimate angle increment from data
        let angle_increment = if sorted.len() > 1 {
            (angle_max - angle_min) / (sorted.len() - 1) as f32
        } else {
            0.0
        };

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

    /// Get the angle for a given index.
    #[inline]
    pub fn angle_at(&self, index: usize) -> f32 {
        self.angle_min + index as f32 * self.angle_increment
    }

    /// Check if a range value is valid.
    #[inline]
    pub fn is_valid_range(&self, range: f32) -> bool {
        range.is_finite() && range > 0.0 && range >= self.range_min && range <= self.range_max
    }

    /// Iterate over (angle, range, intensity) tuples.
    pub fn iter(&self) -> impl Iterator<Item = (f32, f32, u8)> + '_ {
        let intensities = &self.intensities;
        self.ranges.iter().enumerate().map(move |(i, &range)| {
            let angle = self.angle_min + i as f32 * self.angle_increment;
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
        }
    }
}

/// Collection of 2D points in Cartesian coordinates.
///
/// Used as the output of scan preprocessing and input to scan matching.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct PointCloud2D {
    /// Point positions in meters
    pub points: Vec<Point2D>,
    /// Optional intensity values (same length as points)
    pub intensities: Option<Vec<u8>>,
}

impl PointCloud2D {
    /// Create an empty point cloud.
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            intensities: None,
        }
    }

    /// Create a point cloud with pre-allocated capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            intensities: None,
        }
    }

    /// Create from a vector of points.
    pub fn from_points(points: Vec<Point2D>) -> Self {
        Self {
            points,
            intensities: None,
        }
    }

    /// Add a point without intensity.
    #[inline]
    pub fn push(&mut self, point: Point2D) {
        self.points.push(point);
        if let Some(ref mut intensities) = self.intensities {
            intensities.push(0);
        }
    }

    /// Add a point with intensity.
    #[inline]
    pub fn push_with_intensity(&mut self, point: Point2D, intensity: u8) {
        self.points.push(point);
        if self.intensities.is_none() {
            // Initialize intensities vector if this is first intensity
            self.intensities = Some(vec![0; self.points.len() - 1]);
        }
        if let Some(ref mut intensities) = self.intensities {
            intensities.push(intensity);
        }
    }

    /// Number of points.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Clear all points.
    pub fn clear(&mut self) {
        self.points.clear();
        if let Some(ref mut intensities) = self.intensities {
            intensities.clear();
        }
    }

    /// Iterate over points.
    pub fn iter(&self) -> impl Iterator<Item = &Point2D> {
        self.points.iter()
    }

    /// Iterate over points with intensities.
    pub fn iter_with_intensity(&self) -> impl Iterator<Item = (&Point2D, u8)> + '_ {
        let intensities = &self.intensities;
        self.points.iter().enumerate().map(move |(i, p)| {
            let intensity = intensities
                .as_ref()
                .and_then(|v| v.get(i).copied())
                .unwrap_or(0);
            (p, intensity)
        })
    }

    /// Compute bounding box (min_point, max_point).
    pub fn bounds(&self) -> Option<(Point2D, Point2D)> {
        if self.points.is_empty() {
            return None;
        }

        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for p in &self.points {
            min_x = min_x.min(p.x);
            min_y = min_y.min(p.y);
            max_x = max_x.max(p.x);
            max_y = max_y.max(p.y);
        }

        Some((Point2D::new(min_x, min_y), Point2D::new(max_x, max_y)))
    }

    /// Compute centroid (center of mass).
    pub fn centroid(&self) -> Option<Point2D> {
        if self.points.is_empty() {
            return None;
        }

        let sum_x: f32 = self.points.iter().map(|p| p.x).sum();
        let sum_y: f32 = self.points.iter().map(|p| p.y).sum();
        let n = self.points.len() as f32;

        Some(Point2D::new(sum_x / n, sum_y / n))
    }

    /// Transform all points by a pose.
    ///
    /// Applies rotation and translation: p' = R(theta) * p + t
    pub fn transform(&self, pose: &Pose2D) -> PointCloud2D {
        let (sin_t, cos_t) = pose.theta.sin_cos();

        let transformed_points: Vec<Point2D> = self
            .points
            .iter()
            .map(|p| {
                Point2D::new(
                    pose.x + p.x * cos_t - p.y * sin_t,
                    pose.y + p.x * sin_t + p.y * cos_t,
                )
            })
            .collect();

        PointCloud2D {
            points: transformed_points,
            intensities: self.intensities.clone(),
        }
    }

    /// Transform points in place.
    pub fn transform_mut(&mut self, pose: &Pose2D) {
        let (sin_t, cos_t) = pose.theta.sin_cos();

        for p in &mut self.points {
            let new_x = pose.x + p.x * cos_t - p.y * sin_t;
            let new_y = pose.y + p.x * sin_t + p.y * cos_t;
            p.x = new_x;
            p.y = new_y;
        }
    }

    /// Inverse transform (global to local frame).
    pub fn inverse_transform(&self, pose: &Pose2D) -> PointCloud2D {
        let (sin_t, cos_t) = pose.theta.sin_cos();

        let transformed_points: Vec<Point2D> = self
            .points
            .iter()
            .map(|p| {
                let dx = p.x - pose.x;
                let dy = p.y - pose.y;
                Point2D::new(dx * cos_t + dy * sin_t, -dx * sin_t + dy * cos_t)
            })
            .collect();

        PointCloud2D {
            points: transformed_points,
            intensities: self.intensities.clone(),
        }
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

        assert_relative_eq!(transformed.points[0].x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.points[0].y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_2d_transform_with_translation() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 0.0));

        let pose = Pose2D::new(2.0, 3.0, 0.0);
        let transformed = cloud.transform(&pose);

        assert_relative_eq!(transformed.points[0].x, 3.0, epsilon = 1e-6);
        assert_relative_eq!(transformed.points[0].y, 3.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_2d_inverse_transform() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 1.0));

        let pose = Pose2D::new(1.0, 0.0, FRAC_PI_2);
        let local = cloud.inverse_transform(&pose);

        assert_relative_eq!(local.points[0].x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(local.points[0].y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_point_cloud_transform_roundtrip() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 2.0));
        cloud.push(Point2D::new(-1.0, 3.0));

        let pose = Pose2D::new(5.0, -3.0, 1.2);

        let global = cloud.transform(&pose);
        let back = global.inverse_transform(&pose);

        for (orig, recovered) in cloud.points.iter().zip(back.points.iter()) {
            assert_relative_eq!(orig.x, recovered.x, epsilon = 1e-5);
            assert_relative_eq!(orig.y, recovered.y, epsilon = 1e-5);
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

        for (p1, p2) in transformed.points.iter().zip(cloud2.points.iter()) {
            assert_relative_eq!(p1.x, p2.x, epsilon = 1e-6);
            assert_relative_eq!(p1.y, p2.y, epsilon = 1e-6);
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
