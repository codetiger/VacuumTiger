//! Scan conversion between polar and Cartesian representations.
//!
//! Converts LaserScan (polar) to PointCloud2D (Cartesian) and vice versa.

use crate::core::types::{LaserScan, Point2D, PointCloud2D};

/// Scan converter for polar-Cartesian transformations.
///
/// Provides static methods for converting between `LaserScan` (polar coordinates)
/// and `PointCloud2D` (Cartesian coordinates).
pub struct ScanConverter;

impl ScanConverter {
    /// Convert a laser scan to a point cloud.
    ///
    /// Transforms each (angle, range) pair to (x, y) coordinates:
    /// ```text
    /// x = range * cos(angle)
    /// y = range * sin(angle)
    /// ```
    ///
    /// Points with invalid ranges (zero, negative, NaN, infinite) are skipped.
    pub fn to_point_cloud(scan: &LaserScan) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(scan.ranges.len());

        for (angle, range, intensity) in scan.iter() {
            // Skip invalid ranges
            if !scan.is_valid_range(range) {
                continue;
            }

            let (sin_a, cos_a) = angle.sin_cos();
            let point = Point2D::new(range * cos_a, range * sin_a);

            if scan.intensities.is_some() {
                cloud.push_with_intensity(point, intensity);
            } else {
                cloud.push(point);
            }
        }

        cloud
    }

    /// Convert a laser scan to a point cloud, including invalid points as (0, 0).
    ///
    /// This preserves the point count and index correspondence with the original scan.
    /// Invalid points are placed at the origin.
    pub fn to_point_cloud_with_invalid(scan: &LaserScan) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(scan.ranges.len());

        for (angle, range, intensity) in scan.iter() {
            let point = if scan.is_valid_range(range) {
                let (sin_a, cos_a) = angle.sin_cos();
                Point2D::new(range * cos_a, range * sin_a)
            } else {
                Point2D::new(0.0, 0.0)
            };

            if scan.intensities.is_some() {
                cloud.push_with_intensity(point, intensity);
            } else {
                cloud.push(point);
            }
        }

        cloud
    }

    /// Convert a point cloud back to ranges for a given scan configuration.
    ///
    /// This is useful for projecting a point cloud back to scan format.
    /// Each point is converted to (range, angle), then binned into the scan's
    /// angular resolution.
    ///
    /// Returns a vector of ranges, with 0.0 for bins with no points.
    pub fn to_ranges(cloud: &PointCloud2D, angle_min: f32, angle_max: f32, n_bins: usize) -> Vec<f32> {
        if n_bins == 0 {
            return Vec::new();
        }

        let mut ranges = vec![0.0f32; n_bins];
        let angle_range = angle_max - angle_min;
        let bin_width = angle_range / n_bins as f32;

        for point in cloud.iter() {
            let range = (point.x * point.x + point.y * point.y).sqrt();
            let angle = point.y.atan2(point.x);

            // Normalize angle to [angle_min, angle_max)
            let mut normalized = angle;
            while normalized < angle_min {
                normalized += std::f32::consts::TAU;
            }
            while normalized >= angle_max {
                normalized -= std::f32::consts::TAU;
            }

            // Find bin index
            if normalized >= angle_min && normalized < angle_max {
                let bin = ((normalized - angle_min) / bin_width) as usize;
                if bin < n_bins {
                    // Keep the closest (smallest) range for each bin
                    if ranges[bin] == 0.0 || range < ranges[bin] {
                        ranges[bin] = range;
                    }
                }
            }
        }

        ranges
    }

    /// Convert a single polar coordinate to Cartesian.
    #[inline]
    pub fn polar_to_cartesian(angle: f32, range: f32) -> Point2D {
        let (sin_a, cos_a) = angle.sin_cos();
        Point2D::new(range * cos_a, range * sin_a)
    }

    /// Convert a single Cartesian coordinate to polar.
    ///
    /// Returns (angle, range).
    #[inline]
    pub fn cartesian_to_polar(point: &Point2D) -> (f32, f32) {
        let range = (point.x * point.x + point.y * point.y).sqrt();
        let angle = point.y.atan2(point.x);
        (angle, range)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

    #[test]
    fn test_polar_to_cartesian_x_axis() {
        // Point along positive X (angle = 0)
        let point = ScanConverter::polar_to_cartesian(0.0, 1.0);
        assert_relative_eq!(point.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(point.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_polar_to_cartesian_y_axis() {
        // Point along positive Y (angle = π/2)
        let point = ScanConverter::polar_to_cartesian(FRAC_PI_2, 1.0);
        assert_relative_eq!(point.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(point.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_polar_to_cartesian_45_degrees() {
        // Point at 45° (angle = π/4)
        let point = ScanConverter::polar_to_cartesian(FRAC_PI_4, 1.0);
        let expected = 1.0 / 2.0_f32.sqrt();
        assert_relative_eq!(point.x, expected, epsilon = 1e-6);
        assert_relative_eq!(point.y, expected, epsilon = 1e-6);
    }

    #[test]
    fn test_cartesian_to_polar() {
        let point = Point2D::new(3.0, 4.0);
        let (angle, range) = ScanConverter::cartesian_to_polar(&point);

        assert_relative_eq!(range, 5.0, epsilon = 1e-6);
        assert_relative_eq!(angle, (4.0_f32).atan2(3.0), epsilon = 1e-6);
    }

    #[test]
    fn test_roundtrip_polar_cartesian() {
        let original_angle = 1.2;
        let original_range = 5.5;

        let point = ScanConverter::polar_to_cartesian(original_angle, original_range);
        let (angle, range) = ScanConverter::cartesian_to_polar(&point);

        assert_relative_eq!(angle, original_angle, epsilon = 1e-6);
        assert_relative_eq!(range, original_range, epsilon = 1e-6);
    }

    #[test]
    fn test_to_point_cloud() {
        // Create a simple scan with 4 points at 90° intervals
        let ranges = vec![1.0, 1.0, 1.0, 1.0];
        let scan = LaserScan::new(0.0, 3.0 * FRAC_PI_2, FRAC_PI_2, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud(&scan);

        assert_eq!(cloud.len(), 4);

        // First point: angle=0, range=1 → (1, 0)
        assert_relative_eq!(cloud.points[0].x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(cloud.points[0].y, 0.0, epsilon = 1e-6);

        // Second point: angle=π/2, range=1 → (0, 1)
        assert_relative_eq!(cloud.points[1].x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(cloud.points[1].y, 1.0, epsilon = 1e-6);

        // Third point: angle=π, range=1 → (-1, 0)
        assert_relative_eq!(cloud.points[2].x, -1.0, epsilon = 1e-6);
        assert_relative_eq!(cloud.points[2].y, 0.0, epsilon = 1e-6);

        // Fourth point: angle=3π/2, range=1 → (0, -1)
        assert_relative_eq!(cloud.points[3].x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(cloud.points[3].y, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_to_point_cloud_skips_invalid() {
        let ranges = vec![1.0, 0.0, 1.0, f32::NAN, 1.0];
        let scan = LaserScan::new(0.0, TAU, TAU / 5.0, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud(&scan);

        // Only 3 valid points
        assert_eq!(cloud.len(), 3);
    }

    #[test]
    fn test_to_point_cloud_with_invalid() {
        let ranges = vec![1.0, 0.0, 1.0];
        let scan = LaserScan::new(0.0, TAU, TAU / 3.0, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud_with_invalid(&scan);

        // All 3 points preserved
        assert_eq!(cloud.len(), 3);

        // Second point should be at origin
        assert_eq!(cloud.points[1].x, 0.0);
        assert_eq!(cloud.points[1].y, 0.0);
    }

    #[test]
    fn test_preserves_intensities() {
        let ranges = vec![1.0, 1.0, 1.0];
        let intensities = vec![100, 150, 200];
        let scan = LaserScan::new(0.0, PI, FRAC_PI_2, 0.1, 10.0, ranges)
            .with_intensities(intensities);

        let cloud = ScanConverter::to_point_cloud(&scan);

        assert!(cloud.intensities.is_some());
        let cloud_int = cloud.intensities.as_ref().unwrap();
        assert_eq!(cloud_int[0], 100);
        assert_eq!(cloud_int[1], 150);
        assert_eq!(cloud_int[2], 200);
    }

    #[test]
    fn test_to_ranges() {
        // Create a square (4 points at corners)
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 0.0));   // angle = 0
        cloud.push(Point2D::new(0.0, 1.0));   // angle = π/2
        cloud.push(Point2D::new(-1.0, 0.0));  // angle = π
        cloud.push(Point2D::new(0.0, -1.0));  // angle = -π/2 (3π/2)

        let ranges = ScanConverter::to_ranges(&cloud, 0.0, TAU, 4);

        assert_eq!(ranges.len(), 4);
        // All points are at range 1.0
        for &r in &ranges {
            if r > 0.0 {
                assert_relative_eq!(r, 1.0, epsilon = 0.1);
            }
        }
    }

    #[test]
    fn test_empty_scan() {
        let scan = LaserScan::default();
        let cloud = ScanConverter::to_point_cloud(&scan);
        assert!(cloud.is_empty());
    }

    #[test]
    fn test_empty_cloud() {
        let cloud = PointCloud2D::new();
        let ranges = ScanConverter::to_ranges(&cloud, 0.0, TAU, 100);

        assert_eq!(ranges.len(), 100);
        assert!(ranges.iter().all(|&r| r == 0.0));
    }

    // ========================================================================
    // Edge Case Tests
    // ========================================================================

    #[test]
    fn test_all_invalid_scan() {
        // Scan where all ranges are invalid
        let ranges = vec![0.0, -1.0, f32::NAN, f32::INFINITY];
        let scan = LaserScan::new(0.0, TAU, TAU / 4.0, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud(&scan);

        // No valid points should be converted
        assert!(cloud.is_empty());
    }

    #[test]
    fn test_all_invalid_with_preservation() {
        let ranges = vec![0.0, f32::NAN, f32::INFINITY];
        let scan = LaserScan::new(0.0, PI, FRAC_PI_2, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud_with_invalid(&scan);

        // All points preserved at origin
        assert_eq!(cloud.len(), 3);
        for point in cloud.iter() {
            assert_eq!(point.x, 0.0);
            assert_eq!(point.y, 0.0);
        }
    }

    #[test]
    fn test_single_point_scan() {
        let ranges = vec![5.0];
        let scan = LaserScan::new(0.0, 0.0, 0.1, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud(&scan);

        assert_eq!(cloud.len(), 1);
        assert_relative_eq!(cloud.points[0].x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(cloud.points[0].y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_polar_to_cartesian_negative_x_axis() {
        // Point along negative X (angle = π)
        let point = ScanConverter::polar_to_cartesian(PI, 1.0);
        assert_relative_eq!(point.x, -1.0, epsilon = 1e-6);
        assert_relative_eq!(point.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_polar_to_cartesian_negative_y_axis() {
        // Point along negative Y (angle = -π/2 or 3π/2)
        let point = ScanConverter::polar_to_cartesian(-FRAC_PI_2, 1.0);
        assert_relative_eq!(point.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(point.y, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_polar_to_cartesian_zero_range() {
        let point = ScanConverter::polar_to_cartesian(1.234, 0.0);
        assert_eq!(point.x, 0.0);
        assert_eq!(point.y, 0.0);
    }

    #[test]
    fn test_polar_to_cartesian_very_large_range() {
        let point = ScanConverter::polar_to_cartesian(0.0, 1000.0);
        assert_relative_eq!(point.x, 1000.0, epsilon = 1e-3);
        assert_relative_eq!(point.y, 0.0, epsilon = 1e-3);
    }

    #[test]
    fn test_cartesian_to_polar_origin() {
        let point = Point2D::new(0.0, 0.0);
        let (angle, range) = ScanConverter::cartesian_to_polar(&point);

        assert_eq!(range, 0.0);
        // Angle is undefined at origin, but should not panic
        assert!(angle.is_finite() || angle == 0.0);
    }

    #[test]
    fn test_cartesian_to_polar_all_quadrants() {
        // Quadrant 1: (+x, +y)
        let (a1, r1) = ScanConverter::cartesian_to_polar(&Point2D::new(1.0, 1.0));
        assert!(a1 > 0.0 && a1 < FRAC_PI_2);
        assert_relative_eq!(r1, 2.0_f32.sqrt(), epsilon = 1e-6);

        // Quadrant 2: (-x, +y)
        let (a2, _) = ScanConverter::cartesian_to_polar(&Point2D::new(-1.0, 1.0));
        assert!(a2 > FRAC_PI_2 && a2 < PI);

        // Quadrant 3: (-x, -y)
        let (a3, _) = ScanConverter::cartesian_to_polar(&Point2D::new(-1.0, -1.0));
        assert!(a3 < -FRAC_PI_2 && a3 > -PI);

        // Quadrant 4: (+x, -y)
        let (a4, _) = ScanConverter::cartesian_to_polar(&Point2D::new(1.0, -1.0));
        assert!(a4 < 0.0 && a4 > -FRAC_PI_2);
    }

    #[test]
    fn test_to_ranges_zero_bins() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(1.0, 0.0));

        let ranges = ScanConverter::to_ranges(&cloud, 0.0, TAU, 0);

        assert!(ranges.is_empty());
    }

    #[test]
    fn test_to_ranges_single_bin() {
        let mut cloud = PointCloud2D::new();
        cloud.push(Point2D::new(5.0, 0.0));

        let ranges = ScanConverter::to_ranges(&cloud, 0.0, TAU, 1);

        assert_eq!(ranges.len(), 1);
        assert_relative_eq!(ranges[0], 5.0, epsilon = 0.1);
    }

    #[test]
    fn test_to_ranges_multiple_points_same_bin() {
        let mut cloud = PointCloud2D::new();
        // Two points at same angle, different ranges
        cloud.push(Point2D::new(3.0, 0.0));  // range = 3
        cloud.push(Point2D::new(5.0, 0.0));  // range = 5

        let ranges = ScanConverter::to_ranges(&cloud, -0.1, 0.1, 1);

        // Should keep the closest (smallest) range
        assert_relative_eq!(ranges[0], 3.0, epsilon = 0.1);
    }

    #[test]
    fn test_roundtrip_scan_cloud_ranges() {
        // Create a simple scan
        let original_ranges = vec![1.0, 2.0, 3.0, 4.0];
        let scan = LaserScan::new(0.0, 3.0 * FRAC_PI_2, FRAC_PI_2, 0.1, 10.0, original_ranges);

        // Convert to cloud
        let cloud = ScanConverter::to_point_cloud(&scan);

        // Convert back to ranges with same angular parameters
        let reconstructed = ScanConverter::to_ranges(&cloud, 0.0, TAU, 4);

        // Check that at least the occupied bins have values
        let non_zero: Vec<f32> = reconstructed.into_iter().filter(|&r| r > 0.0).collect();
        assert!(!non_zero.is_empty());
    }

    #[test]
    fn test_preserves_intensities_all_valid() {
        let ranges = vec![1.0, 2.0, 3.0];
        let intensities = vec![10, 20, 30];
        let scan = LaserScan::new(0.0, PI, FRAC_PI_2, 0.1, 10.0, ranges)
            .with_intensities(intensities);

        let cloud = ScanConverter::to_point_cloud(&scan);

        assert!(cloud.intensities.is_some());
        let cloud_int = cloud.intensities.as_ref().unwrap();
        assert_eq!(cloud_int.len(), 3);
    }

    #[test]
    fn test_preserves_intensities_skips_invalid() {
        let ranges = vec![1.0, 0.0, 3.0];  // Middle one invalid
        let intensities = vec![10, 20, 30];
        let scan = LaserScan::new(0.0, PI, FRAC_PI_2, 0.1, 10.0, ranges)
            .with_intensities(intensities);

        let cloud = ScanConverter::to_point_cloud(&scan);

        assert_eq!(cloud.len(), 2);
        let cloud_int = cloud.intensities.as_ref().unwrap();
        assert_eq!(cloud_int.len(), 2);
        assert_eq!(cloud_int[0], 10);
        assert_eq!(cloud_int[1], 30);
    }
}
