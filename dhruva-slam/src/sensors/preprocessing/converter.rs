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
    /// Convert a laser scan to a point cloud with optional radial offset.
    ///
    /// The radial offset is added to each range measurement before conversion.
    /// This corrects for the optical offset between the lidar housing center
    /// and the actual laser origin (rotating scanner).
    ///
    /// ```text
    /// corrected_range = measured_range + radial_offset
    /// x = corrected_range * cos(angle)
    /// y = corrected_range * sin(angle)
    /// ```
    pub fn to_point_cloud_with_offset(scan: &LaserScan, radial_offset: f32) -> PointCloud2D {
        let mut cloud = PointCloud2D::with_capacity(scan.ranges.len());

        for (angle, range, intensity) in scan.iter() {
            // Skip invalid ranges
            if !scan.is_valid_range(range) {
                continue;
            }

            // Apply radial offset to range
            let corrected_range = range + radial_offset;

            let (sin_a, cos_a) = angle.sin_cos();
            let point = Point2D::new(corrected_range * cos_a, corrected_range * sin_a);

            if scan.intensities.is_some() {
                cloud.push_with_intensity(point, intensity);
            } else {
                cloud.push(point);
            }
        }

        cloud
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, PI};

    // ========================================================================
    // Radial Offset Tests
    // ========================================================================

    #[test]
    fn test_to_point_cloud_with_offset_positive() {
        // Test positive radial offset (adds to range)
        // Point at angle 0 with range 1.0 and offset 0.5 should be at (1.5, 0)
        let ranges = vec![1.0];
        let scan = LaserScan::new(0.0, 0.0, 0.1, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud_with_offset(&scan, 0.5);

        assert_eq!(cloud.len(), 1);
        assert_relative_eq!(cloud.xs[0], 1.5, epsilon = 1e-6);
        assert_relative_eq!(cloud.ys[0], 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_to_point_cloud_with_offset_at_angle() {
        // Point at 90° with range 2.0 and offset 0.5 should be at (0, 2.5)
        let ranges = vec![2.0];
        let scan = LaserScan::new(FRAC_PI_2, FRAC_PI_2, 0.1, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud_with_offset(&scan, 0.5);

        assert_eq!(cloud.len(), 1);
        assert_relative_eq!(cloud.xs[0], 0.0, epsilon = 1e-6);
        assert_relative_eq!(cloud.ys[0], 2.5, epsilon = 1e-6);
    }

    #[test]
    fn test_to_point_cloud_with_offset_realistic() {
        // Test with realistic 25.8mm optical offset
        // Point straight ahead at 5m should be at 5.0258m after offset
        let ranges = vec![5.0];
        let scan = LaserScan::new(0.0, 0.0, 0.1, 0.1, 10.0, ranges);
        let optical_offset = 0.0258; // 25.8mm in meters

        let cloud = ScanConverter::to_point_cloud_with_offset(&scan, optical_offset);

        assert_eq!(cloud.len(), 1);
        assert_relative_eq!(cloud.xs[0], 5.0258, epsilon = 1e-6);
        assert_relative_eq!(cloud.ys[0], 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_to_point_cloud_with_offset_preserves_intensities() {
        let ranges = vec![1.0, 2.0];
        let intensities = vec![100, 200];
        let scan = LaserScan::new(0.0, FRAC_PI_2, FRAC_PI_2, 0.1, 10.0, ranges)
            .with_intensities(intensities);

        let cloud = ScanConverter::to_point_cloud_with_offset(&scan, 0.1);

        assert!(cloud.intensities.is_some());
        let cloud_int = cloud.intensities.as_ref().unwrap();
        assert_eq!(cloud_int.len(), 2);
        assert_eq!(cloud_int[0], 100);
        assert_eq!(cloud_int[1], 200);
    }

    #[test]
    fn test_to_point_cloud_with_offset_skips_invalid() {
        let ranges = vec![1.0, 0.0, 3.0]; // Middle one invalid
        let scan = LaserScan::new(0.0, PI, FRAC_PI_2, 0.1, 10.0, ranges);

        let cloud = ScanConverter::to_point_cloud_with_offset(&scan, 0.5);

        assert_eq!(cloud.len(), 2); // Only valid points
    }
}
