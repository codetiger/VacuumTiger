//! Type conversions between sangam-io and vastu-map
//!
//! sangam-io's lidar simulator produces `Vec<(f32, f32, u8)>` which maps
//! directly to vastu-map's `PolarScan.points` format.

use vastu_map::core::{PointCloud2D, PolarScan};

/// Convert lidar scan data from sangam-io format to vastu-map PointCloud2D.
///
/// Uses full 360° lidar data for better line extraction quality.
///
/// # Arguments
/// * `lidar_data` - Raw lidar data from LidarSimulator::generate_scan()
/// * `min_quality` - Minimum quality value to include (0-255)
/// * `min_range` - Minimum valid range in meters
/// * `max_range` - Maximum valid range in meters
///
/// # Returns
/// Cartesian point cloud in robot frame, filtered by quality and range.
pub fn lidar_to_point_cloud(
    lidar_data: Vec<(f32, f32, u8)>,
    min_quality: u8,
    min_range: f32,
    max_range: f32,
) -> PointCloud2D {
    // Use full 360° scan data - the split-merge algorithm handles this correctly
    // after the infinite recursion fix in split_recursive()
    let polar = PolarScan { points: lidar_data };
    polar.to_cartesian(min_quality, min_range, max_range)
}

/// Convert lidar scan data without subsampling (for testing with known small datasets).
///
/// # Warning
/// Only use this with small point clouds (<100 points) or datasets that
/// don't form continuous loops.
pub fn lidar_to_point_cloud_full(
    lidar_data: Vec<(f32, f32, u8)>,
    min_quality: u8,
    min_range: f32,
    max_range: f32,
) -> PointCloud2D {
    let polar = PolarScan { points: lidar_data };
    polar.to_cartesian(min_quality, min_range, max_range)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn test_lidar_to_point_cloud_full() {
        // Test the full (unfiltered) conversion
        let lidar_data = vec![
            (0.0, 1.0, 100),                 // Forward, 1m
            (FRAC_PI_2, 2.0, 100),           // Left, 2m
            (std::f32::consts::PI, 1.5, 50), // Behind, low quality
        ];

        let cloud = lidar_to_point_cloud_full(lidar_data, 60, 0.1, 10.0);

        // Third point filtered by quality
        assert_eq!(cloud.len(), 2);

        // First point should be ~(1, 0)
        assert!((cloud.xs[0] - 1.0).abs() < 0.001);
        assert!(cloud.ys[0].abs() < 0.001);

        // Second point should be ~(0, 2)
        assert!(cloud.xs[1].abs() < 0.001);
        assert!((cloud.ys[1] - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_lidar_to_point_cloud_full_360() {
        // Test that full 360 degree scan is preserved
        let lidar_data: Vec<_> = (0..360)
            .map(|i| {
                let angle = (i as f32).to_radians();
                (angle, 1.0, 100u8)
            })
            .collect();

        let cloud = lidar_to_point_cloud(lidar_data, 50, 0.1, 10.0);

        // Should have all 360 points (no subsampling)
        assert_eq!(cloud.len(), 360, "Should have all points");

        // Points should form a circle around origin in robot frame
        for i in 0..cloud.len() {
            let dist = (cloud.xs[i].powi(2) + cloud.ys[i].powi(2)).sqrt();
            assert!((dist - 1.0).abs() < 0.01, "Points should be at radius 1.0");
        }
    }
}
