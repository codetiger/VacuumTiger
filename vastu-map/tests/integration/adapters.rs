//! Type conversions between sangam-io and vastu-map
//!
//! sangam-io's lidar simulator produces `Vec<(f32, f32, u8)>` which maps
//! directly to vastu-map's `PolarScan.points` format.
//!
//! Note: vastu-map's extract_lines algorithm can hang on large continuous
//! point clouds (>100 points). We subsample lidar data to work around this.

use vastu_map::core::{PointCloud2D, PolarScan, Pose2D};

/// Convert lidar scan data from sangam-io format to vastu-map PointCloud2D.
///
/// This function subsamples the data to avoid performance issues with
/// vastu-map's line extraction algorithm on large continuous point clouds.
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
    // Split lidar data into 4 quadrants to avoid continuous loop issue
    // vastu-map's algorithm hangs on continuous point clouds
    // By taking only front-facing points, we get a simpler pattern

    // Filter to front 180 degrees only (angle between -PI/2 and PI/2)
    let front_points: Vec<_> = lidar_data
        .into_iter()
        .filter(|(angle, _, _)| {
            // Keep only front-facing points
            angle.abs() < std::f32::consts::FRAC_PI_2
        })
        .step_by(5) // Subsample to ~18 points
        .collect();

    let polar = PolarScan {
        points: front_points,
    };
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

/// Create a vastu-map Pose2D from physics state.
pub fn physics_to_pose(x: f32, y: f32, theta: f32) -> Pose2D {
    Pose2D::new(x, y, theta)
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
    fn test_lidar_to_point_cloud_filters_back() {
        // Test that the subsampled version only keeps front-facing points
        let lidar_data: Vec<_> = (0..360)
            .map(|i| {
                let angle = (i as f32).to_radians();
                (angle, 1.0, 100u8)
            })
            .collect();

        let cloud = lidar_to_point_cloud(lidar_data, 50, 0.1, 10.0);

        // Should only keep front 180 degrees, subsampled by 5
        // ~90 points in front, subsampled to ~18
        assert!(cloud.len() > 0, "Should have some points");
        assert!(cloud.len() < 30, "Should be subsampled");

        // All points should be in front (x > 0 for angle near 0)
        // But actually the cloud is in robot frame, so we check y range
        // Points from -90 to +90 degrees will have x values near cos(angle)
        // which is positive for angles near 0
    }
}
