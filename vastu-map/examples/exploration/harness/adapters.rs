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
    let polar = PolarScan { points: lidar_data };
    polar.to_cartesian(min_quality, min_range, max_range)
}
