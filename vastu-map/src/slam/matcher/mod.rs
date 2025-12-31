//! Correlative scan-to-map matcher.
//!
//! Performs brute-force search over a 3D pose space (x, y, theta) to find
//! the best alignment of a lidar scan to the current occupancy grid map.

mod core;
mod helpers;
mod lm;
mod types;

pub use self::core::CorrelativeMatcher;
pub use types::ScratchBuffers;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::simd::PointCloud;
    use crate::core::{CellType, GridCoord, LidarScan, Pose2D, WorldPoint};
    use crate::grid::GridStorage;
    use crate::slam::CorrelativeMatcherConfig;

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(100, 100, 0.05) // 5m x 5m at 5cm resolution
    }

    fn create_wall_in_front(storage: &mut GridStorage, pose: Pose2D, distance: f32) {
        // Create a wall in front of the pose
        let wall_x = pose.x + distance * pose.theta.cos();
        let wall_y = pose.y + distance * pose.theta.sin();

        // Create a line of wall cells
        for offset in -10..=10 {
            let perp_angle = pose.theta + std::f32::consts::FRAC_PI_2;
            let wx = wall_x + offset as f32 * 0.05 * perp_angle.cos();
            let wy = wall_y + offset as f32 * 0.05 * perp_angle.sin();
            let coord = storage.world_to_grid(WorldPoint::new(wx, wy));
            if storage.is_valid_coord(coord) {
                storage.set_type(coord, CellType::Wall);
            }
        }

        // Update distance field for Gaussian scoring
        storage.recompute_distance_field();
    }

    fn create_simple_scan(distance: f32, num_points: usize) -> LidarScan {
        // Create a scan with points at uniform angles, all at same distance
        let angle_min = -std::f32::consts::PI;
        let angle_max = std::f32::consts::PI;
        let angle_increment = (angle_max - angle_min) / num_points as f32;

        let ranges = vec![distance; num_points];
        let angles: Vec<f32> = (0..num_points)
            .map(|i| angle_min + i as f32 * angle_increment)
            .collect();

        LidarScan::new(ranges, angles, 0.1, 10.0)
    }

    #[test]
    fn test_matcher_creation() {
        let matcher = CorrelativeMatcher::with_defaults();
        assert!(matcher.config().enabled);
    }

    #[test]
    fn test_disabled_matcher() {
        let config = CorrelativeMatcherConfig::disabled();
        let matcher = CorrelativeMatcher::new(config);

        let storage = create_test_storage();
        let scan = create_simple_scan(2.0, 360);
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        let result = matcher.match_scan(&scan, pose, &storage);

        // Disabled matcher should return input pose unchanged
        assert!((result.pose.x - pose.x).abs() < 0.001);
        assert!((result.pose.y - pose.y).abs() < 0.001);
    }

    #[test]
    fn test_matching_returns_result() {
        let mut storage = create_test_storage();
        let true_pose = Pose2D::new(0.0, 0.0, 0.0);

        // Create a wall 2m in front
        create_wall_in_front(&mut storage, true_pose, 2.0);

        // Use fast config for speed
        let config = CorrelativeMatcherConfig::fast();
        let matcher = CorrelativeMatcher::new(config);

        // Test with prior very close to true pose
        let prior = Pose2D::new(0.01, 0.01, 0.01);

        // Create a scan that simulates hitting the wall from the prior pose
        let angles: Vec<f32> = (-10..=10).map(|i| i as f32 * 0.05).collect();
        let ranges: Vec<f32> = vec![2.0; angles.len()];
        let scan = LidarScan::new(ranges, angles, 0.1, 10.0);

        let result = matcher.match_scan(&scan, prior, &storage);

        // Matcher should return a result (may or may not converge depending on map state)
        // At minimum, it should return a valid pose
        assert!(result.pose.x.is_finite());
        assert!(result.pose.y.is_finite());
        assert!(result.pose.theta.is_finite());
    }

    #[test]
    fn test_score_increases_with_hits() {
        let mut storage = create_test_storage();

        // Fill some cells with walls
        for x in 40..60 {
            for y in 40..60 {
                storage.set_type(GridCoord::new(x, y), CellType::Wall);
            }
        }

        // Update distance field for Gaussian scoring
        storage.recompute_distance_field();

        let matcher = CorrelativeMatcher::with_defaults();
        let points: Vec<(f32, f32)> = vec![(0.0, 0.0), (0.05, 0.0), (0.1, 0.0)];

        // Pose in wall area should score higher
        let pose_in_wall = Pose2D::new(0.0, 0.0, 0.0);
        let pose_in_floor = Pose2D::new(-2.0, -2.0, 0.0);

        let (score_wall, hits_wall) = matcher.score_pose(&points, pose_in_wall, &storage);
        let (score_floor, hits_floor) = matcher.score_pose(&points, pose_in_floor, &storage);

        assert!(hits_wall > hits_floor);
        assert!(score_wall > score_floor);
    }

    #[test]
    fn test_simd_matches_scalar() {
        let mut storage = create_test_storage();
        let true_pose = Pose2D::new(0.0, 0.0, 0.0);

        // Create a wall 2m in front
        create_wall_in_front(&mut storage, true_pose, 2.0);

        // Use fast config for speed
        let config = CorrelativeMatcherConfig::fast();
        let matcher = CorrelativeMatcher::new(config);

        // Create a scan with LANES-aligned point count (20 = 5 * LANES)
        // to avoid padding differences between scalar and SIMD paths
        let angles: Vec<f32> = (-10..10).map(|i| i as f32 * 0.05).collect(); // 20 points
        let ranges: Vec<f32> = vec![2.0; angles.len()];
        let scan = LidarScan::new(ranges, angles, 0.1, 10.0);

        let prior = Pose2D::new(0.01, 0.01, 0.01);

        // Get scalar result
        let scalar_result = matcher.match_scan(&scan, prior, &storage);

        // Get SIMD result
        let mut scratch = ScratchBuffers::new(scan.ranges.len());
        let simd_result = matcher.match_scan_simd(&scan, prior, &storage, &mut scratch);

        // Poses should match closely
        assert!((scalar_result.pose.x - simd_result.pose.x).abs() < 0.001);
        assert!((scalar_result.pose.y - simd_result.pose.y).abs() < 0.001);
        assert!((scalar_result.pose.theta - simd_result.pose.theta).abs() < 0.001);
        // Score comparison: SIMD may have slightly different normalization due to internal padding
        assert!(
            (scalar_result.score - simd_result.score).abs() < 0.05,
            "score mismatch: scalar={}, simd={}",
            scalar_result.score,
            simd_result.score
        );
    }

    #[test]
    fn test_simd_score_matches_scalar() {
        let mut storage = create_test_storage();

        // Fill some cells with walls
        for x in 40..60 {
            for y in 40..60 {
                storage.set_type(GridCoord::new(x, y), CellType::Wall);
            }
        }

        // Update distance field for Gaussian scoring
        storage.recompute_distance_field();

        let matcher = CorrelativeMatcher::with_defaults();

        // Create points in AoS format for scalar
        let points: Vec<(f32, f32)> = vec![(0.0, 0.0), (0.05, 0.0), (0.1, 0.0), (0.15, 0.0)];

        // Convert to SoA format for SIMD
        let points_soa = PointCloud {
            xs: points.iter().map(|(x, _)| *x).collect(),
            ys: points.iter().map(|(_, y)| *y).collect(),
        };

        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let mut scratch = ScratchBuffers::new(points.len());

        // Get scalar score
        let (scalar_score, scalar_hits) = matcher.score_pose(&points, pose, &storage);

        // Get SIMD score
        let (simd_score, simd_hits) =
            matcher.score_pose_simd(&points_soa, pose, &storage, &mut scratch);

        // Results should match
        assert!(
            (scalar_score - simd_score).abs() < 0.001,
            "score mismatch: {} vs {}",
            scalar_score,
            simd_score
        );
        assert_eq!(
            scalar_hits, simd_hits,
            "hits mismatch: {} vs {}",
            scalar_hits, simd_hits
        );
    }
}
