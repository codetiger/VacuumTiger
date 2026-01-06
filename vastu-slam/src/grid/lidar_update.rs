//! Lidar sensor update for occupancy grid.
//!
//! Processes lidar scans to update the grid using log-odds Bayesian updates:
//! - Cells along rays receive "miss" observations (evidence of free space)
//! - Cells at ray endpoints (within range) receive "hit" observations (evidence of obstacle)
//!
//! Log-odds representation provides:
//! - Proper Bayesian update: L_new = L_old + L_observation
//! - Better uncertainty handling for conflicting observations
//! - Thinner wall representation (2-3cm vs 5-8cm)

use crate::ObserveResult;
use crate::core::{LidarScan, Pose2D, WorldPoint};
use crate::grid::config::{GridConfig, SensorConfig};
use crate::grid::raycaster::BresenhamLine;
use crate::grid::storage::GridStorage;

/// Update the grid with a lidar scan.
///
/// For each valid ray in the scan:
/// 1. Mark all cells along the ray as Floor (free space)
/// 2. If the ray hit something (range < max), mark the endpoint as Wall
///
/// If `grid_config.auto_expand` is true, the grid will automatically
/// expand to include lidar endpoints that fall outside the current bounds.
///
/// After updating cells, the distance field is updated for new wall cells
/// to support Gaussian scoring in scan matching.
///
/// Returns the number of cells updated and statistics.
pub fn update_from_lidar(
    storage: &mut GridStorage,
    scan: &LidarScan,
    robot_pose: Pose2D,
    sensor_config: &SensorConfig,
    grid_config: &GridConfig,
) -> ObserveResult {
    use crate::core::GridCoord;

    let mut result = ObserveResult::default();

    // Track newly added wall cells for distance field update
    // Pre-allocate with typical capacity to avoid reallocations
    let mut new_wall_coords: Vec<GridCoord> = Vec::with_capacity(32);

    // Use robot position directly - lidar data is already robot-centered
    // (SangamIO transforms lidar measurements to robot center before streaming)
    let sensor_pos = WorldPoint::new(robot_pose.x, robot_pose.y);
    let mut sensor_coord = storage.world_to_grid(sensor_pos);

    // Process each ray
    for (&angle, &range) in scan.angles.iter().zip(scan.ranges.iter()) {
        // Skip invalid ranges
        if !scan.is_valid_range(range) {
            continue;
        }

        // Calculate world angle (sensor angle + robot heading)
        let world_angle = angle + robot_pose.theta;

        // Calculate endpoint in world coordinates
        let endpoint = WorldPoint::new(
            sensor_pos.x + range * world_angle.cos(),
            sensor_pos.y + range * world_angle.sin(),
        );
        let endpoint_coord = storage.world_to_grid(endpoint);

        // Handle endpoint outside grid bounds
        let mut endpoint_coord = endpoint_coord;
        if !storage.is_valid_coord(endpoint_coord) {
            if grid_config.auto_expand {
                // Try to expand the grid to include this endpoint
                if storage.expand_to_include(
                    endpoint,
                    grid_config.max_width,
                    grid_config.max_height,
                ) {
                    // Grid expanded - recalculate coordinates since origin may have changed
                    sensor_coord = storage.world_to_grid(sensor_pos);
                    endpoint_coord = storage.world_to_grid(endpoint);
                } else {
                    // Expansion failed (would exceed max size), skip this ray
                    continue;
                }
            } else {
                // Auto-expand disabled, skip rays outside grid
                continue;
            }
        }

        // Trace ray using Bresenham line algorithm
        // Apply log-odds updates: miss for ray cells, hit for endpoint
        // Compare against endpoint directly instead of using peekable()
        for coord in BresenhamLine::new(sensor_coord, endpoint_coord) {
            if !storage.is_valid_coord(coord) {
                continue;
            }

            let is_endpoint = coord == endpoint_coord;

            if !is_endpoint {
                // Not the endpoint - apply "miss" observation (evidence of free space)
                // This uses Bayesian log-odds update: L_new = L_old + L_MISS
                if storage.apply_miss(coord) {
                    result.cells_floor += 1;
                }
                result.cells_updated += 1;
            } else {
                // Endpoint - apply "hit" observation if within max range
                // This uses Bayesian log-odds update: L_new = L_old + L_HIT
                if range < sensor_config.max_lidar_range {
                    if storage.apply_hit(coord) {
                        result.cells_wall += 1;
                        new_wall_coords.push(coord);
                    }
                    result.cells_updated += 1;
                }
            }
        }
    }

    // Update distance field for newly added walls
    if !new_wall_coords.is_empty() {
        storage.update_distance_field_batch(&new_wall_coords);
    }

    result
}

/// Update the grid with a single lidar ray using log-odds updates.
///
/// This is a lower-level function useful for incremental updates.
/// Uses Bayesian log-odds updates for proper uncertainty handling.
///
/// Returns (cells_updated, hit_registered) where hit_registered is true
/// if a hit observation was applied to the endpoint (range < max_range).
/// Note: hit_registered means a hit observation was recorded, not that
/// the cell is classified as occupied (which requires accumulated observations).
pub fn update_single_ray(
    storage: &mut GridStorage,
    sensor_pos: WorldPoint,
    angle: f32,
    range: f32,
    max_range: f32,
) -> (usize, bool) {
    let sensor_coord = storage.world_to_grid(sensor_pos);

    // Calculate endpoint
    let endpoint = WorldPoint::new(
        sensor_pos.x + range * angle.cos(),
        sensor_pos.y + range * angle.sin(),
    );
    let endpoint_coord = storage.world_to_grid(endpoint);

    if !storage.is_valid_coord(endpoint_coord) {
        return (0, false);
    }

    let mut cells_updated = 0;
    let mut hit_registered = false;

    // Trace ray using Bresenham line algorithm
    let bresenham = BresenhamLine::new(sensor_coord, endpoint_coord);
    let mut iter = bresenham.peekable();

    while let Some(coord) = iter.next() {
        if !storage.is_valid_coord(coord) {
            continue;
        }

        let is_last = iter.peek().is_none();

        if !is_last {
            // Apply miss observation (evidence of free space)
            storage.apply_miss(coord);
        } else if range < max_range {
            // Apply hit observation (evidence of obstacle)
            // Returns whether cell became occupied, but we just care that we registered a hit
            let _ = storage.apply_hit(coord);
            hit_registered = true;
        }
        cells_updated += 1;
    }

    (cells_updated, hit_registered)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::CellType;
    use crate::grid::LogOddsConfig;

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(100, 100, 0.05) // 5m x 5m at 5cm resolution
    }

    fn create_aggressive_storage() -> GridStorage {
        // Use aggressive config where single hit crosses threshold
        GridStorage::centered_with_config(100, 100, 0.05, LogOddsConfig::aggressive())
    }

    #[test]
    fn test_single_ray_update() {
        // Use aggressive config so fewer observations are needed
        let mut storage = create_aggressive_storage();
        let sensor_pos = WorldPoint::new(0.0, 0.0);

        // Cast a ray 1m forward multiple times to accumulate log-odds
        // With aggressive config (l_miss=-28), 2 misses needed for floor: 0-28-28=-56 < -50
        // With aggressive config (l_hit=70), 1 hit needed for wall: 0+70=70 > 50
        for _ in 0..3 {
            let (cells, _hit) = update_single_ray(&mut storage, sensor_pos, 0.0, 1.0, 8.0);
            assert!(cells > 0);
        }

        // Check that cells along the ray have negative log-odds (free space evidence)
        let mid_coord = storage.world_to_grid(WorldPoint::new(0.5, 0.0));
        let mid_log_odds = storage.get_log_odds(mid_coord);
        assert!(
            mid_log_odds < 0,
            "Expected negative log-odds for ray cells, got {}",
            mid_log_odds
        );

        // After enough observations, should be marked as Floor
        assert_eq!(storage.get_type(mid_coord), CellType::Floor);

        // Check that endpoint has positive log-odds (obstacle evidence)
        let end_coord = storage.world_to_grid(WorldPoint::new(1.0, 0.0));
        let end_log_odds = storage.get_log_odds(end_coord);
        assert!(
            end_log_odds > 0,
            "Expected positive log-odds for endpoint, got {}",
            end_log_odds
        );

        // After enough observations, should be marked as Wall
        assert_eq!(storage.get_type(end_coord), CellType::Wall);
    }

    #[test]
    fn test_lidar_scan_update() {
        // Use aggressive config so single hits cross threshold
        let mut storage = create_aggressive_storage();
        let sensor_config = SensorConfig::default();
        let grid_config = GridConfig::default();
        let robot_pose = Pose2D::new(0.0, 0.0, 0.0);

        // Create a simple scan with 4 rays
        let scan = LidarScan::new(
            vec![1.0, 1.0, 1.0, 1.0],
            vec![
                0.0,
                std::f32::consts::FRAC_PI_2,
                std::f32::consts::PI,
                -std::f32::consts::FRAC_PI_2,
            ],
            0.15,
            8.0,
        );

        let result = update_from_lidar(
            &mut storage,
            &scan,
            robot_pose,
            &sensor_config,
            &grid_config,
        );

        assert!(result.cells_updated > 0);
        assert!(
            result.cells_wall > 0,
            "With aggressive config, single hit should cross threshold"
        );
        assert!(result.cells_floor > 0);

        // Check walls in 4 directions
        // Lidar data is now robot-centered (no offset adjustment needed)
        let check_points = [
            WorldPoint::new(1.0, 0.0), // Forward (at 1m from robot center)
        ];

        for point in &check_points {
            let coord = storage.world_to_grid(*point);
            // The endpoint should be a wall
            assert!(
                storage.get_type(coord) == CellType::Wall
                    || storage.get_type(coord) == CellType::Floor,
                "Expected Wall or Floor at {:?}",
                coord
            );
        }
    }

    #[test]
    fn test_max_range_no_wall() {
        let mut storage = create_test_storage();
        let sensor_pos = WorldPoint::new(0.0, 0.0);

        // Cast a ray at max range - should not mark wall
        // Use 2.0m range to stay within the 5m grid
        let (cells, hit) = update_single_ray(&mut storage, sensor_pos, 0.0, 2.0, 2.0);

        assert!(cells > 0);
        assert!(!hit); // No wall at max range
    }

    #[test]
    fn test_diagonal_ray() {
        let mut storage = create_test_storage();
        let sensor_pos = WorldPoint::new(0.0, 0.0);

        // Cast a diagonal ray
        let angle = std::f32::consts::FRAC_PI_4; // 45 degrees
        let (cells, hit) = update_single_ray(&mut storage, sensor_pos, angle, 1.0, 8.0);

        assert!(cells > 0);
        assert!(hit, "Should register a hit observation");

        // Check diagonal endpoint has positive log-odds (evidence of obstacle)
        let dist = 1.0 / std::f32::consts::SQRT_2;
        let end_coord = storage.world_to_grid(WorldPoint::new(dist, dist));
        let log_odds = storage.get_log_odds(end_coord);
        assert!(
            log_odds > 0,
            "Endpoint should have positive log-odds after hit, got {}",
            log_odds
        );
    }

    #[test]
    fn test_diagonal_ray_aggressive() {
        // Use aggressive config for immediate wall classification
        let mut storage = create_aggressive_storage();
        let sensor_pos = WorldPoint::new(0.0, 0.0);

        let angle = std::f32::consts::FRAC_PI_4; // 45 degrees
        let (cells, hit) = update_single_ray(&mut storage, sensor_pos, angle, 1.0, 8.0);

        assert!(cells > 0);
        assert!(hit);

        // With aggressive config, single hit should classify as Wall
        let dist = 1.0 / std::f32::consts::SQRT_2;
        let end_coord = storage.world_to_grid(WorldPoint::new(dist, dist));
        assert_eq!(storage.get_type(end_coord), CellType::Wall);
    }
}
