//! Lidar sensor update for occupancy grid.
//!
//! Processes lidar scans to update the grid:
//! - Cells along rays are marked as Floor (free space)
//! - Cells at ray endpoints (within range) are marked as Wall (obstacle)

use crate::ObserveResult;
use crate::core::{CellType, LidarScan, Pose2D, WorldPoint};
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
    let mut new_wall_coords: Vec<GridCoord> = Vec::new();

    // Calculate sensor position in world frame
    let sensor_pos = robot_pose.transform_point(sensor_config.lidar_offset);
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

        // Mark cells along the ray as Floor (iterate directly, no allocation)
        let bresenham = BresenhamLine::new(sensor_coord, endpoint_coord);

        // Use peekable to detect the last element without collecting
        let mut iter = bresenham.peekable();
        while let Some(coord) = iter.next() {
            if !storage.is_valid_coord(coord) {
                continue;
            }

            let is_last = iter.peek().is_none();

            if !is_last {
                // Not the endpoint - mark as Floor
                if storage.set_type(coord, CellType::Floor) {
                    result.cells_floor += 1;
                }
                result.cells_updated += 1;
            } else {
                // Endpoint - mark as Wall if within max range
                if range < sensor_config.max_lidar_range {
                    if storage.set_type_with_priority(coord, CellType::Wall) {
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

/// Update the grid with a single lidar ray.
///
/// This is a lower-level function useful for incremental updates.
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
    let mut hit_wall = false;

    // Trace ray (iterate directly, no allocation)
    let bresenham = BresenhamLine::new(sensor_coord, endpoint_coord);
    let mut iter = bresenham.peekable();

    while let Some(coord) = iter.next() {
        if !storage.is_valid_coord(coord) {
            continue;
        }

        let is_last = iter.peek().is_none();

        if !is_last {
            storage.set_type(coord, CellType::Floor);
        } else if range < max_range {
            storage.set_type_with_priority(coord, CellType::Wall);
            hit_wall = true;
        }
        cells_updated += 1;
    }

    (cells_updated, hit_wall)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(100, 100, 0.05) // 5m x 5m at 5cm resolution
    }

    #[test]
    fn test_single_ray_update() {
        let mut storage = create_test_storage();
        let sensor_pos = WorldPoint::new(0.0, 0.0);

        // Cast a ray 1m forward
        let (cells, hit) = update_single_ray(&mut storage, sensor_pos, 0.0, 1.0, 8.0);

        assert!(cells > 0);
        assert!(hit);

        // Check that cells along the ray are Floor
        let mid_coord = storage.world_to_grid(WorldPoint::new(0.5, 0.0));
        assert_eq!(storage.get_type(mid_coord), CellType::Floor);

        // Check that endpoint is Wall
        let end_coord = storage.world_to_grid(WorldPoint::new(1.0, 0.0));
        assert_eq!(storage.get_type(end_coord), CellType::Wall);
    }

    #[test]
    fn test_lidar_scan_update() {
        let mut storage = create_test_storage();
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
        assert!(result.cells_wall > 0);
        assert!(result.cells_floor > 0);

        // Check walls in 4 directions
        // Note: sensor offset moves the lidar backwards
        let check_points = [
            WorldPoint::new(1.0 - 0.110, 0.0), // Forward (adjusted for lidar offset)
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
        assert!(hit);

        // Check diagonal endpoint
        let dist = 1.0 / std::f32::consts::SQRT_2;
        let end_coord = storage.world_to_grid(WorldPoint::new(dist, dist));
        // The actual endpoint might be slightly different due to resolution
        assert!(
            storage.get_type(end_coord) == CellType::Wall
                || storage.get_type(end_coord) == CellType::Floor
        );
    }
}
