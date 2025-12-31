//! Cliff sensor update for occupancy grid.
//!
//! Processes cliff sensor readings to mark cells as Cliff.
//! Cliff cells are floor drop-offs (stairs, ledges) that the robot cannot traverse.

use crate::ObserveResult;
use crate::core::{CellType, CliffSensors, Pose2D, WorldPoint};
use crate::grid::storage::GridStorage;

/// Cliff sensor positions in robot frame (from CRL-200S configuration).
/// These are the physical locations of the 4 cliff sensors on the robot.
const CLIFF_SENSOR_POSITIONS: [(f32, f32); 4] = [
    (0.12, 0.10),  // left_side
    (0.15, 0.05),  // left_front
    (0.15, -0.05), // right_front
    (0.12, -0.10), // right_side
];

/// Update the grid with cliff sensor readings.
///
/// For each triggered cliff sensor:
/// 1. Transform sensor position to world coordinates
/// 2. Mark the cell at that position as Cliff
///
/// Cliff has higher priority than Floor (won't be overwritten by lidar).
pub fn update_from_cliff(
    storage: &mut GridStorage,
    cliffs: &CliffSensors,
    robot_pose: Pose2D,
) -> ObserveResult {
    let mut result = ObserveResult::default();

    // Process each sensor
    let sensors = [
        (cliffs.left_side, CLIFF_SENSOR_POSITIONS[0]),
        (cliffs.left_front, CLIFF_SENSOR_POSITIONS[1]),
        (cliffs.right_front, CLIFF_SENSOR_POSITIONS[2]),
        (cliffs.right_side, CLIFF_SENSOR_POSITIONS[3]),
    ];

    for (triggered, (x, y)) in sensors {
        if triggered {
            // Transform sensor position to world frame
            let sensor_pos = robot_pose.transform_point(WorldPoint::new(x, y));
            let coord = storage.world_to_grid(sensor_pos);

            if storage.is_valid_coord(coord) {
                // Mark as Cliff with priority (won't be overwritten by Floor)
                if storage.set_type_with_priority(coord, CellType::Cliff) {
                    result.cells_cliff += 1;
                }
                result.cells_updated += 1;
            }
        }
    }

    result
}

/// Mark a specific world position as a cliff.
///
/// Useful for manually marking cliff areas or integrating with other sensors.
pub fn mark_cliff_at(storage: &mut GridStorage, position: WorldPoint) -> bool {
    let coord = storage.world_to_grid(position);
    if storage.is_valid_coord(coord) {
        storage.set_type_with_priority(coord, CellType::Cliff)
    } else {
        false
    }
}

/// Mark a circular area as cliff.
///
/// Useful for marking a safety zone around detected cliffs.
pub fn mark_cliff_radius(storage: &mut GridStorage, center: WorldPoint, radius: f32) -> usize {
    let resolution = storage.resolution();
    let cells_radius = (radius / resolution).ceil() as i32;
    let center_coord = storage.world_to_grid(center);
    let mut count = 0;

    for dy in -cells_radius..=cells_radius {
        for dx in -cells_radius..=cells_radius {
            let coord = center_coord + crate::core::GridCoord::new(dx, dy);

            if !storage.is_valid_coord(coord) {
                continue;
            }

            // Check if within circular radius
            let cell_center = storage.grid_to_world(coord);
            if cell_center.distance(&center) <= radius
                && storage.set_type_with_priority(coord, CellType::Cliff)
            {
                count += 1;
            }
        }
    }

    count
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(100, 100, 0.05) // 5m x 5m at 5cm resolution
    }

    #[test]
    fn test_cliff_sensor_update() {
        let mut storage = create_test_storage();
        let robot_pose = Pose2D::new(0.0, 0.0, 0.0);

        // Trigger left_front cliff sensor
        let cliffs = CliffSensors {
            left_side: false,
            left_front: true,
            right_front: false,
            right_side: false,
        };

        let result = update_from_cliff(&mut storage, &cliffs, robot_pose);

        assert_eq!(result.cells_updated, 1);
        assert_eq!(result.cells_cliff, 1);

        // Check that the cell at the sensor position is marked as Cliff
        let sensor_pos = robot_pose.transform_point(WorldPoint::new(0.15, 0.05));
        let coord = storage.world_to_grid(sensor_pos);
        assert_eq!(storage.get_type(coord), CellType::Cliff);
    }

    #[test]
    fn test_multiple_cliff_sensors() {
        let mut storage = create_test_storage();
        let robot_pose = Pose2D::new(0.0, 0.0, 0.0);

        // Trigger both front cliff sensors
        let cliffs = CliffSensors {
            left_side: false,
            left_front: true,
            right_front: true,
            right_side: false,
        };

        let result = update_from_cliff(&mut storage, &cliffs, robot_pose);

        assert_eq!(result.cells_updated, 2);
        assert_eq!(result.cells_cliff, 2);
    }

    #[test]
    fn test_cliff_priority_over_floor() {
        let mut storage = create_test_storage();
        let coord = storage.world_to_grid(WorldPoint::new(0.0, 0.0));

        // First mark as Floor
        storage.set_type(coord, CellType::Floor);
        assert_eq!(storage.get_type(coord), CellType::Floor);

        // Now mark as Cliff - should override
        storage.set_type_with_priority(coord, CellType::Cliff);
        assert_eq!(storage.get_type(coord), CellType::Cliff);

        // Try to mark as Floor again - should NOT override
        storage.set_type_with_priority(coord, CellType::Floor);
        assert_eq!(storage.get_type(coord), CellType::Cliff);
    }

    #[test]
    fn test_mark_cliff_radius() {
        let mut storage = create_test_storage();
        let center = WorldPoint::new(0.0, 0.0);
        let radius = 0.1; // 10cm radius

        let count = mark_cliff_radius(&mut storage, center, radius);

        // Should mark multiple cells in a circle
        assert!(count > 0);

        // Check that center is marked
        let center_coord = storage.world_to_grid(center);
        assert_eq!(storage.get_type(center_coord), CellType::Cliff);
    }

    #[test]
    fn test_rotated_robot() {
        let mut storage = create_test_storage();
        // Robot rotated 90 degrees (facing +Y)
        let robot_pose = Pose2D::new(0.0, 0.0, std::f32::consts::FRAC_PI_2);

        let cliffs = CliffSensors {
            left_side: false,
            left_front: true,
            right_front: false,
            right_side: false,
        };

        let result = update_from_cliff(&mut storage, &cliffs, robot_pose);
        assert_eq!(result.cells_cliff, 1);

        // Sensor should be in a different world position due to rotation
        let expected_pos = robot_pose.transform_point(WorldPoint::new(0.15, 0.05));
        let coord = storage.world_to_grid(expected_pos);
        assert_eq!(storage.get_type(coord), CellType::Cliff);
    }
}
