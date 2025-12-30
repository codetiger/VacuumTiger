//! Bumper sensor update for occupancy grid.
//!
//! Processes bumper collision events to mark cells as Bump.
//! Bump cells represent invisible obstacles (glass, mirrors, thin furniture legs)
//! that lidar cannot detect but the robot physically collides with.

use crate::ObserveResult;
use crate::core::{BumperSensors, CellType, Pose2D, WorldPoint};
use crate::grid::storage::GridStorage;

/// Default robot radius for collision position estimation.
const DEFAULT_ROBOT_RADIUS: f32 = 0.17;

/// Angular offset for left bumper (radians from forward).
const LEFT_BUMPER_ANGLE: f32 = 0.5; // ~30 degrees left

/// Angular offset for right bumper (radians from forward).
const RIGHT_BUMPER_ANGLE: f32 = -0.5; // ~30 degrees right

/// Update the grid with bumper collision readings.
///
/// For each triggered bumper:
/// 1. Estimate collision position based on bumper zone
/// 2. Mark the cell at that position as Bump
///
/// Bump has the highest priority (won't be overwritten by anything).
pub fn update_from_bumper(
    storage: &mut GridStorage,
    bumpers: &BumperSensors,
    robot_pose: Pose2D,
    robot_radius: f32,
) -> ObserveResult {
    let mut result = ObserveResult::default();

    if !bumpers.any_triggered() {
        return result;
    }

    // Determine collision angle(s)
    let collision_angles: Vec<f32> = match (bumpers.left, bumpers.right) {
        (true, true) => {
            // Both bumpers - mark center and spread
            vec![0.0, LEFT_BUMPER_ANGLE * 0.5, RIGHT_BUMPER_ANGLE * 0.5]
        }
        (true, false) => {
            // Left bumper only
            vec![LEFT_BUMPER_ANGLE]
        }
        (false, true) => {
            // Right bumper only
            vec![RIGHT_BUMPER_ANGLE]
        }
        (false, false) => {
            return result;
        }
    };

    // Mark cells at collision positions
    for angle in collision_angles {
        // Collision position in robot frame
        let local_pos = WorldPoint::new(robot_radius * angle.cos(), robot_radius * angle.sin());

        // Transform to world frame
        let world_pos = robot_pose.transform_point(local_pos);
        let coord = storage.world_to_grid(world_pos);

        if storage.is_valid_coord(coord) {
            // Mark as Bump with highest priority
            if storage.set_type_with_priority(coord, CellType::Bump) {
                result.cells_bump += 1;
            }
            result.cells_updated += 1;
        }
    }

    result
}

/// Update the grid with bumper collision using default robot radius.
pub fn update_from_bumper_default(
    storage: &mut GridStorage,
    bumpers: &BumperSensors,
    robot_pose: Pose2D,
) -> ObserveResult {
    update_from_bumper(storage, bumpers, robot_pose, DEFAULT_ROBOT_RADIUS)
}

/// Mark a specific world position as a bump obstacle.
///
/// Useful for manually marking invisible obstacles.
pub fn mark_bump_at(storage: &mut GridStorage, position: WorldPoint) -> bool {
    let coord = storage.world_to_grid(position);
    if storage.is_valid_coord(coord) {
        storage.set_type_with_priority(coord, CellType::Bump)
    } else {
        false
    }
}

/// Mark an arc of cells as bump obstacles.
///
/// Useful for marking a wider area when the exact collision point is uncertain.
pub fn mark_bump_arc(
    storage: &mut GridStorage,
    robot_pose: Pose2D,
    robot_radius: f32,
    center_angle: f32,
    arc_width: f32,
    num_samples: usize,
) -> usize {
    let mut count = 0;
    let half_arc = arc_width / 2.0;
    let step = arc_width / (num_samples as f32 - 1.0).max(1.0);

    for i in 0..num_samples {
        let angle = center_angle - half_arc + step * i as f32;

        let local_pos = WorldPoint::new(robot_radius * angle.cos(), robot_radius * angle.sin());

        let world_pos = robot_pose.transform_point(local_pos);
        let coord = storage.world_to_grid(world_pos);

        if storage.is_valid_coord(coord) && storage.set_type_with_priority(coord, CellType::Bump) {
            count += 1;
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
    fn test_left_bumper_update() {
        let mut storage = create_test_storage();
        let robot_pose = Pose2D::new(0.0, 0.0, 0.0);

        let bumpers = BumperSensors {
            left: true,
            right: false,
        };

        let result = update_from_bumper_default(&mut storage, &bumpers, robot_pose);

        assert!(result.cells_updated > 0);
        assert!(result.cells_bump > 0);

        // Check that a cell in the left-front area is marked as Bump
        let expected_pos = robot_pose.transform_point(WorldPoint::new(
            DEFAULT_ROBOT_RADIUS * LEFT_BUMPER_ANGLE.cos(),
            DEFAULT_ROBOT_RADIUS * LEFT_BUMPER_ANGLE.sin(),
        ));
        let coord = storage.world_to_grid(expected_pos);
        assert_eq!(storage.get_type(coord), CellType::Bump);
    }

    #[test]
    fn test_right_bumper_update() {
        let mut storage = create_test_storage();
        let robot_pose = Pose2D::new(0.0, 0.0, 0.0);

        let bumpers = BumperSensors {
            left: false,
            right: true,
        };

        let result = update_from_bumper_default(&mut storage, &bumpers, robot_pose);

        assert!(result.cells_updated > 0);
        assert!(result.cells_bump > 0);
    }

    #[test]
    fn test_both_bumpers_update() {
        let mut storage = create_test_storage();
        let robot_pose = Pose2D::new(0.0, 0.0, 0.0);

        let bumpers = BumperSensors {
            left: true,
            right: true,
        };

        let result = update_from_bumper_default(&mut storage, &bumpers, robot_pose);

        // Should mark multiple cells (center + sides)
        assert!(result.cells_updated >= 3);
        assert!(result.cells_bump >= 1);
    }

    #[test]
    fn test_no_bumper_triggered() {
        let mut storage = create_test_storage();
        let robot_pose = Pose2D::new(0.0, 0.0, 0.0);

        let bumpers = BumperSensors::CLEAR;

        let result = update_from_bumper_default(&mut storage, &bumpers, robot_pose);

        assert_eq!(result.cells_updated, 0);
        assert_eq!(result.cells_bump, 0);
    }

    #[test]
    fn test_bump_priority_over_wall() {
        let mut storage = create_test_storage();
        let coord = storage.world_to_grid(WorldPoint::new(0.0, 0.0));

        // First mark as Wall
        storage.set_type_with_priority(coord, CellType::Wall);
        assert_eq!(storage.get_type(coord), CellType::Wall);

        // Now mark as Bump - should override Wall
        storage.set_type_with_priority(coord, CellType::Bump);
        assert_eq!(storage.get_type(coord), CellType::Bump);

        // Try to mark as Wall again - should NOT override Bump
        storage.set_type_with_priority(coord, CellType::Wall);
        assert_eq!(storage.get_type(coord), CellType::Bump);
    }

    #[test]
    fn test_mark_bump_arc() {
        let mut storage = create_test_storage();
        let robot_pose = Pose2D::new(0.0, 0.0, 0.0);

        // Mark a 60-degree arc centered at 0 (forward)
        let count = mark_bump_arc(
            &mut storage,
            robot_pose,
            0.17,
            0.0,                         // center angle
            std::f32::consts::FRAC_PI_3, // 60 degrees
            5,                           // 5 samples
        );

        assert!(count > 0);

        // Check that center is marked
        let center_pos = robot_pose.transform_point(WorldPoint::new(0.17, 0.0));
        let coord = storage.world_to_grid(center_pos);
        assert_eq!(storage.get_type(coord), CellType::Bump);
    }

    #[test]
    fn test_rotated_robot_bumper() {
        let mut storage = create_test_storage();
        // Robot rotated 90 degrees (facing +Y)
        let robot_pose = Pose2D::new(0.0, 0.0, std::f32::consts::FRAC_PI_2);

        let bumpers = BumperSensors {
            left: true,
            right: false,
        };

        let result = update_from_bumper_default(&mut storage, &bumpers, robot_pose);
        assert!(result.cells_bump > 0);

        // The bump should be in a different world position due to rotation
        // Left bumper on a robot facing +Y should be in the +X direction
    }
}
