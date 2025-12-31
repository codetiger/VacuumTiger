//! Traversability queries for path planning and navigation.
//!
//! Provides methods to check if positions and paths are safe for robot traversal.

mod checker;
mod footprint;
mod path;

pub use checker::TraversabilityChecker;
pub use footprint::RobotFootprint;
pub use path::PathCheckResult;

use crate::core::WorldPoint;
use crate::grid::GridStorage;

/// Quick check if a point is traversable.
pub fn is_traversable(storage: &GridStorage, point: WorldPoint) -> bool {
    let checker = TraversabilityChecker::with_default_footprint(storage);
    checker.is_point_traversable(point)
}

/// Quick check if a point is safe for the robot.
pub fn is_safe(storage: &GridStorage, point: WorldPoint) -> bool {
    let checker = TraversabilityChecker::with_default_footprint(storage);
    checker.is_position_safe(point)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{CellType, GridCoord};
    use std::simd::{cmp::SimdPartialOrd, u8x16};

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(100, 100, 0.05) // 5m x 5m at 5cm resolution
    }

    // =========================================================================
    // SIMD-OPTIMIZED TEST HELPERS
    // These are test-only implementations to verify SIMD correctness
    // =========================================================================

    /// Check if a rectangular region contains any obstacle cells (SIMD-optimized).
    fn is_rect_obstacle_free(
        checker: &TraversabilityChecker,
        center: GridCoord,
        half_width: i32,
        half_height: i32,
    ) -> bool {
        let width = checker.storage().width() as i32;
        let height = checker.storage().height() as i32;

        let min_x = (center.x - half_width).max(0) as usize;
        let max_x =
            ((center.x + half_width).min(width - 1) as usize).min(checker.storage().width() - 1);
        let min_y = (center.y - half_height).max(0) as usize;
        let max_y =
            ((center.y + half_height).min(height - 1) as usize).min(checker.storage().height() - 1);

        if min_x > max_x || min_y > max_y {
            return false; // Out of bounds
        }

        let cell_types = checker.storage().cell_types_raw();
        let storage_width = checker.storage().width();

        let wall_val = CellType::Wall as u8;
        let wall_vec = u8x16::splat(wall_val);

        for y in min_y..=max_y {
            let row_start = y * storage_width + min_x;
            let row_end = y * storage_width + max_x + 1;
            let row_slice = &cell_types[row_start..row_end];

            // Process 16 cells at a time
            let chunks = row_slice.chunks_exact(16);
            let remainder = chunks.remainder();

            for chunk in chunks {
                let data = u8x16::from_slice(chunk);
                let is_obstacle = data.simd_ge(wall_vec);
                if is_obstacle.to_bitmask() != 0 {
                    return false;
                }
            }

            for &cell_type in remainder {
                if cell_type >= wall_val {
                    return false;
                }
            }
        }

        true
    }

    /// Check if a position is safe using SIMD for bounding box pre-check.
    fn is_position_safe_simd(checker: &TraversabilityChecker, position: WorldPoint) -> bool {
        let center = checker.storage().world_to_grid(position);
        let resolution = checker.storage().resolution();
        let cells_radius = (checker.footprint().total_radius() / resolution).ceil() as i32;

        if !is_rect_obstacle_free(checker, center, cells_radius, cells_radius) {
            return false;
        }

        is_position_safe_check_unknown(checker, position, center, cells_radius)
    }

    /// Check for Unknown cells within the footprint (helper for SIMD version)
    fn is_position_safe_check_unknown(
        checker: &TraversabilityChecker,
        position: WorldPoint,
        center: GridCoord,
        cells_radius: i32,
    ) -> bool {
        for dy in -cells_radius..=cells_radius {
            for dx in -cells_radius..=cells_radius {
                let coord = center + GridCoord::new(dx, dy);

                if !checker.storage().is_valid_coord(coord) {
                    return false;
                }

                let cell_world = checker.storage().grid_to_world(coord);
                let dist = cell_world.distance(&position);

                if dist <= checker.footprint().total_radius() {
                    let cell_type = checker.storage().get_type(coord);
                    if !cell_type.is_traversable() {
                        return false;
                    }
                }
            }
        }

        true
    }

    /// Find nearest obstacle using SIMD for batch distance calculations.
    fn nearest_obstacle_simd(
        checker: &TraversabilityChecker,
        position: WorldPoint,
        max_distance: f32,
    ) -> Option<WorldPoint> {
        let center = checker.storage().world_to_grid(position);
        let resolution = checker.storage().resolution();
        let max_cells = (max_distance / resolution).ceil() as i32;

        let width = checker.storage().width() as i32;
        let height = checker.storage().height() as i32;

        let cell_types = checker.storage().cell_types_raw();
        let storage_width = checker.storage().width();

        let wall_val = CellType::Wall as u8;
        let wall_vec = u8x16::splat(wall_val);

        let mut nearest: Option<(WorldPoint, f32)> = None;

        let min_y = (center.y - max_cells).max(0) as usize;
        let max_y =
            ((center.y + max_cells).min(height - 1) as usize).min(checker.storage().height() - 1);
        let min_x = (center.x - max_cells).max(0) as usize;
        let max_x =
            ((center.x + max_cells).min(width - 1) as usize).min(checker.storage().width() - 1);

        for y in min_y..=max_y {
            let row_start = y * storage_width + min_x;
            let row_end = y * storage_width + max_x + 1;
            let row_slice = &cell_types[row_start..row_end];

            let chunks = row_slice.chunks_exact(16);
            let remainder = chunks.remainder();

            for (chunk_idx, chunk) in chunks.enumerate() {
                let data = u8x16::from_slice(chunk);
                let is_obstacle = data.simd_ge(wall_vec);
                let mask = is_obstacle.to_bitmask();

                if mask != 0 {
                    let base_x = min_x + chunk_idx * 16;
                    for bit in 0..16 {
                        if (mask & (1 << bit)) != 0 {
                            let x = base_x + bit;
                            let coord = GridCoord::new(x as i32, y as i32);
                            let obstacle_pos = checker.storage().grid_to_world(coord);
                            let dist = position.distance(&obstacle_pos);

                            if dist <= max_distance {
                                match nearest {
                                    None => nearest = Some((obstacle_pos, dist)),
                                    Some((_, d)) if dist < d => {
                                        nearest = Some((obstacle_pos, dist))
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                }
            }

            let remainder_start = min_x + (row_slice.len() / 16) * 16;
            for (i, &cell_type) in remainder.iter().enumerate() {
                if cell_type >= wall_val {
                    let x = remainder_start + i;
                    let coord = GridCoord::new(x as i32, y as i32);
                    let obstacle_pos = checker.storage().grid_to_world(coord);
                    let dist = position.distance(&obstacle_pos);

                    if dist <= max_distance {
                        match nearest {
                            None => nearest = Some((obstacle_pos, dist)),
                            Some((_, d)) if dist < d => nearest = Some((obstacle_pos, dist)),
                            _ => {}
                        }
                    }
                }
            }
        }

        nearest.map(|(p, _)| p)
    }

    #[test]
    fn test_traversable_unknown() {
        let storage = create_test_storage();
        let checker = TraversabilityChecker::with_default_footprint(&storage);

        // Unknown cells are not traversable
        assert!(!checker.is_point_traversable(WorldPoint::new(0.0, 0.0)));
    }

    #[test]
    fn test_traversable_floor() {
        let mut storage = create_test_storage();

        // Mark center as floor
        let center = storage.world_to_grid(WorldPoint::new(0.0, 0.0));
        storage.set_type(center, CellType::Floor);

        let checker = TraversabilityChecker::with_default_footprint(&storage);

        // Floor is traversable
        assert!(checker.is_cell_traversable(center));
    }

    #[test]
    fn test_position_safe_requires_footprint() {
        let mut storage = create_test_storage();
        let center = WorldPoint::new(0.0, 0.0);

        // Mark a large floor area
        for dx in -10..=10 {
            for dy in -10..=10 {
                let coord = storage.world_to_grid(center) + GridCoord::new(dx, dy);
                storage.set_type(coord, CellType::Floor);
            }
        }

        let footprint = RobotFootprint::new(0.1, 0.05);
        let checker = TraversabilityChecker::new(&storage, footprint);

        // Position should be safe with floor all around
        assert!(checker.is_position_safe(center));
    }

    #[test]
    fn test_position_unsafe_near_wall() {
        let mut storage = create_test_storage();
        let center = WorldPoint::new(0.0, 0.0);

        // Mark floor area
        let center_coord = storage.world_to_grid(center);
        for dx in -10..=10 {
            for dy in -10..=10 {
                let coord = center_coord + GridCoord::new(dx, dy);
                storage.set_type(coord, CellType::Floor);
            }
        }

        // Put a wall very close (within robot radius)
        let wall_coord = center_coord + GridCoord::new(2, 0); // ~10cm away
        storage.set_type_with_priority(wall_coord, CellType::Wall);

        let footprint = RobotFootprint::new(0.17, 0.05); // ~22cm total radius
        let checker = TraversabilityChecker::new(&storage, footprint);

        // Position should be unsafe due to wall within footprint
        assert!(!checker.is_position_safe(center));
    }

    #[test]
    fn test_path_check_clear() {
        let mut storage = create_test_storage();

        // Create a floor corridor
        for x in -20..=20 {
            for y in -5..=5 {
                let coord = storage.world_to_grid(WorldPoint::ZERO) + GridCoord::new(x, y);
                storage.set_type(coord, CellType::Floor);
            }
        }

        let footprint = RobotFootprint::new(0.1, 0.02);
        let checker = TraversabilityChecker::new(&storage, footprint);

        let start = WorldPoint::new(-0.5, 0.0);
        let end = WorldPoint::new(0.5, 0.0);

        let result = checker.check_path(start, end, None);

        assert!(result.is_clear);
        assert!(result.blocked_at.is_none());
    }

    #[test]
    fn test_path_check_blocked() {
        let mut storage = create_test_storage();

        // Create floor
        for x in -20..=20 {
            for y in -5..=5 {
                let coord = storage.world_to_grid(WorldPoint::ZERO) + GridCoord::new(x, y);
                storage.set_type(coord, CellType::Floor);
            }
        }

        // Add wall in the middle
        let wall_coord = storage.world_to_grid(WorldPoint::new(0.0, 0.0));
        storage.set_type_with_priority(wall_coord, CellType::Wall);

        let footprint = RobotFootprint::new(0.1, 0.02);
        let checker = TraversabilityChecker::new(&storage, footprint);

        let start = WorldPoint::new(-0.5, 0.0);
        let end = WorldPoint::new(0.5, 0.0);

        let result = checker.check_path(start, end, None);

        assert!(!result.is_clear);
        assert!(result.blocked_at.is_some());
        assert!(result.furthest_safe.is_some());
    }

    #[test]
    fn test_clearance() {
        let mut storage = create_test_storage();

        // Create floor
        for x in -20..=20 {
            for y in -10..=10 {
                let coord = storage.world_to_grid(WorldPoint::ZERO) + GridCoord::new(x, y);
                storage.set_type(coord, CellType::Floor);
            }
        }

        // Add wall
        let wall_pos = WorldPoint::new(0.3, 0.0);
        let wall_coord = storage.world_to_grid(wall_pos);
        storage.set_type_with_priority(wall_coord, CellType::Wall);

        let checker = TraversabilityChecker::with_default_footprint(&storage);
        let robot_pos = WorldPoint::new(0.0, 0.0);

        let clearance = checker.clearance(robot_pos, 1.0);

        // Clearance should be approximately the distance to the wall
        assert!(clearance > 0.0);
        assert!(clearance < 0.5);
    }

    #[test]
    fn test_nearest_obstacle() {
        let mut storage = create_test_storage();

        // Create floor
        for x in -20..=20 {
            for y in -20..=20 {
                let coord = storage.world_to_grid(WorldPoint::ZERO) + GridCoord::new(x, y);
                storage.set_type(coord, CellType::Floor);
            }
        }

        // Add walls
        let wall1_pos = WorldPoint::new(0.2, 0.0);
        let wall2_pos = WorldPoint::new(-0.5, 0.0);
        storage.set_type_with_priority(storage.world_to_grid(wall1_pos), CellType::Wall);
        storage.set_type_with_priority(storage.world_to_grid(wall2_pos), CellType::Wall);

        let checker = TraversabilityChecker::with_default_footprint(&storage);
        let robot_pos = WorldPoint::new(0.0, 0.0);

        let nearest = checker.nearest_obstacle(robot_pos, 1.0);

        assert!(nearest.is_some());
        // Nearest should be the closer wall
        let obstacle = nearest.unwrap();
        assert!(obstacle.distance(&robot_pos) < 0.3);
    }

    #[test]
    fn test_simd_position_safe_matches_scalar() {
        let mut storage = create_test_storage();
        let center = WorldPoint::new(0.0, 0.0);

        // Mark a large floor area
        for dx in -10..=10 {
            for dy in -10..=10 {
                let coord = storage.world_to_grid(center) + GridCoord::new(dx, dy);
                storage.set_type(coord, CellType::Floor);
            }
        }

        let footprint = RobotFootprint::new(0.1, 0.05);
        let checker = TraversabilityChecker::new(&storage, footprint);

        // Test positions
        let positions = [
            WorldPoint::new(0.0, 0.0),
            WorldPoint::new(0.1, 0.1),
            WorldPoint::new(-0.1, 0.1),
        ];

        for pos in positions {
            let scalar = checker.is_position_safe(pos);
            let simd = is_position_safe_simd(&checker, pos);
            assert_eq!(scalar, simd, "Mismatch at {:?}", pos);
        }
    }

    #[test]
    fn test_simd_rect_obstacle_free() {
        let mut storage = create_test_storage();
        let center = storage.world_to_grid(WorldPoint::ZERO);

        // Mark floor area
        for dx in -10..=10 {
            for dy in -10..=10 {
                let coord = center + GridCoord::new(dx, dy);
                storage.set_type(coord, CellType::Floor);
            }
        }

        // First check: should be obstacle-free
        {
            let checker = TraversabilityChecker::with_default_footprint(&storage);
            assert!(is_rect_obstacle_free(&checker, center, 5, 5));
        }

        // Add a wall
        storage.set_type_with_priority(center + GridCoord::new(2, 2), CellType::Wall);

        // Second check: should now detect obstacle
        {
            let checker = TraversabilityChecker::with_default_footprint(&storage);
            assert!(!is_rect_obstacle_free(&checker, center, 5, 5));

            // But smaller rect not including wall should be clear
            assert!(is_rect_obstacle_free(
                &checker,
                center + GridCoord::new(-5, 0),
                2,
                2
            ));
        }
    }

    #[test]
    fn test_simd_nearest_obstacle_matches_scalar() {
        let mut storage = create_test_storage();

        // Create floor
        for x in -20..=20 {
            for y in -20..=20 {
                let coord = storage.world_to_grid(WorldPoint::ZERO) + GridCoord::new(x, y);
                storage.set_type(coord, CellType::Floor);
            }
        }

        // Add walls
        let wall1_pos = WorldPoint::new(0.2, 0.0);
        let wall2_pos = WorldPoint::new(-0.5, 0.0);
        storage.set_type_with_priority(storage.world_to_grid(wall1_pos), CellType::Wall);
        storage.set_type_with_priority(storage.world_to_grid(wall2_pos), CellType::Wall);

        let checker = TraversabilityChecker::with_default_footprint(&storage);
        let robot_pos = WorldPoint::new(0.0, 0.0);

        let scalar_nearest = checker.nearest_obstacle(robot_pos, 1.0);
        let simd_nearest = nearest_obstacle_simd(&checker, robot_pos, 1.0);

        assert_eq!(scalar_nearest.is_some(), simd_nearest.is_some());

        if let (Some(s), Some(simd)) = (scalar_nearest, simd_nearest) {
            // Should find the same nearest obstacle (within grid resolution tolerance)
            let scalar_dist = s.distance(&robot_pos);
            let simd_dist = simd.distance(&robot_pos);
            assert!(
                (scalar_dist - simd_dist).abs() < storage.resolution(),
                "Distance mismatch: scalar={}, simd={}",
                scalar_dist,
                simd_dist
            );
        }
    }
}
