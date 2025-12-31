//! A* pathfinding algorithm.
//!
//! Implements A* search on the occupancy grid with support for:
//! - Robot footprint collision checking
//! - Safety margin around obstacles
//! - 8-connected grid movement

mod cached;
mod planner;
mod types;

pub use cached::CachedAStarPlanner;
pub use planner::AStarPlanner;
pub use types::{AStarConfig, PathFailure, PathResult};

use crate::core::WorldPoint;
use crate::grid::GridStorage;

/// Quick path finding with default configuration
pub fn find_path(storage: &GridStorage, start: WorldPoint, goal: WorldPoint) -> PathResult {
    let planner = AStarPlanner::with_defaults(storage);
    planner.find_path_world(start, goal)
}

/// Check if a path exists (faster than full path computation)
pub fn path_exists(storage: &GridStorage, start: WorldPoint, goal: WorldPoint) -> bool {
    find_path(storage, start, goal).success
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{CellType, GridCoord};
    use crate::query::RobotFootprint;

    fn create_test_storage() -> GridStorage {
        GridStorage::centered(50, 50, 0.1) // 5m x 5m at 10cm resolution
    }

    fn fill_floor(storage: &mut GridStorage) {
        for x in 0..50 {
            for y in 0..50 {
                storage.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }
    }

    #[test]
    fn test_simple_path() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        // Small footprint for this test
        let config = AStarConfig::with_footprint(RobotFootprint::new(0.05, 0.01));
        let planner = AStarPlanner::new(&storage, config);

        let start = GridCoord::new(10, 25);
        let goal = GridCoord::new(40, 25);

        let result = planner.find_path(start, goal);

        assert!(result.success);
        assert!(!result.path_grid.is_empty());
        assert_eq!(result.path_grid[0], start);
        assert_eq!(*result.path_grid.last().unwrap(), goal);
    }

    #[test]
    fn test_path_around_obstacle() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        // Add a wall across the middle
        for y in 15..35 {
            storage.set_type_with_priority(GridCoord::new(25, y), CellType::Wall);
        }

        let config = AStarConfig::with_footprint(RobotFootprint::new(0.05, 0.01));
        let planner = AStarPlanner::new(&storage, config);

        let start = GridCoord::new(10, 25);
        let goal = GridCoord::new(40, 25);

        let result = planner.find_path(start, goal);

        assert!(result.success);
        // Path should go around the wall
        assert!(result.path_grid.len() > 30);
    }

    #[test]
    fn test_no_path() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        // Add a complete wall barrier
        for y in 0..50 {
            storage.set_type_with_priority(GridCoord::new(25, y), CellType::Wall);
        }

        let config = AStarConfig::with_footprint(RobotFootprint::new(0.05, 0.01));
        let planner = AStarPlanner::new(&storage, config);

        let start = GridCoord::new(10, 25);
        let goal = GridCoord::new(40, 25);

        let result = planner.find_path(start, goal);

        assert!(!result.success);
        assert_eq!(result.failure_reason, Some(PathFailure::NoPath));
    }

    #[test]
    fn test_start_blocked() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        // Block start position
        storage.set_type_with_priority(GridCoord::new(10, 25), CellType::Wall);

        let config = AStarConfig::with_footprint(RobotFootprint::new(0.05, 0.01));
        let planner = AStarPlanner::new(&storage, config);

        let start = GridCoord::new(10, 25);
        let goal = GridCoord::new(40, 25);

        let result = planner.find_path(start, goal);

        assert!(!result.success);
        assert_eq!(result.failure_reason, Some(PathFailure::StartBlocked));
    }

    #[test]
    fn test_goal_blocked() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        // Block goal position
        storage.set_type_with_priority(GridCoord::new(40, 25), CellType::Wall);

        let config = AStarConfig::with_footprint(RobotFootprint::new(0.05, 0.01));
        let planner = AStarPlanner::new(&storage, config);

        let start = GridCoord::new(10, 25);
        let goal = GridCoord::new(40, 25);

        let result = planner.find_path(start, goal);

        assert!(!result.success);
        assert_eq!(result.failure_reason, Some(PathFailure::GoalBlocked));
    }

    #[test]
    fn test_diagonal_path() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        let config = AStarConfig {
            footprint: RobotFootprint::new(0.05, 0.01),
            allow_diagonal: true,
            ..Default::default()
        };
        let planner = AStarPlanner::new(&storage, config);

        let start = GridCoord::new(10, 10);
        let goal = GridCoord::new(40, 40);

        let result = planner.find_path(start, goal);

        assert!(result.success);
        // Diagonal path should be shorter than manhattan
        // For a 30x30 diagonal, manhattan would be 60, octile should be ~42
        assert!(result.path_grid.len() < 50);
    }

    #[test]
    fn test_4_connected_path() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        let config = AStarConfig {
            footprint: RobotFootprint::new(0.05, 0.01),
            allow_diagonal: false,
            ..Default::default()
        };
        let planner = AStarPlanner::new(&storage, config);

        let start = GridCoord::new(10, 10);
        let goal = GridCoord::new(40, 40);

        let result = planner.find_path(start, goal);

        assert!(result.success);
        // 4-connected path should follow manhattan distance
        // Path length should be approximately dx + dy = 60
        assert!(result.path_grid.len() >= 60);
    }

    #[test]
    fn test_world_coordinates() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        let config = AStarConfig::with_footprint(RobotFootprint::new(0.05, 0.01));
        let planner = AStarPlanner::new(&storage, config);

        let start = WorldPoint::new(-1.0, 0.0);
        let goal = WorldPoint::new(1.0, 0.0);

        let result = planner.find_path_world(start, goal);

        assert!(result.success);
        assert!(result.path_world.len() > 0);
    }

    #[test]
    fn test_path_length_meters() {
        let mut storage = create_test_storage();
        fill_floor(&mut storage);

        let config = AStarConfig::with_footprint(RobotFootprint::new(0.05, 0.01));
        let planner = AStarPlanner::new(&storage, config);

        // Path of 2 meters (20 cells at 10cm resolution)
        let start = WorldPoint::new(-1.0, 0.0);
        let goal = WorldPoint::new(1.0, 0.0);

        let result = planner.find_path_world(start, goal);

        assert!(result.success);
        // Path length should be approximately 2.0 meters
        let length = result.length_meters();
        assert!(length > 1.5 && length < 2.5);
    }
}
