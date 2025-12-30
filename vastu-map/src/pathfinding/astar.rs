//! A* pathfinding algorithm.
//!
//! Implements A* search on the occupancy grid with support for:
//! - Robot footprint collision checking
//! - Safety margin around obstacles
//! - 8-connected grid movement

use crate::core::{GridCoord, WorldPoint};
use crate::grid::GridStorage;
use crate::query::{RobotFootprint, TraversabilityChecker};
use log::{debug, trace};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// A node in the A* search
#[derive(Clone, Debug)]
struct AStarNode {
    coord: GridCoord,
    g_cost: f32, // Cost from start
    f_cost: f32, // g_cost + heuristic
}

impl Eq for AStarNode {}

impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.coord == other.coord
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap behavior
        other
            .f_cost
            .partial_cmp(&self.f_cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// A* pathfinding configuration
#[derive(Clone, Debug)]
pub struct AStarConfig {
    /// Robot footprint for collision checking
    pub footprint: RobotFootprint,
    /// Allow diagonal movement (8-connected vs 4-connected)
    pub allow_diagonal: bool,
    /// Diagonal movement cost multiplier (sqrt(2) ≈ 1.414)
    pub diagonal_cost: f32,
    /// Maximum number of nodes to expand before giving up
    pub max_iterations: usize,
    /// Prefer paths with more clearance from obstacles
    pub clearance_weight: f32,
    /// Exploration mode: allow Unknown cells in footprint
    /// (assumes unknown areas are likely free)
    pub exploration_mode: bool,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            footprint: RobotFootprint::default(),
            allow_diagonal: true,
            diagonal_cost: std::f32::consts::SQRT_2,
            max_iterations: 100_000,
            clearance_weight: 0.0, // Disabled by default for performance
            exploration_mode: false,
        }
    }
}

impl AStarConfig {
    /// Create with custom robot footprint
    pub fn with_footprint(footprint: RobotFootprint) -> Self {
        Self {
            footprint,
            ..Default::default()
        }
    }

    /// Enable clearance-weighted pathfinding
    pub fn with_clearance_weight(mut self, weight: f32) -> Self {
        self.clearance_weight = weight;
        self
    }
}

/// Result of A* pathfinding
#[derive(Clone, Debug)]
pub struct PathResult {
    /// Path as grid coordinates (empty if no path found)
    pub path_grid: Vec<GridCoord>,
    /// Path as world coordinates
    pub path_world: Vec<WorldPoint>,
    /// Total path cost
    pub cost: f32,
    /// Number of nodes expanded during search
    pub nodes_expanded: usize,
    /// Whether a path was found
    pub success: bool,
    /// Reason for failure (if any)
    pub failure_reason: Option<PathFailure>,
}

impl PathResult {
    /// Create a failed result
    fn failed(reason: PathFailure, nodes_expanded: usize) -> Self {
        Self {
            path_grid: Vec::new(),
            path_world: Vec::new(),
            cost: f32::INFINITY,
            nodes_expanded,
            success: false,
            failure_reason: Some(reason),
        }
    }

    /// Path length in cells
    pub fn length_cells(&self) -> usize {
        self.path_grid.len()
    }

    /// Path length in meters (approximate)
    pub fn length_meters(&self) -> f32 {
        if self.path_world.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 1..self.path_world.len() {
            total += self.path_world[i - 1].distance(&self.path_world[i]);
        }
        total
    }
}

/// Reason for path failure
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PathFailure {
    /// Start position is not traversable
    StartBlocked,
    /// Goal position is not traversable
    GoalBlocked,
    /// No path exists between start and goal
    NoPath,
    /// Maximum iterations exceeded
    MaxIterationsExceeded,
    /// Start or goal is out of bounds
    OutOfBounds,
}

/// A* pathfinder
pub struct AStarPlanner<'a> {
    storage: &'a GridStorage,
    config: AStarConfig,
}

impl<'a> AStarPlanner<'a> {
    /// Create a new A* planner
    pub fn new(storage: &'a GridStorage, config: AStarConfig) -> Self {
        Self { storage, config }
    }

    /// Create with default configuration
    pub fn with_defaults(storage: &'a GridStorage) -> Self {
        Self::new(storage, AStarConfig::default())
    }

    /// Find a path from start to goal (grid coordinates)
    pub fn find_path(&self, start: GridCoord, goal: GridCoord) -> PathResult {
        trace!(
            "[AStar] find_path: start=({},{}) goal=({},{})",
            start.x, start.y, goal.x, goal.y
        );

        // Check bounds
        if !self.storage.is_valid_coord(start) || !self.storage.is_valid_coord(goal) {
            debug!("[AStar] FAILED: OutOfBounds - start or goal outside grid");
            return PathResult::failed(PathFailure::OutOfBounds, 0);
        }

        // Check start and goal traversability using footprint
        let checker = TraversabilityChecker::new(self.storage, self.config.footprint.clone());

        let start_world = self.storage.grid_to_world(start);
        let goal_world = self.storage.grid_to_world(goal);

        if !checker.is_position_safe_mode(start_world, self.config.exploration_mode) {
            debug!(
                "[AStar] FAILED: StartBlocked at ({:.2},{:.2}), exploration_mode={}",
                start_world.x, start_world.y, self.config.exploration_mode
            );
            return PathResult::failed(PathFailure::StartBlocked, 0);
        }
        if !checker.is_position_safe_mode(goal_world, self.config.exploration_mode) {
            debug!(
                "[AStar] FAILED: GoalBlocked at ({:.2},{:.2}), exploration_mode={}",
                goal_world.x, goal_world.y, self.config.exploration_mode
            );
            return PathResult::failed(PathFailure::GoalBlocked, 0);
        }

        // A* search
        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashSet::new();
        let mut came_from: HashMap<GridCoord, GridCoord> = HashMap::new();
        let mut g_scores: HashMap<GridCoord, f32> = HashMap::new();

        let h_start = self.heuristic(start, goal);
        open_set.push(AStarNode {
            coord: start,
            g_cost: 0.0,
            f_cost: h_start,
        });
        g_scores.insert(start, 0.0);

        let mut nodes_expanded = 0;

        while let Some(current) = open_set.pop() {
            nodes_expanded += 1;

            if nodes_expanded > self.config.max_iterations {
                debug!(
                    "[AStar] FAILED: MaxIterationsExceeded ({} nodes)",
                    nodes_expanded
                );
                return PathResult::failed(PathFailure::MaxIterationsExceeded, nodes_expanded);
            }

            // Goal reached
            if current.coord == goal {
                return self.reconstruct_path(came_from, goal, current.g_cost, nodes_expanded);
            }

            if closed_set.contains(&current.coord) {
                continue;
            }
            closed_set.insert(current.coord);

            // Explore neighbors
            let neighbors = if self.config.allow_diagonal {
                current.coord.neighbors_8().to_vec()
            } else {
                current.coord.neighbors_4().to_vec()
            };

            for (i, neighbor) in neighbors.iter().enumerate() {
                if closed_set.contains(neighbor) {
                    continue;
                }

                if !self.storage.is_valid_coord(*neighbor) {
                    continue;
                }

                // Check traversability with footprint
                let neighbor_world = self.storage.grid_to_world(*neighbor);
                if !checker.is_position_safe_mode(neighbor_world, self.config.exploration_mode) {
                    continue;
                }

                // Calculate movement cost
                let is_diagonal = self.config.allow_diagonal && i >= 4;
                let move_cost = if is_diagonal {
                    self.config.diagonal_cost
                } else {
                    1.0
                };

                // Add clearance cost if enabled
                let clearance_cost = if self.config.clearance_weight > 0.0 {
                    let clearance = checker.clearance(neighbor_world, 1.0);
                    // Penalize positions close to obstacles
                    self.config.clearance_weight / (clearance + 0.1)
                } else {
                    0.0
                };

                let tentative_g = g_scores[&current.coord] + move_cost + clearance_cost;

                let current_g = g_scores.get(neighbor).copied().unwrap_or(f32::INFINITY);
                if tentative_g < current_g {
                    came_from.insert(*neighbor, current.coord);
                    g_scores.insert(*neighbor, tentative_g);

                    let h = self.heuristic(*neighbor, goal);
                    open_set.push(AStarNode {
                        coord: *neighbor,
                        g_cost: tentative_g,
                        f_cost: tentative_g + h,
                    });
                }
            }
        }

        debug!(
            "[AStar] FAILED: NoPath after expanding {} nodes",
            nodes_expanded
        );
        PathResult::failed(PathFailure::NoPath, nodes_expanded)
    }

    /// Find a path from start to goal (world coordinates)
    pub fn find_path_world(&self, start: WorldPoint, goal: WorldPoint) -> PathResult {
        let start_grid = self.storage.world_to_grid(start);
        let goal_grid = self.storage.world_to_grid(goal);
        self.find_path(start_grid, goal_grid)
    }

    /// Heuristic function (octile distance for 8-connected grid)
    fn heuristic(&self, from: GridCoord, to: GridCoord) -> f32 {
        let dx = (from.x - to.x).abs() as f32;
        let dy = (from.y - to.y).abs() as f32;

        if self.config.allow_diagonal {
            // Octile distance
            let min = dx.min(dy);
            let max = dx.max(dy);
            min * self.config.diagonal_cost + (max - min)
        } else {
            // Manhattan distance
            dx + dy
        }
    }

    /// Reconstruct the path from came_from map
    fn reconstruct_path(
        &self,
        came_from: HashMap<GridCoord, GridCoord>,
        goal: GridCoord,
        cost: f32,
        nodes_expanded: usize,
    ) -> PathResult {
        let mut path_grid = Vec::new();
        let mut current = goal;

        while let Some(&prev) = came_from.get(&current) {
            path_grid.push(current);
            current = prev;
        }
        path_grid.push(current); // Add start
        path_grid.reverse();

        // Convert to world coordinates
        let path_world: Vec<WorldPoint> = path_grid
            .iter()
            .map(|c| self.storage.grid_to_world(*c))
            .collect();

        trace!(
            "[AStar] SUCCESS: path length={} cells, cost={:.2}, nodes_expanded={}",
            path_grid.len(),
            cost,
            nodes_expanded
        );

        PathResult {
            path_grid,
            path_world,
            cost,
            nodes_expanded,
            success: true,
            failure_reason: None,
        }
    }
}

/// Quick path finding with default configuration
pub fn find_path(storage: &GridStorage, start: WorldPoint, goal: WorldPoint) -> PathResult {
    let planner = AStarPlanner::with_defaults(storage);
    planner.find_path_world(start, goal)
}

/// Check if a path exists (faster than full path computation)
pub fn path_exists(storage: &GridStorage, start: WorldPoint, goal: WorldPoint) -> bool {
    find_path(storage, start, goal).success
}

/// A* planner with pre-computed traversability cache.
///
/// This is faster for multiple path queries on the same grid state,
/// as it pre-computes traversability for all cells once.
///
/// # Example
/// ```rust,ignore
/// let cached = CachedAStarPlanner::new(storage, config);
/// let path1 = cached.find_path(start1, goal1);
/// let path2 = cached.find_path(start2, goal2);  // Uses cached traversability
/// ```
pub struct CachedAStarPlanner<'a> {
    storage: &'a GridStorage,
    config: AStarConfig,
    /// Pre-computed traversability for each cell (true = safe to traverse)
    traversability_cache: Vec<bool>,
}

impl<'a> CachedAStarPlanner<'a> {
    /// Create a new cached A* planner.
    ///
    /// Pre-computes traversability for all cells based on the robot footprint.
    /// This has O(width × height × footprint_area) upfront cost but makes
    /// each pathfinding query O(1) for traversability checks.
    pub fn new(storage: &'a GridStorage, config: AStarConfig) -> Self {
        let checker = TraversabilityChecker::new(storage, config.footprint.clone());
        let width = storage.width();
        let height = storage.height();
        let exploration_mode = config.exploration_mode;

        // Pre-compute traversability for all cells
        let mut cache = vec![false; width * height];
        for y in 0..height {
            for x in 0..width {
                let coord = GridCoord::new(x as i32, y as i32);
                let world = storage.grid_to_world(coord);
                cache[y * width + x] = checker.is_position_safe_mode(world, exploration_mode);
            }
        }

        Self {
            storage,
            config,
            traversability_cache: cache,
        }
    }

    /// Create with default configuration
    pub fn with_defaults(storage: &'a GridStorage) -> Self {
        Self::new(storage, AStarConfig::default())
    }

    /// Check if a cell is traversable using the cache
    #[inline]
    fn is_traversable(&self, coord: GridCoord) -> bool {
        if !self.storage.is_valid_coord(coord) {
            return false;
        }
        let idx = coord.y as usize * self.storage.width() + coord.x as usize;
        self.traversability_cache[idx]
    }

    /// Find a path from start to goal (grid coordinates)
    pub fn find_path(&self, start: GridCoord, goal: GridCoord) -> PathResult {
        trace!(
            "[CachedAStar] find_path: start=({},{}) goal=({},{})",
            start.x, start.y, goal.x, goal.y
        );

        // Check bounds
        if !self.storage.is_valid_coord(start) || !self.storage.is_valid_coord(goal) {
            debug!("[CachedAStar] FAILED: OutOfBounds - start or goal outside grid");
            return PathResult::failed(PathFailure::OutOfBounds, 0);
        }

        // Check start and goal traversability using cache
        if !self.is_traversable(start) {
            debug!("[CachedAStar] FAILED: StartBlocked");
            return PathResult::failed(PathFailure::StartBlocked, 0);
        }
        if !self.is_traversable(goal) {
            debug!("[CachedAStar] FAILED: GoalBlocked");
            return PathResult::failed(PathFailure::GoalBlocked, 0);
        }

        // A* search
        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashSet::new();
        let mut came_from: HashMap<GridCoord, GridCoord> = HashMap::new();
        let mut g_scores: HashMap<GridCoord, f32> = HashMap::new();

        let h_start = self.heuristic(start, goal);
        open_set.push(AStarNode {
            coord: start,
            g_cost: 0.0,
            f_cost: h_start,
        });
        g_scores.insert(start, 0.0);

        let mut nodes_expanded = 0;

        while let Some(current) = open_set.pop() {
            nodes_expanded += 1;

            if nodes_expanded > self.config.max_iterations {
                debug!(
                    "[CachedAStar] FAILED: MaxIterationsExceeded ({} nodes)",
                    nodes_expanded
                );
                return PathResult::failed(PathFailure::MaxIterationsExceeded, nodes_expanded);
            }

            // Goal reached
            if current.coord == goal {
                return self.reconstruct_path(came_from, goal, current.g_cost, nodes_expanded);
            }

            if closed_set.contains(&current.coord) {
                continue;
            }
            closed_set.insert(current.coord);

            // Explore neighbors
            let neighbors = if self.config.allow_diagonal {
                current.coord.neighbors_8().to_vec()
            } else {
                current.coord.neighbors_4().to_vec()
            };

            for (i, neighbor) in neighbors.iter().enumerate() {
                if closed_set.contains(neighbor) {
                    continue;
                }

                // Check traversability using cache (O(1) lookup)
                if !self.is_traversable(*neighbor) {
                    continue;
                }

                // Calculate movement cost
                let is_diagonal = self.config.allow_diagonal && i >= 4;
                let move_cost = if is_diagonal {
                    self.config.diagonal_cost
                } else {
                    1.0
                };

                let tentative_g = g_scores[&current.coord] + move_cost;

                let current_g = g_scores.get(neighbor).copied().unwrap_or(f32::INFINITY);
                if tentative_g < current_g {
                    came_from.insert(*neighbor, current.coord);
                    g_scores.insert(*neighbor, tentative_g);

                    let h = self.heuristic(*neighbor, goal);
                    open_set.push(AStarNode {
                        coord: *neighbor,
                        g_cost: tentative_g,
                        f_cost: tentative_g + h,
                    });
                }
            }
        }

        debug!(
            "[CachedAStar] FAILED: NoPath after expanding {} nodes",
            nodes_expanded
        );
        PathResult::failed(PathFailure::NoPath, nodes_expanded)
    }

    /// Find a path from start to goal (world coordinates)
    pub fn find_path_world(&self, start: WorldPoint, goal: WorldPoint) -> PathResult {
        let start_grid = self.storage.world_to_grid(start);
        let goal_grid = self.storage.world_to_grid(goal);
        self.find_path(start_grid, goal_grid)
    }

    /// Heuristic function (octile distance for 8-connected grid)
    fn heuristic(&self, from: GridCoord, to: GridCoord) -> f32 {
        let dx = (from.x - to.x).abs() as f32;
        let dy = (from.y - to.y).abs() as f32;

        if self.config.allow_diagonal {
            let min = dx.min(dy);
            let max = dx.max(dy);
            min * self.config.diagonal_cost + (max - min)
        } else {
            dx + dy
        }
    }

    /// Reconstruct the path from came_from map
    fn reconstruct_path(
        &self,
        came_from: HashMap<GridCoord, GridCoord>,
        goal: GridCoord,
        cost: f32,
        nodes_expanded: usize,
    ) -> PathResult {
        let mut path_grid = Vec::new();
        let mut current = goal;

        while let Some(&prev) = came_from.get(&current) {
            path_grid.push(current);
            current = prev;
        }
        path_grid.push(current); // Add start
        path_grid.reverse();

        let path_world: Vec<WorldPoint> = path_grid
            .iter()
            .map(|c| self.storage.grid_to_world(*c))
            .collect();

        trace!(
            "[CachedAStar] SUCCESS: path length={} cells, cost={:.2}, nodes_expanded={}",
            path_grid.len(),
            cost,
            nodes_expanded
        );

        PathResult {
            path_grid,
            path_world,
            cost,
            nodes_expanded,
            success: true,
            failure_reason: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::CellType;

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
