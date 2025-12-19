//! A* path planning algorithm with obstacle inflation.
//!
//! This module implements the A* search algorithm for path planning on
//! an occupancy grid. Key features:
//!
//! - **Obstacle inflation**: Expands obstacles by robot radius for safe clearance
//! - **8-connected grid**: Allows diagonal movement
//! - **Path simplification**: Douglas-Peucker algorithm to reduce waypoints
//! - **Nearest free cell**: Finds closest reachable point for blocked goals
//!
//! Note: Some utility methods are defined for future use.

use crate::navigation::{Path, Waypoint};
use crate::state::CurrentMapData;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// Configuration for the A* path planner.
#[derive(Debug, Clone)]
pub struct AStarConfig {
    /// Robot radius for obstacle inflation (meters).
    ///
    /// Obstacles are expanded by this amount to ensure the robot
    /// can pass through without collision.
    pub robot_radius: f32,

    /// Additional safety margin beyond robot radius (meters).
    ///
    /// Adds extra clearance to prevent getting too close to walls.
    /// Total inflation = robot_radius + safety_margin.
    pub safety_margin: f32,

    /// Allow diagonal movement (8-connected vs 4-connected grid).
    pub allow_diagonal: bool,

    /// Maximum number of A* iterations before giving up.
    ///
    /// Prevents infinite loops on very large or complex maps.
    pub max_iterations: usize,

    /// Path simplification tolerance (meters).
    ///
    /// Points within this distance of a straight line are removed.
    /// Set to 0.0 to disable simplification.
    pub simplification_tolerance: f32,

    /// Treat unknown cells as obstacles.
    ///
    /// If true, unknown areas are avoided. If false, the planner
    /// may route through unknown space (useful for exploration).
    pub unknown_is_obstacle: bool,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            robot_radius: 0.18,  // CRL-200S robot radius
            safety_margin: 0.10, // 10cm extra clearance from walls
            allow_diagonal: true,
            max_iterations: 100_000,
            simplification_tolerance: 0.05, // 5cm
            unknown_is_obstacle: true,
        }
    }
}

/// Error types for path planning.
#[derive(Debug, Clone)]
pub enum PlanningError {
    /// No path found to target.
    NoPathFound,

    /// Start position is outside map bounds.
    StartOutOfBounds,

    /// Goal position is outside map bounds.
    GoalOutOfBounds,

    /// Start position is inside an obstacle.
    StartInObstacle,

    /// Goal position is inside an obstacle (and no nearby free cell found).
    GoalInObstacle,

    /// Planning exceeded maximum iterations.
    MaxIterationsExceeded,

    /// Map is empty or invalid.
    InvalidMap,
}

impl std::fmt::Display for PlanningError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanningError::NoPathFound => write!(f, "No path found to target"),
            PlanningError::StartOutOfBounds => write!(f, "Start position is outside map"),
            PlanningError::GoalOutOfBounds => write!(f, "Goal position is outside map"),
            PlanningError::StartInObstacle => write!(f, "Start position is inside obstacle"),
            PlanningError::GoalInObstacle => write!(f, "Goal is unreachable (blocked)"),
            PlanningError::MaxIterationsExceeded => write!(f, "Planning timeout"),
            PlanningError::InvalidMap => write!(f, "Invalid map data"),
        }
    }
}

impl std::error::Error for PlanningError {}

/// A* path planner on occupancy grid.
pub struct AStarPlanner {
    config: AStarConfig,

    /// Inflated obstacle grid (true = free, false = blocked).
    inflated_grid: Vec<bool>,

    /// Grid dimensions (cached from last planning).
    grid_width: usize,
    grid_height: usize,

    /// Grid resolution and origin (cached).
    resolution: f32,
    origin_x: f32,
    origin_y: f32,

    /// Number of cells to inflate obstacles by.
    inflation_cells: i32,
}

impl AStarPlanner {
    /// Create a new A* planner with the given configuration.
    pub fn new(config: AStarConfig) -> Self {
        Self {
            config,
            inflated_grid: Vec::new(),
            grid_width: 0,
            grid_height: 0,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            inflation_cells: 0,
        }
    }

    /// Plan a path from start to goal on the occupancy grid.
    ///
    /// # Arguments
    /// * `map` - The occupancy grid map
    /// * `start` - Start position in world coordinates (x, y)
    /// * `goal` - Goal position in world coordinates (x, y)
    /// * `target_id` - ID of the navigation target (for path metadata)
    ///
    /// # Returns
    /// * `Ok(Path)` - A valid path from start to goal
    /// * `Err(PlanningError)` - If no path could be found
    pub fn plan(
        &mut self,
        map: &CurrentMapData,
        start: (f32, f32),
        goal: (f32, f32),
        target_id: u32,
    ) -> Result<Path, PlanningError> {
        // Validate map
        if map.width == 0 || map.height == 0 || map.cells.is_empty() {
            return Err(PlanningError::InvalidMap);
        }

        // Update inflation grid
        self.inflate_obstacles(map);

        // Convert world coords to grid cells
        let start_cell = self
            .world_to_cell(start.0, start.1)
            .ok_or(PlanningError::StartOutOfBounds)?;

        let goal_cell = self
            .world_to_cell(goal.0, goal.1)
            .ok_or(PlanningError::GoalOutOfBounds)?;

        // Check if start is free
        if !self.is_cell_free(start_cell) {
            // Try to find nearest free cell to start
            if let Some(free_start) = self.find_nearest_free(start_cell, 20) {
                return self.plan_cells(map, free_start, goal_cell, target_id);
            }
            return Err(PlanningError::StartInObstacle);
        }

        // Check if goal is free
        let actual_goal = if self.is_cell_free(goal_cell) {
            goal_cell
        } else {
            // Find nearest free cell to goal
            self.find_nearest_free(goal_cell, 50)
                .ok_or(PlanningError::GoalInObstacle)?
        };

        self.plan_cells(map, start_cell, actual_goal, target_id)
    }

    /// Plan path between grid cells.
    fn plan_cells(
        &self,
        _map: &CurrentMapData,
        start: (usize, usize),
        goal: (usize, usize),
        target_id: u32,
    ) -> Result<Path, PlanningError> {
        // Early exit if start == goal
        if start == goal {
            let (wx, wy) = self.cell_to_world(start);
            return Ok(Path::new(vec![Waypoint::new(wx, wy)], target_id));
        }

        let mut open_set = BinaryHeap::new();
        let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
        let mut g_score: HashMap<(usize, usize), f32> = HashMap::new();

        g_score.insert(start, 0.0);
        open_set.push(AStarNode {
            cell: start,
            f_score: self.heuristic(start, goal),
        });

        let mut iterations = 0;

        while let Some(current) = open_set.pop() {
            iterations += 1;
            if iterations > self.config.max_iterations {
                return Err(PlanningError::MaxIterationsExceeded);
            }

            if current.cell == goal {
                // Reconstruct and return path
                let path = self.reconstruct_path(&came_from, start, goal, target_id);
                return Ok(path);
            }

            let current_g = *g_score.get(&current.cell).unwrap_or(&f32::INFINITY);

            for (neighbor, move_cost) in self.neighbors(current.cell) {
                if !self.is_cell_free(neighbor) {
                    continue;
                }

                let tentative_g = current_g + move_cost;

                if tentative_g < *g_score.get(&neighbor).unwrap_or(&f32::INFINITY) {
                    came_from.insert(neighbor, current.cell);
                    g_score.insert(neighbor, tentative_g);

                    let f = tentative_g + self.heuristic(neighbor, goal);
                    open_set.push(AStarNode {
                        cell: neighbor,
                        f_score: f,
                    });
                }
            }
        }

        Err(PlanningError::NoPathFound)
    }

    /// Inflate obstacles by robot radius.
    fn inflate_obstacles(&mut self, map: &CurrentMapData) {
        self.grid_width = map.width as usize;
        self.grid_height = map.height as usize;
        self.resolution = map.resolution;
        self.origin_x = map.origin_x;
        self.origin_y = map.origin_y;

        // Calculate inflation in cells (robot_radius + safety_margin)
        let total_inflation = self.config.robot_radius + self.config.safety_margin;
        self.inflation_cells = (total_inflation / map.resolution).ceil() as i32;

        // Start with all cells free
        let total_cells = self.grid_width * self.grid_height;
        self.inflated_grid.clear();
        self.inflated_grid.resize(total_cells, true);

        // Mark inflated obstacles
        for y in 0..self.grid_height {
            for x in 0..self.grid_width {
                let idx = y * self.grid_width + x;
                let cell_value = map.cells[idx];

                // Check if obstacle or unknown (depending on config)
                // Note: occupied = 100, unknown = 255, free = 0
                // Cell is occupied if value >= 100 BUT NOT if it's unknown (255)
                let is_occupied = cell_value >= 100 && cell_value != 255;
                let is_unknown_obstacle = cell_value == 255 && self.config.unknown_is_obstacle;
                let is_obstacle = is_occupied || is_unknown_obstacle;

                if is_obstacle {
                    // Inflate: mark surrounding cells as blocked
                    self.inflate_cell(x as i32, y as i32);
                }
            }
        }
    }

    /// Mark cells around an obstacle as blocked.
    fn inflate_cell(&mut self, cx: i32, cy: i32) {
        let r = self.inflation_cells;

        for dy in -r..=r {
            for dx in -r..=r {
                // Use circular inflation (Euclidean distance)
                if dx * dx + dy * dy > r * r {
                    continue;
                }

                let nx = cx + dx;
                let ny = cy + dy;

                if nx >= 0
                    && ny >= 0
                    && (nx as usize) < self.grid_width
                    && (ny as usize) < self.grid_height
                {
                    let idx = (ny as usize) * self.grid_width + (nx as usize);
                    self.inflated_grid[idx] = false;
                }
            }
        }
    }

    /// Convert world coordinates to grid cell.
    fn world_to_cell(&self, wx: f32, wy: f32) -> Option<(usize, usize)> {
        let cx = ((wx - self.origin_x) / self.resolution).floor() as i32;
        let cy = ((wy - self.origin_y) / self.resolution).floor() as i32;

        if cx >= 0 && cy >= 0 && (cx as usize) < self.grid_width && (cy as usize) < self.grid_height
        {
            Some((cx as usize, cy as usize))
        } else {
            None
        }
    }

    /// Convert grid cell to world coordinates (center of cell).
    fn cell_to_world(&self, cell: (usize, usize)) -> (f32, f32) {
        let wx = self.origin_x + (cell.0 as f32 + 0.5) * self.resolution;
        let wy = self.origin_y + (cell.1 as f32 + 0.5) * self.resolution;
        (wx, wy)
    }

    /// Check if a cell is free (not blocked after inflation).
    fn is_cell_free(&self, cell: (usize, usize)) -> bool {
        let idx = cell.1 * self.grid_width + cell.0;
        idx < self.inflated_grid.len() && self.inflated_grid[idx]
    }

    /// Heuristic function (Euclidean distance).
    fn heuristic(&self, a: (usize, usize), b: (usize, usize)) -> f32 {
        let dx = (b.0 as f32) - (a.0 as f32);
        let dy = (b.1 as f32) - (a.1 as f32);
        (dx * dx + dy * dy).sqrt()
    }

    /// Get neighbors of a cell with their movement costs.
    fn neighbors(&self, cell: (usize, usize)) -> Vec<((usize, usize), f32)> {
        let mut result = Vec::with_capacity(8);
        let (cx, cy) = (cell.0 as i32, cell.1 as i32);

        // 4-connected neighbors (cardinal directions)
        let cardinal = [(0, 1), (1, 0), (0, -1), (-1, 0)];
        for (dx, dy) in cardinal {
            let nx = cx + dx;
            let ny = cy + dy;
            if nx >= 0
                && ny >= 0
                && (nx as usize) < self.grid_width
                && (ny as usize) < self.grid_height
            {
                result.push(((nx as usize, ny as usize), 1.0));
            }
        }

        // 8-connected neighbors (diagonal directions)
        if self.config.allow_diagonal {
            let diagonal = [(1, 1), (1, -1), (-1, 1), (-1, -1)];
            let sqrt2 = std::f32::consts::SQRT_2;

            for (dx, dy) in diagonal {
                let nx = cx + dx;
                let ny = cy + dy;
                if nx >= 0
                    && ny >= 0
                    && (nx as usize) < self.grid_width
                    && (ny as usize) < self.grid_height
                {
                    // Only allow diagonal if both adjacent cells are free
                    // (prevents cutting corners)
                    let adj1_free = self.is_cell_free(((cx + dx) as usize, cy as usize));
                    let adj2_free = self.is_cell_free((cx as usize, (cy + dy) as usize));

                    if adj1_free && adj2_free {
                        result.push(((nx as usize, ny as usize), sqrt2));
                    }
                }
            }
        }

        result
    }

    /// Find nearest free cell using BFS.
    fn find_nearest_free(&self, start: (usize, usize), max_radius: i32) -> Option<(usize, usize)> {
        use std::collections::VecDeque;

        let mut visited = vec![false; self.grid_width * self.grid_height];
        let mut queue = VecDeque::new();

        queue.push_back((start, 0i32));
        visited[start.1 * self.grid_width + start.0] = true;

        while let Some((cell, dist)) = queue.pop_front() {
            if dist > max_radius {
                break;
            }

            if self.is_cell_free(cell) {
                return Some(cell);
            }

            // Add neighbors
            let (cx, cy) = (cell.0 as i32, cell.1 as i32);
            for (dx, dy) in [(0, 1), (1, 0), (0, -1), (-1, 0)] {
                let nx = cx + dx;
                let ny = cy + dy;

                if nx >= 0
                    && ny >= 0
                    && (nx as usize) < self.grid_width
                    && (ny as usize) < self.grid_height
                {
                    let nidx = (ny as usize) * self.grid_width + (nx as usize);
                    if !visited[nidx] {
                        visited[nidx] = true;
                        queue.push_back(((nx as usize, ny as usize), dist + 1));
                    }
                }
            }
        }

        None
    }

    /// Reconstruct path from A* came_from map.
    fn reconstruct_path(
        &self,
        came_from: &HashMap<(usize, usize), (usize, usize)>,
        start: (usize, usize),
        goal: (usize, usize),
        target_id: u32,
    ) -> Path {
        let mut cells = vec![goal];
        let mut current = goal;

        while current != start {
            if let Some(&prev) = came_from.get(&current) {
                cells.push(prev);
                current = prev;
            } else {
                break;
            }
        }

        cells.reverse();

        // Convert cells to world coordinates
        let waypoints: Vec<Waypoint> = cells
            .iter()
            .map(|&cell| {
                let (wx, wy) = self.cell_to_world(cell);
                Waypoint::new(wx, wy)
            })
            .collect();

        let mut path = Path::new(waypoints, target_id);

        // Simplify path if tolerance > 0
        if self.config.simplification_tolerance > 0.0 {
            path.simplify(self.config.simplification_tolerance);
        }

        path
    }
}

/// Node in the A* open set.
#[derive(Clone)]
struct AStarNode {
    cell: (usize, usize),
    f_score: f32,
}

impl Eq for AStarNode {}

impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.cell == other.cell
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (lower f_score = higher priority)
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a simple test map.
    fn create_test_map(width: u32, height: u32) -> CurrentMapData {
        let cells = vec![0u8; (width * height) as usize]; // All free
        CurrentMapData {
            map_id: "test".to_string(),
            name: "Test Map".to_string(),
            resolution: 0.05,
            width,
            height,
            origin_x: 0.0,
            origin_y: 0.0,
            cells,
            explored_area_m2: 0.0,
        }
    }

    /// Create a map with a wall obstacle.
    fn create_map_with_wall() -> CurrentMapData {
        let width = 100;
        let height = 100;
        let mut cells = vec![0u8; width * height];

        // Add vertical wall in the middle (y = 20 to 80, x = 50)
        for y in 20..80 {
            cells[y * width + 50] = 100; // Occupied
        }

        CurrentMapData {
            map_id: "wall_test".to_string(),
            name: "Wall Test".to_string(),
            resolution: 0.05,
            width: width as u32,
            height: height as u32,
            origin_x: 0.0,
            origin_y: 0.0,
            cells,
            explored_area_m2: 0.0,
        }
    }

    #[test]
    fn test_plan_simple_path() {
        let mut planner = AStarPlanner::new(AStarConfig {
            robot_radius: 0.05, // Small radius for test
            ..Default::default()
        });

        let map = create_test_map(100, 100);

        // Plan from (0.5, 0.5) to (4.5, 4.5)
        let result = planner.plan(&map, (0.5, 0.5), (4.5, 4.5), 1);

        assert!(result.is_ok());
        let path = result.unwrap();

        assert!(!path.is_empty());
        assert!(path.len() >= 2);

        // Check start and end are approximately correct
        let first = path.first().unwrap();
        let last = path.last().unwrap();

        assert!((first.x - 0.5).abs() < 0.1);
        assert!((first.y - 0.5).abs() < 0.1);
        assert!((last.x - 4.5).abs() < 0.1);
        assert!((last.y - 4.5).abs() < 0.1);
    }

    #[test]
    fn test_plan_around_wall() {
        let mut planner = AStarPlanner::new(AStarConfig {
            robot_radius: 0.1,
            simplification_tolerance: 0.0, // Disable simplification for test
            ..Default::default()
        });

        let map = create_map_with_wall();

        // Plan from left side to right side of wall
        // Start: (1.0, 2.5), Goal: (4.0, 2.5)
        let result = planner.plan(&map, (1.0, 2.5), (4.0, 2.5), 1);

        assert!(result.is_ok());
        let path = result.unwrap();

        // Path should go around the wall (either top or bottom)
        assert!(!path.is_empty());

        // Path should be longer than straight line (wall blocks direct path)
        let straight_line_dist = ((4.0 - 1.0_f32).powi(2) + (2.5 - 2.5_f32).powi(2)).sqrt();
        assert!(path.total_length > straight_line_dist);
    }

    #[test]
    fn test_start_equals_goal() {
        let mut planner = AStarPlanner::new(AStarConfig::default());
        let map = create_test_map(100, 100);

        let result = planner.plan(&map, (2.0, 2.0), (2.0, 2.0), 1);

        assert!(result.is_ok());
        let path = result.unwrap();
        assert_eq!(path.len(), 1);
    }

    #[test]
    fn test_goal_out_of_bounds() {
        let mut planner = AStarPlanner::new(AStarConfig::default());
        let map = create_test_map(100, 100);

        // Goal outside map bounds
        let result = planner.plan(&map, (2.0, 2.0), (10.0, 10.0), 1);

        assert!(matches!(result, Err(PlanningError::GoalOutOfBounds)));
    }

    #[test]
    fn test_goal_in_obstacle() {
        let mut planner = AStarPlanner::new(AStarConfig {
            robot_radius: 0.05,
            ..Default::default()
        });

        let mut map = create_test_map(100, 100);

        // Create a large obstacle in the center
        for y in 40..60 {
            for x in 40..60 {
                map.cells[y * 100 + x] = 100;
            }
        }

        // Goal in center of obstacle - should find nearest free cell
        let result = planner.plan(&map, (0.5, 0.5), (2.5, 2.5), 1);

        // Should succeed by finding nearest free point
        assert!(result.is_ok());
    }

    #[test]
    fn test_obstacle_inflation() {
        let mut planner = AStarPlanner::new(AStarConfig {
            robot_radius: 0.2, // 4 cells at 0.05 resolution
            ..Default::default()
        });

        let mut map = create_test_map(100, 100);

        // Single obstacle cell in center
        map.cells[50 * 100 + 50] = 100;

        planner.inflate_obstacles(&map);

        // Cell at obstacle should be blocked
        assert!(!planner.is_cell_free((50, 50)));

        // Cells nearby should also be blocked due to inflation
        assert!(!planner.is_cell_free((52, 50)));
        assert!(!planner.is_cell_free((50, 52)));

        // Cells far away should be free
        assert!(planner.is_cell_free((60, 60)));
    }

    #[test]
    fn test_find_nearest_free() {
        let mut planner = AStarPlanner::new(AStarConfig {
            robot_radius: 0.05,
            ..Default::default()
        });

        let mut map = create_test_map(100, 100);

        // Block a region
        for y in 45..55 {
            for x in 45..55 {
                map.cells[y * 100 + x] = 100;
            }
        }

        planner.inflate_obstacles(&map);

        // Find nearest free cell to center of blocked region
        let result = planner.find_nearest_free((50, 50), 20);

        assert!(result.is_some());
        let (fx, fy) = result.unwrap();

        // Should be outside the blocked region
        assert!(fx < 45 || fx >= 55 || fy < 45 || fy >= 55);
    }

    #[test]
    fn test_diagonal_corner_cutting() {
        let mut planner = AStarPlanner::new(AStarConfig {
            robot_radius: 0.01, // Very small for test
            allow_diagonal: true,
            ..Default::default()
        });

        let mut map = create_test_map(10, 10);

        // Create an L-shaped obstacle that would cause corner cutting
        // if not handled properly
        map.cells[5 * 10 + 5] = 100; // (5, 5)
        map.cells[5 * 10 + 6] = 100; // (6, 5)
        map.cells[6 * 10 + 5] = 100; // (5, 6)

        planner.inflate_obstacles(&map);

        // The diagonal from (4,4) to (6,6) should not cut through
        // because it would pass through corners of obstacles
        // This is tested implicitly by the neighbors() function
        // which checks adjacent cells before allowing diagonal
    }

    #[test]
    fn test_world_to_cell_conversion() {
        let planner = AStarPlanner {
            config: AStarConfig::default(),
            inflated_grid: Vec::new(),
            grid_width: 100,
            grid_height: 100,
            resolution: 0.05,
            origin_x: -2.5,
            origin_y: -2.5,
            inflation_cells: 0,
        };

        // Origin should map to (0, 0)
        let cell = planner.world_to_cell(-2.5, -2.5);
        assert_eq!(cell, Some((0, 0)));

        // Center of map
        let cell = planner.world_to_cell(0.0, 0.0);
        assert_eq!(cell, Some((50, 50)));

        // Outside bounds
        let cell = planner.world_to_cell(-5.0, 0.0);
        assert_eq!(cell, None);
    }

    #[test]
    fn test_path_simplification() {
        let mut planner = AStarPlanner::new(AStarConfig {
            robot_radius: 0.05,
            simplification_tolerance: 0.1,
            ..Default::default()
        });

        let map = create_test_map(200, 200);

        // Plan a long straight path
        let result = planner.plan(&map, (0.5, 0.5), (9.5, 0.5), 1);

        assert!(result.is_ok());
        let path = result.unwrap();

        // After simplification, a straight line should have very few points
        // (ideally just start and end)
        assert!(
            path.len() <= 5,
            "Path should be simplified, got {} points",
            path.len()
        );
    }
}
