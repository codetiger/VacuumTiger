//! Theta* path planner for any-angle pathfinding.
//!
//! Theta* extends A* with line-of-sight checks to produce smoother,
//! shorter paths that aren't constrained to grid directions.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use vastu_slam::{GridCoord, Pose2D, WorldPoint};

use super::cost_map::{CostMap, costs};

/// Configuration for the Theta* path planner.
#[derive(Clone, Debug)]
pub struct ThetaStarConfig {
    /// Maximum iterations before giving up
    pub max_iterations: usize,
    /// Weight for wall proximity penalty in path cost
    pub wall_penalty_weight: f32,
    /// Goal tolerance in cells
    pub goal_tolerance: i32,
}

impl Default for ThetaStarConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50000,
            wall_penalty_weight: 2.0,
            goal_tolerance: 2,
        }
    }
}

/// Result of path planning.
#[derive(Clone, Debug)]
pub struct PlannedPath {
    /// Waypoints in world coordinates
    pub waypoints: Vec<WorldPoint>,
    /// Total path length in meters
    pub length: f32,
}

/// Node in the search graph.
#[derive(Clone, Debug)]
struct SearchNode {
    coord: GridCoord,
    f_score: f32,
}

impl PartialEq for SearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.coord == other.coord
    }
}

impl Eq for SearchNode {}

impl Ord for SearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (lower f_score = higher priority)
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for SearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Theta* path planner.
pub struct ThetaStarPlanner {
    config: ThetaStarConfig,
}

impl ThetaStarPlanner {
    /// Create a new Theta* planner with configuration.
    pub fn new(config: ThetaStarConfig) -> Self {
        Self { config }
    }

    /// Create a new Theta* planner with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(ThetaStarConfig::default())
    }

    /// Plan a path from start pose to goal point.
    ///
    /// Returns `None` if no valid path exists.
    pub fn plan(&self, cost_map: &CostMap, start: Pose2D, goal: WorldPoint) -> Option<PlannedPath> {
        let start_coord = cost_map.world_to_grid(WorldPoint::new(start.x, start.y));
        let goal_coord = cost_map.world_to_grid(goal);

        // Check if start is valid, if not find nearest traversable cell
        let actual_start = if !cost_map.is_traversable(start_coord) {
            tracing::warn!("Start position is not traversable, searching for nearby valid cell");
            if let Some(valid_start) = Self::find_nearest_traversable(cost_map, start_coord, 10) {
                valid_start
            } else {
                tracing::warn!("No traversable cell found near start position");
                return None;
            }
        } else {
            start_coord
        };

        // Check if goal is valid
        // Note: We don't use find_nearest_traversable for the goal because if the specified
        // goal is blocked, we should fail explicitly rather than silently redirecting to a
        // different location. The explorer/caller should handle goal validation.
        if !cost_map.is_traversable(goal_coord) {
            tracing::warn!(
                "Goal position ({}, {}) is not traversable",
                goal_coord.x,
                goal_coord.y
            );
            return None;
        }
        let actual_goal = goal_coord;

        // Run Theta* search
        let path_coords = self.theta_star_search(cost_map, actual_start, actual_goal)?;

        // Convert to world coordinates and compute statistics
        let waypoints: Vec<WorldPoint> = path_coords
            .iter()
            .map(|&coord| cost_map.grid_to_world(coord))
            .collect();

        let length = self.compute_path_length(&waypoints);

        Some(PlannedPath { waypoints, length })
    }

    /// Theta* search algorithm.
    fn theta_star_search(
        &self,
        cost_map: &CostMap,
        start: GridCoord,
        goal: GridCoord,
    ) -> Option<Vec<GridCoord>> {
        let mut open_set = BinaryHeap::new();
        let mut g_score: HashMap<GridCoord, f32> = HashMap::new();
        let mut parent: HashMap<GridCoord, GridCoord> = HashMap::new();
        let mut closed_set: HashMap<GridCoord, bool> = HashMap::new();

        // Initialize start node
        g_score.insert(start, 0.0);
        parent.insert(start, start); // Start is its own parent
        open_set.push(SearchNode {
            coord: start,
            f_score: Self::heuristic(start, goal),
        });

        let mut iterations = 0;

        // 8-connected neighbors
        let neighbors = [
            GridCoord::new(-1, 0),
            GridCoord::new(1, 0),
            GridCoord::new(0, -1),
            GridCoord::new(0, 1),
            GridCoord::new(-1, -1),
            GridCoord::new(1, -1),
            GridCoord::new(-1, 1),
            GridCoord::new(1, 1),
        ];

        while let Some(current_node) = open_set.pop() {
            iterations += 1;
            if iterations > self.config.max_iterations {
                tracing::warn!("Theta* exceeded max iterations");
                return None;
            }

            let current = current_node.coord;

            // Check if we've reached the goal (with tolerance)
            if current.manhattan_distance(&goal) <= self.config.goal_tolerance {
                return Some(self.reconstruct_path(&parent, current));
            }

            // Skip if already processed
            if closed_set.contains_key(&current) {
                continue;
            }
            closed_set.insert(current, true);

            let current_g = *g_score.get(&current).unwrap_or(&f32::MAX);
            let current_parent = *parent.get(&current).unwrap_or(&current);

            // Explore neighbors
            for &offset in &neighbors {
                let neighbor = GridCoord::new(current.x + offset.x, current.y + offset.y);

                // Skip if not traversable
                if cost_map.cost(neighbor) >= costs::INSCRIBED {
                    continue;
                }

                // Skip if already processed
                if closed_set.contains_key(&neighbor) {
                    continue;
                }

                // Theta* key insight: check line-of-sight to grandparent
                let (new_g, new_parent) = if current_parent != current
                    && cost_map.line_of_sight(current_parent, neighbor)
                {
                    // Path 2: Direct path from grandparent (any-angle)
                    let parent_g = *g_score.get(&current_parent).unwrap_or(&f32::MAX);
                    let edge_cost = self.path_cost(cost_map, current_parent, neighbor);
                    (parent_g + edge_cost, current_parent)
                } else {
                    // Path 1: Through current node (grid-constrained)
                    let edge_cost = self.path_cost(cost_map, current, neighbor);
                    (current_g + edge_cost, current)
                };

                let existing_g = *g_score.get(&neighbor).unwrap_or(&f32::MAX);

                if new_g < existing_g {
                    g_score.insert(neighbor, new_g);
                    parent.insert(neighbor, new_parent);

                    let f_score = new_g + Self::heuristic(neighbor, goal);
                    open_set.push(SearchNode {
                        coord: neighbor,
                        f_score,
                    });
                }
            }
        }

        // No path found
        None
    }

    /// Compute heuristic (Euclidean distance).
    #[inline]
    fn heuristic(from: GridCoord, to: GridCoord) -> f32 {
        let dx = (to.x - from.x) as f32;
        let dy = (to.y - from.y) as f32;
        (dx * dx + dy * dy).sqrt()
    }

    /// Compute path cost between two points including wall penalty.
    fn path_cost(&self, cost_map: &CostMap, from: GridCoord, to: GridCoord) -> f32 {
        let dx = (to.x - from.x) as f32;
        let dy = (to.y - from.y) as f32;
        let distance = (dx * dx + dy * dy).sqrt();

        let wall_penalty = cost_map.integrate_wall_penalty(from, to);

        distance + self.config.wall_penalty_weight * wall_penalty * distance
    }

    /// Reconstruct path from parent map.
    fn reconstruct_path(
        &self,
        parent: &HashMap<GridCoord, GridCoord>,
        goal: GridCoord,
    ) -> Vec<GridCoord> {
        let mut path = Vec::new();
        let mut current = goal;

        loop {
            path.push(current);
            match parent.get(&current) {
                Some(&p) if p != current => current = p,
                _ => break,
            }
        }

        path.reverse();
        path
    }

    /// Compute path length.
    fn compute_path_length(&self, waypoints: &[WorldPoint]) -> f32 {
        if waypoints.len() < 2 {
            return 0.0;
        }

        let mut length = 0.0;

        for i in 0..waypoints.len() - 1 {
            let from = waypoints[i];
            let to = waypoints[i + 1];

            let dx = to.x - from.x;
            let dy = to.y - from.y;
            length += (dx * dx + dy * dy).sqrt();
        }

        length
    }

    /// Find the nearest traversable cell to a given coordinate.
    /// Uses BFS to search in expanding rings around the target.
    fn find_nearest_traversable(
        cost_map: &CostMap,
        center: GridCoord,
        max_radius: i32,
    ) -> Option<GridCoord> {
        // Check center first
        if cost_map.is_traversable(center) {
            return Some(center);
        }

        // Search in expanding rings
        for r in 1..=max_radius {
            // Check all cells at distance r (Manhattan-ish)
            for dx in -r..=r {
                for dy in -r..=r {
                    // Only check cells on the "ring" boundary
                    if dx.abs() != r && dy.abs() != r {
                        continue;
                    }
                    let coord = GridCoord::new(center.x + dx, center.y + dy);
                    if cost_map.is_traversable(coord) {
                        return Some(coord);
                    }
                }
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vastu_slam::{CellType, GridStorage};

    fn create_test_grid() -> GridStorage {
        let mut grid = GridStorage::new(50, 50, 0.05, WorldPoint::ZERO);

        // Mark all as floor
        for y in 0..50 {
            for x in 0..50 {
                grid.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        // Add a wall in the middle
        for x in 10..40 {
            grid.set_type(GridCoord::new(x, 25), CellType::Wall);
        }

        grid
    }

    #[test]
    fn test_simple_path() {
        let grid = create_test_grid();
        let cost_map = CostMap::from_grid(&grid, 0.10, 0.05, 0.20);
        let planner = ThetaStarPlanner::with_defaults();

        let start = Pose2D::new(0.5, 0.5, 0.0);
        let goal = WorldPoint::new(2.0, 0.5);

        let path = planner.plan(&cost_map, start, goal);
        assert!(path.is_some());

        let path = path.unwrap();
        assert!(path.waypoints.len() >= 2);
        assert!(path.length > 0.0);
    }

    #[test]
    fn test_path_around_obstacle() {
        let grid = create_test_grid();
        let cost_map = CostMap::from_grid(&grid, 0.10, 0.05, 0.20);
        let planner = ThetaStarPlanner::with_defaults();

        // Path that needs to go around the wall
        let start = Pose2D::new(1.25, 1.0, 0.0);
        let goal = WorldPoint::new(1.25, 1.5);

        let path = planner.plan(&cost_map, start, goal);
        assert!(path.is_some());

        let path = path.unwrap();
        // Path should go around, so it will be longer than direct line
        assert!(path.length > 0.5);
    }

    #[test]
    fn test_no_path_to_blocked_goal() {
        let mut grid = GridStorage::new(20, 20, 0.05, WorldPoint::ZERO);

        // Create an enclosed area
        for y in 0..20 {
            for x in 0..20 {
                grid.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }
        // Surround center with walls
        for x in 8..12 {
            grid.set_type(GridCoord::new(x, 8), CellType::Wall);
            grid.set_type(GridCoord::new(x, 12), CellType::Wall);
        }
        for y in 8..12 {
            grid.set_type(GridCoord::new(8, y), CellType::Wall);
            grid.set_type(GridCoord::new(12, y), CellType::Wall);
        }

        let cost_map = CostMap::from_grid(&grid, 0.10, 0.05, 0.20);
        let planner = ThetaStarPlanner::with_defaults();

        let start = Pose2D::new(0.25, 0.25, 0.0);
        let goal = WorldPoint::new(0.5, 0.5); // Inside enclosed area

        let path = planner.plan(&cost_map, start, goal);
        assert!(path.is_none());
    }
}
