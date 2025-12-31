//! A* planner implementation.

use crate::core::{GridCoord, WorldPoint};
use crate::grid::GridStorage;
use crate::query::TraversabilityChecker;
use log::{debug, trace};
use std::collections::{BinaryHeap, HashMap, HashSet};

use super::types::{AStarConfig, AStarNode, PathFailure, PathResult};

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
