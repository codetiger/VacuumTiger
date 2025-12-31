//! A* pathfinding types.

use crate::core::{GridCoord, WorldPoint};
use crate::query::RobotFootprint;
use std::cmp::Ordering;

/// A node in the A* search
#[derive(Clone, Debug)]
pub(super) struct AStarNode {
    pub coord: GridCoord,
    pub g_cost: f32, // Cost from start
    pub f_cost: f32, // g_cost + heuristic
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
    pub(super) fn failed(reason: PathFailure, nodes_expanded: usize) -> Self {
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
