//! Frontier-based exploration strategy.
//!
//! Finds boundaries between known-free and unknown cells (frontiers),
//! selects the closest reachable frontier, and navigates toward it.
//!
//! Uses A* path planning to verify frontier reachability before selection.

use crate::algorithms::planning::{AStarConfig, AStarPlanner};
use crate::core::types::Pose2D;
use crate::state::CurrentMapData;

use super::strategy::{ExplorationAction, ExplorationStrategy, HazardEvent};

/// Configuration for frontier exploration.
#[derive(Debug, Clone)]
pub struct FrontierConfig {
    /// Minimum number of cells to consider a valid frontier.
    pub min_frontier_size: usize,
    /// Maximum range to search for frontiers (meters).
    pub frontier_detection_range: f32,
    /// Radius around blocked locations to avoid (meters).
    pub blocked_radius: f32,
    /// Distance threshold for grouping frontier cells (meters).
    pub clustering_threshold: f32,
    /// Distance to backup when bumper is hit (meters).
    pub backup_distance: f32,
    /// Rotation angle after backup (radians). Robot rotates this much after each recovery.
    pub recovery_rotation_rad: f32,
    /// Number of directions to check for surrounded detection.
    pub surrounded_check_directions: usize,
    /// Minimum clearance required in a direction to not be considered blocked (meters).
    pub min_clearance_m: f32,
    /// Robot radius for path planning (meters).
    pub robot_radius: f32,
    /// Safety margin beyond robot radius (meters).
    pub safety_margin: f32,
}

impl Default for FrontierConfig {
    fn default() -> Self {
        Self {
            min_frontier_size: 5,
            frontier_detection_range: 10.0,
            blocked_radius: 0.3,
            clustering_threshold: 0.5,
            backup_distance: 0.1,
            recovery_rotation_rad: 0.175,   // ~10 degrees
            surrounded_check_directions: 8, // Check 8 directions (every 45°)
            min_clearance_m: 0.25,          // Need at least 25cm clearance
            robot_radius: 0.18,             // CRL-200S robot radius
            safety_margin: 0.10,            // 10cm extra clearance from walls
        }
    }
}

/// A frontier region on the map boundary.
#[derive(Debug, Clone)]
pub struct Frontier {
    /// Centroid of the frontier in world coordinates.
    pub centroid: Pose2D,
    /// Number of frontier cells in this group.
    pub size: usize,
    /// Distance from robot to frontier centroid.
    pub distance: f32,
    /// Original frontier cell positions (for edge-point fallback).
    pub cells: Vec<(f32, f32)>,
}

/// Recovery state after a hazard event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RecoveryPhase {
    /// No recovery in progress.
    None,
    /// Backing up along the path we came from.
    BackingUp,
    /// Rotating after backup to try a new approach.
    Rotating,
}

/// Frontier-based exploration strategy.
///
/// Finds boundaries between known-free and unknown cells,
/// selects the closest reachable frontier, and navigates toward it.
/// Uses A* path planning to verify frontiers are reachable before selecting them.
pub struct FrontierExploration {
    config: FrontierConfig,
    /// A* planner for reachability checking.
    planner: AStarPlanner,
    /// Frontiers from last computation.
    last_frontiers: Vec<Frontier>,
    /// Locations blocked by bumper (world coordinates).
    blocked_locations: Vec<(f32, f32)>,
    /// Recently visited locations with timestamps (cooldown to prevent re-selection).
    recently_visited: Vec<((f32, f32), std::time::Instant)>,
    /// Cooldown duration for recently visited locations (seconds).
    visited_cooldown_secs: f32,
    /// Current target frontier (if any).
    current_target: Option<Frontier>,
    /// Current navigation target point (may differ from centroid if edge-point fallback used).
    current_target_point: Option<(f32, f32)>,
    /// Path history for reverse direction calculation (recent positions).
    path_history: Vec<(f32, f32)>,
    /// Maximum path history size.
    max_path_history: usize,
    /// Current recovery phase.
    recovery_phase: RecoveryPhase,
    /// Target pose for current recovery action.
    recovery_target: Option<Pose2D>,
    /// Number of recovery attempts on current target.
    recovery_attempts: u32,
    /// Number of successfully reached frontiers (for blocked cleanup).
    successful_visits: u32,
    /// Initial explored area (for completion tracking).
    initial_explored_area: Option<f32>,
    /// Peak explored area seen.
    peak_explored_area: f32,
    /// Last map data for surrounded detection.
    last_map: Option<CurrentMapData>,
}

impl FrontierExploration {
    /// Create a new frontier exploration strategy.
    pub fn new(config: FrontierConfig) -> Self {
        // Configure A* for exploration: allow routing through unknown space
        let planner_config = AStarConfig {
            robot_radius: config.robot_radius,
            safety_margin: config.safety_margin,
            allow_diagonal: true,
            max_iterations: 50_000, // Lower limit for quick reachability check
            simplification_tolerance: 0.0, // Don't need simplified paths for reachability check
            unknown_is_obstacle: false, // Allow planning through unknown (we're exploring!)
        };

        Self {
            config,
            planner: AStarPlanner::new(planner_config),
            last_frontiers: Vec::new(),
            blocked_locations: Vec::new(),
            recently_visited: Vec::new(),
            visited_cooldown_secs: 10.0, // 10 second cooldown before re-visiting
            current_target: None,
            current_target_point: None,
            path_history: Vec::with_capacity(100),
            max_path_history: 100,
            recovery_phase: RecoveryPhase::None,
            recovery_target: None,
            recovery_attempts: 0,
            successful_visits: 0,
            initial_explored_area: None,
            peak_explored_area: 0.0,
            last_map: None,
        }
    }

    /// Check if a frontier was recently visited (in cooldown period).
    fn is_recently_visited(&self, frontier: &Frontier) -> bool {
        let radius_sq = self.config.blocked_radius * self.config.blocked_radius;
        let now = std::time::Instant::now();

        self.recently_visited.iter().any(|((vx, vy), visited_time)| {
            // Check if still in cooldown
            if now.duration_since(*visited_time).as_secs_f32() > self.visited_cooldown_secs {
                return false;
            }
            // Check if frontier is near this visited location
            let dx = frontier.centroid.x - vx;
            let dy = frontier.centroid.y - vy;
            (dx * dx + dy * dy) < radius_sq
        })
    }

    /// Clean up expired entries from recently_visited list.
    fn cleanup_recently_visited(&mut self) {
        let now = std::time::Instant::now();
        self.recently_visited.retain(|(_, visited_time)| {
            now.duration_since(*visited_time).as_secs_f32() < self.visited_cooldown_secs
        });
    }

    /// Record current position in path history.
    fn record_position(&mut self, x: f32, y: f32) {
        // Only record if significantly different from last position
        if let Some(&(last_x, last_y)) = self.path_history.last() {
            let dx = x - last_x;
            let dy = y - last_y;
            let dist_sq = dx * dx + dy * dy;
            if dist_sq < 0.01 * 0.01 {
                // Less than 1cm, don't record
                return;
            }
        }

        self.path_history.push((x, y));

        // Keep history bounded
        if self.path_history.len() > self.max_path_history {
            self.path_history.remove(0);
        }
    }

    /// Get the direction to back up (reverse along path we came from).
    /// Returns the angle in radians pointing back along our path.
    fn get_reverse_direction(&self, current_x: f32, current_y: f32) -> Option<f32> {
        // Find a position in our history that's at least backup_distance away
        let min_dist = self.config.backup_distance * 2.0; // Look for point 2x backup distance away

        for &(hx, hy) in self.path_history.iter().rev() {
            let dx = hx - current_x;
            let dy = hy - current_y;
            let dist = (dx * dx + dy * dy).sqrt();

            if dist >= min_dist {
                // Return angle pointing toward this historical position
                return Some(dy.atan2(dx));
            }
        }

        // If no suitable history point, try the oldest point we have
        if let Some(&(hx, hy)) = self.path_history.first() {
            let dx = hx - current_x;
            let dy = hy - current_y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist > 0.02 {
                // At least 2cm away
                return Some(dy.atan2(dx));
            }
        }

        None
    }

    /// Check if robot is surrounded by obstacles in all directions.
    /// Returns true if there's no clear direction to escape.
    fn is_surrounded(&self, map: &CurrentMapData, robot_pose: &Pose2D) -> bool {
        let num_directions = self.config.surrounded_check_directions;
        let angle_step = 2.0 * std::f32::consts::PI / num_directions as f32;
        let check_distance = self.config.min_clearance_m;

        let mut blocked_count = 0;

        for i in 0..num_directions {
            let angle = i as f32 * angle_step;
            let check_x = robot_pose.x + check_distance * angle.cos();
            let check_y = robot_pose.y + check_distance * angle.sin();

            if self.is_point_blocked(map, check_x, check_y) {
                blocked_count += 1;
            }
        }

        // Surrounded if ALL directions are blocked
        blocked_count >= num_directions
    }

    /// Check if a point is blocked (obstacle or out of bounds).
    fn is_point_blocked(&self, map: &CurrentMapData, x: f32, y: f32) -> bool {
        let cx = ((x - map.origin_x) / map.resolution) as i32;
        let cy = ((y - map.origin_y) / map.resolution) as i32;

        if cx < 0 || cy < 0 || cx >= map.width as i32 || cy >= map.height as i32 {
            return true; // Out of bounds = blocked
        }

        let idx = (cy as usize) * (map.width as usize) + (cx as usize);
        if idx >= map.cells.len() {
            return true;
        }

        // Occupied if value >= 100 (but not unknown 255 for exploration purposes)
        let cell_value = map.cells[idx];
        cell_value >= 100 && cell_value != 255
    }

    /// Check if a frontier is trapped inside an enclosed area.
    ///
    /// A frontier is considered "trapped" if obstacles surround it from all directions
    /// within a short distance. This typically happens when there's a small pocket of
    /// unknown cells completely enclosed by walls - the frontier exists but is unreachable.
    fn is_frontier_trapped(&self, frontier: &Frontier, map: &CurrentMapData) -> bool {
        let num_directions = 8; // Check every 45 degrees
        let angle_step = 2.0 * std::f32::consts::PI / num_directions as f32;
        let check_distance = 0.5; // Check 50cm out from frontier
        let step_size = 0.05; // 5cm steps

        let fx = frontier.centroid.x;
        let fy = frontier.centroid.y;

        let mut blocked_directions = 0;

        for i in 0..num_directions {
            let angle = i as f32 * angle_step;
            let mut hit_obstacle = false;

            // Ray-cast in this direction
            let mut dist = step_size;
            while dist <= check_distance {
                let check_x = fx + dist * angle.cos();
                let check_y = fy + dist * angle.sin();

                if self.is_point_blocked(map, check_x, check_y) {
                    hit_obstacle = true;
                    break;
                }
                dist += step_size;
            }

            if hit_obstacle {
                blocked_directions += 1;
            }
        }

        // Frontier is trapped if ALL or almost all directions hit obstacles
        // Allow at most 1 unblocked direction (could be the frontier edge itself)
        let trapped = blocked_directions >= num_directions - 1;

        if trapped {
            log::debug!(
                "Frontier at ({:.2}, {:.2}) is TRAPPED ({}/{} directions blocked)",
                fx,
                fy,
                blocked_directions,
                num_directions
            );
        }

        trapped
    }

    /// Find the direction with most clearance from obstacles.
    fn find_best_escape_direction(&self, map: &CurrentMapData, robot_pose: &Pose2D) -> f32 {
        let num_directions = 16; // Check every 22.5 degrees
        let angle_step = 2.0 * std::f32::consts::PI / num_directions as f32;
        let max_check_distance = 1.0; // Check up to 1m away
        let step_size = 0.05; // 5cm steps

        let mut best_angle = robot_pose.theta + std::f32::consts::PI; // Default: opposite of current heading
        let mut best_clearance = 0.0f32;

        for i in 0..num_directions {
            let angle = i as f32 * angle_step;
            let mut clearance = 0.0f32;

            // Ray-cast in this direction to find clearance
            let mut dist = step_size;
            while dist <= max_check_distance {
                let check_x = robot_pose.x + dist * angle.cos();
                let check_y = robot_pose.y + dist * angle.sin();

                if self.is_point_blocked(map, check_x, check_y) {
                    break;
                }
                clearance = dist;
                dist += step_size;
            }

            if clearance > best_clearance {
                best_clearance = clearance;
                best_angle = angle;
            }
        }

        best_angle
    }

    /// Find all frontier cells in the map.
    ///
    /// A frontier cell is an Unknown cell adjacent to at least one Free cell.
    fn find_frontiers(&self, map: &CurrentMapData, robot_pose: &Pose2D) -> Vec<Frontier> {
        let width = map.width as usize;
        let height = map.height as usize;
        let resolution = map.resolution;

        if width < 3 || height < 3 {
            return Vec::new();
        }

        // Find all frontier cells (Unknown adjacent to Free)
        let mut frontier_cells: Vec<(f32, f32)> = Vec::new();

        for cy in 1..height - 1 {
            for cx in 1..width - 1 {
                let idx = cy * width + cx;
                let cell_value = map.cells[idx];

                // Unknown cells have value 255
                if cell_value != 255 {
                    continue;
                }

                // Check if adjacent to any Free cell (value 0)
                let has_free_neighbor = self.has_free_neighbor(map, cx, cy, width);

                if has_free_neighbor {
                    // Convert to world coordinates
                    let wx = map.origin_x + (cx as f32 + 0.5) * resolution;
                    let wy = map.origin_y + (cy as f32 + 0.5) * resolution;

                    // Soft range filter: collect all frontiers within extended range
                    // Use 3x the configured range to catch more frontiers without going infinite
                    let dx = wx - robot_pose.x;
                    let dy = wy - robot_pose.y;
                    let dist = (dx * dx + dy * dy).sqrt();

                    if dist <= self.config.frontier_detection_range * 3.0 {
                        frontier_cells.push((wx, wy));
                    }
                }
            }
        }

        // Cluster frontier cells into groups
        self.cluster_frontiers(frontier_cells, robot_pose)
    }

    /// Check if a cell has at least one free neighbor.
    fn has_free_neighbor(&self, map: &CurrentMapData, cx: usize, cy: usize, width: usize) -> bool {
        let height = map.height as usize;
        let neighbors = [
            (cx.wrapping_sub(1), cy),
            (cx + 1, cy),
            (cx, cy.wrapping_sub(1)),
            (cx, cy + 1),
        ];

        for (nx, ny) in neighbors {
            if nx < width && ny < height {
                let idx = ny * width + nx;
                if map.cells[idx] == 0 {
                    // Free cell
                    return true;
                }
            }
        }
        false
    }

    /// Cluster frontier cells into groups and compute centroids.
    fn cluster_frontiers(&self, cells: Vec<(f32, f32)>, robot_pose: &Pose2D) -> Vec<Frontier> {
        if cells.is_empty() {
            return Vec::new();
        }

        // Simple clustering: assign each cell to nearest cluster or create new one
        let threshold_sq = self.config.clustering_threshold * self.config.clustering_threshold;
        let mut clusters: Vec<Vec<(f32, f32)>> = Vec::new();

        for (wx, wy) in cells {
            // Find nearest cluster
            let mut best_cluster = None;
            let mut best_dist_sq = threshold_sq;

            for (i, cluster) in clusters.iter().enumerate() {
                // Check distance to cluster centroid
                let (cx, cy) = self.compute_centroid(cluster);
                let dx = wx - cx;
                let dy = wy - cy;
                let dist_sq = dx * dx + dy * dy;

                if dist_sq < best_dist_sq {
                    best_dist_sq = dist_sq;
                    best_cluster = Some(i);
                }
            }

            // Add to existing cluster or create new one
            if let Some(idx) = best_cluster {
                clusters[idx].push((wx, wy));
            } else {
                clusters.push(vec![(wx, wy)]);
            }
        }

        // Convert clusters to Frontiers (keep small min_size filter for noise reduction)
        let min_size = self.config.min_frontier_size.max(2); // At least 2 cells
        clusters
            .into_iter()
            .filter(|c| c.len() >= min_size)
            .map(|cluster| {
                let (cx, cy) = self.compute_centroid(&cluster);
                let dx = cx - robot_pose.x;
                let dy = cy - robot_pose.y;
                let distance = (dx * dx + dy * dy).sqrt();

                Frontier {
                    centroid: Pose2D::new(cx, cy, 0.0),
                    size: cluster.len(),
                    distance,
                    cells: cluster, // Preserve cells for edge-point fallback
                }
            })
            .collect()
    }

    /// Compute centroid of a set of points.
    fn compute_centroid(&self, points: &[(f32, f32)]) -> (f32, f32) {
        if points.is_empty() {
            return (0.0, 0.0);
        }
        let sum_x: f32 = points.iter().map(|(x, _)| x).sum();
        let sum_y: f32 = points.iter().map(|(_, y)| y).sum();
        let n = points.len() as f32;
        (sum_x / n, sum_y / n)
    }

    /// Check if a frontier is blocked (bumper triggered nearby or navigation failed).
    fn is_frontier_blocked(&self, frontier: &Frontier) -> bool {
        // Use a larger radius for blocking (0.5m) to account for centroid drift
        // The config.blocked_radius is for bumper obstacles, but navigation failures
        // need more margin since centroids shift as map updates
        let blocked_radius = self.config.blocked_radius.max(0.5);
        let radius_sq = blocked_radius * blocked_radius;

        let is_blocked = self.blocked_locations.iter().any(|(bx, by)| {
            let dx = frontier.centroid.x - bx;
            let dy = frontier.centroid.y - by;
            (dx * dx + dy * dy) < radius_sq
        });

        if is_blocked {
            log::debug!(
                "Frontier at ({:.2}, {:.2}) is BLOCKED (within {:.2}m of a blocked location, {} blocked total)",
                frontier.centroid.x,
                frontier.centroid.y,
                blocked_radius,
                self.blocked_locations.len()
            );
        }

        is_blocked
    }

    /// Check if a specific point is reachable via A* path planning.
    fn is_point_reachable(
        &mut self,
        x: f32,
        y: f32,
        robot_pose: &Pose2D,
        map: &CurrentMapData,
    ) -> bool {
        let start = (robot_pose.x, robot_pose.y);
        let goal = (x, y);
        self.planner.plan(map, start, goal, 0).is_ok()
    }

    /// Score a frontier for selection priority.
    /// Higher score = better candidate.
    fn score_frontier(&self, frontier: &Frontier) -> f32 {
        // Distance score: closer is better (inverse relationship)
        // Range: approaches 1.0 for very close, approaches 0 for very far
        let distance_score = 1.0 / (1.0 + frontier.distance);

        // Size score: larger frontiers are slightly better (capped at 1.0)
        // This prevents huge distant frontiers from overriding close small ones
        let size_score = ((frontier.size as f32).sqrt() / 10.0).min(1.0);

        // Heavily prioritize distance (80%) with slight size preference (20%)
        // This ensures we explore systematically from close to far
        distance_score * 0.8 + size_score * 0.2
    }

    /// Find a reachable point on a frontier (centroid first, then edge points).
    fn find_reachable_point_on_frontier(
        &mut self,
        frontier: &Frontier,
        robot_pose: &Pose2D,
        map: &CurrentMapData,
    ) -> Option<(f32, f32)> {
        // 1. Try centroid first (most common case)
        if self.is_point_reachable(frontier.centroid.x, frontier.centroid.y, robot_pose, map) {
            return Some((frontier.centroid.x, frontier.centroid.y));
        }

        // 2. Try edge points sorted by distance to robot
        let mut edge_points: Vec<(f32, f32, f32)> = frontier
            .cells
            .iter()
            .map(|(x, y)| {
                let dx = x - robot_pose.x;
                let dy = y - robot_pose.y;
                (*x, *y, dx * dx + dy * dy)
            })
            .collect();

        edge_points.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));

        // Try up to 5 closest edge points
        for (x, y, _) in edge_points.iter().take(5) {
            if self.is_point_reachable(*x, *y, robot_pose, map) {
                log::debug!(
                    "Using edge point ({:.2}, {:.2}) instead of centroid ({:.2}, {:.2})",
                    x,
                    y,
                    frontier.centroid.x,
                    frontier.centroid.y
                );
                return Some((*x, *y));
            }
        }

        None
    }

    /// Attempt breakthrough when all frontiers are blocked/unreachable.
    /// Finds the nearest unknown cell adjacent to free space that IS reachable.
    fn attempt_breakthrough(
        &mut self,
        map: &CurrentMapData,
        robot_pose: &Pose2D,
    ) -> Option<ExplorationAction> {
        let nearest = self.find_nearest_reachable_unknown(map, robot_pose)?;

        log::info!(
            "Attempting breakthrough to nearest unknown at ({:.2}, {:.2})",
            nearest.0,
            nearest.1
        );

        Some(ExplorationAction::NavigateTo {
            target: Pose2D::new(nearest.0, nearest.1, 0.0),
            reason: "breakthrough to unknown area",
        })
    }

    /// BFS to find nearest unknown cell adjacent to free space that is reachable.
    fn find_nearest_reachable_unknown(
        &mut self,
        map: &CurrentMapData,
        robot_pose: &Pose2D,
    ) -> Option<(f32, f32)> {
        use std::collections::VecDeque;

        let width = map.width as usize;
        let height = map.height as usize;
        let resolution = map.resolution;

        // Convert robot position to cell coordinates
        let robot_cx = ((robot_pose.x - map.origin_x) / resolution) as i32;
        let robot_cy = ((robot_pose.y - map.origin_y) / resolution) as i32;

        if robot_cx < 0 || robot_cy < 0 || robot_cx >= width as i32 || robot_cy >= height as i32 {
            return None;
        }

        let mut visited = vec![false; width * height];
        let mut queue = VecDeque::new();

        queue.push_back((robot_cx as usize, robot_cy as usize, 0));
        visited[robot_cy as usize * width + robot_cx as usize] = true;

        // BFS to find nearest unknown cell adjacent to free space
        while let Some((cx, cy, dist)) = queue.pop_front() {
            // Limit search radius
            if dist > 200 {
                break;
            }

            let idx = cy * width + cx;
            let cell_value = map.cells[idx];

            // Check if this is an unknown cell adjacent to free space
            if cell_value == 255 && self.has_free_neighbor(map, cx, cy, width) {
                // Verify it's actually reachable (find a nearby free cell to navigate to)
                // Navigate to the free cell adjacent to this unknown, not the unknown itself
                if let Some((target_x, target_y)) =
                    self.find_adjacent_free_cell(map, cx, cy, width, resolution)
                    && self.is_point_reachable(target_x, target_y, robot_pose, map)
                {
                    return Some((target_x, target_y));
                }
            }

            // Add neighbors to queue
            for (dx, dy) in [(0, 1), (1, 0), (0, -1), (-1, 0)] {
                let nx = cx as i32 + dx;
                let ny = cy as i32 + dy;

                if nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height {
                    let nidx = ny as usize * width + nx as usize;
                    if !visited[nidx] {
                        visited[nidx] = true;
                        // Only traverse through free or unknown cells
                        let nval = map.cells[nidx];
                        if nval == 0 || nval == 255 {
                            queue.push_back((nx as usize, ny as usize, dist + 1));
                        }
                    }
                }
            }
        }

        None
    }

    /// Find an adjacent free cell to navigate to.
    fn find_adjacent_free_cell(
        &self,
        map: &CurrentMapData,
        cx: usize,
        cy: usize,
        width: usize,
        resolution: f32,
    ) -> Option<(f32, f32)> {
        let height = map.height as usize;
        for (dx, dy) in [(0, 1), (1, 0), (0, -1), (-1, 0)] {
            let nx = cx as i32 + dx;
            let ny = cy as i32 + dy;

            if nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height {
                let nidx = ny as usize * width + nx as usize;
                if map.cells[nidx] == 0 {
                    // Free cell
                    let wx = map.origin_x + (nx as f32 + 0.5) * resolution;
                    let wy = map.origin_y + (ny as f32 + 0.5) * resolution;
                    return Some((wx, wy));
                }
            }
        }
        None
    }

    /// Select the best frontier to explore using distance priority and edge-point fallback.
    ///
    /// Returns the frontier and the specific point to navigate to (may be edge point).
    fn select_frontier<'a>(
        &mut self,
        frontiers: &'a [Frontier],
        robot_pose: &Pose2D,
        map: &CurrentMapData,
    ) -> Option<(&'a Frontier, (f32, f32))> {
        // Clean up expired cooldown entries
        self.cleanup_recently_visited();

        // Filter frontiers that are:
        // - Not blocked (previous navigation failures)
        // - Not recently visited (cooldown period)
        // - Not trapped inside enclosed areas
        let mut candidates: Vec<&Frontier> = frontiers
            .iter()
            .filter(|f| {
                !self.is_frontier_blocked(f)
                    && !self.is_recently_visited(f)
                    && !self.is_frontier_trapped(f, map)
            })
            .collect();

        // Sort by distance (ascending - closest first)
        candidates.sort_by(|a, b| {
            a.distance
                .partial_cmp(&b.distance)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Try to find a reachable point on each frontier in distance order
        for frontier in candidates {
            if let Some(target_point) =
                self.find_reachable_point_on_frontier(frontier, robot_pose, map)
            {
                return Some((frontier, target_point));
            }
        }

        None
    }
}

impl ExplorationStrategy for FrontierExploration {
    fn name(&self) -> &'static str {
        "frontier"
    }

    fn next_action(
        &mut self,
        map: &CurrentMapData,
        current_pose: &Pose2D,
        hazard: Option<&HazardEvent>,
    ) -> ExplorationAction {
        // Store map for surrounded detection
        self.last_map = Some(map.clone());

        // Track explored area for completion estimation
        if self.initial_explored_area.is_none() && map.explored_area_m2 > 0.0 {
            self.initial_explored_area = Some(map.explored_area_m2);
        }
        if map.explored_area_m2 > self.peak_explored_area {
            self.peak_explored_area = map.explored_area_m2;
        }

        // Record position in path history (only when not in recovery)
        if self.recovery_phase == RecoveryPhase::None {
            self.record_position(current_pose.x, current_pose.y);
        }

        // Handle ongoing recovery phases
        match self.recovery_phase {
            RecoveryPhase::BackingUp => {
                // Check if we've reached the backup target
                if let Some(ref target) = self.recovery_target {
                    let dx = target.x - current_pose.x;
                    let dy = target.y - current_pose.y;
                    let dist = (dx * dx + dy * dy).sqrt();

                    if dist < 0.05 {
                        // Reached backup position, now rotate
                        log::info!(
                            "Backup complete, rotating {} degrees before re-routing",
                            (self.config.recovery_rotation_rad * 180.0 / std::f32::consts::PI)
                                as i32
                        );
                        self.recovery_phase = RecoveryPhase::Rotating;

                        // Alternate rotation direction based on recovery attempts
                        let rotation_sign = if self.recovery_attempts.is_multiple_of(2) {
                            1.0
                        } else {
                            -1.0
                        };
                        return ExplorationAction::RotateInPlace {
                            angle_rad: self.config.recovery_rotation_rad * rotation_sign,
                        };
                    }

                    // Continue moving to backup target
                    return ExplorationAction::MoveTo {
                        target: *target,
                        reason: "backing up along path",
                    };
                }
            }

            RecoveryPhase::Rotating => {
                // Rotation is a single action, transition to re-routing
                log::info!(
                    "Recovery attempt {} complete, re-routing to target",
                    self.recovery_attempts
                );
                self.recovery_phase = RecoveryPhase::None;
                self.recovery_target = None;

                // Clear path history after recovery to build fresh path
                self.path_history.clear();

                // Will fall through to normal frontier selection below
            }

            RecoveryPhase::None => {
                // No recovery in progress
            }
        }

        // Handle new hazard events (bumper or cliff)
        if let Some(event) = hazard {
            // Mark the hazard location as blocked (using world position calculated from sensor offset)
            self.blocked_locations
                .push((event.world_position.x, event.world_position.y));

            let hazard_name = if event.hazard_type.is_cliff() {
                "Cliff"
            } else {
                "Bumper"
            };

            self.recovery_attempts += 1;

            log::info!(
                "{} {:?} triggered at robot ({:.2}, {:.2}), hazard at ({:.2}, {:.2}) - recovery attempt {}",
                hazard_name,
                event.hazard_type,
                current_pose.x,
                current_pose.y,
                event.world_position.x,
                event.world_position.y,
                self.recovery_attempts
            );

            // Check if robot is surrounded - only then give up on this target
            if self.is_surrounded(map, current_pose) {
                log::warn!(
                    "Robot is surrounded by obstacles after {} recovery attempts - abandoning target",
                    self.recovery_attempts
                );

                if let Some(ref target) = self.current_target {
                    self.blocked_locations
                        .push((target.centroid.x, target.centroid.y));
                    log::info!(
                        "Marking frontier at ({:.2}, {:.2}) as blocked",
                        target.centroid.x,
                        target.centroid.y
                    );
                }

                self.current_target = None;
                self.current_target_point = None;
                self.recovery_attempts = 0;
                self.recovery_phase = RecoveryPhase::None;
                self.path_history.clear();

                // Try to find best escape direction
                let escape_angle = self.find_best_escape_direction(map, current_pose);
                return ExplorationAction::RotateInPlace {
                    angle_rad: escape_angle - current_pose.theta,
                };
            }

            // Start recovery: backup along the path we came from
            self.recovery_phase = RecoveryPhase::BackingUp;

            // Calculate backup direction - reverse along our path
            let backup_angle = if let Some(reverse_angle) =
                self.get_reverse_direction(current_pose.x, current_pose.y)
            {
                reverse_angle
            } else {
                // No path history - use best escape direction from map analysis
                self.find_best_escape_direction(map, current_pose)
            };

            // Calculate backup position
            let backup_distance = self.config.backup_distance;
            let back_x = current_pose.x + backup_distance * backup_angle.cos();
            let back_y = current_pose.y + backup_distance * backup_angle.sin();

            // Store recovery target
            self.recovery_target = Some(Pose2D::new(back_x, back_y, current_pose.theta));

            log::info!(
                "Starting recovery: backing up to ({:.2}, {:.2}) along path direction {:.1}°",
                back_x,
                back_y,
                backup_angle * 180.0 / std::f32::consts::PI
            );

            return ExplorationAction::MoveTo {
                target: Pose2D::new(back_x, back_y, current_pose.theta),
                reason: "backing up along path after hazard",
            };
        }

        // Check if we've reached current target point
        if let Some((tx, ty)) = self.current_target_point {
            let dx = tx - current_pose.x;
            let dy = ty - current_pose.y;
            let dist = (dx * dx + dy * dy).sqrt();

            if dist < 0.3 {
                // Reached target, rotate to scan area
                log::debug!("Reached frontier target, rotating to scan");
                self.current_target = None;
                self.current_target_point = None;
                self.recovery_attempts = 0; // Reset recovery counter on success
                self.successful_visits += 1;

                // Periodically clean up old blocked locations after successful visits
                if self.successful_visits.is_multiple_of(3) && self.blocked_locations.len() > 5 {
                    let keep_count = self.blocked_locations.len() / 2;
                    let removed = self.blocked_locations.len() - keep_count;
                    self.blocked_locations = self.blocked_locations.split_off(keep_count);
                    log::info!(
                        "Cleared {} old blocked locations (keeping {})",
                        removed,
                        self.blocked_locations.len()
                    );
                }

                return ExplorationAction::RotateInPlace {
                    angle_rad: std::f32::consts::PI / 2.0,
                };
            }
        }

        // Find all frontiers
        let frontiers = self.find_frontiers(map, current_pose);
        self.last_frontiers = frontiers.clone();

        // Select best reachable frontier (with edge-point fallback)
        if let Some((frontier, target_point)) = self.select_frontier(&frontiers, current_pose, map)
        {
            let target = frontier.clone();
            log::debug!(
                "Selected frontier at ({:.2}, {:.2}), target point: ({:.2}, {:.2}), distance: {:.2}m, size: {}, score: {:.3}",
                target.centroid.x,
                target.centroid.y,
                target_point.0,
                target_point.1,
                target.distance,
                target.size,
                self.score_frontier(&target)
            );
            self.current_target = Some(target);
            self.current_target_point = Some(target_point);

            // Use NavigateTo for path-planned navigation to frontier
            ExplorationAction::NavigateTo {
                target: Pose2D::new(target_point.0, target_point.1, 0.0),
                reason: "reachable frontier",
            }
        } else if frontiers.is_empty() {
            // No frontiers found - exploration complete
            log::info!("No frontiers remaining - exploration complete!");
            ExplorationAction::Complete
        } else {
            // All frontiers are blocked or unreachable - try breakthrough
            log::warn!(
                "All {} frontiers blocked/unreachable - attempting breakthrough",
                frontiers.len()
            );

            if let Some(action) = self.attempt_breakthrough(map, current_pose) {
                return action;
            }

            // Truly stuck - no reachable unknown cells
            log::warn!("Breakthrough failed - exploration stuck");
            ExplorationAction::Stuck {
                reason: format!(
                    "All {} frontiers blocked, breakthrough failed",
                    frontiers.len()
                ),
            }
        }
    }

    fn reset(&mut self) {
        self.last_frontiers.clear();
        self.blocked_locations.clear();
        self.current_target = None;
        self.current_target_point = None;
        self.path_history.clear();
        self.recovery_phase = RecoveryPhase::None;
        self.recovery_target = None;
        self.recovery_attempts = 0;
        self.successful_visits = 0;
        self.initial_explored_area = None;
        self.peak_explored_area = 0.0;
        self.last_map = None;
    }

    fn frontier_count(&self) -> usize {
        self.last_frontiers.len()
    }

    fn completion_percent(&self) -> f32 {
        // Rough estimate based on frontier count decrease
        // This is a heuristic - actual completion is when no frontiers remain
        if self.last_frontiers.is_empty() {
            100.0
        } else {
            // More frontiers = less complete
            // Assume starting with ~100 frontiers means 0% complete
            let max_frontiers = 100.0;
            let percent = (1.0 - (self.last_frontiers.len() as f32 / max_frontiers)) * 100.0;
            percent.clamp(0.0, 99.0) // Never 100% until truly complete
        }
    }

    fn mark_target_blocked(&mut self, x: f32, y: f32) {
        log::info!(
            "Marking failed navigation target as blocked: ({:.2}, {:.2})",
            x,
            y
        );
        self.blocked_locations.push((x, y));

        // Clear current target since it failed
        self.current_target = None;
        self.current_target_point = None;
    }

    fn mark_target_reached(&mut self) {
        if let Some(ref target) = self.current_target {
            log::info!(
                "Frontier target reached successfully at ({:.2}, {:.2})",
                target.centroid.x,
                target.centroid.y
            );

            // Add to recently visited cooldown list to prevent immediate re-selection
            // This handles the case where we reached "close enough" but didn't fully clear the frontier
            self.recently_visited
                .push(((target.centroid.x, target.centroid.y), std::time::Instant::now()));
        }

        // Clear current target - navigation completed
        self.current_target = None;
        self.current_target_point = None;
        self.recovery_attempts = 0;
        self.successful_visits += 1;

        // Periodically clean up old blocked locations after successful visits
        if self.successful_visits.is_multiple_of(3) && self.blocked_locations.len() > 5 {
            let keep_count = self.blocked_locations.len() / 2;
            let removed = self.blocked_locations.len() - keep_count;
            self.blocked_locations = self.blocked_locations.split_off(keep_count);
            log::info!(
                "Cleared {} old blocked locations (keeping {})",
                removed,
                self.blocked_locations.len()
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::exploration::strategy;

    use super::*;
    use strategy::HazardType;

    fn create_test_map(width: u32, height: u32) -> CurrentMapData {
        let size = (width * height) as usize;
        let mut cells = vec![255u8; size]; // All unknown

        // Create a small explored region in the center
        let cx = width / 2;
        let cy = height / 2;
        let radius = 3;

        for dy in 0..radius * 2 {
            for dx in 0..radius * 2 {
                let x = (cx as i32 - radius as i32 + dx as i32) as u32;
                let y = (cy as i32 - radius as i32 + dy as i32) as u32;
                if x < width && y < height {
                    let idx = (y * width + x) as usize;
                    cells[idx] = 0; // Free
                }
            }
        }

        CurrentMapData {
            map_id: "test".to_string(),
            name: "Test Map".to_string(),
            resolution: 0.05,
            width,
            height,
            origin_x: 0.0,
            origin_y: 0.0,
            cells,
            explored_area_m2: 1.0,
        }
    }

    #[test]
    fn test_frontier_detection() {
        let config = FrontierConfig::default();
        let strategy = FrontierExploration::new(config);

        let map = create_test_map(20, 20);
        let pose = Pose2D::new(0.5, 0.5, 0.0);

        let frontiers = strategy.find_frontiers(&map, &pose);

        // Should find frontiers around the explored region
        assert!(!frontiers.is_empty());
    }

    #[test]
    fn test_empty_map_no_frontiers() {
        let config = FrontierConfig::default();
        let strategy = FrontierExploration::new(config);

        // Map with all unknown cells (no free cells = no frontiers)
        let map = CurrentMapData {
            map_id: "test".to_string(),
            name: "Test".to_string(),
            resolution: 0.05,
            width: 10,
            height: 10,
            origin_x: 0.0,
            origin_y: 0.0,
            cells: vec![255u8; 100], // All unknown
            explored_area_m2: 0.0,
        };

        let pose = Pose2D::new(0.25, 0.25, 0.0);
        let frontiers = strategy.find_frontiers(&map, &pose);

        assert!(frontiers.is_empty());
    }

    #[test]
    fn test_bumper_blocks_location() {
        let config = FrontierConfig::default();
        let mut strategy = FrontierExploration::new(config);

        let map = create_test_map(20, 20);
        let pose = Pose2D::new(0.5, 0.5, 0.0);

        // Simulate bumper trigger using HazardEvent
        let hazard = HazardEvent::new(HazardType::BumperLeft, &pose);
        let action = strategy.next_action(&map, &pose, Some(&hazard));

        // Should get a backing up action
        assert!(matches!(action, ExplorationAction::MoveTo { .. }));

        // The hazard location should be blocked
        assert!(!strategy.blocked_locations.is_empty());
    }

    #[test]
    fn test_cliff_blocks_location() {
        let config = FrontierConfig::default();
        let mut strategy = FrontierExploration::new(config);

        let map = create_test_map(20, 20);
        let pose = Pose2D::new(0.5, 0.5, 0.0);

        // Simulate cliff sensor trigger
        let hazard = HazardEvent::new(HazardType::CliffLeftFront, &pose);
        let action = strategy.next_action(&map, &pose, Some(&hazard));

        // Should get a backing up action
        assert!(matches!(action, ExplorationAction::MoveTo { .. }));

        // The hazard location should be blocked
        assert!(!strategy.blocked_locations.is_empty());
    }

    #[test]
    fn test_reset_clears_state() {
        let config = FrontierConfig::default();
        let mut strategy = FrontierExploration::new(config);

        // Add some blocked locations
        strategy.blocked_locations.push((1.0, 1.0));
        strategy.blocked_locations.push((2.0, 2.0));
        strategy.path_history.push((0.0, 0.0));
        strategy.recovery_attempts = 5;

        strategy.reset();

        assert!(strategy.blocked_locations.is_empty());
        assert!(strategy.last_frontiers.is_empty());
        assert!(strategy.current_target.is_none());
        assert!(strategy.path_history.is_empty());
        assert_eq!(strategy.recovery_attempts, 0);
        assert_eq!(strategy.recovery_phase, RecoveryPhase::None);
    }

    #[test]
    fn test_path_history_recording() {
        let config = FrontierConfig::default();
        let mut strategy = FrontierExploration::new(config);

        // Record positions
        strategy.record_position(0.0, 0.0);
        strategy.record_position(0.1, 0.0);
        strategy.record_position(0.2, 0.0);
        strategy.record_position(0.3, 0.0);

        assert_eq!(strategy.path_history.len(), 4);

        // Recording same position should not add
        strategy.record_position(0.3, 0.0);
        assert_eq!(strategy.path_history.len(), 4);
    }

    #[test]
    fn test_reverse_direction_calculation() {
        let config = FrontierConfig::default();
        let mut strategy = FrontierExploration::new(config);

        // Build a path history going forward along X axis
        strategy.record_position(0.0, 0.0);
        strategy.record_position(0.1, 0.0);
        strategy.record_position(0.2, 0.0);
        strategy.record_position(0.3, 0.0);

        // Get reverse direction from current position (0.4, 0.0)
        let reverse_angle = strategy.get_reverse_direction(0.4, 0.0);

        assert!(reverse_angle.is_some());
        // Should point back along the negative X direction (approximately PI radians)
        let angle = reverse_angle.unwrap();
        assert!(
            (angle - std::f32::consts::PI).abs() < 0.1
                || (angle + std::f32::consts::PI).abs() < 0.1
        );
    }

    #[test]
    fn test_is_point_blocked() {
        let config = FrontierConfig::default();
        let strategy = FrontierExploration::new(config);

        // Create a map with an obstacle
        let mut map = create_test_map(20, 20);
        // Mark center cell as obstacle
        let center_idx = 10 * 20 + 10;
        map.cells[center_idx] = 100;

        // Point at obstacle should be blocked
        let obstacle_x = map.origin_x + 10.5 * map.resolution;
        let obstacle_y = map.origin_y + 10.5 * map.resolution;
        assert!(strategy.is_point_blocked(&map, obstacle_x, obstacle_y));

        // Point at free cell should not be blocked
        let free_x = map.origin_x + 5.5 * map.resolution;
        let free_y = map.origin_y + 5.5 * map.resolution;
        assert!(!strategy.is_point_blocked(&map, free_x, free_y));
    }
}
