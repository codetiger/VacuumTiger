//! Frontier detection for autonomous exploration.
//!
//! Identifies boundaries between explored and unexplored space,
//! clusters them into frontier regions, and scores them for selection.

use std::collections::{HashSet, VecDeque};

use vastu_slam::{CellType, GridCoord, GridStorage, Pose2D, WorldPoint};

use crate::planning::CostMap;

/// Configuration for frontier detection.
#[derive(Clone, Debug)]
pub struct FrontierConfig {
    /// Minimum number of cells for a valid frontier cluster
    pub min_cluster_size: usize,
    /// Maximum number of frontiers to consider
    pub max_frontiers: usize,
    /// Weight for frontier size in scoring
    pub size_weight: f32,
    /// Weight for distance penalty in scoring
    pub distance_weight: f32,
    /// Weight for information gain (unexplored neighbors)
    pub info_gain_weight: f32,
    /// Distance threshold for clustering frontier cells
    pub cluster_distance: i32,
}

impl Default for FrontierConfig {
    fn default() -> Self {
        Self {
            min_cluster_size: 5,
            max_frontiers: 20,
            // Weights tuned for normalized scoring (all components in [0,1] range)
            // Higher distance_weight prioritizes nearby frontiers to reduce backtracking
            size_weight: 0.5,
            distance_weight: 1.5,
            info_gain_weight: 1.0,
            cluster_distance: 3,
        }
    }
}

/// Unique identifier for a frontier based on its centroid.
/// Used for blacklisting and tracking frontiers across updates.
#[derive(Clone, Debug, Hash, Eq, PartialEq)]
pub struct FrontierId {
    pub centroid: GridCoord,
}

/// A frontier region (cluster of frontier cells).
#[derive(Clone, Debug)]
pub struct Frontier {
    /// Centroid of the frontier (in grid coordinates)
    pub centroid: GridCoord,
    /// Centroid in world coordinates
    pub world_centroid: WorldPoint,
    /// Size (number of cells)
    pub size: usize,
    /// Estimated information gain (unexplored cells nearby)
    pub info_gain: f32,
    /// Euclidean distance to robot (meters)
    pub distance: f32,
    /// Score (higher is better, set by normalize_scores)
    pub score: f32,
}

impl Frontier {
    /// Get a unique identifier for this frontier.
    pub fn id(&self) -> FrontierId {
        FrontierId {
            centroid: self.centroid,
        }
    }
}

/// Frontier detector for exploration.
pub struct FrontierDetector {
    config: FrontierConfig,
}

impl FrontierDetector {
    /// Create a new frontier detector with configuration.
    pub fn new(config: FrontierConfig) -> Self {
        Self { config }
    }

    /// Detect all frontiers in the grid.
    ///
    /// Returns frontiers sorted by score (highest first).
    pub fn detect(
        &self,
        grid: &GridStorage,
        cost_map: &CostMap,
        robot_pose: Pose2D,
    ) -> Vec<Frontier> {
        // Step 1: Find all frontier cells
        let frontier_cells = self.find_frontier_cells(grid, cost_map);

        if frontier_cells.is_empty() {
            return Vec::new();
        }

        // Step 2: Cluster frontier cells
        let clusters = self.cluster_frontiers(&frontier_cells);

        // Step 3: Filter and create frontiers (with raw info_gain/distance)
        let mut frontiers: Vec<Frontier> = clusters
            .into_iter()
            .filter(|cluster| cluster.len() >= self.config.min_cluster_size)
            .map(|cluster| self.create_frontier(cluster, grid, cost_map, robot_pose))
            .collect();

        // Step 3.5: Filter unreachable frontiers
        // Removes frontiers where no valid navigation target exists
        // (e.g., frontiers too close to walls discovered after initial detection)
        let pre_filter_count = frontiers.len();
        frontiers.retain(|frontier| {
            self.navigation_target(frontier, cost_map, robot_pose)
                .is_some()
        });
        if frontiers.len() < pre_filter_count {
            tracing::debug!(
                "Filtered {} unreachable frontiers (kept {})",
                pre_filter_count - frontiers.len(),
                frontiers.len()
            );
        }

        // Step 4: Normalize scores across all frontiers
        // This makes weights meaningful by scaling info_gain, distance, size to [0,1]
        self.normalize_scores(&mut frontiers);

        // Step 5: Sort by normalized score (descending)
        frontiers.sort_by(|a, b| {
            b.score
                .partial_cmp(&a.score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Step 6: Limit number of frontiers
        frontiers.truncate(self.config.max_frontiers);

        frontiers
    }

    /// Find all frontier cells (floor cells adjacent to unknown).
    ///
    /// Note: Frontier cells may be in the inflation zone (near unknown space).
    /// The path planner will find a safe approach point.
    fn find_frontier_cells(&self, grid: &GridStorage, _cost_map: &CostMap) -> Vec<GridCoord> {
        let mut frontier_cells = Vec::new();
        let width = grid.width();
        let height = grid.height();

        // 4-connected neighbors for frontier detection
        let neighbors = [
            GridCoord::new(-1, 0),
            GridCoord::new(1, 0),
            GridCoord::new(0, -1),
            GridCoord::new(0, 1),
        ];

        for y in 1..(height as i32 - 1) {
            for x in 1..(width as i32 - 1) {
                let coord = GridCoord::new(x, y);

                // Must be a floor cell
                if grid.get_type(coord) != CellType::Floor {
                    continue;
                }

                // Check if adjacent to unknown
                let is_frontier = neighbors.iter().any(|offset| {
                    let neighbor = GridCoord::new(coord.x + offset.x, coord.y + offset.y);
                    grid.get_type(neighbor) == CellType::Unknown
                });

                if is_frontier {
                    frontier_cells.push(coord);
                }
            }
        }

        frontier_cells
    }

    /// Cluster frontier cells using flood-fill.
    fn cluster_frontiers(&self, frontier_cells: &[GridCoord]) -> Vec<Vec<GridCoord>> {
        if frontier_cells.is_empty() {
            return Vec::new();
        }

        // Create set for fast lookup
        let frontier_set: HashSet<GridCoord> = frontier_cells.iter().copied().collect();
        let mut visited: HashSet<GridCoord> = HashSet::new();
        let mut clusters = Vec::new();

        // 8-connected neighbors for clustering
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

        for &start_cell in frontier_cells {
            if visited.contains(&start_cell) {
                continue;
            }

            // BFS to find connected frontier cells
            let mut cluster = Vec::new();
            let mut queue = VecDeque::new();
            queue.push_back(start_cell);
            visited.insert(start_cell);

            while let Some(current) = queue.pop_front() {
                cluster.push(current);

                for &_offset in &neighbors {
                    // Check cells within cluster distance
                    for dy in -self.config.cluster_distance..=self.config.cluster_distance {
                        for dx in -self.config.cluster_distance..=self.config.cluster_distance {
                            let candidate = GridCoord::new(current.x + dx, current.y + dy);
                            if frontier_set.contains(&candidate) && !visited.contains(&candidate) {
                                visited.insert(candidate);
                                queue.push_back(candidate);
                            }
                        }
                    }
                }
            }

            if !cluster.is_empty() {
                clusters.push(cluster);
            }
        }

        clusters
    }

    /// Create a Frontier from a cluster of cells.
    /// Note: score is set to 0.0 here and will be computed by normalize_scores().
    fn create_frontier(
        &self,
        cells: Vec<GridCoord>,
        grid: &GridStorage,
        cost_map: &CostMap,
        robot_pose: Pose2D,
    ) -> Frontier {
        let size = cells.len();

        // Compute centroid
        let sum_x: i32 = cells.iter().map(|c| c.x).sum();
        let sum_y: i32 = cells.iter().map(|c| c.y).sum();
        let centroid = GridCoord::new(sum_x / size as i32, sum_y / size as i32);

        // Convert to world coordinates
        let world_centroid = cost_map.grid_to_world(centroid);

        // Compute information gain (count unknown cells near frontier)
        let info_gain = self.estimate_info_gain(&cells, grid);

        // Compute distance to robot
        let robot_pos = WorldPoint::new(robot_pose.x, robot_pose.y);
        let distance = robot_pos.distance(&world_centroid);

        // Score will be set by normalize_scores() after all frontiers are created
        let score = 0.0;

        Frontier {
            centroid,
            world_centroid,
            size,
            info_gain,
            distance,
            score,
        }
    }

    /// Estimate information gain for a frontier.
    fn estimate_info_gain(&self, cells: &[GridCoord], grid: &GridStorage) -> f32 {
        let mut unknown_count = 0;
        let mut checked = HashSet::new();

        // Sample cells to check (for efficiency, check subset)
        let sample_step = (cells.len() / 10).max(1);

        for (i, &cell) in cells.iter().enumerate() {
            if i % sample_step != 0 {
                continue;
            }

            // Check 5x5 neighborhood
            for dy in -2..=2 {
                for dx in -2..=2 {
                    let neighbor = GridCoord::new(cell.x + dx, cell.y + dy);
                    if checked.insert(neighbor) && grid.get_type(neighbor) == CellType::Unknown {
                        unknown_count += 1;
                    }
                }
            }
        }

        unknown_count as f32
    }

    /// Normalize frontier scores to make weights meaningful.
    ///
    /// Normalizes info_gain, distance, and size to [0, 1] range before applying
    /// configured weights. This ensures distance can compete with info_gain
    /// regardless of their raw value ranges.
    fn normalize_scores(&self, frontiers: &mut [Frontier]) {
        if frontiers.is_empty() {
            return;
        }

        const EPSILON: f32 = 1e-6;

        // Find min/max for each component
        let (min_info, max_info) = frontiers
            .iter()
            .map(|f| f.info_gain)
            .fold((f32::MAX, f32::MIN), |(min, max), v| {
                (min.min(v), max.max(v))
            });

        let (min_size, max_size) = frontiers
            .iter()
            .map(|f| (f.size as f32).sqrt())
            .fold((f32::MAX, f32::MIN), |(min, max), v| {
                (min.min(v), max.max(v))
            });

        let (min_dist, max_dist) = frontiers
            .iter()
            .map(|f| f.distance)
            .fold((f32::MAX, f32::MIN), |(min, max), v| {
                (min.min(v), max.max(v))
            });

        // Compute ranges (avoid division by zero)
        let info_range = (max_info - min_info).max(EPSILON);
        let size_range = (max_size - min_size).max(EPSILON);
        let dist_range = (max_dist - min_dist).max(EPSILON);

        // Compute normalized scores for each frontier
        for frontier in frontiers.iter_mut() {
            let norm_info = (frontier.info_gain - min_info) / info_range;
            let norm_size = ((frontier.size as f32).sqrt() - min_size) / size_range;
            let norm_dist = (frontier.distance - min_dist) / dist_range;

            frontier.score = self.config.size_weight * norm_size
                + self.config.info_gain_weight * norm_info
                - self.config.distance_weight * norm_dist;
        }
    }

    /// Get the navigation target for a frontier.
    ///
    /// Returns a traversable point near the frontier that the robot can safely reach.
    /// Uses a multi-stage approach:
    /// 1. Try small safety offset toward robot
    /// 2. Try centroid directly
    /// 3. BFS to find closest traversable cell (up to 5m radius)
    ///
    /// Returns `None` if no traversable point is found.
    pub fn navigation_target(
        &self,
        frontier: &Frontier,
        cost_map: &CostMap,
        robot_pose: Pose2D,
    ) -> Option<WorldPoint> {
        let centroid = frontier.world_centroid;
        let robot_pos = WorldPoint::new(robot_pose.x, robot_pose.y);

        // Calculate direction from centroid toward robot (into explored space)
        let dx = robot_pos.x - centroid.x;
        let dy = robot_pos.y - centroid.y;
        let dist_to_centroid = (dx * dx + dy * dy).sqrt();

        // Stage 1: Try small safety offset (5cm) toward robot
        let safety_offset = 0.05;
        let min_path_distance = 0.20;

        if dist_to_centroid > min_path_distance + safety_offset {
            let scale = safety_offset / dist_to_centroid;
            let safe_target = WorldPoint::new(centroid.x + dx * scale, centroid.y + dy * scale);

            let grid_coord = cost_map.world_to_grid(safe_target);
            if cost_map.is_traversable(grid_coord) {
                tracing::debug!(
                    "Frontier target: centroid ({:.2},{:.2}) -> safe ({:.2},{:.2}), dist={:.2}",
                    centroid.x,
                    centroid.y,
                    safe_target.x,
                    safe_target.y,
                    dist_to_centroid
                );
                return Some(safe_target);
            }
        }

        // Stage 2: Try centroid directly
        let centroid_coord = cost_map.world_to_grid(centroid);
        if cost_map.is_traversable(centroid_coord) {
            tracing::debug!(
                "Frontier target: using centroid directly ({:.2},{:.2})",
                centroid.x,
                centroid.y
            );
            return Some(centroid);
        }

        // Stage 3: BFS to find closest reachable cell (up to 5m)
        let max_search_radius = 5.0; // meters
        if let Some(reachable) = cost_map.find_closest_reachable_world(centroid, max_search_radius)
        {
            let distance_from_centroid = centroid.distance(&reachable);
            tracing::info!(
                "Frontier target: found reachable point {:.2}m from centroid at ({:.2},{:.2})",
                distance_from_centroid,
                reachable.x,
                reachable.y
            );
            return Some(reachable);
        }

        // No reachable point found within search radius
        tracing::warn!(
            "Frontier target: no reachable point found within {:.1}m of ({:.2},{:.2})",
            max_search_radius,
            centroid.x,
            centroid.y
        );
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_grid() -> GridStorage {
        // 50x50 grid with partial exploration (larger grid to avoid inflation issues)
        let mut grid = GridStorage::new(50, 50, 0.05, WorldPoint::ZERO);

        // Mark explored area (floor) in center region
        for y in 10..40 {
            for x in 10..25 {
                grid.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        // Rest is unknown (default)
        grid
    }

    #[test]
    fn test_frontier_detection() {
        let grid = create_test_grid();
        // Use smaller robot radius for test to avoid all frontiers being in inflation zone
        let cost_map = CostMap::from_grid(&grid, 0.05, 0.02, 0.10);
        let detector = FrontierDetector::new(FrontierConfig::default());

        let robot_pose = Pose2D::new(0.75, 1.0, 0.0);
        let frontiers = detector.detect(&grid, &cost_map, robot_pose);

        // Should find frontiers at the boundary of explored area
        assert!(
            !frontiers.is_empty(),
            "Expected frontiers at Floor/Unknown boundary"
        );
    }

    #[test]
    fn test_no_frontiers_fully_explored() {
        // Fully explored grid (all floor)
        let mut grid = GridStorage::new(20, 20, 0.05, WorldPoint::ZERO);
        for y in 0..20 {
            for x in 0..20 {
                grid.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let cost_map = CostMap::from_grid(&grid, 0.05, 0.02, 0.10);
        let detector = FrontierDetector::new(FrontierConfig::default());

        let robot_pose = Pose2D::new(0.5, 0.5, 0.0);
        let frontiers = detector.detect(&grid, &cost_map, robot_pose);

        // No frontiers (no unknown cells)
        assert!(frontiers.is_empty());
    }

    #[test]
    fn test_frontier_scoring() {
        let grid = create_test_grid();
        let cost_map = CostMap::from_grid(&grid, 0.05, 0.02, 0.10);
        let detector = FrontierDetector::new(FrontierConfig::default());

        let robot_pose = Pose2D::new(0.75, 1.0, 0.0);
        let frontiers = detector.detect(&grid, &cost_map, robot_pose);

        if frontiers.len() > 1 {
            // Frontiers should be sorted by score (descending)
            for i in 0..frontiers.len() - 1 {
                assert!(frontiers[i].score >= frontiers[i + 1].score);
            }
        }
    }
}
