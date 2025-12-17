//! Frontier-based exploration strategy.
//!
//! Finds boundaries between known-free and unknown cells (frontiers),
//! selects the closest reachable frontier, and navigates toward it.

use crate::core::types::Pose2D;
use crate::state::CurrentMapData;

use super::strategy::{ExplorationAction, ExplorationStrategy, HazardEvent, HazardType};

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
    /// Maximum number of consecutive stuck detections before giving up on a frontier.
    pub max_stuck_count: u32,
}

impl Default for FrontierConfig {
    fn default() -> Self {
        Self {
            min_frontier_size: 5,
            frontier_detection_range: 10.0,
            blocked_radius: 0.3,
            clustering_threshold: 0.5,
            max_stuck_count: 3,
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
}

/// Frontier-based exploration strategy.
///
/// Finds boundaries between known-free and unknown cells,
/// selects the closest frontier, and navigates toward it.
pub struct FrontierExploration {
    config: FrontierConfig,
    /// Frontiers from last computation.
    last_frontiers: Vec<Frontier>,
    /// Locations blocked by bumper (world coordinates).
    blocked_locations: Vec<(f32, f32)>,
    /// Current target frontier (if any).
    current_target: Option<Frontier>,
    /// Number of times we got stuck trying to reach current target.
    stuck_count: u32,
    /// Initial explored area (for completion tracking).
    initial_explored_area: Option<f32>,
    /// Peak explored area seen.
    peak_explored_area: f32,
}

impl FrontierExploration {
    /// Create a new frontier exploration strategy.
    pub fn new(config: FrontierConfig) -> Self {
        Self {
            config,
            last_frontiers: Vec::new(),
            blocked_locations: Vec::new(),
            current_target: None,
            stuck_count: 0,
            initial_explored_area: None,
            peak_explored_area: 0.0,
        }
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

                    // Filter by detection range
                    let dx = wx - robot_pose.x;
                    let dy = wy - robot_pose.y;
                    let dist = (dx * dx + dy * dy).sqrt();

                    if dist <= self.config.frontier_detection_range {
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

        // Convert clusters to Frontiers
        clusters
            .into_iter()
            .filter(|c| c.len() >= self.config.min_frontier_size)
            .map(|cluster| {
                let (cx, cy) = self.compute_centroid(&cluster);
                let dx = cx - robot_pose.x;
                let dy = cy - robot_pose.y;
                let distance = (dx * dx + dy * dy).sqrt();

                Frontier {
                    centroid: Pose2D::new(cx, cy, 0.0),
                    size: cluster.len(),
                    distance,
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

    /// Check if a frontier is blocked (bumper triggered nearby).
    fn is_frontier_blocked(&self, frontier: &Frontier) -> bool {
        let radius_sq = self.config.blocked_radius * self.config.blocked_radius;
        self.blocked_locations.iter().any(|(bx, by)| {
            let dx = frontier.centroid.x - bx;
            let dy = frontier.centroid.y - by;
            (dx * dx + dy * dy) < radius_sq
        })
    }

    /// Select the best frontier to explore.
    fn select_frontier<'a>(&self, frontiers: &'a [Frontier]) -> Option<&'a Frontier> {
        // Filter out blocked frontiers and select closest valid frontier
        frontiers
            .iter()
            .filter(|f| !self.is_frontier_blocked(f))
            .min_by(|a, b| {
                a.distance
                    .partial_cmp(&b.distance)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
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
        // Track explored area for completion estimation
        if self.initial_explored_area.is_none() && map.explored_area_m2 > 0.0 {
            self.initial_explored_area = Some(map.explored_area_m2);
        }
        if map.explored_area_m2 > self.peak_explored_area {
            self.peak_explored_area = map.explored_area_m2;
        }

        // Handle hazard events (bumper or cliff)
        if let Some(event) = hazard {
            // Mark the hazard location as blocked (using world position calculated from sensor offset)
            self.blocked_locations
                .push((event.world_position.x, event.world_position.y));

            let hazard_name = if event.hazard_type.is_cliff() {
                "Cliff"
            } else {
                "Bumper"
            };

            log::info!(
                "{} {:?} triggered at robot ({:.2}, {:.2}), hazard at ({:.2}, {:.2}) - marking as blocked",
                hazard_name,
                event.hazard_type,
                current_pose.x,
                current_pose.y,
                event.world_position.x,
                event.world_position.y
            );

            // Increment stuck counter
            self.stuck_count += 1;

            // If we've been stuck too many times on this target, abandon it
            if self.stuck_count >= self.config.max_stuck_count {
                if let Some(ref target) = self.current_target {
                    self.blocked_locations
                        .push((target.centroid.x, target.centroid.y));
                    log::info!(
                        "Abandoning frontier at ({:.2}, {:.2}) after {} hazard events",
                        target.centroid.x,
                        target.centroid.y,
                        self.stuck_count
                    );
                }
                self.current_target = None;
                self.stuck_count = 0;
            }

            // Handle bumper/cliff response:
            // - Left bumper hit: turn RIGHT (positive angle in robot frame = CCW, but we want CW away from left obstacle)
            // - Right bumper hit: turn LEFT (negative angle = CW, but we want CCW away from right obstacle)
            // - Both bumpers: back up 0.1m, then turn 90°
            //
            // Note: In standard robot convention, positive theta = CCW rotation when viewed from above.
            // To turn away from a left-side obstacle, we need to turn RIGHT (CW) = negative rotation.
            // To turn away from a right-side obstacle, we need to turn LEFT (CCW) = positive rotation.

            match event.hazard_type {
                // Left bumper hit - back up and turn RIGHT (away from left obstacle)
                HazardType::BumperLeft => {
                    // Back up 0.1m and rotate RIGHT (clockwise = negative angle)
                    let backup_distance = 0.1;
                    let turn_angle = -0.35; // ~20° right (negative = clockwise)
                    let back_x = current_pose.x - backup_distance * current_pose.theta.cos();
                    let back_y = current_pose.y - backup_distance * current_pose.theta.sin();
                    return ExplorationAction::MoveTo {
                        target: Pose2D::new(back_x, back_y, current_pose.theta + turn_angle),
                        reason: "backing up from left bumper, turning right",
                    };
                }

                // Right bumper hit - back up and turn LEFT (away from right obstacle)
                HazardType::BumperRight => {
                    // Back up 0.1m and rotate LEFT (counter-clockwise = positive angle)
                    let backup_distance = 0.1;
                    let turn_angle = 0.35; // ~20° left (positive = counter-clockwise)
                    let back_x = current_pose.x - backup_distance * current_pose.theta.cos();
                    let back_y = current_pose.y - backup_distance * current_pose.theta.sin();
                    return ExplorationAction::MoveTo {
                        target: Pose2D::new(back_x, back_y, current_pose.theta + turn_angle),
                        reason: "backing up from right bumper, turning left",
                    };
                }

                // Both bumpers hit - back up 0.1m and turn 90°
                HazardType::BumperBoth => {
                    // Back up 0.1m and rotate 90° (pick a direction, prefer right)
                    let backup_distance = 0.1;
                    let turn_angle = -std::f32::consts::FRAC_PI_2; // 90° right
                    let back_x = current_pose.x - backup_distance * current_pose.theta.cos();
                    let back_y = current_pose.y - backup_distance * current_pose.theta.sin();
                    return ExplorationAction::MoveTo {
                        target: Pose2D::new(back_x, back_y, current_pose.theta + turn_angle),
                        reason: "backing up from both bumpers, turning 90°",
                    };
                }

                // Cliff sensors - back away from the edge
                HazardType::CliffLeftSide => {
                    // Side cliff - rotate in place away from edge (turn right)
                    return ExplorationAction::RotateInPlace {
                        angle_rad: -0.8, // ~45° right
                    };
                }

                HazardType::CliffLeftFront => {
                    // Front-left cliff - back up and turn right
                    let backup_distance = 0.1;
                    let turn_angle = -0.5; // ~30° right
                    let back_x = current_pose.x - backup_distance * current_pose.theta.cos();
                    let back_y = current_pose.y - backup_distance * current_pose.theta.sin();
                    return ExplorationAction::MoveTo {
                        target: Pose2D::new(back_x, back_y, current_pose.theta + turn_angle),
                        reason: "backing up from left-front cliff",
                    };
                }

                HazardType::CliffRightFront => {
                    // Front-right cliff - back up and turn left
                    let backup_distance = 0.1;
                    let turn_angle = 0.5; // ~30° left
                    let back_x = current_pose.x - backup_distance * current_pose.theta.cos();
                    let back_y = current_pose.y - backup_distance * current_pose.theta.sin();
                    return ExplorationAction::MoveTo {
                        target: Pose2D::new(back_x, back_y, current_pose.theta + turn_angle),
                        reason: "backing up from right-front cliff",
                    };
                }

                HazardType::CliffRightSide => {
                    // Side cliff - rotate in place away from edge (turn left)
                    return ExplorationAction::RotateInPlace {
                        angle_rad: 0.8, // ~45° left
                    };
                }
            }
        }

        // Check if we've reached current target
        if let Some(ref target) = self.current_target {
            let dx = target.centroid.x - current_pose.x;
            let dy = target.centroid.y - current_pose.y;
            let dist = (dx * dx + dy * dy).sqrt();

            if dist < 0.3 {
                // Reached target, rotate to scan area
                log::debug!("Reached frontier, rotating to scan");
                self.current_target = None;
                self.stuck_count = 0;
                return ExplorationAction::RotateInPlace {
                    angle_rad: std::f32::consts::PI / 2.0,
                };
            }
        }

        // Find all frontiers
        let frontiers = self.find_frontiers(map, current_pose);
        self.last_frontiers = frontiers.clone();

        // Select best frontier
        if let Some(frontier) = self.select_frontier(&frontiers) {
            let target = frontier.clone();
            log::debug!(
                "Selected frontier at ({:.2}, {:.2}), distance: {:.2}m, size: {}",
                target.centroid.x,
                target.centroid.y,
                target.distance,
                target.size
            );
            self.current_target = Some(target.clone());

            ExplorationAction::MoveTo {
                target: target.centroid,
                reason: "nearest frontier",
            }
        } else if frontiers.is_empty() {
            // No frontiers found - exploration complete
            log::info!("No frontiers remaining - exploration complete!");
            ExplorationAction::Complete
        } else {
            // All frontiers are blocked
            log::warn!(
                "All {} frontiers are blocked - exploration stuck",
                frontiers.len()
            );
            ExplorationAction::Stuck {
                reason: format!("All {} frontiers blocked by obstacles", frontiers.len()),
            }
        }
    }

    fn reset(&mut self) {
        self.last_frontiers.clear();
        self.blocked_locations.clear();
        self.current_target = None;
        self.stuck_count = 0;
        self.initial_explored_area = None;
        self.peak_explored_area = 0.0;
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
}

#[cfg(test)]
mod tests {
    use super::*;

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

        strategy.reset();

        assert!(strategy.blocked_locations.is_empty());
        assert!(strategy.last_frontiers.is_empty());
        assert!(strategy.current_target.is_none());
    }
}
