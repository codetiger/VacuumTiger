//! Exploration planning with region-based TSP ordering.
//!
//! Provides global exploration planning by ordering regions to minimize
//! total travel distance while ensuring complete coverage.

use std::f32::consts::PI;

use crate::Frontier;
use crate::core::Point2D;

use super::history::ExplorationHistory;
use super::region::{ExplorationRegion, RegionConfig, RegionDetector};

/// Configuration for exploration planning.
#[derive(Clone, Debug)]
pub struct PlannerConfig {
    /// Weight for region priority in selection (0.0 - 1.0).
    /// Default: 0.3
    pub priority_weight: f32,

    /// Weight for distance penalty in region selection.
    /// Default: 0.5
    pub distance_weight: f32,

    /// Penalty for revisiting regions.
    /// Default: 0.5
    pub revisit_penalty: f32,

    /// Bonus per frontier in a region.
    /// Default: 0.1
    pub frontier_bonus: f32,

    /// Weight for heading alignment when selecting frontiers within a region.
    /// Default: 0.3
    pub heading_weight: f32,

    /// Weight for frontier openness.
    /// Default: 0.4
    pub openness_weight: f32,

    /// Weight for local distance within a region.
    /// Default: 0.3
    pub local_distance_weight: f32,

    /// Interval before forcing a full replan (seconds).
    /// Default: 10.0
    pub replan_interval_secs: f32,

    /// Threshold of frontier count change to trigger replan.
    /// Default: 5
    pub frontier_change_threshold: usize,

    /// Region detection configuration.
    pub region_config: RegionConfig,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            priority_weight: 0.3,
            distance_weight: 0.5,
            revisit_penalty: 0.5,
            frontier_bonus: 0.1,
            heading_weight: 0.3,
            openness_weight: 0.4,
            local_distance_weight: 0.3,
            replan_interval_secs: 120.0, // Replan less often to maintain region focus
            frontier_change_threshold: 50, // Only replan on major changes
            region_config: RegionConfig::default(),
        }
    }
}

impl PlannerConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for heading weight.
    pub fn with_heading_weight(mut self, weight: f32) -> Self {
        self.heading_weight = weight;
        self
    }

    /// Builder-style setter for openness weight.
    pub fn with_openness_weight(mut self, weight: f32) -> Self {
        self.openness_weight = weight;
        self
    }

    /// Builder-style setter for local distance weight.
    pub fn with_local_distance_weight(mut self, weight: f32) -> Self {
        self.local_distance_weight = weight;
        self
    }

    /// Builder-style setter for region config.
    pub fn with_region_config(mut self, config: RegionConfig) -> Self {
        self.region_config = config;
        self
    }
}

/// Global exploration plan with ordered regions.
#[derive(Clone, Debug)]
pub struct ExplorationPlan {
    /// Ordered list of regions to visit.
    pub regions: Vec<ExplorationRegion>,

    /// Current region index being explored.
    pub current_region_idx: usize,

    /// Whether the plan needs recomputation.
    pub stale: bool,

    /// Number of frontiers at last planning.
    pub last_frontier_count: usize,

    /// Elapsed time since last replan (seconds).
    pub time_since_replan: f32,
}

impl Default for ExplorationPlan {
    fn default() -> Self {
        Self::new()
    }
}

impl ExplorationPlan {
    /// Create a new empty exploration plan.
    pub fn new() -> Self {
        Self {
            regions: Vec::new(),
            current_region_idx: 0,
            stale: true,
            last_frontier_count: 0,
            time_since_replan: 0.0,
        }
    }

    /// Check if the plan is empty (no regions).
    pub fn is_empty(&self) -> bool {
        self.regions.is_empty()
    }

    /// Get the current region being explored.
    pub fn current_region(&self) -> Option<&ExplorationRegion> {
        self.regions.get(self.current_region_idx)
    }

    /// Get the current region mutably.
    pub fn current_region_mut(&mut self) -> Option<&mut ExplorationRegion> {
        self.regions.get_mut(self.current_region_idx)
    }

    /// Advance to the next region.
    pub fn advance_region(&mut self) {
        if let Some(region) = self.regions.get_mut(self.current_region_idx) {
            region.visit_count += 1;
        }
        self.current_region_idx += 1;
    }

    /// Check if all regions have been processed.
    pub fn is_complete(&self) -> bool {
        self.current_region_idx >= self.regions.len()
    }

    /// Get the total number of remaining frontiers across all regions.
    pub fn remaining_frontier_count(&self) -> usize {
        self.regions
            .iter()
            .skip(self.current_region_idx)
            .map(|r| r.frontier_count())
            .sum()
    }
}

/// Exploration planner that manages region detection and ordering.
pub struct ExplorationPlanner {
    /// Region detector instance.
    region_detector: RegionDetector,

    /// Current exploration plan.
    plan: ExplorationPlan,

    /// Configuration.
    config: PlannerConfig,
}

impl ExplorationPlanner {
    /// Create a new exploration planner.
    pub fn new(config: PlannerConfig) -> Self {
        Self {
            region_detector: RegionDetector::new(),
            plan: ExplorationPlan::new(),
            config,
        }
    }

    /// Get the current exploration plan.
    pub fn plan(&self) -> &ExplorationPlan {
        &self.plan
    }

    /// Get the current exploration plan mutably.
    pub fn plan_mut(&mut self) -> &mut ExplorationPlan {
        &mut self.plan
    }

    /// Update the exploration plan if needed.
    ///
    /// # Arguments
    /// * `frontiers` - Current list of frontiers
    /// * `robot_pos` - Current robot position
    /// * `history` - Exploration history for visit tracking
    /// * `dt` - Time since last update (seconds)
    pub fn update(
        &mut self,
        frontiers: &[Frontier],
        robot_pos: Point2D,
        history: &ExplorationHistory,
        dt: f32,
    ) {
        self.plan.time_since_replan += dt;

        if self.should_replan(frontiers) {
            self.replan(frontiers, robot_pos, history);
        } else {
            // Incremental update: refresh frontiers in current region
            self.update_current_region(frontiers);
        }
    }

    /// Force a full replan.
    pub fn force_replan(
        &mut self,
        frontiers: &[Frontier],
        robot_pos: Point2D,
        history: &ExplorationHistory,
    ) {
        self.replan(frontiers, robot_pos, history);
    }

    /// Check if a replan is needed.
    fn should_replan(&self, frontiers: &[Frontier]) -> bool {
        // Plan is stale
        if self.plan.stale {
            return true;
        }

        // Time-based replan
        if self.plan.time_since_replan >= self.config.replan_interval_secs {
            return true;
        }

        // Frontier count changed significantly
        let frontier_change =
            (frontiers.len() as i32 - self.plan.last_frontier_count as i32).unsigned_abs() as usize;
        if frontier_change >= self.config.frontier_change_threshold {
            return true;
        }

        // Current region exhausted
        if self.plan.current_region_idx >= self.plan.regions.len() {
            return true;
        }

        // Current region has no frontiers left
        if let Some(region) = self.plan.current_region()
            && region.is_empty()
        {
            return true;
        }

        false
    }

    /// Perform a full replan.
    fn replan(&mut self, frontiers: &[Frontier], robot_pos: Point2D, history: &ExplorationHistory) {
        // Remember previous region centroid to try to maintain context
        let prev_centroid = self.plan.current_region().map(|r| r.centroid);

        // Detect regions
        let mut regions = self
            .region_detector
            .detect(frontiers, &self.config.region_config);

        // Order regions using greedy TSP
        let ordered_regions = self.order_regions(&mut regions, robot_pos, history);

        // Update plan
        self.plan.regions = ordered_regions;
        self.plan.stale = false;
        self.plan.last_frontier_count = frontiers.len();
        self.plan.time_since_replan = 0.0;

        // Try to find a region close to where we were exploring
        // This maintains continuity instead of jumping to a new area
        if let Some(prev) = prev_centroid {
            let closest_idx = self
                .plan
                .regions
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    let dist_a = a.centroid.distance(prev);
                    let dist_b = b.centroid.distance(prev);
                    dist_a
                        .partial_cmp(&dist_b)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(i, _)| i)
                .unwrap_or(0);

            // Only use the closest region if it's reasonably close (within 2m)
            if let Some(region) = self.plan.regions.get(closest_idx)
                && region.centroid.distance(prev) < 2.0
            {
                self.plan.current_region_idx = closest_idx;
                return;
            }
        }

        // Default: start from the first (best) region
        self.plan.current_region_idx = 0;
    }

    /// Order regions using greedy TSP with priority weighting.
    fn order_regions(
        &self,
        regions: &mut Vec<ExplorationRegion>,
        robot_pos: Point2D,
        history: &ExplorationHistory,
    ) -> Vec<ExplorationRegion> {
        if regions.is_empty() {
            return Vec::new();
        }

        // Compute priorities for all regions
        for region in regions.iter_mut() {
            region.priority = self.compute_region_priority(region, robot_pos, history);
        }

        // Greedy TSP: always pick the best next region
        let mut ordered = Vec::with_capacity(regions.len());
        let mut remaining: Vec<ExplorationRegion> = std::mem::take(regions);
        let mut current_pos = robot_pos;

        while !remaining.is_empty() {
            // Find best next region
            let best_idx = remaining
                .iter()
                .enumerate()
                .map(|(i, region)| {
                    let distance = current_pos.distance(region.centroid);
                    let score = region.priority * self.config.priority_weight
                        - distance * self.config.distance_weight
                        - region.visit_count as f32 * self.config.revisit_penalty
                        + region.frontier_count() as f32 * self.config.frontier_bonus;
                    (i, score)
                })
                .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i)
                .unwrap_or(0);

            let selected = remaining.remove(best_idx);
            current_pos = selected.centroid;
            ordered.push(selected);
        }

        ordered
    }

    /// Compute the priority of a region.
    fn compute_region_priority(
        &self,
        region: &ExplorationRegion,
        _robot_pos: Point2D,
        history: &ExplorationHistory,
    ) -> f32 {
        // Base priority from estimated area (larger regions are more valuable)
        let area_score = (region.estimated_area / 10.0).min(1.0);

        // Penalty for regions near recently visited areas
        let visit_penalty = if history.is_area_recently_visited(region.centroid) {
            0.5
        } else {
            0.0
        };

        // Bonus for regions with more frontiers
        let frontier_score = (region.frontier_count() as f32 / 5.0).min(1.0);

        area_score * 0.3 + frontier_score * 0.7 - visit_penalty
    }

    /// Update the current region with valid frontiers.
    fn update_current_region(&mut self, frontiers: &[Frontier]) {
        if let Some(region) = self.plan.current_region_mut() {
            region.retain_valid_frontiers(frontiers);
        }
    }

    /// Select the best frontier within the current region.
    ///
    /// Uses heading-aligned selection to prevent ping-pong behavior.
    pub fn select_frontier_in_region<'a>(
        &self,
        region: &'a ExplorationRegion,
        robot_pos: Point2D,
        robot_heading: f32,
    ) -> Option<&'a Frontier> {
        if region.is_empty() {
            return None;
        }

        // Score each frontier

        region
            .frontiers
            .iter()
            .map(|frontier| {
                let score = self.compute_frontier_score(frontier, robot_pos, robot_heading);
                (frontier, score)
            })
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(f, _)| f)
    }

    /// Compute the score of a frontier for selection.
    fn compute_frontier_score(
        &self,
        frontier: &Frontier,
        robot_pos: Point2D,
        robot_heading: f32,
    ) -> f32 {
        // Use viewpoint for distance calculations (safe navigation target)
        let direction = Point2D::new(
            frontier.viewpoint.x - robot_pos.x,
            frontier.viewpoint.y - robot_pos.y,
        );
        let distance = direction.length();

        if distance < 0.001 {
            return 0.0;
        }

        // Heading alignment score (1.0 = same direction, 0.0 = opposite)
        let angle_to_frontier = direction.y.atan2(direction.x);
        let angle_diff = normalize_angle(angle_to_frontier - robot_heading);
        let heading_score = 1.0 - (angle_diff.abs() / PI);

        // Distance score (prefer closer frontiers within the region)
        let distance_score = 1.0 / (1.0 + distance * 0.5);

        // Combined score
        heading_score * self.config.heading_weight
            + distance_score * self.config.local_distance_weight
            + self.config.openness_weight * 0.5 // Placeholder for openness (not currently available on Frontier)
    }

    /// Reset the planner state.
    pub fn reset(&mut self) {
        self.plan = ExplorationPlan::new();
        self.region_detector.reset_ids();
    }
}

/// Normalize an angle to [-PI, PI].
fn normalize_angle(angle: f32) -> f32 {
    let mut result = angle;
    while result > PI {
        result -= 2.0 * PI;
    }
    while result < -PI {
        result += 2.0 * PI;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_frontier(x: f32, y: f32) -> Frontier {
        Frontier {
            viewpoint: Point2D::new(x, y),
            look_direction: Point2D::new(1.0, 0.0),
            endpoint: Point2D::new(x - 0.4, y),
            line_idx: 0,
            estimated_area: 4.0,
        }
    }

    #[test]
    fn test_empty_plan() {
        let config = PlannerConfig::default();
        let planner = ExplorationPlanner::new(config);

        assert!(planner.plan().is_empty());
        assert!(planner.plan().is_complete());
    }

    #[test]
    fn test_replan_with_frontiers() {
        let config = PlannerConfig::default();
        let mut planner = ExplorationPlanner::new(config);
        let history = ExplorationHistory::new(0.5, 100);

        let frontiers = vec![
            make_frontier(1.0, 0.0),
            make_frontier(1.5, 0.0),
            make_frontier(5.0, 5.0),
            make_frontier(5.5, 5.0),
        ];

        planner.force_replan(&frontiers, Point2D::new(0.0, 0.0), &history);

        assert!(!planner.plan().is_empty());
        assert!(!planner.plan().is_complete());
    }

    #[test]
    fn test_frontier_selection_prefers_heading() {
        let config = PlannerConfig::default().with_heading_weight(0.9);
        let planner = ExplorationPlanner::new(config);

        // Robot facing right (heading = 0)
        let robot_pos = Point2D::new(0.0, 0.0);
        let robot_heading = 0.0;

        // Two frontiers: one ahead (x=2), one behind (x=-2)
        let frontiers = vec![make_frontier(2.0, 0.0), make_frontier(-2.0, 0.0)];

        let region = ExplorationRegion::from_frontiers(0, frontiers);

        let selected = planner.select_frontier_in_region(&region, robot_pos, robot_heading);

        assert!(selected.is_some());
        let selected = selected.unwrap();
        // Should prefer the frontier ahead (positive x)
        assert!(selected.viewpoint.x > 0.0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(PI) - PI).abs() < 0.001);
        assert!((normalize_angle(-PI) - (-PI)).abs() < 0.001);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 0.001);
        assert!((normalize_angle(-3.0 * PI) - (-PI)).abs() < 0.001);
    }

    #[test]
    fn test_plan_advance_region() {
        let mut plan = ExplorationPlan::new();
        let frontiers = vec![make_frontier(1.0, 0.0)];
        plan.regions
            .push(ExplorationRegion::from_frontiers(0, frontiers.clone()));
        plan.regions
            .push(ExplorationRegion::from_frontiers(1, frontiers));
        plan.stale = false;

        assert_eq!(plan.current_region_idx, 0);
        assert!(!plan.is_complete());

        plan.advance_region();
        assert_eq!(plan.current_region_idx, 1);
        assert!(!plan.is_complete());

        plan.advance_region();
        assert_eq!(plan.current_region_idx, 2);
        assert!(plan.is_complete());
    }
}
