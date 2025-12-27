//! Exploration controller state machine.
//!
//! Manages frontier-based exploration with collision recovery.
//! The primary goal is complete map coverage, with obstacle avoidance
//! as the secondary priority.
//!
//! # Region-Based Exploration
//!
//! This controller uses a hierarchical exploration strategy:
//! 1. Cluster frontiers into exploration regions
//! 2. Order regions using greedy TSP to minimize travel
//! 3. Select frontiers within each region using heading-aligned selection
//!
//! This approach eliminates ping-pong behavior and improves coverage efficiency.

use std::collections::{HashMap, VecDeque};

use crate::core::{Point2D, Pose2D};
use crate::{Frontier, Map, Path};

use super::collision::{CollisionEvent, VirtualWall};
use super::config::ExplorationConfig;
use super::history::ExplorationHistory;
use super::path_follower::{PathFollower, VelocityCommand};
use super::planner::ExplorationPlanner;

/// Current exploration state.
#[derive(Clone, Debug)]
pub enum ExplorationState {
    /// Idle, waiting for start.
    Idle,

    /// Selecting next frontier to explore.
    SelectingFrontier,

    /// Following path to target frontier.
    FollowingPath {
        /// Current path being followed.
        path: Path,
        /// Current waypoint index.
        waypoint_idx: usize,
        /// Target frontier point.
        target_frontier: Point2D,
        /// CBVG goal node index used for this path (for collision tracking).
        goal_node_idx: Option<usize>,
    },

    /// Recovering from collision (backing off).
    RecoveringFromCollision {
        /// Distance remaining to back off (meters).
        backoff_remaining: f32,
        /// Heading to back off towards (radians).
        backoff_heading: f32,
        /// Position when backoff started.
        start_position: Point2D,
        /// The frontier we were trying to reach (for alternative path planning).
        target_frontier: Point2D,
        /// The CBVG goal node that led to collision (to exclude in retry).
        failed_goal_node: Option<usize>,
        /// Number of recovery collision attempts.
        recovery_attempts: u32,
    },

    /// Exploration complete - no frontiers remain.
    Complete,

    /// Exploration failed (stuck, unreachable frontiers).
    Failed {
        /// Reason for failure.
        reason: String,
    },
}

impl ExplorationState {
    /// Get a short description of the state.
    pub fn name(&self) -> &'static str {
        match self {
            ExplorationState::Idle => "Idle",
            ExplorationState::SelectingFrontier => "SelectingFrontier",
            ExplorationState::FollowingPath { .. } => "FollowingPath",
            ExplorationState::RecoveringFromCollision { .. } => "RecoveringFromCollision",
            ExplorationState::Complete => "Complete",
            ExplorationState::Failed { .. } => "Failed",
        }
    }
}

/// Result of a single exploration step.
#[derive(Clone, Debug)]
pub struct ExplorationStep {
    /// Velocity command to execute (None if no action needed).
    pub velocity: Option<VelocityCommand>,

    /// Current state after this step.
    pub state: ExplorationState,

    /// Virtual wall to add to map (if collision occurred).
    pub virtual_wall: Option<VirtualWall>,

    /// Whether a new path was planned this step.
    pub path_planned: bool,

    /// Current target frontier (if any).
    pub target_frontier: Option<Point2D>,

    /// Distance to current target.
    pub distance_to_target: f32,
}

impl ExplorationStep {
    /// Create a step with no action.
    fn idle(state: ExplorationState) -> Self {
        Self {
            velocity: None,
            state,
            virtual_wall: None,
            path_planned: false,
            target_frontier: None,
            distance_to_target: 0.0,
        }
    }

    /// Create a step with a velocity command.
    fn with_velocity(velocity: VelocityCommand, state: ExplorationState) -> Self {
        Self {
            velocity: Some(velocity),
            state,
            virtual_wall: None,
            path_planned: false,
            target_frontier: None,
            distance_to_target: 0.0,
        }
    }
}

/// Exploration controller.
///
/// Manages autonomous exploration using frontier-based strategy:
///
/// 1. Detect frontiers (unexplored areas)
/// 2. Select the best frontier based on distance and openness
/// 3. Plan a path to the frontier
/// 4. Follow the path while avoiding obstacles
/// 5. Handle collisions by adding virtual walls and replanning
/// 6. Repeat until no frontiers remain
///
/// # Primary Goals
///
/// 1. **Complete map coverage** - Explore every reachable area
/// 2. **Avoid obstacles** - Navigate safely using path planning with robot radius clearance
///
/// # Usage
///
/// ```rust,ignore
/// use vastu_map::exploration::{ExplorationController, ExplorationConfig, CollisionEvent};
/// use vastu_map::{VectorMap, Pose2D};
///
/// let config = ExplorationConfig::default();
/// let mut controller = ExplorationController::new(config);
/// controller.start();
///
/// loop {
///     let map = get_slam_map();
///     let pose = get_current_pose();
///     let collision = check_sensors();
///
///     let step = controller.update(&map, pose, collision);
///
///     if let Some(wall) = step.virtual_wall {
///         add_wall_to_map(wall);
///     }
///
///     if let Some(vel) = step.velocity {
///         send_velocity_command(vel);
///     }
///
///     if controller.is_complete() {
///         break;
///     }
/// }
/// ```
pub struct ExplorationController {
    /// Current exploration state.
    state: ExplorationState,

    /// Path follower for smooth motion.
    path_follower: PathFollower,

    /// Configuration.
    config: ExplorationConfig,

    /// Number of consecutive failed path planning attempts for current frontier.
    failed_planning_attempts: usize,

    /// Maximum failed attempts before giving up on a frontier.
    max_failed_attempts: usize,

    /// The frontier viewpoint we're currently attempting (to track per-frontier failures).
    current_attempt_frontier: Option<Point2D>,

    /// Frontiers that have been marked as unreachable.
    unreachable_frontiers: Vec<Point2D>,

    /// Distance threshold to consider frontiers as same location.
    frontier_match_distance: f32,

    /// Region-based exploration planner.
    planner: ExplorationPlanner,

    /// Exploration history for visit tracking.
    history: ExplorationHistory,

    /// Time accumulator for dt calculation (seconds).
    last_update_time: Option<std::time::Instant>,

    /// Robot position when unreachable frontiers were last cleared.
    /// Used to re-evaluate unreachable frontiers after robot moves significantly.
    position_at_last_unreachable_clear: Option<Point2D>,

    /// CBVG nodes that led to collision for each frontier viewpoint.
    /// Key is grid-quantized frontier position, value is list of failed node indices.
    /// Used to find alternative approach angles after collision.
    collision_nodes: HashMap<(i32, i32), Vec<usize>>,

    /// Recent frontier targets for oscillation detection.
    /// Stores (grid_x, grid_y) of last N selected frontiers.
    recent_targets: VecDeque<(i32, i32)>,

    /// Frontiers temporarily suppressed due to oscillation detection.
    /// Key is grid position, value is suppression count (decrements each cycle).
    suppressed_frontiers: HashMap<(i32, i32), u32>,
}

impl ExplorationController {
    /// Create a new exploration controller.
    pub fn new(config: ExplorationConfig) -> Self {
        let path_follower = PathFollower::new(&config);
        let planner = ExplorationPlanner::new(config.planner_config.clone());
        let history = ExplorationHistory::new(config.history_cell_size, config.history_max_recent);

        Self {
            state: ExplorationState::Idle,
            path_follower,
            config,
            failed_planning_attempts: 0,
            max_failed_attempts: 3,
            current_attempt_frontier: None,
            unreachable_frontiers: Vec::new(),
            frontier_match_distance: 0.5,
            planner,
            history,
            last_update_time: None,
            position_at_last_unreachable_clear: None,
            collision_nodes: HashMap::new(),
            recent_targets: VecDeque::with_capacity(8),
            suppressed_frontiers: HashMap::new(),
        }
    }

    /// Start exploration.
    pub fn start(&mut self) {
        self.state = ExplorationState::SelectingFrontier;
        self.failed_planning_attempts = 0;
        self.unreachable_frontiers.clear();
        self.collision_nodes.clear();
        self.planner.reset();
        self.history.clear();
        self.last_update_time = Some(std::time::Instant::now());
        self.position_at_last_unreachable_clear = None;
        self.recent_targets.clear();
        self.suppressed_frontiers.clear();
    }

    /// Reset to idle state.
    pub fn reset(&mut self) {
        self.state = ExplorationState::Idle;
        self.failed_planning_attempts = 0;
        self.unreachable_frontiers.clear();
        self.collision_nodes.clear();
        self.planner.reset();
        self.history.clear();
        self.last_update_time = None;
        self.position_at_last_unreachable_clear = None;
        self.recent_targets.clear();
        self.suppressed_frontiers.clear();
    }

    /// Get current state.
    pub fn state(&self) -> &ExplorationState {
        &self.state
    }

    /// Check if exploration is complete.
    pub fn is_complete(&self) -> bool {
        matches!(self.state, ExplorationState::Complete)
    }

    /// Check if exploration failed.
    pub fn is_failed(&self) -> bool {
        matches!(self.state, ExplorationState::Failed { .. })
    }

    /// Check if exploration is active (not idle, complete, or failed).
    pub fn is_active(&self) -> bool {
        !matches!(
            self.state,
            ExplorationState::Idle | ExplorationState::Complete | ExplorationState::Failed { .. }
        )
    }

    /// Main update function - call every control loop iteration.
    ///
    /// # Arguments
    /// * `map` - Current SLAM map (implements Map trait)
    /// * `current_pose` - Current robot pose estimate
    /// * `collision` - Collision event if any
    ///
    /// # Returns
    /// Exploration step with velocity command and state updates.
    pub fn update<M: Map>(
        &mut self,
        map: &M,
        current_pose: Pose2D,
        collision: Option<CollisionEvent>,
    ) -> ExplorationStep {
        // Handle collision first (highest priority for safety)
        if let Some(event) = collision {
            return self.handle_collision(event, current_pose);
        }

        // State machine
        match self.state.clone() {
            ExplorationState::Idle => ExplorationStep::idle(self.state.clone()),

            ExplorationState::SelectingFrontier => self.select_frontier(map, current_pose),

            ExplorationState::FollowingPath {
                path,
                waypoint_idx,
                target_frontier,
                goal_node_idx,
            } => self.follow_path(
                map,
                current_pose,
                path,
                waypoint_idx,
                target_frontier,
                goal_node_idx,
            ),

            ExplorationState::RecoveringFromCollision {
                backoff_remaining,
                backoff_heading,
                start_position,
                target_frontier,
                failed_goal_node,
                recovery_attempts,
            } => self.execute_backoff(
                current_pose,
                backoff_remaining,
                backoff_heading,
                start_position,
                target_frontier,
                failed_goal_node,
                recovery_attempts,
            ),

            ExplorationState::Complete => ExplorationStep::idle(self.state.clone()),

            ExplorationState::Failed { .. } => ExplorationStep::idle(self.state.clone()),
        }
    }

    /// Handle a collision event.
    ///
    /// When collision occurs during path following:
    /// 1. Record the failed CBVG goal node (if any)
    /// 2. Enter recovery state with the target frontier preserved
    /// 3. After backoff completes, path planning will try alternative nodes
    ///
    /// When collision occurs during recovery:
    /// 1. Increment recovery attempts counter
    /// 2. If too many attempts, mark frontier as unreachable
    fn handle_collision(&mut self, event: CollisionEvent, current_pose: Pose2D) -> ExplorationStep {
        log::info!(
            "Collision detected: {:?} at ({:.2}, {:.2})",
            event.collision_type,
            event.point.x,
            event.point.y
        );

        // Create virtual wall at collision point
        let virtual_wall = event.to_virtual_wall(self.config.virtual_wall_length);

        // Handle collision during recovery - increment attempts and possibly give up
        if let ExplorationState::RecoveringFromCollision {
            target_frontier,
            failed_goal_node,
            recovery_attempts,
            backoff_heading,
            ..
        } = &self.state
        {
            let attempts = *recovery_attempts + 1;
            let target = *target_frontier;
            let failed_node = *failed_goal_node;

            log::warn!(
                "Recovery collision at ({:.2}, {:.2}) - attempt {} of 3",
                event.point.x,
                event.point.y,
                attempts
            );

            if attempts >= 3 {
                // Too many recovery collisions - mark frontier as unreachable
                log::warn!(
                    "Marking frontier at ({:.2}, {:.2}) as unreachable after {} recovery collisions",
                    target.x,
                    target.y,
                    attempts
                );

                // Track position for unreachable frontier reset logic
                if self.unreachable_frontiers.is_empty() {
                    self.position_at_last_unreachable_clear = Some(current_pose.position());
                }
                self.unreachable_frontiers.push(target);

                // Go back to selecting a different frontier
                self.state = ExplorationState::SelectingFrontier;
                let mut step = ExplorationStep::idle(self.state.clone());
                step.virtual_wall = Some(virtual_wall);
                return step;
            }

            // Continue recovery with incremented counter
            // Keep original start_position so backoff distance calculation is correct
            let original_start =
                if let ExplorationState::RecoveringFromCollision { start_position, .. } =
                    &self.state
                {
                    *start_position
                } else {
                    current_pose.position()
                };

            self.state = ExplorationState::RecoveringFromCollision {
                backoff_remaining: self.config.backoff_distance,
                backoff_heading: *backoff_heading,
                start_position: original_start, // Keep original start for correct distance calculation
                target_frontier: target,
                failed_goal_node: failed_node,
                recovery_attempts: attempts,
            };

            let mut step =
                ExplorationStep::with_velocity(VelocityCommand::stop(), self.state.clone());
            step.virtual_wall = Some(virtual_wall);
            return step;
        }

        // Collision during FollowingPath - extract values then record the failed node
        let (target_frontier, failed_goal_node) = if let ExplorationState::FollowingPath {
            target_frontier,
            goal_node_idx,
            ..
        } = &self.state
        {
            (*target_frontier, *goal_node_idx)
        } else {
            // Collision in some other state (shouldn't happen, but handle gracefully)
            (current_pose.position(), None)
        };

        // Record this node as failed for this frontier (after extracting values to avoid borrow conflict)
        if let Some(node_idx) = failed_goal_node {
            self.add_collision_node_for_frontier(target_frontier, node_idx);
        }

        log::info!(
            "Collision while heading to frontier ({:.2}, {:.2}), failed node: {:?}",
            target_frontier.x,
            target_frontier.y,
            failed_goal_node
        );

        // Transition to recovery state - preserve target frontier for alternative path planning
        self.state = ExplorationState::RecoveringFromCollision {
            backoff_remaining: self.config.backoff_distance,
            backoff_heading: event.backoff_heading(),
            start_position: current_pose.position(),
            target_frontier,
            failed_goal_node,
            recovery_attempts: 0,
        };

        // Return stop command with virtual wall
        let mut step = ExplorationStep::with_velocity(VelocityCommand::stop(), self.state.clone());
        step.virtual_wall = Some(virtual_wall);
        step
    }

    /// Select the next frontier to explore using region-based strategy.
    ///
    /// The algorithm:
    /// 1. Update exploration history with current position
    /// 2. Update the region-based planner with current frontiers
    /// 3. Select the best frontier from the current region
    /// 4. Plan a path to the selected frontier
    /// 5. If path planning fails, try next frontier or advance to next region
    fn select_frontier<M: Map>(&mut self, map: &M, current_pose: Pose2D) -> ExplorationStep {
        let robot_pos = current_pose.position();

        // Calculate dt for planner update
        let dt = self
            .last_update_time
            .map(|t| t.elapsed().as_secs_f32())
            .unwrap_or(0.1);
        self.last_update_time = Some(std::time::Instant::now());

        // Decay suppression counters from oscillation detection
        self.decay_suppressed_frontiers();

        // Update history with current position
        self.history.mark_visited(robot_pos);

        // Get all frontiers
        let all_frontiers = map.frontiers();

        // Re-evaluate unreachable frontiers when robot has moved significantly.
        // This allows frontiers that were previously unreachable from position A
        // to be reconsidered when the robot has moved to position B.
        // Use 1.5m for more aggressive retry - robot tries new positions sooner.
        const UNREACHABLE_RESET_DISTANCE: f32 = 1.5; // meters
        if !self.unreachable_frontiers.is_empty() {
            let should_clear = match self.position_at_last_unreachable_clear {
                Some(last_pos) => robot_pos.distance(last_pos) > UNREACHABLE_RESET_DISTANCE,
                None => false, // First time through, don't clear
            };

            if should_clear {
                log::debug!(
                    "Clearing {} unreachable frontiers and {} collision node entries after robot moved {:.1}m",
                    self.unreachable_frontiers.len(),
                    self.collision_nodes.len(),
                    UNREACHABLE_RESET_DISTANCE
                );
                self.unreachable_frontiers.clear();
                self.collision_nodes.clear(); // Also reset collision history
                self.position_at_last_unreachable_clear = Some(robot_pos);
            }
        }

        // Filter out unreachable and suppressed frontiers (using viewpoint for distance checks)
        let reachable_frontiers: Vec<Frontier> = all_frontiers
            .iter()
            .filter(|f| !self.is_frontier_unreachable(f.viewpoint))
            .filter(|f| !self.is_frontier_suppressed(f.viewpoint))
            .cloned()
            .collect();

        if reachable_frontiers.is_empty() {
            if all_frontiers.is_empty() {
                // No frontiers at all - exploration complete!
                log::info!("Exploration complete: no frontiers remain");
                self.state = ExplorationState::Complete;
                return ExplorationStep::idle(self.state.clone());
            } else {
                // All frontiers are unreachable
                log::warn!(
                    "Exploration stuck: {} frontiers exist but all are unreachable",
                    all_frontiers.len()
                );
                self.state = ExplorationState::Failed {
                    reason: format!("{} frontiers unreachable", all_frontiers.len()),
                };
                return ExplorationStep::idle(self.state.clone());
            }
        }

        // Update the region-based planner
        self.planner
            .update(&reachable_frontiers, robot_pos, &self.history, dt);

        // Try to find a frontier with a valid path
        // Safety limit to prevent infinite loops - max iterations = 2 * number of frontiers + regions
        const MAX_LOOP_ITERATIONS: usize = 100;
        let mut loop_iterations = 0;

        loop {
            loop_iterations += 1;
            if loop_iterations > MAX_LOOP_ITERATIONS {
                log::error!(
                    "Exploration stuck: exceeded {} iterations in select_frontier loop",
                    MAX_LOOP_ITERATIONS
                );
                self.state = ExplorationState::Failed {
                    reason: format!(
                        "frontier selection loop exceeded {} iterations",
                        MAX_LOOP_ITERATIONS
                    ),
                };
                return ExplorationStep::idle(self.state.clone());
            }

            // Check if we've exhausted all regions
            if self.planner.plan().is_complete() {
                // Re-filter frontiers to exclude those marked unreachable during this loop
                let still_reachable: Vec<Frontier> = reachable_frontiers
                    .iter()
                    .filter(|f| !self.is_frontier_unreachable(f.viewpoint))
                    .cloned()
                    .collect();

                if still_reachable.is_empty() {
                    // All frontiers are now unreachable
                    log::warn!(
                        "Exploration stuck: all {} frontiers are unreachable",
                        reachable_frontiers.len()
                    );
                    self.state = ExplorationState::Failed {
                        reason: format!("{} frontiers unreachable", reachable_frontiers.len()),
                    };
                    return ExplorationStep::idle(self.state.clone());
                }

                // Force replan with updated reachable frontiers
                self.planner
                    .force_replan(&still_reachable, robot_pos, &self.history);

                if self.planner.plan().is_complete() || self.planner.plan().is_empty() {
                    // Still no regions - exploration complete
                    log::info!("Exploration complete: all regions exhausted");
                    self.state = ExplorationState::Complete;
                    return ExplorationStep::idle(self.state.clone());
                }
            }

            // Get current region
            let region = match self.planner.plan().current_region() {
                Some(r) => r,
                None => {
                    self.state = ExplorationState::Complete;
                    return ExplorationStep::idle(self.state.clone());
                }
            };

            // Select best frontier in current region (heading-aligned)
            let selected_frontier =
                self.planner
                    .select_frontier_in_region(region, robot_pos, current_pose.theta);

            // Find nearest CBVG node to the frontier - guaranteed reachable
            // CBVG nodes are in visible areas with proper wall clearance
            let (target, frontier_viewpoint) = match selected_frontier {
                Some(f) => {
                    // Try to find nearest CBVG node (within 2m of frontier viewpoint)
                    let target_pos = match map.nearest_cbvg_node(f.viewpoint, 2.0) {
                        Some(node_pos) => node_pos,
                        None => f.viewpoint, // Fallback to viewpoint if no node nearby
                    };
                    (target_pos, f.viewpoint)
                }
                None => {
                    // Current region exhausted, advance to next
                    log::debug!("Region {} exhausted, advancing to next", region.id);
                    self.planner.plan_mut().advance_region();
                    continue;
                }
            };

            // Check if we're already close enough to this viewpoint
            let distance_to_target = robot_pos.distance(target);
            if distance_to_target <= self.config.waypoint_tolerance {
                // Already at this viewpoint - skip to next
                log::debug!(
                    "Skipping frontier viewpoint at ({:.2}, {:.2}) - already there ({:.2}m <= {:.2}m)",
                    target.x,
                    target.y,
                    distance_to_target,
                    self.config.waypoint_tolerance
                );
                // Mark as visited so we don't keep selecting it
                self.history.mark_visited(target);

                // Remove this frontier from the current region so it won't be selected again
                if let Some(region) = self.planner.plan_mut().current_region_mut() {
                    region
                        .frontiers
                        .retain(|f| f.viewpoint.distance(frontier_viewpoint) > 0.1);
                }

                // If region is now empty, advance to next region
                if self
                    .planner
                    .plan()
                    .current_region()
                    .is_none_or(|r| r.is_empty())
                {
                    self.planner.plan_mut().advance_region();
                }
                continue;
            }

            // Get excluded nodes for this frontier (nodes that led to collision before)
            let excluded_nodes = self.get_collision_nodes_for_frontier(frontier_viewpoint);

            log::debug!(
                "Selected frontier viewpoint at ({:.2}, {:.2}) from region {}, distance: {:.2}m, excluded nodes: {:?}",
                target.x,
                target.y,
                region.id,
                distance_to_target,
                excluded_nodes
            );

            // Plan path directly to viewpoint, excluding nodes that led to collision
            match map.get_path_excluding_nodes(robot_pos, target, &excluded_nodes) {
                Some((path, goal_node_idx)) => {
                    log::debug!(
                        "Path planned: {} waypoints, {:.2}m total, via node {}",
                        path.points.len(),
                        path.length,
                        goal_node_idx
                    );

                    // Check for oscillation before committing to this frontier
                    // If oscillation is detected, suppress the "other" frontier and continue with this one
                    if let Some(suppress_key) =
                        self.record_frontier_and_check_oscillation(frontier_viewpoint)
                    {
                        log::info!(
                            "Oscillation detected - suppressing other frontier at grid {:?}, continuing with ({:.2}, {:.2})",
                            suppress_key,
                            frontier_viewpoint.x,
                            frontier_viewpoint.y
                        );
                        // Suppress the other frontier for 10 cycles
                        self.suppressed_frontiers.insert(suppress_key, 10);

                        // Also remove the suppressed frontier from planner's current region
                        // so it won't be selected again in subsequent loop iterations
                        if let Some(region) = self.planner.plan_mut().current_region_mut() {
                            let grid_cell_size = 0.5; // Same as frontier_grid_key
                            region.frontiers.retain(|f| {
                                let key = (
                                    (f.viewpoint.x / grid_cell_size) as i32,
                                    (f.viewpoint.y / grid_cell_size) as i32,
                                );
                                key != suppress_key
                            });
                        }
                        // Continue with the current frontier (don't skip it)
                    }

                    self.failed_planning_attempts = 0;
                    self.current_attempt_frontier = None;

                    // Immediately start following the path
                    let result = self.path_follower.follow(
                        current_pose,
                        &path,
                        0,
                        self.config.waypoint_tolerance,
                    );

                    self.state = ExplorationState::FollowingPath {
                        path: path.clone(),
                        waypoint_idx: result.waypoint_idx,
                        // Use frontier_viewpoint (not target/CBVG node) so collision tracking matches
                        target_frontier: frontier_viewpoint,
                        goal_node_idx: Some(goal_node_idx), // Track which node we're using
                    };

                    let mut step =
                        ExplorationStep::with_velocity(result.velocity, self.state.clone());
                    step.path_planned = true;
                    step.target_frontier = Some(target);
                    step.distance_to_target = result.distance_to_waypoint;
                    return step;
                }
                None => {
                    log::warn!(
                        "Failed to plan path to frontier at ({:.2}, {:.2})",
                        target.x,
                        target.y
                    );

                    // Track per-frontier failures - reset counter if we switched frontiers
                    let is_same_frontier = self
                        .current_attempt_frontier
                        .map(|prev| prev.distance(frontier_viewpoint) < 0.1)
                        .unwrap_or(false);

                    if is_same_frontier {
                        self.failed_planning_attempts += 1;
                    } else {
                        // New frontier - reset counter
                        self.current_attempt_frontier = Some(frontier_viewpoint);
                        self.failed_planning_attempts = 1;
                    }

                    if self.failed_planning_attempts >= self.max_failed_attempts {
                        // Track position when we first mark any frontier as unreachable
                        // (so we can re-evaluate after robot moves 3m)
                        if self.unreachable_frontiers.is_empty() {
                            self.position_at_last_unreachable_clear = Some(robot_pos);
                        }

                        // Mark this frontier as unreachable (use viewpoint, not CBVG node)
                        self.unreachable_frontiers.push(frontier_viewpoint);
                        self.failed_planning_attempts = 0;
                        self.current_attempt_frontier = None;
                        log::warn!(
                            "Frontier marked as unreachable after {} failed attempts",
                            self.max_failed_attempts
                        );

                        // Remove from current region and try again (use viewpoint for matching)
                        if let Some(region) = self.planner.plan_mut().current_region_mut() {
                            region
                                .frontiers
                                .retain(|f| f.viewpoint.distance(frontier_viewpoint) > 0.1);
                        }
                        continue;
                    }

                    // Try another frontier in same region
                    continue;
                }
            }
        }
    }

    /// Follow the current path.
    fn follow_path<M: Map>(
        &mut self,
        _map: &M,
        current_pose: Pose2D,
        path: Path,
        waypoint_idx: usize,
        target_frontier: Point2D,
        goal_node_idx: Option<usize>,
    ) -> ExplorationStep {
        let result = self.path_follower.follow(
            current_pose,
            &path,
            waypoint_idx,
            self.config.waypoint_tolerance,
        );

        if result.path_complete {
            // Path complete - go back to selecting frontier
            log::debug!(
                "Reached frontier at ({:.2}, {:.2})",
                target_frontier.x,
                target_frontier.y
            );
            // Clear collision nodes for this frontier since we reached it successfully
            self.clear_collision_nodes_for_frontier(target_frontier);
            self.state = ExplorationState::SelectingFrontier;

            return ExplorationStep::idle(self.state.clone());
        }

        // Note: We don't check frontier existence every step for performance.
        // If the frontier closes while following, we'll select a new one when
        // the path completes or we get close enough.

        // Update state with new waypoint index
        self.state = ExplorationState::FollowingPath {
            path,
            waypoint_idx: result.waypoint_idx,
            target_frontier,
            goal_node_idx,
        };

        let mut step = ExplorationStep::with_velocity(result.velocity, self.state.clone());
        step.target_frontier = Some(target_frontier);
        step.distance_to_target = result.distance_to_waypoint;
        step
    }

    /// Execute backoff motion after collision.
    fn execute_backoff(
        &mut self,
        current_pose: Pose2D,
        backoff_remaining: f32,
        backoff_heading: f32,
        start_position: Point2D,
        target_frontier: Point2D,
        failed_goal_node: Option<usize>,
        recovery_attempts: u32,
    ) -> ExplorationStep {
        // Calculate how far we've backed up
        let backed_up = current_pose.position().distance(start_position);
        let remaining = backoff_remaining - backed_up;

        if remaining <= 0.0 {
            // Backoff complete - go back to selecting frontier
            // The collision node has already been recorded in handle_collision
            log::debug!(
                "Backoff complete, selecting new frontier (failed node: {:?}, attempts: {})",
                failed_goal_node,
                recovery_attempts
            );
            self.state = ExplorationState::SelectingFrontier;
            return ExplorationStep::idle(self.state.clone());
        }

        // Update remaining distance
        self.state = ExplorationState::RecoveringFromCollision {
            backoff_remaining: remaining,
            backoff_heading,
            start_position,
            target_frontier,
            failed_goal_node,
            recovery_attempts,
        };

        // Command slow reverse motion
        let backup_speed = 0.1; // m/s
        ExplorationStep::with_velocity(VelocityCommand::backup(backup_speed), self.state.clone())
    }

    /// Check if a frontier point is marked as unreachable.
    fn is_frontier_unreachable(&self, point: Point2D) -> bool {
        self.unreachable_frontiers
            .iter()
            .any(|p| p.distance(point) < self.frontier_match_distance)
    }

    /// Get the number of unreachable frontiers.
    pub fn unreachable_frontier_count(&self) -> usize {
        self.unreachable_frontiers.len()
    }

    /// Clear unreachable frontiers (useful after map updates).
    pub fn clear_unreachable_frontiers(&mut self) {
        self.unreachable_frontiers.clear();
        self.position_at_last_unreachable_clear = None;
    }

    /// Get the current configuration.
    pub fn config(&self) -> &ExplorationConfig {
        &self.config
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Oscillation Detection (prevents ping-pong between nearby frontiers)
    // ─────────────────────────────────────────────────────────────────────────

    /// Record a frontier selection and check for oscillation or repeated visits.
    /// Returns the frontier to suppress if a problematic pattern is detected.
    fn record_frontier_and_check_oscillation(&mut self, frontier: Point2D) -> Option<(i32, i32)> {
        let key = self.frontier_grid_key(frontier);

        // Add to recent targets (keep last 8)
        self.recent_targets.push_back(key);
        if self.recent_targets.len() > 8 {
            self.recent_targets.pop_front();
        }

        let targets: Vec<_> = self.recent_targets.iter().collect();
        let len = targets.len();

        // Check 1: A-B-A-B pattern (oscillation between two frontiers)
        if len >= 4 {
            let a = targets[len - 4];
            let b = targets[len - 3];
            let c = targets[len - 2];
            let d = targets[len - 1]; // This is the current selection (key)

            // Pattern: A, B, A, B (same A and B repeating)
            if a == c && b == d && a != b {
                log::warn!(
                    "Oscillation detected between frontiers at grid {:?} and {:?}",
                    a,
                    b
                );

                // Copy the key to suppress before clearing
                let suppress_key = *c;
                self.recent_targets.clear();
                return Some(suppress_key);
            }
        }

        // Check 2: Same frontier visited 3+ times in last 6 selections (stuck on one frontier)
        if len >= 6 {
            let current = targets[len - 1];
            let count = targets[len - 6..].iter().filter(|&&t| t == current).count();

            if count >= 3 {
                log::warn!(
                    "Repeated visits detected: frontier at grid {:?} visited {} times in last 6",
                    current,
                    count
                );

                // Suppress this frontier since we keep returning to it
                let suppress_key = *current;
                self.recent_targets.clear();
                return Some(suppress_key);
            }
        }

        // Check 3: Three frontiers in rotation (A-B-C-A-B-C pattern)
        if len >= 6 {
            let a = targets[len - 6];
            let b = targets[len - 5];
            let c = targets[len - 4];
            let d = targets[len - 3];
            let e = targets[len - 2];
            let f = targets[len - 1];

            // Pattern: A, B, C, A, B, C
            if a == d && b == e && c == f && a != b && b != c && a != c {
                log::warn!("Three-way rotation detected: {:?}, {:?}, {:?}", a, b, c);

                // Suppress the oldest frontier in the pattern
                let suppress_key = *a;
                self.recent_targets.clear();
                return Some(suppress_key);
            }
        }

        None
    }

    /// Check if a frontier is currently suppressed due to oscillation.
    fn is_frontier_suppressed(&self, frontier: Point2D) -> bool {
        let key = self.frontier_grid_key(frontier);
        self.suppressed_frontiers.get(&key).copied().unwrap_or(0) > 0
    }

    /// Decrement suppression counters (call once per frontier selection cycle).
    fn decay_suppressed_frontiers(&mut self) {
        self.suppressed_frontiers.retain(|_, count| {
            *count = count.saturating_sub(1);
            *count > 0
        });
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Collision Node Tracking (for alternative approach angles)
    // ─────────────────────────────────────────────────────────────────────────

    /// Convert a frontier point to a grid key for collision tracking.
    /// Uses 0.5m grid cells to group nearby frontiers.
    fn frontier_grid_key(&self, p: Point2D) -> (i32, i32) {
        ((p.x / 0.5) as i32, (p.y / 0.5) as i32)
    }

    /// Record a failed CBVG goal node for a frontier.
    /// This node led to a collision when trying to reach the frontier.
    fn add_collision_node_for_frontier(&mut self, frontier: Point2D, node_idx: usize) {
        let key = self.frontier_grid_key(frontier);
        let nodes = self.collision_nodes.entry(key).or_default();
        if !nodes.contains(&node_idx) {
            nodes.push(node_idx);
            log::debug!(
                "Recorded failed node {} for frontier at ({:.2}, {:.2})",
                node_idx,
                frontier.x,
                frontier.y
            );
        }
    }

    /// Get failed CBVG goal nodes for a frontier.
    /// Returns nodes that should be excluded when planning paths to this frontier.
    fn get_collision_nodes_for_frontier(&self, frontier: Point2D) -> Vec<usize> {
        let key = self.frontier_grid_key(frontier);
        self.collision_nodes.get(&key).cloned().unwrap_or_default()
    }

    /// Clear collision nodes for a frontier (called when frontier is successfully reached).
    fn clear_collision_nodes_for_frontier(&mut self, frontier: Point2D) {
        let key = self.frontier_grid_key(frontier);
        self.collision_nodes.remove(&key);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Occupancy;
    use crate::core::Bounds;
    use crate::features::Line2D;

    /// Mock map for testing
    struct MockMap {
        frontiers: Vec<Frontier>,
        lines: Vec<Line2D>,
    }

    impl MockMap {
        fn new() -> Self {
            Self {
                frontiers: Vec::new(),
                lines: Vec::new(),
            }
        }

        fn with_frontier(mut self, x: f32, y: f32) -> Self {
            // Create a frontier with viewpoint at (x, y)
            // In real usage, the viewpoint is computed from line endpoints
            self.frontiers.push(Frontier {
                viewpoint: Point2D::new(x, y),
                look_direction: Point2D::new(1.0, 0.0), // Default: looking right
                endpoint: Point2D::new(x - 0.4, y),     // Endpoint is offset back
                line_idx: 0,
                estimated_area: 4.0,
            });
            self
        }
    }

    impl Map for MockMap {
        fn observe(
            &mut self,
            _scan: &crate::core::PointCloud2D,
            _odometry: Pose2D,
        ) -> crate::ObserveResult {
            unimplemented!()
        }

        fn raycast(&self, _from: Point2D, _direction: Point2D, max_range: f32) -> f32 {
            max_range
        }

        fn query(&self, _point: Point2D) -> Occupancy {
            Occupancy::Free
        }

        fn frontiers(&self) -> Vec<Frontier> {
            self.frontiers.clone()
        }

        fn get_path(&self, from: Point2D, to: Point2D) -> Option<Path> {
            // Simple direct path
            Some(Path {
                points: vec![from, to],
                length: from.distance(to),
            })
        }

        fn get_path_excluding_nodes(
            &self,
            from: Point2D,
            to: Point2D,
            _excluded_goal_nodes: &[usize],
        ) -> Option<(Path, usize)> {
            // Simple direct path, return node index 0
            Some((
                Path {
                    points: vec![from, to],
                    length: from.distance(to),
                },
                0,
            ))
        }

        fn nearest_cbvg_node(&self, target: Point2D, _max_distance: f32) -> Option<Point2D> {
            // For testing, just return the target point
            Some(target)
        }

        fn is_path_clear(&self, _from: Point2D, _to: Point2D) -> bool {
            true
        }

        fn bounds(&self) -> Bounds {
            Bounds::new(Point2D::new(-10.0, -10.0), Point2D::new(10.0, 10.0))
        }

        fn clear(&mut self) {
            self.frontiers.clear();
            self.lines.clear();
        }
    }

    #[test]
    fn test_controller_initial_state() {
        let config = ExplorationConfig::default();
        let controller = ExplorationController::new(config);

        assert!(matches!(controller.state(), ExplorationState::Idle));
        assert!(!controller.is_active());
        assert!(!controller.is_complete());
    }

    #[test]
    fn test_controller_start() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);

        controller.start();

        assert!(matches!(
            controller.state(),
            ExplorationState::SelectingFrontier
        ));
        assert!(controller.is_active());
    }

    #[test]
    fn test_exploration_complete_no_frontiers() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);
        controller.start();

        let map = MockMap::new(); // No frontiers
        let pose = Pose2D::identity();

        let step = controller.update(&map, pose, None);

        assert!(controller.is_complete());
        assert!(step.velocity.is_none());
    }

    #[test]
    fn test_exploration_selects_frontier() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);
        controller.start();

        let map = MockMap::new().with_frontier(2.0, 0.0);
        let pose = Pose2D::identity();

        let step = controller.update(&map, pose, None);

        assert!(step.path_planned);
        assert!(step.target_frontier.is_some());
        assert!(matches!(
            controller.state(),
            ExplorationState::FollowingPath { .. }
        ));
    }

    #[test]
    fn test_collision_triggers_recovery() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);
        controller.start();

        let map = MockMap::new().with_frontier(2.0, 0.0);
        let pose = Pose2D::identity();

        // First update to start following path
        controller.update(&map, pose, None);

        // Simulate collision
        let collision = CollisionEvent::new(
            super::super::collision::CollisionType::BumperBoth,
            Point2D::new(0.5, 0.0),
            0.0,
        );

        let step = controller.update(&map, pose, Some(collision));

        assert!(step.virtual_wall.is_some());
        assert!(matches!(
            controller.state(),
            ExplorationState::RecoveringFromCollision { .. }
        ));
    }

    #[test]
    fn test_reset() {
        let config = ExplorationConfig::default();
        let mut controller = ExplorationController::new(config);
        controller.start();

        let map = MockMap::new().with_frontier(2.0, 0.0);
        let pose = Pose2D::identity();

        controller.update(&map, pose, None);
        controller.reset();

        assert!(matches!(controller.state(), ExplorationState::Idle));
        assert!(!controller.is_active());
    }
}
