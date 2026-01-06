//! Autonomous exploration state machine.
//!
//! Integrates frontier detection, path planning, and path following
//! for complete autonomous exploration of unknown environments.

use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};

use vastu_slam::{GridStorage, Pose2D, WorldPoint};

use crate::planning::{CostMap, PathSmoother, SmoothedPath, ThetaStarPlanner};
use crate::utils::normalize_angle;

use super::follower::{FollowerConfig, PathFollower};
use super::frontier::{Frontier, FrontierConfig, FrontierDetector, FrontierId};

/// Configuration for the explorer.
#[derive(Clone, Debug)]
pub struct ExplorerConfig {
    /// Frontier detection configuration
    pub frontier: FrontierConfig,
    /// Path follower configuration
    pub follower: FollowerConfig,
    /// Robot radius for cost map (meters)
    pub robot_radius: f32,
    /// Safety margin beyond robot radius (meters)
    pub safety_margin: f32,
    /// Wall penalty distance (meters)
    pub wall_penalty_distance: f32,
    /// Interval between frontier re-evaluation (seconds)
    pub frontier_refresh_interval: f32,
    /// Maximum number of planning failures before giving up on a frontier
    pub max_planning_failures: usize,
    /// Whether to return to start position when exploration is complete
    pub return_to_start: bool,

    // --- Recovery and stuck detection parameters ---
    /// Maximum replans allowed for a single frontier before blacklisting
    pub max_replans_per_frontier: usize,
    /// Timeout for making progress toward frontier (seconds)
    pub frontier_timeout_secs: f32,
    /// Minimum distance to consider progress made (meters)
    pub min_progress_distance: f32,
    /// Cooldown before blacklisted frontier can be retried (seconds)
    pub blacklist_cooldown_secs: f32,
    /// Distance threshold to consider robot stuck (meters)
    pub stuck_distance_threshold: f32,
    /// Time window for stuck detection (seconds)
    pub stuck_time_window_secs: f32,
    /// Distance to back up during recovery (meters)
    pub recovery_backup_distance: f32,
    /// Angle to rotate during recovery (radians)
    pub recovery_rotation_rad: f32,
    /// Maximum recovery attempts per frontier before blacklisting
    pub max_recovery_attempts: usize,
}

impl Default for ExplorerConfig {
    fn default() -> Self {
        Self {
            frontier: FrontierConfig::default(),
            follower: FollowerConfig::default(),
            robot_radius: 0.10,
            safety_margin: 0.05,
            wall_penalty_distance: 0.20,
            frontier_refresh_interval: 1.5,
            max_planning_failures: 3,
            return_to_start: true,
            // Recovery and stuck detection defaults
            max_replans_per_frontier: 3,
            frontier_timeout_secs: 60.0,
            min_progress_distance: 0.10,
            blacklist_cooldown_secs: 30.0,
            stuck_distance_threshold: 0.02,
            stuck_time_window_secs: 3.0,
            recovery_backup_distance: 0.10,
            recovery_rotation_rad: 0.5,
            max_recovery_attempts: 2,
        }
    }
}

/// Phase of recovery behavior.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecoveryPhase {
    /// Backing up from obstacle
    BackingUp,
    /// Rotating to new heading
    Rotating,
    /// Recovery complete
    Done,
}

/// State of the exploration process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExplorationState {
    /// Initial state, waiting to start
    Idle,
    /// Detecting frontiers and selecting target
    SelectingFrontier,
    /// Planning path to selected frontier
    Planning,
    /// Following path to frontier
    Navigating,
    /// Re-planning due to obstacle or blocked path
    Replanning,
    /// Executing recovery behavior (backup + rotate)
    Recovering,
    /// Escaping to a clear area before retrying exhausted frontiers
    EscapingToRetry,
    /// Returning to starting position
    ReturningHome,
    /// Exploration complete
    Complete,
    /// Exploration failed (no path to any frontier)
    Failed,
}

/// Result of an exploration step.
#[derive(Clone, Debug)]
pub struct ExplorationStep {
    /// Current state
    pub state: ExplorationState,
    /// Velocity command (linear, angular)
    pub velocity: (f32, f32),
    /// Current target frontier (if any)
    pub target_frontier: Option<WorldPoint>,
    /// Number of frontiers remaining
    pub frontiers_remaining: usize,
}

/// Autonomous explorer.
pub struct Explorer {
    config: ExplorerConfig,
    /// Current exploration state
    state: ExplorationState,
    /// Frontier detector
    frontier_detector: FrontierDetector,
    /// Path planner
    planner: ThetaStarPlanner,
    /// Path smoother
    smoother: PathSmoother,
    /// Path follower
    follower: PathFollower,
    /// Current cost map
    cost_map: Option<CostMap>,
    /// Current frontiers
    frontiers: Vec<Frontier>,
    /// Current target frontier index
    current_frontier_idx: usize,
    /// Current path
    current_path: Option<SmoothedPath>,
    /// Starting position
    start_pose: Option<Pose2D>,
    /// Last frontier refresh time
    last_frontier_refresh: Instant,
    /// Planning failure count for current frontier
    planning_failures: usize,

    // --- Per-frontier tracking ---
    /// ID of current frontier being navigated to
    current_frontier_id: Option<FrontierId>,
    /// Time when navigation to current frontier started
    frontier_start_time: Option<Instant>,
    /// Number of replans for current frontier
    replan_count_for_frontier: usize,
    /// Number of recovery attempts for current frontier
    recovery_attempts_for_frontier: usize,

    // --- Progress tracking ---
    /// Time of last progress toward goal
    last_progress_time: Instant,
    /// Pose at last progress check
    last_progress_pose: Option<Pose2D>,
    /// Last recorded distance to goal (for progress detection)
    last_distance_to_goal: Option<f32>,

    // --- Stuck detection ---
    /// Start time for stuck detection window
    stuck_check_start: Instant,
    /// Pose at start of stuck detection window
    stuck_check_pose: Option<Pose2D>,

    // --- Frontier blacklist ---
    /// Blacklisted frontiers with timestamp when blacklisted
    blacklisted_frontiers: HashMap<FrontierId, Instant>,
    /// Permanently blacklisted frontiers (exhausted too many times)
    permanent_blacklist: HashSet<FrontierId>,
    /// Count of exhaustion cycles per frontier
    exhaustion_count: HashMap<FrontierId, u32>,
    /// Whether we've tried escaping to a clear area before permanent blacklist
    tried_escape_for_exhausted: bool,

    // --- Recovery state ---
    /// Current phase of recovery behavior
    recovery_phase: RecoveryPhase,
    /// Pose at start of recovery
    recovery_start_pose: Option<Pose2D>,
}

impl Explorer {
    /// Create a new explorer with configuration.
    pub fn new(config: ExplorerConfig) -> Self {
        let now = Instant::now();
        Self {
            frontier_detector: FrontierDetector::new(config.frontier.clone()),
            planner: ThetaStarPlanner::with_defaults(),
            smoother: PathSmoother::with_defaults(),
            follower: PathFollower::new(config.follower.clone()),
            config,
            state: ExplorationState::Idle,
            cost_map: None,
            frontiers: Vec::new(),
            current_frontier_idx: 0,
            current_path: None,
            start_pose: None,
            last_frontier_refresh: now,
            planning_failures: 0,
            // Per-frontier tracking
            current_frontier_id: None,
            frontier_start_time: None,
            replan_count_for_frontier: 0,
            recovery_attempts_for_frontier: 0,
            // Progress tracking
            last_progress_time: now,
            last_progress_pose: None,
            last_distance_to_goal: None,
            // Stuck detection
            stuck_check_start: now,
            stuck_check_pose: None,
            // Blacklist
            blacklisted_frontiers: HashMap::new(),
            permanent_blacklist: HashSet::new(),
            exhaustion_count: HashMap::new(),
            tried_escape_for_exhausted: false,
            // Recovery state
            recovery_phase: RecoveryPhase::Done,
            recovery_start_pose: None,
        }
    }

    /// Start exploration from current position.
    pub fn start(&mut self, current_pose: Pose2D) {
        let now = Instant::now();
        self.start_pose = Some(current_pose);
        self.state = ExplorationState::SelectingFrontier;
        self.last_frontier_refresh = now;
        self.planning_failures = 0;
        // Reset per-frontier tracking
        self.current_frontier_id = None;
        self.frontier_start_time = None;
        self.replan_count_for_frontier = 0;
        self.recovery_attempts_for_frontier = 0;
        // Reset progress tracking
        self.last_progress_time = now;
        self.last_progress_pose = Some(current_pose);
        self.last_distance_to_goal = None;
        // Reset stuck detection
        self.stuck_check_start = now;
        self.stuck_check_pose = Some(current_pose);
        // Clear blacklist on fresh start
        self.blacklisted_frontiers.clear();
        self.permanent_blacklist.clear();
        self.exhaustion_count.clear();
        self.tried_escape_for_exhausted = false;
        // Reset recovery state
        self.recovery_phase = RecoveryPhase::Done;
        self.recovery_start_pose = None;
        tracing::info!(
            "Exploration started at ({:.2}, {:.2})",
            current_pose.x,
            current_pose.y
        );
    }

    /// Check if exploration is complete.
    pub fn is_complete(&self) -> bool {
        matches!(
            self.state,
            ExplorationState::Complete | ExplorationState::Failed
        )
    }

    /// Update exploration with new map data.
    ///
    /// Returns velocity command and state information.
    pub fn update(&mut self, grid: &GridStorage, current_pose: Pose2D) -> ExplorationStep {
        // Update cost map from grid
        // Use robot_radius + safety_margin as the inscribed radius for path planning
        // This ensures paths stay at least safety_margin away from all obstacles
        let planning_radius = self.config.robot_radius + self.config.safety_margin;
        let cost_map = CostMap::from_grid(
            grid,
            planning_radius, // Larger inscribed zone (blocked)
            0.10,            // Additional inflation for wall penalty
            self.config.wall_penalty_distance,
        );
        self.cost_map = Some(cost_map);

        // Check if we need to refresh frontiers
        let should_refresh = self.last_frontier_refresh.elapsed()
            > Duration::from_secs_f32(self.config.frontier_refresh_interval);

        match self.state {
            ExplorationState::Idle => ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: 0,
            },

            ExplorationState::SelectingFrontier => self.select_frontier(grid, current_pose),

            ExplorationState::Planning => self.plan_path(current_pose),

            ExplorationState::Navigating => self.navigate(grid, current_pose, should_refresh),

            ExplorationState::Replanning => self.replan(current_pose),

            ExplorationState::ReturningHome => self.return_home(current_pose),

            ExplorationState::Recovering => self.execute_recovery(current_pose),

            ExplorationState::EscapingToRetry => self.escape_to_retry(current_pose),

            ExplorationState::Complete | ExplorationState::Failed => ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: 0,
            },
        }
    }

    /// Select a frontier to explore.
    fn select_frontier(&mut self, grid: &GridStorage, current_pose: Pose2D) -> ExplorationStep {
        // Clean up expired blacklist entries first (before borrowing cost_map)
        self.clean_blacklist();

        let cost_map = self.cost_map.as_ref().unwrap();

        // Detect frontiers
        let all_frontiers = self.frontier_detector.detect(grid, cost_map, current_pose);
        self.last_frontier_refresh = Instant::now();

        // Filter out blacklisted and permanently blacklisted frontiers
        let available_frontiers: Vec<Frontier> = all_frontiers
            .into_iter()
            .filter(|f| {
                !self.is_blacklisted(&f.id()) && !self.permanent_blacklist.contains(&f.id())
            })
            .collect();

        let blacklisted_count = self.blacklisted_frontiers.len();
        if blacklisted_count > 0 {
            tracing::debug!(
                "Filtered frontiers: {} available, {} blacklisted",
                available_frontiers.len(),
                blacklisted_count
            );
        }

        // If all frontiers are blacklisted, increment exhaustion count and handle escape/permanent blacklist
        if available_frontiers.is_empty() && blacklisted_count > 0 {
            tracing::warn!(
                "All {} frontiers blacklisted, clearing blacklist and retrying",
                blacklisted_count
            );

            // Increment exhaustion count for all currently blacklisted frontiers
            let blacklisted_ids: Vec<_> = self.blacklisted_frontiers.keys().cloned().collect();
            let mut any_exhausted = false;
            for frontier_id in &blacklisted_ids {
                let count = self
                    .exhaustion_count
                    .entry(frontier_id.clone())
                    .or_insert(0);
                *count += 1;
                if *count >= 3 {
                    any_exhausted = true;
                }
            }

            // If frontiers are exhausted and we haven't tried escaping, escape first
            if any_exhausted && !self.tried_escape_for_exhausted && self.start_pose.is_some() {
                tracing::warn!(
                    "Frontiers exhausted from current position, escaping to start to retry"
                );
                self.tried_escape_for_exhausted = true;
                // Clear temporary blacklist so we can retry after escaping
                self.blacklisted_frontiers.clear();
                self.state = ExplorationState::EscapingToRetry;
                self.follower.clear_path();
                return ExplorationStep {
                    state: self.state,
                    velocity: (0.0, 0.0),
                    target_frontier: None,
                    frontiers_remaining: blacklisted_count,
                };
            }

            // Already tried escaping or no start pose, permanently blacklist exhausted frontiers
            for frontier_id in blacklisted_ids {
                let count = *self.exhaustion_count.get(&frontier_id).unwrap_or(&0);
                if count >= 3 {
                    tracing::warn!(
                        "Frontier at ({}, {}) exhausted {} times, permanently blacklisting",
                        frontier_id.centroid.x,
                        frontier_id.centroid.y,
                        count
                    );
                    self.permanent_blacklist.insert(frontier_id);
                }
            }

            // Clear temporary blacklist
            self.blacklisted_frontiers.clear();

            // Re-detect, filtering out permanently blacklisted
            let new_frontiers = self.frontier_detector.detect(grid, cost_map, current_pose);
            self.frontiers = new_frontiers
                .into_iter()
                .filter(|f| !self.permanent_blacklist.contains(&f.id()))
                .collect();

            if self.frontiers.is_empty() && !self.permanent_blacklist.is_empty() {
                tracing::info!(
                    "No reachable frontiers remaining ({} permanently blacklisted)",
                    self.permanent_blacklist.len()
                );
            }
        } else {
            self.frontiers = available_frontiers;
        }

        if self.frontiers.is_empty() {
            // No more frontiers - exploration complete or return home
            if self.config.return_to_start && self.start_pose.is_some() {
                tracing::info!("No frontiers remaining, returning to start");
                self.state = ExplorationState::ReturningHome;
            } else {
                tracing::info!("Exploration complete: no frontiers remaining");
                self.state = ExplorationState::Complete;
            }

            return ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: 0,
            };
        }

        tracing::debug!("Found {} frontiers", self.frontiers.len());
        self.current_frontier_idx = 0;
        self.planning_failures = 0;

        // Reset per-frontier tracking for the new frontier
        let frontier_id = self.frontiers[0].id();
        self.reset_frontier_tracking(frontier_id);

        self.state = ExplorationState::Planning;

        ExplorationStep {
            state: self.state,
            velocity: (0.0, 0.0),
            target_frontier: Some(self.frontiers[0].world_centroid),
            frontiers_remaining: self.frontiers.len(),
        }
    }

    /// Plan path to current frontier.
    fn plan_path(&mut self, current_pose: Pose2D) -> ExplorationStep {
        let cost_map = self.cost_map.as_ref().unwrap();

        if self.current_frontier_idx >= self.frontiers.len() {
            // Tried all frontiers, none reachable
            tracing::warn!("No reachable frontiers found");
            self.state = ExplorationState::Failed;

            return ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: 0,
            };
        }

        let frontier = &self.frontiers[self.current_frontier_idx];

        // Get navigation target, handling case where no reachable point exists
        let target =
            match self
                .frontier_detector
                .navigation_target(frontier, cost_map, current_pose)
            {
                Some(t) => t,
                None => {
                    tracing::debug!(
                        "No reachable point for frontier {}, trying next",
                        self.current_frontier_idx
                    );
                    self.current_frontier_idx += 1;
                    return self.plan_path(current_pose); // Recursive call to try next frontier
                }
            };

        tracing::debug!(
            "Planning path to frontier {} at ({:.2}, {:.2})",
            self.current_frontier_idx,
            target.x,
            target.y
        );

        // Plan path
        match self.planner.plan(cost_map, current_pose, target) {
            Some(path) => {
                // Check for valid path
                if path.waypoints.len() < 2 {
                    tracing::warn!(
                        "Path has only {} waypoints, skipping frontier",
                        path.waypoints.len()
                    );
                    self.current_frontier_idx += 1;
                    return self.plan_path(current_pose);
                }

                // Smooth the path
                let smoothed = self.smoother.smooth(&path);

                // Check for valid smoothed path
                if smoothed.segments.is_empty() {
                    tracing::warn!("Smoothed path has no segments, skipping frontier");
                    self.current_frontier_idx += 1;
                    return self.plan_path(current_pose);
                }

                tracing::info!(
                    "Path planned: {} waypoints -> {} segments, {:.2}m, {:.1}s",
                    path.waypoints.len(),
                    smoothed.segments.len(),
                    smoothed.total_length,
                    smoothed.estimated_time
                );

                // Log first few waypoints for debugging
                for (i, wp) in path.waypoints.iter().take(3).enumerate() {
                    tracing::debug!("  waypoint {}: ({:.2}, {:.2})", i, wp.x, wp.y);
                }

                self.follower.set_path(smoothed.clone());
                self.current_path = Some(smoothed);
                self.planning_failures = 0;
                self.state = ExplorationState::Navigating;

                ExplorationStep {
                    state: self.state,
                    velocity: (0.0, 0.0),
                    target_frontier: Some(target),
                    frontiers_remaining: self.frontiers.len() - self.current_frontier_idx,
                }
            }
            None => {
                tracing::debug!(
                    "Failed to plan path to frontier {}",
                    self.current_frontier_idx
                );

                self.planning_failures += 1;

                if self.planning_failures >= self.config.max_planning_failures {
                    // Try next frontier
                    self.current_frontier_idx += 1;
                    self.planning_failures = 0;
                }

                ExplorationStep {
                    state: self.state,
                    velocity: (0.0, 0.0),
                    target_frontier: None,
                    frontiers_remaining: self.frontiers.len() - self.current_frontier_idx,
                }
            }
        }
    }

    /// Navigate along current path.
    fn navigate(
        &mut self,
        grid: &GridStorage,
        current_pose: Pose2D,
        should_refresh: bool,
    ) -> ExplorationStep {
        // Check if path following is complete
        if self.follower.is_complete() {
            tracing::info!("Reached frontier, selecting next");
            self.state = ExplorationState::SelectingFrontier;

            return ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: self.frontiers.len(),
            };
        }

        // Get current goal for progress tracking
        let goal = self.follower.goal();

        // --- STUCK DETECTION ---
        // Check if robot is physically stuck (not moving despite commands)
        // Do this before borrowing cost_map to avoid borrow checker issues
        if self.check_stuck(current_pose) {
            tracing::warn!("Stuck detected, initiating recovery");
            self.state = ExplorationState::Recovering;
            self.recovery_phase = RecoveryPhase::BackingUp;
            self.recovery_start_pose = None; // Will be set on first recovery call
            self.follower.clear_path();
            return ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: goal,
                frontiers_remaining: self.frontiers.len(),
            };
        }

        // --- PROGRESS TIMEOUT CHECK ---
        // Check if making progress toward goal
        // Do this before borrowing cost_map to avoid borrow checker issues
        if let Some(goal_point) = goal
            && !self.check_progress(current_pose, goal_point)
        {
            // No progress timeout - blacklist frontier and try another
            tracing::warn!("Progress timeout, blacklisting frontier");
            if let Some(ref frontier_id) = self.current_frontier_id.clone() {
                self.blacklist_frontier(frontier_id.clone());
            }
            self.state = ExplorationState::SelectingFrontier;
            self.follower.clear_path();
            return ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: self.frontiers.len(),
            };
        }

        // Now borrow cost_map for remaining operations
        let cost_map = self.cost_map.as_ref().unwrap();

        // SAFETY CHECK 1: Check if current position is dangerously close to obstacles
        let current_pos = WorldPoint::new(current_pose.x, current_pose.y);
        let current_coord = cost_map.world_to_grid(current_pos);
        let current_distance = cost_map.obstacle_distance(current_coord);
        // Minimum safe distance is the physical robot radius (not planning radius)
        let min_safe_distance = self.config.robot_radius / cost_map.resolution();

        if current_distance < min_safe_distance {
            tracing::warn!(
                "Robot too close to obstacle! dist={:.2} cells ({:.2}m), min={:.2} cells ({:.2}m), triggering replan",
                current_distance,
                current_distance * cost_map.resolution(),
                min_safe_distance,
                min_safe_distance * cost_map.resolution()
            );
            // Stop immediately and replan
            self.state = ExplorationState::Replanning;
            return ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: self.frontiers.len(),
            };
        }

        // Compute velocity first - this determines if we're moving forward or turning
        let mut velocity = self.follower.compute_velocity(current_pose);

        // SAFETY CHECK 2: Check immediate path ahead in robot's heading direction
        // Only check when moving forward (linear velocity > 0), not when turning in place.
        // This prevents replan loops when the robot needs to turn to face a new path.
        if velocity.0 > 0.01 {
            let check_distance = self.config.robot_radius + 0.05; // 15cm ahead
            let heading_x = current_pose.x + check_distance * current_pose.theta.cos();
            let heading_y = current_pose.y + check_distance * current_pose.theta.sin();
            let heading_coord = cost_map.world_to_grid(WorldPoint::new(heading_x, heading_y));

            if !cost_map.is_traversable(heading_coord) {
                // Increment replan counter for this frontier
                self.replan_count_for_frontier += 1;
                tracing::warn!(
                    "Obstacle directly ahead at ({:.2}, {:.2}), replan #{} for frontier",
                    heading_x,
                    heading_y,
                    self.replan_count_for_frontier
                );

                // Check if too many replans for this frontier
                if self.replan_count_for_frontier >= self.config.max_replans_per_frontier {
                    // Too many replans - need to physically move away from obstacle
                    // Trigger recovery (backup + rotate) before trying new frontier
                    tracing::warn!(
                        "Max replans ({}) reached, triggering recovery to escape obstacle",
                        self.config.max_replans_per_frontier
                    );
                    self.state = ExplorationState::Recovering;
                    self.recovery_phase = RecoveryPhase::BackingUp;
                    self.recovery_start_pose = None;
                    self.follower.clear_path();
                    return ExplorationStep {
                        state: self.state,
                        velocity: (0.0, 0.0),
                        target_frontier: None,
                        frontiers_remaining: self.frontiers.len(),
                    };
                }

                // Obstacle in our path - need to replan to find alternative route
                self.state = ExplorationState::Replanning;
                self.follower.clear_path();
                return ExplorationStep {
                    state: self.state,
                    velocity: (0.0, 0.0),
                    target_frontier: None,
                    frontiers_remaining: self.frontiers.len(),
                };
            }
        }

        // SAFETY: Scale down velocity when within 2x safety margin of obstacles
        // This gives more reaction time near walls
        let safe_distance =
            (self.config.robot_radius + self.config.safety_margin) / cost_map.resolution();
        if current_distance < safe_distance * 2.0 {
            // Scale velocity: at safe_distance -> 50% speed, at 2x -> 100% speed
            let scale = (current_distance / safe_distance / 2.0).clamp(0.3, 1.0);
            velocity.0 *= scale;
            tracing::debug!(
                "Near obstacle (dist={:.2}m, safe={:.2}m), scaling velocity to {:.0}%",
                current_distance * cost_map.resolution(),
                safe_distance * cost_map.resolution(),
                scale * 100.0
            );
        }

        // Log velocity at debug level
        if velocity.0.abs() > 0.001 || velocity.1.abs() > 0.001 {
            tracing::debug!(
                "Navigate velocity: linear={:.3}, angular={:.3}",
                velocity.0,
                velocity.1
            );
        } else {
            tracing::warn!(
                "Navigate returning zero velocity at ({:.2}, {:.2})",
                current_pose.x,
                current_pose.y
            );
        }

        // Check if we should refresh frontiers (might find a better one)
        if should_refresh {
            let new_frontiers = self.frontier_detector.detect(grid, cost_map, current_pose);
            self.last_frontier_refresh = Instant::now();

            if new_frontiers.is_empty() {
                // Map fully explored while navigating
                if self.config.return_to_start && self.start_pose.is_some() {
                    self.state = ExplorationState::ReturningHome;
                    self.follower.clear_path();
                } else {
                    self.state = ExplorationState::Complete;
                }
            } else if !new_frontiers.is_empty() && self.frontiers.is_empty() {
                // Found new frontiers after thinking we were done
                self.frontiers = new_frontiers;
            }
        }

        ExplorationStep {
            state: self.state,
            velocity,
            target_frontier: goal,
            frontiers_remaining: self.frontiers.len(),
        }
    }

    /// Replan due to blocked path.
    fn replan(&mut self, current_pose: Pose2D) -> ExplorationStep {
        self.follower.clear_path();
        self.state = ExplorationState::Planning;
        self.plan_path(current_pose)
    }

    /// Return to starting position.
    fn return_home(&mut self, current_pose: Pose2D) -> ExplorationStep {
        let start = match self.start_pose {
            Some(pose) => WorldPoint::new(pose.x, pose.y),
            None => {
                self.state = ExplorationState::Complete;
                return ExplorationStep {
                    state: self.state,
                    velocity: (0.0, 0.0),
                    target_frontier: None,
                    frontiers_remaining: 0,
                };
            }
        };

        // Check if already home
        let current_pos = WorldPoint::new(current_pose.x, current_pose.y);
        let distance_to_start = current_pos.distance(&start);

        if distance_to_start < self.config.follower.goal_tolerance {
            tracing::info!("Returned to start position");
            self.state = ExplorationState::Complete;

            return ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: 0,
            };
        }

        // Follow path home
        // Note: We trust the path planner (Theta*) to find obstacle-free paths.
        // No obstacle checking here - if robot hits something, bumper triggers safety stop.
        if self.follower.has_path() {
            if self.follower.is_complete() {
                self.state = ExplorationState::Complete;
                return ExplorationStep {
                    state: self.state,
                    velocity: (0.0, 0.0),
                    target_frontier: None,
                    frontiers_remaining: 0,
                };
            }

            let velocity = self.follower.compute_velocity(current_pose);
            return ExplorationStep {
                state: self.state,
                velocity,
                target_frontier: Some(start),
                frontiers_remaining: 0,
            };
        }

        // Plan path home
        let cost_map = self.cost_map.as_ref().unwrap();
        match self.planner.plan(cost_map, current_pose, start) {
            Some(path) => {
                let smoothed = self.smoother.smooth(&path);
                self.follower.set_path(smoothed);
                tracing::info!("Path to start planned, {:.2}m", path.length);
            }
            None => {
                tracing::error!("Cannot find path back to start");
                self.state = ExplorationState::Failed;
            }
        }

        ExplorationStep {
            state: self.state,
            velocity: (0.0, 0.0),
            target_frontier: Some(start),
            frontiers_remaining: 0,
        }
    }

    /// Navigate toward start position to escape from trapped area, then retry frontiers.
    fn escape_to_retry(&mut self, current_pose: Pose2D) -> ExplorationStep {
        let start = match self.start_pose {
            Some(pose) => WorldPoint::new(pose.x, pose.y),
            None => {
                // No start pose, go back to selecting frontier
                tracing::warn!("No start pose for escape, returning to frontier selection");
                self.state = ExplorationState::SelectingFrontier;
                return ExplorationStep {
                    state: self.state,
                    velocity: (0.0, 0.0),
                    target_frontier: None,
                    frontiers_remaining: self.frontiers.len(),
                };
            }
        };

        // Check if we've reached near the start position
        let current_pos = WorldPoint::new(current_pose.x, current_pose.y);
        let distance_to_start = current_pos.distance(&start);

        // Use a larger tolerance (0.5m) since we just need to get to a different area
        if distance_to_start < 0.5 {
            tracing::info!(
                "Escaped to near start position ({:.2}m away), resetting exhaustion and retrying frontiers",
                distance_to_start
            );
            // Reset exhaustion counts so frontiers get another chance from this position
            self.exhaustion_count.clear();
            self.blacklisted_frontiers.clear();
            self.state = ExplorationState::SelectingFrontier;

            return ExplorationStep {
                state: self.state,
                velocity: (0.0, 0.0),
                target_frontier: None,
                frontiers_remaining: 0,
            };
        }

        // Follow path to start
        if self.follower.has_path() {
            if self.follower.is_complete() {
                // Path complete but not close enough yet - replan
                self.follower.clear_path();
            } else {
                let velocity = self.follower.compute_velocity(current_pose);
                return ExplorationStep {
                    state: self.state,
                    velocity,
                    target_frontier: Some(start),
                    frontiers_remaining: self.frontiers.len(),
                };
            }
        }

        // Plan path to start
        let cost_map = self.cost_map.as_ref().unwrap();
        match self.planner.plan(cost_map, current_pose, start) {
            Some(path) => {
                let smoothed = self.smoother.smooth(&path);
                tracing::info!("Escaping to start: path planned, {:.2}m", path.length);
                self.follower.set_path(smoothed);
            }
            None => {
                // Can't find path to start - just go back to selecting frontier
                tracing::warn!("Cannot plan escape path, retrying frontiers from current position");
                self.exhaustion_count.clear();
                self.blacklisted_frontiers.clear();
                self.state = ExplorationState::SelectingFrontier;
            }
        }

        ExplorationStep {
            state: self.state,
            velocity: (0.0, 0.0),
            target_frontier: Some(start),
            frontiers_remaining: self.frontiers.len(),
        }
    }

    // --- Helper methods for stuck detection and recovery ---

    /// Check if robot is stuck (hasn't moved significantly in time window).
    fn check_stuck(&mut self, current_pose: Pose2D) -> bool {
        let now = Instant::now();
        let time_window = Duration::from_secs_f32(self.config.stuck_time_window_secs);

        // Initialize stuck check pose if needed
        if self.stuck_check_pose.is_none() {
            self.stuck_check_pose = Some(current_pose);
            self.stuck_check_start = now;
            return false;
        }

        // Check if time window has elapsed
        if now.duration_since(self.stuck_check_start) < time_window {
            return false;
        }

        // Calculate distance moved since start of window
        let start_pose = self.stuck_check_pose.unwrap();
        let dx = current_pose.x - start_pose.x;
        let dy = current_pose.y - start_pose.y;
        let distance_moved = (dx * dx + dy * dy).sqrt();

        // Check if stuck
        let is_stuck = distance_moved < self.config.stuck_distance_threshold;

        if is_stuck {
            tracing::warn!(
                "Robot stuck! Moved only {:.3}m in {:.1}s (threshold: {:.3}m)",
                distance_moved,
                self.config.stuck_time_window_secs,
                self.config.stuck_distance_threshold
            );
        }

        // Reset window for next check
        self.stuck_check_pose = Some(current_pose);
        self.stuck_check_start = now;

        is_stuck
    }

    /// Check if making progress toward goal. Returns false if timeout reached.
    fn check_progress(&mut self, current_pose: Pose2D, goal: WorldPoint) -> bool {
        let now = Instant::now();
        let timeout = Duration::from_secs_f32(self.config.frontier_timeout_secs);

        // Calculate current distance to goal
        let current_pos = WorldPoint::new(current_pose.x, current_pose.y);
        let distance_to_goal = current_pos.distance(&goal);

        // Check if we made progress
        if let Some(last_distance) = self.last_distance_to_goal {
            let progress = last_distance - distance_to_goal;
            if progress > self.config.min_progress_distance {
                // Made progress, reset timer
                self.last_progress_time = now;
                self.last_progress_pose = Some(current_pose);
                self.last_distance_to_goal = Some(distance_to_goal);
                return true;
            }
        } else {
            // First check, initialize
            self.last_distance_to_goal = Some(distance_to_goal);
            self.last_progress_time = now;
            self.last_progress_pose = Some(current_pose);
            return true;
        }

        // Check if timeout
        if now.duration_since(self.last_progress_time) >= timeout {
            tracing::warn!(
                "No progress timeout! No progress toward goal in {:.1}s (dist: {:.2}m)",
                self.config.frontier_timeout_secs,
                distance_to_goal
            );
            return false;
        }

        // Update distance tracking
        self.last_distance_to_goal = Some(distance_to_goal);
        true
    }

    /// Blacklist a frontier.
    fn blacklist_frontier(&mut self, frontier_id: FrontierId) {
        tracing::info!(
            "Blacklisting frontier at ({}, {})",
            frontier_id.centroid.x,
            frontier_id.centroid.y
        );
        self.blacklisted_frontiers
            .insert(frontier_id, Instant::now());
    }

    /// Check if a frontier is blacklisted (and cooldown hasn't expired).
    fn is_blacklisted(&self, frontier_id: &FrontierId) -> bool {
        if let Some(blacklist_time) = self.blacklisted_frontiers.get(frontier_id) {
            let cooldown = Duration::from_secs_f32(self.config.blacklist_cooldown_secs);
            let expired = blacklist_time.elapsed() >= cooldown;
            !expired
        } else {
            false
        }
    }

    /// Clean up expired blacklist entries.
    fn clean_blacklist(&mut self) {
        let cooldown = Duration::from_secs_f32(self.config.blacklist_cooldown_secs);
        self.blacklisted_frontiers
            .retain(|_, time| time.elapsed() < cooldown);
    }

    /// Reset tracking for a new frontier.
    fn reset_frontier_tracking(&mut self, frontier_id: FrontierId) {
        let now = Instant::now();
        self.current_frontier_id = Some(frontier_id);
        self.frontier_start_time = Some(now);
        self.replan_count_for_frontier = 0;
        self.recovery_attempts_for_frontier = 0;
        self.last_progress_time = now;
        self.last_progress_pose = None;
        self.last_distance_to_goal = None;
        self.stuck_check_start = now;
        self.stuck_check_pose = None;
    }

    /// Execute recovery behavior (backup + rotate).
    fn execute_recovery(&mut self, current_pose: Pose2D) -> ExplorationStep {
        match self.recovery_phase {
            RecoveryPhase::BackingUp => {
                // Initialize recovery if first call
                if self.recovery_start_pose.is_none() {
                    self.recovery_start_pose = Some(current_pose);
                    tracing::info!(
                        "Starting recovery: backing up {:.2}m",
                        self.config.recovery_backup_distance
                    );
                }

                // Calculate distance backed up
                let start = self.recovery_start_pose.unwrap();
                let dx = current_pose.x - start.x;
                let dy = current_pose.y - start.y;
                let backed_up = (dx * dx + dy * dy).sqrt();

                if backed_up >= self.config.recovery_backup_distance {
                    // Done backing up, start rotating
                    tracing::debug!("Backup complete ({:.2}m), starting rotation", backed_up);
                    self.recovery_phase = RecoveryPhase::Rotating;
                    self.recovery_start_pose = Some(current_pose);

                    ExplorationStep {
                        state: self.state,
                        velocity: (0.0, 0.0), // Stop before rotating
                        target_frontier: None,
                        frontiers_remaining: self.frontiers.len(),
                    }
                } else {
                    // Continue backing up (negative linear velocity)
                    ExplorationStep {
                        state: self.state,
                        velocity: (-0.1, 0.0), // Back up at 0.1 m/s
                        target_frontier: None,
                        frontiers_remaining: self.frontiers.len(),
                    }
                }
            }

            RecoveryPhase::Rotating => {
                // Calculate angle rotated (normalized to handle wraparound)
                let start = self.recovery_start_pose.unwrap();
                let angle_diff = current_pose.theta - start.theta;
                let angle_rotated = normalize_angle(angle_diff).abs();

                if angle_rotated >= self.config.recovery_rotation_rad {
                    // Done rotating, recovery complete
                    tracing::info!(
                        "Recovery complete (rotated {:.1}°)",
                        angle_rotated.to_degrees()
                    );
                    self.recovery_phase = RecoveryPhase::Done;
                    self.recovery_start_pose = None;
                    self.recovery_attempts_for_frontier += 1;

                    // Check if too many recovery attempts
                    if self.recovery_attempts_for_frontier >= self.config.max_recovery_attempts {
                        tracing::warn!(
                            "Max recovery attempts ({}) reached, blacklisting frontier",
                            self.config.max_recovery_attempts
                        );
                        if let Some(ref frontier_id) = self.current_frontier_id.clone() {
                            self.blacklist_frontier(frontier_id.clone());
                        }
                        self.state = ExplorationState::SelectingFrontier;
                    } else {
                        // Try replanning
                        self.state = ExplorationState::Replanning;
                    }

                    ExplorationStep {
                        state: self.state,
                        velocity: (0.0, 0.0),
                        target_frontier: None,
                        frontiers_remaining: self.frontiers.len(),
                    }
                } else {
                    // Continue rotating
                    ExplorationStep {
                        state: self.state,
                        velocity: (0.0, 0.4), // Rotate at 0.4 rad/s
                        target_frontier: None,
                        frontiers_remaining: self.frontiers.len(),
                    }
                }
            }

            RecoveryPhase::Done => {
                // Shouldn't happen, transition to replanning
                self.state = ExplorationState::Replanning;
                ExplorationStep {
                    state: self.state,
                    velocity: (0.0, 0.0),
                    target_frontier: None,
                    frontiers_remaining: self.frontiers.len(),
                }
            }
        }
    }

    /// Get current path for visualization.
    pub fn current_path(&self) -> Option<&SmoothedPath> {
        self.current_path.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vastu_slam::{CellType, GridCoord};

    fn create_partial_grid() -> GridStorage {
        // 60x60 grid, larger to avoid inflation issues
        let mut grid = GridStorage::new(60, 60, 0.05, WorldPoint::ZERO);

        // Mark explored area (floor) in a region
        for y in 10..30 {
            for x in 10..30 {
                grid.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        // Add a wall in the middle of explored area
        for x in 15..25 {
            grid.set_type(GridCoord::new(x, 20), CellType::Wall);
        }

        grid
    }

    #[test]
    fn test_explorer_creation() {
        let explorer = Explorer::new(ExplorerConfig::default());
        assert!(!explorer.is_complete());
    }

    #[test]
    fn test_explorer_start() {
        let mut explorer = Explorer::new(ExplorerConfig::default());
        let pose = Pose2D::new(0.75, 0.75, 0.0);

        explorer.start(pose);
        assert!(!explorer.is_complete());
    }

    #[test]
    fn test_frontier_detection_on_update() {
        // Use smaller robot radius for this test
        let mut explorer = Explorer::new(ExplorerConfig {
            robot_radius: 0.05,
            safety_margin: 0.02,
            wall_penalty_distance: 0.10,
            ..Default::default()
        });
        let grid = create_partial_grid();
        let pose = Pose2D::new(0.75, 0.75, 0.0);

        explorer.start(pose);
        let step = explorer.update(&grid, pose);

        // Should have found frontiers
        assert!(
            step.frontiers_remaining > 0,
            "Expected frontiers at Floor/Unknown boundary"
        );
    }

    #[test]
    fn test_complete_on_full_exploration() {
        let mut explorer = Explorer::new(ExplorerConfig {
            return_to_start: false,
            robot_radius: 0.05,
            safety_margin: 0.02,
            wall_penalty_distance: 0.10,
            ..Default::default()
        });

        // Fully explored grid (all floor)
        let mut grid = GridStorage::new(30, 30, 0.05, WorldPoint::ZERO);
        for y in 0..30 {
            for x in 0..30 {
                grid.set_type(GridCoord::new(x, y), CellType::Floor);
            }
        }

        let pose = Pose2D::new(0.75, 0.75, 0.0);
        explorer.start(pose);
        let step = explorer.update(&grid, pose);

        // Should complete immediately (no frontiers)
        assert_eq!(step.state, ExplorationState::Complete);
        assert!(explorer.is_complete());
    }
}
