//! Exploration thread: Frontier detection and path planning.
//!
//! This thread reads from the shared grid and:
//! - Detects frontiers (boundaries between known and unknown space)
//! - Plans paths to frontiers using Theta*
//! - Follows paths using pure pursuit
//! - Updates velocity commands in shared state

use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::config::DhruvaConfig;
use crate::exploration::{
    ExplorationState, Explorer, ExplorerConfig, FollowerConfig, FrontierConfig,
};
use crate::shared::{SharedDebugViz, SharedGrid, SharedState};

/// Exploration thread state and logic.
pub struct ExplorationThread {
    shared_state: Arc<SharedState>,
    shared_grid: SharedGrid,
    shared_debug_viz: SharedDebugViz,
    explorer: Explorer,
    last_status_time: Instant,
    status_interval: Duration,
    started: bool,
    min_scans_to_start: u32,
}

impl ExplorationThread {
    /// Create a new exploration thread.
    pub fn new(
        config: DhruvaConfig,
        shared_state: Arc<SharedState>,
        shared_grid: SharedGrid,
        shared_debug_viz: SharedDebugViz,
    ) -> Self {
        // Configure exploration from config
        let explorer_config = ExplorerConfig {
            frontier: FrontierConfig {
                min_cluster_size: config.exploration.min_frontier_size,
                ..Default::default()
            },
            follower: FollowerConfig {
                goal_tolerance: config.exploration.goal_tolerance,
                max_linear_vel: config.robot.max_linear_vel,
                max_angular_vel: config.robot.max_angular_vel,
                ..Default::default()
            },
            robot_radius: config.exploration.robot_radius,
            safety_margin: config.exploration.safety_margin,
            wall_penalty_distance: config.exploration.wall_penalty_distance,
            frontier_refresh_interval: config.exploration.frontier_refresh_interval,
            return_to_start: config.exploration.return_to_start,
            // Recovery and stuck detection parameters
            max_replans_per_frontier: config.exploration.max_replans_per_frontier,
            frontier_timeout_secs: config.exploration.frontier_timeout_secs,
            min_progress_distance: config.exploration.min_progress_distance,
            blacklist_cooldown_secs: config.exploration.blacklist_cooldown_secs,
            stuck_distance_threshold: config.exploration.stuck_distance_threshold,
            stuck_time_window_secs: config.exploration.stuck_time_window_secs,
            recovery_backup_distance: config.exploration.recovery_backup_distance,
            recovery_rotation_rad: config.exploration.recovery_rotation_rad,
            max_recovery_attempts: config.exploration.max_recovery_attempts,
            ..Default::default()
        };

        Self {
            shared_state,
            shared_grid,
            shared_debug_viz,
            explorer: Explorer::new(explorer_config),
            last_status_time: Instant::now(),
            status_interval: Duration::from_secs(3),
            started: false,
            min_scans_to_start: 10,
        }
    }

    /// Run the exploration thread main loop.
    pub fn run(&mut self) {
        tracing::info!("Exploration thread started");

        // Target ~20Hz for exploration updates
        let loop_interval = Duration::from_millis(50);

        loop {
            let loop_start = Instant::now();

            // Check for shutdown
            if self.shared_state.should_shutdown() {
                tracing::info!("Exploration thread shutting down");
                break;
            }

            // Check for safety stop - set velocity to zero
            if self.shared_state.is_safety_stop() {
                self.shared_state.set_velocity(0.0, 0.0);
                std::thread::sleep(Duration::from_millis(100));
                continue;
            }

            // Wait for enough scans before starting exploration
            if !self.started {
                if self.shared_state.scan_count() < self.min_scans_to_start {
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }
                // Start exploration
                let current_pose = self.shared_state.pose();
                self.explorer.start(current_pose);
                self.started = true;
                tracing::info!(
                    "Exploration starting after {} scans",
                    self.min_scans_to_start
                );
            }

            // Get current pose
            let current_pose = self.shared_state.pose();

            // Read grid snapshot
            let grid = match self.shared_grid.read() {
                Ok(g) => g.clone(), // Clone to release lock quickly
                Err(e) => {
                    tracing::error!("Failed to read grid: {}", e);
                    std::thread::sleep(loop_interval);
                    continue;
                }
            };

            // Update exploration
            let step = self.explorer.update(&grid, current_pose);

            // Update shared state
            self.shared_state
                .set_velocity(step.velocity.0, step.velocity.1);
            self.shared_state
                .set_frontiers_remaining(step.frontiers_remaining);

            // Update debug visualization with frontier and waypoint data
            self.update_debug_viz(&step);

            // Check if exploration is complete
            if self.explorer.is_complete() {
                match step.state {
                    ExplorationState::Complete => {
                        tracing::info!("Exploration complete!");
                    }
                    ExplorationState::Failed => {
                        tracing::warn!("Exploration failed: no reachable frontiers");
                    }
                    _ => {}
                }
                self.shared_state.set_velocity(0.0, 0.0);
                self.shared_state.set_exploration_complete();
                break;
            }

            // Log status periodically
            if self.last_status_time.elapsed() >= self.status_interval {
                self.log_status(&step, current_pose);
                self.last_status_time = Instant::now();
            }

            // Maintain target loop rate
            let elapsed = loop_start.elapsed();
            if elapsed < loop_interval {
                std::thread::sleep(loop_interval - elapsed);
            }
        }

        tracing::info!("Exploration thread exited");
    }

    /// Log exploration status.
    fn log_status(&self, step: &crate::exploration::ExplorationStep, pose: vastu_slam::Pose2D) {
        let scans = self.shared_state.scan_count();
        let state_str = match step.state {
            ExplorationState::Idle => "Idle",
            ExplorationState::SelectingFrontier => "Selecting frontier",
            ExplorationState::Planning => "Planning path",
            ExplorationState::Navigating => "Navigating",
            ExplorationState::Replanning => "Replanning",
            ExplorationState::Recovering => "Recovering",
            ExplorationState::EscapingToRetry => "Escaping to retry",
            ExplorationState::ReturningHome => "Returning home",
            ExplorationState::Complete => "Complete",
            ExplorationState::Failed => "Failed",
        };

        tracing::info!(
            "Exploring: state={}, frontiers={}, pose=({:.2}, {:.2}, {:.1}°), scans={}",
            state_str,
            step.frontiers_remaining,
            pose.x,
            pose.y,
            pose.theta.to_degrees(),
            scans
        );

        if let Some(target) = step.target_frontier {
            tracing::debug!("Target frontier: ({:.2}, {:.2})", target.x, target.y);
        }
    }

    /// Update debug visualization data for SVG output.
    fn update_debug_viz(&self, step: &crate::exploration::ExplorationStep) {
        // Record frontier targets when navigating
        if let Some(target) = step.target_frontier
            && let Ok(mut debug_viz) = self.shared_debug_viz.write()
        {
            // Add target frontier if not already recorded
            let target_point = vastu_slam::WorldPoint::new(target.x, target.y);
            if !debug_viz
                .visited_frontiers
                .iter()
                .any(|p| (p.x - target_point.x).abs() < 0.1 && (p.y - target_point.y).abs() < 0.1)
            {
                debug_viz.visited_frontiers.push(target_point);
            }
        }

        // Record waypoints from the current path
        if let Some(path) = self.explorer.current_path()
            && let Ok(mut debug_viz) = self.shared_debug_viz.write()
        {
            for segment in &path.segments {
                let start = segment.start_point();
                let end = segment.end_point();

                // Add start point if not already recorded (avoiding duplicates)
                let start_wp = vastu_slam::WorldPoint::new(start.x, start.y);
                if !debug_viz
                    .waypoints
                    .iter()
                    .any(|p| (p.x - start_wp.x).abs() < 0.05 && (p.y - start_wp.y).abs() < 0.05)
                {
                    debug_viz.waypoints.push(start_wp);
                }

                // Add end point
                let end_wp = vastu_slam::WorldPoint::new(end.x, end.y);
                if !debug_viz
                    .waypoints
                    .iter()
                    .any(|p| (p.x - end_wp.x).abs() < 0.05 && (p.y - end_wp.y).abs() < 0.05)
                {
                    debug_viz.waypoints.push(end_wp);
                }
            }
        }
    }
}
