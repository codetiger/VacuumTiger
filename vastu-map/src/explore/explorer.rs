//! VastuExplorer - High-level autonomous exploration.

use std::time::Instant;

use super::controller::{
    ExplorationCommand, ExplorationController, ExplorationEvent, ExplorationProgress,
};
use super::error::ExplorationError;
use super::velocity::{
    VelocityConfig, compute_angular_velocity_to_heading, compute_velocity_to_target,
};
use crate::OccupancyGridMap;
use crate::core::Pose2D;
use crate::slam::CorrelativeMatcher;
use crate::slam::loop_closure::{LoopClosureDetector, PoseGraph};

use super::config::ExplorerConfig;
use super::motion_filter::MotionFilter;
use super::source::SensorSource;

/// Result of a completed exploration.
pub enum ExploreResult {
    /// Exploration completed successfully - all frontiers explored.
    Complete(OccupancyGridMap),
    /// Exploration failed or was interrupted.
    Failed {
        /// Partial map collected before failure.
        map: OccupancyGridMap,
        /// Error that caused the failure.
        error: ExplorationError,
    },
}

impl ExploreResult {
    /// Check if exploration completed successfully.
    pub fn is_complete(&self) -> bool {
        matches!(self, Self::Complete(_))
    }

    /// Get the map regardless of success/failure.
    pub fn into_map(self) -> OccupancyGridMap {
        match self {
            Self::Complete(map) => map,
            Self::Failed { map, .. } => map,
        }
    }

    /// Get the error if exploration failed.
    pub fn error(&self) -> Option<&ExplorationError> {
        match self {
            Self::Complete(_) => None,
            Self::Failed { error, .. } => Some(error),
        }
    }
}

/// Status returned from a single exploration step.
#[derive(Clone, Debug)]
pub enum ExploreStatus {
    /// Still exploring.
    InProgress {
        /// Current exploration progress.
        progress: ExplorationProgress,
    },
    /// Exploration complete.
    Complete,
    /// Exploration failed.
    Failed {
        /// Error that caused the failure.
        error: ExplorationError,
    },
}

/// High-level autonomous exploration controller.
///
/// Combines all components needed for autonomous map exploration:
/// - Occupancy grid mapping
/// - Scan-to-map matching for drift correction
/// - Loop closure detection
/// - Pose graph optimization
/// - Frontier-based exploration
///
/// # Example
///
/// ```ignore
/// let mut explorer = VastuExplorer::new(ExplorerConfig::default());
/// let mut robot = MyRobotSource::new();
///
/// // Run exploration to completion
/// match explorer.explore(&mut robot) {
///     ExploreResult::Complete(map) => { /* save map */ }
///     ExploreResult::Failed { map, reason } => { /* handle error */ }
/// }
/// ```
pub struct VastuExplorer {
    /// Configuration.
    config: ExplorerConfig,
    /// Occupancy grid map.
    map: OccupancyGridMap,
    /// Exploration controller.
    controller: ExplorationController,
    /// Scan-to-map matcher.
    matcher: CorrelativeMatcher,
    /// Loop closure detector.
    loop_detector: LoopClosureDetector,
    /// Pose graph for optimization.
    pose_graph: PoseGraph,
    /// Motion filter for gating scan processing.
    motion_filter: MotionFilter,
    /// Velocity computation configuration.
    velocity_config: VelocityConfig,
    /// Last corrected pose.
    last_pose: Pose2D,
    /// Last odometry pose (for delta calculation).
    last_odom_pose: Option<Pose2D>,
    /// Start time.
    start_time: Option<Instant>,
    /// Whether exploration has started.
    started: bool,
}

impl VastuExplorer {
    /// Create a new explorer with the given configuration.
    pub fn new(config: ExplorerConfig) -> Self {
        let map_config = config.to_map_config();
        let map = OccupancyGridMap::new(map_config);

        let controller = ExplorationController::new(config.exploration.clone());
        let matcher = CorrelativeMatcher::new(config.matching.clone());
        let loop_detector = LoopClosureDetector::new(config.loop_closure.clone());
        let pose_graph = PoseGraph::new(config.pose_graph.clone());
        let motion_filter = MotionFilter::new(config.motion_filter.clone());
        let velocity_config = VelocityConfig::default();

        Self {
            config,
            map,
            controller,
            matcher,
            loop_detector,
            pose_graph,
            motion_filter,
            velocity_config,
            last_pose: Pose2D::default(),
            last_odom_pose: None,
            start_time: None,
            started: false,
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(ExplorerConfig::default())
    }

    /// Get the current map.
    pub fn map(&self) -> &OccupancyGridMap {
        &self.map
    }

    /// Get exploration progress.
    pub fn progress(&self) -> ExplorationProgress {
        let mut progress = self.controller.progress();
        let map_stats = self.map.coverage_stats();
        progress.frontiers_explored = map_stats.frontier_count;
        progress
    }

    /// Get the current corrected pose.
    pub fn current_pose(&self) -> Pose2D {
        self.last_pose
    }

    /// Get elapsed time since exploration started.
    pub fn elapsed_secs(&self) -> f32 {
        self.start_time
            .map(|t| t.elapsed().as_secs_f32())
            .unwrap_or(0.0)
    }

    /// Run exploration to completion (blocking).
    ///
    /// This method loops until exploration is complete or fails.
    /// It handles all sensor integration, SLAM, and navigation internally.
    ///
    /// # Arguments
    /// * `source` - Sensor source providing lidar, odometry, and accepting commands
    ///
    /// # Returns
    /// `ExploreResult::Complete` with the finished map, or
    /// `ExploreResult::Failed` with a partial map and error.
    pub fn explore<S: SensorSource>(mut self, source: &mut S) -> ExploreResult {
        // Start exploration
        self.controller.start();
        self.start_time = Some(Instant::now());
        self.started = true;

        // Control loop
        let step_duration = std::time::Duration::from_secs_f32(1.0 / self.config.update_rate_hz);

        loop {
            let step_start = Instant::now();

            // Check connection
            if !source.is_connected() {
                source.stop();
                return ExploreResult::Failed {
                    map: self.map,
                    error: ExplorationError::ConnectionLost,
                };
            }

            // Check battery
            if let Some(battery) = source.battery_percent()
                && battery < self.config.min_battery_percent
            {
                source.stop();
                return ExploreResult::Failed {
                    map: self.map,
                    error: ExplorationError::LowBattery {
                        percent: battery,
                        threshold: self.config.min_battery_percent,
                    },
                };
            }

            // Check time limit
            let elapsed = self.elapsed_secs();
            if self.config.max_time_secs > 0.0 && elapsed > self.config.max_time_secs {
                source.stop();
                return ExploreResult::Failed {
                    map: self.map,
                    error: ExplorationError::TimeoutExceeded {
                        elapsed_secs: elapsed,
                        limit_secs: self.config.max_time_secs,
                    },
                };
            }

            // Run one exploration step
            match self.step(source) {
                ExploreStatus::InProgress { .. } => {
                    // Continue
                }
                ExploreStatus::Complete => {
                    source.stop();
                    return ExploreResult::Complete(self.map);
                }
                ExploreStatus::Failed { error } => {
                    source.stop();
                    return ExploreResult::Failed {
                        map: self.map,
                        error,
                    };
                }
            }

            // Rate limiting
            let elapsed = step_start.elapsed();
            if elapsed < step_duration {
                std::thread::sleep(step_duration - elapsed);
            }
        }
    }

    /// Run a single exploration step (non-blocking).
    ///
    /// Use this for custom control loops or when you need to
    /// interleave exploration with other tasks.
    ///
    /// # Arguments
    /// * `source` - Sensor source
    ///
    /// # Returns
    /// Current exploration status.
    pub fn step<S: SensorSource>(&mut self, source: &mut S) -> ExploreStatus {
        // Initialize if needed
        if !self.started {
            self.controller.start();
            self.start_time = Some(Instant::now());
            self.started = true;
        }

        // Poll sensors
        source.poll();

        // Get current odometry pose
        let odom_pose = source.get_pose();

        // Calculate odometry delta
        let odom_delta = self.last_odom_pose.map(|last| {
            Pose2D::new(
                odom_pose.x - last.x,
                odom_pose.y - last.y,
                odom_pose.theta - last.theta,
            )
        });
        self.last_odom_pose = Some(odom_pose);

        // Process lidar scan if available and motion filter allows
        if let Some(scan) = source.get_lidar_scan() {
            // Check motion filter - only process if sufficient motion occurred
            if self.motion_filter.should_process(odom_pose) {
                // Scan matching
                let match_result = self
                    .matcher
                    .match_scan(&scan, odom_pose, self.map.storage());

                // Use matched pose if good, else use odometry
                let corrected_pose = if match_result.converged {
                    match_result.pose
                } else {
                    odom_pose
                };

                // Integrate scan into map
                self.map.observe_lidar(&scan, corrected_pose);

                // Update pose graph
                self.pose_graph.add_pose(corrected_pose, odom_delta);

                // Check for loop closure
                if let Some(closure) = self.loop_detector.add_scan(&scan, corrected_pose) {
                    self.pose_graph.add_loop_closure(closure);
                    // Optimize when loop detected
                    self.pose_graph.optimize();
                }

                self.last_pose = corrected_pose;
            } else if let Some(delta) = odom_delta {
                // Scan filtered out - update pose from odometry delta only
                self.last_pose = Pose2D::new(
                    self.last_pose.x + delta.x,
                    self.last_pose.y + delta.y,
                    self.last_pose.theta + delta.theta,
                );
            }
        } else if let Some(delta) = odom_delta {
            // No scan available - update pose from odometry delta
            // This prevents the controller from using stale positions
            self.last_pose = Pose2D::new(
                self.last_pose.x + delta.x,
                self.last_pose.y + delta.y,
                self.last_pose.theta + delta.theta,
            );
        }

        // Handle cliff/bumper events
        if source.is_cliff_detected() {
            self.controller
                .handle_event(ExplorationEvent::ObstacleDetected);
        }
        if source.is_bumper_triggered() {
            self.controller
                .handle_event(ExplorationEvent::ObstacleDetected);
        }

        // Update exploration controller
        let command = self.controller.update(self.last_pose, self.map.storage());

        // Execute command
        match command {
            ExplorationCommand::MoveTo { target, max_speed } => {
                let (linear, angular) = compute_velocity_to_target(
                    self.last_pose,
                    target,
                    max_speed,
                    &self.velocity_config,
                );
                source.send_velocity(linear, angular);
            }
            ExplorationCommand::Rotate {
                target_heading,
                max_angular_speed,
            } => {
                let angular = compute_angular_velocity_to_heading(
                    self.last_pose.theta,
                    target_heading,
                    max_angular_speed,
                    &self.velocity_config,
                );
                source.send_velocity(0.0, angular);
            }
            ExplorationCommand::Stop => {
                source.stop();
            }
            ExplorationCommand::ExplorationComplete => {
                source.stop();
                return ExploreStatus::Complete;
            }
            ExplorationCommand::ExplorationFailed { reason } => {
                source.stop();
                return ExploreStatus::Failed {
                    error: ExplorationError::from_message(reason),
                };
            }
            ExplorationCommand::None => {
                // Continue with current velocity or stop
            }
        }

        ExploreStatus::InProgress {
            progress: self.controller.progress(),
        }
    }

    /// Reset explorer for a new exploration run.
    pub fn reset(&mut self) {
        let map_config = self.config.to_map_config();
        self.map = OccupancyGridMap::new(map_config);
        self.controller = ExplorationController::new(self.config.exploration.clone());
        self.loop_detector.reset();
        self.pose_graph.reset();
        self.motion_filter.reset();
        self.last_pose = Pose2D::default();
        self.last_odom_pose = None;
        self.start_time = None;
        self.started = false;
    }

    /// Get motion filter statistics.
    ///
    /// Returns (passed_count, filtered_count, efficiency).
    pub fn motion_filter_stats(&self) -> (u64, u64, f32) {
        (
            self.motion_filter.passed_count(),
            self.motion_filter.filtered_count(),
            self.motion_filter.filter_efficiency(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::normalize_angle;

    #[test]
    fn test_explorer_creation() {
        let explorer = VastuExplorer::with_defaults();
        assert!(!explorer.started);
        assert_eq!(explorer.elapsed_secs(), 0.0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(std::f32::consts::PI * 2.0) - 0.0).abs() < 0.001);
    }
}
