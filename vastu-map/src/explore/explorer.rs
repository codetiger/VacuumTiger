//! VastuExplorer - High-level autonomous exploration.

use std::time::Instant;

use crate::OccupancyGridMap;
use crate::core::Pose2D;
use crate::exploration::{ExplorationCommand, ExplorationController, ExplorationProgress};
use crate::slam::CorrelativeMatcher;
use crate::slam::loop_closure::{LoopClosureDetector, PoseGraph};

use super::config::ExplorerConfig;
use super::source::SensorSource;

/// Result of a completed exploration.
pub enum ExploreResult {
    /// Exploration completed successfully - all frontiers explored.
    Complete(OccupancyGridMap),
    /// Exploration failed or was interrupted.
    Failed {
        /// Partial map collected before failure.
        map: OccupancyGridMap,
        /// Reason for failure.
        reason: String,
    },
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
        /// Reason for failure.
        reason: String,
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

        Self {
            config,
            map,
            controller,
            matcher,
            loop_detector,
            pose_graph,
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
    /// `ExploreResult::Failed` with a partial map and reason.
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
                    reason: "Robot connection lost".to_string(),
                };
            }

            // Check battery
            if let Some(battery) = source.battery_percent()
                && battery < self.config.min_battery_percent
            {
                source.stop();
                return ExploreResult::Failed {
                    map: self.map,
                    reason: format!("Battery low: {}%", battery),
                };
            }

            // Check time limit
            if self.config.max_time_secs > 0.0 && self.elapsed_secs() > self.config.max_time_secs {
                source.stop();
                return ExploreResult::Failed {
                    map: self.map,
                    reason: "Time limit exceeded".to_string(),
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
                ExploreStatus::Failed { reason } => {
                    source.stop();
                    return ExploreResult::Failed {
                        map: self.map,
                        reason,
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

        // Process lidar scan if available
        if let Some(scan) = source.get_lidar_scan() {
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
                .handle_event(crate::exploration::ExplorationEvent::ObstacleDetected);
        }
        if source.is_bumper_triggered() {
            self.controller
                .handle_event(crate::exploration::ExplorationEvent::ObstacleDetected);
        }

        // Update exploration controller
        let command = self.controller.update(self.last_pose, self.map.storage());

        // Execute command
        match command {
            ExplorationCommand::MoveTo { target, max_speed } => {
                let (linear, angular) =
                    self.compute_velocity(self.last_pose, target.x, target.y, max_speed);
                source.send_velocity(linear, angular);
            }
            ExplorationCommand::Rotate {
                target_heading,
                max_angular_speed,
            } => {
                let angular = self.compute_angular_velocity(
                    self.last_pose.theta,
                    target_heading,
                    max_angular_speed,
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
                return ExploreStatus::Failed { reason };
            }
            ExplorationCommand::None => {
                // Continue with current velocity or stop
            }
        }

        ExploreStatus::InProgress {
            progress: self.controller.progress(),
        }
    }

    /// Compute velocity command to move towards target.
    fn compute_velocity(
        &self,
        pose: Pose2D,
        target_x: f32,
        target_y: f32,
        max_speed: f32,
    ) -> (f32, f32) {
        let dx = target_x - pose.x;
        let dy = target_y - pose.y;
        let distance = (dx * dx + dy * dy).sqrt();

        if distance < 0.05 {
            return (0.0, 0.0);
        }

        let target_angle = dy.atan2(dx);
        let angle_error = normalize_angle(target_angle - pose.theta);

        // Turn towards target first if angle is large
        if angle_error.abs() > 0.3 {
            let angular = angle_error.signum() * 2.0f32.min(angle_error.abs() * 2.0);
            return (0.0, angular);
        }

        // Move forward with proportional angular correction
        let linear = max_speed.min(distance);
        let angular = (angle_error * 2.0).clamp(-2.0, 2.0);

        (linear, angular)
    }

    /// Compute angular velocity to rotate to target heading.
    fn compute_angular_velocity(&self, current: f32, target: f32, max_speed: f32) -> f32 {
        let error = normalize_angle(target - current);
        if error.abs() < 0.05 {
            0.0
        } else {
            (error * 2.0).clamp(-max_speed, max_speed)
        }
    }

    /// Reset explorer for a new exploration run.
    pub fn reset(&mut self) {
        let map_config = self.config.to_map_config();
        self.map = OccupancyGridMap::new(map_config);
        self.controller = ExplorationController::new(self.config.exploration.clone());
        self.loop_detector.reset();
        self.pose_graph.reset();
        self.last_pose = Pose2D::default();
        self.last_odom_pose = None;
        self.start_time = None;
        self.started = false;
    }
}

/// Normalize angle to [-π, π).
pub fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % std::f32::consts::TAU;
    if a > std::f32::consts::PI {
        a -= std::f32::consts::TAU;
    } else if a < -std::f32::consts::PI {
        a += std::f32::consts::TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

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
