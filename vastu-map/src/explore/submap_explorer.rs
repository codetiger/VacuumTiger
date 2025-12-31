//! VastuSubmapExplorer - Submap-based autonomous exploration.
//!
//! This explorer uses the submap architecture for proper loop closure correction.
//! Unlike `VastuExplorer` which uses a single global map, this version:
//! - Stores raw scans in submaps for reversible map updates
//! - Uses submap-level pose graph optimization
//! - Can correctly apply loop closure corrections without ghost walls

use std::time::Instant;

use serde::{Deserialize, Serialize};

use super::controller::{
    ExplorationCommand, ExplorationController, ExplorationEvent, ExplorationProgress,
};
use super::error::ExplorationError;
use super::motion_filter::MotionFilter;
use super::velocity::{
    VelocityConfig, compute_angular_velocity_to_heading, compute_velocity_to_target,
};
use crate::core::{LidarScan, Pose2D};
use crate::grid::GridStorage;
use crate::slam::CorrelativeMatcherConfig;
use crate::submap::{
    MultiSubmapMatchConfig, SubmapConfig, SubmapLoopClosure, SubmapManager, SubmapPoseGraph,
};

use super::config::ExplorerConfig;
use super::explorer::{ExploreResult, ExploreStatus};
use super::source::SensorSource;

/// Configuration specific to submap-based exploration.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SubmapExplorerConfig {
    /// Base explorer configuration.
    #[serde(default)]
    pub base: ExplorerConfig,
    /// Submap configuration.
    #[serde(default)]
    pub submap: SubmapConfig,
    /// Multi-submap matching configuration.
    #[serde(default)]
    pub multi_match: MultiSubmapMatchConfig,
    /// Minimum score to accept a loop closure.
    #[serde(default = "default_loop_closure_min_score")]
    pub loop_closure_min_score: f32,
    /// Minimum submap gap for loop closure (submaps between).
    #[serde(default = "default_loop_closure_min_gap")]
    pub loop_closure_min_gap: usize,
}

fn default_loop_closure_min_score() -> f32 {
    0.7
}

fn default_loop_closure_min_gap() -> usize {
    3
}

impl Default for SubmapExplorerConfig {
    fn default() -> Self {
        Self {
            base: ExplorerConfig::default(),
            submap: SubmapConfig::default(),
            multi_match: MultiSubmapMatchConfig::default(),
            loop_closure_min_score: default_loop_closure_min_score(),
            loop_closure_min_gap: default_loop_closure_min_gap(),
        }
    }
}

impl SubmapExplorerConfig {
    /// Create a fast configuration for quick testing.
    pub fn fast() -> Self {
        Self {
            base: ExplorerConfig::fast(),
            submap: SubmapConfig {
                scans_per_submap: 30,
                overlap_scans: 5,
                ..Default::default()
            },
            ..Default::default()
        }
    }
}

/// High-level submap-based exploration controller.
///
/// This explorer uses the submap architecture for improved SLAM accuracy,
/// especially during loop closure. When a loop is detected, submap origins
/// are corrected and grids are regenerated from stored scans.
///
/// # Key Differences from VastuExplorer
///
/// 1. **Submap Management**: Scans are organized into locally-consistent submaps
/// 2. **Reversible Updates**: Raw scans are stored for regeneration after optimization
/// 3. **Submap-level Pose Graph**: Fewer nodes (one per submap) for efficient optimization
/// 4. **Multi-submap Matching**: Can match against active + overlapping finalized submaps
pub struct VastuSubmapExplorer {
    /// Configuration.
    config: SubmapExplorerConfig,
    /// Submap manager (replaces OccupancyGridMap).
    manager: SubmapManager,
    /// Submap-level pose graph.
    pose_graph: SubmapPoseGraph,
    /// Exploration controller.
    controller: ExplorationController,
    /// Matcher configuration.
    matcher_config: CorrelativeMatcherConfig,
    /// Motion filter for gating scan processing.
    motion_filter: MotionFilter,
    /// Velocity computation configuration.
    velocity_config: VelocityConfig,
    /// Last corrected world pose.
    last_pose: Pose2D,
    /// Last odometry pose (for delta calculation).
    last_odom_pose: Option<Pose2D>,
    /// Last submap origin pose (for computing delta between submaps).
    last_submap_origin: Option<Pose2D>,
    /// Scan counter for timestamps.
    scan_count: u64,
    /// Last submap count (to detect new submap creation).
    last_submap_count: usize,
    /// Start time.
    start_time: Option<Instant>,
    /// Whether exploration has started.
    started: bool,
}

impl VastuSubmapExplorer {
    /// Create a new submap explorer with the given configuration.
    pub fn new(config: SubmapExplorerConfig) -> Self {
        let map_config = config.base.to_map_config();
        let manager = SubmapManager::new(config.submap.clone(), &map_config);
        let pose_graph = SubmapPoseGraph::new(Default::default());
        let controller = ExplorationController::new(config.base.exploration.clone());
        let matcher_config = config.base.matching.clone();
        let motion_filter = MotionFilter::new(config.base.motion_filter.clone());
        let velocity_config = VelocityConfig::default();

        Self {
            config,
            manager,
            pose_graph,
            controller,
            matcher_config,
            motion_filter,
            velocity_config,
            last_pose: Pose2D::default(),
            last_odom_pose: None,
            last_submap_origin: None,
            scan_count: 0,
            last_submap_count: 0,
            start_time: None,
            started: false,
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(SubmapExplorerConfig::default())
    }

    /// Get a reference to the current global grid for visualization/planning.
    ///
    /// This lazily regenerates the global grid if needed.
    pub fn global_grid(&mut self) -> &GridStorage {
        self.manager.global_grid()
    }

    /// Get the submap manager for inspection.
    pub fn manager(&self) -> &SubmapManager {
        &self.manager
    }

    /// Get exploration progress.
    pub fn progress(&self) -> ExplorationProgress {
        self.controller.progress()
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

    /// Get the number of submaps created.
    pub fn submap_count(&self) -> usize {
        self.manager.submaps().len()
    }

    /// Get the number of scans processed.
    pub fn scan_count(&self) -> u64 {
        self.scan_count
    }

    /// Has a loop closure been detected and applied?
    pub fn has_loop_closures(&self) -> bool {
        self.pose_graph.has_loops()
    }

    /// Run exploration to completion (blocking).
    pub fn explore<S: SensorSource>(mut self, source: &mut S) -> ExploreResult {
        self.controller.start();
        self.start_time = Some(Instant::now());
        self.started = true;

        let step_duration =
            std::time::Duration::from_secs_f32(1.0 / self.config.base.update_rate_hz);

        loop {
            let step_start = Instant::now();

            // Check connection
            if !source.is_connected() {
                source.stop();
                return ExploreResult::Failed {
                    map: self.build_occupancy_grid_map(),
                    error: ExplorationError::ConnectionLost,
                };
            }

            // Check battery
            if let Some(battery) = source.battery_percent()
                && battery < self.config.base.min_battery_percent
            {
                source.stop();
                return ExploreResult::Failed {
                    map: self.build_occupancy_grid_map(),
                    error: ExplorationError::LowBattery {
                        percent: battery,
                        threshold: self.config.base.min_battery_percent,
                    },
                };
            }

            // Check time limit
            if self.config.base.max_time_secs > 0.0
                && self.elapsed_secs() > self.config.base.max_time_secs
            {
                source.stop();
                return ExploreResult::Failed {
                    map: self.build_occupancy_grid_map(),
                    error: ExplorationError::TimeoutExceeded {
                        elapsed_secs: self.elapsed_secs(),
                        limit_secs: self.config.base.max_time_secs,
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
                    return ExploreResult::Complete(self.build_occupancy_grid_map());
                }
                ExploreStatus::Failed { error } => {
                    source.stop();
                    return ExploreResult::Failed {
                        map: self.build_occupancy_grid_map(),
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
            if self.motion_filter.should_process(odom_pose) {
                let corrected_pose = self.process_scan(&scan, odom_pose);
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
            // No scan - update from odometry delta
            self.last_pose = Pose2D::new(
                self.last_pose.x + delta.x,
                self.last_pose.y + delta.y,
                self.last_pose.theta + delta.theta,
            );
        }

        // Handle cliff/bumper events
        if source.is_cliff_detected() || source.is_bumper_triggered() {
            self.controller
                .handle_event(ExplorationEvent::ObstacleDetected);
        }

        // Get current global grid for planning
        let global_grid = self.manager.global_grid();

        // Update exploration controller
        let command = self.controller.update(self.last_pose, global_grid);

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

    /// Process a lidar scan with submap-based SLAM.
    fn process_scan(&mut self, scan: &LidarScan, odom_pose: Pose2D) -> Pose2D {
        // Match against active + overlapping submaps
        let match_result = self.manager.match_scan_multi(
            scan,
            odom_pose,
            &self.matcher_config,
            &self.config.multi_match,
        );

        // Use matched pose if converged
        let corrected_pose = if match_result.converged() {
            match_result.world_pose()
        } else {
            odom_pose
        };

        // Track submap count before insert
        let submaps_before = self.manager.submaps().len();

        // Insert scan into submap manager
        self.scan_count += 1;
        let _insert_result = self
            .manager
            .insert_scan(scan, corrected_pose, self.scan_count);

        // Check if new submap was created
        let submaps_after = self.manager.submaps().len();
        if submaps_after > submaps_before {
            // New submap created - add to pose graph
            if let Some(active) = self.manager.active_submap() {
                // Compute delta from previous submap origin
                let odom_delta = self.last_submap_origin.map(|prev| {
                    Pose2D::new(
                        active.origin.x - prev.x,
                        active.origin.y - prev.y,
                        active.origin.theta - prev.theta,
                    )
                });

                self.pose_graph
                    .add_submap(active.id, active.origin, odom_delta);
                self.last_submap_origin = Some(active.origin);
            }

            // Check for loop closure candidates
            self.check_loop_closure(scan, corrected_pose);

            self.last_submap_count = submaps_after;
        }

        corrected_pose
    }

    /// Check for loop closure against finalized submaps.
    fn check_loop_closure(&mut self, scan: &LidarScan, current_pose: Pose2D) {
        use crate::submap::SubmapId;

        // Collect candidate info to avoid borrow issues
        let candidates: Vec<(SubmapId, Pose2D)> = self
            .manager
            .loop_closure_candidates(current_pose)
            .iter()
            .map(|s| (s.id, s.origin))
            .collect();

        for (candidate_id, candidate_origin) in candidates {
            // Skip if already have a loop edge to this submap
            if self
                .pose_graph
                .loop_edges()
                .iter()
                .any(|e| e.from_id == candidate_id || e.to_id == candidate_id)
            {
                continue;
            }

            // Get the candidate submap's grid for matching
            let Some(candidate) = self.manager.get_submap(candidate_id) else {
                continue;
            };

            // Match against the candidate submap
            let matcher = crate::slam::CorrelativeMatcher::new(self.matcher_config.clone());
            let local_pose = candidate_origin.inverse().compose(&current_pose);
            let match_result = matcher.match_scan(scan, local_pose, &candidate.grid);

            if match_result.converged && match_result.score >= self.config.loop_closure_min_score {
                // Good match - add loop closure constraint
                let matched_world_pose = candidate_origin.compose(&match_result.pose);

                if let Some(active) = self.manager.active_submap() {
                    // Compute relative pose between submaps
                    let relative_pose = candidate_origin.inverse().compose(&matched_world_pose);

                    let closure = SubmapLoopClosure {
                        from_submap: active.id,
                        to_submap: candidate_id,
                        relative_pose,
                        confidence: match_result.score,
                    };

                    self.pose_graph.add_loop_closure(closure);

                    // Optimize pose graph
                    let (_, corrections) = self.pose_graph.optimize();

                    // Apply corrections to submaps
                    if !corrections.is_empty() {
                        self.manager.apply_corrections(&corrections);
                    }
                }
            }
        }
    }

    /// Convert to OccupancyGridMap for compatibility with ExploreResult.
    fn build_occupancy_grid_map(&mut self) -> crate::OccupancyGridMap {
        let map_config = self.config.base.to_map_config();
        let mut map = crate::OccupancyGridMap::new(map_config);

        // Copy global grid contents
        let global = self.manager.global_grid();
        let dest_storage = map.storage_mut();

        // Copy each cell from global grid to map storage
        for (coord, cell) in global.iter() {
            if let Some(dest_cell) = dest_storage.get_mut(coord) {
                *dest_cell.cell_type = cell.cell_type as u8;
                *dest_cell.observation_count = cell.observation_count;
                *dest_cell.swept = if cell.swept { 1 } else { 0 };
            }
        }

        map
    }

    /// Reset explorer for a new exploration run.
    pub fn reset(&mut self) {
        let map_config = self.config.base.to_map_config();
        self.manager = SubmapManager::new(self.config.submap.clone(), &map_config);
        self.pose_graph = SubmapPoseGraph::new(Default::default());
        self.controller = ExplorationController::new(self.config.base.exploration.clone());
        self.motion_filter.reset();
        self.last_pose = Pose2D::default();
        self.last_odom_pose = None;
        self.last_submap_origin = None;
        self.scan_count = 0;
        self.last_submap_count = 0;
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

    #[test]
    fn test_submap_explorer_creation() {
        let explorer = VastuSubmapExplorer::with_defaults();
        assert!(!explorer.started);
        assert_eq!(explorer.scan_count(), 0);
        assert_eq!(explorer.submap_count(), 0);
    }

    #[test]
    fn test_submap_explorer_config() {
        let config = SubmapExplorerConfig::fast();
        assert_eq!(config.submap.scans_per_submap, 30);
    }
}
