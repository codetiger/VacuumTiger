//! Autonomous mapping feature using navigation stack.
//!
//! MappingFeature implements autonomous map building that interacts with
//! robot navigation **exclusively through the target stack API** (`push_target`,
//! `clear_targets`). It does NOT directly control robot motion.
//!
//! Note: Some config fields and stats are defined for future use.
//!
//! # State Machine
//!
//! ```text
//! StartMapping
//!     │
//!     ▼
//! INITIALIZING ──► ANALYZING_CLEARANCE
//!                         │
//!           ┌─────────────┼─────────────┐
//!           │             │             │
//!      in dock      confined      clearance OK
//!           │             │             │
//!           ▼             ▼             │
//!     ESCAPING ◄──────────┘             │
//!           │                           │
//!           └───────► SCANNING ◄────────┘
//!                         │
//!                         ▼
//!                FINDING_FRONTIER
//!                         │
//!           ┌─────────────┼─────────────┐
//!           │             │             │
//!     frontier found   no frontiers     │
//!           │             │             │
//!           ▼             ▼             │
//!     NAVIGATING      COMPLETE          │
//!           │                           │
//!           └───────────────────────────┘
//! ```

use crate::core::types::{LaserScan, Point2D, Pose2D};
use crate::navigation::{NavTarget, NavTargetSource, NavTargetType, NavigationState};
use crate::state::{CurrentMapData, MappingProgressState, MappingProgressStateEnum};
use std::time::Instant;

use super::clearance::{ClearanceAnalysis, DockConfidence, MIN_ROTATION_CLEARANCE};
use super::frontier::{find_frontiers, select_best_frontier};

/// Configuration for the mapping feature.
#[derive(Debug, Clone)]
pub struct MappingConfig {
    /// Minimum clearance needed for 360° rotation (meters).
    /// Should be slightly larger than robot radius.
    pub min_rotation_clearance: f32,

    /// Maximum escape attempts before failing.
    pub max_escape_attempts: u8,

    /// Minimum escape distance per attempt (meters).
    pub min_escape_distance: f32,

    /// Back distance threshold to detect dock (meters).
    pub dock_back_threshold: f32,

    /// Side distance threshold to detect dock (meters).
    pub dock_side_threshold: f32,

    /// Front distance threshold for dock (must be open).
    pub dock_front_min: f32,

    /// Minimum frontier cluster size (cells).
    pub min_frontier_size: usize,

    /// Minimum distance to frontier (meters).
    pub min_frontier_distance: f32,

    /// Distance to consider frontiers as same cluster (meters).
    pub frontier_cluster_distance: f32,

    /// Maximum blocked frontiers before giving up.
    pub max_blocked_frontiers: usize,
}

impl Default for MappingConfig {
    fn default() -> Self {
        Self {
            min_rotation_clearance: MIN_ROTATION_CLEARANCE,
            max_escape_attempts: 3,
            min_escape_distance: 0.15,
            dock_back_threshold: 0.15,
            dock_side_threshold: 0.20,
            dock_front_min: 0.40,
            min_frontier_size: 3,
            min_frontier_distance: 0.30,
            frontier_cluster_distance: 0.30,
            max_blocked_frontiers: 10,
        }
    }
}

/// State machine for mapping operation.
#[derive(Debug, Clone, PartialEq)]
pub enum MappingState {
    /// Not mapping.
    Idle,

    /// Initializing hardware (lidar, motors).
    Initializing,

    /// Analyzing clearance and detecting dock.
    AnalyzingClearance,

    /// Escaping confined space (dock, corner, etc.).
    EscapingConfined {
        /// Number of escape attempts made.
        attempts: u8,
    },

    /// Rotating 360° to scan surroundings.
    Scanning {
        /// Number of 90° segments completed (0-4).
        segments_completed: u8,
    },

    /// Analyzing map for next frontier.
    FindingFrontier,

    /// Navigating to frontier (waiting for Navigator).
    NavigatingToFrontier {
        /// Target ID being navigated to.
        target_id: u32,
    },

    /// Mapping complete - all areas explored.
    Complete,

    /// Mapping failed.
    Failed {
        /// Reason for failure.
        reason: String,
    },
}

impl MappingState {
    /// Convert to string for proto serialization.
    pub fn as_str(&self) -> &'static str {
        match self {
            MappingState::Idle => "IDLE",
            MappingState::Initializing => "INITIALIZING",
            MappingState::AnalyzingClearance => "ANALYZING_CLEARANCE",
            MappingState::EscapingConfined { .. } => "ESCAPING_CONFINED",
            MappingState::Scanning { .. } => "SCANNING",
            MappingState::FindingFrontier => "FINDING_FRONTIER",
            MappingState::NavigatingToFrontier { .. } => "NAVIGATING",
            MappingState::Complete => "COMPLETE",
            MappingState::Failed { .. } => "FAILED",
        }
    }
}

/// Statistics for mapping operation.
#[derive(Debug, Clone, Default)]
pub struct MappingStats {
    /// Number of frontiers visited.
    pub frontiers_visited: u32,

    /// Number of frontiers that failed (blocked).
    pub frontiers_blocked: u32,

    /// Number of 360° scans performed.
    pub scans_completed: u32,

    /// Total distance traveled (meters).
    pub distance_traveled: f32,

    /// Mapping start time.
    pub start_time: Option<Instant>,

    /// Mapping end time.
    pub end_time: Option<Instant>,
}

impl MappingStats {
    /// Get elapsed time in seconds.
    pub fn elapsed_secs(&self) -> f32 {
        match self.start_time {
            Some(start) => {
                let end = self.end_time.unwrap_or_else(Instant::now);
                end.duration_since(start).as_secs_f32()
            }
            None => 0.0,
        }
    }
}

/// Autonomous mapping feature.
///
/// Interacts with navigation ONLY through target stack API.
pub struct MappingFeature {
    /// Current mapping state.
    state: MappingState,

    /// Configuration.
    config: MappingConfig,

    /// Blocked frontier positions (failed to reach).
    blocked_frontiers: Vec<Point2D>,

    /// Starting pose (for potential return after completion).
    start_pose: Option<Pose2D>,

    /// Statistics.
    stats: MappingStats,

    /// Last known clearance analysis (for state transitions).
    last_clearance: Option<ClearanceAnalysis>,

    /// Expected scan segment headings for verification.
    scan_segment_headings: Vec<f32>,

    /// Current status message for UI.
    status_message: String,
}

impl MappingFeature {
    /// Create new mapping feature with default config.
    pub fn new() -> Self {
        Self::with_config(MappingConfig::default())
    }

    /// Create new mapping feature with config.
    pub fn with_config(config: MappingConfig) -> Self {
        Self {
            state: MappingState::Idle,
            config,
            blocked_frontiers: Vec::new(),
            start_pose: None,
            stats: MappingStats::default(),
            last_clearance: None,
            scan_segment_headings: Vec::new(),
            status_message: String::new(),
        }
    }

    /// Start mapping operation.
    pub fn start(&mut self, nav_state: &mut NavigationState, pose: &Pose2D) {
        log::info!("Mapping started at ({:.2}, {:.2})", pose.x, pose.y);

        self.state = MappingState::Initializing;
        self.start_pose = Some(*pose);
        self.blocked_frontiers.clear();
        self.last_clearance = None;
        self.scan_segment_headings.clear();
        self.stats = MappingStats {
            start_time: Some(Instant::now()),
            ..Default::default()
        };

        nav_state.active_feature = Some(NavTargetSource::Mapping);
        nav_state.set_status("Mapping: Initializing");
    }

    /// Stop mapping operation.
    pub fn stop(&mut self, nav_state: &mut NavigationState) {
        log::info!("Mapping stopped. Stats: {:?}", self.stats);

        nav_state.clear_targets();
        nav_state.active_feature = None;
        nav_state.set_status("Mapping stopped");

        self.stats.end_time = Some(Instant::now());
        self.state = MappingState::Idle;
    }

    /// Get current state.
    pub fn state(&self) -> &MappingState {
        &self.state
    }

    /// Get statistics.
    pub fn stats(&self) -> &MappingStats {
        &self.stats
    }

    /// Check if mapping is active.
    pub fn is_active(&self) -> bool {
        !matches!(
            self.state,
            MappingState::Idle | MappingState::Complete | MappingState::Failed { .. }
        )
    }

    /// Called when hardware is initialized and first lidar scan is available.
    ///
    /// Transitions from Initializing to AnalyzingClearance.
    pub fn on_hardware_ready(&mut self, nav_state: &mut NavigationState) {
        if matches!(self.state, MappingState::Initializing) {
            log::info!("Mapping: Hardware ready, analyzing clearance");
            self.state = MappingState::AnalyzingClearance;
            nav_state.set_status("Mapping: Analyzing surroundings");
        }
    }

    /// Called with lidar scan to analyze clearance and detect dock.
    ///
    /// Should be called when state is AnalyzingClearance or after escape target reached.
    pub fn on_lidar_scan(
        &mut self,
        nav_state: &mut NavigationState,
        scan: &LaserScan,
        pose: &Pose2D,
        is_charging: bool,
    ) {
        let clearance = ClearanceAnalysis::from_laser_scan(scan);
        self.last_clearance = Some(clearance.clone());

        match &self.state {
            MappingState::AnalyzingClearance => {
                self.handle_clearance_analysis(nav_state, &clearance, pose, is_charging);
            }
            MappingState::EscapingConfined { attempts } => {
                // After escape, re-check clearance
                let attempts = *attempts;
                self.handle_escape_complete(nav_state, &clearance, pose, is_charging, attempts);
            }
            _ => {}
        }
    }

    /// Handle clearance analysis results.
    fn handle_clearance_analysis(
        &mut self,
        nav_state: &mut NavigationState,
        clearance: &ClearanceAnalysis,
        pose: &Pose2D,
        is_charging: bool,
    ) {
        let dock_detection = clearance.detect_dock(is_charging);

        match dock_detection.confidence {
            DockConfidence::Certain => {
                if is_charging {
                    log::info!("Robot is charging in dock, will exit first");
                    nav_state.set_status("Mapping: Exiting dock (was charging)");
                } else {
                    log::info!("Robot detected in dock by lidar U-pattern");
                    nav_state.set_status("Mapping: Exiting dock");
                }
                self.push_escape_target(nav_state, clearance, pose);
                self.state = MappingState::EscapingConfined { attempts: 1 };
            }

            DockConfidence::Likely => {
                log::info!("Robot likely in confined space");
                if clearance.can_rotate_safely() {
                    // Enough clearance despite pattern - can scan
                    self.push_scan_targets(nav_state, pose);
                    self.state = MappingState::Scanning {
                        segments_completed: 0,
                    };
                    nav_state.set_status("Mapping: Scanning surroundings");
                } else {
                    self.push_escape_target(nav_state, clearance, pose);
                    self.state = MappingState::EscapingConfined { attempts: 1 };
                    nav_state.set_status("Mapping: Moving to open area");
                }
            }

            DockConfidence::Unlikely => {
                if clearance.can_rotate_safely() {
                    log::info!(
                        "Clearance OK ({:.2}m min), starting scan",
                        clearance.overall_min
                    );
                    self.push_scan_targets(nav_state, pose);
                    self.state = MappingState::Scanning {
                        segments_completed: 0,
                    };
                    nav_state.set_status("Mapping: Scanning surroundings");
                } else {
                    log::info!(
                        "Insufficient clearance ({:.2}m < {:.2}m), escaping",
                        clearance.overall_min,
                        self.config.min_rotation_clearance
                    );
                    self.push_escape_target(nav_state, clearance, pose);
                    self.state = MappingState::EscapingConfined { attempts: 1 };
                    nav_state.set_status("Mapping: Finding open space");
                }
            }
        }
    }

    /// Handle escape target completion.
    fn handle_escape_complete(
        &mut self,
        nav_state: &mut NavigationState,
        clearance: &ClearanceAnalysis,
        pose: &Pose2D,
        is_charging: bool,
        attempts: u8,
    ) {
        // Should not be charging anymore after moving
        if is_charging {
            log::warn!("Still charging after escape - unexpected");
        }

        if clearance.can_rotate_safely() {
            // Success - now have clearance for rotation
            log::info!(
                "Escaped to clearance {:.2}m, starting scan",
                clearance.overall_min
            );
            self.push_scan_targets(nav_state, pose);
            self.state = MappingState::Scanning {
                segments_completed: 0,
            };
            nav_state.set_status("Mapping: Scanning surroundings");
        } else if attempts >= self.config.max_escape_attempts {
            // Give up after max attempts
            log::warn!("Failed to find clear space after {} attempts", attempts);
            self.state = MappingState::Failed {
                reason: "Cannot find space to rotate".to_string(),
            };
            nav_state.set_status("Mapping failed: No clear space found");
            nav_state.clear_targets();
        } else {
            // Try again with updated clearance info
            log::info!(
                "Still confined ({:.2}m), attempt {}/{}",
                clearance.overall_min,
                attempts + 1,
                self.config.max_escape_attempts
            );
            self.push_escape_target(nav_state, clearance, pose);
            self.state = MappingState::EscapingConfined {
                attempts: attempts + 1,
            };
        }
    }

    /// Push escape target to navigation stack.
    fn push_escape_target(
        &self,
        nav_state: &mut NavigationState,
        clearance: &ClearanceAnalysis,
        pose: &Pose2D,
    ) {
        let escape_point = clearance.calculate_escape_target(
            pose,
            self.config.min_rotation_clearance,
            self.config.min_escape_distance,
        );

        let target = NavTarget::new(
            escape_point,
            None, // Navigator chooses optimal heading
            NavTargetType::Intermediate,
            NavTargetSource::Mapping,
            "Escape to open space",
        )
        .with_position_tolerance(0.05) // 5cm
        .with_heading_tolerance(0.3); // ~17°

        log::info!(
            "Pushing escape target: ({:.2}, {:.2})",
            escape_point.x,
            escape_point.y
        );
        nav_state.push_target(target);
    }

    /// Push 4 rotation targets for 360° scan (90° each).
    fn push_scan_targets(&mut self, nav_state: &mut NavigationState, pose: &Pose2D) {
        let start_theta = pose.theta;

        // Clear previous scan headings
        self.scan_segment_headings.clear();

        // Push 4 targets for 90° each
        let targets: Vec<NavTarget> = (1..=4)
            .map(|i| {
                let target_theta =
                    normalize_angle(start_theta + (i as f32) * std::f32::consts::FRAC_PI_2);

                self.scan_segment_headings.push(target_theta);

                NavTarget::new(
                    Point2D::new(pose.x, pose.y), // Stay in place
                    Some(target_theta),
                    NavTargetType::Intermediate,
                    NavTargetSource::Mapping,
                    format!("Scan {}/4", i),
                )
                .with_position_tolerance(0.03) // 3cm - stay in place
                .with_heading_tolerance(0.15) // ~8.6°
            })
            .collect();

        log::info!("Pushing {} scan targets for 360° rotation", targets.len());
        nav_state.push_targets(targets);
    }

    /// Called when a navigation target is reached.
    pub fn on_target_reached(
        &mut self,
        nav_state: &mut NavigationState,
        map: Option<&CurrentMapData>,
        pose: &Pose2D,
        target: &NavTarget,
    ) {
        log::info!("Mapping: Target reached - {}", target.description);

        match &self.state {
            MappingState::EscapingConfined { .. } => {
                // Need to re-analyze clearance - will be called via on_lidar_scan
                log::info!("Escape target reached, will re-analyze clearance");
            }

            MappingState::Scanning { segments_completed } => {
                let completed = segments_completed + 1;
                log::info!("Scan segment {}/4 completed", completed);

                if completed >= 4 {
                    // Full scan complete
                    self.stats.scans_completed += 1;
                    self.state = MappingState::FindingFrontier;
                    nav_state.set_status("Mapping: Finding frontiers");

                    // Find frontiers now
                    if let Some(map) = map {
                        self.find_and_push_frontier(nav_state, map, pose);
                    }
                } else {
                    self.state = MappingState::Scanning {
                        segments_completed: completed,
                    };
                }
            }

            MappingState::NavigatingToFrontier { .. } => {
                // Frontier reached - do another scan
                self.stats.frontiers_visited += 1;
                log::info!("Frontier reached, starting new scan");

                self.push_scan_targets(nav_state, pose);
                self.state = MappingState::Scanning {
                    segments_completed: 0,
                };
                nav_state.set_status("Mapping: Scanning surroundings");
            }

            _ => {
                log::debug!(
                    "Target reached in unexpected state: {:?}",
                    self.state.as_str()
                );
            }
        }
    }

    /// Called when a navigation target fails.
    pub fn on_target_failed(
        &mut self,
        nav_state: &mut NavigationState,
        map: Option<&CurrentMapData>,
        pose: &Pose2D,
        target: &NavTarget,
        reason: &str,
    ) {
        log::warn!(
            "Mapping: Target failed - {} (reason: {})",
            target.description,
            reason
        );

        match &self.state {
            MappingState::EscapingConfined { attempts } => {
                if *attempts >= self.config.max_escape_attempts {
                    self.state = MappingState::Failed {
                        reason: format!("Escape failed: {}", reason),
                    };
                    nav_state.set_status("Mapping failed: Cannot escape confined space");
                } else {
                    // Will retry via handle_escape_complete when lidar scan arrives
                    log::info!("Escape target failed, will retry");
                }
            }

            MappingState::Scanning { .. } => {
                // Scan rotation failed - try to continue with partial scan
                log::warn!("Scan rotation failed, attempting to find frontiers anyway");
                self.stats.scans_completed += 1; // Count as completed (partial)
                self.state = MappingState::FindingFrontier;

                if let Some(map) = map {
                    self.find_and_push_frontier(nav_state, map, pose);
                }
            }

            MappingState::NavigatingToFrontier { .. } => {
                // Mark frontier as blocked and try another
                self.stats.frontiers_blocked += 1;
                self.blocked_frontiers.push(target.position);

                log::info!(
                    "Frontier blocked ({} total), trying alternate",
                    self.blocked_frontiers.len()
                );

                if self.blocked_frontiers.len() >= self.config.max_blocked_frontiers {
                    self.state = MappingState::Complete;
                    nav_state.active_feature = None;
                    nav_state.set_status("Mapping complete (many blocked frontiers)");
                    self.stats.end_time = Some(Instant::now());
                } else {
                    // Try to find another frontier
                    self.state = MappingState::FindingFrontier;
                    if let Some(map) = map {
                        self.find_and_push_frontier(nav_state, map, pose);
                    }
                }
            }

            _ => {
                log::debug!(
                    "Target failed in unexpected state: {:?}",
                    self.state.as_str()
                );
            }
        }
    }

    /// Find frontiers and push the best one as a navigation target.
    fn find_and_push_frontier(
        &mut self,
        nav_state: &mut NavigationState,
        map: &CurrentMapData,
        pose: &Pose2D,
    ) {
        let robot_pos = Point2D::new(pose.x, pose.y);

        // Find all frontier clusters
        let frontiers = find_frontiers(
            map,
            &robot_pos,
            self.config.min_frontier_size,
            self.config.frontier_cluster_distance,
        );

        log::info!("Found {} frontier candidates", frontiers.len());

        // Select best frontier (closest, large enough, not blocked)
        let best = select_best_frontier(
            &frontiers,
            &self.blocked_frontiers,
            self.config.min_frontier_distance,
            self.config.min_frontier_size,
            self.config.frontier_cluster_distance,
        );

        if let Some(frontier) = best {
            let target = NavTarget::frontier(frontier.centroid.x, frontier.centroid.y);
            let target_id = target.id;

            log::info!(
                "Pushing frontier target: ({:.2}, {:.2}), dist={:.2}m, size={}",
                frontier.centroid.x,
                frontier.centroid.y,
                frontier.distance,
                frontier.size
            );

            nav_state.push_target(target);

            self.state = MappingState::NavigatingToFrontier { target_id };
            nav_state.set_status(format!(
                "Mapping: Navigating to frontier ({} remaining)",
                frontiers.len() - 1
            ));
        } else {
            // No more frontiers - mapping complete!
            log::info!("Mapping complete - no more frontiers");

            self.state = MappingState::Complete;
            nav_state.active_feature = None;
            nav_state.set_status("Mapping complete");
            self.stats.end_time = Some(Instant::now());
        }
    }

    /// Called periodically to check for state timeouts or other conditions.
    pub fn update(&mut self, nav_state: &mut NavigationState, _pose: &Pose2D) {
        // Check for stuck conditions, timeouts, etc.
        if let MappingState::Initializing = &self.state {
            // Could add timeout for initialization
        }

        // Update status message with progress
        if let MappingState::NavigatingToFrontier { .. } = &self.state
            && let Some(_path) = nav_state.current_path()
        {
            let progress = nav_state.path_progress() * 100.0;
            nav_state.set_status(format!(
                "Mapping: Navigating to frontier ({:.0}%)",
                progress
            ));
        }

        // Update internal status message for UI streaming
        self.status_message = self.build_status_message();
    }

    /// Get current progress for UI streaming.
    pub fn get_progress(&self, area_explored_m2: f32) -> MappingProgressState {
        use MappingProgressStateEnum::*;

        let state_enum = match &self.state {
            MappingState::Idle => Idle,
            MappingState::Initializing => Initializing,
            MappingState::AnalyzingClearance => Analyzing,
            MappingState::EscapingConfined { .. } => Escaping,
            MappingState::Scanning { .. } => Scanning,
            MappingState::FindingFrontier => FindingFrontier,
            MappingState::NavigatingToFrontier { .. } => Navigating,
            MappingState::Complete => Complete,
            MappingState::Failed { .. } => Failed,
        };

        let failure_reason = match &self.state {
            MappingState::Failed { reason } => reason.clone(),
            _ => String::new(),
        };

        MappingProgressState {
            state: state_enum,
            frontiers_found: 0, // Not currently tracked
            frontiers_visited: self.stats.frontiers_visited,
            frontiers_failed: self.stats.frontiers_blocked,
            bumper_obstacles_marked: 0, // Not currently tracked
            elapsed_time_ms: (self.stats.elapsed_secs() * 1000.0) as u64,
            scan_count: self.stats.scans_completed,
            current_frontier: None, // Could add if we track current frontier
            frontier_distance: None,
            area_explored_m2,
            exploration_percent: 0.0, // Hard to estimate
            status_message: self.status_message.clone(),
            failure_reason,
        }
    }

    /// Build current status message based on state.
    fn build_status_message(&self) -> String {
        match &self.state {
            MappingState::Idle => "Idle".to_string(),
            MappingState::Initializing => "Initializing hardware".to_string(),
            MappingState::AnalyzingClearance => "Analyzing clearance".to_string(),
            MappingState::EscapingConfined { attempts } => {
                format!("Escaping confined space (attempt {})", attempts)
            }
            MappingState::Scanning { segments_completed } => {
                format!("Scanning ({}/4)", segments_completed)
            }
            MappingState::FindingFrontier => "Finding next frontier".to_string(),
            MappingState::NavigatingToFrontier { .. } => "Navigating to frontier".to_string(),
            MappingState::Complete => "Mapping complete".to_string(),
            MappingState::Failed { reason } => format!("Failed: {}", reason),
        }
    }
}

impl Default for MappingFeature {
    fn default() -> Self {
        Self::new()
    }
}

/// Normalize angle to [-π, π].
fn normalize_angle(angle: f32) -> f32 {
    let mut result = angle;
    while result > std::f32::consts::PI {
        result -= std::f32::consts::TAU;
    }
    while result < -std::f32::consts::PI {
        result += std::f32::consts::TAU;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mapping_config_default() {
        let config = MappingConfig::default();
        assert!(config.min_rotation_clearance > 0.0);
        assert!(config.max_escape_attempts > 0);
        assert!(config.min_frontier_size > 0);
    }

    #[test]
    fn test_mapping_state_strings() {
        assert_eq!(MappingState::Idle.as_str(), "IDLE");
        assert_eq!(MappingState::Initializing.as_str(), "INITIALIZING");
        assert_eq!(MappingState::Complete.as_str(), "COMPLETE");
        assert_eq!(
            MappingState::EscapingConfined { attempts: 1 }.as_str(),
            "ESCAPING_CONFINED"
        );
    }

    #[test]
    fn test_mapping_feature_start() {
        let mut feature = MappingFeature::new();
        let mut nav_state = NavigationState::new();
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        feature.start(&mut nav_state, &pose);

        assert!(feature.is_active());
        assert!(matches!(feature.state, MappingState::Initializing));
        assert!(feature.start_pose.is_some());
        assert!(feature.stats.start_time.is_some());
        assert_eq!(nav_state.active_feature, Some(NavTargetSource::Mapping));
    }

    #[test]
    fn test_mapping_feature_stop() {
        let mut feature = MappingFeature::new();
        let mut nav_state = NavigationState::new();
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        feature.start(&mut nav_state, &pose);
        feature.stop(&mut nav_state);

        assert!(!feature.is_active());
        assert!(matches!(feature.state, MappingState::Idle));
        assert!(feature.stats.end_time.is_some());
        assert_eq!(nav_state.active_feature, None);
    }

    #[test]
    fn test_mapping_stats_elapsed() {
        let mut stats = MappingStats::default();
        assert_eq!(stats.elapsed_secs(), 0.0);

        stats.start_time = Some(Instant::now());
        std::thread::sleep(std::time::Duration::from_millis(10));
        assert!(stats.elapsed_secs() > 0.0);
    }

    #[test]
    fn test_normalize_angle() {
        use std::f32::consts::PI;

        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(PI) - PI).abs() < 0.001);
        assert!((normalize_angle(-PI) - (-PI)).abs() < 0.001);
        assert!((normalize_angle(2.0 * PI) - 0.0).abs() < 0.001);
        assert!((normalize_angle(-2.0 * PI) - 0.0).abs() < 0.001);
        // 3π normalizes to π (equivalent to -π but returns π)
        assert!(normalize_angle(3.0 * PI).abs() - PI < 0.001);
    }
}
