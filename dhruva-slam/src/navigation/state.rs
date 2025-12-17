//! Navigation state management.
//!
//! [`NavigationState`] holds the target stack and current path, with
//! dirty flags for publish-on-change semantics.
//!
//! Note: Some utility methods are defined for future use.

use super::{NavTarget, NavTargetSource, Path};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::time::Instant;

/// Navigation execution state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum NavState {
    /// No active navigation.
    #[default]
    Idle,

    /// Planning path to current target.
    Planning,

    /// Following path to target.
    Navigating,

    /// Escaping from obstacle before replanning.
    /// Robot backs away from wall/obstacle to get clearance.
    Escaping,

    /// Rotating to final heading at target.
    RotatingToHeading,

    /// Current target reached, will pop and continue.
    TargetReached,

    /// Navigation failed (path blocked, stuck, etc.).
    Failed,

    /// Navigation cancelled by user.
    Cancelled,
}

impl NavState {
    /// Check if navigation is active (not idle, reached, failed, or cancelled).
    pub fn is_active(&self) -> bool {
        matches!(
            self,
            NavState::Planning
                | NavState::Navigating
                | NavState::Escaping
                | NavState::RotatingToHeading
        )
    }

    /// Convert to string for proto serialization.
    pub fn as_str(&self) -> &'static str {
        match self {
            NavState::Idle => "IDLE",
            NavState::Planning => "PLANNING",
            NavState::Navigating => "NAVIGATING",
            NavState::Escaping => "ESCAPING",
            NavState::RotatingToHeading => "ROTATING_TO_HEADING",
            NavState::TargetReached => "TARGET_REACHED",
            NavState::Failed => "FAILED",
            NavState::Cancelled => "CANCELLED",
        }
    }
}

/// Reason for path planning failure.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PathFailureReason {
    /// No path found to target (blocked or unreachable).
    NoPathFound,

    /// Path blocked, replanning failed after N attempts.
    PathBlocked { replan_attempts: u32 },

    /// Robot stuck (no progress for timeout).
    Stuck { stuck_duration_secs: f32 },

    /// Target became invalid (e.g., now occupied).
    TargetInvalid,

    /// Operation cancelled by user.
    Cancelled,

    /// Hardware error (motors, sensors).
    HardwareError { message: String },
}

impl PathFailureReason {
    /// Get a human-readable description.
    pub fn description(&self) -> String {
        match self {
            PathFailureReason::NoPathFound => "No path found to target".to_string(),
            PathFailureReason::PathBlocked { replan_attempts } => {
                format!("Path blocked after {} replan attempts", replan_attempts)
            }
            PathFailureReason::Stuck {
                stuck_duration_secs,
            } => {
                format!("Robot stuck for {:.1}s", stuck_duration_secs)
            }
            PathFailureReason::TargetInvalid => "Target location is no longer valid".to_string(),
            PathFailureReason::Cancelled => "Navigation cancelled".to_string(),
            PathFailureReason::HardwareError { message } => format!("Hardware error: {}", message),
        }
    }
}

/// Navigation state held in SharedState.
///
/// Published to UI whenever targets or path change.
///
/// # Dirty Flags
///
/// The state maintains two dirty flags for publish-on-change:
/// - `targets_dirty`: Set when target stack is modified (push/pop/clear)
/// - `path_dirty`: Set when path is modified (plan/replan/prune)
///
/// Publisher thread checks these flags and publishes updates only when dirty.
#[derive(Debug, Clone, Default)]
pub struct NavigationState {
    /// Target stack - front is current target.
    ///
    /// Uses VecDeque for efficient push_front/pop_front operations.
    target_stack: VecDeque<NavTarget>,

    /// Current path to target_stack[0] (if any).
    current_path: Option<Path>,

    /// Navigation execution state.
    pub nav_state: NavState,

    /// Index of current waypoint in path.
    pub current_waypoint_index: usize,

    /// Active feature that's generating targets.
    pub active_feature: Option<NavTargetSource>,

    /// Failure reason (if nav_state == Failed).
    pub failure_reason: Option<PathFailureReason>,

    /// Human-readable status message.
    pub status_message: String,

    /// Number of targets completed in current operation.
    pub targets_completed: u32,

    /// Number of replan attempts for current target.
    pub replan_attempts: u32,

    /// Dirty flag: targets stack was modified.
    targets_dirty: bool,

    /// Dirty flag: path was modified.
    path_dirty: bool,

    /// Flag indicating a replan is needed.
    needs_replan: bool,

    /// Time when escape maneuver started (for timeout).
    pub escape_start_time: Option<Instant>,

    /// Direction to escape (radians, in world frame).
    pub escape_direction: f32,
}

impl NavigationState {
    /// Create a new navigation state.
    pub fn new() -> Self {
        Self::default()
    }

    // ========================================================================
    // Target Stack Operations
    // ========================================================================

    /// Push a new target onto the stack.
    ///
    /// Target becomes current if stack was empty.
    /// Sets `targets_dirty` flag.
    pub fn push_target(&mut self, target: NavTarget) {
        let was_empty = self.target_stack.is_empty();
        self.target_stack.push_back(target);
        self.targets_dirty = true;

        if was_empty {
            // First target - need to plan path
            self.needs_replan = true;
            self.nav_state = NavState::Planning;
        }
    }

    /// Push multiple targets onto the stack.
    ///
    /// First target in the vec becomes current if stack was empty.
    /// Sets `targets_dirty` flag.
    pub fn push_targets(&mut self, targets: impl IntoIterator<Item = NavTarget>) {
        let was_empty = self.target_stack.is_empty();

        for target in targets {
            self.target_stack.push_back(target);
        }

        self.targets_dirty = true;

        if was_empty && !self.target_stack.is_empty() {
            self.needs_replan = true;
            self.nav_state = NavState::Planning;
        }
    }

    /// Pop current target (reached or failed).
    ///
    /// Automatically requests path plan to next target if any.
    /// Sets `targets_dirty` flag.
    ///
    /// Returns the popped target.
    pub fn pop_target(&mut self) -> Option<NavTarget> {
        let target = self.target_stack.pop_front();
        self.targets_dirty = true;

        if target.is_some() {
            self.targets_completed += 1;
        }

        if !self.target_stack.is_empty() {
            // Plan path to new current target
            self.needs_replan = true;
            self.nav_state = NavState::Planning;
            self.current_waypoint_index = 0;
            self.replan_attempts = 0;
        } else {
            // No more targets - clear path
            self.current_path = None;
            self.path_dirty = true;
            self.nav_state = NavState::Idle;
            self.active_feature = None;
        }

        target
    }

    /// Clear all targets (cancel operation).
    ///
    /// Sets both `targets_dirty` and `path_dirty` flags.
    pub fn clear_targets(&mut self) {
        self.target_stack.clear();
        self.current_path = None;
        self.nav_state = NavState::Cancelled;
        self.active_feature = None;
        self.current_waypoint_index = 0;
        self.needs_replan = false;
        self.replan_attempts = 0;
        self.targets_dirty = true;
        self.path_dirty = true;
    }

    /// Get current target (front of stack).
    pub fn current_target(&self) -> Option<&NavTarget> {
        self.target_stack.front()
    }

    /// Get current target mutably.
    pub fn current_target_mut(&mut self) -> Option<&mut NavTarget> {
        self.target_stack.front_mut()
    }

    /// Get all pending targets.
    pub fn targets(&self) -> &VecDeque<NavTarget> {
        &self.target_stack
    }

    /// Get number of targets in stack.
    pub fn target_count(&self) -> usize {
        self.target_stack.len()
    }

    /// Check if there are any targets.
    pub fn has_targets(&self) -> bool {
        !self.target_stack.is_empty()
    }

    // ========================================================================
    // Path Operations
    // ========================================================================

    /// Set the current path.
    ///
    /// Sets `path_dirty` flag.
    pub fn set_path(&mut self, path: Path) {
        self.current_path = Some(path);
        self.current_waypoint_index = 0;
        self.nav_state = NavState::Navigating;
        self.needs_replan = false;
        self.path_dirty = true;
    }

    /// Clear the current path.
    ///
    /// Sets `path_dirty` flag.
    pub fn clear_path(&mut self) {
        self.current_path = None;
        self.current_waypoint_index = 0;
        self.path_dirty = true;
    }

    /// Get the current path.
    pub fn current_path(&self) -> Option<&Path> {
        self.current_path.as_ref()
    }

    /// Get the current path mutably.
    pub fn current_path_mut(&mut self) -> Option<&mut Path> {
        self.current_path.as_mut()
    }

    /// Get current waypoint from the path.
    pub fn current_waypoint(&self) -> Option<&super::Waypoint> {
        self.current_path
            .as_ref()
            .and_then(|p| p.get(self.current_waypoint_index))
    }

    /// Advance to the next waypoint.
    ///
    /// Sets `path_dirty` flag.
    ///
    /// Returns `true` if there are more waypoints, `false` if at end of path.
    pub fn advance_waypoint(&mut self) -> bool {
        if let Some(path) = &self.current_path
            && self.current_waypoint_index + 1 < path.len()
        {
            self.current_waypoint_index += 1;
            self.path_dirty = true;
            return true;
        }
        false
    }

    /// Get remaining path length to current target.
    pub fn path_remaining(&self) -> f32 {
        self.current_path
            .as_ref()
            .map(|p| p.remaining_length(self.current_waypoint_index))
            .unwrap_or(0.0)
    }

    /// Get total path length.
    pub fn path_total_length(&self) -> f32 {
        self.current_path
            .as_ref()
            .map(|p| p.total_length)
            .unwrap_or(0.0)
    }

    // ========================================================================
    // Replan Operations
    // ========================================================================

    /// Request a path replan.
    ///
    /// Called when obstacle detected or significant deviation from path.
    pub fn request_replan(&mut self) {
        self.needs_replan = true;
        self.replan_attempts += 1;
        self.nav_state = NavState::Planning;
    }

    /// Check if a replan is needed.
    pub fn needs_replan(&self) -> bool {
        self.needs_replan
    }

    /// Clear the replan flag (called after planning).
    pub fn clear_replan_flag(&mut self) {
        self.needs_replan = false;
    }

    // ========================================================================
    // Escape Operations
    // ========================================================================

    /// Start an escape maneuver (backing away from obstacle).
    ///
    /// Called when hazard is detected during navigation.
    /// Robot will back away before attempting to replan.
    pub fn start_escape(&mut self, direction: f32) {
        self.nav_state = NavState::Escaping;
        self.escape_start_time = Some(Instant::now());
        self.escape_direction = direction;
        self.status_message = "Escaping from obstacle".to_string();
        log::info!("Starting escape maneuver, direction={:.2} rad", direction);
    }

    /// Complete escape and transition to replanning.
    ///
    /// Called when escape timeout or clearance achieved.
    pub fn finish_escape_and_replan(&mut self) {
        self.escape_start_time = None;
        self.replan_attempts += 1;
        self.needs_replan = true;
        self.nav_state = NavState::Planning;
        log::info!(
            "Escape complete, transitioning to replan (attempt {})",
            self.replan_attempts
        );
    }

    /// Check if currently escaping.
    pub fn is_escaping(&self) -> bool {
        self.nav_state == NavState::Escaping
    }

    /// Get elapsed time since escape started.
    pub fn escape_elapsed_secs(&self) -> f32 {
        self.escape_start_time
            .map(|t| t.elapsed().as_secs_f32())
            .unwrap_or(0.0)
    }

    // ========================================================================
    // Dirty Flag Operations
    // ========================================================================

    /// Check and clear targets dirty flag.
    ///
    /// Returns `true` if targets were modified since last check.
    pub fn take_targets_dirty(&mut self) -> bool {
        std::mem::take(&mut self.targets_dirty)
    }

    /// Check and clear path dirty flag.
    ///
    /// Returns `true` if path was modified since last check.
    pub fn take_path_dirty(&mut self) -> bool {
        std::mem::take(&mut self.path_dirty)
    }

    /// Check if targets are dirty (without clearing).
    pub fn is_targets_dirty(&self) -> bool {
        self.targets_dirty
    }

    /// Check if path is dirty (without clearing).
    pub fn is_path_dirty(&self) -> bool {
        self.path_dirty
    }

    /// Force both dirty flags (for initial publish).
    pub fn mark_all_dirty(&mut self) {
        self.targets_dirty = true;
        self.path_dirty = true;
    }

    // ========================================================================
    // Failure Handling
    // ========================================================================

    /// Set navigation as failed.
    pub fn set_failed(&mut self, reason: PathFailureReason) {
        self.nav_state = NavState::Failed;
        self.status_message = reason.description();
        self.failure_reason = Some(reason);
        self.path_dirty = true;
    }

    /// Set status message.
    pub fn set_status(&mut self, message: impl Into<String>) {
        self.status_message = message.into();
    }

    // ========================================================================
    // State Queries
    // ========================================================================

    /// Check if navigation is active.
    pub fn is_active(&self) -> bool {
        self.nav_state.is_active() || self.has_targets()
    }

    /// Check if currently navigating (following path).
    pub fn is_navigating(&self) -> bool {
        self.nav_state == NavState::Navigating
    }

    /// Check if waiting for path planning.
    pub fn is_planning(&self) -> bool {
        self.nav_state == NavState::Planning
    }

    /// Get progress percentage (0.0 to 1.0) for current path.
    pub fn path_progress(&self) -> f32 {
        let total = self.path_total_length();
        if total <= 0.0 {
            return 0.0;
        }
        let remaining = self.path_remaining();
        1.0 - (remaining / total).min(1.0)
    }

    /// Reset state for a new operation.
    pub fn reset(&mut self) {
        self.target_stack.clear();
        self.current_path = None;
        self.nav_state = NavState::Idle;
        self.current_waypoint_index = 0;
        self.active_feature = None;
        self.failure_reason = None;
        self.status_message.clear();
        self.targets_completed = 0;
        self.replan_attempts = 0;
        self.needs_replan = false;
        self.escape_start_time = None;
        self.escape_direction = 0.0;
        self.targets_dirty = true;
        self.path_dirty = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;
    use crate::navigation::{NavTargetSource, NavTargetType, Waypoint};

    fn make_test_target(x: f32, y: f32) -> NavTarget {
        NavTarget::new(
            Point2D::new(x, y),
            None,
            NavTargetType::Waypoint,
            NavTargetSource::UserClick,
            format!("Test ({}, {})", x, y),
        )
    }

    #[test]
    fn test_push_target() {
        let mut state = NavigationState::new();

        assert!(!state.has_targets());
        assert!(!state.is_active());

        state.push_target(make_test_target(1.0, 1.0));

        assert!(state.has_targets());
        assert_eq!(state.target_count(), 1);
        assert!(state.is_targets_dirty());
        assert!(state.needs_replan());
        assert_eq!(state.nav_state, NavState::Planning);
    }

    #[test]
    fn test_push_multiple_targets() {
        let mut state = NavigationState::new();

        state.push_targets(vec![
            make_test_target(1.0, 1.0),
            make_test_target(2.0, 2.0),
            make_test_target(3.0, 3.0),
        ]);

        assert_eq!(state.target_count(), 3);
        assert!(state.current_target().is_some());
        assert!((state.current_target().unwrap().position.x - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_pop_target() {
        let mut state = NavigationState::new();

        state.push_targets(vec![make_test_target(1.0, 1.0), make_test_target(2.0, 2.0)]);

        // Clear dirty flag
        state.take_targets_dirty();

        let popped = state.pop_target();
        assert!(popped.is_some());
        assert!((popped.unwrap().position.x - 1.0).abs() < 1e-6);

        assert_eq!(state.target_count(), 1);
        assert!(state.is_targets_dirty());
        assert!(state.needs_replan());
        assert_eq!(state.targets_completed, 1);
    }

    #[test]
    fn test_pop_last_target() {
        let mut state = NavigationState::new();

        state.push_target(make_test_target(1.0, 1.0));
        state.pop_target();

        assert!(!state.has_targets());
        assert_eq!(state.nav_state, NavState::Idle);
        assert!(state.current_path.is_none());
    }

    #[test]
    fn test_clear_targets() {
        let mut state = NavigationState::new();

        state.push_targets(vec![make_test_target(1.0, 1.0), make_test_target(2.0, 2.0)]);

        state.clear_targets();

        assert!(!state.has_targets());
        assert_eq!(state.nav_state, NavState::Cancelled);
        assert!(state.is_targets_dirty());
        assert!(state.is_path_dirty());
    }

    #[test]
    fn test_set_path() {
        let mut state = NavigationState::new();

        state.push_target(make_test_target(5.0, 5.0));

        let path = Path::new(
            vec![
                Waypoint::new(0.0, 0.0),
                Waypoint::new(2.5, 2.5),
                Waypoint::new(5.0, 5.0),
            ],
            state.current_target().unwrap().id,
        );

        state.set_path(path);

        assert!(state.current_path().is_some());
        assert_eq!(state.nav_state, NavState::Navigating);
        assert!(state.is_path_dirty());
        assert!(!state.needs_replan());
    }

    #[test]
    fn test_advance_waypoint() {
        let mut state = NavigationState::new();

        state.push_target(make_test_target(3.0, 0.0));

        let path = Path::new(
            vec![
                Waypoint::new(0.0, 0.0),
                Waypoint::new(1.0, 0.0),
                Waypoint::new(2.0, 0.0),
                Waypoint::new(3.0, 0.0),
            ],
            state.current_target().unwrap().id,
        );

        state.set_path(path);
        state.take_path_dirty();

        assert_eq!(state.current_waypoint_index, 0);

        assert!(state.advance_waypoint());
        assert_eq!(state.current_waypoint_index, 1);
        assert!(state.is_path_dirty());

        state.take_path_dirty();

        assert!(state.advance_waypoint());
        assert!(state.advance_waypoint());
        assert!(!state.advance_waypoint()); // At end
        assert_eq!(state.current_waypoint_index, 3);
    }

    #[test]
    fn test_dirty_flags() {
        let mut state = NavigationState::new();

        assert!(!state.is_targets_dirty());
        assert!(!state.is_path_dirty());

        state.push_target(make_test_target(1.0, 1.0));

        assert!(state.is_targets_dirty());

        // Take clears the flag
        assert!(state.take_targets_dirty());
        assert!(!state.is_targets_dirty());
        assert!(!state.take_targets_dirty()); // Already cleared
    }

    #[test]
    fn test_path_progress() {
        let mut state = NavigationState::new();

        // No path = 0 progress
        assert!((state.path_progress() - 0.0).abs() < 1e-6);

        state.push_target(make_test_target(4.0, 0.0));

        let path = Path::new(
            vec![
                Waypoint::new(0.0, 0.0),
                Waypoint::new(2.0, 0.0),
                Waypoint::new(4.0, 0.0),
            ],
            1,
        );

        state.set_path(path);

        // At start = 0 progress
        assert!((state.path_progress() - 0.0).abs() < 1e-6);

        // Advance one waypoint
        state.advance_waypoint();
        assert!((state.path_progress() - 0.5).abs() < 1e-6);

        // Advance to end
        state.advance_waypoint();
        assert!((state.path_progress() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_request_replan() {
        let mut state = NavigationState::new();

        state.push_target(make_test_target(1.0, 1.0));
        state.clear_replan_flag();

        assert!(!state.needs_replan());
        assert_eq!(state.replan_attempts, 0);

        state.request_replan();

        assert!(state.needs_replan());
        assert_eq!(state.replan_attempts, 1);
        assert_eq!(state.nav_state, NavState::Planning);

        state.request_replan();
        assert_eq!(state.replan_attempts, 2);
    }

    #[test]
    fn test_set_failed() {
        let mut state = NavigationState::new();

        state.set_failed(PathFailureReason::NoPathFound);

        assert_eq!(state.nav_state, NavState::Failed);
        assert!(state.failure_reason.is_some());
        assert!(!state.status_message.is_empty());
    }

    #[test]
    fn test_nav_state_is_active() {
        assert!(!NavState::Idle.is_active());
        assert!(NavState::Planning.is_active());
        assert!(NavState::Navigating.is_active());
        assert!(NavState::Escaping.is_active());
        assert!(NavState::RotatingToHeading.is_active());
        assert!(!NavState::TargetReached.is_active());
        assert!(!NavState::Failed.is_active());
        assert!(!NavState::Cancelled.is_active());
    }
}
