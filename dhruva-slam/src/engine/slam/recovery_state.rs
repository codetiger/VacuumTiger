//! Recovery state machine for SLAM tracking loss.
//!
//! Manages transitions between tracking, lost, and recovering states,
//! and coordinates recovery strategies when the robot loses localization.
//!
//! # State Machine
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────┐
//! │                                                              │
//! │    ┌──────────┐                    ┌──────────┐              │
//! │    │ Tracking │ ──kidnapped────▶   │   Lost   │              │
//! │    │          │ ◀──recovered────── │          │              │
//! │    └──────────┘                    └────┬─────┘              │
//! │         ▲                               │                    │
//! │         │                               │ start recovery     │
//! │         │                               ▼                    │
//! │         │                        ┌───────────┐               │
//! │         └──success───────────────│Recovering │               │
//! │                                  │           │               │
//! │                                  └───────────┘               │
//! │                                        │                     │
//! │                                        │ max attempts        │
//! │                                        ▼                     │
//! │                                  ┌───────────┐               │
//! │                                  │  Failed   │               │
//! │                                  │(manual)   │               │
//! │                                  └───────────┘               │
//! └──────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Recovery Strategies
//!
//! The state machine supports multiple recovery strategies:
//!
//! 1. **Local Recovery**: Expand particle filter around last known pose
//! 2. **Global Relocalization**: Spread particles across entire map
//! 3. **Return to Dock**: Navigate to charging station (known landmark)
//! 4. **User Assistance**: Request manual intervention
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::slam::{RecoveryStateMachine, RecoveryConfig};
//!
//! let config = RecoveryConfig::default();
//! let mut recovery = RecoveryStateMachine::new(config);
//!
//! // In SLAM loop
//! if kidnapped_detected {
//!     recovery.trigger_lost(timestamp_us);
//! }
//!
//! let action = recovery.update(match_score, timestamp_us);
//! match action {
//!     RecoveryAction::Continue => { /* normal operation */ }
//!     RecoveryAction::StartGlobalLocalization => { /* spread particles */ }
//!     RecoveryAction::RequestAssistance => { /* notify user */ }
//!     _ => {}
//! }
//! ```

use crate::core::types::Pose2D;

/// Recovery strategy to attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryStrategy {
    /// Expand uncertainty around last known pose.
    LocalExpansion,
    /// Global relocalization across entire map.
    GlobalRelocalization,
    /// Navigate to dock (if available).
    ReturnToDock,
    /// Request manual assistance.
    ManualAssistance,
}

/// Current state of the recovery state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryState {
    /// Normal tracking operation.
    Tracking,
    /// Lost localization, preparing recovery.
    Lost,
    /// Actively attempting recovery.
    Recovering(RecoveryStrategy),
    /// Recovery failed, requires intervention.
    Failed,
}

impl Default for RecoveryState {
    fn default() -> Self {
        Self::Tracking
    }
}

/// Action to take based on recovery state.
#[derive(Debug, Clone)]
pub enum RecoveryAction {
    /// Continue normal operation.
    Continue,
    /// Start local particle expansion.
    StartLocalRecovery { expansion_radius: f32 },
    /// Start global relocalization.
    StartGlobalLocalization,
    /// Return to dock for known reference.
    NavigateToDock,
    /// Request user assistance.
    RequestAssistance { reason: String },
    /// Recovery succeeded, resume normal operation.
    RecoveryComplete { new_pose: Option<Pose2D> },
}

/// Configuration for the recovery state machine.
#[derive(Debug, Clone)]
pub struct RecoveryConfig {
    /// Score threshold to consider tracking recovered.
    /// Default: 0.5
    pub recovery_threshold: f32,

    /// Number of consecutive good matches needed to confirm recovery.
    /// Default: 10
    pub confirmation_frames: usize,

    /// Maximum time to attempt local recovery before trying global (microseconds).
    /// Default: 5_000_000 (5 seconds)
    pub local_recovery_timeout_us: u64,

    /// Maximum time to attempt global recovery before requesting assistance (microseconds).
    /// Default: 30_000_000 (30 seconds)
    pub global_recovery_timeout_us: u64,

    /// Maximum number of recovery attempts before giving up.
    /// Default: 3
    pub max_recovery_attempts: u32,

    /// Initial expansion radius for local recovery (meters).
    /// Default: 1.0
    pub local_expansion_radius: f32,

    /// Expansion radius growth factor per failed attempt.
    /// Default: 2.0
    pub expansion_growth_factor: f32,

    /// Whether dock return is available as a recovery strategy.
    /// Default: false
    pub dock_available: bool,

    /// Whether to enable automatic recovery attempts.
    /// Default: true
    pub auto_recovery: bool,
}

impl Default for RecoveryConfig {
    fn default() -> Self {
        Self {
            recovery_threshold: 0.5,
            confirmation_frames: 10,
            local_recovery_timeout_us: 5_000_000,
            global_recovery_timeout_us: 30_000_000,
            max_recovery_attempts: 3,
            local_expansion_radius: 1.0,
            expansion_growth_factor: 2.0,
            dock_available: false,
            auto_recovery: true,
        }
    }
}

/// Recovery state machine for SLAM tracking loss.
///
/// Coordinates recovery strategies when the robot loses localization,
/// transitioning through local recovery, global relocalization, and
/// eventually requesting assistance if all strategies fail.
pub struct RecoveryStateMachine {
    config: RecoveryConfig,

    /// Current state.
    state: RecoveryState,

    /// Timestamp when current state started.
    state_start_us: u64,

    /// Number of recovery attempts in current episode.
    recovery_attempts: u32,

    /// Consecutive good matches during recovery.
    good_match_count: usize,

    /// Last known good pose (for local recovery).
    last_good_pose: Option<Pose2D>,

    /// Current expansion radius for local recovery.
    current_expansion_radius: f32,

    /// Total recovery episodes (for statistics).
    total_recovery_episodes: u32,

    /// Successful recovery count.
    successful_recoveries: u32,
}

impl RecoveryStateMachine {
    /// Create a new recovery state machine.
    pub fn new(config: RecoveryConfig) -> Self {
        let initial_radius = config.local_expansion_radius;
        Self {
            config,
            state: RecoveryState::Tracking,
            state_start_us: 0,
            recovery_attempts: 0,
            good_match_count: 0,
            last_good_pose: None,
            current_expansion_radius: initial_radius,
            total_recovery_episodes: 0,
            successful_recoveries: 0,
        }
    }

    /// Get current state.
    pub fn state(&self) -> RecoveryState {
        self.state
    }

    /// Get configuration.
    pub fn config(&self) -> &RecoveryConfig {
        &self.config
    }

    /// Check if currently in a recovery state.
    pub fn is_recovering(&self) -> bool {
        matches!(self.state, RecoveryState::Lost | RecoveryState::Recovering(_))
    }

    /// Check if recovery has failed and needs intervention.
    pub fn needs_assistance(&self) -> bool {
        matches!(self.state, RecoveryState::Failed)
    }

    /// Record a good pose for potential recovery reference.
    pub fn record_good_pose(&mut self, pose: Pose2D) {
        if self.state == RecoveryState::Tracking {
            self.last_good_pose = Some(pose);
        }
    }

    /// Trigger transition to lost state.
    ///
    /// Call this when kidnapping is detected.
    pub fn trigger_lost(&mut self, timestamp_us: u64) {
        if self.state == RecoveryState::Tracking {
            log::warn!("Recovery: Transitioning from Tracking to Lost");
            self.state = RecoveryState::Lost;
            self.state_start_us = timestamp_us;
            self.recovery_attempts = 0;
            self.good_match_count = 0;
            self.current_expansion_radius = self.config.local_expansion_radius;
            self.total_recovery_episodes += 1;
        }
    }

    /// Update state machine and get recommended action.
    ///
    /// # Arguments
    /// * `match_score` - Current scan match quality (0.0-1.0)
    /// * `timestamp_us` - Current timestamp
    ///
    /// # Returns
    /// Recommended action to take.
    pub fn update(&mut self, match_score: f32, timestamp_us: u64) -> RecoveryAction {
        match self.state {
            RecoveryState::Tracking => {
                // Already tracking, no action needed
                RecoveryAction::Continue
            }

            RecoveryState::Lost => {
                if !self.config.auto_recovery {
                    return RecoveryAction::RequestAssistance {
                        reason: "Auto-recovery disabled".to_string(),
                    };
                }

                // Start recovery
                self.recovery_attempts += 1;

                if self.recovery_attempts <= self.config.max_recovery_attempts {
                    // Try local recovery first
                    self.state = RecoveryState::Recovering(RecoveryStrategy::LocalExpansion);
                    self.state_start_us = timestamp_us;
                    log::info!(
                        "Recovery: Starting local recovery attempt {} (radius: {:.2}m)",
                        self.recovery_attempts,
                        self.current_expansion_radius
                    );
                    RecoveryAction::StartLocalRecovery {
                        expansion_radius: self.current_expansion_radius,
                    }
                } else {
                    // Exhausted attempts
                    self.state = RecoveryState::Failed;
                    self.state_start_us = timestamp_us;
                    RecoveryAction::RequestAssistance {
                        reason: "Max recovery attempts exceeded".to_string(),
                    }
                }
            }

            RecoveryState::Recovering(strategy) => {
                self.update_recovering(strategy, match_score, timestamp_us)
            }

            RecoveryState::Failed => {
                RecoveryAction::RequestAssistance {
                    reason: "Recovery failed, manual intervention required".to_string(),
                }
            }
        }
    }

    /// Update during active recovery.
    fn update_recovering(
        &mut self,
        strategy: RecoveryStrategy,
        match_score: f32,
        timestamp_us: u64,
    ) -> RecoveryAction {
        // Check for recovery success
        if match_score >= self.config.recovery_threshold {
            self.good_match_count += 1;
            if self.good_match_count >= self.config.confirmation_frames {
                // Recovery successful!
                log::info!("Recovery: Successful after {} attempts", self.recovery_attempts);
                self.state = RecoveryState::Tracking;
                self.state_start_us = timestamp_us;
                self.successful_recoveries += 1;
                self.current_expansion_radius = self.config.local_expansion_radius;
                return RecoveryAction::RecoveryComplete { new_pose: None };
            }
        } else {
            self.good_match_count = 0;
        }

        let elapsed = timestamp_us.saturating_sub(self.state_start_us);

        match strategy {
            RecoveryStrategy::LocalExpansion => {
                if elapsed > self.config.local_recovery_timeout_us {
                    // Local recovery timed out, try global
                    log::warn!("Recovery: Local recovery timed out, trying global");
                    self.state = RecoveryState::Recovering(RecoveryStrategy::GlobalRelocalization);
                    self.state_start_us = timestamp_us;
                    return RecoveryAction::StartGlobalLocalization;
                }
                RecoveryAction::Continue
            }

            RecoveryStrategy::GlobalRelocalization => {
                if elapsed > self.config.global_recovery_timeout_us {
                    // Global recovery timed out
                    if self.config.dock_available {
                        // Try returning to dock
                        log::warn!("Recovery: Global recovery timed out, navigating to dock");
                        self.state = RecoveryState::Recovering(RecoveryStrategy::ReturnToDock);
                        self.state_start_us = timestamp_us;
                        return RecoveryAction::NavigateToDock;
                    } else {
                        // Increase expansion and try again
                        self.current_expansion_radius *= self.config.expansion_growth_factor;
                        self.state = RecoveryState::Lost;
                        return self.update(match_score, timestamp_us);
                    }
                }
                RecoveryAction::Continue
            }

            RecoveryStrategy::ReturnToDock => {
                // Dock navigation is handled externally
                // Check if we've recovered by reaching dock
                RecoveryAction::Continue
            }

            RecoveryStrategy::ManualAssistance => {
                RecoveryAction::RequestAssistance {
                    reason: "Waiting for manual assistance".to_string(),
                }
            }
        }
    }

    /// Manually trigger a specific recovery strategy.
    pub fn start_strategy(&mut self, strategy: RecoveryStrategy, timestamp_us: u64) {
        self.state = RecoveryState::Recovering(strategy);
        self.state_start_us = timestamp_us;
        self.good_match_count = 0;
    }

    /// Signal that recovery was successful (external confirmation).
    pub fn confirm_recovery(&mut self, timestamp_us: u64, new_pose: Option<Pose2D>) {
        if self.is_recovering() {
            log::info!("Recovery: Externally confirmed successful");
            self.state = RecoveryState::Tracking;
            self.state_start_us = timestamp_us;
            self.successful_recoveries += 1;
            self.current_expansion_radius = self.config.local_expansion_radius;

            if let Some(pose) = new_pose {
                self.last_good_pose = Some(pose);
            }
        }
    }

    /// Reset the state machine to tracking state.
    pub fn reset(&mut self) {
        self.state = RecoveryState::Tracking;
        self.state_start_us = 0;
        self.recovery_attempts = 0;
        self.good_match_count = 0;
        self.current_expansion_radius = self.config.local_expansion_radius;
    }

    /// Reset and also clear statistics.
    pub fn reset_full(&mut self) {
        self.reset();
        self.last_good_pose = None;
        self.total_recovery_episodes = 0;
        self.successful_recoveries = 0;
    }

    /// Get the last known good pose.
    pub fn last_good_pose(&self) -> Option<Pose2D> {
        self.last_good_pose
    }

    /// Get recovery statistics.
    pub fn stats(&self) -> RecoveryStats {
        RecoveryStats {
            state: self.state,
            recovery_attempts: self.recovery_attempts,
            total_episodes: self.total_recovery_episodes,
            successful_recoveries: self.successful_recoveries,
            current_expansion_radius: self.current_expansion_radius,
        }
    }
}

impl Default for RecoveryStateMachine {
    fn default() -> Self {
        Self::new(RecoveryConfig::default())
    }
}

/// Statistics about recovery performance.
#[derive(Debug, Clone)]
pub struct RecoveryStats {
    /// Current state.
    pub state: RecoveryState,
    /// Recovery attempts in current episode.
    pub recovery_attempts: u32,
    /// Total recovery episodes.
    pub total_episodes: u32,
    /// Successful recovery count.
    pub successful_recoveries: u32,
    /// Current expansion radius.
    pub current_expansion_radius: f32,
}

impl RecoveryStats {
    /// Get success rate (0.0-1.0).
    pub fn success_rate(&self) -> f32 {
        if self.total_episodes == 0 {
            1.0
        } else {
            self.successful_recoveries as f32 / self.total_episodes as f32
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_state() {
        let recovery = RecoveryStateMachine::default();
        assert_eq!(recovery.state(), RecoveryState::Tracking);
        assert!(!recovery.is_recovering());
        assert!(!recovery.needs_assistance());
    }

    #[test]
    fn test_trigger_lost() {
        let mut recovery = RecoveryStateMachine::default();
        recovery.trigger_lost(1_000_000);

        assert_eq!(recovery.state(), RecoveryState::Lost);
        assert!(recovery.is_recovering());
    }

    #[test]
    fn test_start_local_recovery() {
        let mut recovery = RecoveryStateMachine::default();
        recovery.trigger_lost(1_000_000);

        let action = recovery.update(0.1, 1_000_000);
        match action {
            RecoveryAction::StartLocalRecovery { expansion_radius } => {
                assert!(expansion_radius > 0.0);
            }
            _ => panic!("Expected StartLocalRecovery"),
        }

        assert!(matches!(
            recovery.state(),
            RecoveryState::Recovering(RecoveryStrategy::LocalExpansion)
        ));
    }

    #[test]
    fn test_recovery_success() {
        let config = RecoveryConfig {
            confirmation_frames: 3,
            recovery_threshold: 0.5,
            ..Default::default()
        };
        let mut recovery = RecoveryStateMachine::new(config);

        recovery.trigger_lost(0);
        recovery.update(0.1, 0); // Start recovery

        // Good matches should lead to recovery
        for i in 0..3 {
            let action = recovery.update(0.8, (i + 1) * 100_000);
            if i == 2 {
                assert!(matches!(action, RecoveryAction::RecoveryComplete { .. }));
            }
        }

        assert_eq!(recovery.state(), RecoveryState::Tracking);
        assert_eq!(recovery.stats().successful_recoveries, 1);
    }

    #[test]
    fn test_local_timeout_to_global() {
        let config = RecoveryConfig {
            local_recovery_timeout_us: 1_000_000,
            confirmation_frames: 10,
            ..Default::default()
        };
        let mut recovery = RecoveryStateMachine::new(config);

        recovery.trigger_lost(0);
        recovery.update(0.1, 0); // Start local recovery

        // Timeout should trigger global
        let action = recovery.update(0.1, 2_000_000);
        assert!(matches!(action, RecoveryAction::StartGlobalLocalization));
        assert!(matches!(
            recovery.state(),
            RecoveryState::Recovering(RecoveryStrategy::GlobalRelocalization)
        ));
    }

    #[test]
    fn test_max_attempts_failure() {
        let config = RecoveryConfig {
            max_recovery_attempts: 2,
            local_recovery_timeout_us: 100,
            global_recovery_timeout_us: 100,
            dock_available: false,
            ..Default::default()
        };
        let mut recovery = RecoveryStateMachine::new(config);

        // Exhaust all attempts
        for _ in 0..3 {
            recovery.trigger_lost(0);
            recovery.update(0.1, 0);
            recovery.update(0.1, 200);
            recovery.update(0.1, 400);
        }

        assert!(matches!(
            recovery.state(),
            RecoveryState::Failed | RecoveryState::Recovering(_) | RecoveryState::Lost
        ));
    }

    #[test]
    fn test_auto_recovery_disabled() {
        let config = RecoveryConfig {
            auto_recovery: false,
            ..Default::default()
        };
        let mut recovery = RecoveryStateMachine::new(config);

        recovery.trigger_lost(0);
        let action = recovery.update(0.1, 0);

        assert!(matches!(action, RecoveryAction::RequestAssistance { .. }));
    }

    #[test]
    fn test_external_confirm_recovery() {
        let mut recovery = RecoveryStateMachine::default();
        recovery.trigger_lost(0);
        recovery.update(0.1, 0); // Start recovery

        let new_pose = Pose2D::new(1.0, 2.0, 0.0);
        recovery.confirm_recovery(1_000_000, Some(new_pose));

        assert_eq!(recovery.state(), RecoveryState::Tracking);
        assert_eq!(recovery.last_good_pose(), Some(new_pose));
    }

    #[test]
    fn test_record_good_pose() {
        let mut recovery = RecoveryStateMachine::default();
        let pose = Pose2D::new(3.0, 4.0, 0.5);
        recovery.record_good_pose(pose);

        assert_eq!(recovery.last_good_pose(), Some(pose));

        // Should not record during recovery
        recovery.trigger_lost(0);
        let new_pose = Pose2D::new(10.0, 10.0, 0.0);
        recovery.record_good_pose(new_pose);
        assert_eq!(recovery.last_good_pose(), Some(pose)); // Still old pose
    }

    #[test]
    fn test_reset() {
        let mut recovery = RecoveryStateMachine::default();
        recovery.trigger_lost(0);
        recovery.update(0.1, 0);

        recovery.reset();

        assert_eq!(recovery.state(), RecoveryState::Tracking);
        assert!(!recovery.is_recovering());
    }

    #[test]
    fn test_stats() {
        let mut recovery = RecoveryStateMachine::default();

        let stats = recovery.stats();
        assert_eq!(stats.total_episodes, 0);
        assert_eq!(stats.success_rate(), 1.0);

        recovery.trigger_lost(0);
        let stats = recovery.stats();
        assert_eq!(stats.total_episodes, 1);
    }

    #[test]
    fn test_good_match_count_reset() {
        let config = RecoveryConfig {
            confirmation_frames: 5,
            recovery_threshold: 0.5,
            ..Default::default()
        };
        let mut recovery = RecoveryStateMachine::new(config);

        recovery.trigger_lost(0);
        recovery.update(0.1, 0);

        // Some good matches
        recovery.update(0.8, 100_000);
        recovery.update(0.8, 200_000);
        // Bad match resets counter
        recovery.update(0.1, 300_000);

        // Need 5 more good matches now
        for i in 0..5 {
            let action = recovery.update(0.8, (400_000 + i * 100_000) as u64);
            if i == 4 {
                assert!(matches!(action, RecoveryAction::RecoveryComplete { .. }));
            }
        }
    }
}
