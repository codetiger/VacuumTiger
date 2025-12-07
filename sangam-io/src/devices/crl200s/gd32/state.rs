//! Component state management for GD32 driver
//!
//! This module defines the shared state used by the heartbeat thread to refresh
//! component commands every 20ms. All fields use atomic types to allow lockless reads.

use std::sync::atomic::{AtomicBool, AtomicI16, AtomicU64, AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

// Lidar auto-tuning constants
const SCAN_TIMEOUT_MS: u64 = 500; // Consider "not receiving" if no scan for 500ms
const SETTLING_TIMEOUT_MS: u64 = 1000; // Wait 1 second after PWM change before evaluating
const MIN_STEP: u8 = 2; // Converge when step reaches 2%
const MIN_PWM: u8 = 30; // Never go below 30%
const MAX_PWM: u8 = 100; // Never exceed 100%
const INITIAL_PWM: u8 = 60; // Start at known working value
const RAMP_STEP: u8 = 5; // Step size when ramping up to find max
const RECOVERY_STEP: u8 = 5; // Fixed step for recovery mode

// Lidar tuning phases
const PHASE_RAMP_UP: u8 = 0; // Ramping up to find where scans stop
const PHASE_FINE_TUNE: u8 = 1; // Binary search to find exact max stable PWM
const PHASE_STABLE: u8 = 2; // Optimal found, not sending PWM
const PHASE_RECOVERY: u8 = 3; // Lost scans, recovering

/// Get current time in milliseconds since epoch
fn current_time_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Shared component state for periodic refresh
///
/// All fields use atomic types to allow lockless reads by the heartbeat thread.
/// The heartbeat thread reads these values every 20ms and sends corresponding commands.
///
/// # Fields
///
/// - `vacuum`: Air pump speed (0-100)
/// - `main_brush`: Main roller brush speed (0-100)
/// - `side_brush`: Side brush speed (0-100)
/// - `water_pump`: Water pump speed for 2-in-1 mop box (0-100)
/// - `motor_mode_set`: Whether motor mode 0x02 (navigation) is currently active
/// - `lidar_enabled`: Whether lidar motor should be spinning
/// - `linear_velocity`: Forward/backward velocity in mm/s (signed)
/// - `angular_velocity`: Rotation velocity in mrad/s (signed)
/// - `wheel_motor_enabled`: Explicit flag to keep mode 0x02 active even without motion
///
/// ## Lidar Auto-Tuning State
///
/// The lidar PWM is automatically discovered using an adaptive binary-search algorithm:
/// - Starts at 100% PWM and ramps down with 20% steps
/// - Halves step size on each direction reversal (scan state changes)
/// - Converges when step size reaches 2% and scans are stable
/// - Stops sending PWM commands once stable
pub struct ComponentState {
    pub vacuum: AtomicU8,
    pub main_brush: AtomicU8,
    pub side_brush: AtomicU8,
    pub water_pump: AtomicU8,
    pub motor_mode_set: AtomicBool,
    pub lidar_enabled: AtomicBool,
    pub linear_velocity: AtomicI16,
    pub angular_velocity: AtomicI16,
    pub wheel_motor_enabled: AtomicBool,

    // Lidar auto-tuning state
    pub lidar_current_pwm: AtomicU8,
    pub lidar_step_size: AtomicU8,
    pub lidar_tuning_phase: AtomicU8,
    pub lidar_last_pwm_change_ms: AtomicU64,
    pub lidar_max_working_pwm: AtomicU8, // Highest PWM where scans worked
    pub lidar_min_failing_pwm: AtomicU8, // Lowest PWM where scans stopped

    // Shared with Delta2D driver (set via setter after construction)
    pub lidar_last_scan_ms: Arc<AtomicU64>,
}

impl Default for ComponentState {
    fn default() -> Self {
        Self {
            vacuum: AtomicU8::new(0),
            main_brush: AtomicU8::new(0),
            side_brush: AtomicU8::new(0),
            water_pump: AtomicU8::new(0),
            motor_mode_set: AtomicBool::new(false),
            lidar_enabled: AtomicBool::new(false),
            linear_velocity: AtomicI16::new(0),
            angular_velocity: AtomicI16::new(0),
            wheel_motor_enabled: AtomicBool::new(false),
            // Lidar auto-tuning defaults
            lidar_current_pwm: AtomicU8::new(INITIAL_PWM),
            lidar_step_size: AtomicU8::new(RAMP_STEP),
            lidar_tuning_phase: AtomicU8::new(PHASE_RAMP_UP),
            lidar_last_pwm_change_ms: AtomicU64::new(0),
            lidar_max_working_pwm: AtomicU8::new(0),
            lidar_min_failing_pwm: AtomicU8::new(MAX_PWM + 1),
            // Shared state (will be replaced via setter)
            lidar_last_scan_ms: Arc::new(AtomicU64::new(0)),
        }
    }
}

impl ComponentState {
    /// Clear all component states (used by emergency stop)
    pub fn clear_all(&self) {
        self.vacuum.store(0, Ordering::Relaxed);
        self.main_brush.store(0, Ordering::Relaxed);
        self.side_brush.store(0, Ordering::Relaxed);
        self.water_pump.store(0, Ordering::Relaxed);
        self.lidar_enabled.store(false, Ordering::Relaxed);
        self.linear_velocity.store(0, Ordering::Relaxed);
        self.angular_velocity.store(0, Ordering::Relaxed);
        self.wheel_motor_enabled.store(false, Ordering::Relaxed);
        self.motor_mode_set.store(false, Ordering::Relaxed);
        // Reset lidar tuning state
        self.lidar_current_pwm.store(INITIAL_PWM, Ordering::Relaxed);
        self.lidar_step_size.store(RAMP_STEP, Ordering::Relaxed);
        self.lidar_tuning_phase
            .store(PHASE_RAMP_UP, Ordering::Relaxed);
        self.lidar_last_pwm_change_ms.store(0, Ordering::Relaxed);
        self.lidar_max_working_pwm.store(0, Ordering::Relaxed);
        self.lidar_min_failing_pwm
            .store(MAX_PWM + 1, Ordering::Relaxed);
    }

    /// Check if any component is active (determines if motor mode 0x02 is needed)
    pub fn any_active(&self) -> bool {
        self.vacuum.load(Ordering::Relaxed) > 0
            || self.main_brush.load(Ordering::Relaxed) > 0
            || self.side_brush.load(Ordering::Relaxed) > 0
            || self.water_pump.load(Ordering::Relaxed) > 0
            || self.lidar_enabled.load(Ordering::Relaxed)
            || self.wheel_motor_enabled.load(Ordering::Relaxed)
    }

    /// Get current velocity values (linear_mm_s, angular_mrad_s)
    pub fn get_velocities(&self) -> (i16, i16) {
        (
            self.linear_velocity.load(Ordering::Relaxed),
            self.angular_velocity.load(Ordering::Relaxed),
        )
    }

    /// Get component speeds (vacuum, main_brush, side_brush, water_pump)
    pub fn get_component_speeds(&self) -> (u8, u8, u8, u8) {
        (
            self.vacuum.load(Ordering::Relaxed),
            self.main_brush.load(Ordering::Relaxed),
            self.side_brush.load(Ordering::Relaxed),
            self.water_pump.load(Ordering::Relaxed),
        )
    }

    /// Initialize lidar auto-tuning state for a new enable cycle
    pub fn init_lidar_tuning(&self) {
        let now = current_time_ms();
        self.lidar_current_pwm.store(INITIAL_PWM, Ordering::Relaxed);
        self.lidar_step_size.store(RAMP_STEP, Ordering::Relaxed);
        self.lidar_tuning_phase
            .store(PHASE_RAMP_UP, Ordering::Relaxed);
        self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
        self.lidar_max_working_pwm.store(0, Ordering::Relaxed);
        self.lidar_min_failing_pwm
            .store(MAX_PWM + 1, Ordering::Relaxed);
    }

    /// Update lidar PWM tuning state machine.
    ///
    /// Returns `Some(pwm)` if a PWM command should be sent, `None` if stable/settling.
    ///
    /// Algorithm to find MAXIMUM stable PWM (higher = better for SLAM):
    /// 1. PHASE_RAMP_UP: Start at 60%, increase by 5% until scans stop
    /// 2. PHASE_FINE_TUNE: Binary search between max_working and min_failing
    /// 3. PHASE_STABLE: Found optimal, stop sending PWM
    /// 4. PHASE_RECOVERY: If scans lost, decrease PWM to recover
    pub fn update_lidar_tuning(&self) -> Option<u8> {
        let now = current_time_ms();

        // Check if still settling after PWM change (wait 1 second)
        let time_since_change =
            now.saturating_sub(self.lidar_last_pwm_change_ms.load(Ordering::Relaxed));
        if time_since_change < SETTLING_TIMEOUT_MS {
            return None; // Don't change PWM while settling
        }

        // Evaluate scan success (only after settling)
        let last_scan = self.lidar_last_scan_ms.load(Ordering::Relaxed);
        let has_scans = now.saturating_sub(last_scan) < SCAN_TIMEOUT_MS;

        let phase = self.lidar_tuning_phase.load(Ordering::Relaxed);
        let current_pwm = self.lidar_current_pwm.load(Ordering::Relaxed);

        match phase {
            PHASE_RAMP_UP => {
                // Ramping up to find where scans stop working
                if has_scans {
                    // Scans working at this PWM - record as max working
                    self.lidar_max_working_pwm
                        .store(current_pwm, Ordering::Relaxed);

                    if current_pwm >= MAX_PWM {
                        // Already at max and scans work - we're done!
                        self.lidar_tuning_phase
                            .store(PHASE_STABLE, Ordering::Relaxed);
                        log::info!("Lidar stable at max PWM {}%", current_pwm);
                        return None;
                    }

                    // Try higher PWM
                    let new_pwm = (current_pwm + RAMP_STEP).min(MAX_PWM);
                    self.lidar_current_pwm.store(new_pwm, Ordering::Relaxed);
                    self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                    log::info!("Lidar ramp up: {}% -> {}% (scans OK)", current_pwm, new_pwm);
                    Some(new_pwm)
                } else {
                    // Scans not working at this PWM
                    self.lidar_min_failing_pwm
                        .store(current_pwm, Ordering::Relaxed);

                    let max_working = self.lidar_max_working_pwm.load(Ordering::Relaxed);
                    if max_working > 0 {
                        // We found the boundary - switch to fine tuning
                        self.lidar_tuning_phase
                            .store(PHASE_FINE_TUNE, Ordering::Relaxed);
                        let mid = (max_working + current_pwm) / 2;
                        self.lidar_current_pwm.store(mid, Ordering::Relaxed);
                        self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                        log::info!(
                            "Lidar found boundary: works at {}%, fails at {}%, trying {}%",
                            max_working,
                            current_pwm,
                            mid
                        );
                        Some(mid)
                    } else {
                        // Haven't found working PWM yet - try lower
                        let new_pwm = current_pwm.saturating_sub(RAMP_STEP).max(MIN_PWM);
                        if new_pwm == current_pwm {
                            // Can't go lower, stuck at minimum
                            log::error!("Lidar: no working PWM found, stuck at {}%", current_pwm);
                            return None;
                        }
                        self.lidar_current_pwm.store(new_pwm, Ordering::Relaxed);
                        self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                        log::info!(
                            "Lidar ramp down: {}% -> {}% (no scans yet)",
                            current_pwm,
                            new_pwm
                        );
                        Some(new_pwm)
                    }
                }
            }
            PHASE_FINE_TUNE => {
                // Binary search between max_working and min_failing
                if has_scans {
                    // This PWM works - update max_working
                    self.lidar_max_working_pwm
                        .store(current_pwm, Ordering::Relaxed);
                } else {
                    // This PWM fails - update min_failing
                    self.lidar_min_failing_pwm
                        .store(current_pwm, Ordering::Relaxed);
                }

                let max_working = self.lidar_max_working_pwm.load(Ordering::Relaxed);
                let min_failing = self.lidar_min_failing_pwm.load(Ordering::Relaxed);

                // Check if converged (gap <= MIN_STEP)
                if min_failing.saturating_sub(max_working) <= MIN_STEP {
                    // Converged! Use max_working as final value
                    self.lidar_current_pwm.store(max_working, Ordering::Relaxed);
                    self.lidar_tuning_phase
                        .store(PHASE_STABLE, Ordering::Relaxed);
                    self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                    log::info!("Lidar tuning complete: optimal PWM = {}%", max_working);
                    return Some(max_working);
                }

                // Binary search: try midpoint
                let mid = (max_working + min_failing) / 2;
                self.lidar_current_pwm.store(mid, Ordering::Relaxed);
                self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                log::debug!(
                    "Lidar fine tune: trying {}% (range {}%-{}%)",
                    mid,
                    max_working,
                    min_failing
                );
                Some(mid)
            }
            PHASE_STABLE => {
                if has_scans {
                    None // All good, don't send PWM
                } else {
                    // Lost scans unexpectedly - enter recovery
                    self.lidar_tuning_phase
                        .store(PHASE_RECOVERY, Ordering::Relaxed);
                    log::warn!(
                        "Lidar lost scans at PWM {}%, entering recovery",
                        current_pwm
                    );
                    // Decrease PWM to try to recover
                    let new_pwm = current_pwm.saturating_sub(RECOVERY_STEP).max(MIN_PWM);
                    self.lidar_current_pwm.store(new_pwm, Ordering::Relaxed);
                    self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                    Some(new_pwm)
                }
            }
            PHASE_RECOVERY => {
                if has_scans {
                    // Recovered! Return to stable
                    self.lidar_tuning_phase
                        .store(PHASE_STABLE, Ordering::Relaxed);
                    log::info!("Lidar recovered at PWM {}%", current_pwm);
                    None
                } else {
                    // Still no scans, decrease PWM further
                    let new_pwm = current_pwm.saturating_sub(RECOVERY_STEP).max(MIN_PWM);
                    if new_pwm == current_pwm {
                        log::error!("Lidar: cannot recover, stuck at min PWM {}%", current_pwm);
                        return None;
                    }
                    self.lidar_current_pwm.store(new_pwm, Ordering::Relaxed);
                    self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                    log::debug!("Lidar recovery: {} -> {}%", current_pwm, new_pwm);
                    Some(new_pwm)
                }
            }
            _ => None,
        }
    }

    /// Get current lidar PWM value (for initial enable command)
    pub fn get_lidar_pwm(&self) -> u8 {
        self.lidar_current_pwm.load(Ordering::Relaxed)
    }
}
