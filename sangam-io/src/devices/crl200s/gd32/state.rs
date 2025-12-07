//! Component state management for GD32 driver
//!
//! This module defines the shared state used by the heartbeat thread to refresh
//! component commands every 20ms. All fields use atomic types to allow lockless reads.

use std::sync::atomic::{AtomicBool, AtomicI16, AtomicI8, AtomicU64, AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

// Lidar auto-tuning constants
const SCAN_TIMEOUT_MS: u64 = 200; // Consider "not receiving" if no scan for 200ms
const SETTLING_TIMEOUT_MS: u64 = 250; // Wait after PWM change before evaluating
const SETTLING_HEALTH_COUNT: u8 = 3; // OR wait for this many health packets
const INITIAL_STEP: u8 = 20; // Start with 20% step size
const MIN_STEP: u8 = 2; // Converge when step reaches 2%
const MIN_PWM: u8 = 30; // Never go below 30%
const MAX_PWM: u8 = 100; // Never exceed 100%
const RECOVERY_STEP: u8 = 5; // Fixed step for recovery mode

// Lidar tuning phases
const PHASE_RAMPING: u8 = 0; // Adaptive stepping to find max stable PWM
const PHASE_STABLE: u8 = 1; // Optimal found, not sending PWM
const PHASE_RECOVERY: u8 = 2; // Lost scans, recovering

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
    pub lidar_direction: AtomicI8, // +1 = increasing, -1 = decreasing
    pub lidar_tuning_phase: AtomicU8,
    pub lidar_last_pwm_change_ms: AtomicU64,
    pub lidar_last_had_scans: AtomicBool,

    // Shared with Delta2D driver (set via setters after construction)
    pub lidar_last_scan_ms: Arc<AtomicU64>,
    pub lidar_health_count: Arc<AtomicU8>,
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
            lidar_current_pwm: AtomicU8::new(MAX_PWM),
            lidar_step_size: AtomicU8::new(INITIAL_STEP),
            lidar_direction: AtomicI8::new(-1), // Start decreasing
            lidar_tuning_phase: AtomicU8::new(PHASE_RAMPING),
            lidar_last_pwm_change_ms: AtomicU64::new(0),
            lidar_last_had_scans: AtomicBool::new(false),
            // Shared state (will be replaced via setters)
            lidar_last_scan_ms: Arc::new(AtomicU64::new(0)),
            lidar_health_count: Arc::new(AtomicU8::new(0)),
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
        self.lidar_current_pwm.store(MAX_PWM, Ordering::Relaxed);
        self.lidar_step_size.store(INITIAL_STEP, Ordering::Relaxed);
        self.lidar_direction.store(-1, Ordering::Relaxed);
        self.lidar_tuning_phase.store(PHASE_RAMPING, Ordering::Relaxed);
        self.lidar_last_pwm_change_ms.store(0, Ordering::Relaxed);
        self.lidar_last_had_scans.store(false, Ordering::Relaxed);
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
        self.lidar_current_pwm.store(MAX_PWM, Ordering::Relaxed);
        self.lidar_step_size.store(INITIAL_STEP, Ordering::Relaxed);
        self.lidar_direction.store(-1, Ordering::Relaxed); // Start decreasing
        self.lidar_tuning_phase.store(PHASE_RAMPING, Ordering::Relaxed);
        self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
        self.lidar_last_had_scans.store(false, Ordering::Relaxed);
        self.lidar_health_count.store(0, Ordering::Relaxed);
    }

    /// Update lidar PWM tuning state machine.
    ///
    /// Returns `Some(pwm)` if a PWM command should be sent, `None` if stable/settling.
    ///
    /// This implements an adaptive binary-search algorithm:
    /// - Starts at 100% PWM and ramps down with large steps (20%)
    /// - Halves step size on each direction reversal (when scan state changes)
    /// - Converges when step size reaches MIN_STEP and scans are stable
    /// - Waits for settling period (250ms or 3 health packets) after each PWM change
    pub fn update_lidar_tuning(&self) -> Option<u8> {
        let now = current_time_ms();

        // Check if still settling after PWM change
        let time_since_change = now.saturating_sub(self.lidar_last_pwm_change_ms.load(Ordering::Relaxed));
        let health_packets = self.lidar_health_count.load(Ordering::Relaxed);
        let is_settling =
            time_since_change < SETTLING_TIMEOUT_MS && health_packets < SETTLING_HEALTH_COUNT;

        if is_settling {
            return None; // Don't change PWM while settling
        }

        // Evaluate scan success (only after settling)
        let last_scan = self.lidar_last_scan_ms.load(Ordering::Relaxed);
        let has_scans = now.saturating_sub(last_scan) < SCAN_TIMEOUT_MS;

        let phase = self.lidar_tuning_phase.load(Ordering::Relaxed);

        match phase {
            PHASE_RAMPING => {
                let prev_had_scans = self.lidar_last_had_scans.swap(has_scans, Ordering::Relaxed);

                // Detect direction change (scan state flipped)
                if has_scans != prev_had_scans {
                    // Direction reversal - halve step size
                    let mut step = self.lidar_step_size.load(Ordering::Relaxed);
                    step = (step / 2).max(MIN_STEP);
                    self.lidar_step_size.store(step, Ordering::Relaxed);

                    // Flip direction
                    let dir = self.lidar_direction.load(Ordering::Relaxed);
                    self.lidar_direction.store(-dir, Ordering::Relaxed);

                    let pwm = self.lidar_current_pwm.load(Ordering::Relaxed);
                    log::info!(
                        "Lidar PWM direction reversal at {}%, new step={}%",
                        pwm,
                        step
                    );
                }

                let step = self.lidar_step_size.load(Ordering::Relaxed);

                // Check if converged (small step + stable scans)
                if step <= MIN_STEP && has_scans {
                    self.lidar_tuning_phase.store(PHASE_STABLE, Ordering::Relaxed);
                    let pwm = self.lidar_current_pwm.load(Ordering::Relaxed);
                    log::info!("Lidar stable at PWM {}%", pwm);
                    return None; // Stop sending PWM
                }

                // Apply step in current direction
                let dir = self.lidar_direction.load(Ordering::Relaxed);
                let current_pwm = self.lidar_current_pwm.load(Ordering::Relaxed);
                let new_pwm = if dir < 0 {
                    current_pwm.saturating_sub(step).max(MIN_PWM)
                } else {
                    current_pwm.saturating_add(step).min(MAX_PWM)
                };

                self.lidar_current_pwm.store(new_pwm, Ordering::Relaxed);
                self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                self.lidar_health_count.store(0, Ordering::Relaxed);

                log::debug!(
                    "Lidar PWM: {} -> {}% (step={}%, dir={})",
                    current_pwm,
                    new_pwm,
                    step,
                    if dir < 0 { "down" } else { "up" }
                );

                Some(new_pwm)
            }
            PHASE_STABLE => {
                if has_scans {
                    None // All good, don't send PWM
                } else {
                    // Lost scans unexpectedly
                    self.lidar_tuning_phase.store(PHASE_RECOVERY, Ordering::Relaxed);
                    let pwm = self.lidar_current_pwm.load(Ordering::Relaxed);
                    log::warn!("Lidar lost scans at PWM {}%, entering recovery", pwm);
                    self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                    self.lidar_health_count.store(0, Ordering::Relaxed);
                    Some(pwm)
                }
            }
            PHASE_RECOVERY => {
                if has_scans {
                    // Recovered! Return to stable
                    self.lidar_tuning_phase.store(PHASE_STABLE, Ordering::Relaxed);
                    let pwm = self.lidar_current_pwm.load(Ordering::Relaxed);
                    log::info!("Lidar recovered at PWM {}%", pwm);
                    None
                } else {
                    // Still no scans, increase PWM
                    let current_pwm = self.lidar_current_pwm.load(Ordering::Relaxed);
                    let new_pwm = current_pwm.saturating_add(RECOVERY_STEP).min(MAX_PWM);
                    self.lidar_current_pwm.store(new_pwm, Ordering::Relaxed);
                    self.lidar_last_pwm_change_ms.store(now, Ordering::Relaxed);
                    self.lidar_health_count.store(0, Ordering::Relaxed);
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
