//! Component state management for GD32 driver
//!
//! This module defines the shared state used by the heartbeat thread to refresh
//! component commands every 20ms. All fields use atomic types to allow lockless reads.

use std::sync::atomic::{AtomicBool, AtomicI16, AtomicU8, Ordering};

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
/// - `lidar_pwm`: Lidar motor PWM percentage (0-100)
/// - `linear_velocity`: Forward/backward velocity in mm/s (signed)
/// - `angular_velocity`: Rotation velocity in mrad/s (signed)
/// - `wheel_motor_enabled`: Explicit flag to keep mode 0x02 active even without motion
#[derive(Default)]
pub struct ComponentState {
    pub vacuum: AtomicU8,
    pub main_brush: AtomicU8,
    pub side_brush: AtomicU8,
    pub water_pump: AtomicU8,
    pub motor_mode_set: AtomicBool,
    pub lidar_enabled: AtomicBool,
    pub lidar_pwm: AtomicU8,
    pub linear_velocity: AtomicI16,
    pub angular_velocity: AtomicI16,
    pub wheel_motor_enabled: AtomicBool,
}

impl ComponentState {
    /// Clear all component states (used by emergency stop)
    pub fn clear_all(&self) {
        self.vacuum.store(0, Ordering::Relaxed);
        self.main_brush.store(0, Ordering::Relaxed);
        self.side_brush.store(0, Ordering::Relaxed);
        self.water_pump.store(0, Ordering::Relaxed);
        self.lidar_enabled.store(false, Ordering::Relaxed);
        self.lidar_pwm.store(0, Ordering::Relaxed);
        self.linear_velocity.store(0, Ordering::Relaxed);
        self.angular_velocity.store(0, Ordering::Relaxed);
        self.wheel_motor_enabled.store(false, Ordering::Relaxed);
        self.motor_mode_set.store(false, Ordering::Relaxed);
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
}
