//! Component state management for GD32 driver
//!
//! This module defines the shared state used by the heartbeat thread to refresh
//! component commands every 20ms. All fields use atomic types to allow lockless reads.

use std::sync::atomic::{AtomicBool, AtomicI16, AtomicU8, Ordering};

/// Default lidar PWM (60% gives ~330 RPM / 5.5Hz scan rate)
const DEFAULT_LIDAR_PWM: u8 = 60;

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
/// - `lidar_pwm`: Static PWM value for lidar motor (0-100)
/// - `linear_velocity`: Forward/backward velocity in mm/s (signed)
/// - `angular_velocity`: Rotation velocity in mrad/s (signed)
/// - `wheel_motor_enabled`: Explicit flag to keep mode 0x02 active even without motion
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
    /// Create a new ComponentState with custom initial lidar PWM
    pub fn new(lidar_pwm: u8) -> Self {
        Self {
            vacuum: AtomicU8::new(0),
            main_brush: AtomicU8::new(0),
            side_brush: AtomicU8::new(0),
            water_pump: AtomicU8::new(0),
            motor_mode_set: AtomicBool::new(false),
            lidar_enabled: AtomicBool::new(false),
            lidar_pwm: AtomicU8::new(lidar_pwm.min(100)),
            linear_velocity: AtomicI16::new(0),
            angular_velocity: AtomicI16::new(0),
            wheel_motor_enabled: AtomicBool::new(false),
        }
    }

    /// Clear all component states (used by emergency stop)
    pub fn clear_all(&self) {
        self.vacuum.store(0, Ordering::Relaxed);
        self.main_brush.store(0, Ordering::Relaxed);
        self.side_brush.store(0, Ordering::Relaxed);
        self.water_pump.store(0, Ordering::Relaxed);
        self.lidar_enabled.store(false, Ordering::Relaxed);
        self.lidar_pwm.store(DEFAULT_LIDAR_PWM, Ordering::Relaxed);
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

    /// Get current lidar PWM value (set from config during initialization)
    pub fn get_lidar_pwm(&self) -> u8 {
        self.lidar_pwm.load(Ordering::Relaxed)
    }
}

impl Default for ComponentState {
    fn default() -> Self {
        Self::new(DEFAULT_LIDAR_PWM)
    }
}
