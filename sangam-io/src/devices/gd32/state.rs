//! GD32 device state management

/// GD32 device state
///
/// Shared between main thread and heartbeat thread
#[derive(Debug, Clone, Default)]
pub struct Gd32State {
    /// Target left motor speed (ticks per cycle)
    pub target_left: i32,
    /// Target right motor speed (ticks per cycle)
    pub target_right: i32,
    /// Current battery voltage (V)
    pub battery_voltage: f32,
    /// Current battery current (A)
    pub battery_current: f32,
    /// Battery charge level (0-100%)
    pub battery_level: u8,
    /// Left encoder count (cumulative ticks)
    pub encoder_left: i32,
    /// Right encoder count (cumulative ticks)
    pub encoder_right: i32,
    /// Error code from device
    pub error_code: u8,
}
