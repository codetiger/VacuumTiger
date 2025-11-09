//! GD32 device state management

use std::time::Instant;

/// GD32 device state
///
/// Shared between main thread and heartbeat thread
#[derive(Debug, Clone)]
pub struct Gd32State {
    /// Target left motor speed (ticks per cycle)
    pub target_left: i32,
    /// Target right motor speed (ticks per cycle)
    pub target_right: i32,

    // Battery Information
    /// Current battery voltage (V)
    pub battery_voltage: f32,
    /// Current battery current (A)
    pub battery_current: f32,
    /// Battery charge level (0-100%)
    pub battery_level: u8,

    // Encoder Counts
    /// Left encoder count (cumulative ticks)
    pub encoder_left: i32,
    /// Right encoder count (cumulative ticks)
    pub encoder_right: i32,

    // Error and Status Flags
    /// Error code from device
    pub error_code: u8,
    /// Status flag from device
    pub status_flag: u8,
    /// Charging flag (indicates battery is charging)
    pub charging_flag: bool,
    /// Battery state flag
    pub battery_state_flag: bool,
    /// Percent value ÷ 100
    pub percent_value: f32,

    // IR Proximity Sensors (Button Detection)
    /// IR sensor 1 (×5 scaling)
    pub ir_sensor_1: u16,
    /// Point button IR sensor (×5 scaling, 100-199 = pressed)
    pub point_button_ir: u16,
    /// Dock button IR sensor (×5 scaling, 100-199 = pressed)
    pub dock_button_ir: u16,

    // Packet tracking (matches AuxCtrl architecture)
    /// Consecutive lost packet count (for error detection)
    pub lost_packet_count: u32,
    /// Total packets received (diagnostics)
    pub total_rx_packets: u64,
    /// Total packets transmitted (diagnostics)
    pub total_tx_packets: u64,
    /// Last successful receive timestamp
    pub last_rx_time: Instant,
    /// Sleep mode flag (matches g_sleep_mode @ 0x81baf in AuxCtrl)
    /// When true, GD32 is in sleep mode and needs wake sequence
    pub sleep_mode: bool,
}

impl Default for Gd32State {
    fn default() -> Self {
        Self {
            target_left: 0,
            target_right: 0,
            battery_voltage: 0.0,
            battery_current: 0.0,
            battery_level: 0,
            encoder_left: 0,
            encoder_right: 0,
            error_code: 0,
            status_flag: 0,
            charging_flag: false,
            battery_state_flag: false,
            percent_value: 0.0,
            ir_sensor_1: 0,
            point_button_ir: 0,
            dock_button_ir: 0,
            lost_packet_count: 0,
            total_rx_packets: 0,
            total_tx_packets: 0,
            last_rx_time: Instant::now(),
            sleep_mode: false,
        }
    }
}
