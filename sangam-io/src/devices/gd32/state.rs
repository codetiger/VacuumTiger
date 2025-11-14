//! GD32 device state management

use parking_lot::Mutex;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::time::Instant;

/// Connection state for the GD32 device
#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionState {
    /// Waiting for first STATUS_DATA packet
    Initializing,
    /// Receiving packets normally
    Connected,
    /// Device in sleep mode, attempting wake
    SleepMode,
}

/// Command state (written by main thread, read by WRITE thread)
#[derive(Debug, Clone)]
pub struct CommandState {
    /// Current motor control mode (0x01 = direct, 0x02 = velocity)
    pub motor_mode: u8,
    /// Target left motor speed (ticks per cycle)
    pub target_left: i32,
    /// Target right motor speed (ticks per cycle)
    pub target_right: i32,
    /// Last side brush speed sent (for detecting 0→non-zero transitions)
    pub last_side_brush: u8,
    /// Last rolling brush speed sent (for detecting 0→non-zero transitions)
    pub last_rolling_brush: u8,
    /// Last air pump/blower speed sent (for detecting 0→non-zero transitions)
    pub last_air_pump: u16,
    /// Last lidar PWM sent (for detecting enable/disable transitions)
    pub last_lidar_pwm: i32,
    /// Lidar power state (for detecting power on/off transitions)
    pub lidar_powered: bool,
}

/// Telemetry state (written by READ thread, read by main thread)
#[derive(Debug, Clone, Default)]
pub struct TelemetryState {
    // Battery Information (all Option<T> to distinguish "no data" from "zero")
    /// Battery charge level (0-100%)
    pub battery_level: Option<u8>,

    // Encoder Counts (raw values as received, no wraparound handling)
    /// Left encoder count (cumulative ticks)
    pub encoder_left: Option<i32>,
    /// Right encoder count (cumulative ticks)
    pub encoder_right: Option<i32>,

    // Error and Status Flags
    /// Error code from device
    pub error_code: Option<u8>,
    /// Status flag from device
    #[allow(dead_code)] // Used in telemetry streaming
    pub status_flag: Option<u8>,
    /// Charging flag (indicates battery is charging)
    pub charging_flag: Option<bool>,
    /// Battery state flag
    #[allow(dead_code)] // Used in telemetry streaming
    pub battery_state_flag: Option<bool>,
    /// Percent value ÷ 100
    #[allow(dead_code)] // Used in telemetry streaming
    pub percent_value: Option<f32>,

    // IR Proximity Sensors (Button Detection)
    /// IR sensor 1 (×5 scaling)
    pub ir_sensor_1: Option<u16>,
    /// Start button IR sensor (×5 scaling, 100-199 = pressed)
    pub start_button_ir: Option<u16>,
    /// Dock button IR sensor (×5 scaling, 100-199 = pressed)
    pub dock_button_ir: Option<u16>,

    // Bumper Sensor
    /// Bumper sensor (any contact detected)
    /// true = bumper pressed, false = no contact
    /// Note: Protocol byte position still under investigation
    pub bumper_triggered: Option<bool>,
}

/// Diagnostic counters using atomics for lock-free access
pub struct DiagnosticCounters {
    /// Consecutive lost packet count (for error detection)
    pub lost_packet_count: AtomicU32,
    /// Total packets received (diagnostics)
    pub total_rx_packets: AtomicU64,
    /// Total packets transmitted (diagnostics)
    pub total_tx_packets: AtomicU64,
    /// Last successful receive timestamp (protected by mutex for non-atomic type)
    pub last_rx_time: Mutex<Instant>,
    /// Sleep mode flag (matches g_sleep_mode @ 0x81baf in AuxCtrl)
    pub sleep_mode: AtomicBool,
}

/// GD32 device state with proper separation of concerns
///
/// Shared between main thread and heartbeat threads
pub struct Gd32State {
    /// Command state (written by main thread)
    pub command: Arc<Mutex<CommandState>>,

    /// Telemetry state (written by READ thread)
    /// None until first STATUS_DATA packet received
    pub telemetry: Arc<Mutex<Option<TelemetryState>>>,

    /// Timestamp of last telemetry update (for staleness detection)
    /// None until first STATUS_DATA packet received
    pub telemetry_timestamp: Arc<Mutex<Option<Instant>>>,

    /// Connection state tracking
    pub connection_state: Arc<Mutex<ConnectionState>>,

    /// Diagnostic counters (lock-free atomics)
    pub diagnostics: Arc<DiagnosticCounters>,
}

impl Default for CommandState {
    fn default() -> Self {
        Self {
            motor_mode: 0x00, // Start in initialization mode, switch to 0x02 after init completes
            target_left: 0,
            target_right: 0,
            last_side_brush: 0,
            last_rolling_brush: 0,
            last_air_pump: 0,
            last_lidar_pwm: 0,
            lidar_powered: false,
        }
    }
}

impl Default for DiagnosticCounters {
    fn default() -> Self {
        Self {
            lost_packet_count: AtomicU32::new(0),
            total_rx_packets: AtomicU64::new(0),
            total_tx_packets: AtomicU64::new(0),
            last_rx_time: Mutex::new(Instant::now()),
            sleep_mode: AtomicBool::new(false),
        }
    }
}

impl Default for Gd32State {
    fn default() -> Self {
        Self {
            command: Arc::new(Mutex::new(CommandState::default())),
            telemetry: Arc::new(Mutex::new(None)),
            telemetry_timestamp: Arc::new(Mutex::new(None)),
            connection_state: Arc::new(Mutex::new(ConnectionState::Initializing)),
            diagnostics: Arc::new(DiagnosticCounters::default()),
        }
    }
}

impl Gd32State {
    // TODO: These methods were used by the removed getter API - keep for potential future use

    /// Get telemetry age if telemetry has been received
    #[allow(dead_code)]
    pub fn telemetry_age(&self) -> Option<std::time::Duration> {
        self.telemetry_timestamp
            .lock()
            .as_ref()
            .map(|timestamp| timestamp.elapsed())
    }

    /// Check if telemetry is fresh (not stale)
    #[allow(dead_code)]
    pub fn is_telemetry_fresh(&self, max_age: std::time::Duration) -> bool {
        self.telemetry_age()
            .map(|age| age <= max_age)
            .unwrap_or(false)
    }

    /// Update telemetry from STATUS_DATA packet
    pub fn update_telemetry(&self, telemetry: TelemetryState) {
        let now = Instant::now();

        // Update telemetry data
        *self.telemetry.lock() = Some(telemetry);

        // Update timestamp
        *self.telemetry_timestamp.lock() = Some(now);

        // Update connection state if this is first packet
        let mut conn_state = self.connection_state.lock();
        if *conn_state == ConnectionState::Initializing {
            *conn_state = ConnectionState::Connected;
            log::info!("GD32: First STATUS_DATA received, connection established");
        }

        // Reset lost packet counter on successful receive
        self.diagnostics
            .lost_packet_count
            .store(0, Ordering::Relaxed);

        // Update last receive time
        *self.diagnostics.last_rx_time.lock() = now;

        // Increment receive counter
        self.diagnostics
            .total_rx_packets
            .fetch_add(1, Ordering::Relaxed);
    }
}
