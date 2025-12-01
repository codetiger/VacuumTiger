//! GD32 motor controller driver
//!
//! This module manages communication with the GD32F103 microcontroller that controls
//! the CRL-200S robot's motors, components, and sensors.
//!
//! # Architecture
//!
//! The driver uses a multi-threaded design for real-time operation:
//!
//! ## Thread Model
//!
//! 1. **Heartbeat Thread** (20ms cycle, safety-critical):
//!    - Sends continuous commands to maintain GD32 watchdog timer
//!    - Manages motor mode state machine (0x00 idle ↔ 0x02 navigation)
//!    - Sends velocity commands when motors active
//!    - Refreshes component states (vacuum, brushes, lidar PWM)
//!    - Uses blocking mutex locks to guarantee timing
//!
//! 2. **Reader Thread** (continuous):
//!    - Parses incoming status packets (CMD=0x15 @ ~500Hz)
//!    - Updates shared sensor data in-place (no allocations)
//!    - Requests version info after first packet
//!    - Handles protocol synchronization and checksums
//!
//! ## Motor Mode State Machine
//!
//! The GD32 requires motor mode 0x02 to be set before any components work:
//!
//! ```text
//! ┌─────────────┐  any_component_active  ┌──────────────────┐
//! │ Mode 0x00   │ ──────────────────────▶│ Mode 0x02        │
//! │ (Idle)      │                        │ (Navigation)     │
//! │             │◀────────────────────── │                  │
//! └─────────────┘  all_components_off    └──────────────────┘
//!     • Regular heartbeat (0x06)            • Velocity (0x66)
//!                                            • Components enabled
//! ```
//!
//! ## Synchronization Strategy
//!
//! While the architecture objectives mention "lock-free streaming", this module uses
//! mutexes pragmatically for two reasons:
//!
//! 1. **Serial port exclusivity**: Only one thread can write at a time
//! 2. **Real-time guarantees**: Blocking locks in heartbeat ensure commands always sent
//!
//! Critical sections are kept minimal (<1ms) and released before sleeping.
//! Atomic types (`AtomicU8`, `AtomicI16`, `AtomicBool`) handle lockless state reads
//! for velocity/component values.
//!
//! ## Safety Properties
//!
//! - Heartbeat never stops while driver active (20-50ms requirement)
//! - All velocities/components zeroed on shutdown
//! - Motor mode 0x02 sent periodically to prevent timeout
//! - Emergency stop clears all states and disables motor mode

// Submodules
mod commands;
mod heartbeat;
pub mod packet;
pub mod protocol;
mod reader;
mod ring_buffer;
mod state;

// Public re-exports
pub use state::ComponentState;

// Internal imports
use crate::core::types::{Command, SensorGroupData};
use crate::devices::crl200s::constants::{INIT_RETRY_DELAY_MS, SERIAL_READ_TIMEOUT_MS};
use crate::error::{Error, Result};
use packet::{initialize_packet, TxPacket};
use serialport::SerialPort;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// GD32 motor controller driver with heartbeat and reader threads.
pub struct GD32Driver {
    /// Serial port shared between heartbeat (write) and reader (read) threads
    port: Arc<Mutex<Box<dyn SerialPort>>>,
    /// Heartbeat interval from config (typically 20ms)
    heartbeat_interval_ms: u64,
    /// Shutdown signal - set to true to stop all threads
    shutdown: Arc<AtomicBool>,
    /// Heartbeat thread handle - joined on shutdown
    heartbeat_handle: Option<JoinHandle<()>>,
    /// Reader thread handle - joined on shutdown
    reader_handle: Option<JoinHandle<()>>,
    /// Component states (velocity, brushes, etc.) - atomic for lockless heartbeat reads
    component_state: Arc<ComponentState>,
}

impl GD32Driver {
    /// Create a new GD32 driver
    pub fn new(port_path: &str, heartbeat_interval_ms: u64) -> Result<Self> {
        let port = serialport::new(port_path, 115200)
            .timeout(Duration::from_millis(SERIAL_READ_TIMEOUT_MS))
            .open()
            .map_err(Error::Serial)?;

        Ok(Self {
            port: Arc::new(Mutex::new(port)),
            heartbeat_interval_ms,
            shutdown: Arc::new(AtomicBool::new(false)),
            heartbeat_handle: None,
            reader_handle: None,
            component_state: Arc::new(ComponentState::default()),
        })
    }

    /// Initialize the GD32 device by sending init commands
    ///
    /// Version is requested by reader thread after first packet received.
    pub fn initialize(&mut self) -> Result<()> {
        log::info!("Initializing GD32 device...");

        // Send init commands (like sangam-io2-backup approach)
        // Don't wait for response here - reader thread will handle it
        let init_pkt = initialize_packet();

        for attempt in 1..=5 {
            {
                let mut port = self.port.lock().map_err(|_| Error::MutexPoisoned)?;
                if let Err(e) = init_pkt.send_to(&mut *port) {
                    log::warn!("Init send failed (attempt {}): {}", attempt, e);
                }
            }
            thread::sleep(Duration::from_millis(INIT_RETRY_DELAY_MS));
        }

        log::info!("GD32 initialization sequence sent");
        Ok(())
    }

    /// Start heartbeat and reader threads
    pub fn start(
        &mut self,
        sensor_data: Arc<Mutex<SensorGroupData>>,
        version_data: Option<Arc<Mutex<SensorGroupData>>>,
    ) -> Result<()> {
        let shutdown = Arc::clone(&self.shutdown);
        let port = Arc::clone(&self.port);
        let interval_ms = self.heartbeat_interval_ms;
        let component_state = Arc::clone(&self.component_state);

        // Start heartbeat thread
        let heartbeat_shutdown = Arc::clone(&shutdown);
        let heartbeat_port = Arc::clone(&port);
        let heartbeat_components = Arc::clone(&component_state);
        self.heartbeat_handle = Some(
            thread::Builder::new()
                .name("gd32-heartbeat".to_string())
                .spawn(move || {
                    heartbeat::heartbeat_loop(
                        heartbeat_port,
                        heartbeat_shutdown,
                        interval_ms,
                        heartbeat_components,
                    );
                })
                .map_err(|e| Error::Other(format!("Failed to spawn heartbeat thread: {}", e)))?,
        );

        // Start reader thread
        let reader_shutdown = Arc::clone(&shutdown);
        let reader_port = Arc::clone(&port);
        self.reader_handle = Some(
            thread::Builder::new()
                .name("gd32-reader".to_string())
                .spawn(move || {
                    reader::reader_loop(reader_port, reader_shutdown, sensor_data, version_data);
                })
                .map_err(|e| Error::Other(format!("Failed to spawn reader thread: {}", e)))?,
        );

        log::info!("GD32 driver started");
        Ok(())
    }

    /// Send a command to the GD32
    pub fn send_command(&self, cmd: Command) -> Result<()> {
        commands::send_command(&self.port, &self.component_state, &self.shutdown, cmd)
    }

    /// Shutdown the driver
    pub fn shutdown(&mut self) -> Result<()> {
        log::info!("Shutting down GD32 driver...");
        self.shutdown.store(true, Ordering::Relaxed);

        // Wait for threads to finish
        if let Some(handle) = self.heartbeat_handle.take() {
            handle.join().map_err(|_| Error::ThreadPanic)?;
        }
        if let Some(handle) = self.reader_handle.take() {
            handle.join().map_err(|_| Error::ThreadPanic)?;
        }

        // Send stop command
        let mut stop_pkt = TxPacket::new();
        stop_pkt.set_velocity(0, 0);
        if let Ok(mut port) = self.port.lock() {
            let _ = stop_pkt.send_to(&mut *port);
        }

        log::info!("GD32 driver shutdown complete");
        Ok(())
    }
}

impl Drop for GD32Driver {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}
