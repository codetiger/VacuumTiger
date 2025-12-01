//! DeviceDriver trait - the core abstraction for adding new robot hardware.
//!
//! To add a new device, implement this trait and register it in [`crate::devices::create_device`].
//! See [`crate::devices::crl200s::CRL200SDriver`] for a complete implementation example.

use crate::core::types::{Command, SensorGroupData};
use crate::error::Result;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Hardware abstraction trait for robot devices.
///
/// # Lifecycle
/// 1. Created via [`crate::devices::create_device`] based on config
/// 2. [`initialize`](Self::initialize) called once at daemon startup
/// 3. [`send_command`](Self::send_command) called for each inbound TCP command
/// 4. `Drop` triggers cleanup when daemon shuts down
///
/// # Threading
/// Drivers typically spawn internal threads for:
/// - **Heartbeat**: Safety-critical timing loop (e.g., motor watchdog)
/// - **Reader**: Continuous sensor data parsing
///
/// Use `Arc<Mutex<>>` for state shared across threads, `Arc<AtomicBool>` for shutdown signals.
///
/// # Example Structure
/// ```ignore
/// pub struct MyDeviceDriver {
///     port: Arc<Mutex<Box<dyn SerialPort>>>,  // Shared serial access
///     shutdown: Arc<AtomicBool>,               // Thread shutdown signal
///     reader_handle: Option<JoinHandle<()>>,   // Thread handle for cleanup
/// }
///
/// impl DeviceDriver for MyDeviceDriver {
///     fn initialize(&mut self) -> Result<HashMap<String, Arc<Mutex<SensorGroupData>>>> {
///         // 1. Create sensor groups
///         // 2. Spawn reader/heartbeat threads
///         // 3. Return sensor map for TCP streaming
///     }
///     fn send_command(&mut self, cmd: Command) -> Result<()> {
///         // Handle ComponentControl commands
///     }
/// }
///
/// impl Drop for MyDeviceDriver {
///     fn drop(&mut self) {
///         self.shutdown.store(true, Ordering::Relaxed);
///         // Join threads...
///     }
/// }
/// ```
pub trait DeviceDriver: Send {
    /// Initialize hardware and start background threads.
    ///
    /// Returns sensor groups keyed by group ID (e.g., "gd32_status", "lidar").
    /// The TCP publisher uses these to stream sensor data to clients.
    fn initialize(&mut self) -> Result<HashMap<String, Arc<Mutex<SensorGroupData>>>>;

    /// Process a command from TCP clients.
    ///
    /// Most commands are [`Command::ComponentControl`] targeting specific components.
    /// Return `Err` for invalid commands; the error is logged but doesn't stop the daemon.
    fn send_command(&mut self, cmd: Command) -> Result<()>;
}
