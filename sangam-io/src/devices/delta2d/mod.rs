//! 3iRobotix Delta-2D Lidar driver

mod protocol;

use super::LidarDriver;
use crate::error::{Error, Result};
use crate::transport::Transport;
use crate::types::LidarScan;

use parking_lot::Mutex;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

pub use protocol::CommandType;

/// Type alias for the lidar scan callback
type ScanCallback = Box<dyn Fn(&LidarScan) + Send>;

/// 3iRobotix Delta-2D Lidar driver
///
/// Communicates via UART at 115200 baud. Power control is handled externally
/// via GD32 CMD=0x97 (Lidar Power command).
///
/// This driver owns a background thread that continuously reads scan data
/// and invokes a registered callback when new scans are available.
pub struct Delta2DDriver {
    /// Transport layer for serial I/O (shared with read thread)
    transport: Arc<Mutex<Box<dyn Transport>>>,

    /// Read thread handle
    read_thread: Option<JoinHandle<()>>,

    /// Shutdown flag for thread coordination
    shutdown: Arc<AtomicBool>,

    /// Single callback for scan data (None when no callback registered)
    callback: Arc<Mutex<Option<ScanCallback>>>,

    /// Scan counter for statistics
    scan_count: Arc<AtomicU64>,

    /// Error counter for statistics
    error_count: Arc<AtomicU64>,
}

impl Delta2DDriver {
    /// Create new Delta-2D lidar driver
    ///
    /// Note: The lidar motor must be powered on separately via GD32 driver
    /// using set_lidar_power(true).
    pub fn new<T: Transport + 'static>(transport: T) -> Result<Self> {
        let transport = Arc::new(Mutex::new(Box::new(transport) as Box<dyn Transport>));

        log::info!("Delta2D: Driver initialized, waiting for power-on");

        Ok(Delta2DDriver {
            transport,
            read_thread: None,
            shutdown: Arc::new(AtomicBool::new(false)),
            callback: Arc::new(Mutex::new(None)),
            scan_count: Arc::new(AtomicU64::new(0)),
            error_count: Arc::new(AtomicU64::new(0)),
        })
    }
}

impl Drop for Delta2DDriver {
    fn drop(&mut self) {
        log::debug!("Delta2D: Dropping driver, shutting down scan thread");

        // Stop scanning thread if running
        if self.read_thread.is_some()
            && let Err(e) = self.stop()
        {
            log::error!("Delta2D: Error during shutdown in drop: {}", e);
        }
    }
}

// ===== Scan Thread Functions =====

/// Read exact number of bytes from transport
fn read_exact(transport: &Arc<Mutex<Box<dyn Transport>>>, buf: &mut [u8]) -> Result<()> {
    let mut transport = transport.lock();
    let mut offset = 0;

    while offset < buf.len() {
        let read = transport.read(&mut buf[offset..])?;
        if read == 0 {
            return Err(Error::Io(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "Unexpected EOF while reading",
            )));
        }
        offset += read;
    }

    Ok(())
}

/// Try to read a scan packet from the transport
///
/// Returns Ok(Some(scan)) if measurement data received,
/// Ok(None) if health packet or no data available,
/// Err(_) on read/parse error.
fn try_read_scan(transport: &Arc<Mutex<Box<dyn Transport>>>) -> Result<Option<LidarScan>> {
    let mut transport_lock = transport.lock();

    // Check if enough data available for header
    let available = transport_lock.available()?;

    // Python uses "if ser.in_waiting > 8" - need MORE than 8 bytes
    // This ensures we have header + some payload data ready
    if available <= 8 {
        return Ok(None);
    }

    // Read header
    let mut header_buf = [0u8; 8];
    let read = transport_lock.read(&mut header_buf)?;

    if read < 8 {
        log::debug!("Delta2D: Header read incomplete: got {} bytes", read);
        return Ok(None);
    }

    let header = match protocol::PacketHeader::parse(&header_buf) {
        Ok(h) => h,
        Err(e) => {
            log::debug!("Delta2D: Failed to parse header: {}", e);
            return Err(e);
        }
    };

    // Wait for payload + CRC to arrive
    let required_bytes = header.payload_length as usize + 2; // +2 for CRC
    let wait_start = Instant::now();
    let wait_timeout = Duration::from_millis(100);

    loop {
        let available = transport_lock.available()?;
        if available >= required_bytes {
            break;
        }

        if wait_start.elapsed() > wait_timeout {
            log::debug!("Delta2D: Timeout waiting for payload");
            return Ok(None);
        }

        // Release lock briefly to let data arrive
        drop(transport_lock);
        thread::sleep(Duration::from_millis(1));
        transport_lock = transport.lock();
    }

    // Release transport lock before reading payload
    drop(transport_lock);

    // Read payload
    let mut payload = vec![0u8; header.payload_length as usize];
    read_exact(transport, &mut payload)?;

    // Read CRC (currently not validated)
    let mut crc_buf = [0u8; 2];
    read_exact(transport, &mut crc_buf)?;

    match header.command_type {
        CommandType::Measurement => {
            let scan = protocol::parse_measurement(&payload)?;
            Ok(Some(scan))
        }
        CommandType::Health => {
            // Health packets don't contain scan data
            log::debug!("Delta2D: Received health packet");
            Ok(None)
        }
    }
}

/// Scan thread main loop
fn scan_thread_loop(
    transport: Arc<Mutex<Box<dyn Transport>>>,
    callback: Arc<Mutex<Option<ScanCallback>>>,
    shutdown: Arc<AtomicBool>,
    scan_count: Arc<AtomicU64>,
    error_count: Arc<AtomicU64>,
) {
    log::info!("Delta2D: Scan thread started");

    let mut last_scan_time = Instant::now();
    let mut scan_rate_timer = Instant::now();
    let mut scan_count_at_timer = 0u64;
    let mut min_points = u32::MAX;
    let mut max_points = 0u32;

    while !shutdown.load(Ordering::Relaxed) {
        match try_read_scan(&transport) {
            Ok(Some(scan)) => {
                let count = scan_count.fetch_add(1, Ordering::Relaxed) + 1;
                let point_count = scan.points.len() as u32;

                // Track min/max points
                min_points = min_points.min(point_count);
                max_points = max_points.max(point_count);

                // Find range statistics
                let (min_range, max_range) = if !scan.points.is_empty() {
                    let distances: Vec<f32> = scan.points.iter().map(|p| p.distance).collect();
                    let min = distances
                        .iter()
                        .min_by(|a, b| a.partial_cmp(b).unwrap())
                        .copied()
                        .unwrap_or(0.0);
                    let max = distances
                        .iter()
                        .max_by(|a, b| a.partial_cmp(b).unwrap())
                        .copied()
                        .unwrap_or(0.0);
                    (min / 1000.0, max / 1000.0)
                } else {
                    (0.0, 0.0)
                };

                // Log detailed scan info every 100 scans
                if count % 100 == 0 {
                    log::debug!(
                        "Delta2D: Scan #{}: {} points, range [{:.2}m - {:.2}m]",
                        count,
                        point_count,
                        min_range,
                        max_range
                    );
                }

                // Warn about scan quality issues
                if point_count < 100 {
                    log::warn!(
                        "Delta2D: Scan quality degraded - only {} points (expected > 100)",
                        point_count
                    );
                }

                // Calculate and log scan rate every 1000 scans
                if count % 1000 == 0 {
                    let elapsed = scan_rate_timer.elapsed().as_secs_f32();
                    let scans_in_period = count - scan_count_at_timer;
                    let scan_rate = scans_in_period as f32 / elapsed;

                    let error_rate = if count > 0 {
                        (error_count.load(Ordering::Relaxed) as f32 / count as f32) * 100.0
                    } else {
                        0.0
                    };

                    log::info!(
                        "Delta2D: Statistics - {:.1} Hz, {} total scans, {} errors ({:.1}% loss), points range: {}-{}",
                        scan_rate,
                        count,
                        error_count.load(Ordering::Relaxed),
                        error_rate,
                        min_points,
                        max_points
                    );

                    // Reset statistics
                    scan_rate_timer = Instant::now();
                    scan_count_at_timer = count;
                    min_points = u32::MAX;
                    max_points = 0;
                }

                // Warn if scan rate is too low
                let scan_interval = last_scan_time.elapsed();
                if scan_interval > Duration::from_millis(300) {
                    // Expected ~250ms for 4Hz
                    log::warn!(
                        "Delta2D: Scan rate low - {:.1} Hz (expected ~4 Hz)",
                        1.0 / scan_interval.as_secs_f32()
                    );
                }
                last_scan_time = Instant::now();

                // Invoke callback if registered
                let callback_lock = callback.lock();
                if let Some(ref cb) = *callback_lock {
                    // Protected invocation to prevent thread termination on panic
                    if let Err(e) =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| cb(&scan)))
                    {
                        log::error!("Delta2D: Callback panicked: {:?}", e);
                    }
                }
            }
            Ok(None) => {
                // No data available, brief sleep
                thread::sleep(Duration::from_millis(1));
            }
            Err(e) => {
                error_count.fetch_add(1, Ordering::Relaxed);
                log::debug!("Delta2D: Scan error: {}", e);
                thread::sleep(Duration::from_millis(10));
            }
        }
    }

    let total_scans = scan_count.load(Ordering::Relaxed);
    let total_errors = error_count.load(Ordering::Relaxed);
    log::info!(
        "Delta2D: Scan thread exiting (scans: {}, errors: {})",
        total_scans,
        total_errors
    );
}

impl LidarDriver for Delta2DDriver {
    fn start<F>(&mut self, callback: F) -> Result<()>
    where
        F: Fn(&LidarScan) + Send + 'static,
    {
        // Check if already running
        if self.read_thread.is_some() {
            return Err(Error::AlreadyScanning);
        }

        log::info!("Delta2D: Starting scan thread");

        // Set the callback
        {
            let mut cb_lock = self.callback.lock();
            *cb_lock = Some(Box::new(callback));
            log::info!("Delta2D: Callback registered");
        }

        // Reset shutdown flag
        self.shutdown.store(false, Ordering::Relaxed);

        // Clone shared state for the thread
        let transport = Arc::clone(&self.transport);
        let callback = Arc::clone(&self.callback);
        let shutdown = Arc::clone(&self.shutdown);
        let scan_count = Arc::clone(&self.scan_count);
        let error_count = Arc::clone(&self.error_count);

        // Spawn scan thread
        let thread = thread::Builder::new()
            .name("delta2d-scan".to_string())
            .spawn(move || {
                scan_thread_loop(transport, callback, shutdown, scan_count, error_count);
            })
            .map_err(Error::Io)?;

        self.read_thread = Some(thread);
        log::debug!("Delta2D: Scan thread spawned successfully");
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        log::info!("Delta2D: Stopping scan thread");

        // Signal shutdown
        self.shutdown.store(true, Ordering::Relaxed);

        // Wait for thread to exit with timeout
        if let Some(thread) = self.read_thread.take() {
            const SHUTDOWN_TIMEOUT: Duration = Duration::from_millis(500);
            let start = Instant::now();

            // Poll for thread completion
            while !thread.is_finished() {
                if start.elapsed() > SHUTDOWN_TIMEOUT {
                    log::error!("Delta2D: Thread shutdown timeout exceeded");
                    return Err(Error::ShutdownTimeout);
                }
                thread::sleep(Duration::from_millis(10));
            }

            // Now safe to join
            match thread.join() {
                Ok(()) => log::info!("Delta2D: Scan thread stopped cleanly"),
                Err(e) => {
                    log::error!("Delta2D: Scan thread panicked: {:?}", e);
                    return Err(Error::ThreadPanic);
                }
            }
        }

        Ok(())
    }

    fn is_active(&self) -> bool {
        self.read_thread.is_some() && !self.shutdown.load(Ordering::Relaxed)
    }

    fn get_stats(&self) -> (u64, u64) {
        let scans = self.scan_count.load(Ordering::Relaxed);
        let errors = self.error_count.load(Ordering::Relaxed);
        (scans, errors)
    }
}
