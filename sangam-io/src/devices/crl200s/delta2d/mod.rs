//! Delta-2D Lidar driver
//!
//! # Scan Geometry and Accumulation
//!
//! The Delta-2D lidar provides continuous 360° scans at approximately 5Hz.
//! Each scan is delivered as a stream of individual point measurements.
//!
//! ## Coordinate Frame (Output)
//!
//! After applying the configurable `AffineTransform1D` from `sangamio.toml`,
//! output data follows **ROS REP-103** convention:
//!
//! - **0° = forward**: Robot's front direction (+X axis)
//! - **Rotation**: Counter-clockwise (CCW) positive when viewed from above
//! - **90° = left**: Robot's left side (+Y axis)
//! - **Angle range**: 0.0 to 2π radians
//! - **Distance units**: Meters (m)
//!
//! ## Raw Hardware (Before Transform)
//!
//! The Delta-2D hardware outputs:
//! - **0° = backward**: Opposite to robot forward (lidar mounted facing rear)
//! - **Rotation**: Clockwise (CW) when viewed from above
//! - **Distance units**: 0.25mm per unit
//!
//! The CRL-200S transform (`scale=-1, offset=π`) converts to ROS convention.
//!
//! ## Scan Accumulation State Machine
//!
//! ```text
//! ┌─────────────────┐
//! │ Collecting      │ ◄─────┐
//! │ Points          │       │
//! │ (angle ↑)       │       │
//! └────────┬────────┘       │
//!          │                │
//!          │ angle < last   │
//!          │ AND count>50   │
//!          ▼                │
//! ┌─────────────────┐       │
//! │ Publish Scan    │       │
//! │ Clear Buffer    │───────┘
//! └─────────────────┘
//! ```
//!
//! **Scan completion detection:**
//! 1. Monitor angle of each incoming point
//! 2. When angle decreases (e.g., 359° → 1°), a 360° wrap occurred
//! 3. Require minimum 50 points to avoid false triggers from noise
//! 4. Publish accumulated points as complete scan
//! 5. Reset buffer and continue collecting
//!
//! **Why 50 points minimum?**
//! - Prevents false scan boundaries from angle jitter or bad readings
//! - At 5Hz scan rate with ~360 points/scan, valid scans have >300 points
//! - 50-point threshold allows partial scans during startup
//!
//! # Robustness Features
//!
//! This driver includes several robustness improvements:
//!
//! 1. **Serial buffer flush on startup**: Clears stale data from previous sessions
//! 2. **Scan accumulation timeout**: Publishes partial scans if no angle wrap for 2 seconds
//! 3. **Diagnostic logging**: Tracks bytes discarded and CRC failures for debugging

pub mod protocol;

use crate::config::AffineTransform1D;
use crate::core::types::{SensorGroupData, SensorValue};
use crate::error::{Error, Result};
use protocol::{Delta2DPacketReader, ParseResult};
use serialport::SerialPort;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const LIDAR_BAUD_RATE: u32 = 115200;
const LIDAR_READ_TIMEOUT_MS: u64 = 100;

/// Minimum points required before accepting a scan as complete
/// This prevents false scan boundaries from angle noise or single bad readings
const MIN_SCAN_POINTS: usize = 50;

/// Maximum time to wait for a complete scan before publishing partial results
/// If no angle wrap occurs within this time, publish whatever we have
const SCAN_TIMEOUT_SECS: f32 = 2.0;

/// Delta-2D lidar driver
pub struct Delta2DDriver {
    port_path: String,
    shutdown: Arc<AtomicBool>,
    reader_handle: Option<JoinHandle<()>>,
    scan_count: Arc<AtomicU64>,
    error_count: Arc<AtomicU64>,
    /// Angle transform for coordinate frame conversion
    angle_transform: AffineTransform1D,
}

impl Delta2DDriver {
    /// Create a new Delta-2D driver
    ///
    /// # Arguments
    /// - `port_path`: Serial port path (e.g., "/dev/ttyS1")
    /// - `angle_transform`: Transform applied to all lidar angles (use `AffineTransform1D::identity()` for no change)
    pub fn new(port_path: &str, angle_transform: AffineTransform1D) -> Self {
        Self {
            port_path: port_path.to_string(),
            shutdown: Arc::new(AtomicBool::new(false)),
            reader_handle: None,
            scan_count: Arc::new(AtomicU64::new(0)),
            error_count: Arc::new(AtomicU64::new(0)),
            angle_transform,
        }
    }

    /// Start the lidar reader thread
    pub fn start(&mut self, sensor_data: Arc<Mutex<SensorGroupData>>) -> Result<()> {
        // Open serial port
        let port = serialport::new(&self.port_path, LIDAR_BAUD_RATE)
            .timeout(Duration::from_millis(LIDAR_READ_TIMEOUT_MS))
            .open()
            .map_err(Error::Serial)?;

        // Flush serial buffer to clear stale data from previous sessions
        // This prevents processing old packets that may have accumulated
        if let Err(e) = port.clear(serialport::ClearBuffer::Input) {
            log::warn!("Failed to clear lidar serial input buffer: {}", e);
        } else {
            log::debug!("Cleared lidar serial input buffer");
        }

        let shutdown = Arc::clone(&self.shutdown);
        let scan_count = Arc::clone(&self.scan_count);
        let error_count = Arc::clone(&self.error_count);
        let angle_transform = self.angle_transform;

        self.reader_handle = Some(
            thread::Builder::new()
                .name("delta2d-reader".to_string())
                .spawn(move || {
                    Self::reader_loop(
                        port,
                        shutdown,
                        sensor_data,
                        scan_count,
                        error_count,
                        angle_transform,
                    );
                })
                .map_err(|e| Error::Other(format!("Failed to spawn lidar thread: {}", e)))?,
        );

        log::info!("Delta-2D lidar driver started on {}", self.port_path);
        Ok(())
    }

    /// Reader loop - reads scans and updates shared data
    fn reader_loop(
        mut port: Box<dyn SerialPort>,
        shutdown: Arc<AtomicBool>,
        sensor_data: Arc<Mutex<SensorGroupData>>,
        scan_count: Arc<AtomicU64>,
        error_count: Arc<AtomicU64>,
        angle_transform: AffineTransform1D,
    ) {
        let mut reader = Delta2DPacketReader::with_transform(angle_transform);
        let mut accumulated_points: Vec<(f32, f32, u8)> = Vec::with_capacity(400);
        let mut last_angle: f32 = 0.0;
        let mut last_scan_time = Instant::now();
        let mut last_diagnostic_time = Instant::now();

        // Clear reader buffer on startup
        reader.clear();

        while !shutdown.load(Ordering::Relaxed) {
            // Step 1: Read bytes from serial port into buffer
            match reader.read_bytes(&mut port) {
                Ok(_bytes_read) => {
                    // Step 2: Drain ALL complete packets from buffer
                    loop {
                        match reader.parse_next() {
                            Ok(ParseResult::Scan(scan)) => {
                                let count = scan_count.fetch_add(1, Ordering::Relaxed) + 1;

                                // Accumulate points for a complete 360° scan
                                // Detect scan completion by checking each point for wrap
                                // Wrap occurs when angle jumps significantly (crosses 0°/360° boundary)
                                for point in &scan.points {
                                    // After CCW transform, angles DECREASE (359° → 358° → ... → 0°)
                                    // Real wrap: small → large (e.g., 0.5° → 359.5°) = POSITIVE jump
                                    // False wrap: 0° → 359° = NEGATIVE jump (just crossing forward direction)
                                    // Only detect POSITIVE jumps > π as real wraps
                                    let angle_diff = point.angle - last_angle;
                                    let wrapped = angle_diff > std::f32::consts::PI
                                        && accumulated_points.len() > MIN_SCAN_POINTS;

                                    if wrapped {
                                        // Angle wrapped around 360° - publish complete scan
                                        Self::publish_scan(&sensor_data, &accumulated_points);
                                        accumulated_points.clear();
                                        last_scan_time = Instant::now();
                                    }

                                    // Add this point to accumulator
                                    accumulated_points.push((
                                        point.angle,
                                        point.distance,
                                        point.quality,
                                    ));
                                    last_angle = point.angle;
                                }

                                // Check for scan timeout - if no angle wrap for too long,
                                // publish partial scan to avoid infinite accumulation
                                let elapsed = last_scan_time.elapsed().as_secs_f32();
                                if elapsed > SCAN_TIMEOUT_SECS
                                    && accumulated_points.len() > MIN_SCAN_POINTS
                                {
                                    log::warn!(
                                        "Lidar scan timeout ({:.1}s), publishing partial scan with {} points",
                                        elapsed,
                                        accumulated_points.len()
                                    );
                                    Self::publish_scan(&sensor_data, &accumulated_points);
                                    accumulated_points.clear();
                                    last_scan_time = Instant::now();
                                }

                                // Log statistics periodically
                                if count.is_multiple_of(100) {
                                    log::debug!(
                                        "Lidar: {} packets, {} points accumulated",
                                        count,
                                        accumulated_points.len()
                                    );
                                }

                                // Log diagnostics every 2 seconds
                                if last_diagnostic_time.elapsed().as_secs() >= 60 {
                                    let errors = error_count.load(Ordering::Relaxed);
                                    let (bytes_discarded, crc_failures, buffer_size) =
                                        reader.diagnostics();
                                    log::info!(
                                        "Lidar stats: {} packets, {} errors, {} bytes discarded, {} CRC failures, {} bytes buffered",
                                        count,
                                        errors,
                                        bytes_discarded,
                                        crc_failures,
                                        buffer_size
                                    );
                                    last_diagnostic_time = Instant::now();
                                }
                            }
                            Ok(ParseResult::Health) => {
                                // Health packet (0xAE) - motor is spinning
                            }
                            Ok(ParseResult::None) => {
                                // No more complete packets in buffer - break inner loop
                                break;
                            }
                            Err(e) => {
                                error_count.fetch_add(1, Ordering::Relaxed);
                                log::error!("Lidar parse error: {}", e);
                                break;
                            }
                        }
                    }

                    // Check for timeout on accumulated points (after draining all packets)
                    let elapsed = last_scan_time.elapsed().as_secs_f32();
                    if elapsed > SCAN_TIMEOUT_SECS && accumulated_points.len() > MIN_SCAN_POINTS {
                        log::warn!(
                            "Lidar scan timeout during idle ({:.1}s), publishing {} points",
                            elapsed,
                            accumulated_points.len()
                        );
                        Self::publish_scan(&sensor_data, &accumulated_points);
                        accumulated_points.clear();
                        last_scan_time = Instant::now();
                    }

                    // Small sleep to avoid busy-waiting when no data available
                    thread::sleep(Duration::from_millis(1));
                }
                Err(e) => {
                    error_count.fetch_add(1, Ordering::Relaxed);
                    log::error!("Lidar read error: {}", e);
                    thread::sleep(Duration::from_millis(10));
                }
            }
        }

        log::info!("Delta-2D reader thread exiting");
    }

    /// Publish accumulated scan to sensor data
    fn publish_scan(sensor_data: &Arc<Mutex<SensorGroupData>>, points: &[(f32, f32, u8)]) {
        let Ok(mut data) = sensor_data.lock() else {
            log::error!("Failed to lock sensor data for lidar scan");
            return;
        };
        data.touch();
        data.set("scan", SensorValue::PointCloud2D(points.to_vec()));

        log::trace!("Published lidar scan with {} points", points.len());
    }

    /// Shutdown the driver
    pub fn shutdown(&mut self) -> Result<()> {
        log::info!("Shutting down Delta-2D driver...");
        self.shutdown.store(true, Ordering::Relaxed);

        if let Some(handle) = self.reader_handle.take() {
            handle.join().map_err(|_| Error::ThreadPanic)?;
        }

        log::info!("Delta-2D driver shutdown complete");
        Ok(())
    }
}

impl Drop for Delta2DDriver {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}
