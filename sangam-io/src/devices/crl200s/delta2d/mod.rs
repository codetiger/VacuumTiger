//! Delta-2D Lidar driver
//!
//! # Scan Geometry and Accumulation
//!
//! The Delta-2D lidar provides continuous 360° scans at approximately 5Hz.
//! Each scan is delivered as a stream of individual point measurements.
//!
//! ## Coordinate Frame
//!
//! - **0° (forward)**: Robot's front direction
//! - **Rotation**: Clockwise (CW) when viewed from above
//! - **Angle range**: 0.0° to 360.0° (wraps at 360°)
//! - **Distance units**: Millimeters (mm)
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

pub mod protocol;

use crate::core::types::{SensorGroupData, SensorValue};
use crate::error::{Error, Result};
use protocol::Delta2DPacketReader;
use serialport::SerialPort;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

const LIDAR_BAUD_RATE: u32 = 115200;
const LIDAR_READ_TIMEOUT_MS: u64 = 100;

/// Minimum points required before accepting a scan as complete
/// This prevents false scan boundaries from angle noise or single bad readings
const MIN_SCAN_POINTS: usize = 50;

/// Delta-2D lidar driver
pub struct Delta2DDriver {
    port_path: String,
    shutdown: Arc<AtomicBool>,
    reader_handle: Option<JoinHandle<()>>,
    scan_count: Arc<AtomicU64>,
    error_count: Arc<AtomicU64>,
}

impl Delta2DDriver {
    /// Create a new Delta-2D driver
    pub fn new(port_path: &str) -> Self {
        Self {
            port_path: port_path.to_string(),
            shutdown: Arc::new(AtomicBool::new(false)),
            reader_handle: None,
            scan_count: Arc::new(AtomicU64::new(0)),
            error_count: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Start the lidar reader thread
    pub fn start(&mut self, sensor_data: Arc<Mutex<SensorGroupData>>) -> Result<()> {
        // Open serial port
        let port = serialport::new(&self.port_path, LIDAR_BAUD_RATE)
            .timeout(Duration::from_millis(LIDAR_READ_TIMEOUT_MS))
            .open()
            .map_err(Error::Serial)?;

        let shutdown = Arc::clone(&self.shutdown);
        let scan_count = Arc::clone(&self.scan_count);
        let error_count = Arc::clone(&self.error_count);

        self.reader_handle = Some(
            thread::Builder::new()
                .name("delta2d-reader".to_string())
                .spawn(move || {
                    Self::reader_loop(port, shutdown, sensor_data, scan_count, error_count);
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
    ) {
        let mut reader = Delta2DPacketReader::new();
        let mut accumulated_points: Vec<(f32, f32, u8)> = Vec::with_capacity(360);
        let mut last_angle: f32 = 0.0;

        while !shutdown.load(Ordering::Relaxed) {
            match reader.read_scan(&mut port) {
                Ok(Some(scan)) => {
                    let count = scan_count.fetch_add(1, Ordering::Relaxed) + 1;

                    // Accumulate points for a complete 360° scan
                    // Detect scan completion when angle wraps around (359° → 1°)
                    for point in &scan.points {
                        if point.angle < last_angle && accumulated_points.len() > MIN_SCAN_POINTS {
                            // Angle wrapped around 360° - publish complete scan
                            Self::publish_scan(&sensor_data, &accumulated_points);
                            accumulated_points.clear();
                        }
                        accumulated_points.push((point.angle, point.distance, point.quality));
                        last_angle = point.angle;
                    }

                    // Log statistics
                    if count % 100 == 0 {
                        log::debug!(
                            "Lidar: {} scans, {} points accumulated",
                            count,
                            accumulated_points.len()
                        );
                    }
                    if count % 1000 == 0 {
                        let errors = error_count.load(Ordering::Relaxed);
                        let error_rate = if count > 0 {
                            (errors as f64 / count as f64) * 100.0
                        } else {
                            0.0
                        };
                        log::info!(
                            "Lidar stats: {} scans, {:.2}% error rate",
                            count,
                            error_rate
                        );
                    }
                }
                Ok(None) => {
                    // No scan yet
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
        data.update("scan", SensorValue::PointCloud2D(points.to_vec()));

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
