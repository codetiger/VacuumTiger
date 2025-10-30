//! Test scenario for GD32 + Delta-2D Lidar integration
//!
//! Demonstrates complete robot initialization, lidar power control,
//! scan data collection, and clean shutdown.
//!
//! See [examples/README.md](README.md) for build instructions and expected output.

use sangam_io::devices::{Delta2DDriver, Gd32Driver};
use sangam_io::drivers::LidarDriver;
use sangam_io::transport::SerialTransport;
use std::thread;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    log::info!("=== SangamIO Lidar Test Scenario ===");
    log::info!("");

    // Step 1: Initialize GD32 motor controller
    log::info!("Step 1: Initializing GD32 motor controller...");
    let gd32_transport = SerialTransport::open("/dev/ttyS3", 115200)?;
    let mut gd32 = Gd32Driver::new(gd32_transport)?;
    log::info!("✓ GD32 initialized successfully");
    log::info!("  - Heartbeat running at 20ms intervals");
    log::info!("");

    // Wait a bit for stabilization
    thread::sleep(Duration::from_millis(500));

    // Step 2: Initialize Delta-2D Lidar
    log::info!("Step 2: Initializing Delta-2D Lidar...");
    let lidar_transport = SerialTransport::open("/dev/ttyS2", 115200)?;
    let mut lidar = Delta2DDriver::new(lidar_transport)?;
    log::info!("✓ Lidar driver initialized");
    log::info!("");

    // Step 3: Power on Lidar motor via GD32
    log::info!("Step 3: Powering on Lidar motor...");
    gd32.set_lidar_power(true)?;
    log::info!("✓ Lidar motor powered on");
    log::info!("  - Waiting 2 seconds for motor spin-up...");
    thread::sleep(Duration::from_secs(2));
    log::info!("");

    // Step 4: Start Lidar scanning
    log::info!("Step 4: Starting Lidar scan...");
    lidar.start()?;
    log::info!("✓ Lidar scanning started");
    log::info!("");

    // Step 5: Read and log Lidar data for 5 seconds
    log::info!("Step 5: Reading Lidar data for 5 seconds...");
    let scan_start = Instant::now();
    let scan_duration = Duration::from_secs(5);
    let mut scan_count = 0;
    let mut total_points = 0;

    while scan_start.elapsed() < scan_duration {
        match lidar.get_scan() {
            Ok(Some(scan)) => {
                scan_count += 1;
                total_points += scan.points.len();

                log::info!(
                    "Scan #{}: {} points, timestamp: {:?}ms",
                    scan_count,
                    scan.points.len(),
                    scan.timestamp_ms
                );

                // Log first few points as samples
                if scan_count == 1 {
                    log::info!("  Sample points:");
                    for (i, point) in scan.points.iter().take(5).enumerate() {
                        log::info!(
                            "    Point {}: angle={:.2}°, distance={:.3}m, quality={}",
                            i + 1,
                            point.angle.to_degrees(),
                            point.distance,
                            point.quality
                        );
                    }
                }
            }
            Ok(None) => {
                // No scan data available, wait a bit
                thread::sleep(Duration::from_millis(10));
            }
            Err(e) => {
                log::warn!("Error reading scan: {}", e);
                thread::sleep(Duration::from_millis(10));
            }
        }
    }

    log::info!("");
    log::info!("✓ Scan complete:");
    log::info!("  - Total scans received: {}", scan_count);
    log::info!("  - Total points: {}", total_points);
    if scan_count > 0 {
        log::info!("  - Average points per scan: {}", total_points / scan_count);
    }
    log::info!("");

    // Step 6: Stop Lidar scanning
    log::info!("Step 6: Stopping Lidar scan...");
    lidar.stop()?;
    log::info!("✓ Lidar scanning stopped");
    log::info!("");

    // Step 7: Power off Lidar motor
    log::info!("Step 7: Powering off Lidar motor...");
    gd32.set_lidar_power(false)?;
    log::info!("✓ Lidar motor powered off");
    log::info!("");

    // Step 8: Clean exit (Drop implementations will handle cleanup)
    log::info!("Step 8: Clean exit...");
    log::info!("  - GD32 heartbeat thread will stop");
    log::info!("  - Motors will stop");
    log::info!("");

    drop(gd32);
    drop(lidar);

    log::info!("=== Test scenario completed successfully ===");

    Ok(())
}
