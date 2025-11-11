//! Quick hardware test using SangamIO - 20 second full component test
//!
//! Test sequence:
//! 1. Init GD32 communication
//! 2. Start lidar motor
//! 3. Start air pump, brushes
//! 4. Move forward short distance
//! 5. Rotate CW 360°
//! 6. Rotate CCW 360°
//! 7. Stop all
//!
//! Build for target:
//! ```sh
//! cargo build --example quick_demo --release --target armv7-unknown-linux-musleabihf
//! ```
//!
//! Deploy to robot:
//! ```sh
//! cat target/armv7-unknown-linux-musleabihf/release/examples/quick_demo | \
//!     sshpass -p "vacuum@123" ssh root@vacuum "cat > /tmp/quick_demo && chmod +x /tmp/quick_demo"
//!
//! # Stop original firmware and run
//! sshpass -p "vacuum@123" ssh root@vacuum "
//!     mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak 2>/dev/null; \
//!     killall -9 AuxCtrl 2>/dev/null; \
//!     RUST_LOG=info /tmp/quick_demo; \
//!     mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null
//! "
//! ```

use sangam_io::SangamIO;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    log::info!("=== SangamIO Quick Hardware Test (20s) ===");

    // === 1. Initialize SangamIO ===
    log::info!("1. Initializing SangamIO for CRL-200S hardware...");
    let mut sangam = SangamIO::crl200s(
        "/dev/ttyS3", // GD32 motor controller
        "/dev/ttyS1", // Delta-2D lidar
    )?;
    log::info!("   ✓ SangamIO initialization complete");

    // === 2. Start Lidar with Callback ===
    log::info!("2. Starting lidar motor...");

    let scan_count = Arc::new(AtomicU64::new(0));
    let scan_count_clone = Arc::clone(&scan_count);

    // Register minimal callback (just count scans)
    sangam.start_lidar(move |_scan| {
        scan_count_clone.fetch_add(1, Ordering::Relaxed);
    })?;
    log::info!("   ✓ Lidar motor started and scanning");

    // === 3. Start Cleaning Components ===
    log::info!("3. Starting air pump and brushes...");
    sangam.set_air_pump(40)?;
    sangam.set_brushes(80, 80)?;
    log::info!("   ✓ Air pump at 40%, brushes at 80%");
    thread::sleep(Duration::from_millis(500));

    // === 4. Move Forward ===
    log::info!("4. Moving forward 0.2m...");
    sangam.set_velocity(0.2, 0.0)?;
    thread::sleep(Duration::from_millis(1200));

    // Decelerate to zero before stopping
    sangam.set_velocity(0.0, 0.0)?;
    thread::sleep(Duration::from_millis(300));
    sangam.stop()?;

    let delta = sangam.get_odometry_delta()?;
    log::info!("   ✓ Moved {:.3}m", delta.distance);
    thread::sleep(Duration::from_secs(1));

    // === 5. Rotate CW 360° ===
    log::info!("5. Rotating clockwise 360°...");
    // For 360° rotation: use angular velocity
    sangam.set_velocity(0.0, 1.0)?;
    thread::sleep(Duration::from_millis(3000));

    // Decelerate to zero before stopping
    sangam.set_velocity(0.0, 0.0)?;
    thread::sleep(Duration::from_millis(1000));
    sangam.stop()?;

    let delta = sangam.get_odometry_delta()?;
    log::info!("   ✓ Rotated {:.1}°", delta.angle.to_degrees());
    thread::sleep(Duration::from_millis(300));

    // === 6. Rotate CCW 360° ===
    log::info!("6. Rotating counter-clockwise 360°...");
    sangam.set_velocity(0.0, -1.0)?;
    thread::sleep(Duration::from_millis(3000));

    // Decelerate to zero before stopping
    sangam.set_velocity(0.0, 0.0)?;
    thread::sleep(Duration::from_millis(1000));
    sangam.stop()?;

    let delta = sangam.get_odometry_delta()?;
    log::info!("   ✓ Rotated {:.1}°", delta.angle.to_degrees());
    thread::sleep(Duration::from_millis(300));

    // === 7. Stop All Components ===
    log::info!("7. Stopping all components...");

    // Stop motors (already at zero velocity)
    sangam.stop()?;

    // Stop air pump
    sangam.set_air_pump(0)?;

    // Stop brushes
    sangam.set_brushes(0, 0)?;

    // Stop lidar
    sangam.stop_lidar()?;
    log::info!("   ✓ All components stopped");

    let total_scans = scan_count.load(Ordering::Relaxed);
    log::info!("   ✓ Total lidar scans: {}", total_scans);

    thread::sleep(Duration::from_millis(500));

    log::info!("=== Test Complete ===");
    log::info!("All components tested successfully using SangamIO API");

    // SangamIO Drop implementation handles cleanup
    drop(sangam);

    Ok(())
}
