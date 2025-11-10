//! Comprehensive test scenario for all robot components
//!
//! Tests GD32 motor controller, Delta-2D Lidar, wheel motors, brushes, vacuum,
//! and reads all sensor data from the robot.
//!
//! See [examples/README.md](README.md) for build instructions and expected output.

use sangam_io::devices::{Delta2DDriver, Gd32Driver};
use sangam_io::drivers::{LidarDriver, MotorDriver};
use sangam_io::transport::SerialTransport;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging - respects RUST_LOG environment variable
    env_logger::init();

    log::info!("=== SangamIO Movement Test with All Components Active ===");
    log::info!("");

    // Step 1: Initialize GD32 motor controller
    log::info!("Step 1: Initializing GD32 motor controller...");
    let gd32_transport = SerialTransport::open("/dev/ttyS3", 115200)?;
    let mut gd32 = Gd32Driver::new(gd32_transport)?;
    log::info!("✓ GD32 initialized successfully");
    log::info!("  - Heartbeat running at 20ms intervals");
    log::info!("  - Initialization sequence completed");
    log::info!("");

    // CRITICAL: Wait 1.4s after init before powering lidar (matches AuxCtrl timing)
    log::info!("Waiting 1.4 seconds for motor controller stabilization...");
    thread::sleep(Duration::from_millis(1400));

    // Step 2: Power on Lidar motor via GD32
    log::info!("Step 2: Powering on Lidar motor...");

    // CRITICAL: Complete AuxCtrl sequence discovered via MITM capture:
    // 1. CMD=0x65 mode=0x01 - Switch to direct motor control mode
    // 2. CMD=0xA2 (LidarPrep) - preparation command
    // 3. CMD=0x97 (LidarPower) - enable GPIO 233
    // 4. CMD=0x71 (LidarPWM) - set motor speed

    // Switch to mode 0x01 (direct motor control)
    gd32.set_motor_mode(0x01)?;
    log::info!("✓ GD32 switched to mode 0x01 (direct motor control)");

    // Send preparation command (CMD=0xA2)
    gd32.send_lidar_prep()?;
    log::info!("✓ Lidar prep command sent (CMD=0xA2)");

    // Enable power (CMD=0x97)
    gd32.set_lidar_power(true)?;
    log::info!("✓ Lidar power enabled (CMD=0x97)");

    // Set PWM speed (CMD=0x71)
    gd32.set_lidar_pwm(100)?; // Full speed (100%)
    log::info!("✓ Lidar PWM set to 100% (CMD=0x71)");

    log::info!("  - Waiting 2 seconds for motor spin-up...");
    thread::sleep(Duration::from_secs(2));
    log::info!("");

    // Step 3: Initialize Delta-2D Lidar
    log::info!("Step 3: Initializing Delta-2D Lidar...");
    let lidar_transport = SerialTransport::open("/dev/ttyS1", 115200)?;
    let mut lidar = Delta2DDriver::new(lidar_transport)?;
    log::info!("✓ Lidar driver initialized");
    log::info!("");

    // Step 4: Start Lidar scanning
    log::info!("Step 4: Starting Lidar scan...");
    lidar.start()?;
    log::info!("✓ Lidar scanning started");
    log::info!("");

    // Step 5: Start Lidar logging thread (runs in background)
    log::info!("Step 5: Starting Lidar logging thread...");
    let lidar_shutdown = Arc::new(AtomicBool::new(false));
    let lidar_shutdown_clone = Arc::clone(&lidar_shutdown);

    let lidar_thread = thread::spawn(move || {
        let mut scan_count = 0u32;
        log::info!("Lidar thread: Started continuous scanning (logging every 200 scans)");

        while !lidar_shutdown_clone.load(Ordering::Relaxed) {
            match lidar.get_scan() {
                Ok(Some(scan)) => {
                    scan_count += 1;
                    // Only log every 200 scans to reduce output spam
                    if scan_count % 200 == 0 {
                        log::info!("Lidar: Scan #{}: {} points", scan_count, scan.points.len());
                    }
                }
                Ok(None) => {
                    thread::sleep(Duration::from_millis(10));
                }
                Err(e) => {
                    log::warn!("Lidar thread: Error reading scan: {}", e);
                    thread::sleep(Duration::from_millis(10));
                }
            }
        }

        log::info!("Lidar thread: Shutting down (total scans: {})", scan_count);
    });
    log::info!("✓ Lidar logging thread started");
    log::info!("");

    // Step 6: Using mode 0x01 (Direct Control)
    log::info!("Step 6: Using mode 0x01 (direct motor control)...");
    log::info!("✓ Mode 0x01 active (CMD=0x67 MotorSpeed)");
    log::info!("  Note: Mode 0x01 was set for encoder testing");
    thread::sleep(Duration::from_millis(500));
    log::info!("");

    // Step 7: Turn ON all components
    log::info!("Step 7: Activating all components...");

    log::info!("  - Turning ON vacuum (100%)");
    gd32.set_vacuum(100)?;

    log::info!("  - Turning ON main brush (100%)");
    gd32.set_main_brush(100)?;

    log::info!("  - Turning ON side brush (100%)");
    gd32.set_side_brush(100)?;

    log::info!("✓ All components active");
    thread::sleep(Duration::from_millis(500));
    log::info!("");

    // Step 8: Movement Tests
    log::info!("========================================");
    log::info!("Step 8: Running Movement Tests");
    log::info!("========================================");
    log::info!("");

    // Test 1: Move forward at 500 speed
    log::info!("Test 1: Move forward at 500 speed for 2 seconds");
    let (start_left, start_right) = gd32.get_encoder_counts();
    log::info!(
        "  Starting encoders: left={}, right={}",
        start_left,
        start_right
    );

    gd32.set_raw_motor_speeds(500, 500)?;
    thread::sleep(Duration::from_secs(2));
    gd32.set_raw_motor_speeds(0, 0)?;

    let (end_left, end_right) = gd32.get_encoder_counts();
    log::info!("  Ending encoders: left={}, right={}", end_left, end_right);
    log::info!(
        "  Delta: left={}, right={}",
        end_left - start_left,
        end_right - start_right
    );
    log::info!("✓ Forward movement complete");
    log::info!("");
    thread::sleep(Duration::from_secs(1));

    // Test 2: Move backward at 1000 speed
    log::info!("Test 2: Move backward at 1000 speed for 2 seconds");
    let (start_left, start_right) = gd32.get_encoder_counts();
    log::info!(
        "  Starting encoders: left={}, right={}",
        start_left,
        start_right
    );

    gd32.set_raw_motor_speeds(-500, -500)?;
    thread::sleep(Duration::from_secs(2));
    gd32.set_raw_motor_speeds(0, 0)?;

    let (end_left, end_right) = gd32.get_encoder_counts();
    log::info!("  Ending encoders: left={}, right={}", end_left, end_right);
    log::info!(
        "  Delta: left={}, right={}",
        end_left - start_left,
        end_right - start_right
    );
    log::info!("✓ Backward movement complete");
    log::info!("");
    thread::sleep(Duration::from_secs(1));

    // Test 3: Rotate clockwise at 1000 speed
    log::info!("Test 3: Rotate clockwise at 1000 speed for 3 seconds");
    let (start_left, start_right) = gd32.get_encoder_counts();
    log::info!(
        "  Starting encoders: left={}, right={}",
        start_left,
        start_right
    );

    gd32.set_raw_motor_speeds(500, -500)?;
    thread::sleep(Duration::from_secs(3));
    gd32.set_raw_motor_speeds(0, 0)?;

    let (end_left, end_right) = gd32.get_encoder_counts();
    log::info!("  Ending encoders: left={}, right={}", end_left, end_right);
    log::info!(
        "  Delta: left={}, right={}",
        end_left - start_left,
        end_right - start_right
    );
    log::info!("✓ Clockwise rotation complete");
    log::info!("");
    thread::sleep(Duration::from_secs(1));

    // Test 4: Rotate counter-clockwise at 1000 speed
    log::info!("Test 4: Rotate counter-clockwise at 1000 speed for 3 seconds");
    let (start_left, start_right) = gd32.get_encoder_counts();
    log::info!(
        "  Starting encoders: left={}, right={}",
        start_left,
        start_right
    );

    gd32.set_raw_motor_speeds(-500, 500)?;
    thread::sleep(Duration::from_secs(3));
    gd32.set_raw_motor_speeds(0, 0)?;

    let (end_left, end_right) = gd32.get_encoder_counts();
    log::info!("  Ending encoders: left={}, right={}", end_left, end_right);
    log::info!(
        "  Delta: left={}, right={}",
        end_left - start_left,
        end_right - start_right
    );
    log::info!("✓ Counter-clockwise rotation complete");
    log::info!("");

    // Step 9: Turn OFF all components
    log::info!("========================================");
    log::info!("Step 9: Deactivating all components");
    log::info!("========================================");
    log::info!("");

    log::info!("  - Turning OFF vacuum");
    gd32.set_vacuum(0)?;

    log::info!("  - Turning OFF main brush");
    gd32.set_main_brush(0)?;

    log::info!("  - Turning OFF side brush");
    gd32.set_side_brush(0)?;

    log::info!("✓ All components deactivated");
    thread::sleep(Duration::from_millis(500));
    log::info!("");

    // Step 10: Read all sensor data
    log::info!("Step 10: Reading all sensor data from GD32...");
    log::info!("");

    // Battery information
    let (voltage, current, level) = gd32.get_battery_info();
    log::info!("Battery Status:");
    log::info!("  - Voltage: {:.2} V", voltage);
    log::info!("  - Current: {:.2} A", current);
    log::info!("  - Level: {}%", level);
    log::info!("");

    // Encoder counts
    let (left_enc, right_enc) = gd32.get_encoder_counts();
    log::info!("Encoder Counts:");
    log::info!("  - Left wheel: {} ticks", left_enc);
    log::info!("  - Right wheel: {} ticks", right_enc);
    log::info!("");

    // Error code
    let error_code = gd32.get_error_code();
    log::info!("Error Status:");
    log::info!("  - Error code: 0x{:02X}", error_code);
    if error_code == 0 {
        log::info!("  - Status: No errors");
    } else {
        log::warn!("  - Status: Error detected!");
    }
    log::info!("");

    // Status flags (NEW!)
    let (status_flag, charging, battery_state, percent_val) = gd32.get_status_flags();
    log::info!("Status Flags:");
    log::info!("  - Status flag: 0x{:02X}", status_flag);
    log::info!("  - Charging: {}", charging);
    log::info!("  - Battery state flag: {}", battery_state);
    log::info!("  - Percent value: {:.2}", percent_val);
    log::info!("");

    // IR sensors (NEW!)
    let (ir1, point_btn, dock_btn) = gd32.get_ir_sensors();
    log::info!("IR Proximity Sensors:");
    log::info!("  - IR sensor 1: {}", ir1);
    log::info!(
        "  - Point button IR: {} {}",
        point_btn,
        if gd32.is_point_button_pressed() {
            "(PRESSED)"
        } else {
            ""
        }
    );
    log::info!(
        "  - Dock button IR: {} {}",
        dock_btn,
        if gd32.is_dock_button_pressed() {
            "(PRESSED)"
        } else {
            ""
        }
    );
    log::info!("");

    // Packet statistics
    let (rx_packets, tx_packets, lost_packets) = gd32.get_packet_stats();
    log::info!("Communication Statistics:");
    log::info!("  - Packets received: {}", rx_packets);
    log::info!("  - Packets transmitted: {}", tx_packets);
    log::info!("  - Lost packets: {}", lost_packets);
    log::info!("");

    log::info!("NOTE: 24 of 88 STATUS_DATA bytes decoded (27% coverage)");
    log::info!("      Remaining fields: IMU data, cliff sensors, bumpers, wall sensor");
    log::info!("      See parse_status_packet() in src/devices/gd32/protocol.rs for details");
    log::info!("");

    // Step 11: Stop Lidar logging thread
    log::info!("Step 11: Stopping Lidar logging thread...");
    lidar_shutdown.store(true, Ordering::Relaxed);
    lidar_thread.join().expect("Failed to join lidar thread");
    log::info!("✓ Lidar logging thread stopped");
    log::info!("");

    // Step 12: Power off Lidar motor
    log::info!("Step 12: Powering off Lidar motor...");

    // First stop PWM (CMD=0x71)
    gd32.set_lidar_pwm(0)?;
    log::info!("✓ Lidar PWM set to 0% (CMD=0x71)");

    // Then disable power (CMD=0x97)
    gd32.set_lidar_power(false)?;
    log::info!("✓ Lidar power disabled (CMD=0x97)");
    log::info!("");

    // Step 13: Clean exit (Drop implementations will handle cleanup)
    log::info!("Step 13: Clean exit...");
    log::info!("  - GD32 heartbeat thread will stop");
    log::info!("  - Motors will stop");
    log::info!("");

    drop(gd32);

    log::info!("=== All tests completed successfully ===");

    Ok(())
}
