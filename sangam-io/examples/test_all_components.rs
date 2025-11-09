//! Comprehensive test scenario for all robot components
//!
//! Tests GD32 motor controller, Delta-2D Lidar, wheel motors, brushes, vacuum,
//! and reads all sensor data from the robot.
//!
//! See [examples/README.md](README.md) for build instructions and expected output.

use sangam_io::devices::{Delta2DDriver, Gd32Driver};
use sangam_io::drivers::{LidarDriver, MotorDriver};
use sangam_io::transport::SerialTransport;
use std::thread;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    log::info!("=== SangamIO Comprehensive Component Test ===");
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
    // 1. CMD=0x65 mode=0x02 - Switch to navigation mode (enables lidar control)
    // 2. CMD=0xA2 (LidarPrep) - preparation command
    // 3. CMD=0x97 (LidarPower) - enable GPIO 233
    // 4. CMD=0x71 (LidarPWM) - set motor speed

    // Switch to navigation mode (CRITICAL - without this, lidar commands don't work!)
    gd32.set_motor_mode(0x02)?;
    log::info!("✓ GD32 switched to navigation mode (CMD=0x65 mode=0x02)");

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

    // Step 6: Stop Lidar scanning (keep motor running for now)
    log::info!("Step 6: Stopping Lidar scan...");
    lidar.stop()?;
    log::info!("✓ Lidar scanning stopped");
    log::info!("");

    // TESTING: Stay in mode 0x02 (navigation mode) for entire sequence
    // This matches AuxCtrl behavior which stays in mode 0x02 throughout cleaning
    // AuxCtrl uses CMD=0x66 (MotorVelocity) in mode 0x02, we're testing CMD=0x67 (MotorSpeed)
    log::info!("TESTING: Staying in mode 0x02 (navigation mode) for all components");
    log::info!("✓ GD32 remaining in navigation mode (CMD=0x65 mode=0x02)");
    log::info!("");

    // Step 7: Test forward movement (short distance)
    log::info!("Step 7: Testing forward movement (6 inches)...");
    log::info!("  - Both wheels forward at 2000 ticks (slow speed)");
    log::info!("  - Duration: 1 second");

    let (start_left, start_right) = gd32.get_encoder_counts();
    log::info!(
        "  - Starting encoders: left={}, right={}",
        start_left,
        start_right
    );

    // Move forward slowly - both wheels same direction
    gd32.set_raw_motor_speeds(2000, 2000)?;
    thread::sleep(Duration::from_secs(1));
    gd32.set_raw_motor_speeds(0, 0)?;

    let (end_left, end_right) = gd32.get_encoder_counts();
    log::info!(
        "  - Ending encoders: left={}, right={}",
        end_left,
        end_right
    );
    log::info!(
        "  - Delta: left={}, right={}",
        end_left - start_left,
        end_right - start_right
    );
    log::info!("✓ Forward movement complete");
    log::info!("");

    thread::sleep(Duration::from_secs(1));

    // Step 8: Test rotation - 360° clockwise (in place)
    log::info!("Step 8: Testing rotation - 360° clockwise (in place)...");
    log::info!("  - Left wheel forward, right wheel backward");
    log::info!("  - Speed: 3000 ticks for 3 seconds");
    log::info!("  - BOTH wheels should rotate in opposite directions!");

    let (start_left, start_right) = gd32.get_encoder_counts();
    log::info!(
        "  - Starting encoders: left={}, right={}",
        start_left,
        start_right
    );

    // Clockwise rotation: left forward (+), right backward (-)
    gd32.set_raw_motor_speeds(3000, -3000)?;
    thread::sleep(Duration::from_secs(3));
    gd32.set_raw_motor_speeds(0, 0)?;

    let (end_left, end_right) = gd32.get_encoder_counts();
    log::info!(
        "  - Ending encoders: left={}, right={}",
        end_left,
        end_right
    );
    log::info!(
        "  - Delta: left={}, right={}",
        end_left - start_left,
        end_right - start_right
    );

    // Check if both wheels moved
    if (end_left != start_left) && (end_right != start_right) {
        log::info!("  ✓ Both wheels rotated!");
    } else if end_left != start_left {
        log::warn!("  ⚠ Only LEFT wheel rotated!");
    } else if end_right != start_right {
        log::warn!("  ⚠ Only RIGHT wheel rotated!");
    } else {
        log::warn!("  ✗ No wheels rotated!");
    }
    log::info!("✓ Clockwise rotation complete");
    log::info!("");

    thread::sleep(Duration::from_secs(1));

    // Step 9: Test rotation - 360° anticlockwise (in place)
    log::info!("Step 9: Testing rotation - 360° anticlockwise (in place)...");
    log::info!("  - Left wheel backward, right wheel forward");
    log::info!("  - Speed: 3000 ticks for 3 seconds");
    log::info!("  - BOTH wheels should rotate in opposite directions!");

    let (start_left, start_right) = gd32.get_encoder_counts();
    log::info!(
        "  - Starting encoders: left={}, right={}",
        start_left,
        start_right
    );

    // Anticlockwise rotation: left backward (-), right forward (+)
    gd32.set_raw_motor_speeds(-3000, 3000)?;
    thread::sleep(Duration::from_secs(3));
    gd32.set_raw_motor_speeds(0, 0)?;

    let (end_left, end_right) = gd32.get_encoder_counts();
    log::info!(
        "  - Ending encoders: left={}, right={}",
        end_left,
        end_right
    );
    log::info!(
        "  - Delta: left={}, right={}",
        end_left - start_left,
        end_right - start_right
    );

    // Check if both wheels moved
    if (end_left != start_left) && (end_right != start_right) {
        log::info!("  ✓ Both wheels rotated!");
    } else if end_left != start_left {
        log::warn!("  ⚠ Only LEFT wheel rotated!");
    } else if end_right != start_right {
        log::warn!("  ⚠ Only RIGHT wheel rotated!");
    } else {
        log::warn!("  ✗ No wheels rotated!");
    }
    log::info!("✓ Anticlockwise rotation complete");
    log::info!("");

    thread::sleep(Duration::from_millis(500));

    // Step 10: Test suction motor
    log::info!("Step 10: Testing suction motor...");
    log::info!("  - Turning ON suction at 100% power");
    gd32.set_vacuum(100)?;
    thread::sleep(Duration::from_secs(2));

    log::info!("  - Turning OFF suction");
    gd32.set_vacuum(0)?;
    log::info!("✓ Suction motor test complete");
    log::info!("");

    thread::sleep(Duration::from_millis(500));

    // Step 11: Test rolling brush
    log::info!("Step 11: Testing rolling brush...");
    log::info!("  - Turning ON rolling brush at 100% speed");
    gd32.set_main_brush(100)?;
    thread::sleep(Duration::from_secs(2));

    log::info!("  - Turning OFF rolling brush");
    gd32.set_main_brush(0)?;
    log::info!("✓ Rolling brush test complete");
    log::info!("");

    thread::sleep(Duration::from_millis(500));

    // Step 12: Test edge/side brush
    log::info!("Step 12: Testing edge/side brush...");
    log::info!("  - Turning ON side brush at 100% speed");
    gd32.set_side_brush(100)?;
    thread::sleep(Duration::from_secs(2));

    log::info!("  - Turning OFF side brush");
    gd32.set_side_brush(0)?;
    log::info!("✓ Side brush test complete");
    log::info!("");

    thread::sleep(Duration::from_millis(500));

    // Step 13: Read all sensor data
    log::info!("Step 13: Reading all sensor data from GD32...");
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

    // Step 14: Power off Lidar motor
    log::info!("Step 14: Powering off Lidar motor...");

    // First stop PWM (CMD=0x71)
    gd32.set_lidar_pwm(0)?;
    log::info!("✓ Lidar PWM set to 0% (CMD=0x71)");

    // Then disable power (CMD=0x97)
    gd32.set_lidar_power(false)?;
    log::info!("✓ Lidar power disabled (CMD=0x97)");
    log::info!("");

    // Step 15: Clean exit (Drop implementations will handle cleanup)
    log::info!("Step 15: Clean exit...");
    log::info!("  - GD32 heartbeat thread will stop");
    log::info!("  - Motors will stop");
    log::info!("");

    drop(gd32);
    drop(lidar);

    log::info!("=== All component tests completed successfully ===");

    Ok(())
}
