//! Integration tests for SangamIO connection and odometry.
//!
//! These tests require a live SangamIO daemon running on the robot.
//! Run with: cargo test -- --ignored
//!
//! Note: The wire format must match SangamIO's hardware.json configuration.
//! Default is JSON (wire_format: "json").

#[allow(unused_imports)]
use dhruva_slam::{
    ComplementaryConfig, ComplementaryFilter, LaserScan, Pose2D, PreprocessorConfig, SangamClient,
    ScanPreprocessor, WheelOdometry, WheelOdometryConfig, WireFormat,
};
use std::time::Duration;

/// Test connecting to SangamIO and receiving messages (JSON format).
///
/// Run with: cargo test test_sangam_connection -- --ignored
#[test]
#[ignore]
fn test_sangam_connection() {
    // Connect with JSON format (matches default SangamIO config)
    let mut client =
        SangamClient::connect("192.168.68.101:5555").expect("Failed to connect to SangamIO");

    println!("Using wire format: {:?}", client.wire_format());

    client
        .set_timeout(Some(Duration::from_secs(5)))
        .expect("Failed to set timeout");

    println!(
        "Connected to SangamIO from {}",
        client.local_addr().unwrap()
    );

    let mut lidar_count = 0;
    let mut encoder_count = 0;
    let mut gyro_count = 0;

    // Receive 100 messages and count types
    for i in 0..100 {
        match client.recv() {
            Ok(msg) => {
                if msg.as_lidar().is_some() {
                    lidar_count += 1;
                    if lidar_count == 1 {
                        let lidar = msg.as_lidar().unwrap();
                        println!(
                            "First lidar scan: {} points at timestamp {}",
                            lidar.data.len(),
                            lidar.timestamp_us
                        );
                    }
                }
                if msg.encoder_ticks().is_some() {
                    encoder_count += 1;
                    if encoder_count == 1 {
                        let (left, right) = msg.encoder_ticks().unwrap();
                        println!("First encoder reading: left={}, right={}", left, right);
                    }
                }
                if msg.gyro_z_raw().is_some() {
                    gyro_count += 1;
                    if gyro_count == 1 {
                        let gyro = msg.gyro_raw().unwrap();
                        println!(
                            "First gyro reading (raw): [{}, {}, {}]",
                            gyro[0], gyro[1], gyro[2]
                        );
                    }
                }

                if i % 20 == 0 {
                    println!(
                        "Message {}: topic={}, timestamp={}",
                        i,
                        msg.group_id(),
                        msg.timestamp_us()
                    );
                }
            }
            Err(e) => {
                println!("Error receiving message {}: {}", i, e);
                break;
            }
        }
    }

    println!("\n=== Summary ===");
    println!("Lidar messages: {}", lidar_count);
    println!("Encoder messages: {}", encoder_count);
    println!("Gyro messages: {}", gyro_count);

    // We should receive some encoder data at least (500Hz from sensor_status)
    assert!(encoder_count > 0, "Should receive encoder data");
}

/// Test timeout behavior
#[test]
#[ignore]
fn test_recv_timeout() {
    let mut client =
        SangamClient::connect("192.168.68.101:5555").expect("Failed to connect to SangamIO");

    // First, receive one message to ensure connection is working
    client
        .set_timeout(Some(Duration::from_secs(5)))
        .expect("Failed to set timeout");
    let msg = client.recv().expect("Failed to receive first message");
    println!("Received message from group: {}", msg.group_id());

    // Now test timeout - with very short timeout, we should sometimes get None
    for _ in 0..10 {
        match client.recv_timeout(Duration::from_millis(1)) {
            Ok(Some(msg)) => println!("Got message: {}", msg.group_id()),
            Ok(None) => println!("Timeout (expected)"),
            Err(e) => println!("Error: {}", e),
        }
    }
}

// ============================================================================
// Phase 2: Odometry Integration Tests
// ============================================================================

/// Test wheel odometry with live encoder data.
///
/// Run with: cargo test test_wheel_odometry_hardware -- --ignored --nocapture
///
/// Keep the robot stationary during this test to verify zero motion detection.
#[test]
#[ignore]
fn test_wheel_odometry_hardware() {
    let mut client =
        SangamClient::connect("192.168.68.101:5555").expect("Failed to connect to SangamIO");

    client
        .set_timeout(Some(Duration::from_secs(5)))
        .expect("Failed to set timeout");

    // Configure wheel odometry
    // NOTE: These values need calibration for your specific robot!
    let config = WheelOdometryConfig {
        ticks_per_meter: 1000.0, // Estimated, needs calibration
        wheel_base: 0.17,        // CRL-200S approximate wheel base
    };
    let mut odom = WheelOdometry::new(config);

    let mut total_distance = 0.0f32;
    let mut total_rotation = 0.0f32;
    let mut message_count = 0;

    println!("Collecting wheel odometry data for ~1 second...");
    println!("Keep the robot stationary for baseline test.");

    // Collect ~500 samples (at 500Hz, this is ~1 second)
    for _ in 0..500 {
        match client.recv() {
            Ok(msg) => {
                if let Some((left, right)) = msg.encoder_ticks() {
                    if let Some(delta) = odom.update(left, right) {
                        let dist = (delta.x * delta.x + delta.y * delta.y).sqrt();
                        total_distance += dist;
                        total_rotation += delta.theta.abs();
                        message_count += 1;

                        // Print occasional updates
                        if message_count % 100 == 0 {
                            println!(
                                "  Sample {}: encoders=({}, {}), delta=({:.4}, {:.4}, {:.4}°)",
                                message_count,
                                left,
                                right,
                                delta.x,
                                delta.y,
                                delta.theta.to_degrees()
                            );
                        }
                    }
                }
            }
            Err(e) => {
                println!("Error: {}", e);
                break;
            }
        }
    }

    println!("\n=== Wheel Odometry Summary ===");
    println!("Encoder updates processed: {}", message_count);
    println!("Total distance: {:.4} m", total_distance);
    println!("Total rotation: {:.2}°", total_rotation.to_degrees());

    // If robot was stationary, distance and rotation should be minimal
    // Allow for small noise in encoders
    if message_count > 0 {
        println!("\nIf robot was stationary, expect near-zero values (encoder noise only).");
    }
}

/// Test complementary filter with live encoder + gyro data.
///
/// Run with: cargo test test_complementary_filter_hardware -- --ignored --nocapture
#[test]
#[ignore]
fn test_complementary_filter_hardware() {
    let mut client =
        SangamClient::connect("192.168.68.101:5555").expect("Failed to connect to SangamIO");

    client
        .set_timeout(Some(Duration::from_secs(5)))
        .expect("Failed to set timeout");

    // Configure wheel odometry
    let odom_config = WheelOdometryConfig {
        ticks_per_meter: 1000.0,
        wheel_base: 0.17,
    };
    let mut odom = WheelOdometry::new(odom_config);

    // Configure complementary filter
    // NOTE: gyro_scale and gyro_bias need calibration!
    let filter_config = ComplementaryConfig {
        alpha: 0.98,
        gyro_scale: 0.001, // Estimated, needs calibration based on IMU datasheet
        gyro_bias_z: 0.0,  // Should be calibrated at startup when stationary
    };
    let mut filter = ComplementaryFilter::new(filter_config);

    let mut message_count = 0;
    let mut last_pose = Pose2D::identity();

    println!("Running complementary filter for ~2 seconds...");
    println!("Move the robot to see pose changes, or keep stationary for baseline.");

    // Collect ~1000 samples (at 500Hz, this is ~2 seconds)
    for _ in 0..1000 {
        match client.recv() {
            Ok(msg) => {
                // Get encoder data
                let encoder_delta = if let Some((left, right)) = msg.encoder_ticks() {
                    odom.update(left, right).unwrap_or(Pose2D::identity())
                } else {
                    continue;
                };

                // Get gyro data
                let gyro_z_raw = msg.gyro_z_raw().unwrap_or(0);
                let timestamp_us = msg.timestamp_us();

                // Update filter
                let pose = filter.update(encoder_delta, gyro_z_raw, timestamp_us);
                message_count += 1;

                // Print occasional updates
                if message_count % 200 == 0 {
                    println!(
                        "  Sample {}: pose=({:.3}, {:.3}, {:.1}°), gyro_z_raw={}",
                        message_count,
                        pose.x,
                        pose.y,
                        pose.theta.to_degrees(),
                        gyro_z_raw
                    );
                }

                last_pose = pose;
            }
            Err(e) => {
                println!("Error: {}", e);
                break;
            }
        }
    }

    println!("\n=== Complementary Filter Summary ===");
    println!("Updates processed: {}", message_count);
    println!(
        "Final pose: ({:.4}, {:.4}, {:.2}°)",
        last_pose.x,
        last_pose.y,
        last_pose.theta.to_degrees()
    );
}

/// Test gyro bias calibration.
///
/// Run with: cargo test test_gyro_bias_calibration -- --ignored --nocapture
///
/// Keep the robot COMPLETELY stationary during this test.
/// This will measure the gyro bias that should be used in ComplementaryConfig.
#[test]
#[ignore]
fn test_gyro_bias_calibration() {
    let mut client =
        SangamClient::connect("192.168.68.101:5555").expect("Failed to connect to SangamIO");

    client
        .set_timeout(Some(Duration::from_secs(5)))
        .expect("Failed to set timeout");

    println!("Calibrating gyro bias...");
    println!("KEEP THE ROBOT COMPLETELY STATIONARY!");
    println!("Collecting 1000 samples (~2 seconds)...\n");

    let mut gyro_sum = [0i64; 3];
    let mut sample_count = 0u64;

    for _ in 0..1000 {
        match client.recv() {
            Ok(msg) => {
                if let Some(gyro) = msg.gyro_raw() {
                    gyro_sum[0] += gyro[0] as i64;
                    gyro_sum[1] += gyro[1] as i64;
                    gyro_sum[2] += gyro[2] as i64;
                    sample_count += 1;
                }
            }
            Err(e) => {
                println!("Error: {}", e);
                break;
            }
        }
    }

    if sample_count > 0 {
        let bias_x = gyro_sum[0] as f32 / sample_count as f32;
        let bias_y = gyro_sum[1] as f32 / sample_count as f32;
        let bias_z = gyro_sum[2] as f32 / sample_count as f32;

        println!("=== Gyro Bias Calibration Results ===");
        println!("Samples collected: {}", sample_count);
        println!("Gyro X bias (raw): {:.2}", bias_x);
        println!("Gyro Y bias (raw): {:.2}", bias_y);
        println!("Gyro Z bias (raw): {:.2}", bias_z);
        println!("\nUse gyro_bias_z: {:.2} in ComplementaryConfig", bias_z);
    } else {
        println!("No gyro samples collected!");
    }
}

// ============================================================================
// Phase 3: Scan Processing Integration Tests
// ============================================================================

/// Test scan preprocessing with live LiDAR data.
///
/// Run with: cargo test test_scan_preprocessing_hardware -- --ignored --nocapture
#[test]
#[ignore]
fn test_scan_preprocessing_hardware() {
    let mut client =
        SangamClient::connect("192.168.68.101:5555").expect("Failed to connect to SangamIO");

    client
        .set_timeout(Some(Duration::from_secs(5)))
        .expect("Failed to set timeout");

    // Create preprocessor with default config
    let preprocessor = ScanPreprocessor::default();

    println!("Processing LiDAR scans...");
    println!("Waiting for lidar data (may take a few seconds)...\n");

    let mut scan_count = 0;
    let mut total_input_points = 0usize;
    let mut total_output_points = 0usize;

    // Process up to 10 scans (lidar comes at ~5Hz, so this takes ~2 seconds)
    for _ in 0..500 {
        // May need many sensor_status messages to get 10 lidar
        match client.recv() {
            Ok(msg) => {
                if let Some(lidar) = msg.as_lidar() {
                    // Convert to LaserScan
                    let scan = LaserScan::from_lidar_scan(lidar.data);
                    let input_count = scan.len();

                    // Process through pipeline
                    let cloud = preprocessor.process(&scan);
                    let output_count = cloud.len();

                    total_input_points += input_count;
                    total_output_points += output_count;
                    scan_count += 1;

                    println!(
                        "Scan {}: {} points → {} points ({:.1}% reduction)",
                        scan_count,
                        input_count,
                        output_count,
                        100.0 * (1.0 - output_count as f32 / input_count.max(1) as f32)
                    );

                    // Print bounds
                    if let Some((min, max)) = cloud.bounds() {
                        println!(
                            "  Bounds: ({:.2}, {:.2}) to ({:.2}, {:.2})",
                            min.x, min.y, max.x, max.y
                        );
                    }

                    if scan_count >= 10 {
                        break;
                    }
                }
            }
            Err(e) => {
                println!("Error: {}", e);
                break;
            }
        }
    }

    println!("\n=== Scan Preprocessing Summary ===");
    println!("Scans processed: {}", scan_count);
    if scan_count > 0 {
        println!(
            "Average: {} points → {} points",
            total_input_points / scan_count,
            total_output_points / scan_count
        );
    }
}

/// Test full preprocessing pipeline performance.
///
/// Run with: cargo test test_preprocessing_performance -- --ignored --nocapture
#[test]
#[ignore]
fn test_preprocessing_performance() {
    let mut client =
        SangamClient::connect("192.168.68.101:5555").expect("Failed to connect to SangamIO");

    client
        .set_timeout(Some(Duration::from_secs(5)))
        .expect("Failed to set timeout");

    let preprocessor = ScanPreprocessor::default();

    println!("Measuring preprocessing performance...");
    println!("Collecting 50 scans for timing analysis...\n");

    let mut processing_times = Vec::new();

    for _ in 0..1000 {
        // Look for up to 50 lidar scans
        match client.recv() {
            Ok(msg) => {
                if let Some(lidar) = msg.as_lidar() {
                    let scan = LaserScan::from_lidar_scan(lidar.data);

                    // Time the preprocessing
                    let start = std::time::Instant::now();
                    let _cloud = preprocessor.process(&scan);
                    let elapsed = start.elapsed();

                    processing_times.push(elapsed.as_micros() as f64);

                    if processing_times.len() >= 50 {
                        break;
                    }
                }
            }
            Err(e) => {
                println!("Error: {}", e);
                break;
            }
        }
    }

    if !processing_times.is_empty() {
        let avg = processing_times.iter().sum::<f64>() / processing_times.len() as f64;
        let max = processing_times.iter().cloned().fold(0.0, f64::max);
        let min = processing_times.iter().cloned().fold(f64::MAX, f64::min);

        println!("=== Preprocessing Performance ===");
        println!("Scans timed: {}", processing_times.len());
        println!("Average: {:.1} µs ({:.2} ms)", avg, avg / 1000.0);
        println!("Min: {:.1} µs", min);
        println!("Max: {:.1} µs", max);
        println!(
            "\nTarget: <5ms on A33 (this host: {:.2} ms avg)",
            avg / 1000.0
        );
    } else {
        println!("No lidar scans received!");
    }
}
