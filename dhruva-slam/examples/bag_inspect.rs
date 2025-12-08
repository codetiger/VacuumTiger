//! Bag file inspector for analyzing recorded sensor data.
//!
//! Usage:
//!   cargo run --example bag_inspect -- <bag_file>           - Show bag file info
//!   cargo run --example bag_inspect -- <bag_file> --summary - Show summary
//!   cargo run --example bag_inspect -- <bag_file> --timing  - Show timing analysis
//!   cargo run --example bag_inspect -- <bag_file> --imu     - Detailed IMU analysis
//!   cargo run --example bag_inspect -- <bag_file> --pose    - Pose estimation debug
//!   cargo run --example bag_inspect -- <bag_file> --verbose - Verbose info

use dhruva_slam::core::types::{LaserScan, Pose2D};
use dhruva_slam::io::bag::{BagMessage, BagPlayer};
use dhruva_slam::sensors::odometry::{WheelOdometry, WheelOdometryConfig};
use dhruva_slam::sensors::preprocessing::{PreprocessorConfig, ScanPreprocessor};
use dhruva_slam::utils::{
    GYRO_SCALE, GYRO_SIGN, I16Stats, WHEEL_BASE, WHEEL_TICKS_PER_METER, std_dev_i16,
};
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!(
            "Usage: {} <bag_file> [--summary | --timing | --imu | --pose | --verbose]",
            args[0]
        );
        std::process::exit(1);
    }

    let bag_path = &args[1];
    let mode = args.get(2).map(|s| s.as_str());

    match mode {
        Some("--timing") => show_timing_analysis(bag_path),
        Some("--imu") => show_imu_analysis(bag_path),
        Some("--pose") => show_pose_analysis(bag_path),
        Some("--summary") => show_summary(bag_path),
        Some("--verbose") => show_info(bag_path, true),
        _ => show_info(bag_path, false),
    }
}

/// Pose estimation debug analysis
fn show_pose_analysis(bag_path: &str) {
    use dhruva_slam::algorithms::matching::{
        HybridP2LMatcher, HybridP2LMatcherConfig, ScanMatcher,
    };

    let player = BagPlayer::open(bag_path).expect("Failed to open bag file");

    println!("╔══════════════════════════════════════════════════════════════════╗");
    println!("║                    POSE DEBUG ANALYSIS                           ║");
    println!("╚══════════════════════════════════════════════════════════════════╝\n");

    let odom_config = WheelOdometryConfig {
        ticks_per_meter: WHEEL_TICKS_PER_METER,
        wheel_base: WHEEL_BASE,
    };
    let mut wheel_odom = WheelOdometry::new(odom_config);
    let preprocessor = ScanPreprocessor::new(PreprocessorConfig::default());
    let mut matcher = HybridP2LMatcher::new(HybridP2LMatcherConfig::default());

    // Track poses
    let mut odom_only_pose = Pose2D::identity(); // Pure odometry
    let mut slam_pose = Pose2D::identity(); // Scan-matched pose (like in benchmark)
    let mut prev_odom_at_scan = Pose2D::identity();
    let mut odom_pose = Pose2D::identity();

    // Previous scan for matching
    use dhruva_slam::core::types::PointCloud2D;
    let mut previous_scan: Option<PointCloud2D> = None;

    // Encoder tracking
    let mut first_encoder: Option<(u16, u16)> = None;
    let mut last_encoder: Option<(u16, u16)> = None;

    let mut scan_count = 0;
    let mut sensor_count = 0u64;

    println!("Processing bag: {}\n", bag_path);

    for msg in player {
        match msg {
            Ok(BagMessage::SensorStatus(status)) => {
                sensor_count += 1;

                if first_encoder.is_none() {
                    first_encoder = Some((status.encoder.left, status.encoder.right));
                }
                last_encoder = Some((status.encoder.left, status.encoder.right));

                if let Some(delta) = wheel_odom.update(status.encoder.left, status.encoder.right) {
                    // Compose delta into global pose
                    odom_pose = odom_pose.compose(&delta);

                    // Show first few deltas
                    if sensor_count <= 5 {
                        println!(
                            "  Sensor #{}: enc=[{}, {}] delta=(x={:.4}, y={:.4}, θ={:.2}°)",
                            sensor_count,
                            status.encoder.left,
                            status.encoder.right,
                            delta.x,
                            delta.y,
                            delta.theta.to_degrees()
                        );
                    }
                }
            }
            Ok(BagMessage::Lidar(timestamped)) => {
                scan_count += 1;
                let laser_scan = LaserScan::from_lidar_scan(&timestamped.data);
                let processed = preprocessor.process(&laser_scan);

                if processed.len() >= 50 {
                    // Compute odom delta since last scan
                    let odom_delta = prev_odom_at_scan.inverse().compose(&odom_pose);

                    // Update pure odometry pose
                    odom_only_pose = odom_only_pose.compose(&odom_delta);

                    // Now do scan matching (exactly like in slam_benchmark.rs)
                    if let Some(ref prev_scan) = previous_scan {
                        // EXPERIMENT: Try using odom_delta directly without inversion
                        // The matcher should find the transform that aligns source→target
                        // which is the robot motion itself
                        let initial_guess = odom_delta; // Changed from odom_delta.inverse()
                        let match_result =
                            matcher.match_scans(&processed, prev_scan, &initial_guess);

                        // The match result gives transform from current→previous
                        // Invert to get motion from previous→current
                        let matched_delta = match_result.transform.inverse();

                        // The match_result.transform aligns source to target
                        // i.e., it transforms points from source frame so they overlap target
                        // This is the same as robot motion from previous to current!
                        // BUG FIX: Do NOT invert - use match_result.transform directly
                        let matched_delta_fixed = match_result.transform;

                        // For debugging, show the transforms
                        if scan_count <= 10 {
                            println!("\nScan #{} (at sensor #{})", scan_count, sensor_count);
                            println!("  Points in scan: {}", processed.len());
                            println!(
                                "  Odom delta (prev→curr): (x={:.4}m, y={:.4}m, θ={:.2}°)",
                                odom_delta.x,
                                odom_delta.y,
                                odom_delta.theta.to_degrees()
                            );
                            println!(
                                "  Initial guess (odom direct): (x={:.4}m, y={:.4}m, θ={:.2}°)",
                                initial_guess.x,
                                initial_guess.y,
                                initial_guess.theta.to_degrees()
                            );
                            println!(
                                "  Match result (raw): (x={:.4}m, y={:.4}m, θ={:.2}°) score={:.3}",
                                match_result.transform.x,
                                match_result.transform.y,
                                match_result.transform.theta.to_degrees(),
                                match_result.score
                            );
                            println!(
                                "  Old matched delta (inverted): (x={:.4}m, y={:.4}m, θ={:.2}°)",
                                matched_delta.x,
                                matched_delta.y,
                                matched_delta.theta.to_degrees()
                            );
                            println!(
                                "  NEW matched delta (direct): (x={:.4}m, y={:.4}m, θ={:.2}°)",
                                matched_delta_fixed.x,
                                matched_delta_fixed.y,
                                matched_delta_fixed.theta.to_degrees()
                            );
                        }

                        // Update SLAM pose with FIXED matched delta (no inversion!)
                        slam_pose = slam_pose.compose(&matched_delta_fixed);
                    }

                    if scan_count <= 10 || scan_count % 50 == 0 {
                        if scan_count > 1 {
                            println!(
                                "  SLAM pose: (x={:.4}m, y={:.4}m, θ={:.2}°)",
                                slam_pose.x,
                                slam_pose.y,
                                slam_pose.theta.to_degrees()
                            );
                            println!(
                                "  Odom pose: (x={:.4}m, y={:.4}m, θ={:.2}°)",
                                odom_only_pose.x,
                                odom_only_pose.y,
                                odom_only_pose.theta.to_degrees()
                            );
                        }
                    }

                    prev_odom_at_scan = odom_pose;
                    previous_scan = Some(processed);
                }
            }
            _ => {}
        }
    }

    // Replace global_pose with slam_pose for validation
    let global_pose = slam_pose;

    // Summary
    println!("\n╔══════════════════════════════════════════════════════════════════╗");
    println!("║                         SUMMARY                                  ║");
    println!("╚══════════════════════════════════════════════════════════════════╝\n");

    if let (Some((l1, r1)), Some((l2, r2))) = (first_encoder, last_encoder) {
        let delta_left = l2 as i32 - l1 as i32;
        let delta_right = r2 as i32 - r1 as i32;
        let avg_ticks = (delta_left + delta_right) as f32 / 2.0;
        let distance_m = avg_ticks / WHEEL_TICKS_PER_METER;

        println!("Encoder Analysis:");
        println!("  First: left={}, right={}", l1, r1);
        println!("  Last:  left={}, right={}", l2, r2);
        println!("  Delta: left={}, right={}", delta_left, delta_right);
        println!(
            "  Expected distance: {:.3}m (using {} ticks/m)",
            distance_m, WHEEL_TICKS_PER_METER
        );
        println!();
    }

    println!("Final Poses:");
    println!(
        "  Raw odom_pose: (x={:.4}m, y={:.4}m, θ={:.2}°)",
        odom_pose.x,
        odom_pose.y,
        odom_pose.theta.to_degrees()
    );
    println!(
        "  Odom-only accumulated: (x={:.4}m, y={:.4}m, θ={:.2}°)",
        odom_only_pose.x,
        odom_only_pose.y,
        odom_only_pose.theta.to_degrees()
    );
    println!(
        "  SLAM pose (scan-matched): (x={:.4}m, y={:.4}m, θ={:.2}°)",
        global_pose.x,
        global_pose.y,
        global_pose.theta.to_degrees()
    );

    println!();
    println!("Scan statistics:");
    println!("  Total sensor messages: {}", sensor_count);
    println!("  Total lidar scans: {}", scan_count);

    // Expected vs actual check
    if let (Some((l1, r1)), Some((l2, r2))) = (first_encoder, last_encoder) {
        let delta_left = l2 as i32 - l1 as i32;
        let delta_right = r2 as i32 - r1 as i32;
        let expected_x = ((delta_left + delta_right) as f32 / 2.0) / WHEEL_TICKS_PER_METER;

        println!();
        println!("═══════════════════════════════════════════════════════════════════");
        println!("                        VALIDATION");
        println!("═══════════════════════════════════════════════════════════════════");
        println!();

        if delta_left > 0 && delta_right > 0 {
            println!(
                "Motion detected: FORWARD ({} ticks)",
                (delta_left + delta_right) / 2
            );
            println!("Expected X position: {:.4}m", expected_x);
            println!("Actual X position:   {:.4}m", global_pose.x);

            if global_pose.x > 0.0 && expected_x > 0.0 {
                println!("  ✓ Sign is CORRECT (positive X for forward motion)");
            } else if global_pose.x < 0.0 && expected_x > 0.0 {
                println!("  ✗ Sign is WRONG! (negative X but robot moved forward)");
                println!("    This indicates a coordinate frame issue.");
            }
        } else if delta_left < 0 && delta_right < 0 {
            println!("Motion detected: BACKWARD");
        } else {
            println!("Motion detected: MIXED/ROTATION");
        }
    }
}

/// Detailed IMU analysis comparing gyroscope with encoder-derived rotation
fn show_imu_analysis(bag_path: &str) {
    let player = BagPlayer::open(bag_path).expect("Failed to open bag file");

    println!("╔══════════════════════════════════════════════════════════════════╗");
    println!("║                    IMU vs ENCODER ANALYSIS                       ║");
    println!("╚══════════════════════════════════════════════════════════════════╝");
    println!();

    // Gyro statistics
    let mut gyro_x_values: Vec<i16> = Vec::new();
    let mut gyro_y_values: Vec<i16> = Vec::new();
    let mut gyro_z_values: Vec<i16> = Vec::new();

    // Accel statistics
    let mut accel_x_values: Vec<i16> = Vec::new();
    let mut accel_y_values: Vec<i16> = Vec::new();
    let mut accel_z_values: Vec<i16> = Vec::new();

    // For integration (ROS REP-103: Z axis is yaw)
    let mut last_timestamp_us: Option<u64> = None;
    let mut integrated_gyro_z_rad: f64 = 0.0; // Yaw (heading) - primary for 2D nav
    let mut integrated_gyro_x_rad: f64 = 0.0; // Roll - for reference only

    // Wheel odometry for comparison
    let odom_config = WheelOdometryConfig {
        ticks_per_meter: WHEEL_TICKS_PER_METER,
        wheel_base: WHEEL_BASE,
    };
    let mut wheel_odom = WheelOdometry::new(odom_config);
    let mut encoder_total_theta_rad: f64 = 0.0;

    let mut sensor_count = 0u64;
    let mut first_timestamp_us: Option<u64> = None;
    let mut last_ts_for_duration: u64 = 0;

    for msg in player {
        if let Ok(BagMessage::SensorStatus(status)) = msg {
            sensor_count += 1;

            // Store values for statistics
            gyro_x_values.push(status.gyro_raw[0]);
            gyro_y_values.push(status.gyro_raw[1]);
            gyro_z_values.push(status.gyro_raw[2]);
            accel_x_values.push(status.accel_raw[0]);
            accel_y_values.push(status.accel_raw[1]);
            accel_z_values.push(status.accel_raw[2]);

            if first_timestamp_us.is_none() {
                first_timestamp_us = Some(status.timestamp_us);
            }
            last_ts_for_duration = status.timestamp_us;

            // Integrate gyroscope (ROS REP-103: gyro_raw[2] is yaw/Z axis)
            if let Some(last_ts) = last_timestamp_us
                && status.timestamp_us > last_ts
            {
                let dt_s = (status.timestamp_us - last_ts) as f64 / 1_000_000.0;
                // Z axis is yaw (heading) - this is what we compare to encoder rotation
                let gyro_z_rad_s = status.gyro_raw[2] as f64 * GYRO_SCALE as f64 * GYRO_SIGN as f64;
                // X axis is roll - included for reference only
                let gyro_x_rad_s = status.gyro_raw[0] as f64 * GYRO_SCALE as f64;
                integrated_gyro_z_rad += gyro_z_rad_s * dt_s;
                integrated_gyro_x_rad += gyro_x_rad_s * dt_s;
            }
            last_timestamp_us = Some(status.timestamp_us);

            // Compute encoder-derived rotation
            if let Some(delta) = wheel_odom.update(status.encoder.left, status.encoder.right) {
                encoder_total_theta_rad += delta.theta as f64;
            }
        }
    }

    let duration_s = first_timestamp_us
        .map(|first| (last_ts_for_duration - first) as f64 / 1_000_000.0)
        .unwrap_or(0.0);

    println!("Recording duration: {:.2} seconds", duration_s);
    println!("Sensor messages: {}", sensor_count);
    println!(
        "Sample rate: {:.1} Hz",
        if duration_s > 0.0 {
            sensor_count as f64 / duration_s
        } else {
            0.0
        }
    );
    println!();

    // Gyroscope statistics
    println!("═══════════════════════════════════════════════════════════════════");
    println!("                        GYROSCOPE STATISTICS");
    println!("═══════════════════════════════════════════════════════════════════");
    println!("  (Raw units: 0.01 deg/s, i.e. 1 raw = 0.01 deg/s)");
    println!();

    if let Some(stats) = I16Stats::compute(&gyro_x_values) {
        stats.print("Gyro X");
    }
    if let Some(stats) = I16Stats::compute(&gyro_y_values) {
        stats.print("Gyro Y");
    }
    if let Some(stats) = I16Stats::compute(&gyro_z_values) {
        stats.print("Gyro Z");
    }

    // Accelerometer statistics
    println!();
    println!("═══════════════════════════════════════════════════════════════════");
    println!("                      ACCELEROMETER STATISTICS");
    println!("═══════════════════════════════════════════════════════════════════");
    println!();

    if let Some(stats) = I16Stats::compute(&accel_x_values) {
        stats.print("Accel X");
    }
    if let Some(stats) = I16Stats::compute(&accel_y_values) {
        stats.print("Accel Y");
    }
    if let Some(stats) = I16Stats::compute(&accel_z_values) {
        stats.print("Accel Z");
    }

    // Rotation comparison
    println!();
    println!("═══════════════════════════════════════════════════════════════════");
    println!("                       ROTATION COMPARISON");
    println!("═══════════════════════════════════════════════════════════════════");
    println!();

    let gyro_z_mean = if !gyro_z_values.is_empty() {
        gyro_z_values.iter().map(|&v| v as f64).sum::<f64>() / gyro_z_values.len() as f64
    } else {
        0.0
    };
    let _gyro_x_mean = if !gyro_x_values.is_empty() {
        gyro_x_values.iter().map(|&v| v as f64).sum::<f64>() / gyro_x_values.len() as f64
    } else {
        0.0
    };

    println!(
        "Encoder rotation:   {:>8.2}° ({:.4} rad)",
        encoder_total_theta_rad.to_degrees(),
        encoder_total_theta_rad
    );
    println!();

    // Show gyro Z (yaw axis per ROS REP-103 - PRIMARY for 2D navigation)
    println!("--- Gyro Z (Yaw axis, ROS REP-103) ---");
    println!(
        "Integrated Gyro Z:  {:>8.2}° ({:.4} rad)",
        integrated_gyro_z_rad.to_degrees(),
        integrated_gyro_z_rad
    );
    let diff_z_deg = (integrated_gyro_z_rad - encoder_total_theta_rad).to_degrees();
    println!("Difference:         {:>8.2}°", diff_z_deg);

    // Bias-corrected gyro Z
    let bias_corrected_gyro_z_rad =
        integrated_gyro_z_rad - (gyro_z_mean * GYRO_SCALE as f64 * GYRO_SIGN as f64 * duration_s);
    println!(
        "Gyro Z mean (bias): {:>8.2} raw ({:.4} deg/s)",
        gyro_z_mean,
        gyro_z_mean * 0.01025 // 0.01025 deg/s per raw unit (calibrated)
    );
    println!(
        "Bias-corrected:     {:>8.2}° ({:.4} rad)",
        bias_corrected_gyro_z_rad.to_degrees(),
        bias_corrected_gyro_z_rad
    );
    let bias_corrected_z_diff = (bias_corrected_gyro_z_rad - encoder_total_theta_rad).to_degrees();
    println!("Corrected diff:     {:>8.2}°", bias_corrected_z_diff);

    println!();

    // Show gyro X (roll axis, for reference only)
    println!("--- Gyro X (Roll axis, for reference) ---");
    println!(
        "Integrated Gyro X:  {:>8.2}° ({:.4} rad)",
        integrated_gyro_x_rad.to_degrees(),
        integrated_gyro_x_rad
    );
    let diff_x_deg = (integrated_gyro_x_rad - encoder_total_theta_rad).to_degrees();
    println!("Difference:         {:>8.2}°", diff_x_deg);

    // Use gyro Z for main assessment (correct yaw axis per ROS REP-103)
    let diff_deg = diff_z_deg;
    let _bias_corrected_diff = bias_corrected_z_diff;

    // Drift rate
    if duration_s > 0.0 {
        let drift_rate_deg_s = diff_deg / duration_s;
        println!();
        println!("Raw gyro drift rate: {:.4} deg/s", drift_rate_deg_s);
        println!(
            "                     {:.2} deg/min",
            drift_rate_deg_s * 60.0
        );
    }

    // Quality assessment
    println!();
    println!("═══════════════════════════════════════════════════════════════════");
    println!("                        QUALITY ASSESSMENT");
    println!("═══════════════════════════════════════════════════════════════════");
    println!();

    let gyro_z_std = std_dev_i16(&gyro_z_values);
    let encoder_moved = encoder_total_theta_rad.abs() > 0.01; // More than ~0.5 degrees

    if !encoder_moved {
        println!("Scenario: STATIC (robot stationary)");
        println!();
        if gyro_z_std < 5.0 {
            println!("  [OK] Gyro noise is low (std={:.2})", gyro_z_std);
        } else {
            println!("  [WARN] Gyro noise is high (std={:.2})", gyro_z_std);
        }

        if gyro_z_mean.abs() < 10.0 {
            println!("  [OK] Gyro bias is small (mean={:.2})", gyro_z_mean);
        } else {
            println!(
                "  [WARN] Gyro has significant bias (mean={:.2}, {:.2} deg/s)",
                gyro_z_mean,
                gyro_z_mean * 1.0
            );
        }
    } else {
        println!(
            "Scenario: MOTION (encoder detected {:.1}° rotation)",
            encoder_total_theta_rad.to_degrees()
        );
        println!();

        // For motion scenarios, compare RAW integrated gyro Z (not bias-corrected)
        // Bias correction during motion removes the actual rotation signal!
        let agreement_pct = if encoder_total_theta_rad.abs() > 0.01 {
            100.0
                * (1.0
                    - (integrated_gyro_z_rad - encoder_total_theta_rad).abs()
                        / encoder_total_theta_rad.abs())
        } else {
            0.0
        };

        if agreement_pct > 95.0 {
            println!(
                "  [EXCELLENT] Gyro and encoder agree very well ({:.1}%)",
                agreement_pct
            );
        } else if agreement_pct > 90.0 {
            println!("  [OK] Gyro and encoder agree well ({:.1}%)", agreement_pct);
        } else if agreement_pct > 70.0 {
            println!("  [WARN] Moderate agreement ({:.1}%)", agreement_pct);
        } else {
            println!(
                "  [BAD] Poor agreement ({:.1}%) - check IMU calibration",
                agreement_pct
            );
        }
    }
}

fn show_timing_analysis(bag_path: &str) {
    let player = BagPlayer::open(bag_path).expect("Failed to open bag file");

    println!("╔══════════════════════════════════════════════════════════════════╗");
    println!("║             MESSAGE TIMING AND INTERLEAVING ANALYSIS             ║");
    println!("╚══════════════════════════════════════════════════════════════════╝");
    println!();

    let mut lidar_count = 0;
    let mut sensor_count_between_lidar = 0;
    let mut last_lidar_ts: Option<u64> = None;
    let mut first_lidar_ts: Option<u64> = None;

    // Track sensor messages between consecutive lidar scans
    let mut sensors_per_lidar: Vec<u32> = Vec::new();
    let mut lidar_intervals_ms: Vec<f64> = Vec::new();
    let mut encoder_at_lidar: Vec<(u16, u16)> = Vec::new();
    let mut last_sensor_enc: Option<(u16, u16)> = None;

    // For first 10 lidar scans, show detailed interleaving
    let mut show_detail = true;

    for msg in player {
        match msg {
            Ok(BagMessage::SensorStatus(status)) => {
                sensor_count_between_lidar += 1;
                last_sensor_enc = Some((status.encoder.left, status.encoder.right));

                // Show first few sensor messages after each of the first 5 lidar scans
                if show_detail && lidar_count <= 5 && sensor_count_between_lidar <= 3 {
                    let rel_ts = if let Some(lidar_ts) = last_lidar_ts {
                        status.timestamp_us.saturating_sub(lidar_ts) as f64 / 1000.0
                    } else {
                        // Before first lidar - show as negative relative to a reference
                        -(first_lidar_ts
                            .unwrap_or(status.timestamp_us)
                            .saturating_sub(status.timestamp_us) as f64)
                            / 1000.0
                    };
                    println!(
                        "    Sensor #{} at {:+.1}ms: enc=[{}, {}] ts={}",
                        sensor_count_between_lidar,
                        rel_ts,
                        status.encoder.left,
                        status.encoder.right,
                        status.timestamp_us
                    );
                }
            }
            Ok(BagMessage::Lidar(timestamped)) => {
                lidar_count += 1;

                // Record sensor count between this and previous lidar
                if lidar_count > 1 {
                    sensors_per_lidar.push(sensor_count_between_lidar);
                }

                // Compute interval from previous lidar
                if let Some(prev_ts) = last_lidar_ts {
                    let interval_us = timestamped.timestamp_us.saturating_sub(prev_ts);
                    lidar_intervals_ms.push(interval_us as f64 / 1000.0);
                }

                if first_lidar_ts.is_none() {
                    first_lidar_ts = Some(timestamped.timestamp_us);
                }

                // Show lidar arrival for first 10
                if show_detail && lidar_count <= 10 {
                    let rel_ts = timestamped
                        .timestamp_us
                        .saturating_sub(first_lidar_ts.unwrap());
                    let interval = if let Some(prev) = last_lidar_ts {
                        format!(
                            " (interval: {:.1}ms)",
                            (timestamped.timestamp_us - prev) as f64 / 1000.0
                        )
                    } else {
                        String::new()
                    };

                    // Show encoder delta from previous lidar
                    let enc_delta = if let Some((prev_l, prev_r)) = encoder_at_lidar.last() {
                        let curr_l = last_sensor_enc.unwrap_or((0, 0)).0;
                        let curr_r = last_sensor_enc.unwrap_or((0, 0)).1;
                        format!(
                            " enc_delta=[{}, {}]",
                            curr_l as i32 - *prev_l as i32,
                            curr_r as i32 - *prev_r as i32
                        )
                    } else {
                        String::new()
                    };

                    println!(
                        "\nLidar #{} at t={:.1}ms{} - {} sensor msgs{}",
                        lidar_count,
                        rel_ts as f64 / 1000.0,
                        interval,
                        sensor_count_between_lidar,
                        enc_delta
                    );
                }

                // Track encoder at lidar time
                if let Some(enc) = last_sensor_enc {
                    encoder_at_lidar.push(enc);
                }

                if lidar_count == 10 {
                    show_detail = false;
                    println!("\n... (showing first 10 lidar scans only) ...\n");
                }

                last_lidar_ts = Some(timestamped.timestamp_us);
                sensor_count_between_lidar = 0;
            }
            _ => {}
        }
    }

    // Print statistics
    println!("\n========== TIMING STATISTICS ==========\n");
    println!("Total lidar scans: {}", lidar_count);
    println!(
        "Sensors per lidar interval: {} measurements",
        sensors_per_lidar.len()
    );

    if !sensors_per_lidar.is_empty() {
        let sum: u32 = sensors_per_lidar.iter().sum();
        let avg = sum as f64 / sensors_per_lidar.len() as f64;
        let min = *sensors_per_lidar.iter().min().unwrap();
        let max = *sensors_per_lidar.iter().max().unwrap();
        println!("  Average sensor msgs between lidars: {:.1}", avg);
        println!("  Min: {}, Max: {}", min, max);

        // Expected at 110Hz sensors and 5Hz lidar: ~22 sensor messages
        println!("  Expected (110Hz/5Hz): ~22 sensor msgs");
    }

    if !lidar_intervals_ms.is_empty() {
        let sum: f64 = lidar_intervals_ms.iter().sum();
        let avg = sum / lidar_intervals_ms.len() as f64;
        let min = lidar_intervals_ms
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max = lidar_intervals_ms.iter().cloned().fold(0.0f64, f64::max);
        println!("\nLidar interval:");
        println!("  Average: {:.1}ms ({:.1}Hz)", avg, 1000.0 / avg);
        println!("  Min: {:.1}ms, Max: {:.1}ms", min, max);
    }
}

fn show_summary(bag_path: &str) {
    let player = BagPlayer::open(bag_path).expect("Failed to open bag file");

    let mut sensor_count = 0;
    let mut lidar_count = 0;
    let mut first_encoder: Option<(u16, u16)> = None;
    let mut last_encoder: Option<(u16, u16)> = None;
    let mut first_gyro: Option<[i16; 3]> = None;
    let mut last_gyro: Option<[i16; 3]> = None;
    let mut first_accel: Option<[i16; 3]> = None;
    let mut last_accel: Option<[i16; 3]> = None;

    // Track unique values
    let mut unique_encoder_left: std::collections::HashSet<u16> = std::collections::HashSet::new();
    let mut unique_encoder_right: std::collections::HashSet<u16> = std::collections::HashSet::new();
    let mut unique_gyro_z: std::collections::HashSet<i16> = std::collections::HashSet::new();

    for msg in player {
        match msg {
            Ok(BagMessage::SensorStatus(status)) => {
                sensor_count += 1;
                let enc = &status.encoder;
                let gyro = &status.gyro_raw;
                let accel = &status.accel_raw;

                unique_encoder_left.insert(enc.left);
                unique_encoder_right.insert(enc.right);
                unique_gyro_z.insert(gyro[2]);

                if first_encoder.is_none() {
                    first_encoder = Some((enc.left, enc.right));
                    first_gyro = Some(*gyro);
                    first_accel = Some(*accel);
                }
                last_encoder = Some((enc.left, enc.right));
                last_gyro = Some(*gyro);
                last_accel = Some(*accel);

                // Print first 10 and every 1000th message
                if sensor_count <= 10 || sensor_count % 1000 == 0 {
                    println!(
                        "Sensor #{}: enc=[{}, {}] gyro=[{}, {}, {}] accel=[{}, {}, {}]",
                        sensor_count,
                        enc.left,
                        enc.right,
                        gyro[0],
                        gyro[1],
                        gyro[2],
                        accel[0],
                        accel[1],
                        accel[2]
                    );
                }
            }
            Ok(BagMessage::Lidar(_)) => {
                lidar_count += 1;
            }
            _ => {}
        }
    }

    println!("\n========== BAG FILE SUMMARY ==========");
    println!("Total sensor messages: {}", sensor_count);
    println!("Total lidar messages: {}", lidar_count);

    if let (Some(first), Some(last)) = (first_encoder, last_encoder) {
        println!("\nEncoder:");
        println!("  First: left={}, right={}", first.0, first.1);
        println!("  Last:  left={}, right={}", last.0, last.1);
        let delta_left = last.0 as i32 - first.0 as i32;
        let delta_right = last.1 as i32 - first.1 as i32;
        println!("  Delta: left={}, right={}", delta_left, delta_right);
        println!("  Unique left values: {}", unique_encoder_left.len());
        println!("  Unique right values: {}", unique_encoder_right.len());
    }

    if let (Some(first), Some(last)) = (first_gyro, last_gyro) {
        println!("\nGyroscope:");
        println!("  First: [{}, {}, {}]", first[0], first[1], first[2]);
        println!("  Last:  [{}, {}, {}]", last[0], last[1], last[2]);
        println!("  Unique Z values: {}", unique_gyro_z.len());
    }

    if let (Some(first), Some(last)) = (first_accel, last_accel) {
        println!("\nAccelerometer:");
        println!("  First: [{}, {}, {}]", first[0], first[1], first[2]);
        println!("  Last:  [{}, {}, {}]", last[0], last[1], last[2]);
    }
}

/// Show bag file info (header, message counts, rates, file size).
/// This is the default mode when no flags are provided.
fn show_info(bag_path: &str, verbose: bool) {
    let mut player = BagPlayer::open(bag_path).expect("Failed to open bag file");

    // Extract header values
    let version = player.header().version;
    let flags = player.header().flags;
    let start_time_us = player.header().start_time_us;
    let end_time_us = player.header().end_time_us;
    let duration_secs = player.header().duration_secs();
    let message_count = player.header().message_count;

    println!("Bag File Information");
    println!("====================");
    println!("File: {}", bag_path);
    println!();

    println!("Header Information:");
    println!("  Format version: {}", version);
    println!("  Flags: 0x{:04x}", flags);
    println!(
        "  Start time: {} us ({:.3} s)",
        start_time_us,
        start_time_us as f64 / 1_000_000.0
    );
    println!(
        "  End time: {} us ({:.3} s)",
        end_time_us,
        end_time_us as f64 / 1_000_000.0
    );
    println!("  Duration: {:.3} seconds", duration_secs);
    println!("  Message count (header): {}", message_count);
    println!();

    // Count messages
    println!("Scanning messages...");
    player.rewind().expect("Failed to rewind bag file");

    let mut sensor_count = 0u64;
    let mut lidar_count = 0u64;
    let mut odometry_count = 0u64;
    let mut total_count = 0u64;

    let mut first_timestamp: Option<u64> = None;
    let mut last_timestamp = 0u64;

    // Collect first few messages for verbose mode
    let mut first_sensors: Vec<(u64, (u16, u16), [i16; 3])> = Vec::new();
    let mut first_lidar: Option<(u64, usize, f32, f32)> = None;

    for msg in (&mut player).flatten() {
        total_count += 1;
        let ts = msg.timestamp_us();

        if first_timestamp.is_none() {
            first_timestamp = Some(ts);
        }
        last_timestamp = ts;

        match &msg {
            BagMessage::SensorStatus(status) => {
                sensor_count += 1;
                if verbose && first_sensors.len() < 5 {
                    first_sensors.push((
                        status.timestamp_us,
                        (status.encoder.left, status.encoder.right),
                        status.gyro_raw,
                    ));
                }
            }
            BagMessage::Lidar(lidar) => {
                lidar_count += 1;
                if verbose && first_lidar.is_none() && !lidar.data.is_empty() {
                    let (min_range, max_range) = lidar
                        .data
                        .iter()
                        .map(|(_, r, _)| *r)
                        .fold((f32::MAX, f32::MIN), |(min, max), r| {
                            (min.min(r), max.max(r))
                        });
                    first_lidar =
                        Some((lidar.timestamp_us, lidar.data.len(), min_range, max_range));
                }
            }
            BagMessage::Odometry(_) => {
                odometry_count += 1;
            }
        }
    }

    let actual_duration = last_timestamp.saturating_sub(first_timestamp.unwrap_or(0));

    println!();
    println!("Message Statistics:");
    println!("  Total messages: {}", total_count);
    println!(
        "  Sensor status: {} ({:.1}%)",
        sensor_count,
        100.0 * sensor_count as f64 / total_count.max(1) as f64
    );
    println!(
        "  LiDAR scans: {} ({:.1}%)",
        lidar_count,
        100.0 * lidar_count as f64 / total_count.max(1) as f64
    );
    println!(
        "  Odometry: {} ({:.1}%)",
        odometry_count,
        100.0 * odometry_count as f64 / total_count.max(1) as f64
    );
    println!();

    let duration_s = actual_duration as f64 / 1_000_000.0;
    if duration_s > 0.0 {
        println!("Rates:");
        println!("  Overall: {:.1} Hz", total_count as f64 / duration_s);
        if sensor_count > 0 {
            println!(
                "  Sensor status: {:.1} Hz",
                sensor_count as f64 / duration_s
            );
        }
        if lidar_count > 0 {
            println!("  LiDAR: {:.1} Hz", lidar_count as f64 / duration_s);
        }
        if odometry_count > 0 {
            println!("  Odometry: {:.1} Hz", odometry_count as f64 / duration_s);
        }
    }

    // Verbose mode: show sample messages
    if verbose {
        if !first_sensors.is_empty() {
            println!();
            println!("First {} sensor messages:", first_sensors.len());
            for (ts, (l, r), gyro) in &first_sensors {
                println!(
                    "  [{:>12} us] encoders: ({:>5}, {:>5}), gyro: ({:>6}, {:>6}, {:>6})",
                    ts, l, r, gyro[0], gyro[1], gyro[2]
                );
            }
        }

        if let Some((ts, points, min_r, max_r)) = first_lidar {
            println!();
            println!("First LiDAR scan:");
            println!("  Timestamp: {} us", ts);
            println!("  Points: {}", points);
            println!("  Range: {:.3} - {:.3} m", min_r, max_r);
        }
    }

    // File size info
    let metadata = std::fs::metadata(bag_path).expect("Failed to get file metadata");
    let file_size = metadata.len();
    println!();
    println!("File Size:");
    println!(
        "  {} bytes ({:.2} KB, {:.2} MB)",
        file_size,
        file_size as f64 / 1024.0,
        file_size as f64 / 1_048_576.0
    );

    if message_count > 0 {
        println!(
            "  Average per message: {:.1} bytes",
            file_size as f64 / message_count as f64
        );
    }
}
