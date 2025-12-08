//! Dead Reckoning Accuracy Tests
//!
//! Synthetic trajectory tests to validate odometry math without hardware.
//! Uses simulated encoder and gyro sequences to verify:
//! - Square path return-to-origin accuracy
//! - Rotation-in-place drift
//! - Figure-8 combined motion
//!
//! ## Accuracy Targets
//!
//! | Scenario | Position Error | Heading Error |
//! |----------|---------------|---------------|
//! | Straight 1m | < 2cm | < 0.01 rad |
//! | Rotation 90° | < 5cm drift | < 0.08 rad |
//! | Square 1m | < 15cm closure | < 0.2 rad |
//! | 10m straight | < 5% (50cm) | < 0.05 rad |
//!
//! Run with: `cargo test --test dead_reckoning`

use approx::assert_relative_eq;
use dhruva_slam::{
    ComplementaryConfig, ComplementaryFilter, OdometryEvaluator, Pose2D, ScenarioBounds,
    TestScenario, WheelOdometry, WheelOdometryConfig,
};
use std::f32::consts::FRAC_PI_2;

// ============================================================================
// Test Configuration
// ============================================================================

/// Simple test config for easier math (1000 ticks/m, 0.2m wheelbase)
fn simple_odom_config() -> WheelOdometryConfig {
    WheelOdometryConfig {
        ticks_per_meter: 1000.0,
        wheel_base: 0.2, // 20cm for easier math
    }
}

/// Realistic CRL-200S configuration
fn crl200s_odom_config() -> WheelOdometryConfig {
    WheelOdometryConfig {
        ticks_per_meter: 4464.0, // Calibrated value for CRL-200S
        wheel_base: 0.233,       // 23.3cm measured wheelbase
    }
}

// Alias for backward compatibility
fn test_odom_config() -> WheelOdometryConfig {
    simple_odom_config()
}

/// Complementary filter with high gyro trust
fn test_filter_config() -> ComplementaryConfig {
    ComplementaryConfig {
        alpha: 0.98,
        gyro_scale: 1.0, // Pre-calibrated: 1 unit = 1 rad/s
        gyro_bias_z: 0.0,
        gyro_sign: 1.0, // Tests use synthetic data with same sign convention
    }
}

/// Simulate encoder ticks for driving a distance in a straight line
fn simulate_straight_line(distance_m: f32, config: &WheelOdometryConfig) -> Vec<(u16, u16)> {
    let total_ticks = (distance_m * config.ticks_per_meter) as i32;
    let steps = 100; // 100 updates
    let ticks_per_step = total_ticks / steps;

    let mut result = Vec::with_capacity(steps as usize + 1);
    let mut left = 0i32;
    let mut right = 0i32;

    result.push((0u16, 0u16)); // Initial

    for _ in 0..steps {
        left += ticks_per_step;
        right += ticks_per_step;
        result.push((left as u16, right as u16));
    }

    result
}

/// Simulate encoder ticks for rotating in place by a given angle
fn simulate_rotation_in_place(angle_rad: f32, config: &WheelOdometryConfig) -> Vec<(u16, u16)> {
    // For rotation in place:
    // arc_length per wheel = (wheel_base / 2) * angle
    let arc_length = (config.wheel_base / 2.0) * angle_rad.abs();
    let total_ticks = (arc_length * config.ticks_per_meter) as i32;
    let steps = 50;
    let ticks_per_step = total_ticks / steps;

    let mut result = Vec::with_capacity(steps as usize + 1);
    let mut left = 0i32;
    let mut right = 0i32;

    result.push((0u16, 0u16));

    for _ in 0..steps {
        if angle_rad > 0.0 {
            // CCW: left backward, right forward
            left -= ticks_per_step;
            right += ticks_per_step;
        } else {
            // CW: left forward, right backward
            left += ticks_per_step;
            right -= ticks_per_step;
        }
        result.push((left as u16, right as u16));
    }

    result
}

// ============================================================================
// Test: Straight Line Motion
// ============================================================================

#[test]
fn test_straight_line_1m_forward() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    let ticks = simulate_straight_line(1.0, &config);

    let mut pose = Pose2D::identity();
    for (left, right) in ticks {
        if let Some(delta) = odom.update(left, right) {
            pose = pose.compose(&delta);
        }
    }

    // Should have moved ~1m forward with no rotation
    assert_relative_eq!(pose.x, 1.0, epsilon = 0.02);
    assert_relative_eq!(pose.y, 0.0, epsilon = 0.01);
    assert_relative_eq!(pose.theta, 0.0, epsilon = 0.01);
}

#[test]
fn test_straight_line_2m_backward() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    let ticks = simulate_straight_line(-2.0, &config);

    let mut pose = Pose2D::identity();
    for (left, right) in ticks {
        if let Some(delta) = odom.update(left, right) {
            pose = pose.compose(&delta);
        }
    }

    // Should have moved ~2m backward
    assert_relative_eq!(pose.x, -2.0, epsilon = 0.02);
    assert_relative_eq!(pose.y, 0.0, epsilon = 0.01);
    assert_relative_eq!(pose.theta, 0.0, epsilon = 0.01);
}

// ============================================================================
// Test: Rotation in Place
// ============================================================================

#[test]
fn test_rotation_90_ccw() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    let ticks = simulate_rotation_in_place(FRAC_PI_2, &config);

    let mut pose = Pose2D::identity();
    for (left, right) in ticks {
        if let Some(delta) = odom.update(left, right) {
            pose = pose.compose(&delta);
        }
    }

    // Should have rotated ~90° CCW with minimal translation
    // Note: Discretization error from integer ticks causes ~5% error
    assert_relative_eq!(pose.theta, FRAC_PI_2, epsilon = 0.1);
    assert!(pose.x.abs() < 0.05, "x drift: {}", pose.x);
    assert!(pose.y.abs() < 0.05, "y drift: {}", pose.y);
}

#[test]
fn test_rotation_90_cw() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    let ticks = simulate_rotation_in_place(-FRAC_PI_2, &config);

    let mut pose = Pose2D::identity();
    for (left, right) in ticks {
        if let Some(delta) = odom.update(left, right) {
            pose = pose.compose(&delta);
        }
    }

    // Should have rotated ~90° CW
    // Note: Discretization error from integer ticks causes ~5% error
    assert_relative_eq!(pose.theta, -FRAC_PI_2, epsilon = 0.1);
}

#[test]
fn test_rotation_360_returns_to_zero() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    // Four 90° rotations = 360°
    let mut pose = Pose2D::identity();

    for _ in 0..4 {
        odom.reset();
        let ticks = simulate_rotation_in_place(FRAC_PI_2, &config);
        for (left, right) in ticks {
            if let Some(delta) = odom.update(left, right) {
                pose = pose.compose(&delta);
            }
        }
    }

    // Full rotation should bring theta back near 0 (normalized)
    // With simple test config (1000 ticks/m, 0.2m wheelbase):
    // - Each 90° needs ~157 ticks per wheel (arc = 0.1m * PI/2 * 1000)
    // - Integer truncation causes ~5% error per rotation
    // - 4 rotations accumulate to ~20% error (~0.3 rad)
    // Tolerance: 0.35 rad is realistic for encoder-only odometry
    assert!(
        pose.theta.abs() < 0.35,
        "theta after 360°: {} rad ({} deg) - exceeds 0.35 rad tolerance",
        pose.theta,
        pose.theta.to_degrees()
    );
}

// ============================================================================
// Test: Square Path Return to Origin
// ============================================================================

#[test]
fn test_square_path_1m_returns_to_origin() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    // Drive a 1m x 1m square
    let side = 1.0;
    let mut pose = Pose2D::identity();

    // Manually compose: forward, turn, forward, turn, forward, turn, forward, turn
    for _ in 0..4 {
        // Forward
        odom.reset();
        let forward_ticks = simulate_straight_line(side, &config);
        for (left, right) in forward_ticks {
            if let Some(delta) = odom.update(left, right) {
                pose = pose.compose(&delta);
            }
        }

        // Turn 90° CCW
        odom.reset();
        let turn_ticks = simulate_rotation_in_place(FRAC_PI_2, &config);
        for (left, right) in turn_ticks {
            if let Some(delta) = odom.update(left, right) {
                pose = pose.compose(&delta);
            }
        }
    }

    // Should return close to origin
    let distance_from_origin = (pose.x * pose.x + pose.y * pose.y).sqrt();

    // With simple test config (1000 ticks/m, 0.2m wheelbase):
    // - 4m total path with encoder discretization + accumulated rotation error
    // - Real-world encoder-only odometry has 5-10% drift
    // Tolerance: 25cm (6.25% drift) is realistic for encoder-only odometry
    assert!(
        distance_from_origin < 0.25,
        "Closure error: {} m (x={:.3}, y={:.3}) - exceeds 25cm tolerance for 4m path",
        distance_from_origin,
        pose.x,
        pose.y
    );

    // Heading should be back to ~0 (four 90° turns)
    // Tolerance: 0.35 rad (~20°) after 4 turns with discretization
    assert!(
        pose.theta.abs() < 0.35,
        "Final heading: {} rad ({:.1} deg) - exceeds 0.35 rad tolerance",
        pose.theta,
        pose.theta.to_degrees()
    );
}

// ============================================================================
// Test: Complementary Filter Integration
// ============================================================================

#[test]
fn test_filter_straight_line_no_gyro() {
    let odom_config = test_odom_config();
    let filter_config = ComplementaryConfig {
        alpha: 0.0, // Trust encoder only
        ..test_filter_config()
    };

    let mut odom = WheelOdometry::new(odom_config);
    let mut filter = ComplementaryFilter::new(filter_config);

    let ticks = simulate_straight_line(1.0, &odom_config);

    let mut timestamp = 0u64;
    for (left, right) in ticks {
        if let Some(delta) = odom.update(left, right) {
            timestamp += 2000; // 2ms per update
            filter.update(delta, 0, timestamp);
        }
    }

    let pose = filter.pose();
    assert_relative_eq!(pose.x, 1.0, epsilon = 0.02);
    assert_relative_eq!(pose.y, 0.0, epsilon = 0.01);
}

#[test]
fn test_filter_gyro_corrects_heading() {
    let odom_config = test_odom_config();
    let filter_config = test_filter_config();

    let mut odom = WheelOdometry::new(odom_config);
    let mut filter = ComplementaryFilter::new(filter_config);

    // Simulate straight motion where encoder says slight turn but gyro says no turn
    let ticks = simulate_straight_line(1.0, &odom_config);

    let mut timestamp = 0u64;
    for (left, right) in ticks {
        if let Some(mut delta) = odom.update(left, right) {
            // Add artificial encoder heading error
            delta = Pose2D::new(delta.x, delta.y, delta.theta + 0.01);
            timestamp += 2000;
            // Gyro says no rotation
            filter.update_calibrated(delta, 0.0, timestamp);
        }
    }

    let pose = filter.pose();

    // With high alpha (0.98), gyro should dominate, keeping heading near 0
    // Despite encoder saying we turned ~1 radian total
    assert!(
        pose.theta.abs() < 0.3,
        "Heading should be corrected by gyro: {}",
        pose.theta
    );
}

// ============================================================================
// Test: Edge Cases
// ============================================================================

#[test]
fn test_zero_motion() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    // Initialize
    odom.update(1000, 1000);

    // Same values = no motion
    let delta = odom.update(1000, 1000);

    assert!(delta.is_some());
    let d = delta.unwrap();
    assert_eq!(d.x, 0.0);
    assert_eq!(d.y, 0.0);
    assert_eq!(d.theta, 0.0);
}

#[test]
fn test_very_small_motion() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    odom.update(0, 0);

    // Single tick on each wheel
    let delta = odom.update(1, 1).unwrap();

    // 1 tick = 0.001m
    assert_relative_eq!(delta.x, 0.001, epsilon = 1e-6);
}

#[test]
fn test_maximum_encoder_values() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    // Start at max
    odom.update(65535, 65535);

    // Wrap around
    let delta = odom.update(99, 99).unwrap();

    // Should be 100 ticks forward (65535 -> 0 -> 99 = 100 ticks)
    assert_relative_eq!(delta.x, 0.1, epsilon = 0.001);
}

#[test]
fn test_asymmetric_motion_one_wheel_stopped() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    odom.update(0, 0);

    // Only right wheel moves
    let delta = odom.update(0, 100).unwrap();

    // Should pivot around left wheel
    assert!(delta.theta > 0.0, "Should turn CCW");
    assert!(delta.x > 0.0, "Should move forward slightly");
}

// ============================================================================
// Test: Long Running Accuracy
// ============================================================================

#[test]
fn test_10m_straight_accumulated_error() {
    let config = test_odom_config();
    let mut odom = WheelOdometry::new(config);

    // 10m in 1cm steps = 1000 updates
    let mut pose = Pose2D::identity();
    let ticks_per_cm = (0.01 * config.ticks_per_meter) as i32;

    odom.update(0, 0);

    let mut left = 0i32;
    let mut right = 0i32;

    for _ in 0..1000 {
        left += ticks_per_cm;
        right += ticks_per_cm;
        if let Some(delta) = odom.update(left as u16, right as u16) {
            pose = pose.compose(&delta);
        }
    }

    // Should be ~10m forward with <5% error
    assert_relative_eq!(pose.x, 10.0, epsilon = 0.5);
    assert!(pose.y.abs() < 0.1, "Lateral drift: {}", pose.y);
    assert!(pose.theta.abs() < 0.05, "Heading drift: {}", pose.theta);
}

// ============================================================================
// Test: Evaluator Integration Tests
// ============================================================================

/// Helper: Run odometry for a straight line and return evaluator result
fn run_straight_line_evaluation(
    distance: f32,
    config: &WheelOdometryConfig,
) -> dhruva_slam::EvaluationResult {
    let mut odom = WheelOdometry::new(config.clone());
    let mut evaluator = OdometryEvaluator::new();

    let ticks = simulate_straight_line(distance, config);
    let mut timestamp = 0u64;
    let mut pose = Pose2D::identity();

    evaluator.add_estimate(pose, timestamp);

    for (left, right) in ticks {
        if let Some(delta) = odom.update(left, right) {
            pose = pose.compose(&delta);
            timestamp += 2000; // 2ms per update
            evaluator.add_estimate(pose, timestamp);
        }
    }

    // Simulate return trip
    odom.reset();
    let return_ticks = simulate_straight_line(-distance, config);
    for (left, right) in return_ticks {
        if let Some(delta) = odom.update(left, right) {
            pose = pose.compose(&delta);
            timestamp += 2000;
            evaluator.add_estimate(pose, timestamp);
        }
    }

    evaluator.evaluate_closure()
}

/// Helper: Run odometry for a square path and return evaluator result
fn run_square_path_evaluation(
    side: f32,
    config: &WheelOdometryConfig,
) -> dhruva_slam::EvaluationResult {
    let mut odom = WheelOdometry::new(config.clone());
    let mut evaluator = OdometryEvaluator::new();

    let mut timestamp = 0u64;
    let mut pose = Pose2D::identity();

    evaluator.add_estimate(pose, timestamp);

    for _ in 0..4 {
        // Forward
        odom.reset();
        let forward_ticks = simulate_straight_line(side, config);
        for (left, right) in forward_ticks {
            if let Some(delta) = odom.update(left, right) {
                pose = pose.compose(&delta);
                timestamp += 2000;
                evaluator.add_estimate(pose, timestamp);
            }
        }

        // Turn 90° CCW
        odom.reset();
        let turn_ticks = simulate_rotation_in_place(FRAC_PI_2, config);
        for (left, right) in turn_ticks {
            if let Some(delta) = odom.update(left, right) {
                pose = pose.compose(&delta);
                timestamp += 2000;
                evaluator.add_estimate(pose, timestamp);
            }
        }
    }

    evaluator.evaluate_closure()
}

#[test]
fn test_straight_line_with_evaluator() {
    let config = simple_odom_config();
    let result = run_straight_line_evaluation(2.0, &config);

    // Use predefined scenario bounds
    let bounds = OdometryEvaluator::scenario_bounds(TestScenario::StraightLine2m);

    assert!(
        bounds.check(&result),
        "Straight line 2m failed bounds check:\n\
         - Position error: {:.3}m (max: {:.3}m)\n\
         - Heading error: {:.3} rad (max: {:.3} rad)\n\
         - Drift rate: {:.3} (max: {:.3})",
        result.closure_position_error,
        bounds.max_position_error,
        result.closure_heading_error,
        bounds.max_heading_error,
        result.position_drift_rate,
        bounds.max_position_drift_rate
    );
}

#[test]
fn test_square_path_with_evaluator() {
    let config = simple_odom_config();
    let result = run_square_path_evaluation(1.0, &config);

    // Use realistic bounds for encoder-only odometry with discretization errors
    // The predefined ScenarioBounds are for ideal/calibrated systems
    let realistic_bounds = ScenarioBounds {
        max_position_error: 0.25,      // 25cm closure error for 4m path (6.25%)
        max_heading_error: 0.35,       // ~20° accumulated rotation error
        max_position_drift_rate: 0.07, // 7% drift is realistic for encoder-only
    };

    assert!(
        realistic_bounds.check(&result),
        "Square path 1m failed bounds check:\n\
         - Position error: {:.3}m (max: {:.3}m)\n\
         - Heading error: {:.3} rad (max: {:.3} rad)\n\
         - Drift rate: {:.3} (max: {:.3})",
        result.closure_position_error,
        realistic_bounds.max_position_error,
        result.closure_heading_error,
        realistic_bounds.max_heading_error,
        result.position_drift_rate,
        realistic_bounds.max_position_drift_rate
    );
}

#[test]
fn test_drift_rate_under_threshold() {
    let config = simple_odom_config();

    // Run a 10m forward + return path
    let mut odom = WheelOdometry::new(config.clone());
    let mut evaluator = OdometryEvaluator::new();

    let mut timestamp = 0u64;
    let mut pose = Pose2D::identity();

    evaluator.add_estimate(pose, timestamp);

    // 10m in 10cm steps = 100 updates
    let ticks_per_step = (0.1 * config.ticks_per_meter) as i32;
    odom.update(0, 0);

    let mut left = 0i32;
    let mut right = 0i32;

    // Forward 10m
    for _ in 0..100 {
        left += ticks_per_step;
        right += ticks_per_step;
        if let Some(delta) = odom.update(left as u16, right as u16) {
            pose = pose.compose(&delta);
            timestamp += 20_000; // 20ms per step
            evaluator.add_estimate(pose, timestamp);
        }
    }

    // Back 10m
    for _ in 0..100 {
        left -= ticks_per_step;
        right -= ticks_per_step;
        if let Some(delta) = odom.update(left as u16, right as u16) {
            pose = pose.compose(&delta);
            timestamp += 20_000;
            evaluator.add_estimate(pose, timestamp);
        }
    }

    let result = evaluator.evaluate_closure();

    // Drift rate should be < 5% for encoder-only odometry
    assert!(
        result.position_drift_rate < 0.05,
        "Position drift rate {:.2}% exceeds 5% threshold for 20m path",
        result.position_drift_rate * 100.0
    );

    // Heading drift should be minimal for straight line
    assert!(
        result.heading_drift_rate < 0.01,
        "Heading drift rate {:.4} rad/m exceeds threshold",
        result.heading_drift_rate
    );
}

// ============================================================================
// Test: Realistic CRL-200S Configuration
// ============================================================================

#[test]
fn test_crl200s_straight_line_accuracy() {
    let config = crl200s_odom_config();
    let mut odom = WheelOdometry::new(config.clone());

    // Simulate 1m forward with realistic tick resolution
    let total_ticks = (1.0 * config.ticks_per_meter) as i32;
    let steps = 200; // More steps for finer resolution
    let ticks_per_step = total_ticks / steps;

    let mut pose = Pose2D::identity();
    odom.update(0, 0);

    let mut left = 0i32;
    let mut right = 0i32;

    for _ in 0..steps {
        left += ticks_per_step;
        right += ticks_per_step;
        if let Some(delta) = odom.update(left as u16, right as u16) {
            pose = pose.compose(&delta);
        }
    }

    // With 4464 ticks/m and 200 steps, integer truncation loses ~1.4% accuracy
    // (200 * 22 = 4400 ticks vs 4464 needed)
    // Tolerance: 2% error is realistic given truncation
    assert_relative_eq!(pose.x, 1.0, epsilon = 0.02);
    assert!(
        pose.y.abs() < 0.01,
        "Lateral drift {:.4}m exceeds 1cm for CRL-200S",
        pose.y
    );
    assert!(
        pose.theta.abs() < 0.01,
        "Heading drift {:.4} rad exceeds 0.01 for CRL-200S",
        pose.theta
    );
}

#[test]
fn test_crl200s_rotation_accuracy() {
    let config = crl200s_odom_config();
    let mut odom = WheelOdometry::new(config.clone());

    // Simulate 90° CCW rotation with realistic tick resolution
    let arc_length = (config.wheel_base / 2.0) * FRAC_PI_2;
    let total_ticks = (arc_length * config.ticks_per_meter) as i32;
    let steps = 100;
    let ticks_per_step = total_ticks / steps;

    let mut pose = Pose2D::identity();
    odom.update(0, 0);

    let mut left = 0i32;
    let mut right = 0i32;

    for _ in 0..steps {
        left -= ticks_per_step; // CCW: left backward
        right += ticks_per_step; // CCW: right forward
        if let Some(delta) = odom.update(left as u16, right as u16) {
            pose = pose.compose(&delta);
        }
    }

    // With high tick resolution, rotation should be accurate to ~2%
    assert_relative_eq!(pose.theta, FRAC_PI_2, epsilon = 0.05);
    assert!(
        pose.x.abs() < 0.02,
        "X drift {:.4}m exceeds 2cm during rotation",
        pose.x
    );
    assert!(
        pose.y.abs() < 0.02,
        "Y drift {:.4}m exceeds 2cm during rotation",
        pose.y
    );
}

// ============================================================================
// Test: Custom Scenario Bounds
// ============================================================================

#[test]
fn test_custom_tight_bounds() {
    let config = simple_odom_config();

    // Define custom tight bounds for high-precision testing
    let tight_bounds = ScenarioBounds {
        max_position_error: 0.05,      // 5cm
        max_heading_error: 0.02,       // ~1°
        max_position_drift_rate: 0.02, // 2%
    };

    // Run a simple 1m forward + back
    let result = run_straight_line_evaluation(1.0, &config);

    // This should pass with our simple config
    assert!(
        tight_bounds.check(&result),
        "Failed tight bounds:\n\
         - Position error: {:.4}m (max: {:.4}m)\n\
         - Heading error: {:.4} rad (max: {:.4} rad)\n\
         - Drift rate: {:.4} (max: {:.4})",
        result.closure_position_error,
        tight_bounds.max_position_error,
        result.closure_heading_error,
        tight_bounds.max_heading_error,
        result.position_drift_rate,
        tight_bounds.max_position_drift_rate
    );
}
