//! YAML-based scenario tests
//!
//! Runs all test scenarios defined in YAML files in the scenarios directory.
//!
//! # Usage
//!
//! ```bash
//! # Run all scenarios
//! cargo test --features integration-tests -- yaml_tests --nocapture
//!
//! # Run a single scenario by name
//! cargo test --features integration-tests -- test_static_scan --nocapture
//! cargo test --features integration-tests -- test_straight_line --nocapture
//! cargo test --features integration-tests -- test_loop_closure --nocapture
//! ```
//!
//! # Adding New Scenarios
//!
//! Create a new `.yaml` file in `tests/integration/scenarios/` with:
//!
//! ```yaml
//! name: "my_scenario"
//! map: "maps/medium_room.yaml"
//! start_pose:
//!   x: 4.0
//!   y: 4.0
//!   theta: 0.0
//! path:
//!   - {left: 0.5, right: 0.5}   # forward 0.5m
//!   - {left: -0.183, right: 0.183}  # turn left 90°
//! ```

use crate::yaml_runner;

/// Run a single scenario by filename (without .yaml extension)
fn run_single_scenario(name: &str) {
    env_logger::try_init().ok();

    let scenario_path = format!(
        "{}/tests/integration/scenarios/{}.yaml",
        env!("CARGO_MANIFEST_DIR"),
        name
    );

    println!("\n--- Running scenario: {} ---", name);

    match yaml_runner::run_scenario(&scenario_path) {
        Ok(result) => {
            println!("{}", result.metrics.summary());
            println!(
                "Ground truth: ({:.2}, {:.2}, {:.1}°)",
                result.ground_truth_pose.x,
                result.ground_truth_pose.y,
                result.ground_truth_pose.theta.to_degrees()
            );
            println!(
                "SLAM pose:    ({:.2}, {:.2}, {:.1}°)",
                result.final_pose.x,
                result.final_pose.y,
                result.final_pose.theta.to_degrees()
            );
            println!("Observations: {}", result.observations);
            println!("Sim time:     {:.2}s", result.sim_time);
        }
        Err(e) => {
            panic!("Scenario {} failed: {}", name, e);
        }
    }
}

// Individual scenario tests - run with: cargo test --features integration-tests -- test_<name>

#[test]
fn test_static_scan() {
    run_single_scenario("static_scan_test");
}

#[test]
fn test_straight_line() {
    run_single_scenario("straight_line_test");
}

#[test]
fn test_pure_rotation() {
    run_single_scenario("pure_rotation_test");
}

#[test]
fn test_loop_closure() {
    run_single_scenario("loop_closure_test");
}

#[test]
fn test_medium_room_perimeter() {
    run_single_scenario("medium_room_perimeter");
}

#[test]
fn test_medium_room_exploration() {
    run_single_scenario("medium_room_exploration");
}

#[test]
fn test_medium_room_obstacles() {
    run_single_scenario("medium_room_obstacles");
}

#[test]
fn test_simple_room_straight_line() {
    run_single_scenario("simple_room_straight_line");
}

// New stress-test scenarios for ICP convergence and drift validation

#[test]
fn test_icp_sparse_corridor() {
    run_single_scenario("icp_sparse_corridor");
}

#[test]
fn test_drift_long_path() {
    run_single_scenario("drift_long_path");
}

#[test]
fn test_drift_continuous_curve() {
    run_single_scenario("drift_continuous_curve");
}

#[test]
fn test_yaml_scenarios() {
    env_logger::try_init().ok();

    let scenarios_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/integration/scenarios");

    println!("\n========================================");
    println!("  YAML Integration Test Scenarios");
    println!("========================================");

    let results = yaml_runner::run_all_scenarios(scenarios_dir);

    if results.is_empty() {
        println!("\nNo YAML scenarios found in: {}", scenarios_dir);
        println!("Create .yaml files in tests/integration/scenarios/");
        return;
    }

    println!("\n========================================");
    println!("  Results Summary");
    println!("========================================\n");

    let mut passed = 0;
    let mut failed = 0;
    let mut failures = Vec::new();

    for (name, result) in &results {
        match result {
            Ok(r) => {
                passed += 1;
                println!("PASS: {}", name);
                println!("      Position error: {:.3}m", r.metrics.position_error);
                println!(
                    "      Wall accuracy:  {:.0}%",
                    r.metrics.wall_accuracy * 100.0
                );
                println!("      Lines detected: {}", r.metrics.detected_walls);
                println!(
                    "      Final pose:     ({:.2}, {:.2}, {:.1}°)",
                    r.final_pose.x,
                    r.final_pose.y,
                    r.final_pose.theta.to_degrees()
                );
                println!();
            }
            Err(e) => {
                failed += 1;
                println!("FAIL: {}", name);
                println!("      Error: {}\n", e);
                failures.push(name.clone());
            }
        }
    }

    println!("========================================");
    println!("  Total: {} passed, {} failed", passed, failed);
    println!("========================================\n");

    assert!(failures.is_empty(), "Failed scenarios: {:?}", failures);
}
