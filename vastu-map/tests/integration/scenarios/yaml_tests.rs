//! YAML-based scenario tests
//!
//! Runs all test scenarios defined in YAML files in the scenarios directory.
//!
//! # Usage
//!
//! ```bash
//! cargo test --features integration-tests -- --nocapture
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
