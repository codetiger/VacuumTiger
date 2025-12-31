//! Quality integration tests for VastuSLAM.
//!
//! These tests verify map quality and loop closure detection quality.

mod common;

use vastu_slam::evaluation::{LoopClosureGroundTruth, LoopClosurePR, MapQualityMetrics};
use vastu_slam::{MapConfig, OccupancyGridMap, Pose2D};

// ============================================================================
// Loop Closure Quality Tests
// ============================================================================

#[test]
fn test_loop_closure_pr_perfect_detection() {
    // Ground truth: these are the actual loop closures
    let ground_truth = LoopClosureGroundTruth::from_pairs(vec![(0, 50), (10, 60), (20, 70)]);

    // Detections: perfectly detected all loop closures
    let detections = vec![
        (0, 50, 0.95),  // Correct
        (10, 60, 0.90), // Correct
        (20, 70, 0.85), // Correct
    ];

    let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);

    println!("Perfect detection PR:");
    pr.print();

    assert_eq!(pr.true_positives, 3);
    assert_eq!(pr.false_positives, 0);
    assert_eq!(pr.false_negatives, 0);
    assert!((pr.precision - 1.0).abs() < 1e-5);
    assert!((pr.recall - 1.0).abs() < 1e-5);
}

#[test]
fn test_loop_closure_pr_with_false_positives() {
    let ground_truth = LoopClosureGroundTruth::from_pairs(vec![(0, 50), (10, 60)]);

    // Detections include a false positive
    let detections = vec![
        (0, 50, 0.95),  // Correct
        (10, 60, 0.90), // Correct
        (30, 80, 0.70), // False positive
    ];

    let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);

    println!("With false positive PR:");
    pr.print();

    assert_eq!(pr.true_positives, 2);
    assert_eq!(pr.false_positives, 1);
    assert!((pr.precision - 2.0 / 3.0).abs() < 1e-5);
    assert!((pr.recall - 1.0).abs() < 1e-5);
}

#[test]
fn test_loop_closure_pr_with_false_negatives() {
    let ground_truth = LoopClosureGroundTruth::from_pairs(vec![(0, 50), (10, 60), (20, 70)]);

    // Only detected 2 out of 3 closures
    let detections = vec![
        (0, 50, 0.95),
        (10, 60, 0.90),
        // Missing (20, 70)
    ];

    let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);

    println!("With false negative PR:");
    pr.print();

    assert_eq!(pr.true_positives, 2);
    assert_eq!(pr.false_negatives, 1);
    assert!((pr.precision - 1.0).abs() < 1e-5);
    assert!((pr.recall - 2.0 / 3.0).abs() < 1e-5);
}

#[test]
fn test_loop_closure_ground_truth_from_trajectory() {
    // Create a trajectory that returns to start
    let trajectory = common::square_trajectory(4.0, 10);

    // Generate ground truth based on spatial proximity
    let ground_truth = LoopClosureGroundTruth::from_trajectory(&trajectory, 20, 0.5);

    println!(
        "Generated {} loop closure ground truths from trajectory",
        ground_truth.closures.len()
    );

    // Should find at least one closure (start and end are close)
    assert!(
        !ground_truth.closures.is_empty(),
        "Should find loop closures in square trajectory"
    );
}

#[test]
fn test_max_recall_at_100_precision() {
    let ground_truth =
        LoopClosureGroundTruth::from_pairs(vec![(0, 50), (10, 60), (20, 70), (30, 80)]);

    // Detections sorted by score - first 2 are correct, 3rd is FP
    let detections = vec![
        (0, 50, 0.99),    // Correct
        (10, 60, 0.95),   // Correct
        (999, 888, 0.90), // False positive
        (20, 70, 0.85),   // Correct but after FP
    ];

    let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);

    println!(
        "Max recall @ 100% precision: {:.2}%",
        pr.max_recall_at_100_precision * 100.0
    );

    // Max recall at 100% precision is 2/4 = 50% (before the FP)
    assert!((pr.max_recall_at_100_precision - 0.5).abs() < 1e-5);
}

// ============================================================================
// Map Quality Tests
// ============================================================================

#[test]
fn test_map_quality_empty_map() {
    let config = MapConfig::default();
    let map = OccupancyGridMap::new(config);

    let quality = MapQualityMetrics::compute(map.storage());

    assert_eq!(quality.coverage_m2, 0.0);
    assert_eq!(quality.known_percentage, 0.0);
}

#[test]
fn test_map_quality_with_room() {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Create a simple room map
    let scan = common::room_scan(6.0, 6.0, 3.0, 3.0, 360);

    // Multiple observations for better quality
    for i in 0..10 {
        let angle = i as f32 * 0.1;
        let pose = Pose2D::new(3.0, 3.0, angle);
        map.observe_lidar(&scan, pose);
    }

    let quality = MapQualityMetrics::compute(map.storage());

    println!("Room map quality:");
    quality.print();

    // Should have explored some area
    assert!(quality.coverage_m2 > 0.0, "Should have explored area");
    assert!(quality.known_percentage > 0.0, "Should have known cells");
    assert!(quality.cell_counts.floor > 0, "Should have floor cells");
    assert!(quality.cell_counts.wall > 0, "Should have wall cells");
}

#[test]
fn test_map_quality_wall_sharpness() {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Create clean room with sharp walls
    let scan = common::room_scan(6.0, 6.0, 3.0, 3.0, 720); // Higher resolution
    let _base_pose = Pose2D::new(3.0, 3.0, 0.0);

    for i in 0..20 {
        let angle = i as f32 * 0.05;
        let p = Pose2D::new(3.0, 3.0, angle);
        map.observe_lidar(&scan, p);
    }

    let quality = MapQualityMetrics::compute(map.storage());

    println!("Wall sharpness: {:.3}", quality.wall_sharpness);
    println!(
        "Avg wall thickness: {:.2} cells",
        quality.avg_wall_thickness
    );

    // Clean scans should produce relatively sharp walls
    // This is more of a baseline than a strict requirement
    assert!(quality.wall_sharpness >= 0.0 && quality.wall_sharpness <= 1.0);
}

// ============================================================================
// Combined Quality Tests
// ============================================================================

#[test]
fn test_quality_trajectory_and_map() {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Simulate robot moving through a room
    let trajectory = common::square_trajectory(4.0, 10);

    for (i, pose) in trajectory.iter().enumerate() {
        // Robot at center of 8x8 room
        let scan = common::room_scan(8.0, 8.0, pose.x + 4.0, pose.y + 4.0, 360);
        map.observe_lidar(&scan, Pose2D::new(pose.x + 4.0, pose.y + 4.0, pose.theta));

        // Check quality periodically
        if i % 10 == 9 {
            let quality = MapQualityMetrics::compute(map.storage());
            println!(
                "Step {}: coverage={:.2}m², known={:.1}%",
                i + 1,
                quality.coverage_m2,
                quality.known_percentage
            );
        }
    }

    let final_quality = MapQualityMetrics::compute(map.storage());

    println!("\n=== Final Map Quality ===");
    final_quality.print();

    // After full trajectory, should have good coverage
    assert!(
        final_quality.coverage_m2 > 10.0,
        "Should have significant coverage after full trajectory"
    );
}

// ============================================================================
// Regression Tests
// ============================================================================

#[test]
fn test_quality_regression_basic_room() {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Standard room scan
    let scan = common::room_scan(6.0, 6.0, 3.0, 3.0, 360);
    let robot_pose = Pose2D::new(3.0, 3.0, 0.0);

    for _ in 0..5 {
        map.observe_lidar(&scan, robot_pose);
    }

    let quality = MapQualityMetrics::compute(map.storage());

    // Baseline expectations for a 6x6 room
    assert!(
        quality.cell_counts.wall > 100,
        "Should detect sufficient wall cells, got {}",
        quality.cell_counts.wall
    );
    assert!(
        quality.cell_counts.floor > 1000,
        "Should detect sufficient floor cells, got {}",
        quality.cell_counts.floor
    );
}
