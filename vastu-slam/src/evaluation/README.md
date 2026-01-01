# Evaluation Module

Cartographer-style evaluation framework for VastuSLAM accuracy, performance, and quality metrics.

## Overview

The evaluation framework is built on three pillars:

1. **Accuracy** - Pose relations metrics, ATE (Absolute Trajectory Error), RPE (Relative Pose Error)
2. **Performance** - Latency benchmarks, resource usage tracking
3. **Quality** - Map consistency, loop closure precision/recall

## Cartographer-Style Evaluation

The core idea is to evaluate using **relative poses** between trajectory nodes, which works without external ground truth (GPS, motion capture).

### Two-Step Process

1. **Generate ground truth**: Extract pose relations from an optimized trajectory (with loop closures applied)
2. **Evaluate**: Compare a test trajectory against these ground truth relations

```rust
use vastu_slam::evaluation::{GroundTruthRelations, RelationsMetrics, GroundTruthConfig};

// Step 1: Generate ground truth from optimized trajectory
let config = GroundTruthConfig::default();
let ground_truth = GroundTruthRelations::generate(
    &optimized_poses,
    &loop_closures,
    config,
);

// Step 2: Evaluate test trajectory
let metrics = RelationsMetrics::compute(&test_poses, &ground_truth);
metrics.print();
// Output:
// Abs translational error 0.00832 +/- 0.00421 m
// Abs rotational error 0.15234 +/- 0.08123 deg
```

## Module Structure

```
evaluation/
├── mod.rs              # Module exports
├── ground_truth.rs     # GroundTruthRelations, PoseRelation
├── relations_metrics.rs # RelationsMetrics (Cartographer format)
├── accuracy.rs         # ATE, RPE (evo-compatible metrics)
├── performance.rs      # PerformanceTracker, timing utilities
└── quality.rs          # Map quality, loop closure precision/recall
```

## Key Types

### Ground Truth Generation

| Type | Purpose |
|------|---------|
| `GroundTruthConfig` | Configuration for relation extraction (min distance, outlier thresholds) |
| `PoseRelation` | Relative transform between two trajectory nodes |
| `GroundTruthRelations` | Collection of relations from an optimized trajectory |

### Accuracy Metrics

| Type | Purpose |
|------|---------|
| `RelationsMetrics` | Cartographer-style pose relations evaluation |
| `AbsoluteTrajectoryError` | ATE after SE(2) alignment (evo `evo_ape` compatible) |
| `RelativePoseError` | RPE over fixed distances (evo `evo_rpe` compatible) |
| `AccuracyMetrics` | Container for ATE + RPE |
| `TrajectoryError` | Statistics (RMSE, mean, std, min, max, median) |

### Performance Metrics

| Type | Purpose |
|------|---------|
| `PerformanceTracker` | Record operation timings during SLAM |
| `PerformanceMetrics` | Aggregated timing statistics |
| `OperationTiming` | Per-operation statistics (mean, std, min, max) |
| `TimingGuard` | RAII guard for automatic timing |

### Quality Metrics

| Type | Purpose |
|------|---------|
| `LoopClosurePR` | Precision/Recall for loop closure detection |
| `LoopClosureGroundTruth` | Ground truth loop closure pairs |
| `MapQualityMetrics` | Coverage, wall sharpness, cell distribution |

## Usage Examples

### Pose Relations Evaluation (Cartographer-style)

```rust
use vastu_slam::evaluation::{GroundTruthRelations, RelationsMetrics, GroundTruthConfig};

// Generate ground truth from optimized trajectory
let config = GroundTruthConfig {
    min_covered_distance: 0.5,  // 50cm minimum between nodes
    outlier_threshold_meters: 0.05,  // 5cm outlier threshold
    outlier_threshold_radians: 0.035, // ~2 degrees
    node_sampling_interval: 1,
};

let ground_truth = GroundTruthRelations::generate(
    &optimized_trajectory,
    &loop_closure_pairs,
    config,
);

// Evaluate test trajectory
let metrics = RelationsMetrics::compute_with_threshold(
    &test_trajectory,
    &ground_truth,
    0.01, // 1cm pass/fail threshold
);

println!("{}", metrics.summary());
// Output: trans: 0.0083±0.0042m, rot: 0.15±0.08°, 45 relations, PASS
```

### ATE/RPE Evaluation (evo-compatible)

```rust
use vastu_slam::evaluation::AccuracyMetrics;

// Compute both ATE and RPE
let metrics = AccuracyMetrics::compute(
    &estimated_trajectory,
    &ground_truth_trajectory,
    1.0, // 1m delta for RPE
);

metrics.print();
// === Absolute Trajectory Error (ATE) ===
// Alignment: SE(2)
// Translation RMSE: 0.012345 m (mean: 0.010234, std: 0.006789)
// Rotation RMSE:    0.034567 rad (mean: 0.028901, std: 0.019012)
//
// === Relative Pose Error (RPE) ===
// Delta: 1 m, Pairs: 42
// Translation RMSE: 0.008765 m (mean: 0.007654, std: 0.004321)
// Rotation RMSE:    0.023456 rad (mean: 0.019876, std: 0.012345)
```

### Performance Tracking

```rust
use vastu_slam::evaluation::PerformanceTracker;
use std::time::Duration;

let mut tracker = PerformanceTracker::with_scan_interval(Duration::from_millis(200));
tracker.start();

// Manual timing
let start = std::time::Instant::now();
// ... perform scan matching ...
tracker.record("scan_matching", start.elapsed());

// RAII timing guard
{
    let _guard = tracker.time("map_update");
    // ... update occupancy grid ...
} // Duration recorded automatically on drop

tracker.record_scan();

// Get metrics
let metrics = tracker.metrics();
metrics.print();
// === Performance Metrics ===
// Scans processed: 100
// Scans per second: 12.50
// Real-time factor: 2.50x
// Total time: 8000.00 ms
//
// Operation timings:
//   scan_matching: 5.23 ± 1.12 ms (min: 3.45, max: 8.90, n=100)
//   map_update: 2.34 ± 0.56 ms (min: 1.23, max: 4.56, n=100)
```

### Loop Closure Quality

```rust
use vastu_slam::evaluation::{LoopClosurePR, LoopClosureGroundTruth};

// Create ground truth from trajectory
let ground_truth = LoopClosureGroundTruth::from_trajectory(
    &trajectory,
    50,   // Min 50 nodes separation
    0.5,  // Max 0.5m distance for loop closure
);

// Evaluate detections (from_node, to_node, score)
let detections = vec![
    (0, 100, 0.95),
    (25, 125, 0.87),
    (50, 150, 0.82),
];

let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);
pr.print();
// === Loop Closure Quality ===
// True Positives:  3
// False Positives: 0
// False Negatives: 1
// Precision:       100.00%
// Recall:          75.00%
// F1 Score:        85.71%
// Max Recall @100% Precision: 75.00%
```

### Map Quality Metrics

```rust
use vastu_slam::evaluation::MapQualityMetrics;

let quality = MapQualityMetrics::compute(&grid);
quality.print();
// === Map Quality Metrics ===
// Coverage:        45.23 m²
// Known cells:     78.5%
// Wall sharpness:  0.87
// Avg wall thickness: 2.34 cells
//
// Cell distribution:
//   Floor:   125000
//   Wall:    8500
//   Cliff:   120
//   Bump:    45
//   Unknown: 34335
```

## Evaluation Strategy

### When External Ground Truth is Unavailable

1. Run full SLAM with loop closure optimization
2. Extract ground truth relations from the optimized trajectory
3. Re-run SLAM (or use odometry-only) and compare against relations

This tests the **self-consistency** of the SLAM system.

### When External Ground Truth is Available

Use ATE/RPE for absolute accuracy comparison:

- **ATE**: Best for evaluating global accuracy after alignment
- **RPE**: Best for evaluating local drift/odometry quality

## Threshold Checking

```rust
use std::collections::HashMap;

let mut thresholds = HashMap::new();
thresholds.insert("scan_matching".to_string(), 15.0); // 15ms max
thresholds.insert("map_update".to_string(), 5.0);     // 5ms max

let violations = metrics.check_thresholds(&thresholds);
if !violations.is_empty() {
    for v in violations {
        eprintln!("Performance regression: {}", v);
    }
}
```

## Key Design Decisions

1. **Cartographer-style relations**: Works without external ground truth
2. **evo compatibility**: ATE/RPE metrics match evo tool output format
3. **RTAB-Map style loop closure**: Precision/recall at 100% precision threshold
4. **RAII timing guards**: Automatic duration recording for cleaner code
5. **Serde support**: All metrics can be serialized for reporting
