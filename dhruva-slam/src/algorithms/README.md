# Algorithms Module

Core SLAM algorithm implementations: scan matching, mapping, localization, and descriptors.

## Overview

The algorithms module provides:
- **Matching** - Align laser scans to estimate relative transforms
- **Mapping** - Build and update occupancy grid maps
- **Localization** - Motion and sensor models for particle filter
- **Descriptors** - Place recognition for loop closure

## Module Structure

```
algorithms/
├── mod.rs              # Module exports
├── matching/           # Scan matching algorithms
│   ├── mod.rs          # Exports, ScanMatcher trait, ScanMatchResult
│   ├── icp.rs          # Point-to-point ICP
│   ├── point_to_line_icp/ # Point-to-line ICP (modular)
│   │   ├── mod.rs
│   │   ├── config.rs
│   │   ├── line2d.rs
│   │   ├── correspondence.rs
│   │   └── gauss_newton.rs
│   ├── correlative.rs  # Exhaustive correlative search
│   ├── multi_resolution.rs # Multi-resolution correlative
│   ├── hybrid.rs       # Combined correlative + ICP
│   ├── dynamic.rs      # Runtime algorithm selection
│   ├── robust_kernels.rs # Robust loss functions
│   ├── icp_common.rs   # Shared ICP utilities
│   ├── validation.rs   # Match result validation
│   └── test_utils.rs   # Test helpers
├── mapping/            # Occupancy grid mapping
│   ├── mod.rs          # Exports, MapRegion
│   ├── occupancy_grid/ # Grid map implementation
│   │   ├── mod.rs
│   │   ├── config.rs
│   │   ├── export.rs
│   │   └── serialization.rs
│   ├── ray_tracer.rs   # Bresenham ray tracing
│   ├── integrator.rs   # Scan-to-map integration
│   └── features.rs     # Line/corner extraction
├── localization/       # Particle filter components
│   ├── mod.rs          # Exports
│   ├── motion_model.rs # Odometry-based motion model
│   └── sensor_model.rs # Likelihood field sensor model
└── descriptors/        # Place recognition
    ├── mod.rs          # Exports
    └── lidar_iris.rs   # LiDAR-IRIS binary descriptor
```

## Matching

Scan alignment algorithms that find the transform between two point clouds.

### ScanMatcher Trait

All matchers implement this common interface:

```rust
pub trait ScanMatcher {
    fn match_scans(
        &mut self,
        source: &PointCloud2D,  // Scan to align (current)
        target: &PointCloud2D,  // Reference (previous/map)
        initial_guess: &Pose2D, // Starting estimate (e.g., odometry)
    ) -> ScanMatchResult;
}
```

### ScanMatchResult

```rust
pub struct ScanMatchResult {
    pub transform: Pose2D,       // Estimated transform
    pub covariance: Covariance2D, // Uncertainty
    pub score: f32,              // Match quality (0-1)
    pub converged: bool,         // Did algorithm converge?
    pub iterations: u32,         // Iterations performed
    pub mse: f32,                // Mean squared error
}
```

### Transform Semantics (IMPORTANT!)

The returned `transform` is the **robot motion** from target time to source time.

For sequential SLAM where:
- `source` = current scan (robot at time t+1)
- `target` = previous scan (robot at time t)

Use the transform directly (no inversion):

```rust
let result = matcher.match_scans(&current_scan, &previous_scan, &odom_guess);
slam_pose = slam_pose.compose(&result.transform);  // Correct!

// WRONG: Don't invert!
// slam_pose = slam_pose.compose(&result.transform.inverse());
```

### Available Matchers

| Matcher | Config | Use Case |
|---------|--------|----------|
| `PointToPointIcp` | `IcpConfig` | General purpose, small errors |
| `PointToLineIcp` | `PointToLineIcpConfig` | Structured environments (walls) |
| `CorrelativeMatcher` | `CorrelativeConfig` | Large initial errors |
| `MultiResolutionMatcher` | `MultiResolutionConfig` | Balanced performance |
| `HybridMatcher<C, F>` | `HybridConfig` | Coarse + fine refinement |

### DynMatcher (Runtime Selection)

The `DynMatcher` allows selecting the algorithm at runtime:

```rust
use crate::algorithms::matching::{DynMatcher, MatcherType};

let matcher_type = MatcherType::MultiRes;  // From config
let mut matcher = DynMatcher::new(matcher_type);

let result = matcher.match_scans(&source, &target, &initial_guess);
```

Available types:
- `MatcherType::Icp` - Point-to-point ICP
- `MatcherType::P2l` - Point-to-line ICP
- `MatcherType::Correlative` - Correlative search
- `MatcherType::MultiRes` - Multi-resolution correlative
- `MatcherType::HybridIcp` - Correlative + P2P ICP
- `MatcherType::HybridP2l` - Correlative + P2L ICP

### Robust Kernels

ICP supports robust loss functions for outlier handling:

```rust
pub enum RobustKernel {
    None,           // L2 loss
    Huber,          // Huber loss
    Cauchy,         // Cauchy loss
    Welsch,         // Welsch loss (recommended)
}
```

## Mapping

Occupancy grid mapping with ray tracing.

### OccupancyGrid

2D grid map with log-odds representation.

```rust
use crate::algorithms::mapping::{OccupancyGrid, OccupancyGridConfig, CellState};

let config = OccupancyGridConfig {
    resolution: 0.05,          // 5cm per cell
    initial_width: 400,        // 20m
    initial_height: 400,
    origin_x: -10.0,           // Origin at center
    origin_y: -10.0,
    free_threshold: 0.3,
    occupied_threshold: 0.7,
    clamp_min: -5.0,           // Log-odds limits
    clamp_max: 5.0,
};

let mut grid = OccupancyGrid::new(config);

// Query cell state
match grid.get_state(cx, cy) {
    CellState::Free => { /* navigable */ }
    CellState::Occupied => { /* obstacle */ }
    CellState::Unknown => { /* unexplored */ }
}

// Get probability
let prob = grid.get_probability(cx, cy);

// World <-> cell conversion
let (cx, cy) = grid.world_to_cell(world_x, world_y);
let (wx, wy) = grid.cell_to_world(cx, cy);
```

### MapIntegrator

Integrates point clouds into the occupancy grid:

```rust
use crate::algorithms::mapping::{MapIntegrator, MapIntegratorConfig};

let config = MapIntegratorConfig {
    hit_log_odds: 0.9,         // Increment for occupied
    miss_log_odds: -0.4,       // Decrement for free
    max_range: 8.0,
};

let integrator = MapIntegrator::new(config);

// Integrate scan at robot pose
integrator.integrate_cloud(&mut grid, &point_cloud, &robot_pose);
```

### FeatureExtractor

Extracts geometric features (lines, corners) from maps:

```rust
use crate::algorithms::mapping::{FeatureExtractor, FeatureExtractorConfig, MapFeatures};

let extractor = FeatureExtractor::new(FeatureExtractorConfig::default());
let features: MapFeatures = extractor.extract(&grid);

// Access extracted features
for line in &features.lines {
    println!("Line: ({}, {}) -> ({}, {})",
        line.start.x, line.start.y, line.end.x, line.end.y);
}
for corner in &features.corners {
    println!("Corner at ({}, {})", corner.x, corner.y);
}
```

### RayTracer

Bresenham ray tracing for marking free space:

```rust
use crate::algorithms::mapping::RayTracer;

let tracer = RayTracer::new();

// Trace ray, calling callback for each cell
tracer.trace(start_x, start_y, end_x, end_y, |cx, cy| {
    // Process cell at (cx, cy)
});
```

## Localization

Components for Monte Carlo Localization (particle filter).

### MotionModel

Odometry-based motion prediction with noise:

```rust
use crate::algorithms::localization::MotionModel;

let model = MotionModel::new(
    0.1,  // alpha1: rotation noise from rotation
    0.1,  // alpha2: rotation noise from translation
    0.1,  // alpha3: translation noise from translation
    0.1,  // alpha4: translation noise from rotation
);

// Sample noisy motion
let noisy_pose = model.sample(&current_pose, &odom_delta);
```

### SensorModel

Likelihood field for evaluating scan against map:

```rust
use crate::algorithms::localization::SensorModel;

let model = SensorModel::new(
    0.2,   // sigma_hit
    0.05,  // z_rand
    0.95,  // z_hit
    8.0,   // max_range
);

// Compute likelihood of scan at pose
let likelihood = model.likelihood(&pose, &scan, &map);
```

## Descriptors

Place recognition for loop closure detection.

### LidarIris

Binary descriptor for fast scan comparison:

```rust
use crate::algorithms::descriptors::{LidarIris, LidarIrisConfig};

let config = LidarIrisConfig {
    num_rows: 80,
    num_cols: 360,
    max_range: 8.0,
    signature_bits: 640,
};

// Generate descriptor from scan
let iris = LidarIris::from_scan(&point_cloud, &config);

// Compare descriptors (Hamming distance)
let distance = iris.distance(&other_iris);

// Rotation-invariant matching
let (distance, rotation_offset) = iris.match_with_rotation(&other_iris);
```

**Properties:**
- 80-byte binary signature (640 bits)
- ~30x faster than cosine similarity (ScanContext)
- Rotation-invariant matching via bit rotation

## Algorithm Selection Guide

### By Environment

| Environment | Recommended Matcher |
|-------------|---------------------|
| Structured (walls) | `HybridP2l` or `P2l` |
| Cluttered (furniture) | `MultiRes` or `HybridIcp` |
| Large open spaces | `MultiRes` with wide search |
| Dynamic (people) | Any + temporal filtering |

### By Initial Error

| Initial Error | Recommended Matcher |
|---------------|---------------------|
| < 5cm, < 5° | `Icp` or `P2l` |
| < 20cm, < 15° | `MultiRes` |
| < 50cm, < 30° | `HybridIcp` or `HybridP2l` |
| > 50cm | `Correlative` then ICP |

### By CPU Budget

| Budget | Recommended |
|--------|-------------|
| < 10ms | `Icp` with small iteration limit |
| < 30ms | `MultiRes` |
| < 50ms | `HybridIcp` |
| < 100ms | `HybridP2l` |

## Performance

| Operation | Time | Notes |
|-----------|------|-------|
| ICP (180 pts, 30 iter) | ~5ms | ARM A33 |
| P2L ICP (180 pts) | ~8ms | ARM A33 |
| Correlative (±30cm) | ~15ms | 3cm resolution |
| Multi-resolution | ~20ms | 3 levels |
| Map integration | ~5ms | Ray tracing |
| Feature extraction | ~20ms | Hough transform |
| IRIS descriptor | ~2ms | 80-byte output |
