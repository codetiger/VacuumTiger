# Algorithms Module

Core SLAM algorithm implementations: scan matching, mapping, and localization.

## Overview

The algorithms module provides:
- **Matching** - Align laser scans to estimate relative transforms
- **Mapping** - Build and update occupancy grid maps
- **Localization** - Particle filter for global localization

## Matching

Scan alignment algorithms that find the transform between two point clouds.

### ScanMatcher Trait

All matchers implement this common interface:

```rust
pub trait ScanMatcher {
    fn match_scans(
        &self,
        source: &PointCloud2D,  // Scan to align
        target: &PointCloud2D,  // Reference scan
        initial_guess: &Pose2D, // Starting estimate
    ) -> ScanMatchResult;
}
```

### ScanMatchResult

Common output format:

```rust
pub struct ScanMatchResult {
    pub transform: Pose2D,       // Estimated transform (source → target)
    pub covariance: Covariance2D, // Uncertainty estimate
    pub score: f32,              // Match quality (0-1, higher = better)
    pub converged: bool,         // Did algorithm converge?
    pub iterations: u32,         // Iterations performed
    pub mse: f32,                // Mean squared error
}
```

### Point-to-Point ICP

Classic Iterative Closest Point algorithm.

```rust
use dhruva_slam::algorithms::matching::PointToPointIcp;

let config = IcpConfig {
    max_iterations: 50,
    translation_epsilon: 0.001,   // Convergence threshold (m)
    rotation_epsilon: 0.001,      // Convergence threshold (rad)
    max_correspondence_distance: 0.5,
    min_correspondences: 10,
    outlier_ratio: 0.1,
};

let icp = PointToPointIcp::new(config);
let result = icp.match_scans(&source, &target, &initial_guess);

if result.converged && result.score > 0.8 {
    let corrected_pose = initial_guess.compose(&result.transform);
}
```

**Algorithm:**
1. Find nearest neighbors (using k-d tree)
2. Compute optimal transform via SVD
3. Apply transform to source points
4. Repeat until convergence

**Best for:** Small initial errors (<20cm, <10°)

### Point-to-Line ICP

Variant that aligns points to line segments.

```rust
use dhruva_slam::algorithms::matching::PointToLineIcp;

let icp = PointToLineIcp::new(config);
let result = icp.match_scans(&source, &target, &initial_guess);
```

**Best for:** Structured environments (walls, corridors)

### Correlative Matcher

Exhaustive grid search over transform space.

```rust
use dhruva_slam::algorithms::matching::CorrelativeMatcher;

let config = CorrelativeConfig {
    search_grid_resolution: 0.02,   // 2cm grid cells
    search_range_x: 0.3,            // ±30cm search
    search_range_y: 0.3,
    search_range_theta: 0.3,        // ±17° search
    min_score_threshold: 0.5,
};

let matcher = CorrelativeMatcher::new(config);
let result = matcher.match_scans(&source, &target, &initial_guess);
```

**Best for:** Large initial errors, global matching

### MultiResolutionMatcher

Hierarchical correlative matching for efficiency.

```rust
use dhruva_slam::algorithms::matching::MultiResolutionMatcher;

let config = MultiResolutionConfig {
    resolutions: vec![0.08, 0.04, 0.02],  // Coarse to fine
    search_grid_resolution: 0.02,
    // ...
};

let matcher = MultiResolutionMatcher::new(config);
```

### HybridMatcher

Combines correlative (global) with ICP (refinement).

```rust
use dhruva_slam::algorithms::matching::HybridMatcher;

let matcher = HybridMatcher::new(correlative_config, icp_config);
let result = matcher.match_scans(&source, &target, &initial_guess);
```

**Strategy:**
1. Run correlative matcher for coarse alignment
2. Refine with ICP for sub-centimeter accuracy

## Mapping

Occupancy grid mapping with ray tracing.

### OccupancyGrid

2D grid map with log-odds representation.

```rust
use dhruva_slam::algorithms::mapping::OccupancyGrid;

let config = OccupancyGridConfig {
    resolution: 0.05,          // 5cm per cell
    width: 400,                // 20m wide
    height: 400,               // 20m tall
    origin_x: -10.0,           // Origin at center
    origin_y: -10.0,
    free_threshold: 0.3,       // Below = free
    occupied_threshold: 0.7,   // Above = occupied
};

let mut grid = OccupancyGrid::new(config);

// Query cell state
let cell = grid.get_cell(world_x, world_y);
match cell {
    CellState::Free => { /* navigable */ }
    CellState::Occupied => { /* obstacle */ }
    CellState::Unknown => { /* unexplored */ }
}

// Get probability
let prob = grid.get_probability(world_x, world_y);
```

**Log-odds representation:**
```
log_odds = log(p / (1 - p))

p = 0.5 (unknown)  → log_odds = 0
p = 0.7 (occupied) → log_odds = 0.85
p = 0.3 (free)     → log_odds = -0.85
```

### RayTracer

Bresenham ray tracing for free space.

```rust
use dhruva_slam::algorithms::mapping::RayTracer;

let tracer = RayTracer::new();

// Trace ray from origin to endpoint
let cells = tracer.trace(
    &grid,
    origin_x, origin_y,
    endpoint_x, endpoint_y,
);
// Returns list of (cell_x, cell_y) along ray
```

### MapIntegrator

Integrates laser scans into the occupancy grid.

```rust
use dhruva_slam::algorithms::mapping::MapIntegrator;

let config = MapIntegratorConfig {
    hit_log_odd: 0.9,          // Increment for occupied
    miss_log_odd: -0.4,        // Decrement for free
    update_free_space: true,   // Trace rays through free space
};

let integrator = MapIntegrator::new(config);

// Integrate scan at robot pose
integrator.integrate(&mut grid, &point_cloud, &robot_pose);
```

**Integration process:**
1. For each point in scan:
   - Transform point to world coordinates
   - Trace ray from robot to point (mark free)
   - Mark endpoint as occupied

### MapRegion

Bounding box for regional updates.

```rust
use dhruva_slam::algorithms::mapping::MapRegion;

let region = MapRegion {
    min_x: 0.0,
    min_y: 0.0,
    max_x: 5.0,
    max_y: 5.0,
};

// Update only within region
integrator.integrate_region(&mut grid, &cloud, &pose, &region);
```

## Localization

Monte Carlo Localization (Particle Filter).

### ParticleFilter

Full Bayes filter with particle representation.

```rust
use dhruva_slam::algorithms::localization::ParticleFilter;

let config = ParticleFilterConfig {
    num_particles: 500,
    resample_threshold: 0.5,  // Effective sample ratio
};

let mut pf = ParticleFilter::new(config, initial_pose, &map);

// Motion update (prediction)
pf.predict(&odometry_delta, &motion_model);

// Sensor update (correction)
pf.update(&point_cloud, &sensor_model);

// Get estimate
let pose = pf.mean_pose();
let covariance = pf.covariance();
```

### MotionModel

Odometry-based motion prediction with noise.

```rust
use dhruva_slam::algorithms::localization::MotionModel;

let config = MotionModelConfig {
    alpha1: 0.1,  // Rotation noise from rotation
    alpha2: 0.1,  // Rotation noise from translation
    alpha3: 0.1,  // Translation noise from translation
    alpha4: 0.1,  // Translation noise from rotation
};

let motion_model = MotionModel::new(config);
```

### SensorModel (Likelihood Field)

Evaluates scan likelihood against map.

```rust
use dhruva_slam::algorithms::localization::LikelihoodFieldModel;

let config = LikelihoodFieldConfig {
    max_range: 8.0,
    beam_count: 60,       // Subsample scan
    sigma: 0.2,           // Gaussian std dev
    z_hit: 0.9,           // Weight for hit model
    z_random: 0.1,        // Weight for random model
};

let sensor_model = LikelihoodFieldModel::new(config, &map);

// Evaluate particle
let weight = sensor_model.likelihood(&particle_pose, &point_cloud);
```

**Likelihood field:**
- Pre-computes distance transform of map
- Fast lookup: O(n) for n beams (no ray tracing)

## File Structure

```
algorithms/
├── mod.rs              # Module exports
├── matching/
│   ├── mod.rs          # Matching exports
│   ├── icp.rs          # Point-to-point ICP
│   ├── icp_line.rs     # Point-to-line ICP
│   ├── correlative.rs  # Correlative matcher
│   ├── multi_res.rs    # Multi-resolution
│   ├── hybrid.rs       # Hybrid matcher
│   └── result.rs       # ScanMatchResult
├── mapping/
│   ├── mod.rs          # Mapping exports
│   ├── occupancy.rs    # OccupancyGrid
│   ├── ray_tracer.rs   # Bresenham ray tracing
│   ├── integrator.rs   # MapIntegrator
│   └── region.rs       # MapRegion
└── localization/
    ├── mod.rs          # Localization exports
    ├── particle.rs     # ParticleFilter
    ├── motion.rs       # MotionModel
    └── sensor.rs       # LikelihoodFieldModel
```

## Algorithm Selection Guide

| Scenario | Recommended Algorithm |
|----------|----------------------|
| Small errors (<10cm) | Point-to-Point ICP |
| Structured environments | Point-to-Line ICP |
| Large errors (>20cm) | Correlative Matcher |
| Real-time performance | Multi-Resolution + ICP |
| Global localization | Particle Filter |
| Incremental mapping | MapIntegrator |

## Performance

| Operation | Time | Notes |
|-----------|------|-------|
| ICP (180 points) | ~2ms | 50 iterations max |
| Correlative (±30cm) | ~10ms | 0.02m resolution |
| Multi-resolution | ~5ms | 3 resolution levels |
| Map integration | ~1ms | Ray tracing included |
| Particle filter (500) | ~5ms | 60-beam likelihood |

## References

- **ICP**: Besl & McKay, "A Method for Registration of 3-D Shapes", 1992
- **Correlative**: Olson, "Real-Time Correlative Scan Matching", 2009
- **Likelihood Field**: Thrun et al., "Probabilistic Robotics", Chapter 6
- **MCL**: Dellaert et al., "Monte Carlo Localization for Mobile Robots", 1999
