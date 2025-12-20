# Vastu-Map

A high-performance feature-based 2D SLAM library with SIMD-friendly data layouts.

## Features

- **VectorMap Representation**: Uses line segments and corners instead of occupancy grids for memory-efficient mapping
- **Full SLAM Pipeline**: Localization and mapping integrated into a clean API
- **SIMD Optimized**: Data layouts designed for LLVM auto-vectorization (works on ARM, x86, etc.)
- **SoA Data Layout**: Struct-of-Arrays for cache-friendly SIMD operations

## Quick Start

```rust
use vastu_map::{VectorMap, VectorMapConfig, Map};
use vastu_map::core::{Pose2D, PointCloud2D, Point2D};

// Create map with default configuration
let config = VectorMapConfig::default();
let mut map = VectorMap::new(config);

// Process a scan observation
let scan = PointCloud2D::from_points(&[
    Point2D::new(1.0, 0.0),
    Point2D::new(1.0, 0.1),
    Point2D::new(1.0, 0.2),
]);
let odometry = Pose2D::identity();
let result = map.observe(&scan, odometry);

println!("Pose: ({:.2}, {:.2}), confidence: {:.2}",
    result.pose.x, result.pose.y, result.confidence);
```

## Architecture

```
                         ┌─────────────────┐
                         │   Lidar Scan    │
                         │  (PolarScan)    │
                         └────────┬────────┘
                                  │ to_cartesian()
                                  ▼
                         ┌─────────────────┐
                         │  PointCloud2D   │
                         │   (SoA layout)  │
                         └────────┬────────┘
                                  │
             ┌────────────────────┼────────────────────┐
             │                    │                    │
             ▼                    ▼                    ▼
    ┌────────────────┐   ┌────────────────┐   ┌────────────────┐
    │   Extraction   │   │    Matching    │   │  Loop Closure  │
    │  (Split-Merge) │   │  (Point-Line   │   │  (Keyframes +  │
    │                │   │      ICP)      │   │  Descriptors)  │
    └───────┬────────┘   └───────┬────────┘   └───────┬────────┘
            │                    │                    │
            ▼                    ▼                    │
    ┌────────────────┐   ┌────────────────┐           │
    │  Lines/Corners │   │  Matched Pose  │           │
    │  (FeatureSet)  │   │  + Confidence  │           │
    └───────┬────────┘   └───────┬────────┘           │
            │                    │                    │
            └──────────┬─────────┘                    │
                       │                              │
                       ▼                              │
              ┌────────────────┐                      │
              │  Integration   │◄─────────────────────┘
              │ (Merge/Add to  │   Loop constraints
              │  VectorMap)    │
              └───────┬────────┘
                      │
                      ▼
              ┌────────────────┐
              │   VectorMap    │──► Query (raycast, occupancy)
              │   (Lines +     │──► Frontiers (exploration)
              │   Corners)     │──► Path Planning
              └────────────────┘
```

## Modules

| Module | Description | Documentation |
|--------|-------------|---------------|
| [`core`](src/core/) | Fundamental types (Point2D, Pose2D, PointCloud2D, Bounds) | [README](src/core/README.md) |
| [`features`](src/features/) | Feature types (Line2D, Corner2D, descriptors) | [README](src/features/README.md) |
| [`extraction`](src/extraction/) | Line extraction (split-merge, corner detection) | [README](src/extraction/README.md) |
| [`matching`](src/matching/) | Scan matching (ICP, RANSAC, Gauss-Newton) | [README](src/matching/README.md) |
| [`integration`](src/integration/) | Map integration (spatial index, association, merging) | [README](src/integration/README.md) |
| [`query`](src/query/) | Map queries (raycast, occupancy, frontier detection) | [README](src/query/README.md) |
| [`vector_map`](src/vector_map/) | Main VectorMap SLAM implementation | [README](src/vector_map/README.md) |
| [`loop_closure`](src/loop_closure/) | Keyframe-based loop closure detection | [README](src/loop_closure/README.md) |
| [`simd`](src/simd/) | SIMD-friendly types for auto-vectorization | [README](src/simd/README.md) |

## Coordinate Frame

All coordinates follow the ROS REP-103 convention:
- **X-forward**: Positive X is in front of the robot
- **Y-left**: Positive Y is to the left of the robot
- **Z-up**: Positive Z is upward (not used in 2D)
- **Rotation**: Counter-clockwise positive

## SIMD and Portability

The library uses LLVM auto-vectorization patterns that work across different architectures. No platform-specific intrinsics are used—users can configure compiler flags appropriate for their target platform.

## Building

### Development (Host)

```bash
cargo build --release
cargo test
```

### Cross-compilation

For cross-compilation, add your target and configure appropriate compiler flags:

```bash
# Example: ARM target
rustup target add armv7-unknown-linux-musleabihf
cargo build --release --target armv7-unknown-linux-musleabihf
```

### Integration Tests

Integration tests require the `integration-tests` feature:

```bash
cargo test --features integration-tests
```

## Performance Characteristics

- **Memory**: ~100 bytes per line feature (vs ~1MB per 1000x1000 occupancy grid)
- **Matching**: <10ms for typical indoor scans on embedded ARM
- **Binary Size**: ~100KB stripped

## Extensibility

The library provides traits for custom implementations:

```rust
// Custom line extraction
pub trait LineExtractor: Send + Sync {
    fn extract(&self, points: &[Point2D]) -> Vec<Line2D>;
}

// Custom scan matching
pub trait ScanMatcher: Send + Sync {
    fn match_scan(&self, points: &[Point2D], lines: &[Line2D], initial: Pose2D) -> MatchResult;
}

// Custom loop closure detection
pub trait LoopDetector: Send + Sync {
    fn detect(&mut self, pose: Pose2D, ...) -> Option<LoopClosure>;
}
```

## License

MIT
