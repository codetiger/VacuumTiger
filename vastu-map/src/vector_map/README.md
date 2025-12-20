# VectorMap Module

The main SLAM map implementation using line and corner features.

## Components

| File | Description |
|------|-------------|
| `map.rs` | `VectorMap` - Main SLAM map implementing the `Map` trait |
| `config.rs` | `VectorMapConfig` - Configuration for all SLAM components |
| `line_store.rs` | `LineStore` - Line storage with synchronized auxiliary structures |

## Overview

VectorMap is the primary SLAM implementation. It:

1. **Extracts** features from lidar scans (lines and corners)
2. **Matches** scans against the map (localization via ICP)
3. **Integrates** new features (mapping)
4. **Queries** the map (raycast, occupancy, frontiers)

## Quick Start

```rust
use vastu_map::{VectorMap, VectorMapConfig, Map};
use vastu_map::core::{Pose2D, PointCloud2D, Point2D};

let config = VectorMapConfig::default();
let mut map = VectorMap::new(config);

// Process scans
let scan = PointCloud2D::from_points(&points);
let odometry = Pose2D::identity();
let result = map.observe(&scan, odometry);

println!("Pose: ({:.2}, {:.2}), confidence: {:.2}",
    result.pose.x, result.pose.y, result.confidence);
```

## Map Trait

VectorMap implements the `Map` trait:

```rust
pub trait Map: Send + Sync {
    fn observe(&mut self, scan: &PointCloud2D, odometry: Pose2D) -> ObserveResult;
    fn raycast(&self, from: Point2D, direction: Point2D, max_range: f32) -> f32;
    fn query(&self, point: Point2D) -> Occupancy;
    fn frontiers(&self) -> Vec<Frontier>;
    fn get_path(&self, from: Point2D, to: Point2D) -> Option<Path>;
    fn bounds(&self) -> Bounds;
    fn clear(&mut self);
}
```

## Configuration

```rust
let config = VectorMapConfig {
    // Feature extraction
    extraction: SplitMergeConfig {
        split_threshold: 0.03,
        min_segment_points: 5,
        min_segment_length: 0.2,
        ..Default::default()
    },

    // Scan matching
    matching: IcpConfig {
        max_iterations: 20,
        convergence_threshold: 0.001,
        max_correspondence_distance: 0.5,
        ..Default::default()
    },

    // Map integration
    association: AssociationConfig::default(),
    merger: MergerConfig::default(),

    // Corner detection
    corner: CornerConfig::default(),

    // Loop closure
    loop_closure: LoopClosureConfig::default(),

    // Query settings
    occupancy: OccupancyConfig::default(),
    frontier: FrontierConfig::default(),

    // Minimum confidence to integrate scan
    min_integration_confidence: 0.3,

    // Maximum map size (lines)
    max_map_lines: 10000,
};
```

## Observe Result

```rust
pub struct ObserveResult {
    pub pose: Pose2D,              // Localized pose
    pub confidence: f32,           // Match confidence (0.0-1.0)
    pub features_extracted: usize, // Lines extracted from scan
    pub features_added: usize,     // New lines added to map
    pub features_merged: usize,    // Lines merged with existing
    pub icp_iterations: usize,     // ICP iterations performed
    pub icp_converged: bool,       // Did ICP converge?
    pub loop_closure_detected: bool, // Was loop closure detected?
}
```

## LineStore

Internal storage with synchronized spatial index:

```rust
pub struct LineStore {
    lines: Vec<Line2D>,
    spatial_index: SpatialIndex,
    observation_counts: Vec<usize>,
}

impl LineStore {
    fn push(&mut self, line: Line2D);
    fn get(&self, idx: usize) -> Option<&Line2D>;
    fn query_near(&self, point: Point2D, radius: f32) -> Vec<usize>;
    fn rebuild_index(&mut self);
}
```

## SLAM Pipeline

```
                          ┌─────────────────┐
                          │   Lidar Scan    │
                          └────────┬────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │   Extraction    │ → Lines + Corners
                          └────────┬────────┘
                                   │
              ┌────────────────────┼────────────────────┐
              │                    │                    │
              ▼                    ▼                    ▼
     ┌────────────────┐   ┌────────────────┐   ┌────────────────┐
     │   ICP Match    │   │  Loop Closure  │   │                │
     │   (if enough   │   │   Detection    │   │                │
     │    features)   │   │                │   │                │
     └───────┬────────┘   └───────┬────────┘   │                │
             │                    │            │                │
             ▼                    ▼            │                │
     ┌────────────────┐   ┌────────────────┐   │                │
     │  Confidence?   │   │  Loop Found?   │   │                │
     │   >= 0.3       │   │                │   │                │
     └───────┬────────┘   └───────┬────────┘   │                │
             │                    │            │                │
             └──────────┬─────────┘            │                │
                        │                      │                │
                        ▼                      │                │
               ┌────────────────┐              │                │
               │  Integration   │◄─────────────┘                │
               │  (Associate +  │                               │
               │   Merge + Add) │                               │
               └───────┬────────┘                               │
                       │                                        │
                       ▼                                        │
               ┌────────────────┐                               │
               │   VectorMap    │───────────────────────────────┘
               │   (Lines +     │
               │   Corners +    │ → Queries (raycast, occupancy)
               │   Index)       │ → Frontiers (exploration)
               └────────────────┘
```

## Thread Safety

VectorMap implements `Send + Sync`, allowing it to be shared across threads with proper synchronization (e.g., `Arc<Mutex<VectorMap>>`).
