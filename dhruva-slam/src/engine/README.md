# Engine Module

SLAM orchestration: online SLAM, keyframes, submaps, and pose graph optimization.

## Overview

The engine module provides:
- **SLAM** - Online SLAM with scan matching and mapping
- **Graph** - Pose graph optimization and loop closure detection

## Module Structure

```
engine/
├── mod.rs              # Module exports, SlamEngine trait
├── slam/
│   ├── mod.rs          # SLAM exports, SlamResult, SlamStatus
│   ├── online_slam.rs  # Main SLAM engine
│   ├── keyframe.rs     # Keyframe management
│   ├── submap.rs       # Submap lifecycle
│   ├── kidnapped_detector.rs  # Kidnap detection
│   └── recovery_state.rs      # Recovery state machine
└── graph/
    ├── mod.rs          # Graph exports
    ├── pose_graph.rs   # Graph data structure
    ├── loop_detector.rs    # Loop closure detection
    ├── optimizer.rs        # Gauss-Newton optimizer
    └── sparse_solver.rs    # Sparse CG solver
```

## SLAM

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      LOCAL SLAM                              │
│                                                              │
│  Scan → Preprocess → Match to Submap → Update Submap        │
│                           │                                  │
│                           ▼                                  │
│                   Keyframe Decision                          │
│                           │                                  │
│              ┌────────────┴────────────┐                     │
│              │                         │                     │
│              ▼                         ▼                     │
│      Create Keyframe           Continue in Submap            │
│              │                                               │
│              ▼                                               │
│      Submap Full? ──Yes──▶ Finish Submap, Create New        │
└─────────────────────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                     GLOBAL SLAM                              │
│                  (Loop Closure)                              │
│                                                              │
│  Finished Submaps → Loop Detection → Graph Optimization      │
└─────────────────────────────────────────────────────────────┘
```

### SlamEngine Trait

Common interface for all SLAM implementations:

```rust
pub trait SlamEngine {
    fn process_scan(
        &mut self,
        scan: &PointCloud2D,
        odom_delta: &Pose2D,
        timestamp_us: u64,
    ) -> SlamResult;

    fn current_pose(&self) -> Pose2D;
    fn map(&self) -> &OccupancyGrid;
    fn status(&self) -> SlamStatus;
    fn reset(&mut self);
    fn reset_to(&mut self, pose: Pose2D);
}
```

### SlamResult

```rust
pub struct SlamResult {
    pub pose: Pose2D,              // Corrected pose
    pub match_score: f32,          // Scan match quality
    pub keyframe_created: bool,    // New keyframe?
    pub submap_finished: bool,     // Submap completed?
    pub mode: SlamMode,            // Current mode
}
```

### SlamStatus

```rust
pub struct SlamStatus {
    pub mode: SlamMode,
    pub num_scans: u64,
    pub num_keyframes: usize,
    pub num_submaps: usize,
    pub last_match_score: f32,
    pub is_lost: bool,
}
```

### SlamMode

```rust
pub enum SlamMode {
    Mapping,        // Building new map
    Localization,   // Localizing in existing map
    Lost,           // Lost, attempting recovery
    Idle,           // Not processing
}
```

### Usage Example

```rust
use crate::engine::slam::{OnlineSlam, OnlineSlamConfig, SlamEngine};

// Create SLAM engine
let config = OnlineSlamConfig::default();
let mut slam = OnlineSlam::new(config);

// Process scans in sensor loop
loop {
    let scan = preprocess(&raw_scan);
    let result = slam.process_scan(&scan, &odom_delta, timestamp_us);

    if result.keyframe_created {
        log::info!("Keyframe created, score: {:.2}", result.match_score);
    }

    // Get current state
    let pose = slam.current_pose();
    let map = slam.map();
    let status = slam.status();
}
```

### Keyframe Management

Keyframes are created when:
1. Robot travels `keyframe_min_distance` meters
2. Robot rotates `keyframe_min_rotation` radians
3. Minimum time `keyframe_min_interval_us` has passed

```rust
pub struct Keyframe {
    pub id: u64,
    pub pose: Pose2D,
    pub scan: PointCloud2D,
    pub timestamp_us: u64,
}
```

### Kidnap Detection

Detects when the robot is "kidnapped" (relocated without motion):

```rust
// Triggered when match score drops significantly
if match_score < kidnap_score_threshold {
    slam.handle_kidnap();  // Triggers recovery mode
}
```

### Recovery State Machine

```
Normal ──low score──▶ Warning ──persistent low score──▶ Lost
   ▲                      │                               │
   └──────────────────────┴───────good match──────────────┘
```

## Graph

Pose graph optimization for loop closure correction.

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      POSE GRAPH                              │
│                                                              │
│    Nodes: Robot poses at keyframe times                     │
│                                                              │
│    Edges: Relative constraints between poses                │
│           - Odometry edges (sequential)                      │
│           - Loop closure edges (non-sequential)              │
│                                                              │
│    [P0] ──odom──▶ [P1] ──odom──▶ [P2] ──odom──▶ [P3]        │
│     │                              ▲                         │
│     └────────── loop closure ──────┘                         │
└─────────────────────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                     OPTIMIZATION                             │
│                                                              │
│    Minimize: Σ ||error(edge)||² weighted by information      │
│                                                              │
│    Method: Gauss-Newton with Sparse CG solver                │
└─────────────────────────────────────────────────────────────┘
```

### PoseGraph

```rust
use crate::engine::graph::{PoseGraph, Information2D};

let mut graph = PoseGraph::new();

// Add nodes (poses)
graph.add_node(pose0, timestamp0);
graph.add_node(pose1, timestamp1);

// Add odometry edge
let odom_info = Information2D::from_std_dev(0.05, 0.05, 0.02);
graph.add_odometry_edge(0, 1, relative_pose, odom_info);

// Add loop closure edge
let loop_info = Information2D::from_std_dev(0.1, 0.1, 0.05);
graph.add_loop_closure_edge(0, 10, closure_transform, loop_info, 0.95);
```

### Information2D

Information matrix (inverse covariance) for pose constraints:

```rust
// Create from standard deviations
let info = Information2D::from_std_dev(
    0.05,  // x uncertainty (m)
    0.05,  // y uncertainty (m)
    0.02,  // theta uncertainty (rad)
);

// Create identity (equal weight)
let info = Information2D::identity();

// Create scaled
let info = Information2D::scaled(100.0);
```

### LoopDetector

Detects loop closures using LiDAR-IRIS descriptors:

```rust
use crate::engine::graph::{LoopDetector, LoopDetectorConfig, LoopClosureCandidate};

let config = LoopDetectorConfig {
    min_keyframe_gap: 20,        // Skip recent poses
    max_candidates: 5,           // Top-K candidates for ICP
    max_hamming_distance: 100,   // IRIS threshold
    min_icp_score: 0.5,          // ICP verification threshold
};

let mut detector = LoopDetector::new(config);

// Add keyframe to database
detector.add_keyframe(kf_id, &scan, &pose);

// Detect loop closures
let candidates: Vec<LoopClosureCandidate> = detector.detect(
    query_id,
    &query_scan,
    &query_pose,
    &keyframes,
);

for candidate in candidates {
    println!("Loop: {} -> {} (confidence: {:.2})",
        candidate.query_id, candidate.match_id, candidate.confidence);
}
```

### LoopClosureCandidate

```rust
pub struct LoopClosureCandidate {
    pub query_id: u64,           // Current keyframe
    pub match_id: u64,           // Matched keyframe
    pub relative_pose: Pose2D,   // Transform from match to query
    pub information: Information2D,
    pub confidence: f32,         // Match confidence (0-1)
}
```

### GraphOptimizer

Optimizes pose graph using Gauss-Newton:

```rust
use crate::engine::graph::{GraphOptimizer, GraphOptimizerConfig};

let config = GraphOptimizerConfig {
    max_iterations: 50,
    tolerance: 1e-6,
    solver: SolverType::SparseCg,  // or DenseCholesky
};

let optimizer = GraphOptimizer::new(config);

// Optimize graph (modifies node poses in-place)
let result = optimizer.optimize(&mut graph);

if result.converged {
    println!("Converged in {} iterations", result.iterations);

    // Apply corrected poses to keyframes
    for node in graph.nodes() {
        keyframes[node.id].pose = node.pose;
    }
}
```

### OptimizationResult

```rust
pub struct OptimizationResult {
    pub converged: bool,
    pub iterations: usize,
    pub initial_error: f64,
    pub final_error: f64,
    pub termination_reason: TerminationReason,
}

pub enum TerminationReason {
    Converged,
    MaxIterations,
    SmallDelta,
    NumericalError,
}
```

### Sparse CG Solver

For large pose graphs (500+ nodes), use the sparse Conjugate Gradient solver:

```rust
let config = GraphOptimizerConfig {
    solver: SolverType::SparseCg,
    max_cg_iterations: 100,
    cg_tolerance: 1e-8,
    ..Default::default()
};
```

**Comparison:**

| Nodes | Dense Cholesky | Sparse CG |
|-------|----------------|-----------|
| 100 | 10ms | 2ms |
| 500 | 1,250ms | 20ms |
| 1,000 | 10,000ms | 50ms |
| 5,000 | OOM | 200ms |

## Integration Example

Complete SLAM with loop closure:

```rust
use crate::engine::slam::{OnlineSlam, SlamEngine};
use crate::engine::graph::{PoseGraph, LoopDetector, GraphOptimizer};

let mut slam = OnlineSlam::new(slam_config);
let mut graph = PoseGraph::new();
let mut detector = LoopDetector::new(detector_config);
let optimizer = GraphOptimizer::new(optimizer_config);

let mut pending_closures = Vec::new();

loop {
    // Process scan
    let result = slam.process_scan(&scan, &odom_delta, timestamp_us);

    if result.keyframe_created {
        let kf = slam.latest_keyframe();

        // Add to pose graph
        graph.add_node(kf.pose, kf.timestamp_us);
        if kf.id > 0 {
            graph.add_odometry_edge(kf.id - 1, kf.id, odom_delta, odom_info);
        }

        // Add to loop detector
        detector.add_keyframe(kf.id, &kf.scan, &kf.pose);

        // Check for loops periodically
        if kf.id % 5 == 0 && kf.id > 20 {
            let closures = detector.detect(kf.id, &kf.scan, &kf.pose, &keyframes);

            for closure in closures {
                graph.add_loop_closure_edge(
                    closure.query_id,
                    closure.match_id,
                    closure.relative_pose,
                    closure.information,
                    closure.confidence,
                );
                pending_closures.push(closure);
            }

            // Optimize after accumulating closures
            if pending_closures.len() >= 3 {
                let result = optimizer.optimize(&mut graph);
                if result.converged {
                    apply_corrections(&mut slam, &graph);
                    pending_closures.clear();
                }
            }
        }
    }
}
```

## Performance

| Operation | Time | Notes |
|-----------|------|-------|
| Scan matching | 20-50ms | Depends on matcher |
| Map integration | 5-10ms | Per scan |
| Keyframe creation | <1ms | Bookkeeping only |
| Loop detection | 2-5ms | IRIS + ICP verify |
| Graph optimization (100 nodes) | 10ms | Sparse CG |
| Graph optimization (1000 nodes) | 50ms | Sparse CG |
