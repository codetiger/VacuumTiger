# Engine Module

SLAM orchestration layer that coordinates algorithms into a complete system.

## Overview

The engine module provides:
- **Online SLAM** - Real-time SLAM with keyframes and submaps
- **Pose Graph** - Global optimization and loop closure

## Online SLAM

The main SLAM engine that processes scans in real-time.

### SlamEngine Trait

Common interface for SLAM implementations:

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

### OnlineSlam

Production SLAM implementation with keyframes, submaps, and recovery.

```rust
use dhruva_slam::engine::slam::OnlineSlam;

let config = OnlineSlamConfig {
    // Preprocessing
    min_range: 0.15,
    max_range: 8.0,
    target_points: 180,

    // Scan matching
    matcher_type: MatcherType::Hybrid,
    icp_config: IcpConfig { ... },
    correlative_config: CorrelativeConfig { ... },

    // Keyframes
    keyframe_config: KeyframeConfig {
        min_translation_m: 0.3,
        min_rotation_rad: 0.2,
        max_frames_per_submap: 20,
    },

    // Mapping
    map_resolution: 0.05,
    map_size_cells: 400,
};

let mut slam = OnlineSlam::new(config);

// Process each scan
let result = slam.process_scan(&cloud, &odom_delta, timestamp_us);

if result.converged {
    let corrected_pose = result.pose;
    let created_keyframe = result.keyframe_created;
}
```

### SlamResult

Output from processing a scan:

```rust
pub struct SlamResult {
    pub pose: Pose2D,              // Corrected robot pose
    pub covariance: Covariance2D,  // Uncertainty
    pub keyframe_created: bool,    // New keyframe added?
    pub new_submap: bool,          // New submap started?
    pub match_score: f32,          // Match quality (0-1)
    pub pose_refined: bool,        // Was pose corrected?
}
```

### SlamStatus

Current system state:

```rust
pub struct SlamStatus {
    pub mode: SlamMode,            // Mapping/Localization/Idle
    pub num_scans: u64,            // Total scans processed
    pub num_keyframes: usize,      // Total keyframes
    pub num_submaps: usize,        // Total submaps
    pub num_finished_submaps: usize,
    pub last_match_score: f32,
    pub is_lost: bool,             // Kidnapped detection
    pub memory_usage: usize,       // Bytes
}
```

### SlamMode

Operating modes:

```rust
pub enum SlamMode {
    Mapping,       // Build map while localizing
    Localization,  // Localize in existing map
    Idle,          // Pause SLAM processing
}

// Change mode
slam.set_mode(SlamMode::Localization);
```

### Keyframes

Pose-stamped scans for loop closure.

```rust
pub struct Keyframe {
    pub id: u64,
    pub pose: Pose2D,              // Global pose
    pub scan: PointCloud2D,        // Original scan
    pub timestamp_us: u64,
    pub submap_id: u64,
}

// Access keyframe's global scan
let global_scan = keyframe.global_scan();

// Get scan context for loop closure
let context = keyframe.scan_context();
```

### ScanContext

Rotation-invariant descriptor for place recognition.

```rust
pub struct ScanContext {
    descriptor: Vec<Vec<f32>>,  // 60 sectors × 20 rings
    ring_key: Vec<f32>,         // Fast lookup key
}

// Compare two scan contexts
let similarity = context1.compare(&context2);
```

**Structure:**
- Divides scan into 60 angular sectors
- Each sector has 20 range bins (rings)
- Stores max range in each bin
- Ring key enables fast candidate retrieval

### KeyframeManager

Decides when to create new keyframes.

```rust
use dhruva_slam::engine::slam::KeyframeManager;

let config = KeyframeManagerConfig {
    min_translation_m: 0.3,     // At least 30cm movement
    min_rotation_rad: 0.2,      // Or 11° rotation
    max_frames_per_submap: 20,  // Before new submap
};

let mut manager = KeyframeManager::new(config);

// Check if keyframe should be created
if manager.should_create_keyframe(&current_pose, &last_keyframe_pose) {
    let keyframe = manager.create_keyframe(pose, scan, timestamp_us);
}
```

### Submaps

Local occupancy grid partitions.

```rust
pub struct Submap {
    pub id: u64,
    pub state: SubmapState,
    pub grid: OccupancyGrid,      // Local map
    pub keyframes: Vec<Keyframe>,
    pub origin: Pose2D,           // Submap origin in global frame
}

pub enum SubmapState {
    Unfinished,       // Still receiving scans
    Finished,         // Complete, optimizable
    TrimmedFinished,  // Complete, memory-optimized
}
```

### SubmapManager

Manages submap lifecycle.

```rust
use dhruva_slam::engine::slam::SubmapManager;

let config = SubmapManagerConfig {
    max_keyframes_per_submap: 20,
    submap_resolution: 0.05,
    submap_grid_size: 200,
};

let mut manager = SubmapManager::new(config);

// Add keyframe (may create new submap)
manager.add_keyframe(keyframe);

// Get active submap for integration
let active = manager.active_submap_mut();

// Finish current submap
manager.finish_current_submap();
```

### KidnappedDetector

Detects if robot is lost (poor match quality).

```rust
use dhruva_slam::engine::slam::KidnappedDetector;

let config = KidnappedDetectorConfig {
    match_score_threshold: 0.3,
    samples_window: 10,
};

let mut detector = KidnappedDetector::new(config);

// Update with match result
detector.update(match_score);

if detector.is_kidnapped() {
    // Trigger recovery
}
```

### RecoveryStateMachine

Recovers from kidnapped state.

```rust
use dhruva_slam::engine::slam::RecoveryStateMachine;

let mut recovery = RecoveryStateMachine::new(config);

// Attempt recovery
match recovery.attempt(&scan, &map) {
    RecoveryResult::Success(pose) => { /* relocalized */ }
    RecoveryResult::InProgress => { /* still searching */ }
    RecoveryResult::Failed => { /* give up */ }
}
```

## Pose Graph

Global optimization via pose graph.

### PoseGraph

Graph structure for optimization.

```rust
use dhruva_slam::engine::graph::PoseGraph;

let mut graph = PoseGraph::new();

// Add nodes (poses)
graph.add_node(PoseNode { id: 0, pose: Pose2D::origin() });
graph.add_node(PoseNode { id: 1, pose: Pose2D::new(1.0, 0.0, 0.0) });

// Add edges (constraints)
graph.add_edge(PoseEdge {
    from: 0,
    to: 1,
    relative_pose: Pose2D::new(1.0, 0.0, 0.0),
    information: Information2D::from_diagonal(100.0, 100.0, 1000.0),
    edge_type: EdgeType::Odometry,
});
```

### PoseNode

A pose in the graph:

```rust
pub struct PoseNode {
    pub id: u64,
    pub pose: Pose2D,
}
```

### PoseEdge

Constraint between poses:

```rust
pub struct PoseEdge {
    pub from: u64,                    // Source node ID
    pub to: u64,                      // Target node ID
    pub relative_pose: Pose2D,        // Measured transform
    pub information: Information2D,   // Inverse covariance (confidence)
    pub edge_type: EdgeType,
}

pub enum EdgeType {
    Odometry,     // Sequential constraint
    LoopClosure,  // Non-sequential (detected)
}
```

### Information2D

Inverse covariance matrix (confidence weighting):

```rust
pub struct Information2D {
    pub data: [[f32; 3]; 3],
}

// Higher values = more confident
let info = Information2D::from_diagonal(
    100.0,   // x confidence
    100.0,   // y confidence
    1000.0,  // theta confidence
);
```

### LoopDetector

Detects loop closures using scan context.

```rust
use dhruva_slam::engine::graph::LoopDetector;

let config = LoopDetectorConfig {
    max_candidates: 5,
    min_score_threshold: 0.5,
    yaw_search_range: 0.5,  // ±28°
    min_keyframe_gap: 20,   // Skip recent keyframes
};

let detector = LoopDetector::new(config);

// Find loop closure candidates
let candidates = detector.detect(&current_keyframe, &all_keyframes);

for candidate in candidates {
    if candidate.score > 0.7 {
        // Verify with scan matching
        let result = matcher.match_scans(
            &current_keyframe.scan,
            &candidate.keyframe.scan,
            &candidate.initial_guess,
        );

        if result.converged {
            // Add loop closure edge to graph
        }
    }
}
```

### LoopClosureCandidate

Potential loop closure:

```rust
pub struct LoopClosureCandidate {
    pub keyframe_id: u64,
    pub score: f32,               // Context similarity
    pub initial_guess: Pose2D,    // Estimated transform
}
```

### GraphOptimizer

Optimizes pose graph to minimize error.

```rust
use dhruva_slam::engine::graph::GraphOptimizer;

let config = GraphOptimizerConfig {
    max_iterations: 100,
    convergence_threshold: 1e-6,
    algorithm: OptimizationAlgorithm::GaussNewton,
};

let optimizer = GraphOptimizer::new(config);

// Optimize graph
let result = optimizer.optimize(&mut graph);

match result.termination {
    Termination::Converged => { /* success */ }
    Termination::MaxIterations => { /* partial */ }
    Termination::NoProgress => { /* stuck */ }
}

// Graph nodes now have optimized poses
```

### OptimizationResult

Optimization outcome:

```rust
pub struct OptimizationResult {
    pub iterations: u32,
    pub initial_error: f64,
    pub final_error: f64,
    pub termination: Termination,
}
```

## Data Flow

```
Odometry + Scan
      │
      ▼
┌─────────────────┐
│   OnlineSlam    │
│                 │
│ ┌─────────────┐ │     ┌──────────────┐
│ │ Preprocessor│─┼────►│ Scan Matcher │
│ └─────────────┘ │     └──────┬───────┘
│                 │            │
│ ┌─────────────┐ │            ▼
│ │  Keyframe   │◄┼────── Match Result
│ │  Manager    │ │
│ └──────┬──────┘ │
│        │        │
│ ┌──────▼──────┐ │
│ │   Submap    │ │
│ │   Manager   │ │
│ └──────┬──────┘ │
└────────┼────────┘
         │
         ▼
┌─────────────────┐
│   PoseGraph     │
│                 │
│ ┌─────────────┐ │
│ │ LoopDetector│ │
│ └──────┬──────┘ │
│        ▼        │
│ ┌─────────────┐ │
│ │  Optimizer  │ │
│ └─────────────┘ │
└─────────────────┘
```

## File Structure

```
engine/
├── mod.rs              # Module exports
├── slam/
│   ├── mod.rs          # SLAM exports
│   ├── online.rs       # OnlineSlam
│   ├── result.rs       # SlamResult, SlamStatus
│   ├── keyframe.rs     # Keyframe, KeyframeManager
│   ├── scan_context.rs # ScanContext
│   ├── submap.rs       # Submap, SubmapManager
│   ├── kidnapped.rs    # KidnappedDetector
│   └── recovery.rs     # RecoveryStateMachine
└── graph/
    ├── mod.rs          # Graph exports
    ├── pose_graph.rs   # PoseGraph, PoseNode, PoseEdge
    ├── loop_detector.rs # LoopDetector
    └── optimizer.rs    # GraphOptimizer
```

## Usage Example

Complete SLAM pipeline:

```rust
use dhruva_slam::engine::slam::OnlineSlam;
use dhruva_slam::engine::graph::{PoseGraph, GraphOptimizer};

// Initialize SLAM
let mut slam = OnlineSlam::new(config);

// Main loop
loop {
    // Get sensor data
    let (odom_delta, scan) = get_sensor_data();

    // Process scan
    let result = slam.process_scan(&scan, &odom_delta, timestamp_us);

    // Check for loop closure opportunity
    if result.keyframe_created {
        if let Some(closure) = detect_loop_closure(&slam) {
            // Add to pose graph
            slam.add_loop_closure(closure);

            // Optimize if enough closures
            if slam.status().num_loop_closures > 5 {
                slam.optimize();
            }
        }
    }

    // Get current estimate
    let pose = slam.current_pose();
    let map = slam.map();

    // Publish results
    publish_pose(&pose);
    publish_map(&map);
}
```

## Performance

| Operation | Time | Notes |
|-----------|------|-------|
| process_scan (no keyframe) | ~5ms | Preprocessing + matching |
| process_scan (with keyframe) | ~10ms | + map integration |
| Loop detection | ~2ms | Scan context comparison |
| Graph optimization (100 nodes) | ~50ms | 10 iterations |
