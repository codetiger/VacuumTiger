# Vastu-Map Accuracy Improvement Proposal

## Overview

This document outlines remaining architectural and algorithmic gaps between Vastu-Map and Google Cartographer, with proposed solutions to achieve comparable accuracy for 2D SLAM on embedded platforms.

**Goal**: Improve scan-to-map matching accuracy while maintaining exploration-friendly occupancy grid representation.

**Constraints**:
- ARM Cortex-A7 target (Allwinner A33)
- No external optimizer dependencies (Ceres, g2o)
- Real-time operation at 5Hz lidar rate
- Memory budget: <50MB for map storage

---

## Completed Items

The following features from the original proposal have been fully implemented:

### Submap Architecture (Gap 1 - COMPLETED)

Full submap architecture with state machine lifecycle:
- `Submap` struct with origin, local grid, raw scan storage
- `SubmapManager` with Active → Filling → Finalized state machine
- Overlap scan handling during submap transitions
- Lazy global grid recomposition with dirty flag caching
- Raw scan storage for pose graph adjustment

**Location**: `src/submap/` (types.rs, manager.rs)

### Loop Closure Detection (COMPLETED)

LiDAR-IRIS descriptor-based loop closure detection:
- Descriptor computation and matching
- Pose graph structure with odometry and loop edges

**Location**: `src/slam/loop_closure/`

### SIMD Optimizations (PARTIAL - Foundation Complete)

Basic SIMD infrastructure using `std::simd`:
- `f32x4` batch point transformations
- SoA grid storage with `u8x16` vectorization
- ARM NEON / x86 SSE portable implementation

**Location**: `src/core/simd.rs`, `src/grid/storage.rs`

### Gauss-Newton Refinement (PARTIAL)

Scan matching with Cartographer-style prior constraints:
- Prior weights for translation (500.0) and rotation (20.0)
- Fixed LM damping (λ=1e-3)

**Location**: `src/slam/matcher.rs`

---

## Remaining Gaps (Priority Order)

### Priority 1: Motion Filter

**Current State**:
Every lidar scan triggers scan matching and grid update.

**Problem**:
- Stationary robot accumulates redundant observations
- Wasted computation on identical poses
- Submap size varies with robot speed

**Cartographer's Approach**:
Motion filter gates scan insertion:
- `max_time_seconds`: 5s (insert at least every 5s)
- `max_distance_meters`: 0.2m
- `max_angle_radians`: 0.17 rad (~10°)

**Proposed Solution**:

```rust
pub struct MotionFilter {
    last_pose: Pose2D,
    last_time: Instant,
    config: MotionFilterConfig,
}

pub struct MotionFilterConfig {
    pub max_time: Duration,      // 5s
    pub max_distance: f32,       // 0.2m
    pub max_angle: f32,          // 0.17 rad
}

impl MotionFilter {
    pub fn should_process(&mut self, pose: Pose2D, time: Instant) -> bool {
        let dt = time.duration_since(self.last_time);
        let dist = (pose.x - self.last_pose.x).hypot(pose.y - self.last_pose.y);
        let angle = (pose.theta - self.last_pose.theta).abs();

        let should = dt > self.config.max_time
            || dist > self.config.max_distance
            || angle > self.config.max_angle;

        if should {
            self.last_pose = pose;
            self.last_time = time;
        }

        should
    }
}
```

**Implementation Effort**: Low (1-2 hours)
**Impact**: Reduces CPU usage, consistent submap sizes

---

### Priority 2: Log-Odds Probability Cells

**Current State**:
Cells use a `confidence: u8` counter with priority-based type overwriting:
```rust
pub struct Cell {
    pub cell_type: CellType,       // Unknown/Floor/Wall/Cliff/Bump
    pub confidence: u8,            // 0-255, saturates
    pub observation_count: u8,
    pub swept: bool,
}
```

**Problem**:
- Counter saturates quickly, losing uncertainty information
- No principled way to combine conflicting observations
- Priority system prevents dynamic obstacle handling

**Cartographer's Approach**:
Proper Bayesian update using log-odds representation:
```
P(occupied | observations) ∝ P(observations | occupied) × P(occupied)
```

In log-odds space, multiplication becomes addition:
```
L(x) = log(P(x) / (1 - P(x)))
L_new = L_old + L_observation
```

**Proposed Solution**:

```rust
pub struct Cell {
    /// Log-odds occupancy (fixed-point: actual = value / 100)
    /// Range: [-200, 200] maps to probability [0.12, 0.88]
    pub log_odds: i16,

    /// Semantic type (Wall, Floor, Cliff, Bump)
    pub cell_type: CellType,

    /// Last observation timestamp (for decay)
    pub last_seen: u16,
}

impl Cell {
    // Observation log-odds (tunable parameters)
    const L_HIT: i16 = 70;    // P=0.67 → confident occupied
    const L_MISS: i16 = -30;  // P=0.43 → slight evidence of free
    const L_MIN: i16 = -200;  // P=0.12 → very likely free
    const L_MAX: i16 = 200;   // P=0.88 → very likely occupied

    pub fn apply_hit(&mut self) {
        self.log_odds = (self.log_odds.saturating_add(Self::L_HIT))
            .clamp(Self::L_MIN, Self::L_MAX);
        self.cell_type = CellType::Wall;
    }

    pub fn apply_miss(&mut self) {
        self.log_odds = (self.log_odds.saturating_add(Self::L_MISS))
            .clamp(Self::L_MIN, Self::L_MAX);
        if self.log_odds < 0 {
            self.cell_type = CellType::Floor;
        }
    }

    pub fn probability(&self) -> f32 {
        let l = self.log_odds as f32 / 100.0;
        1.0 / (1.0 + (-l).exp())
    }

    pub fn is_occupied(&self) -> bool {
        self.log_odds > 50  // P > 0.62
    }

    pub fn is_free(&self) -> bool {
        self.log_odds < -50  // P < 0.38
    }
}
```

**Migration Path**:
1. Add `log_odds: i16` to GridStorage (SoA layout already supports this)
2. Update `apply_observation()` to use log-odds arithmetic
3. Keep `cell_type` for semantic information (Cliff, Bump sensors)
4. Update scan matching scoring to use probability values

**Implementation Effort**: Medium (4-6 hours)
**Impact**: Better uncertainty handling, dynamic obstacle support, thinner walls

---

### Priority 3: Adaptive Levenberg-Marquardt

**Current State**:
Gauss-Newton with fixed LM damping (λ=1e-3) in both scan matching and pose graph:
```rust
// src/slam/matcher.rs line 273
let damping = self.config.gn_damping;  // Fixed 1e-3
```

**Problem**:
- Fixed damping may be too aggressive (slow convergence) or too weak (overshoot)
- No step acceptance/rejection logic
- Convergence not guaranteed

**Cartographer's Approach**:
Ceres Solver with trust-region method (automatic damping adjustment).

**Proposed Solution** (without Ceres dependency):

```rust
pub struct AdaptiveLM {
    lambda: f64,
    lambda_factor: f64,
    min_lambda: f64,
    max_lambda: f64,
}

impl AdaptiveLM {
    pub fn new() -> Self {
        Self {
            lambda: 1e-3,
            lambda_factor: 10.0,
            min_lambda: 1e-7,
            max_lambda: 1e7,
        }
    }

    pub fn step(
        &mut self,
        current_cost: f64,
        jacobian: &Matrix3xN,
        residuals: &VectorN,
    ) -> (Vector3, StepResult) {
        let jtj = jacobian.transpose() * jacobian;
        let jtr = jacobian.transpose() * residuals;

        loop {
            // Add damping to diagonal
            let mut damped = jtj.clone();
            damped[(0,0)] += self.lambda;
            damped[(1,1)] += self.lambda;
            damped[(2,2)] += self.lambda;

            // Solve for step
            let delta = solve_3x3(&damped, &(-jtr));

            // Predict cost reduction using linear model
            let predicted_reduction = -delta.dot(&jtr)
                - 0.5 * delta.dot(&(jtj * &delta));

            if predicted_reduction > 0.0 {
                // Step looks promising
                return (delta, StepResult::TryStep {
                    predicted_reduction,
                    lambda_on_success: (self.lambda / self.lambda_factor)
                        .max(self.min_lambda),
                    lambda_on_failure: (self.lambda * self.lambda_factor)
                        .min(self.max_lambda),
                });
            }

            // Predicted to increase cost, increase damping
            self.lambda = (self.lambda * self.lambda_factor).min(self.max_lambda);

            if self.lambda >= self.max_lambda {
                return (Vector3::zeros(), StepResult::Converged);
            }
        }
    }

    pub fn accept_step(&mut self, actual_reduction: f64, predicted_reduction: f64) {
        let rho = actual_reduction / predicted_reduction;

        if rho > 0.25 {
            // Good step, decrease damping
            self.lambda = (self.lambda / self.lambda_factor).max(self.min_lambda);
        } else if rho < 0.1 {
            // Bad step, increase damping
            self.lambda = (self.lambda * self.lambda_factor).min(self.max_lambda);
        }
        // Otherwise keep lambda unchanged
    }
}
```

**Implementation Effort**: Medium (3-4 hours)
**Impact**: Better convergence, fewer iterations, more robust matching

---

### Priority 4: Branch-and-Bound Search

**Current State**:
Correlative scan matching uses 2-level coarse-to-fine search:
- Coarse: resolution × 2.0 factor over 30cm window
- Fine: Full resolution around best coarse candidate

Evaluates ~2,000-5,000 poses per match.

**Problem**:
- Cannot efficiently handle larger search windows (>30cm)
- Poor odometry requires 50cm+ windows
- Computational cost scales linearly with window size

**Cartographer's Approach**:
Branch-and-bound with precomputed multi-resolution grids:
- 4 resolution levels (1×, 4×, 16×, 64× cell size)
- Each coarse cell stores max probability of all finer cells
- DFS search with pruning: if upper_bound ≤ best_score, skip subtree

**Proposed Solution**:

```
Precomputed Grid Hierarchy:

Level 0 (2.5cm):  [0.8][0.2][0.9][0.1][0.7][0.3][0.8][0.2]
                    \   /       \   /       \   /       \   /
Level 1 (10cm):     [0.8]       [0.9]       [0.7]       [0.8]
                      \           /           \           /
Level 2 (40cm):         [0.9]                   [0.8]
                            \                   /
Level 3 (160cm):              [0.9]

Upper bound: max of children guarantees no better solution exists below
```

```rust
pub struct PrecomputedGrids {
    /// Resolution levels: [2.5cm, 10cm, 40cm, 160cm]
    levels: [Grid; 4],
}

impl PrecomputedGrids {
    pub fn from_submap(grid: &LocalGrid) -> Self {
        let mut levels = [Grid::default(); 4];
        levels[0] = grid.probability_grid();

        for i in 1..4 {
            levels[i] = Self::downsample_max(&levels[i-1], 4);
        }

        Self { levels }
    }

    fn downsample_max(grid: &Grid, factor: usize) -> Grid {
        // Each output cell = max of factor×factor input cells
        let mut out = Grid::new(grid.width / factor, grid.height / factor);
        for y in 0..out.height {
            for x in 0..out.width {
                let mut max_val = 0.0f32;
                for dy in 0..factor {
                    for dx in 0..factor {
                        max_val = max_val.max(
                            grid.get(x * factor + dx, y * factor + dy)
                        );
                    }
                }
                out.set(x, y, max_val);
            }
        }
        out
    }
}

pub fn branch_and_bound_match(
    scan: &[Point2D],
    grids: &PrecomputedGrids,
    search_window: SearchWindow,
) -> Option<(Pose2D, f32)> {
    let mut best = (Pose2D::identity(), 0.0f32);
    let mut stack = Vec::with_capacity(1024);

    // Start with coarsest level covering entire window
    stack.push(SearchNode {
        level: 3,
        x_range: search_window.x_range,
        y_range: search_window.y_range,
        theta_range: search_window.theta_range,
    });

    while let Some(node) = stack.pop() {
        // Compute upper bound at this resolution
        let upper_bound = score_scan_upper_bound(
            scan,
            &grids.levels[node.level],
            node.center_pose(),
        );

        // Prune if cannot beat current best
        if upper_bound <= best.1 {
            continue;
        }

        if node.level == 0 {
            // Finest level: compute exact score
            let score = score_scan_exact(scan, &grids.levels[0], node.center_pose());
            if score > best.1 {
                best = (node.center_pose(), score);
            }
        } else {
            // Subdivide into 8 children (2×2×2 in x,y,theta)
            for child in node.subdivide() {
                stack.push(child);
            }
        }
    }

    (best.1 > MIN_MATCH_SCORE).then_some(best)
}
```

**Complexity**:
- Brute force: O(W² × A) where W=window size, A=angular steps
- Branch-and-bound: O(log(W) × k) where k=matched poses (typically <100)

**Implementation Effort**: High (8-12 hours)
**Impact**: 10× speedup for large search windows, enables 50cm+ windows

---

### Priority 5: Background Pose Graph Optimization

**Current State**:
Pose graph optimization runs synchronously when loop closure is detected.

**Problem**:
- Blocks scan processing during optimization (100-500ms)
- Cannot continuously refine poses as new constraints arrive
- Single-shot optimization may not converge fully

**Cartographer's Approach**:
- Background thread runs optimization continuously
- Triggered every N nodes (configurable)
- Results applied incrementally to trajectory
- Multiple iterations for better convergence

**Proposed Solution**:

```
┌─────────────────────────────────────────────────────────┐
│                     SLAM Thread                          │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────┐  │
│  │  Scan    │───▶│  Match   │───▶│ Insert to Submap │  │
│  │ Receive  │    │  (Local) │    │ + Add Graph Node │  │
│  └──────────┘    └──────────┘    └────────┬─────────┘  │
│                                           │             │
│                                           ▼             │
│                              ┌────────────────────────┐ │
│                              │ Loop Closure Detector  │ │
│                              │ (descriptor matching)  │ │
│                              └───────────┬────────────┘ │
│                                          │              │
└──────────────────────────────────────────┼──────────────┘
                                           │ trigger
                                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Background Optimizer Thread              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │ Copy Graph  │───▶│  Optimize   │───▶│   Publish   │  │
│  │  Snapshot   │    │ (50 iters)  │    │ Corrections │  │
│  └─────────────┘    └─────────────┘    └─────────────┘  │
└──────────────────────────────────────────────────────────┘
```

```rust
pub struct BackgroundOptimizer {
    graph: Arc<RwLock<PoseGraph>>,
    config: OptimizationConfig,
}

pub struct OptimizationConfig {
    pub trigger_every_n_nodes: usize,      // 20
    pub max_iterations: usize,             // 50
    pub convergence_threshold: f64,        // 1e-5
    pub huber_scale: f64,                  // 0.1 (10cm)
    pub odometry_weight: f64,              // 100.0
    pub loop_closure_weight: f64,          // 50.0
}

impl BackgroundOptimizer {
    pub fn run(&self, trigger_rx: Receiver<()>, result_tx: Sender<PoseCorrections>) {
        while trigger_rx.recv().is_ok() {
            // Snapshot current graph
            let graph_snapshot = self.graph.read().clone();

            // Run optimization (doesn't block SLAM thread)
            let optimized = self.optimize(&graph_snapshot);

            // Compute corrections
            let corrections = self.compute_corrections(&graph_snapshot, &optimized);

            // Send back to SLAM thread
            let _ = result_tx.send(corrections);
        }
    }

    fn optimize(&self, graph: &PoseGraph) -> Vec<Pose2D> {
        let mut poses = graph.poses.clone();

        for iteration in 0..self.config.max_iterations {
            let (h, b) = self.build_linear_system(graph, &poses);
            let delta = self.solve_sparse(&h, &b);

            // Apply update
            let max_update = self.apply_update(&mut poses, &delta);

            if max_update < self.config.convergence_threshold {
                break;
            }
        }

        poses
    }
}
```

**Implementation Effort**: Medium-High (6-8 hours)
**Impact**: Non-blocking optimization, better convergence

**Note**: For embedded ARM target, background threading may not be critical. Synchronous optimization at 100-500ms is acceptable if triggered only on loop closure events (rare).

---

## Updated Implementation Roadmap

```
Phase 1: Quick Wins (Immediate)
├── Motion filter                    ← PRIORITY 1 (1-2 hours)
└── Adaptive LM refinement           ← PRIORITY 3 (3-4 hours)

Phase 2: Accuracy Improvement
└── Log-odds probability cells       ← PRIORITY 2 (4-6 hours)

Phase 3: Performance
├── Branch-and-bound matcher         ← PRIORITY 4 (8-12 hours)
├── Precomputed grid hierarchy
└── SIMD for B&B scoring

Phase 4: Optional (if needed)
└── Background optimizer thread      ← PRIORITY 5 (6-8 hours)
    (may not be needed for embedded target)
```

---

## Memory Budget Analysis (Updated)

| Component | Current | After Phase 2 | Notes |
|-----------|---------|---------------|-------|
| Global Grid (800×800) | 2.5 MB | 4 MB | +log_odds (i16) |
| Submaps (10 active) | 2 MB | 2 MB | Already implemented |
| Raw Scans (1000) | 2 MB | 2 MB | Already implemented |
| Precomputed Grids | - | 3 MB | Phase 3 |
| Pose Graph | 0.5 MB | 0.5 MB | Already implemented |
| **Total** | ~7 MB | ~12 MB | Within 50MB budget |

---

## Expected Accuracy Improvements

| Metric | Current | After Phase 1 | After Phase 2 | Method |
|--------|---------|---------------|---------------|--------|
| Drift (per 10m) | ~4cm | ~3cm | ~2cm | Adaptive LM |
| Loop closure error | ~2cm | ~2cm | ~1cm | Already good |
| Wall thickness | 5-8cm | 5-8cm | 2-3cm | Log-odds |
| CPU per match | 15ms | 12ms | 12ms | Motion filter |
| Ghost artifacts | Rare | Rare | Very rare | Already good |

---

## References

1. Hess, W., et al. "Real-Time Loop Closure in 2D LIDAR SLAM." IEEE ICRA 2016.
2. Cartographer ROS Documentation: https://google-cartographer-ros.readthedocs.io/
3. Olson, E. "Real-Time Correlative Scan Matching." IEEE ICRA 2009.
4. Konolige, K. "Improved Occupancy Grids for Map Building." Autonomous Robots 1997.
