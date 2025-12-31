# Matching Module

Scan-to-map matching and loop closure capabilities for correcting encoder drift during mapping.

## Module Structure

```
matching/
├── mod.rs                  # Module exports
├── config.rs               # CorrelativeMatcherConfig
├── types.rs                # ScanMatchResult, MatchQuality
├── traits.rs               # ScanMatcher trait
├── robust.rs               # Robust loss functions (Huber, Cauchy)
├── matcher/                # Correlative scan matcher
│   ├── mod.rs              # Module tests
│   ├── core.rs             # CorrelativeMatcher implementation
│   ├── lm.rs               # Adaptive Levenberg-Marquardt optimizer
│   ├── helpers.rs          # Utility functions
│   └── types.rs            # ScratchBuffers for SIMD
├── branch_bound/           # Branch-and-bound matcher
│   ├── mod.rs              # Module exports and tests
│   ├── config.rs           # BranchBoundConfig, BranchBoundResult
│   ├── grids.rs            # PrecomputedGrids (multi-resolution)
│   ├── algorithm.rs        # Search algorithm
│   └── search_node.rs      # Priority queue node
├── loop_closure/           # Loop closure detection
│   ├── mod.rs              # Module exports
│   ├── descriptor.rs       # LiDAR-IRIS descriptor
│   ├── detector.rs         # LoopClosureDetector
│   ├── validator.rs        # Geometric validation
│   └── graph.rs            # PoseGraph with optimization
└── background_optimizer.rs # Async pose graph optimization
```

## Scan Matching Pipeline

```
┌──────────────────────────────────────────────────────────────────────┐
│                     SCAN MATCHING PIPELINE                           │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  LidarScan + Encoder Pose                                            │
│       │                                                              │
│       ▼                                                              │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │              COARSE SEARCH (Multi-Resolution)                   │ │
│  │  ┌──────────────────────────────────────────────────────────┐   │ │
│  │  │ Search Window: ±30cm × ±30cm × ±8.6°                     │   │ │
│  │  │ Resolution: 4cm linear, 2.3° angular                     │   │ │
│  │  │ Method: Brute-force grid search                          │   │ │
│  │  │ Scoring: Gaussian (distance field) or binary (cell type) │   │ │
│  │  └──────────────────────────────────────────────────────────┘   │ │
│  └───────────────────────────┬─────────────────────────────────────┘ │
│                              │ Best candidate                        │
│                              ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │                    FINE SEARCH                                  │ │
│  │  ┌──────────────────────────────────────────────────────────┐   │ │
│  │  │ Search Window: ±6cm × ±6cm × ±3.4°                       │   │ │
│  │  │ Resolution: 2cm linear, 1.1° angular                     │   │ │
│  │  │ Centered on coarse result, penalized by prior            │   │ │
│  │  └──────────────────────────────────────────────────────────┘   │ │
│  └───────────────────────────┬─────────────────────────────────────┘ │
│                              │ Refined candidate                     │
│                              ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │              GAUSS-NEWTON REFINEMENT                            │ │
│  │  ┌──────────────────────────────────────────────────────────┐   │ │
│  │  │ Method: Adaptive Levenberg-Marquardt                     │   │ │
│  │  │ Objective: Minimize distance to walls + prior constraint │   │ │
│  │  │ Convergence: Sub-millimeter accuracy                     │   │ │
│  │  └──────────────────────────────────────────────────────────┘   │ │
│  └───────────────────────────┬─────────────────────────────────────┘ │
│                              │                                       │
│                              ▼                                       │
│                    ScanMatchResult                                   │
│                   (pose, score, quality)                             │
└──────────────────────────────────────────────────────────────────────┘
```

## Key Components

### CorrelativeMatcher

The primary scan matcher using brute-force search with multi-resolution refinement:

```rust
use vastu_slam::matching::{CorrelativeMatcher, CorrelativeMatcherConfig, ScanMatcher};

// Create matcher with default config
let matcher = CorrelativeMatcher::with_defaults();

// Match scan against map
let result = matcher.match_scan(&lidar_scan, encoder_pose, &grid_storage);

if result.converged {
    let corrected_pose = result.pose;
    let confidence = result.score;  // 0.0 - 1.0
}
```

**Configuration Options:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `search_x` | 0.3m | X search window (±) |
| `search_y` | 0.3m | Y search window (±) |
| `search_theta` | 0.15 rad | Theta search window (±8.6°) |
| `linear_resolution` | 0.02m | Linear step size |
| `angular_resolution` | 0.02 rad | Angular step size (1.1°) |
| `multi_resolution` | true | Use coarse-to-fine search |
| `use_gaussian_scoring` | true | Continuous distance-based scoring |
| `gaussian_sigma` | 0.10m | Scoring fall-off distance |

### Gaussian Scoring

Instead of binary hit/miss, uses the distance field for smooth gradients:

```
Score(point) = exp(-d² / 2σ²)

Distance 0:     score = 1.0 (on wall)
Distance σ:     score = 0.61
Distance 2σ:    score = 0.14
Distance 3σ:    score = 0.01
```

This provides:
- Smooth score landscape for optimization
- Better gradients for Gauss-Newton refinement
- Robustness to small alignment errors

### Branch-and-Bound Matcher

Efficient search for large search windows using precomputed multi-resolution grids:

```rust
use vastu_slam::matching::{
    BranchBoundConfig, PrecomputedGrids, branch_and_bound_match
};

// Precompute multi-resolution probability grids
let grids = PrecomputedGrids::from_storage(&grid_storage);

// Run branch-and-bound search
let config = BranchBoundConfig::default();
let result = branch_and_bound_match(&scan_points, &grids, prior_pose, &config);
```

**Grid Hierarchy:**

```
Level 0 (1x):   [0.8][0.2][0.9][0.1][0.7][0.3][0.8][0.2]  ← Full resolution
                  \   /       \   /       \   /       \   /
Level 1 (4x):     [0.8]       [0.9]       [0.7]       [0.8]   ← Max of 4
                    \           /           \           /
Level 2 (16x):        [0.9]                   [0.8]           ← Max of 16
```

Coarse levels provide upper bounds for pruning - if upper bound < best score, skip entire subtree.

### Loop Closure Detection

Detects when the robot revisits a previously mapped location:

```rust
use vastu_slam::matching::loop_closure::{
    LoopClosureDetector, LoopClosureConfig, LoopValidator
};

// Create detector with LiDAR-IRIS descriptors
let mut detector = LoopClosureDetector::new(LoopClosureConfig::default());

// Add scan to database
detector.add_scan(pose_idx, &lidar_scan, robot_pose);

// Check for loop closures
if let Some(closure) = detector.detect_loop(current_pose) {
    // Validate geometrically
    let validator = LoopValidator::new(LoopValidatorConfig::default());
    if validator.validate(&closure, &current_scan, &reference_scan).is_valid {
        // Apply loop closure constraint
    }
}
```

**LiDAR-IRIS Descriptor:**

Binary descriptor based on scan structure:
1. Project scan to radial signature (range histogram by angle)
2. Extract features using Fourier transform
3. Compare using Hamming distance

Advantages:
- Rotation-invariant
- Compact (few KB per scan)
- Fast comparison (bit operations)

### Pose Graph Optimization

Optimize pose graph to correct accumulated drift:

```rust
use vastu_slam::matching::{BackgroundOptimizer, BackgroundOptimizerConfig};

let mut optimizer = BackgroundOptimizer::with_defaults();

// Add poses with odometry constraints
let (pose_idx, _) = optimizer.add_pose(pose, Some(odom_delta));

// Add loop closure
let trigger = optimizer.add_loop_closure(loop_closure);

// Run optimization (synchronous mode for embedded)
let result = optimizer.optimize_sync();

// Apply corrections
for correction in result.corrections {
    // Update submap poses, etc.
}
```

**Architecture:**

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SLAM Thread                                  │
│  Scan → Match → Insert Submap → Loop Closure Detector → trigger     │
└─────────────────────────────────────┬───────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    Background Optimizer                             │
│  Copy Graph Snapshot → Optimize (N iters) → Return Corrections      │
└─────────────────────────────────────────────────────────────────────┘
```

**Modes:**
- **Synchronous** (embedded): Call `optimize_sync()` directly
- **Threaded** (desktop): Use background thread with channels

## SIMD Optimization

The matcher supports SIMD-accelerated operations:

```rust
use vastu_slam::matching::ScratchBuffers;

// Create reusable scratch buffers
let mut scratch = ScratchBuffers::new(scan.ranges.len());

// SIMD-optimized matching
let result = matcher.match_scan_simd(&scan, prior_pose, &storage, &mut scratch);
```

**SIMD-Accelerated Operations:**
1. Point transformation (sensor → world frame)
2. World → grid coordinate conversion
3. Scoring (partially - memory access is scalar)

**Performance:**
- ~2-3x speedup on ARM NEON
- ~3-4x speedup on x86 AVX2

## Match Quality Classification

```rust
pub enum MatchQuality {
    Excellent,  // score > 0.8
    Good,       // score > 0.6
    Marginal,   // score > 0.4
    Poor,       // score > 0.2
    Failed,     // score <= 0.2
}
```

**Acceptance Policy:**
- `Excellent`, `Good`, `Marginal`: Accept for pose correction
- `Poor`, `Failed`: Reject (use encoder pose)

## Gauss-Newton Refinement

Sub-pixel optimization using distance field gradients:

```
Objective: min Σᵢ d(Tᵢ)² + λₜ||t - t₀||² + λᵣ(θ - θ₀)²

Where:
- d(Tᵢ) = distance from transformed point i to nearest wall
- t, t₀  = current and prior translation
- θ, θ₀  = current and prior rotation
- λₜ, λᵣ = translation/rotation prior weights
```

**Adaptive Levenberg-Marquardt:**
- Damping increases when steps are rejected
- Damping decreases when steps improve cost
- Automatic step size control

## Configuration Presets

```rust
// Default: Balanced for typical use
let config = CorrelativeMatcherConfig::default();

// Fast: Reduced search for real-time with good odometry
let config = CorrelativeMatcherConfig::fast();

// Thorough: Large search for poor odometry or recovery
let config = CorrelativeMatcherConfig::thorough();

// Disabled: Pass-through (for odometry-only mode)
let config = CorrelativeMatcherConfig::disabled();
```

## Thread Safety

| Type | Thread Safety |
|------|---------------|
| `CorrelativeMatcher` | `Send + Sync` |
| `ScratchBuffers` | `Send` (not `Sync`) |
| `PrecomputedGrids` | `Send + Sync` |
| `LoopClosureDetector` | `Send` (not `Sync` - internal state) |
| `BackgroundOptimizer` | `Send` (owns mutex-protected graph) |
