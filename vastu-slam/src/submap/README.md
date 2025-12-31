# Submap Module

Submap-based SLAM architecture for reversible map updates, inspired by Google Cartographer.

## Module Structure

```
submap/
├── mod.rs              # Module exports
├── config.rs           # SubmapConfig with validation
├── types.rs            # Submap, StoredScan, SubmapId, SubmapState
├── manager.rs          # SubmapManager - lifecycle orchestration
├── graph.rs            # SubmapPoseGraph - submap-level optimization
└── matching.rs         # MultiSubmapMatcher - cross-submap matching
```

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                            World Frame                                   │
│                                                                          │
│    Submap 0              Submap 1              Submap 2                 │
│   ┌─────────┐           ┌─────────┐           ┌─────────┐              │
│   │ Local   │  ──T01──▶ │ Local   │  ──T12──▶ │ Local   │              │
│   │ Grid    │           │ Grid    │           │ Grid    │              │
│   │ [scans] │           │ [scans] │           │ [scans] │              │
│   └────┬────┘           └────┬────┘           └────┬────┘              │
│        │                     │                     │                    │
│        ▼                     ▼                     ▼                    │
│   origin_0              origin_1              origin_2                 │
│   (fixed anchor)        (optimizable)         (optimizable)            │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

**Key Insight:** Only submap origins change during optimization. Local grids are regenerated from stored scans at new positions.

## Submap Lifecycle

```
                         ┌──────────────────────────────────────────────┐
                         │               LIFECYCLE                       │
                         ├──────────────────────────────────────────────┤
                         │                                               │
     Create first ──────▶│  ACTIVE                                      │
                         │  • Receives all scans                         │
                         │  • Builds local grid                          │
                         │  • Only ONE active at a time                  │
                         │                                               │
     Threshold reached ─▶├──────────────────────────────────────────────┤
                         │  FILLING (overlap period)                     │
                         │  • Receives overlap scans                     │
                         │  • New Active created in parallel             │
                         │  • Scans go to BOTH submaps                   │
                         │                                               │
     Overlap complete ──▶├──────────────────────────────────────────────┤
                         │  FINALIZED                                    │
                         │  • No more scan updates                       │
                         │  • Eligible for loop closure matching         │
                         │  • Origin can be adjusted by optimizer        │
                         │                                               │
                         └──────────────────────────────────────────────┘
```

**Transition Triggers:**
- Scan count: `scans_per_submap` (default: 50)
- Distance: `max_distance_per_submap` (default: 3.0m)
- Boundary: Robot exits local grid bounds

## Key Types

### Submap

```rust
pub struct Submap {
    pub id: SubmapId,           // Unique identifier
    pub origin: Pose2D,         // World frame origin (optimizable)
    pub grid: GridStorage,      // Local occupancy grid
    pub scans: Vec<StoredScan>, // Raw scans for regeneration
    pub state: SubmapState,     // Active, Filling, or Finalized
}
```

### StoredScan

```rust
pub struct StoredScan {
    pub local_pose: Pose2D,  // Pose relative to submap origin
    pub timestamp_us: u64,   // Timestamp
    pub scan: LidarScan,     // Original polar data
}
```

Scans are stored in original polar format for:
- Perfect regeneration fidelity
- Memory efficiency (~1.4KB per scan)
- Fast re-projection

### SubmapManager

Central orchestrator for the submap architecture:

```rust
pub struct SubmapManager {
    submaps: Vec<Submap>,
    active_idx: Option<usize>,
    filling_idx: Option<usize>,
    global_grid: Option<CachedGlobalGrid>,
}
```

**Responsibilities:**
- Submap lifecycle transitions
- Scan insertion routing
- Global grid composition
- Applying pose corrections

## Usage Examples

### Basic Submap Management

```rust
use vastu_slam::submap::{SubmapManager, SubmapConfig};
use vastu_slam::grid::MapConfig;

// Create manager
let config = SubmapConfig::default();
let map_config = MapConfig::default();
let mut manager = SubmapManager::new(config, &map_config);

// Insert scans (manager handles all transitions)
for observation in observations {
    let result = manager.insert_scan(
        &observation.lidar,
        observation.pose,
        observation.timestamp_us,
    );

    if result.new_submap_created {
        println!("New submap created");
    }
    if result.submap_finalized {
        println!("Submap finalized, ready for loop closure");
    }
}

// Get composed global grid (lazy regeneration)
let global_grid = manager.global_grid();
```

### Applying Pose Corrections

After pose graph optimization:

```rust
use vastu_slam::submap::SubmapCorrection;

let corrections = vec![
    SubmapCorrection {
        submap_id: SubmapId::new(1),
        new_origin: Pose2D::new(1.0, 0.05, 0.02),  // Adjusted origin
    },
    SubmapCorrection {
        submap_id: SubmapId::new(2),
        new_origin: Pose2D::new(2.0, 0.08, 0.03),
    },
];

// Apply corrections - global grid marked dirty
manager.apply_corrections(&corrections);

// Or apply and immediately regenerate
manager.apply_corrections_and_regenerate(&corrections);
```

### Multi-Submap Matching

When robot returns to previously mapped area:

```rust
use vastu_slam::submap::{MultiSubmapMatcher, MultiSubmapMatchConfig};

let multi_config = MultiSubmapMatchConfig {
    max_overlap_distance: 5.0,  // Consider submaps within 5m
    include_active: true,
    include_finalized: true,
    max_submaps: 3,
    ..Default::default()
};

let result = manager.match_scan_multi(
    &scan,
    prior_pose,
    &matcher_config,
    &multi_config,
);

if result.converged() {
    println!("Matched in submap {}: score {:.2}",
        result.matched_submap_id, result.score());
}
```

### Loop Closure Candidates

```rust
// Get finalized submaps suitable for loop closure
let candidates = manager.loop_closure_candidates(current_pose);

for candidate in candidates {
    // Compare current scan with candidate's stored scans
    // Use LiDAR-IRIS or scan matching for loop detection
}
```

## Submap Pose Graph

Operates on submap origins (~50× fewer nodes than scan-level):

```rust
use vastu_slam::submap::{SubmapPoseGraph, SubmapLoopClosure};

let mut graph = SubmapPoseGraph::with_defaults();

// Add submaps (first is anchor)
graph.add_submap(SubmapId::new(0), origin0, None);
graph.add_submap(SubmapId::new(1), origin1, Some(odom_delta_01));
graph.add_submap(SubmapId::new(2), origin2, Some(odom_delta_12));

// Add loop closure
graph.add_loop_closure(SubmapLoopClosure {
    from_submap: SubmapId::new(2),
    to_submap: SubmapId::new(0),
    relative_pose: measured_relative,
    confidence: 0.9,
});

// Optimize
let (iterations, corrections) = graph.optimize();

// Apply to submaps
manager.apply_corrections(&corrections);
```

## Configuration

### SubmapConfig

```rust
pub struct SubmapConfig {
    pub scans_per_submap: usize,        // 50
    pub overlap_scans: usize,           // 10
    pub local_grid_size: usize,         // 200 (5m × 5m)
    pub resolution: f32,                // 0.025 (2.5cm)
    pub max_distance_per_submap: f32,   // 3.0m
    pub max_submaps: usize,             // 100
    pub min_loop_closure_gap: usize,    // 3 submaps
    pub max_loop_closure_distance: f32, // 10.0m
}
```

**Presets:**

```rust
// Small rooms (< 20m²)
let config = SubmapConfig::small_room();
// 30 scans, 160 cells (4m), 2m distance, 50 max

// Large spaces (> 50m²)
let config = SubmapConfig::large_space();
// 80 scans, 300 cells (7.5m), 5m distance, 200 max
```

## Memory Estimation

```rust
let config = SubmapConfig::default();

// Per submap: ~324KB
//   Grid: 200 × 200 × 6 bytes = 240KB
//   Scans: 60 × 1.4KB = 84KB
let per_submap = config.estimated_memory_per_submap();

// Total for 100 submaps: ~32MB
let total = config.estimated_total_memory();
```

## Global Grid Composition

The global grid is composed lazily from all submaps:

```rust
// First access triggers composition
let global = manager.global_grid();

// Subsequent accesses use cache
let global = manager.global_grid();  // Cached

// Insert scan marks dirty
manager.insert_scan(...);
assert!(manager.is_global_dirty());

// Next access regenerates
let global = manager.global_grid();  // Regenerated
```

**Composition Process:**
1. Compute bounds from all submap world bounds
2. Create new grid covering all submaps
3. For each submap, for each stored scan:
   - Transform scan pose to world frame
   - Apply lidar update to global grid

## Thread Safety

| Type | Thread Safety |
|------|---------------|
| `SubmapManager` | `Send` (single owner) |
| `Submap` | `Send` (single owner) |
| `SubmapPoseGraph` | `Send` (single owner) |
| `MultiSubmapMatcher` | `Send + Sync` |

The typical pattern is:
- Main thread owns `SubmapManager`
- Optimization can be done in background (clone graph)
- Apply corrections back on main thread

## Why Submaps?

1. **Reversible Updates**: Loop closure requires adjusting past poses. With submaps, only origins change - local grids regenerate from stored scans.

2. **Efficient Optimization**: 50× fewer nodes than scan-level graph (50 scans per submap).

3. **Bounded Memory**: Old submaps can be evicted if needed.

4. **Natural Loop Closure**: Finalized submaps provide stable reference maps for matching.

5. **Incremental Updates**: Active submap updates continuously; finalized submaps are stable.
