# VastuSLAM

A high-performance 2D SLAM library for robot vacuum cleaners, featuring multi-sensor occupancy grid mapping, scan matching, and loop closure.

## Features

- **Multi-Sensor Occupancy Grid**: Semantic cell types (Floor, Wall, Cliff, Bump) with Bayesian probability updates
- **Scan Matching**: Correlative and branch-and-bound scan-to-map alignment with Gauss-Newton refinement
- **Submap Architecture**: Cartographer-inspired reversible map updates for loop closure correction
- **Loop Closure**: LiDAR-IRIS descriptors with pose graph optimization
- **SIMD Optimized**: Explicit `std::simd` operations with `f32x4` for portable SIMD (ARM NEON, x86 SSE/AVX)
- **SoA Data Layout**: Structure-of-Arrays for cache-friendly, SIMD-efficient memory access

## Quick Start

```rust
use vastu_slam::{OccupancyGridMap, MapConfig, Pose2D, LidarScan};

// Create map with default configuration (800×800 cells, 2.5cm resolution = 20m×20m)
let config = MapConfig::default();
let mut map = OccupancyGridMap::new(config);

// Process lidar scan
let pose = Pose2D::new(0.0, 0.0, 0.0);
let scan = LidarScan::new(ranges, angles, 0.15, 8.0);
let result = map.observe_lidar(&scan, pose);

println!("Updated {} cells", result.cells_updated);
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              VastuSLAM Pipeline                                 │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  Sensors                          SLAM Core                         Output      │
│  ───────                          ─────────                         ──────      │
│                                                                                 │
│  ┌─────────┐     ┌─────────────────────────────────────────────┐   ┌─────────┐  │
│  │ LiDAR   │────▶│           Scan Matching                     │──▶│ Pose    │  │
│  │ 360°    │     │  ┌──────────────────────────────────────┐   │   │ Output  │  │
│  └─────────┘     │  │ Coarse: Correlative / Branch-Bound   │   │   └─────────┘  │
│                  │  │ Fine: Gauss-Newton + Distance Field  │   │                │
│  ┌─────────┐     │  └──────────────────────────────────────┘   │   ┌─────────┐  │
│  │ Cliff   │────▶│                    │                        │──▶│ Map     │  │
│  │ Sensors │     │                    ▼                        │   │ .vastu  │  │
│  └─────────┘     │  ┌──────────────────────────────────────┐   │   │ .pgm    │  │
│                  │  │         Occupancy Grid               │   │   └─────────┘  │
│  ┌─────────┐     │  │  • Log-odds Bayesian updates         │   │                │
│  │ Bumper  │────▶│  │  • Priority: Bump > Cliff > Wall     │   │   ┌─────────┐  │
│  │ Sensors │     │  │  • Bresenham ray casting             │   │──▶│ Paths   │  │
│  └─────────┘     │  └──────────────────────────────────────┘   │   │ A*      │  │
│                  │                    │                        │   └─────────┘  │
│  ┌─────────┐     │                    ▼                        │                │
│  │ Encoder │────▶│  ┌──────────────────────────────────────┐   │                │
│  │ Odom    │     │  │         Submap Manager               │   │                │
│  └─────────┘     │  │  • Local grid per submap             │   │                │
│                  │  │  • Stored scans for regeneration     │   │                │
│                  │  │  • Origin-only optimization          │   │                │
│                  │  └──────────────────────────────────────┘   │                │
│                  │                    │                        │                │
│                  │                    ▼                        │                │
│                  │  ┌──────────────────────────────────────┐   │                │
│                  │  │         Loop Closure                 │   │                │
│                  │  │  • LiDAR-IRIS descriptors            │   │                │
│                  │  │  • Pose graph optimization           │   │                │
│                  │  │  • Submap origin correction          │   │                │
│                  │  └──────────────────────────────────────┘   │                │
│                  └─────────────────────────────────────────────┘                │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Core Algorithms

### 1. Occupancy Grid with Log-Odds

VastuSLAM uses a Bayesian occupancy grid with log-odds representation for numerically stable probability updates:

```
Log-Odds Update:
L(cell | observation) = L(cell) + L(sensor_model)

Where:
• L_hit = +20 (observation confirms obstacle)
• L_miss = -4 (observation confirms free space)
• L_occupied_threshold = +50 → classify as Wall
• L_free_threshold = -50 → classify as Floor
```

The asymmetric weights (20 vs -4) mean a single hit strongly indicates an obstacle, while multiple passes are needed to clear a false positive.

### 2. Multi-Resolution Scan Matching

Scan matching corrects encoder drift by aligning LiDAR scans to the existing map:

```
Pipeline:
┌───────────────────────────────────────────────────────────────────┐
│ Step 1: COARSE SEARCH                                             │
│   • Window: ±30cm × ±30cm × ±8.6°                                 │
│   • Resolution: 4cm linear, 2.3° angular                          │
│   • Method: Brute-force or branch-and-bound                       │
│   • Scoring: Gaussian distance field (smooth gradients)           │
├───────────────────────────────────────────────────────────────────┤
│ Step 2: FINE SEARCH                                               │
│   • Window: ±6cm × ±6cm × ±3.4°                                   │
│   • Resolution: 2cm linear, 1.1° angular                          │
│   • Prior-penalized scoring                                       │
├───────────────────────────────────────────────────────────────────┤
│ Step 3: GAUSS-NEWTON REFINEMENT                                   │
│   • Adaptive Levenberg-Marquardt                                  │
│   • Sub-millimeter accuracy                                       │
│   • Convergence: cost change < 1e-6                               │
└───────────────────────────────────────────────────────────────────┘
```

**Gaussian Scoring** provides smooth gradients for optimization:
```
Score(point) = exp(-d² / 2σ²)

Where d = distance to nearest wall, σ = 0.10m (typical)
```

### 3. Submap Architecture

Inspired by Google Cartographer, VastuSLAM uses submaps for reversible map updates:

```
┌───────────────────────────────────────────────────────────────┐
│                        World Frame                            │
│                                                               │
│    Submap 0              Submap 1              Submap 2       │
│   ┌─────────┐           ┌─────────┐           ┌─────────┐     │
│   │ Local   │  ──T01──▶ │ Local   │  ──T12──▶ │ Local   │     │
│   │ Grid    │           │ Grid    │           │ Grid    │     │
│   │ [scans] │           │ [scans] │           │ [scans] │     │
│   └────┬────┘           └────┬────┘           └────┬────┘     │
│        ▼                     ▼                     ▼          │
│   origin_0              origin_1              origin_2        │
│   (fixed)               (adjustable)          (adjustable)    │
└───────────────────────────────────────────────────────────────┘

Key Insight: Only submap origins change during optimization.
             Local grids regenerate from stored scans.
```

**Submap Lifecycle:**
1. **Active**: Receives scans, builds local grid
2. **Filling**: Overlap period (scans go to both old and new submap)
3. **Finalized**: No more updates, eligible for loop closure

### 4. Loop Closure Detection

Uses LiDAR-IRIS descriptors for efficient place recognition:

```
LiDAR-IRIS Pipeline:
1. Project scan → radial signature (range histogram by angle)
2. Extract features via Fourier transform
3. Create binary descriptor (rotation-invariant)
4. Compare using Hamming distance (fast bit operations)
5. Geometric validation with scan matching
6. Pose graph optimization if validated
```

### 5. Cell Type Priority

Multiple sensors update the same cells. Priority ensures correct classification:

```
Priority (highest to lowest):
┌─────────────────────────────────────────────────────┐
│ 4. Bump (invisible obstacle - glass, chair legs)    │ ← Bumper sensor
├─────────────────────────────────────────────────────┤
│ 3. Cliff (floor drop-off - stairs, ledges)          │ ← Cliff sensor
├─────────────────────────────────────────────────────┤
│ 2. Wall (visible obstacle)                          │ ← LiDAR endpoint
├─────────────────────────────────────────────────────┤
│ 1. Floor (traversable)                              │ ← LiDAR ray
├─────────────────────────────────────────────────────┤
│ 0. Unknown (not observed)                           │ ← Initial state
└─────────────────────────────────────────────────────┘
```

## Module Documentation

| Module | Description | Documentation |
|--------|-------------|---------------|
| [`core`](src/core/) | Fundamental types: Pose2D, WorldPoint, GridCoord, LidarScan, Cell | [README](src/core/README.md) |
| [`grid`](src/grid/) | Occupancy grid storage with SoA layout, ray casting, sensor updates | [README](src/grid/README.md) |
| [`matching`](src/matching/) | Scan matching (correlative, branch-bound), loop closure, pose graph | [README](src/matching/README.md) |
| [`submap`](src/submap/) | Submap manager, pose graph, multi-submap matching | [README](src/submap/README.md) |
| [`config`](src/config/) | YAML configuration loading with sensible defaults | [README](src/config/README.md) |
| [`io`](src/io/) | Persistence: native .vastu format, ROS .pgm export, SVG visualization | [README](src/io/README.md) |
| [`modes`](src/modes/) | Operation modes: localization-only (no map updates) | [README](src/modes/README.md) |
| [`evaluation`](src/evaluation/) | Cartographer-style evaluation: ATE, RPE, pose relations metrics | [README](src/evaluation/README.md) |

## Coordinate System

All coordinates follow the ROS REP-103 convention:

```
                    +X (Forward)
                        ▲
                        │
                        │
        +Y (Left) ◄─────┼───── -Y (Right)
                        │
                        │
                        ▼
                    -X (Backward)

Rotation: Counter-clockwise positive (θ = 0 points to +X)
```

## Configuration

### YAML Configuration

```yaml
# config.yaml
grid:
  resolution: 0.025       # 2.5cm cells
  initial_width: 800      # 20m × 20m
  initial_height: 800

sensor:
  robot:
    radius: 0.17          # Robot radius for collision
  lidar:
    offset_x: -0.110      # LiDAR position relative to robot center
    max_range: 8.0

slam:
  correlative:
    enabled: true
    search_x: 0.30        # ±30cm search window
    search_y: 0.30
    search_theta: 0.15    # ±8.6° search window
    multi_resolution: true
    use_gaussian_scoring: true

  loop_closure:
    enabled: true
    min_score_threshold: 0.6
```

### Programmatic Configuration

```rust
use vastu_slam::config::VastuConfig;
use std::path::Path;

// Load from YAML file
let config = VastuConfig::load(Path::new("config.yaml"))?;

// Or use defaults
let config = VastuConfig::default();

// Convert to runtime configs
let map_config = config.to_map_config();
let matcher_config = config.correlative_matcher_config();
```

## Memory Layout

VastuSLAM uses Structure-of-Arrays (SoA) for SIMD efficiency:

```
Traditional AoS (Array of Structs):
┌──────────────────────────────────────────────────┐
│ Cell0{type,conf,swept} │ Cell1{...} │ Cell2{...} │ ...
└──────────────────────────────────────────────────┘
  ↑ Cache line may span multiple cells, mixed data types

VastuSLAM SoA (Struct of Arrays):
┌───────────────────────────────────────┐
│ types:  [t0, t1, t2, t3, t4, t5, ...] │ ← f32x4 SIMD loads
├───────────────────────────────────────┤
│ confs:  [c0, c1, c2, c3, c4, c5, ...] │ ← Contiguous memory
├───────────────────────────────────────┤
│ swept:  [s0, s1, s2, s3, s4, s5, ...] │ ← Cache-friendly
└───────────────────────────────────────┘
```

## Performance Characteristics

| Metric | Typical Value | Notes |
|--------|---------------|-------|
| Binary size | ~100KB | Stripped, statically linked |
| Memory (800×800) | ~4MB | SoA grid storage |
| Scan matching | 5-15ms | ARM Cortex-A7 @ 1GHz |
| Map save/load | <10ms | Native .vastu format |
| Resolution | 2.5cm | Default, configurable |

## Building

### Development

```bash
# Requires nightly Rust for portable_simd
rustup override set nightly

# Build and test
cargo build --release
cargo test
```

### Cross-Compilation (ARM)

```bash
rustup target add armv7-unknown-linux-musleabihf
cargo build --release --target armv7-unknown-linux-musleabihf
```

### Documentation

```bash
cargo +nightly doc --no-deps --open
```

## Usage Examples

### Basic Mapping

```rust
use vastu_slam::{OccupancyGridMap, MapConfig, Pose2D, LidarScan, SensorObservation};

let mut map = OccupancyGridMap::new(MapConfig::default());

// Process complete sensor observation
let observation = SensorObservation {
    pose: Pose2D::new(0.0, 0.0, 0.0),
    lidar: Some(scan),
    cliffs: cliff_sensors,
    bumpers: bumper_sensors,
    timestamp_us: 0,
};

let result = map.observe(&observation);
println!("Floor: {}, Wall: {}, Cliff: {}, Bump: {}",
    result.cells_floor, result.cells_wall,
    result.cells_cliff, result.cells_bump);
```

### Scan Matching with Drift Correction

```rust
use vastu_slam::{OccupancyGridMap, CorrelativeMatcherConfig};

let mut map = OccupancyGridMap::new(MapConfig::default());
let matcher_config = CorrelativeMatcherConfig::default();

// Match scan and integrate at corrected pose
let (result, corrected_pose, match_result) =
    map.observe_lidar_with_matching(&scan, encoder_pose, &matcher_config);

if match_result.converged {
    println!("Corrected pose: ({:.3}, {:.3})",
        corrected_pose.x, corrected_pose.y);
}
```

### Localization on Known Map

```rust
use vastu_slam::modes::{Localizer, LocalizerConfig};
use vastu_slam::io::load_vastu;

// Load pre-built map
let map = load_vastu(Path::new("map.vastu"))?;

// Create localizer
let mut localizer = Localizer::new(map, LocalizerConfig::default());
localizer.set_initial_pose(dock_pose);

// Localization loop
let result = localizer.localize(&scan, Some(odom_delta));
if result.converged {
    println!("Robot at: ({:.2}, {:.2})", result.pose.x, result.pose.y);
}
```

### Submap-Based SLAM

```rust
use vastu_slam::submap::{SubmapManager, SubmapConfig};

let mut manager = SubmapManager::new(SubmapConfig::default(), &map_config);

// Insert scans (manager handles submap transitions)
let result = manager.insert_scan(&scan, pose, timestamp_us);
if result.new_submap_created {
    println!("Started new submap");
}

// Get composed global grid
let global_grid = manager.global_grid();
```

## File Formats

| Format | Extension | Purpose | Preserves |
|--------|-----------|---------|-----------|
| Native | `.vastu` | Full persistence | All cell data (type, confidence, swept) |
| ROS Map | `.pgm` + `.yaml` | ROS integration | Occupancy only (Free/Occupied/Unknown) |
| SVG | `.svg` | Visualization | Visual map + trajectory overlay |

## Thread Safety

| Type | Thread Safety | Typical Usage |
|------|---------------|---------------|
| `OccupancyGridMap` | `Send` | Single-threaded map owner |
| `GridStorage` | `Send` | Single-threaded access |
| `CorrelativeMatcher` | `Send + Sync` | Shared across threads |
| `SubmapManager` | `Send` | Single-threaded manager |
| `Localizer` | `Send` | Single-threaded localizer |

## License

MIT
