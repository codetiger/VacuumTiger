# Grid Module

Occupancy grid implementation with multi-sensor fusion and SIMD-optimized storage.

## Module Structure

```
grid/
├── mod.rs              # Module exports
├── config.rs           # Configuration types (GridConfig, SensorConfig, MapConfig)
├── raycaster.rs        # Bresenham ray casting for lidar
├── lidar_update.rs     # Lidar scan processing (log-odds Bayesian updates)
├── cliff_update.rs     # Cliff sensor processing
├── bumper_update.rs    # Bumper collision processing
└── storage/
    ├── mod.rs          # Storage module exports
    ├── core.rs         # GridStorage implementation (SoA layout)
    └── types.rs        # Helper types (CellMut, CellCounts)
```

## Architecture Overview

### Structure-of-Arrays (SoA) Memory Layout

The grid uses SoA layout for SIMD-optimized operations:

```
Traditional Array-of-Structs (AoS):
cells: [Cell₀, Cell₁, Cell₂, Cell₃, ...]
        ├─────────────────────────────────┤
        type conf obs swept type conf obs swept ...

Structure-of-Arrays (SoA) - Our Layout:
cell_types:   [T T T T T T T T T T T T T T T T|...]  ← SIMD-friendly
confidences:  [C C C C C C C C C C C C C C C C|...]
observations: [O O O O O O O O O O O O O O O O|...]
swept_flags:  [S S S S S S S S S S S S S S S S|...]
log_odds:     [L L L L L L L L L L L L L L L L|...]
distance:     [D D D D D D D D D D D D D D D D|...]
```

Benefits:
- SIMD operations process 16 cells at once (`u8x16`)
- Cache-friendly for type-specific queries
- Memory bandwidth optimization

### Log-Odds Occupancy Model

Based on Google Cartographer's probabilistic grid model:

```
Probability ←→ Log-Odds Conversion:
L(x) = log(P(x) / (1 - P(x)))

Bayesian Update:
L_new = L_old + L_observation

Fixed-Point Storage:
Stored as i16, actual = value / 100
```

**Update Parameters:**

| Config | L_hit | L_miss | Obs. for Wall | Use Case |
|--------|-------|--------|---------------|----------|
| Cartographer | +20 | -4 | 3-5 | Robust mapping |
| Balanced | +30 | -12 | 2-3 | General purpose |
| Aggressive | +70 | -28 | 1 | Fast/controlled |

**Probability Thresholds:**
- `L > 50` → Occupied (Wall)
- `L < -50` → Free (Floor)
- `-50 ≤ L ≤ 50` → Unknown

## Sensor Update Pipeline

```
                      ┌─────────────────┐
                      │  LidarScan      │
                      │  (360° rays)    │
                      └────────┬────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────┐
│                    Lidar Update                          │
│  1. For each valid ray:                                  │
│     - Apply L_MISS to cells along ray path (Bresenham)   │
│     - Apply L_HIT to endpoint cell (if range < max)      │
│  2. Update distance field for new walls                  │
└──────────────────────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────┐
│                    Cliff Update                          │
│  - Transform sensor positions to world frame             │
│  - Mark triggered sensor cells as Cliff (priority > Wall)│
└──────────────────────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────┐
│                    Bumper Update                         │
│  - Estimate collision position from bumper zone          │
│  - Mark collision cells as Bump (highest priority)       │
└──────────────────────────────────────────────────────────┘
```

## Key Types

### GridStorage

Core storage with SoA layout:

```rust
pub struct GridStorage {
    cell_types: Vec<u8>,        // CellType as u8
    confidences: Vec<u8>,       // 0-255 confidence
    observation_counts: Vec<u8>, // Saturating counter
    swept_flags: Vec<u8>,       // Coverage tracking
    log_odds: Vec<i16>,         // Bayesian occupancy
    distance_field: Vec<f32>,   // Distance to nearest wall
    // ... metadata
}
```

**Key Methods:**
- `apply_hit(coord)` - Add obstacle evidence (log-odds += L_HIT)
- `apply_miss(coord)` - Add free space evidence (log-odds += L_MISS)
- `set_type_with_priority(coord, type)` - Respects priority hierarchy
- `count_by_type()` - SIMD-optimized cell counting

### Configuration Types

```rust
// Grid dimensions and resolution
pub struct GridConfig {
    pub resolution: f32,      // 0.025 = 2.5cm cells
    pub initial_width: usize, // 800 = 20m
    pub initial_height: usize,
    pub auto_expand: bool,    // Grow grid as needed
    pub max_width: usize,     // Memory limit
    pub max_height: usize,
}

// Robot and sensor parameters
// Note: Lidar offset is now handled by SangamIO
pub struct SensorConfig {
    pub robot_radius: f32,      // 0.17m
    pub max_lidar_range: f32,   // 8.0m
    pub min_lidar_range: f32,   // 0.15m
}

// Log-odds update parameters
pub struct LogOddsConfig {
    pub l_hit: i16,              // Evidence for obstacle
    pub l_miss: i16,             // Evidence for free space
    pub l_occupied_threshold: i16, // Wall classification threshold
    pub l_free_threshold: i16,   // Floor classification threshold
}
```

## Usage Examples

### Basic Grid Operations

```rust
use vastu_slam::grid::{GridStorage, GridConfig, MapConfig};
use vastu_slam::core::{WorldPoint, GridCoord, CellType};

// Create centered grid (20m × 20m at 2.5cm resolution)
let config = MapConfig::default();
let mut storage = GridStorage::centered(
    config.grid.initial_width,
    config.grid.initial_height,
    config.grid.resolution,
);

// Coordinate conversion
let world = WorldPoint::new(1.0, 2.0);
let grid = storage.world_to_grid(world);    // GridCoord
let back = storage.grid_to_world(grid);     // Cell center

// Cell access
let cell_type = storage.get_type(grid);
storage.set_type_with_priority(grid, CellType::Wall);
```

### Processing Lidar Scans

```rust
use vastu_slam::grid::lidar_update::update_from_lidar;

let result = update_from_lidar(
    &mut storage,
    &lidar_scan,
    robot_pose,
    &sensor_config,
    &grid_config,
);

println!("Updated {} cells, {} walls, {} floor",
    result.cells_updated, result.cells_wall, result.cells_floor);
```

### Log-Odds Queries

```rust
// Get occupancy probability at a position
let prob = storage.get_probability(coord);  // 0.0 - 1.0

// Check occupancy status
if storage.is_occupied(coord) {
    // P > 0.62 (L > 50)
}
if storage.is_free(coord) {
    // P < 0.38 (L < -50)
}

// Raw log-odds value
let log_odds = storage.get_log_odds(coord);  // i16
```

### Distance Field Queries

```rust
// Distance to nearest wall (meters)
let dist = storage.get_distance(coord);

// Interpolated distance with gradients (for optimization)
let (dist, grad_x, grad_y) = storage.get_distance_interpolated(world_point);
```

## Ray Casting

### Bresenham Line Algorithm

Efficient integer-based ray tracing:

```rust
use vastu_slam::grid::raycaster::{BresenhamLine, cells_along_ray};

// Iterate cells along a ray
let start = GridCoord::new(0, 0);
let end = GridCoord::new(10, 5);

for cell in BresenhamLine::new(start, end) {
    // Process each cell
}

// Or collect all cells
let cells = cells_along_ray(start, end);
```

### Continuous Ray Casting

For queries requiring distance information:

```rust
use vastu_slam::grid::raycaster::RayCast;

let ray = RayCast::new(
    start_point,    // WorldPoint
    angle,          // Radians
    max_range,      // Meters
    grid_origin,    // Grid origin
    resolution,     // Grid resolution
);

for (coord, distance) in ray {
    // coord: GridCoord, distance: f32
}
```

## SIMD Operations

### Cell Counting

```rust
// SIMD-optimized (processes 16 cells per iteration)
let counts = storage.count_by_type();

println!("Floor: {}, Wall: {}, Cliff: {}, Bump: {}",
    counts.floor, counts.wall, counts.cliff, counts.bump);

// Area calculation
let explored_area = counts.known() as f32 * resolution * resolution;
```

### Rect Traversability Check

```rust
// SIMD-optimized check if rectangular region is all Floor
let all_floor = storage.is_rect_all_floor(
    start_x, start_y,
    width, height,
);
```

## Grid Expansion

Dynamic grid growth when robot explores beyond bounds:

```rust
// Auto-expand is typically enabled via config
let config = GridConfig {
    auto_expand: true,
    max_width: 2000,   // Limit to 50m
    max_height: 2000,
    ..Default::default()
};

// Manual expansion
let expanded = storage.expand_to_include(
    far_point,
    max_width,
    max_height,
);
```

Expansion preserves all existing cell data and adjusts the origin.

## Memory Considerations

| Grid Size | Resolution | Memory |
|-----------|------------|--------|
| 20m × 20m | 2.5cm | ~2.5 MB |
| 50m × 50m | 2.5cm | ~16 MB |
| 100m × 100m | 5cm | ~16 MB |

Memory per cell: ~10 bytes (types + confidence + observations + log_odds + distance)

## Distance Field

The distance field stores the Euclidean distance to the nearest wall cell:

```
Wall cells:     distance = 0.0
Adjacent cells: distance = resolution
Further cells:  distance = Euclidean distance to nearest wall
Max distance:   Capped at 1.0m by default
```

**Uses:**
- Gaussian scoring in scan matching
- Navigation cost maps
- Obstacle inflation

**Update Methods:**
- `update_distance_field_from_wall(coord)` - Single wall update
- `update_distance_field_batch(&[coords])` - Batch update (faster)
- `recompute_distance_field()` - Full recomputation
