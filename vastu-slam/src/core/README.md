# Core Module

Foundation types for the VastuSLAM library. This module defines the fundamental data structures used throughout the SLAM pipeline.

## Module Structure

```
core/
├── mod.rs          # Module exports
├── cell.rs         # Grid cell types (CellType, Cell)
├── point.rs        # Coordinate types (GridCoord, WorldPoint)
├── pose.rs         # Robot pose (Pose2D, transforms)
├── motion.rs       # Odometry model (MotionModel, Odometry)
├── sensors.rs      # Sensor types (LidarScan, CliffSensors, BumperSensors)
└── simd.rs         # SIMD-optimized batch operations
```

## Type Overview

### Coordinate Types (`point.rs`)

| Type | Description | Use Case |
|------|-------------|----------|
| `GridCoord` | Integer (i32, i32) grid cell indices | Occupancy grid access, path planning |
| `WorldPoint` | Float (f32, f32) world coordinates in meters | Sensor data, robot position |

**Coordinate System (ROS REP-103):**
```
        +X (Forward)
           ↑
           │
    +Y ←───┼───→ -Y
   (Left)  │   (Right)
           ↓
        -X (Back)
```

### Robot Pose (`pose.rs`)

`Pose2D` represents the robot's position and orientation:

```rust
pub struct Pose2D {
    pub x: f32,      // Position in meters (forward)
    pub y: f32,      // Position in meters (left)
    pub theta: f32,  // Orientation in radians (CCW from +X)
}
```

**Key Operations:**
- `transform_point()` - Convert robot-frame point to world-frame
- `inverse_transform_point()` - Convert world-frame point to robot-frame
- `compose()` - Combine two poses (pose composition)
- `inverse()` - Compute inverse transform

### Cell Types (`cell.rs`)

Semantic cell classification for multi-sensor fusion:

```
Priority (highest to lowest):
┌─────────┬──────────────────────────────────────────────────────┐
│ Bump    │ Physical collision detected (glass, mirrors)         │
├─────────┼──────────────────────────────────────────────────────┤
│ Cliff   │ Floor drop-off detected (stairs, ledges)             │
├─────────┼──────────────────────────────────────────────────────┤
│ Wall    │ Lidar-detected obstacle (walls, furniture)           │
├─────────┼──────────────────────────────────────────────────────┤
│ Floor   │ Traversable surface (free space)                     │
├─────────┼──────────────────────────────────────────────────────┤
│ Unknown │ Not yet observed                                     │
└─────────┴──────────────────────────────────────────────────────┘
```

Higher-priority types override lower-priority ones. For example:
- A cell marked as `Floor` can be upgraded to `Wall` if lidar detects an obstacle
- A cell marked as `Wall` can be upgraded to `Bump` if the bumper triggers
- A cell marked as `Bump` cannot be downgraded (physical evidence is definitive)

### Sensor Types (`sensors.rs`)

| Type | Description | Update Rate |
|------|-------------|-------------|
| `LidarScan` | 360° range measurements | ~5 Hz |
| `CliffSensors` | 4 IR floor drop-off detectors | ~110 Hz |
| `BumperSensors` | 2 mechanical collision sensors | Event-driven |
| `SensorObservation` | Combined sensor snapshot | Per SLAM tick |

**Sensor Positions (CRL-200S Robot):**
```
           Front
      LF ─────── RF      (Cliff sensors)
       ╲    ↑    ╱
        ╲   │   ╱
    LS ──┬──┼──┬── RS    (Cliff sensors)
         │     │
      L ─┴─────┴─ R      (Bumpers)
           Back

LF/RF = Left/Right Front cliff (±5cm from center, 15cm forward)
LS/RS = Left/Right Side cliff (±10cm from center, 12cm forward)
L/R   = Left/Right bumper (covers front arc)
```

### Motion Model (`motion.rs`)

Probabilistic motion model for odometry uncertainty:

```rust
pub struct MotionModel {
    pub alpha1: f32,  // Rotation noise from rotation (rad/rad)
    pub alpha2: f32,  // Rotation noise from translation (rad/m)
    pub alpha3: f32,  // Translation noise from translation (m/m)
    pub alpha4: f32,  // Translation noise from rotation (m/rad)
}
```

Based on Thrun's "Probabilistic Robotics" formulation. The model predicts pose uncertainty based on the motion magnitude.

### SIMD Operations (`simd.rs`)

SIMD-accelerated batch operations using `std::simd` (portable SIMD):

| Function | Description | Use Case |
|----------|-------------|----------|
| `transform_points_simd4` | Batch point transformation | Lidar scan projection |
| `world_to_grid_simd4` | Batch coordinate conversion | Grid updates |
| `distance_squared_simd4` | Batch distance calculation | Range filtering |
| `is_valid_coords_simd4` | Batch bounds checking | Grid validation |

**PointCloud (SoA Layout):**
```
Structure-of-Arrays for cache-friendly SIMD access:

xs: [x₀, x₁, x₂, x₃, x₄, x₅, x₆, x₇, ...]
ys: [y₀, y₁, y₂, y₃, y₄, y₅, y₆, y₇, ...]
     └────────────┘
      4-wide SIMD load
```

All SIMD operations require input lengths to be multiples of 4 (LANES). Use `*_padded` constructors for automatic padding.

## Usage Examples

### Coordinate Transforms

```rust
use vastu_slam::core::{Pose2D, WorldPoint};

// Robot at (1, 2) facing 90° left
let robot_pose = Pose2D::new(1.0, 2.0, std::f32::consts::FRAC_PI_2);

// Point 1m ahead in robot frame
let sensor_point = WorldPoint::new(1.0, 0.0);

// Transform to world coordinates
let world_point = robot_pose.transform_point(sensor_point);
// Result: (1.0, 3.0) - 1m in +Y direction
```

### Processing Lidar Scans

```rust
use vastu_slam::core::{LidarScan, Pose2D};
use vastu_slam::core::simd::{PointCloud, transform_points};

// Create scan from raw data
let scan = LidarScan::new(ranges, angles, 0.15, 8.0);

// Convert to SIMD-friendly point cloud
let local_points = PointCloud::from_scan(&scan);

// Transform to world frame
let robot_pose = Pose2D::new(x, y, theta);
let world_points = transform_points(&local_points, &robot_pose);
```

### Cell Priority Updates

```rust
use vastu_slam::core::{Cell, CellType};

let mut cell = Cell::with_type(CellType::Floor);

// Wall overrides Floor
cell.observe_with_priority(CellType::Wall);
assert_eq!(cell.cell_type, CellType::Wall);

// Floor does NOT override Wall (lower priority)
cell.observe_with_priority(CellType::Floor);
assert_eq!(cell.cell_type, CellType::Wall);

// Bump overrides everything
cell.observe_with_priority(CellType::Bump);
assert_eq!(cell.cell_type, CellType::Bump);
```

## Performance Notes

1. **SIMD Alignment**: Always use `PointCloud::from_scan()` or `from_tuples()` - they auto-pad to 4-element boundaries.

2. **Avoid `distance()`**: Use `distance_squared()` when comparing distances (avoids sqrt).

3. **Pre-compute Rotations**: For repeated transforms, use `RotationMatrix4::from_pose()` once and reuse.

4. **Batch Operations**: Process points in batches using SIMD functions rather than iterating one-by-one.

## Thread Safety

All types in this module are `Send + Sync` (no interior mutability). The SIMD functions are stateless and thread-safe.
