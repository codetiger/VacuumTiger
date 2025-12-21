# Core Module

Fundamental geometric types for 2D SLAM operations.

## Components

| File | Description |
|------|-------------|
| `point.rs` | `Point2D` - 2D point/vector with geometric operations |
| `pose.rs` | `Pose2D` - 2D position and orientation with frame transforms |
| `scan.rs` | `PolarScan` and `PointCloud2D` - Lidar data representations |
| `bounds.rs` | `Bounds` - Axis-aligned bounding box |
| `math.rs` | Common math utilities (angle normalization, etc.) |

## Coordinate Frame

All coordinates follow ROS REP-103 convention:
- **X-forward**: Positive X is ahead of the robot
- **Y-left**: Positive Y is to the robot's left
- **Rotation**: Counter-clockwise positive (radians)

## Key Types

### Point2D

2D point with full geometric operations:

```rust
let p = Point2D::new(3.0, 4.0);
p.length();           // Distance from origin (5.0)
p.normalized();       // Unit vector
p.rotate(angle);      // Rotate around origin
p.dot(other);         // Dot product
p.cross(other);       // 2D cross product (scalar)
```

### Pose2D

Robot pose with frame transformation:

```rust
let pose = Pose2D::new(x, y, theta);

// Transform point from robot frame to world frame
let world_pt = pose.transform_point(local_pt);

// Compose poses: pose_C = pose_A.compose(pose_B)
let combined = pose_a.compose(pose_b);

// Inverse transform
let local_pt = pose.inverse().transform_point(world_pt);
```

### PointCloud2D

Point cloud with SoA (Struct-of-Arrays) layout for SIMD efficiency:

```rust
let cloud = PointCloud2D::from_points(&points);

// Transform entire cloud
let world_cloud = cloud.transform(&pose);

// Access raw arrays for SIMD
let xs: &[f32] = cloud.xs();
let ys: &[f32] = cloud.ys();
```

### PolarScan

Raw lidar data in polar coordinates:

```rust
let mut scan = PolarScan::new();
scan.push(angle, distance, quality);

// Convert to Cartesian, filtering by quality/range
let cloud = scan.to_cartesian(min_quality, min_range, max_range);
```

## Data Layout

`PointCloud2D` uses Struct-of-Arrays for cache-friendly SIMD operations:

```
AoS (typical):              SoA (this implementation):
[Point{x,y}]                xs: [x0, x1, x2, x3, ...]
[Point{x,y}]    →           ys: [y0, y1, y2, y3, ...]
[Point{x,y}]
```

This enables 4-wide SIMD operations via `std::simd::f32x4`.
