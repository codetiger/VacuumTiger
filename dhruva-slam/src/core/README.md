# Core Module

Foundation types and math primitives for DhruvaSLAM. This module has zero internal dependencies and is used by all higher layers.

## Overview

The core module provides:
- **Geometric types** - Points, poses, and transformations in 2D
- **Sensor types** - Laser scans, point clouds
- **Math utilities** - Angle normalization, interpolation
- **SIMD primitives** - Vectorized operations for ARM NEON

## Module Structure

```
core/
├── mod.rs           # Module exports
├── math.rs          # Angle utilities, interpolation
├── simd/
│   ├── mod.rs       # SIMD exports
│   └── f32x4.rs     # 4-wide float vector (Float4)
└── types/
    ├── mod.rs       # Type exports
    ├── pose.rs      # Pose2D, Point2D
    ├── scan.rs      # LaserScan, PointCloud2D
    ├── odometry.rs  # Covariance2D
    ├── timestamped.rs # Timestamped<T>
    └── pose_tracker.rs # PoseTracker
```

## Types

### Point2D

A 2D point in Cartesian coordinates.

```rust
pub struct Point2D {
    pub x: f32,  // meters
    pub y: f32,  // meters
}

// Example
let point = Point2D::new(1.0, 2.0);
let distance = point.norm();             // Distance from origin
let angle = point.angle();               // Angle from origin
let normalized = point.normalize();      // Unit vector
```

### Pose2D

Robot pose in 2D space (position + orientation).

```rust
pub struct Pose2D {
    pub x: f32,      // meters
    pub y: f32,      // meters
    pub theta: f32,  // radians, normalized to [-π, π]
}
```

**Key operations:**

```rust
// Identity pose (origin, no rotation)
let origin = Pose2D::identity();

// Composition: chain two transforms
let global_pose = robot_pose.compose(&local_offset);

// Inverse: compute reverse transform
let inverse = pose.inverse();

// Transform point: local to global coordinates
let global_point = pose.transform_point(&local_point);

// Inverse transform: global to local coordinates
let local_point = pose.inverse_transform_point(&global_point);
```

### LaserScan

Lidar scan in polar coordinates (raw sensor data).

```rust
pub struct LaserScan {
    pub angle_min: f32,       // Start angle (radians)
    pub angle_max: f32,       // End angle (radians)
    pub angle_increment: f32, // Angular step (radians)
    pub range_min: f32,       // Minimum valid range (meters)
    pub range_max: f32,       // Maximum valid range (meters)
    pub ranges: Vec<f32>,     // Distance measurements (meters)
    pub intensities: Vec<f32>, // Signal strengths (optional)
}

// Create from SangamIO lidar data
let scan = LaserScan::from_lidar_scan(lidar_data);
```

### PointCloud2D

Collection of 2D points in Cartesian coordinates (processed scan).

```rust
pub struct PointCloud2D {
    xs: Vec<f32>,  // X coordinates
    ys: Vec<f32>,  // Y coordinates
}

// Create from points
let cloud = PointCloud2D::from_points(points);

// Transform to global frame
let global_cloud = cloud.transform(&robot_pose);

// Access points
let len = cloud.len();
let (x, y) = cloud.get(0).unwrap();

// Iteration
for i in 0..cloud.len() {
    let (x, y) = cloud.get(i).unwrap();
}
```

### Covariance2D

3x3 covariance matrix for 2D pose uncertainty (x, y, theta).

```rust
pub struct Covariance2D {
    data: [[f32; 3]; 3],
}

// Create diagonal covariance
let cov = Covariance2D::diagonal(0.01, 0.01, 0.001);

// Zero covariance (no uncertainty)
let zero = Covariance2D::zero();
```

### PoseTracker

Utility for tracking pose and computing deltas.

```rust
pub struct PoseTracker {
    pose: Pose2D,
    snapshot: Pose2D,
}

let mut tracker = PoseTracker::new();

// Update current pose
tracker.set(new_pose);

// Take snapshot (for computing deltas)
tracker.take_snapshot();

// Get delta since snapshot
let delta = tracker.delta_since_snapshot();
```

### Timestamped<T>

Generic wrapper adding microsecond timestamps to any type.

```rust
pub struct Timestamped<T> {
    pub data: T,
    pub timestamp_us: u64,
}

let stamped_pose = Timestamped::new(pose, timestamp_us);
```

## Math Utilities

### Angle Normalization

```rust
use crate::core::math::normalize_angle;

// Normalize angle to [-π, π]
let normalized = normalize_angle(angle);
```

### Angle Difference

```rust
use crate::core::math::angle_diff;

// Compute shortest angular difference
let diff = angle_diff(angle1, angle2);
```

## SIMD Module

The `simd` module provides `Float4`, a 4-wide float vector optimized for ARM NEON:

```rust
use crate::core::simd::Float4;

// Create vector
let a = Float4::splat(1.0);
let b = Float4::new(1.0, 2.0, 3.0, 4.0);

// Operations
let sum = a + b;
let diff = a - b;
let prod = a * b;
let sum_all = a.horizontal_sum();

// FMA (fused multiply-add) - single instruction on NEON
let fma = a.mul_add(b, c);  // a * b + c
```

**Note:** FMA is particularly useful for:
- Pose transformations (rotation matrices)
- Point cloud operations
- Correlation scoring

## Design Notes

- All angles are in **radians**
- All distances are in **meters**
- Pose theta is always normalized to **[-π, π]**
- Timestamps use **microseconds** (u64)
- Types implement `Clone`, `Copy`, `Debug`
- PointCloud2D uses SoA (struct of arrays) layout for cache efficiency

## Usage Example

### Pose Composition (Transform Chaining)

```rust
use crate::core::types::{Pose2D, Point2D};

// Robot at (1, 0) facing +Y
let robot_pose = Pose2D::new(1.0, 0.0, std::f32::consts::FRAC_PI_2);

// Sensor offset: 10cm forward
let sensor_offset = Pose2D::new(0.1, 0.0, 0.0);

// Global sensor pose
let sensor_pose = robot_pose.compose(&sensor_offset);
// Result: (1.0, 0.1, π/2)
```

### Point Transformation

```rust
// Point in robot frame (1m ahead)
let local_point = Point2D::new(1.0, 0.0);

// Transform to global frame
let global_point = robot_pose.transform_point(&local_point);

// Transform back to robot frame
let back_to_local = robot_pose.inverse_transform_point(&global_point);
```

### Processing a Scan

```rust
use crate::core::types::{LaserScan, PointCloud2D, Pose2D};

// Convert scan to point cloud
let cloud = PointCloud2D::from_laser_scan(&scan);

// Transform to global frame
let global_cloud = cloud.transform(&robot_pose);
```
