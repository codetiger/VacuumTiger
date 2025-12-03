# Core Module

Foundation types and math primitives for DhruvaSLAM. This module has zero internal dependencies and is used by all higher layers.

## Overview

The core module provides:
- **Geometric types** - Points, poses, and transformations in 2D
- **Sensor types** - Laser scans, point clouds, IMU readings
- **Math utilities** - Angle normalization, interpolation
- **Uncertainty** - Covariance matrices for probabilistic reasoning

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
let distance = point.distance_to(&Point2D::origin());
let normalized = point.normalize();
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
// Composition: combine two transforms
let global_pose = robot_pose.compose(&local_offset);

// Inverse: compute reverse transform
let inverse = pose.inverse();

// Transform point: local to global coordinates
let global_point = pose.transform_point(&local_point);

// Inverse transform: global to local coordinates
let local_point = pose.inverse_transform_point(&global_point);

// Interpolation: smooth between poses
let mid_pose = Pose2D::interpolate(&start, &end, mid_time);
```

### Twist2D

2D velocity (linear and angular).

```rust
pub struct Twist2D {
    pub linear: f32,   // m/s
    pub angular: f32,  // rad/s
}
```

### Timestamped<T>

Generic wrapper adding microsecond timestamps to any type.

```rust
pub struct Timestamped<T> {
    pub data: T,
    pub timestamp_us: u64,
}

// Example
let stamped_pose = Timestamped::new(pose, timestamp_us);
```

### LaserScan

Lidar scan in polar coordinates.

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
```

### PointCloud2D

Collection of 2D points in Cartesian coordinates.

```rust
pub struct PointCloud2D {
    pub points: Vec<Point2D>,
}

// Example
let cloud = PointCloud2D::from_laser_scan(&scan);
let transformed = cloud.transform(&pose);
let centroid = cloud.centroid();
```

### ImuReading

IMU sensor data.

```rust
pub struct ImuReading {
    pub gyro: [f32; 3],   // Angular velocity [x, y, z] (rad/s)
    pub accel: [f32; 3],  // Linear acceleration [x, y, z] (m/s²)
}
```

### Covariance2D

3x3 uncertainty matrix for 2D pose (x, y, theta).

```rust
pub struct Covariance2D {
    pub data: [[f32; 3]; 3],
}

// Example
let cov = Covariance2D::from_diagonal(0.01, 0.01, 0.001);
let scaled = cov.scale(2.0);
```

## Math Utilities

### Angle Normalization

```rust
use dhruva_slam::core::math::normalize_angle;

// Normalize angle to [-π, π]
let normalized = normalize_angle(angle);
```

### Angle Difference

```rust
use dhruva_slam::core::math::angle_diff;

// Compute shortest angular difference
let diff = angle_diff(angle1, angle2);
```

### Angle Interpolation

```rust
use dhruva_slam::core::math::angle_lerp;

// Linear interpolation with angle wrapping
let mid_angle = angle_lerp(start_angle, end_angle, 0.5);
```

## File Structure

```
core/
├── mod.rs           # Module exports
├── types/
│   ├── mod.rs       # Type exports
│   ├── point.rs     # Point2D
│   ├── pose.rs      # Pose2D, Twist2D
│   ├── scan.rs      # LaserScan, PointCloud2D
│   ├── imu.rs       # ImuReading
│   ├── covariance.rs # Covariance2D
│   └── timestamped.rs # Timestamped<T>
└── math.rs          # Math utilities
```

## Usage Examples

### Pose Composition (Transform Chaining)

```rust
use dhruva_slam::core::types::{Pose2D, Point2D};

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

### Temporal Interpolation

```rust
let pose1 = Timestamped::new(Pose2D::new(0.0, 0.0, 0.0), 1000000);
let pose2 = Timestamped::new(Pose2D::new(1.0, 0.0, 0.5), 2000000);

// Interpolate at t=1.5s
let mid_pose = Pose2D::interpolate(
    &pose1.data,
    &pose2.data,
    1500000,  // target time
);
```

## Design Notes

- All angles are in **radians**
- All distances are in **meters**
- Pose theta is always normalized to **[-π, π]**
- Timestamps use **microseconds** (u64)
- Types implement `Clone`, `Copy`, `Debug`, `PartialEq`
- Serialization via `serde` (optional feature)
