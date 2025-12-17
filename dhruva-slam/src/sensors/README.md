# Sensors Module

Sensor data processing: odometry estimation and lidar preprocessing.

## Overview

The sensors module provides:
- **Odometry** - Wheel encoder and IMU fusion for pose estimation
- **Preprocessing** - LiDAR scan filtering and conversion
- **Calibration** - Gyroscope bias estimation

## Module Structure

```
sensors/
├── mod.rs              # Module exports
├── calibration.rs      # Gyro bias estimation
├── odometry/
│   ├── mod.rs          # Exports
│   ├── wheel_odometry.rs   # Encoder-only odometry
│   ├── complementary.rs    # Complementary filter (encoder + gyro)
│   ├── eskf.rs             # Error-State Kalman Filter
│   ├── mahony.rs           # Mahony AHRS (recommended)
│   ├── dynamic.rs          # Runtime algorithm selection
│   ├── fused_tracker.rs    # SLAM + odometry fusion
│   ├── imu_analysis.rs     # IMU analysis utilities
│   ├── calibration.rs      # Odometry calibration
│   └── evaluator.rs        # Drift metrics
└── preprocessing/
    ├── mod.rs              # Exports, ScanPreprocessor
    ├── range_filter.rs     # Min/max range filtering
    ├── outlier_filter.rs   # Statistical outlier removal
    ├── downsampler.rs      # Angular downsampling
    ├── converter.rs        # Polar to Cartesian
    └── dynamic.rs          # Runtime filter selection
```

## Odometry

Convert wheel encoders and IMU readings into pose estimates.

### DynOdometry (Runtime Selection)

The `DynOdometry` type allows selecting the algorithm at runtime:

```rust
use crate::sensors::odometry::{DynOdometry, DynOdometryConfig, OdometryType};

let config = DynOdometryConfig {
    algorithm: OdometryType::Mahony,  // From config
    ticks_per_meter: 4464.0,
    wheel_base: 0.233,
    // ... other parameters
};

let mut odom = DynOdometry::new(config);

// Process sensor readings
if let Some(delta) = odom.process(left_ticks, right_ticks, gyro_z, timestamp_us) {
    // delta is pose change since last output
}
```

Available algorithms:
- `OdometryType::Wheel` - Encoders only
- `OdometryType::Complementary` - Simple encoder + gyro fusion
- `OdometryType::Eskf` - Error-State Kalman Filter
- `OdometryType::Mahony` - **Recommended** - AHRS with auto gyro calibration

### WheelOdometry

Converts encoder tick deltas to pose deltas using differential drive kinematics.

```rust
use crate::sensors::odometry::{WheelOdometry, WheelOdometryConfig};

let config = WheelOdometryConfig {
    ticks_per_meter: 4464.0,  // Encoder calibration
    wheel_base: 0.233,        // Distance between wheels (m)
};

let mut odom = WheelOdometry::new(config);

// First call initializes state
odom.update(left_ticks, right_ticks);

// Subsequent calls return pose delta
if let Some(delta) = odom.update(left_ticks, right_ticks) {
    println!("Moved: x={:.3}m, y={:.3}m, theta={:.3}rad",
        delta.x, delta.y, delta.theta);
}
```

**Differential drive model:**
```
v_left = (left_ticks - prev_left) / ticks_per_meter
v_right = (right_ticks - prev_right) / ticks_per_meter

v_linear = (v_left + v_right) / 2
v_angular = (v_right - v_left) / wheel_base

dx = v_linear * cos(theta)
dy = v_linear * sin(theta)
dtheta = v_angular
```

### ComplementaryFilter

Fuses wheel encoder heading with gyroscope for improved angular estimation.

```rust
use crate::sensors::odometry::{ComplementaryFilter, ComplementaryConfig};

let config = ComplementaryConfig {
    alpha: 0.8,              // Gyro weight (0-1)
    gyro_scale: 0.0001745,   // Raw units to rad/s
    gyro_bias_z: 0.0,        // Z-axis bias (auto-calibrated)
};

let mut filter = ComplementaryFilter::new(config);

// Fuse encoder delta with gyro reading
let fused_delta = filter.update(&encoder_delta, gyro_z_raw, dt_secs);
```

**Filter equation:**
```
theta = alpha * gyro_theta + (1 - alpha) * encoder_theta
```

### MahonyAhrs

Mahony Attitude and Heading Reference System with automatic gyroscope bias calibration.

```rust
use crate::sensors::odometry::{MahonyAhrs, MahonyConfig, RawImuData};

let config = MahonyConfig {
    kp: 2.0,                        // Proportional gain
    ki: 0.005,                      // Integral gain
    gyro_scale: 0.0001745,          // Raw to rad/s
    gyro_bias_calibration_samples: 1500,  // Samples for auto-calibration
};

let mut ahrs = MahonyAhrs::new(config);

// Create IMU reading (gyro_z for yaw, optional tilt)
let imu = RawImuData::new(gyro_raw, tilt_raw);

// Update returns (roll, pitch, yaw) in radians
let (roll, pitch, yaw) = ahrs.update(&imu, timestamp_us);

// Check calibration status
if ahrs.is_calibrated() {
    println!("Gyro bias: {:.4} rad/s", ahrs.gyro_bias());
}
```

**Advantages:**
- Automatic gyro bias calibration at startup
- Adaptive gain based on motion
- Low CPU overhead

### Eskf

Error-State Kalman Filter for production-quality sensor fusion with uncertainty estimation.

```rust
use crate::sensors::odometry::{Eskf, EskfConfig};

let config = EskfConfig {
    process_noise_xy: 0.01,
    process_noise_theta: 0.001,
    measurement_noise_encoder: 0.05,
    measurement_noise_gyro: 0.01,
};

let mut eskf = Eskf::new(config);

// Predict step (from encoders)
eskf.predict(&encoder_delta, dt_secs);

// Update step (from gyro)
eskf.update_gyro(gyro_z, dt_secs);

// Get pose with covariance
let pose = eskf.pose();
let covariance = eskf.covariance();
```

**State vector:** `[x, y, theta, v_left, v_right]`

## Preprocessing

Filter and convert LiDAR scans for SLAM.

### ScanPreprocessor

Complete preprocessing pipeline:

```rust
use crate::sensors::preprocessing::{ScanPreprocessor, PreprocessorConfig};

let config = PreprocessorConfig {
    min_range: 0.15,          // Robot radius
    max_range: 8.0,           // Lidar spec
    target_points: 180,       // Downsample target
    outlier_threshold: 2.0,   // Std dev for outlier removal
};

let preprocessor = ScanPreprocessor::new(config);

// Full pipeline: range filter → outlier filter → downsample → convert
let cloud = preprocessor.process(&laser_scan);
```

### Pipeline Stages

```
LaserScan (polar, 360+ points)
    │
    ▼
┌───────────────┐
│  RangeFilter  │  Remove invalid/out-of-range points
└───────┬───────┘
        ▼
┌───────────────┐
│ OutlierFilter │  Statistical outlier removal
└───────┬───────┘
        ▼
┌─────────────────────┐
│ AngularDownsampler  │  Reduce to target point count
└───────┬─────────────┘
        ▼
┌───────────────┐
│ ScanConverter │  Polar to Cartesian
└───────┬───────┘
        ▼
PointCloud2D (Cartesian, ~180 points)
```

### RangeFilter

Removes points with invalid or out-of-range distances:

```rust
use crate::sensors::preprocessing::{RangeFilter, RangeFilterConfig};

let config = RangeFilterConfig {
    min_range: 0.15,       // Minimum valid distance (m)
    max_range: 8.0,        // Maximum valid distance (m)
    invalidate_zero: true, // Treat 0.0 as invalid
};

let filter = RangeFilter::new(config);
let filtered = filter.filter(&scan);
```

### OutlierFilter

Statistical outlier removal based on range distribution:

```rust
use crate::sensors::preprocessing::{OutlierFilter, OutlierFilterConfig};

let config = OutlierFilterConfig {
    outlier_threshold: 2.0,  // Standard deviations
};

let filter = OutlierFilter::new(config);
let cleaned = filter.filter(&scan);
```

### AngularDownsampler

Reduces point count while preserving scan structure:

```rust
use crate::sensors::preprocessing::{AngularDownsampler, DownsamplerConfig, DownsampleMethod};

let config = DownsamplerConfig {
    target_points: 180,
    method: DownsampleMethod::AngularBinning,
};

let downsampler = AngularDownsampler::new(config);
let reduced = downsampler.downsample(&scan);
```

**Methods:**
- `AngularBinning` - Groups points into angular bins, keeps mean per bin
- `MaxRange` - Keeps point with maximum range in each bin

### ScanConverter

Converts polar LaserScan to Cartesian PointCloud2D with optional radial offset:

```rust
use crate::sensors::preprocessing::ScanConverter;

let cloud = ScanConverter::to_point_cloud_with_offset(&laser_scan, 0.0);
```

## Calibration

### GyroCalibrator

Estimates gyroscope bias during stationary periods:

```rust
use crate::sensors::calibration::GyroCalibrator;

let mut calibrator = GyroCalibrator::new(1500);  // Samples needed

// Feed samples while robot is stationary
calibrator.add_sample(gyro_z_raw);

if calibrator.is_calibrated() {
    let bias = calibrator.bias();
    println!("Gyro bias: {:.4} units", bias);
}
```

## CRL-200S Calibration

Calibrated values for the CRL-200S robot:

```toml
[odometry]
algorithm = "mahony"
ticks_per_meter = 4464.0
wheel_base = 0.233
gyro_scale = 0.0001745  # 0.01°/s per unit → rad/s

[preprocessing]
min_range = 0.15        # 15cm minimum (robot radius)
max_range = 8.0         # 8m maximum (lidar spec)
target_points = 180     # Downsample to 180 points
```

## Algorithm Selection

| Algorithm | Use Case | CPU | Notes |
|-----------|----------|-----|-------|
| `Wheel` | Testing/baseline | Very Low | No gyro needed |
| `Complementary` | Simple fusion | Low | Requires gyro bias |
| `Eskf` | Uncertainty tracking | Medium | Full covariance |
| `Mahony` | **Production** | Low | Auto gyro calibration |

## Performance

| Operation | Time | Notes |
|-----------|------|-------|
| WheelOdometry.update | <1μs | Single tick processing |
| ComplementaryFilter.update | <1μs | Simple fusion |
| MahonyAhrs.update | ~2μs | AHRS with bias tracking |
| Eskf.predict + update | ~5μs | Full Kalman filter |
| ScanPreprocessor.process | ~50μs | 360→180 points |
