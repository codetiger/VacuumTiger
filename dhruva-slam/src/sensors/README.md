# Sensors Module

Raw sensor data processing and fusion. This module transforms noisy sensor inputs into clean estimates suitable for SLAM algorithms.

## Overview

The sensors module provides:
- **Odometry** - Wheel encoder and gyroscope fusion for pose estimation
- **Preprocessing** - Lidar scan filtering and conversion

## Odometry

Combines wheel encoders with IMU gyroscope to estimate robot motion.

### WheelOdometry

Converts encoder tick deltas to pose deltas using differential drive kinematics.

```rust
use dhruva_slam::sensors::odometry::WheelOdometry;

let config = WheelOdometryConfig {
    ticks_per_meter: 4464.0,  // Encoder calibration
    wheel_base: 0.233,        // Distance between wheels (m)
};

let mut odom = WheelOdometry::new(config);

// Process encoder readings
if let Some(delta) = odom.update(left_ticks, right_ticks, timestamp_us) {
    // delta is the Pose2D change since last update
}
```

**Parameters:**
| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `ticks_per_meter` | Encoder ticks per meter traveled | 4464 (CRL-200S) |
| `wheel_base` | Distance between wheel centers | 0.233m |

### ComplementaryFilter

Fuses wheel encoder heading with gyroscope for improved angular estimation.

```rust
use dhruva_slam::sensors::odometry::ComplementaryFilter;

let config = ComplementaryFilterConfig {
    alpha: 0.8,              // Gyro weight (0-1)
    gyro_scale: 0.0001745,   // Raw units to rad/s
    gyro_bias_z: 0.0,        // Z-axis bias
};

let mut filter = ComplementaryFilter::new(config);

// Process with gyro reading
let fused_delta = filter.update(&encoder_delta, gyro_z_raw, dt);
```

**Filter equation:**
```
theta = alpha * gyro_theta + (1 - alpha) * encoder_theta
```

**Parameters:**
| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `alpha` | Gyro weight (higher = trust gyro more) | 0.8 |
| `gyro_scale` | Conversion from raw to rad/s | 0.0001745 |
| `gyro_bias_z` | Static gyro offset (auto-calibrated) | 0.0 |

### ESKF (Error-State Kalman Filter)

Production-quality sensor fusion with uncertainty estimation.

```rust
use dhruva_slam::sensors::odometry::Eskf;

let config = EskfConfig {
    process_noise_xy: 0.01,
    process_noise_theta: 0.001,
    measurement_noise_encoder: 0.05,
    measurement_noise_gyro: 0.01,
};

let mut eskf = Eskf::new(config);

// Predict step (from encoders)
eskf.predict(&encoder_delta, dt);

// Update step (from gyro)
eskf.update_gyro(gyro_z, dt);

let pose = eskf.pose();
let covariance = eskf.covariance();
```

**State vector:** `[x, y, theta, v_left, v_right]`

### OdometryEvaluator

Calibration and validation tool for odometry parameters.

```rust
use dhruva_slam::sensors::odometry::OdometryEvaluator;

let evaluator = OdometryEvaluator::new();

// Run evaluation on test scenario
let result = evaluator.evaluate(&scenario, &config);

println!("Distance error: {}%", result.distance_error_percent);
println!("Angular error: {} rad", result.angular_error);
```

## Preprocessing

Lidar scan filtering pipeline to prepare data for scan matching.

### Processing Pipeline

```
LaserScan (polar)
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
│ AngularDownsampler  │  Reduce point count
└───────┬─────────────┘
        ▼
┌───────────────┐
│ ScanConverter │  Polar to Cartesian
└───────┬───────┘
        ▼
PointCloud2D (Cartesian)
```

### RangeFilter

Removes points with invalid or out-of-range distances.

```rust
use dhruva_slam::sensors::preprocessing::RangeFilter;

let config = RangeFilterConfig {
    min_range: 0.15,       // Minimum valid distance (m)
    max_range: 8.0,        // Maximum valid distance (m)
    invalidate_zero: true, // Treat 0.0 as invalid
};

let filter = RangeFilter::new(config);
let filtered = filter.filter(&scan);
```

### OutlierFilter

Statistical outlier removal based on range distribution.

```rust
use dhruva_slam::sensors::preprocessing::OutlierFilter;

let config = OutlierFilterConfig {
    outlier_threshold: 2.0,  // Standard deviations
};

let filter = OutlierFilter::new(config);
let cleaned = filter.filter(&scan);
```

### AngularDownsampler

Reduces point count while preserving scan structure.

```rust
use dhruva_slam::sensors::preprocessing::{AngularDownsampler, DownsampleMethod};

let config = AngularDownsamplerConfig {
    target_points: 180,                    // Output point count
    method: DownsampleMethod::AngularBinning,
};

let downsampler = AngularDownsampler::new(config);
let reduced = downsampler.downsample(&scan);
```

**Methods:**
- `AngularBinning` - Groups points into angular bins, keeps one per bin
- `MaxRange` - Keeps point with maximum range in each bin

### ScanConverter

Converts polar LaserScan to Cartesian PointCloud2D.

```rust
use dhruva_slam::sensors::preprocessing::ScanConverter;

let cloud = ScanConverter::to_point_cloud(&scan);
```

### ScanPreprocessor

Complete pipeline orchestrator.

```rust
use dhruva_slam::sensors::preprocessing::ScanPreprocessor;

let config = ScanPreprocessorConfig {
    range_filter: RangeFilterConfig { ... },
    outlier_filter: Some(OutlierFilterConfig { ... }),
    downsampler: Some(AngularDownsamplerConfig { ... }),
};

let preprocessor = ScanPreprocessor::new(config);

// Full pipeline with downsampling
let cloud = preprocessor.process(&scan);

// Without downsampling (for mapping)
let full_cloud = preprocessor.process_full_resolution(&scan);

// Minimal processing (range filter only)
let minimal = preprocessor.process_minimal(&scan);
```

### DynamicFilter

Adaptive filtering with runtime statistics.

```rust
use dhruva_slam::sensors::preprocessing::DynamicFilter;

let filter = DynamicFilter::new(config);
let (cloud, stats) = filter.filter(&scan);

println!("Points before: {}", stats.input_count);
println!("Points after: {}", stats.output_count);
println!("Rejection rate: {}%", stats.rejection_rate * 100.0);
```

## File Structure

```
sensors/
├── mod.rs              # Module exports
├── odometry/
│   ├── mod.rs          # Odometry exports
│   ├── wheel.rs        # WheelOdometry
│   ├── complementary.rs # ComplementaryFilter
│   ├── eskf.rs         # Error-State Kalman Filter
│   └── evaluator.rs    # OdometryEvaluator
└── preprocessing/
    ├── mod.rs          # Preprocessing exports
    ├── range_filter.rs
    ├── outlier_filter.rs
    ├── downsampler.rs
    ├── converter.rs
    ├── preprocessor.rs
    └── dynamic.rs
```

## Usage Example

Complete odometry pipeline:

```rust
use dhruva_slam::sensors::odometry::{WheelOdometry, ComplementaryFilter};
use dhruva_slam::sensors::preprocessing::ScanPreprocessor;

// Setup odometry
let mut wheel_odom = WheelOdometry::new(wheel_config);
let mut filter = ComplementaryFilter::new(filter_config);

// Setup preprocessing
let preprocessor = ScanPreprocessor::new(preprocess_config);

// Process sensor data
fn process_sensors(
    left_ticks: u16,
    right_ticks: u16,
    gyro_z: i16,
    scan: &LaserScan,
    timestamp_us: u64,
) -> (Option<Pose2D>, PointCloud2D) {
    // Compute odometry
    let odom_delta = wheel_odom.update(left_ticks, right_ticks, timestamp_us)
        .map(|delta| filter.update(&delta, gyro_z, dt));

    // Preprocess lidar
    let cloud = preprocessor.process(scan);

    (odom_delta, cloud)
}
```

## CRL-200S Calibration

Calibrated values for the CRL-200S robot:

```toml
[odometry]
ticks_per_meter = 4464.0
wheel_base = 0.233
alpha = 0.8
gyro_scale = 0.0001745  # 0.01°/s per unit → rad/s
gyro_bias_z = 0.0       # Auto-calibrated at startup

[preprocessing]
min_range = 0.15        # 15cm minimum (robot radius)
max_range = 8.0         # 8m maximum (lidar spec)
target_points = 180     # Downsample to 180 points
```

## Performance

| Operation | Time | Notes |
|-----------|------|-------|
| WheelOdometry.update | <1μs | Single tick processing |
| ComplementaryFilter.update | <1μs | Simple fusion |
| ESKF.predict + update | ~5μs | Full Kalman filter |
| ScanPreprocessor.process | ~50μs | 360→180 points |
