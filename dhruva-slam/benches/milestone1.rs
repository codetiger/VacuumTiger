//! Milestone 1 Benchmarks: Dead Reckoning
//!
//! Benchmarks for the core dead reckoning components:
//! - Math operations (angle normalization, diff, lerp)
//! - Pose operations (compose, inverse, transform)
//! - Wheel odometry
//! - Complementary filter
//! - Scan preprocessing pipeline
//!
//! Run with: `cargo bench`
//! View HTML reports in: `target/criterion/`

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use std::f32::consts::{PI, TAU};

use dhruva_slam::{
    AngularDownsampler, AngularDownsamplerConfig, ComplementaryConfig, ComplementaryFilter,
    LaserScan, OutlierFilter, OutlierFilterConfig, Point2D, PointCloud2D, Pose2D,
    PreprocessorConfig, RangeFilter, RangeFilterConfig, ScanConverter, ScanPreprocessor,
    WheelOdometry, WheelOdometryConfig,
    math::{angle_diff, angle_lerp, normalize_angle},
};

// ============================================================================
// Test Fixtures
// ============================================================================

/// Standard wheel odometry config matching CRL-200S
fn benchmark_odom_config() -> WheelOdometryConfig {
    WheelOdometryConfig {
        ticks_per_meter: 1000.0,
        wheel_base: 0.17,
    }
}

/// Standard complementary filter config
fn benchmark_filter_config() -> ComplementaryConfig {
    ComplementaryConfig {
        alpha: 0.98,
        gyro_scale: 0.001,
        gyro_bias_z: 0.0,
    }
}

/// Create a realistic 360-point scan (Delta-2D typical)
fn create_benchmark_scan(n_points: usize) -> LaserScan {
    let angle_increment = TAU / n_points as f32;
    let ranges: Vec<f32> = (0..n_points)
        .map(|i| {
            // Simulate a room with walls at varying distances
            let angle = i as f32 * angle_increment;
            let base = 3.0 + 2.0 * angle.cos().abs();
            // Add some noise
            base + 0.05 * ((i as f32 * 0.1).sin())
        })
        .collect();

    LaserScan::new(
        0.0,
        TAU - angle_increment,
        angle_increment,
        0.15,
        12.0,
        ranges,
    )
}

/// Create a scan with some invalid points for filtering tests
fn create_noisy_scan(n_points: usize) -> LaserScan {
    let angle_increment = TAU / n_points as f32;
    let ranges: Vec<f32> = (0..n_points)
        .map(|i| {
            // Every 10th point is invalid, every 20th is an outlier
            if i % 10 == 0 {
                0.0 // Invalid
            } else if i % 20 == 5 {
                15.0 // Out of range (outlier)
            } else {
                5.0 + 0.1 * (i as f32 * 0.1).sin()
            }
        })
        .collect();

    LaserScan::new(
        0.0,
        TAU - angle_increment,
        angle_increment,
        0.15,
        12.0,
        ranges,
    )
}

// ============================================================================
// Group 1: Math Operations
// ============================================================================

fn bench_math(c: &mut Criterion) {
    let mut group = c.benchmark_group("math");

    // Single angle normalization
    group.bench_function("normalize_angle", |b| {
        let angle = 7.5; // ~2.4π, needs wrapping
        b.iter(|| normalize_angle(black_box(angle)))
    });

    // Angle difference
    group.bench_function("angle_diff", |b| {
        let a = PI - 0.1;
        let b_angle = -PI + 0.1;
        b.iter(|| angle_diff(black_box(a), black_box(b_angle)))
    });

    // Angular interpolation
    group.bench_function("angle_lerp", |b| {
        let a = 0.0;
        let b_angle = PI / 2.0;
        let t = 0.5;
        b.iter(|| angle_lerp(black_box(a), black_box(b_angle), black_box(t)))
    });

    // Batch normalization (1000 angles)
    group.throughput(Throughput::Elements(1000));
    group.bench_function("normalize_angle_batch_1000", |b| {
        let angles: Vec<f32> = (0..1000).map(|i| i as f32 * 0.1 - 50.0).collect();
        b.iter(|| {
            angles
                .iter()
                .map(|&a| normalize_angle(black_box(a)))
                .collect::<Vec<_>>()
        })
    });

    group.finish();
}

// ============================================================================
// Group 2: Pose Operations
// ============================================================================

fn bench_pose(c: &mut Criterion) {
    let mut group = c.benchmark_group("pose");

    let pose_a = Pose2D::new(1.0, 2.0, 0.5);
    let pose_b = Pose2D::new(0.1, 0.0, 0.1);
    let point = Point2D::new(1.0, 0.0);

    // SE(2) composition
    group.bench_function("compose", |b| {
        b.iter(|| black_box(&pose_a).compose(black_box(&pose_b)))
    });

    // Pose inverse
    group.bench_function("inverse", |b| b.iter(|| black_box(&pose_a).inverse()));

    // Single point transform
    group.bench_function("transform_point", |b| {
        b.iter(|| black_box(&pose_a).transform_point(black_box(&point)))
    });

    // Inverse transform
    group.bench_function("inverse_transform_point", |b| {
        b.iter(|| black_box(&pose_a).inverse_transform_point(black_box(&point)))
    });

    // Batch transform (360 points - typical scan)
    group.throughput(Throughput::Elements(360));
    group.bench_function("transform_batch_360", |b| {
        let points: Vec<Point2D> = (0..360)
            .map(|i| {
                let angle = i as f32 * TAU / 360.0;
                Point2D::new(5.0 * angle.cos(), 5.0 * angle.sin())
            })
            .collect();
        let cloud = PointCloud2D::from_points(points);

        b.iter(|| black_box(&cloud).transform(black_box(&pose_a)))
    });

    group.finish();
}

// ============================================================================
// Group 3: Wheel Odometry
// ============================================================================

fn bench_wheel_odometry(c: &mut Criterion) {
    let mut group = c.benchmark_group("wheel_odometry");

    let config = benchmark_odom_config();

    // Straight line motion (both wheels equal)
    group.bench_function("straight", |b| {
        let mut odom = WheelOdometry::new(config);
        odom.update(0, 0); // Initialize
        let mut left = 0u16;
        let mut right = 0u16;

        b.iter(|| {
            left = left.wrapping_add(100);
            right = right.wrapping_add(100);
            odom.update(black_box(left), black_box(right))
        })
    });

    // Arc motion (triggers sin/cos calculations)
    group.bench_function("arc", |b| {
        let mut odom = WheelOdometry::new(config);
        odom.update(0, 0); // Initialize
        let mut left = 0u16;
        let mut right = 0u16;

        b.iter(|| {
            left = left.wrapping_add(80);
            right = right.wrapping_add(120);
            odom.update(black_box(left), black_box(right))
        })
    });

    // Wraparound handling
    group.bench_function("wraparound", |b| {
        let mut odom = WheelOdometry::new(config);
        odom.update(65500, 65500); // Near max
        let mut left = 65500u16;
        let mut right = 65500u16;

        b.iter(|| {
            left = left.wrapping_add(100);
            right = right.wrapping_add(100);
            odom.update(black_box(left), black_box(right))
        })
    });

    group.finish();
}

// ============================================================================
// Group 4: Complementary Filter
// ============================================================================

fn bench_complementary_filter(c: &mut Criterion) {
    let mut group = c.benchmark_group("complementary_filter");

    let config = benchmark_filter_config();

    // Update with raw gyro
    group.bench_function("update_raw", |b| {
        let mut filter = ComplementaryFilter::new(config);
        let encoder_delta = Pose2D::new(0.01, 0.0, 0.01);
        let mut timestamp = 0u64;

        b.iter(|| {
            timestamp += 2000; // 2ms = 500Hz
            filter.update(
                black_box(encoder_delta),
                black_box(50i16),
                black_box(timestamp),
            )
        })
    });

    // Update with pre-calibrated gyro
    group.bench_function("update_calibrated", |b| {
        let mut filter = ComplementaryFilter::new(config);
        let encoder_delta = Pose2D::new(0.01, 0.0, 0.01);
        let mut timestamp = 0u64;

        b.iter(|| {
            timestamp += 2000;
            filter.update_calibrated(
                black_box(encoder_delta),
                black_box(0.05f32),
                black_box(timestamp),
            )
        })
    });

    // Sequence of 500 updates (1 second @ 500Hz)
    group.throughput(Throughput::Elements(500));
    group.bench_function("update_sequence_500", |b| {
        b.iter(|| {
            let mut filter = ComplementaryFilter::new(config);
            let encoder_delta = Pose2D::new(0.001, 0.0, 0.001);

            for i in 0..500 {
                let timestamp = (i as u64 + 1) * 2000;
                filter.update(encoder_delta, 50, timestamp);
            }
            filter.pose()
        })
    });

    group.finish();
}

// ============================================================================
// Group 5: Scan Preprocessing Pipeline
// ============================================================================

fn bench_preprocessing(c: &mut Criterion) {
    let mut group = c.benchmark_group("preprocessing");

    // Test with different scan sizes
    for size in [180, 360, 720] {
        let scan = create_benchmark_scan(size);
        let noisy_scan = create_noisy_scan(size);

        group.throughput(Throughput::Elements(size as u64));

        // Range filter
        group.bench_with_input(
            BenchmarkId::new("range_filter", size),
            &noisy_scan,
            |b, scan| {
                let filter = RangeFilter::new(RangeFilterConfig::default());
                b.iter(|| filter.apply(black_box(scan)))
            },
        );

        // Outlier filter
        group.bench_with_input(
            BenchmarkId::new("outlier_filter", size),
            &scan,
            |b, scan| {
                let filter = OutlierFilter::new(OutlierFilterConfig::default());
                b.iter(|| filter.apply(black_box(scan)))
            },
        );

        // Downsampler
        group.bench_with_input(BenchmarkId::new("downsampler", size), &scan, |b, scan| {
            let downsampler = AngularDownsampler::new(AngularDownsamplerConfig::default());
            b.iter(|| downsampler.apply(black_box(scan)))
        });

        // Scan converter
        group.bench_with_input(
            BenchmarkId::new("scan_converter", size),
            &scan,
            |b, scan| b.iter(|| ScanConverter::to_point_cloud(black_box(scan))),
        );

        // Full pipeline
        group.bench_with_input(
            BenchmarkId::new("full_pipeline", size),
            &noisy_scan,
            |b, scan| {
                let preprocessor = ScanPreprocessor::new(PreprocessorConfig::default());
                b.iter(|| preprocessor.process(black_box(scan)))
            },
        );
    }

    group.finish();
}

// ============================================================================
// Group 6: Point Cloud Operations
// ============================================================================

fn bench_point_cloud(c: &mut Criterion) {
    let mut group = c.benchmark_group("point_cloud");

    // Create test clouds of varying sizes
    for size in [100, 360, 1000] {
        let points: Vec<Point2D> = (0..size)
            .map(|i| {
                let angle = i as f32 * TAU / size as f32;
                Point2D::new(5.0 * angle.cos(), 5.0 * angle.sin())
            })
            .collect();
        let cloud = PointCloud2D::from_points(points);

        group.throughput(Throughput::Elements(size as u64));

        // Bounds calculation
        group.bench_with_input(BenchmarkId::new("bounds", size), &cloud, |b, cloud| {
            b.iter(|| black_box(cloud).bounds())
        });

        // Centroid calculation
        group.bench_with_input(BenchmarkId::new("centroid", size), &cloud, |b, cloud| {
            b.iter(|| black_box(cloud).centroid())
        });

        // Transform (creates new cloud)
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        group.bench_with_input(BenchmarkId::new("transform", size), &cloud, |b, cloud| {
            b.iter(|| black_box(cloud).transform(black_box(&pose)))
        });

        // Transform in place
        group.bench_with_input(
            BenchmarkId::new("transform_mut", size),
            &cloud,
            |b, cloud| {
                b.iter_batched(
                    || cloud.clone(),
                    |mut c| c.transform_mut(black_box(&pose)),
                    criterion::BatchSize::SmallInput,
                )
            },
        );
    }

    group.finish();
}

// ============================================================================
// Main
// ============================================================================

criterion_group!(
    benches,
    bench_math,
    bench_pose,
    bench_wheel_odometry,
    bench_complementary_filter,
    bench_preprocessing,
    bench_point_cloud,
);

criterion_main!(benches);
