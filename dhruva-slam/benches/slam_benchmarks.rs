//! Focused SLAM Benchmarks
//!
//! Benchmarks for high-level CPU-heavy SLAM operations:
//! - Scan matching (Correlative, ICP, P2L-ICP, Multi-resolution)
//! - Map integration (occupancy grid, ray tracing)
//! - Particle filter localization
//! - Point cloud SIMD operations
//!
//! Run with: `cargo bench`
//! View HTML reports in: `target/criterion/`

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use std::f32::consts::TAU;
use std::time::Duration;

use dhruva_slam::{
    CorrelativeConfig, CorrelativeMatcher, IcpConfig, LaserScan, MapIntegrator,
    MapIntegratorConfig, MultiResolutionConfig, MultiResolutionMatcher, OccupancyGrid,
    OccupancyGridConfig, ParticleFilter, ParticleFilterConfig, PointCloud2D, PointToLineIcp,
    PointToLineIcpConfig, PointToPointIcp, Pose2D, RayTracer, ScanMatcher,
};

// ============================================================================
// Test Fixtures
// ============================================================================

/// Create a room-shaped point cloud (L-shaped room).
/// This creates realistic geometry for scan matching benchmarks.
fn create_room_cloud(n_points: usize) -> PointCloud2D {
    let mut cloud = PointCloud2D::with_capacity(n_points);

    for i in 0..n_points {
        let angle = (i as f32 / n_points as f32) * TAU;
        let (sin_a, cos_a) = angle.sin_cos();

        // L-shaped room: longer in +x and +y directions
        let distance = if cos_a > 0.0 && sin_a > 0.0 {
            // Quadrant 1: corner at (3, 4)
            let dx = 3.0 / cos_a;
            let dy = 4.0 / sin_a;
            dx.min(dy)
        } else if cos_a < 0.0 && sin_a > 0.0 {
            // Quadrant 2: wall at x = -2
            (-2.0 / cos_a).min(4.0 / sin_a)
        } else if cos_a < 0.0 && sin_a < 0.0 {
            // Quadrant 3: corner at (-2, -2)
            (-2.0 / cos_a).min(-2.0 / sin_a)
        } else {
            // Quadrant 4: wall at y = -2
            (3.0 / cos_a).min(-2.0 / sin_a)
        };

        let distance = distance.clamp(0.5, 8.0);
        cloud.push_xy(distance * cos_a, distance * sin_a);
    }

    cloud
}

/// Create a benchmark scan (360 points)
fn create_benchmark_scan(n_points: usize) -> LaserScan {
    let angle_increment = TAU / n_points as f32;
    let ranges: Vec<f32> = (0..n_points)
        .map(|i| {
            let angle = i as f32 * angle_increment;
            let cos_a = angle.cos();
            let base = 3.0 + 2.0 * cos_a.abs();
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

/// Create a pre-populated occupancy grid for benchmarks
fn create_benchmark_grid() -> OccupancyGrid {
    let config = OccupancyGridConfig {
        resolution: 0.05,
        initial_width: 20.0,
        initial_height: 20.0,
        ..Default::default()
    };
    let mut grid = OccupancyGrid::new(config);

    // Add some walls to make it realistic
    let integrator = MapIntegrator::default();
    let scan = create_benchmark_scan(360);
    integrator.integrate_scan(&mut grid, &scan, &Pose2D::identity());

    grid
}

// ============================================================================
// Scan Matching Benchmarks (Highest Impact)
// ============================================================================

fn bench_scan_matching(c: &mut Criterion) {
    let mut group = c.benchmark_group("scan_matching");
    group.sample_size(20);
    group.measurement_time(Duration::from_secs(3));
    group.warm_up_time(Duration::from_secs(1));

    // Create test data
    let source = create_room_cloud(360);
    let target = source.transform(&Pose2D::new(0.05, 0.03, 0.02)); // Small offset

    // Correlative Matcher (most expensive)
    group.bench_function("correlative/match", |b| {
        let config = CorrelativeConfig {
            search_window_x: 0.3,
            search_window_y: 0.3,
            search_window_theta: 0.2,
            linear_resolution: 0.02,
            angular_resolution: 0.02,
            ..Default::default()
        };
        let initial_guess = Pose2D::identity();
        b.iter_batched(
            || CorrelativeMatcher::new(config.clone()),
            |mut matcher| {
                matcher.match_scans(
                    black_box(&source),
                    black_box(&target),
                    black_box(&initial_guess),
                )
            },
            criterion::BatchSize::SmallInput,
        )
    });

    // Point-to-Point ICP
    group.bench_function("icp/match", |b| {
        let config = IcpConfig {
            max_iterations: 20,
            max_correspondence_distance: 0.5,
            translation_epsilon: 1e-5,
            rotation_epsilon: 1e-5,
            ..Default::default()
        };
        let initial_guess = Pose2D::identity();
        b.iter_batched(
            || PointToPointIcp::new(config.clone()),
            |mut icp| {
                icp.match_scans(
                    black_box(&source),
                    black_box(&target),
                    black_box(&initial_guess),
                )
            },
            criterion::BatchSize::SmallInput,
        )
    });

    // Point-to-Line ICP
    group.bench_function("p2l_icp/match", |b| {
        let config = PointToLineIcpConfig {
            max_iterations: 20,
            max_correspondence_distance: 0.5,
            translation_epsilon: 1e-5,
            rotation_epsilon: 1e-5,
            ..Default::default()
        };
        let initial_guess = Pose2D::identity();
        b.iter_batched(
            || PointToLineIcp::new(config.clone()),
            |mut icp| {
                icp.match_scans(
                    black_box(&source),
                    black_box(&target),
                    black_box(&initial_guess),
                )
            },
            criterion::BatchSize::SmallInput,
        )
    });

    // Multi-resolution Matcher
    group.bench_function("multi_resolution/match", |b| {
        let config = MultiResolutionConfig::default();
        let initial_guess = Pose2D::identity();
        b.iter_batched(
            || MultiResolutionMatcher::new(config.clone()),
            |mut matcher| {
                matcher.match_scans(
                    black_box(&source),
                    black_box(&target),
                    black_box(&initial_guess),
                )
            },
            criterion::BatchSize::SmallInput,
        )
    });

    group.finish();
}

// ============================================================================
// Map Integration Benchmarks
// ============================================================================

fn bench_map_integration(c: &mut Criterion) {
    let mut group = c.benchmark_group("map_integration");
    group.sample_size(20);
    group.measurement_time(Duration::from_secs(3));
    group.warm_up_time(Duration::from_secs(1));

    let scan = create_benchmark_scan(360);
    let cloud = create_room_cloud(360);
    let pose = Pose2D::new(1.0, 1.0, 0.5);

    // Full scan integration
    group.bench_function("integrate_scan/360", |b| {
        let config = OccupancyGridConfig {
            resolution: 0.05,
            initial_width: 20.0,
            initial_height: 20.0,
            ..Default::default()
        };
        let integrator_config = MapIntegratorConfig::default();
        let integrator = MapIntegrator::new(integrator_config);

        b.iter_batched(
            || OccupancyGrid::new(config.clone()),
            |mut grid| {
                integrator.integrate_scan(black_box(&mut grid), black_box(&scan), black_box(&pose))
            },
            criterion::BatchSize::SmallInput,
        )
    });

    // Point cloud integration
    group.bench_function("integrate_cloud/360", |b| {
        let config = OccupancyGridConfig {
            resolution: 0.05,
            initial_width: 20.0,
            initial_height: 20.0,
            ..Default::default()
        };
        let integrator_config = MapIntegratorConfig::default();
        let integrator = MapIntegrator::new(integrator_config);

        b.iter_batched(
            || OccupancyGrid::new(config.clone()),
            |mut grid| {
                integrator.integrate_cloud(
                    black_box(&mut grid),
                    black_box(&cloud),
                    black_box(&pose),
                )
            },
            criterion::BatchSize::SmallInput,
        )
    });

    // Ray tracing only (360 rays)
    group.bench_function("ray_trace/360", |b| {
        let config = OccupancyGridConfig {
            resolution: 0.05,
            initial_width: 20.0,
            initial_height: 20.0,
            ..Default::default()
        };
        let tracer = RayTracer::default();

        b.iter_batched(
            || OccupancyGrid::new(config.clone()),
            |mut grid| {
                for i in 0..360 {
                    let angle = (i as f32 / 360.0) * TAU;
                    let (sin_a, cos_a) = angle.sin_cos();
                    let end_x = 5.0 * cos_a;
                    let end_y = 5.0 * sin_a;
                    tracer.trace_ray(&mut grid, 0.0, 0.0, end_x, end_y, true);
                }
            },
            criterion::BatchSize::SmallInput,
        )
    });

    group.finish();
}

// ============================================================================
// Particle Filter Benchmarks
// ============================================================================

fn bench_particle_filter(c: &mut Criterion) {
    let mut group = c.benchmark_group("particle_filter");
    group.sample_size(20);
    group.measurement_time(Duration::from_secs(3));
    group.warm_up_time(Duration::from_secs(1));

    let grid = create_benchmark_grid();
    let scan = create_benchmark_scan(180);

    // Motion model prediction (1000 particles)
    group.bench_function("predict/1000", |b| {
        let config = ParticleFilterConfig {
            num_particles: 1000,
            ..Default::default()
        };
        let odometry_delta = Pose2D::new(0.01, 0.0, 0.01);

        b.iter_batched(
            || ParticleFilter::new(config.clone(), Pose2D::identity(), &grid),
            |mut filter| {
                filter.predict(black_box(&odometry_delta));
            },
            criterion::BatchSize::SmallInput,
        )
    });

    // Sensor model update (1000 particles, 180 points)
    group.bench_function("update/1000", |b| {
        let config = ParticleFilterConfig {
            num_particles: 1000,
            ..Default::default()
        };

        b.iter_batched(
            || ParticleFilter::new(config.clone(), Pose2D::identity(), &grid),
            |mut filter| {
                filter.update(black_box(&scan), black_box(&grid));
            },
            criterion::BatchSize::SmallInput,
        )
    });

    // Full cycle: predict + update (update includes resampling internally)
    group.bench_function("full_cycle/1000", |b| {
        let config = ParticleFilterConfig {
            num_particles: 1000,
            resampling_threshold: 0.5,
            ..Default::default()
        };
        let odometry_delta = Pose2D::new(0.01, 0.0, 0.01);

        b.iter_batched(
            || ParticleFilter::new(config.clone(), Pose2D::identity(), &grid),
            |mut filter| {
                filter.predict(black_box(&odometry_delta));
                filter.update(black_box(&scan), black_box(&grid));
            },
            criterion::BatchSize::SmallInput,
        )
    });

    group.finish();
}

// ============================================================================
// Point Cloud SIMD Benchmarks (Verify Optimizations)
// ============================================================================

fn bench_point_cloud_simd(c: &mut Criterion) {
    let mut group = c.benchmark_group("point_cloud_simd");
    group.sample_size(50);
    group.measurement_time(Duration::from_secs(2));
    group.warm_up_time(Duration::from_secs(1));

    let cloud = create_room_cloud(360);
    let pose = Pose2D::new(1.0, 2.0, 0.5);

    // SIMD transform
    group.bench_function("transform/360", |b| {
        b.iter(|| black_box(&cloud).transform(black_box(&pose)))
    });

    // SIMD inverse transform
    group.bench_function("inverse_transform/360", |b| {
        b.iter(|| black_box(&cloud).inverse_transform(black_box(&pose)))
    });

    // SIMD bounds
    group.bench_function("bounds/360", |b| b.iter(|| black_box(&cloud).bounds()));

    // SIMD centroid
    group.bench_function("centroid/360", |b| b.iter(|| black_box(&cloud).centroid()));

    // Larger cloud (1000 points)
    let large_cloud = create_room_cloud(1000);
    group.bench_function("transform/1000", |b| {
        b.iter(|| black_box(&large_cloud).transform(black_box(&pose)))
    });

    group.finish();
}

// ============================================================================
// Main
// ============================================================================

criterion_group!(
    benches,
    bench_scan_matching,
    bench_map_integration,
    bench_particle_filter,
    bench_point_cloud_simd,
);

criterion_main!(benches);
