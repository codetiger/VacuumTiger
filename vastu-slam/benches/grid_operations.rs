//! Benchmark grid operations performance.

use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use std::f32::consts::PI;
use vastu_slam::{LidarScan, MapConfig, OccupancyGridMap, Pose2D};

/// Create a room scan for benchmarking.
fn room_scan(
    room_width: f32,
    room_height: f32,
    robot_x: f32,
    robot_y: f32,
    num_points: usize,
) -> LidarScan {
    let angle_increment = 2.0 * PI / num_points as f32;
    let max_range = (room_width * room_width + room_height * room_height).sqrt();

    let mut ranges = Vec::with_capacity(num_points);
    let mut angles = Vec::with_capacity(num_points);

    for i in 0..num_points {
        let angle = i as f32 * angle_increment - PI;
        angles.push(angle);

        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let mut range = max_range;

        // Simple room ray casting
        if cos_a > 0.0 {
            let t = (room_width - robot_x) / cos_a;
            if t > 0.0 && t < range {
                let y = robot_y + t * sin_a;
                if y >= 0.0 && y <= room_height {
                    range = t;
                }
            }
        }
        if cos_a < 0.0 {
            let t = -robot_x / cos_a;
            if t > 0.0 && t < range {
                let y = robot_y + t * sin_a;
                if y >= 0.0 && y <= room_height {
                    range = t;
                }
            }
        }
        if sin_a > 0.0 {
            let t = (room_height - robot_y) / sin_a;
            if t > 0.0 && t < range {
                let x = robot_x + t * cos_a;
                if x >= 0.0 && x <= room_width {
                    range = t;
                }
            }
        }
        if sin_a < 0.0 {
            let t = -robot_y / sin_a;
            if t > 0.0 && t < range {
                let x = robot_x + t * cos_a;
                if x >= 0.0 && x <= room_width {
                    range = t;
                }
            }
        }

        ranges.push(range.min(max_range));
    }

    LidarScan::new(ranges, angles, 0.15, max_range)
}

fn bench_map_update(c: &mut Criterion) {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    let scan = room_scan(6.0, 6.0, 3.0, 3.0, 360);
    let pose = Pose2D::new(3.0, 3.0, 0.0);

    // Warm up
    for _ in 0..5 {
        map.observe_lidar(&scan, pose);
    }

    c.bench_function("grid_update_360pts", |b| {
        b.iter(|| {
            let result = map.observe_lidar(black_box(&scan), black_box(pose));
            black_box(result)
        })
    });
}

fn bench_map_update_resolutions(c: &mut Criterion) {
    let mut group = c.benchmark_group("grid_update_resolution");

    for num_points in [180, 360, 720].iter() {
        let config = MapConfig::default();
        let mut map = OccupancyGridMap::new(config);

        let scan = room_scan(6.0, 6.0, 3.0, 3.0, *num_points);
        let pose = Pose2D::new(3.0, 3.0, 0.0);

        // Warm up
        for _ in 0..5 {
            map.observe_lidar(&scan, pose);
        }

        group.bench_with_input(
            BenchmarkId::from_parameter(num_points),
            num_points,
            |b, _| {
                b.iter(|| {
                    let result = map.observe_lidar(black_box(&scan), black_box(pose));
                    black_box(result)
                })
            },
        );
    }

    group.finish();
}

fn bench_coverage_stats(c: &mut Criterion) {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Build up a map
    for i in 0..20 {
        let angle = i as f32 * 0.2;
        let pose = Pose2D::new(3.0, 3.0, angle);
        let scan = room_scan(6.0, 6.0, 3.0, 3.0, 360);
        map.observe_lidar(&scan, pose);
    }

    c.bench_function("coverage_stats", |b| {
        b.iter(|| {
            let stats = map.coverage_stats();
            black_box(stats)
        })
    });
}

criterion_group!(
    benches,
    bench_map_update,
    bench_map_update_resolutions,
    bench_coverage_stats
);
criterion_main!(benches);
