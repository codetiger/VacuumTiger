//! Benchmark scan matching performance.

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use std::f32::consts::PI;
use std::hint::black_box;
use vastu_slam::{
    CorrelativeMatcher, CorrelativeMatcherConfig, LidarScan, MapConfig, OccupancyGridMap, Pose2D,
};

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

        // Right wall
        if cos_a > 0.0 {
            let t = (room_width - robot_x) / cos_a;
            if t > 0.0 && t < range {
                let y = robot_y + t * sin_a;
                if y >= 0.0 && y <= room_height {
                    range = t;
                }
            }
        }
        // Left wall
        if cos_a < 0.0 {
            let t = -robot_x / cos_a;
            if t > 0.0 && t < range {
                let y = robot_y + t * sin_a;
                if y >= 0.0 && y <= room_height {
                    range = t;
                }
            }
        }
        // Top wall
        if sin_a > 0.0 {
            let t = (room_height - robot_y) / sin_a;
            if t > 0.0 && t < range {
                let x = robot_x + t * cos_a;
                if x >= 0.0 && x <= room_width {
                    range = t;
                }
            }
        }
        // Bottom wall
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

fn setup_map() -> (OccupancyGridMap, LidarScan) {
    let config = MapConfig::default();
    let mut map = OccupancyGridMap::new(config);

    // Initialize map
    for i in 0..10 {
        let angle = i as f32 * 0.1;
        let pose = Pose2D::new(3.0, 3.0, angle);
        let scan = room_scan(6.0, 6.0, 3.0, 3.0, 360);
        map.observe_lidar(&scan, pose);
    }

    let scan = room_scan(6.0, 6.0, 3.1, 3.0, 360);
    (map, scan)
}

fn bench_scan_matching(c: &mut Criterion) {
    let (map, scan) = setup_map();
    let initial_pose = Pose2D::new(3.0, 3.0, 0.0);

    // Sequential version
    let config_seq = CorrelativeMatcherConfig {
        use_parallel: false,
        ..CorrelativeMatcherConfig::default()
    };
    let matcher_seq = CorrelativeMatcher::new(config_seq);

    c.bench_function("scan_matching_360pts_sequential", |b| {
        b.iter(|| {
            let result =
                matcher_seq.match_scan(black_box(&scan), black_box(initial_pose), map.storage());
            black_box(result)
        })
    });

    // Parallel version
    let config_par = CorrelativeMatcherConfig {
        use_parallel: true,
        ..CorrelativeMatcherConfig::default()
    };
    let matcher_par = CorrelativeMatcher::new(config_par);

    c.bench_function("scan_matching_360pts_parallel", |b| {
        b.iter(|| {
            let result =
                matcher_par.match_scan(black_box(&scan), black_box(initial_pose), map.storage());
            black_box(result)
        })
    });
}

fn bench_scan_matching_resolutions(c: &mut Criterion) {
    let mut group = c.benchmark_group("scan_matching_resolution");

    for num_points in [180, 360, 720].iter() {
        let config = MapConfig::default();
        let mut map = OccupancyGridMap::new(config);

        for i in 0..10 {
            let angle = i as f32 * 0.1;
            let pose = Pose2D::new(3.0, 3.0, angle);
            let scan = room_scan(6.0, 6.0, 3.0, 3.0, *num_points);
            map.observe_lidar(&scan, pose);
        }

        let scan = room_scan(6.0, 6.0, 3.1, 3.0, *num_points);
        let initial_pose = Pose2D::new(3.0, 3.0, 0.0);

        let matcher_config = CorrelativeMatcherConfig::default();
        let matcher = CorrelativeMatcher::new(matcher_config);

        group.bench_with_input(
            BenchmarkId::from_parameter(num_points),
            num_points,
            |b, _| {
                b.iter(|| {
                    let result = matcher.match_scan(
                        black_box(&scan),
                        black_box(initial_pose),
                        map.storage(),
                    );
                    black_box(result)
                })
            },
        );
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_scan_matching,
    bench_scan_matching_resolutions
);
criterion_main!(benches);
