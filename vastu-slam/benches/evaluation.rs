//! Benchmark evaluation metrics performance.

use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use vastu_slam::Pose2D;
use vastu_slam::evaluation::{
    AbsoluteTrajectoryError, GroundTruthConfig, GroundTruthRelations, RelationsMetrics,
    RelativePoseError,
};

fn create_trajectory(n: usize, spacing: f32) -> Vec<Pose2D> {
    (0..n)
        .map(|i| Pose2D::new(i as f32 * spacing, 0.0, 0.0))
        .collect()
}

fn add_noise(trajectory: &[Pose2D], trans_std: f32) -> Vec<Pose2D> {
    trajectory
        .iter()
        .enumerate()
        .map(|(i, p)| {
            // Simple deterministic "noise"
            let noise = (
                (i as f32 * 0.7).sin() * trans_std,
                (i as f32 * 1.3).cos() * trans_std,
            );
            Pose2D::new(p.x + noise.0, p.y + noise.1, p.theta)
        })
        .collect()
}

fn bench_ground_truth_generation(c: &mut Criterion) {
    let mut group = c.benchmark_group("ground_truth_generation");

    for n in [50, 100, 500].iter() {
        let trajectory = create_trajectory(*n, 0.1);
        let config = GroundTruthConfig::default();

        group.bench_with_input(BenchmarkId::from_parameter(n), n, |b, _| {
            b.iter(|| {
                let gt =
                    GroundTruthRelations::generate(black_box(&trajectory), &[], config.clone());
                black_box(gt)
            })
        });
    }

    group.finish();
}

fn bench_relations_metrics(c: &mut Criterion) {
    let mut group = c.benchmark_group("relations_metrics");

    for n in [50, 100, 500].iter() {
        let ground_trajectory = create_trajectory(*n, 0.1);
        let config = GroundTruthConfig::default();
        let ground_truth = GroundTruthRelations::generate(&ground_trajectory, &[], config);

        let test_trajectory = add_noise(&ground_trajectory, 0.001);

        group.bench_with_input(BenchmarkId::from_parameter(n), n, |b, _| {
            b.iter(|| {
                let metrics = RelationsMetrics::compute(black_box(&test_trajectory), &ground_truth);
                black_box(metrics)
            })
        });
    }

    group.finish();
}

fn bench_ate(c: &mut Criterion) {
    let mut group = c.benchmark_group("ate");

    for n in [50, 100, 500].iter() {
        let ground_truth = create_trajectory(*n, 0.1);
        let estimated = add_noise(&ground_truth, 0.001);

        group.bench_with_input(BenchmarkId::from_parameter(n), n, |b, _| {
            b.iter(|| {
                let ate = AbsoluteTrajectoryError::compute(
                    black_box(&estimated),
                    black_box(&ground_truth),
                );
                black_box(ate)
            })
        });
    }

    group.finish();
}

fn bench_rpe(c: &mut Criterion) {
    let mut group = c.benchmark_group("rpe");

    for n in [50, 100, 500].iter() {
        let ground_truth = create_trajectory(*n, 0.1);
        let estimated = add_noise(&ground_truth, 0.001);

        group.bench_with_input(BenchmarkId::from_parameter(n), n, |b, _| {
            b.iter(|| {
                let rpe = RelativePoseError::compute(
                    black_box(&estimated),
                    black_box(&ground_truth),
                    1.0,
                );
                black_box(rpe)
            })
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_ground_truth_generation,
    bench_relations_metrics,
    bench_ate,
    bench_rpe
);
criterion_main!(benches);
