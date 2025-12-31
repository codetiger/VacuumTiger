//! Test utilities for VastuSLAM evaluation.
//!
//! This module provides helpers for creating test trajectories, maps, and scenarios.

#![allow(dead_code)]

use std::f32::consts::FRAC_PI_2;
use vastu_slam::{LidarScan, Pose2D};

/// Create a straight-line trajectory.
pub fn straight_trajectory(n: usize, spacing: f32) -> Vec<Pose2D> {
    (0..n)
        .map(|i| Pose2D::new(i as f32 * spacing, 0.0, 0.0))
        .collect()
}

/// Create a square loop trajectory.
pub fn square_trajectory(side_length: f32, points_per_side: usize) -> Vec<Pose2D> {
    let mut poses = Vec::new();
    let spacing = side_length / points_per_side as f32;

    // Bottom side (left to right)
    for i in 0..points_per_side {
        poses.push(Pose2D::new(i as f32 * spacing, 0.0, 0.0));
    }

    // Right side (bottom to top)
    for i in 0..points_per_side {
        poses.push(Pose2D::new(side_length, i as f32 * spacing, FRAC_PI_2));
    }

    // Top side (right to left)
    for i in 0..points_per_side {
        poses.push(Pose2D::new(
            side_length - i as f32 * spacing,
            side_length,
            std::f32::consts::PI,
        ));
    }

    // Left side (top to bottom, back to start)
    for i in 0..points_per_side {
        poses.push(Pose2D::new(
            0.0,
            side_length - i as f32 * spacing,
            -FRAC_PI_2,
        ));
    }

    poses
}

/// Create a figure-8 trajectory.
pub fn figure_eight_trajectory(radius: f32, points: usize) -> Vec<Pose2D> {
    let mut poses = Vec::new();
    let half = points / 2;

    // First circle (counter-clockwise)
    for i in 0..half {
        let angle = 2.0 * std::f32::consts::PI * i as f32 / half as f32;
        let x = radius * (1.0 - angle.cos());
        let y = radius * angle.sin();
        let theta = angle + FRAC_PI_2;
        poses.push(Pose2D::new(x, y, theta));
    }

    // Second circle (clockwise)
    for i in 0..half {
        let angle = 2.0 * std::f32::consts::PI * i as f32 / half as f32;
        let x = 2.0 * radius + radius * (angle.cos() - 1.0);
        let y = -radius * angle.sin();
        let theta = -angle - FRAC_PI_2;
        poses.push(Pose2D::new(x, y, theta));
    }

    poses
}

/// Add Gaussian noise to a trajectory.
pub fn add_noise(trajectory: &[Pose2D], trans_std: f32, rot_std: f32, seed: u64) -> Vec<Pose2D> {
    use std::num::Wrapping;

    // Simple LCG PRNG for reproducibility
    let mut state = Wrapping(seed);
    let a = Wrapping(1664525u64);
    let c = Wrapping(1013904223u64);

    let mut random = || -> f32 {
        state = a * state + c;
        // Box-Muller transform for Gaussian
        let u1 = (state.0 & 0xFFFF) as f32 / 65536.0 + 0.0001;
        state = a * state + c;
        let u2 = (state.0 & 0xFFFF) as f32 / 65536.0;
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    };

    trajectory
        .iter()
        .map(|p| {
            Pose2D::new(
                p.x + random() * trans_std,
                p.y + random() * trans_std,
                p.theta + random() * rot_std,
            )
        })
        .collect()
}

/// Add drift to a trajectory (cumulative error).
pub fn add_drift(trajectory: &[Pose2D], drift_per_m: f32) -> Vec<Pose2D> {
    if trajectory.is_empty() {
        return Vec::new();
    }

    let mut result = Vec::with_capacity(trajectory.len());
    result.push(trajectory[0]);

    let mut cumulative_drift = 0.0f32;

    for i in 1..trajectory.len() {
        let dist = trajectory[i].distance(&trajectory[i - 1]);
        cumulative_drift += dist * drift_per_m;

        let p = &trajectory[i];
        result.push(Pose2D::new(p.x + cumulative_drift, p.y, p.theta));
    }

    result
}

/// Create a simple lidar scan for testing.
pub fn simple_scan(num_points: usize, max_range: f32) -> LidarScan {
    let angle_increment = 2.0 * std::f32::consts::PI / num_points as f32;
    let angles: Vec<f32> = (0..num_points)
        .map(|i| i as f32 * angle_increment - std::f32::consts::PI)
        .collect();

    // Simple pattern: range varies with angle
    let ranges: Vec<f32> = angles
        .iter()
        .map(|a| max_range * 0.5 * (1.0 + a.cos().abs()))
        .collect();

    LidarScan::new(ranges, angles, 0.15, max_range)
}

/// Create a scan that hits walls in a square room.
pub fn room_scan(
    room_width: f32,
    room_height: f32,
    robot_x: f32,
    robot_y: f32,
    num_points: usize,
) -> LidarScan {
    let angle_increment = 2.0 * std::f32::consts::PI / num_points as f32;
    let max_range = (room_width * room_width + room_height * room_height).sqrt();

    let mut ranges = Vec::with_capacity(num_points);
    let mut angles = Vec::with_capacity(num_points);

    for i in 0..num_points {
        let angle = i as f32 * angle_increment - std::f32::consts::PI;
        angles.push(angle);

        // Ray cast to find wall intersection
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        let mut range = max_range;

        // Check intersection with each wall
        // Right wall (x = room_width)
        if cos_a > 0.0 {
            let t = (room_width - robot_x) / cos_a;
            if t > 0.0 && t < range {
                let y = robot_y + t * sin_a;
                if y >= 0.0 && y <= room_height {
                    range = t;
                }
            }
        }
        // Left wall (x = 0)
        if cos_a < 0.0 {
            let t = -robot_x / cos_a;
            if t > 0.0 && t < range {
                let y = robot_y + t * sin_a;
                if y >= 0.0 && y <= room_height {
                    range = t;
                }
            }
        }
        // Top wall (y = room_height)
        if sin_a > 0.0 {
            let t = (room_height - robot_y) / sin_a;
            if t > 0.0 && t < range {
                let x = robot_x + t * cos_a;
                if x >= 0.0 && x <= room_width {
                    range = t;
                }
            }
        }
        // Bottom wall (y = 0)
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

/// Assert two poses are approximately equal.
pub fn assert_poses_close(a: &Pose2D, b: &Pose2D, trans_tol: f32, rot_tol: f32) {
    let trans_error = a.distance(b);
    let rot_error = (a.theta - b.theta).abs();

    assert!(
        trans_error < trans_tol,
        "Translation error {} exceeds tolerance {}",
        trans_error,
        trans_tol
    );
    assert!(
        rot_error < rot_tol,
        "Rotation error {} exceeds tolerance {}",
        rot_error,
        rot_tol
    );
}

/// Assert trajectory ATE is under threshold.
pub fn assert_ate_under(estimated: &[Pose2D], ground_truth: &[Pose2D], threshold: f32) {
    use vastu_slam::evaluation::AbsoluteTrajectoryError;

    let ate = AbsoluteTrajectoryError::compute(estimated, ground_truth);

    assert!(
        ate.translation.rmse < threshold,
        "ATE RMSE {} exceeds threshold {}",
        ate.translation.rmse,
        threshold
    );
}
