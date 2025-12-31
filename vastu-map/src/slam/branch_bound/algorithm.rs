//! Branch-and-bound search algorithm implementation.

use crate::core::simd::PointCloud;
use crate::core::{LidarScan, Pose2D, WorldPoint};

use super::config::{BranchBoundConfig, BranchBoundResult, NUM_LEVELS};
use super::grids::PrecomputedGrids;
use super::search_node::SearchNode;

/// Perform branch-and-bound scan matching.
///
/// This is more efficient than brute-force for large search windows.
/// For small windows (< 30cm), brute-force may be faster due to overhead.
///
/// # Arguments
/// * `points` - Scan points in sensor frame (x, y)
/// * `grids` - Precomputed multi-resolution grids
/// * `prior_pose` - Initial pose estimate
/// * `config` - Search configuration
///
/// # Returns
/// The best matching pose and score.
pub fn branch_and_bound_match(
    points: &[(f32, f32)],
    grids: &PrecomputedGrids,
    prior_pose: Pose2D,
    config: &BranchBoundConfig,
) -> BranchBoundResult {
    if points.is_empty() {
        return BranchBoundResult {
            pose: prior_pose,
            score: 0.0,
            nodes_expanded: 0,
            converged: false,
        };
    }

    // Calculate angular bins
    let num_theta_bins = (config.search_theta * 2.0 / config.angular_resolution).ceil() as i32;
    let theta_step = config.angular_resolution;

    // Calculate search bounds in coarsest grid coordinates
    let coarsest_level = NUM_LEVELS - 1;

    let search_min = WorldPoint::new(
        prior_pose.x - config.search_x,
        prior_pose.y - config.search_y,
    );
    let search_max = WorldPoint::new(
        prior_pose.x + config.search_x,
        prior_pose.y + config.search_y,
    );

    let min_coord = grids.world_to_level_grid(search_min, coarsest_level);
    let max_coord = grids.world_to_level_grid(search_max, coarsest_level);

    // Initialize search with coarsest nodes
    let mut stack: Vec<SearchNode> = Vec::with_capacity(256);
    let mut best_score = config.min_score;
    let mut best_pose = prior_pose;
    let mut nodes_expanded = 0usize;

    // Start theta search around prior
    let prior_theta_bin = (prior_pose.theta / theta_step).round() as i32;
    let theta_range = (config.search_theta / theta_step).ceil() as i32;

    for y in min_coord.y..=max_coord.y {
        for x in min_coord.x..=max_coord.x {
            for dt in -theta_range..=theta_range {
                let theta_bin = prior_theta_bin + dt;
                stack.push(SearchNode {
                    level: coarsest_level,
                    x,
                    y,
                    theta_bin,
                    upper_bound: f32::MAX, // Will compute on first access
                });
            }
        }
    }

    // Branch-and-bound search
    while let Some(mut node) = stack.pop() {
        nodes_expanded += 1;

        // Compute upper bound if not yet computed
        if node.upper_bound == f32::MAX {
            let pose = node.center_pose(grids, theta_step);
            node.upper_bound =
                grids.score_scan_at_level(points, pose, node.level, config.sensor_offset);
        }

        // Prune if upper bound is worse than current best
        if node.upper_bound <= best_score {
            continue;
        }

        if node.level == 0 {
            // Finest level - this is the actual score
            let pose = node.center_pose(grids, theta_step);
            let score = grids.score_scan_at_level(points, pose, 0, config.sensor_offset);

            if score > best_score {
                best_score = score;
                best_pose = pose;
            }
        } else {
            // Expand to finer level
            let children = node.subdivide(grids, num_theta_bins);
            for mut child in children {
                let child_pose = child.center_pose(grids, theta_step);
                child.upper_bound = grids.score_scan_at_level(
                    points,
                    child_pose,
                    child.level,
                    config.sensor_offset,
                );

                // Only add if upper bound could beat current best
                if child.upper_bound > best_score {
                    stack.push(child);
                }
            }
        }
    }

    BranchBoundResult {
        pose: best_pose,
        score: best_score,
        nodes_expanded,
        converged: best_score > config.min_score,
    }
}

/// SIMD-optimized branch-and-bound scan matching.
///
/// Uses PointCloud (SoA format) for efficient SIMD point transformations.
/// This is the preferred method for loop closure verification.
///
/// # Arguments
/// * `points` - Scan points in sensor frame (PointCloud SoA format)
/// * `grids` - Precomputed multi-resolution grids
/// * `prior_pose` - Initial pose estimate
/// * `config` - Search configuration
///
/// # Returns
/// The best matching pose and score.
pub fn branch_and_bound_match_simd(
    points: &PointCloud,
    grids: &PrecomputedGrids,
    prior_pose: Pose2D,
    config: &BranchBoundConfig,
) -> BranchBoundResult {
    if points.is_empty() {
        return BranchBoundResult {
            pose: prior_pose,
            score: 0.0,
            nodes_expanded: 0,
            converged: false,
        };
    }

    // Reusable scratch buffers for SIMD transforms
    let mut world_xs = Vec::with_capacity(points.len());
    let mut world_ys = Vec::with_capacity(points.len());

    // Calculate angular bins
    let num_theta_bins = (config.search_theta * 2.0 / config.angular_resolution).ceil() as i32;
    let theta_step = config.angular_resolution;

    // Calculate search bounds in coarsest grid coordinates
    let coarsest_level = NUM_LEVELS - 1;

    let search_min = WorldPoint::new(
        prior_pose.x - config.search_x,
        prior_pose.y - config.search_y,
    );
    let search_max = WorldPoint::new(
        prior_pose.x + config.search_x,
        prior_pose.y + config.search_y,
    );

    let min_coord = grids.world_to_level_grid(search_min, coarsest_level);
    let max_coord = grids.world_to_level_grid(search_max, coarsest_level);

    // Initialize search with coarsest nodes
    let mut stack: Vec<SearchNode> = Vec::with_capacity(256);
    let mut best_score = config.min_score;
    let mut best_pose = prior_pose;
    let mut nodes_expanded = 0usize;

    // Start theta search around prior
    let prior_theta_bin = (prior_pose.theta / theta_step).round() as i32;
    let theta_range = (config.search_theta / theta_step).ceil() as i32;

    for y in min_coord.y..=max_coord.y {
        for x in min_coord.x..=max_coord.x {
            for dt in -theta_range..=theta_range {
                let theta_bin = prior_theta_bin + dt;
                stack.push(SearchNode {
                    level: coarsest_level,
                    x,
                    y,
                    theta_bin,
                    upper_bound: f32::MAX,
                });
            }
        }
    }

    // Branch-and-bound search with SIMD scoring
    while let Some(mut node) = stack.pop() {
        nodes_expanded += 1;

        // Compute upper bound if not yet computed
        if node.upper_bound == f32::MAX {
            let pose = node.center_pose(grids, theta_step);
            node.upper_bound = grids.score_scan_simd_with_scratch(
                points,
                pose,
                node.level,
                config.sensor_offset,
                &mut world_xs,
                &mut world_ys,
            );
        }

        // Prune if upper bound is worse than current best
        if node.upper_bound <= best_score {
            continue;
        }

        if node.level == 0 {
            // Finest level - this is the actual score
            let pose = node.center_pose(grids, theta_step);
            let score = grids.score_scan_simd_with_scratch(
                points,
                pose,
                0,
                config.sensor_offset,
                &mut world_xs,
                &mut world_ys,
            );

            if score > best_score {
                best_score = score;
                best_pose = pose;
            }
        } else {
            // Expand to finer level
            let children = node.subdivide(grids, num_theta_bins);
            for mut child in children {
                let child_pose = child.center_pose(grids, theta_step);
                child.upper_bound = grids.score_scan_simd_with_scratch(
                    points,
                    child_pose,
                    child.level,
                    config.sensor_offset,
                    &mut world_xs,
                    &mut world_ys,
                );

                // Only add if upper bound could beat current best
                if child.upper_bound > best_score {
                    stack.push(child);
                }
            }
        }
    }

    BranchBoundResult {
        pose: best_pose,
        score: best_score,
        nodes_expanded,
        converged: best_score > config.min_score,
    }
}

/// Convenience function to match a LidarScan directly.
///
/// Converts the scan to PointCloud format and calls branch_and_bound_match_simd.
pub fn branch_and_bound_match_scan(
    scan: &LidarScan,
    grids: &PrecomputedGrids,
    prior_pose: Pose2D,
    config: &BranchBoundConfig,
) -> BranchBoundResult {
    let points = PointCloud::from_scan(scan);
    branch_and_bound_match_simd(&points, grids, prior_pose, config)
}
