//! Ground truth generation for Cartographer-style evaluation.
//!
//! This module provides the [`GroundTruthRelations`] type for extracting pose relations
//! from an optimized trajectory, equivalent to Cartographer's `autogenerate_ground_truth`.
//!
//! ## Concept
//!
//! Instead of requiring external ground truth (GPS, motion capture), we extract
//! **relative pose constraints** from an optimized trajectory that has had loop
//! closures applied. These relations can then be used to evaluate other trajectories.

use crate::Pose2D;
use serde::{Deserialize, Serialize};

/// Configuration for ground truth generation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GroundTruthConfig {
    /// Minimum distance covered between nodes to create a relation (meters).
    /// Shorter distances are less meaningful for evaluation.
    pub min_covered_distance: f32,

    /// Maximum translational error to consider a relation valid (meters).
    /// Relations with larger errors are treated as outliers.
    pub outlier_threshold_meters: f32,

    /// Maximum rotational error to consider a relation valid (radians).
    pub outlier_threshold_radians: f32,

    /// Sample every N nodes to create relations.
    /// Higher values reduce computation but may miss details.
    pub node_sampling_interval: usize,
}

impl Default for GroundTruthConfig {
    fn default() -> Self {
        Self {
            min_covered_distance: 0.5,        // 50cm minimum
            outlier_threshold_meters: 0.05,   // 5cm threshold
            outlier_threshold_radians: 0.035, // ~2 degrees
            node_sampling_interval: 1,
        }
    }
}

/// A pose relation constraint between two trajectory nodes.
///
/// This represents the relative transform from one node to another,
/// along with the distance traveled between them (for weighting).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseRelation {
    /// Index of the starting node.
    pub from_node: usize,

    /// Index of the ending node.
    pub to_node: usize,

    /// Relative pose: transform from `from_node` to `to_node`.
    /// Apply as: to_pose = from_pose.compose(&relative_pose)
    pub relative_pose: Pose2D,

    /// Distance covered between nodes (cumulative path length).
    /// Used for weighting in metric computation.
    pub covered_distance: f32,
}

impl PoseRelation {
    /// Create a new pose relation.
    pub fn new(
        from_node: usize,
        to_node: usize,
        relative_pose: Pose2D,
        covered_distance: f32,
    ) -> Self {
        Self {
            from_node,
            to_node,
            relative_pose,
            covered_distance,
        }
    }

    /// Compute the relative pose from two absolute poses.
    pub fn from_poses(
        from_node: usize,
        to_node: usize,
        from_pose: &Pose2D,
        to_pose: &Pose2D,
        covered_distance: f32,
    ) -> Self {
        // relative_pose = from_pose.inverse().compose(to_pose)
        let relative_pose = from_pose.inverse().compose(to_pose);
        Self::new(from_node, to_node, relative_pose, covered_distance)
    }
}

/// Ground truth relations extracted from an optimized trajectory.
///
/// This is the Cartographer-style ground truth format. It stores relative
/// pose constraints between trajectory nodes, which can be compared against
/// a test trajectory to compute accuracy metrics.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GroundTruthRelations {
    /// The pose relations extracted from the optimized trajectory.
    pub relations: Vec<PoseRelation>,

    /// Configuration used for generation.
    pub config: GroundTruthConfig,

    /// Total trajectory length (for reference).
    pub trajectory_length: f32,

    /// Number of nodes in the original trajectory.
    pub num_nodes: usize,
}

impl GroundTruthRelations {
    /// Generate ground truth relations from an optimized trajectory.
    ///
    /// This is equivalent to Cartographer's `autogenerate_ground_truth`.
    /// It extracts relative pose constraints between nodes in the trajectory.
    ///
    /// # Arguments
    ///
    /// * `optimized_trajectory` - Poses from an optimized trajectory (with loop closures)
    /// * `loop_closures` - Pairs of node indices that form loop closures (optional)
    /// * `config` - Configuration for relation generation
    ///
    /// # Returns
    ///
    /// Ground truth relations that can be used for evaluation.
    pub fn generate(
        optimized_trajectory: &[Pose2D],
        loop_closures: &[(usize, usize)],
        config: GroundTruthConfig,
    ) -> Self {
        let num_nodes = optimized_trajectory.len();
        if num_nodes < 2 {
            return Self {
                relations: Vec::new(),
                config,
                trajectory_length: 0.0,
                num_nodes,
            };
        }

        // Compute cumulative distances along trajectory
        let mut cumulative_distances = vec![0.0f32; num_nodes];
        for i in 1..num_nodes {
            let dist = optimized_trajectory[i].distance(&optimized_trajectory[i - 1]);
            cumulative_distances[i] = cumulative_distances[i - 1] + dist;
        }
        let trajectory_length = cumulative_distances[num_nodes - 1];

        let mut relations = Vec::new();

        // Generate sequential relations
        let interval = config.node_sampling_interval.max(1);
        for i in (0..num_nodes).step_by(interval) {
            for j in ((i + 1)..num_nodes).step_by(interval) {
                let covered_distance = cumulative_distances[j] - cumulative_distances[i];

                // Skip if distance is too short
                if covered_distance < config.min_covered_distance {
                    continue;
                }

                let relation = PoseRelation::from_poses(
                    i,
                    j,
                    &optimized_trajectory[i],
                    &optimized_trajectory[j],
                    covered_distance,
                );

                relations.push(relation);
            }
        }

        // Add loop closure relations with high weight
        for &(from, to) in loop_closures {
            if from < num_nodes && to < num_nodes && from != to {
                // For loop closures, covered distance is the full loop
                let covered_distance = if from < to {
                    cumulative_distances[to] - cumulative_distances[from]
                } else {
                    cumulative_distances[from] - cumulative_distances[to]
                };

                let relation = PoseRelation::from_poses(
                    from.min(to),
                    from.max(to),
                    &optimized_trajectory[from.min(to)],
                    &optimized_trajectory[from.max(to)],
                    covered_distance,
                );

                // Avoid duplicates
                if !relations
                    .iter()
                    .any(|r| r.from_node == relation.from_node && r.to_node == relation.to_node)
                {
                    relations.push(relation);
                }
            }
        }

        Self {
            relations,
            config,
            trajectory_length,
            num_nodes,
        }
    }

    /// Generate ground truth from a trajectory with timestamps.
    ///
    /// Convenience method that accepts timestamped poses.
    pub fn generate_timestamped(
        trajectory: &[(f64, Pose2D)],
        loop_closures: &[(usize, usize)],
        config: GroundTruthConfig,
    ) -> Self {
        let poses: Vec<Pose2D> = trajectory.iter().map(|(_, pose)| *pose).collect();
        Self::generate(&poses, loop_closures, config)
    }

    /// Get the number of relations.
    pub fn len(&self) -> usize {
        self.relations.len()
    }

    /// Check if there are no relations.
    pub fn is_empty(&self) -> bool {
        self.relations.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn test_pose_relation_from_poses() {
        let from_pose = Pose2D::new(0.0, 0.0, 0.0);
        let to_pose = Pose2D::new(1.0, 0.0, 0.0);

        let relation = PoseRelation::from_poses(0, 1, &from_pose, &to_pose, 1.0);

        assert_eq!(relation.from_node, 0);
        assert_eq!(relation.to_node, 1);
        assert!((relation.relative_pose.x - 1.0).abs() < 1e-5);
        assert!((relation.relative_pose.y - 0.0).abs() < 1e-5);
    }

    #[test]
    fn test_pose_relation_with_rotation() {
        let from_pose = Pose2D::new(0.0, 0.0, 0.0);
        let to_pose = Pose2D::new(1.0, 1.0, FRAC_PI_2);

        let relation = PoseRelation::from_poses(0, 1, &from_pose, &to_pose, 1.414);

        // Relative pose should encode the transform
        let reconstructed = from_pose.compose(&relation.relative_pose);
        assert!((reconstructed.x - to_pose.x).abs() < 1e-5);
        assert!((reconstructed.y - to_pose.y).abs() < 1e-5);
        assert!((reconstructed.theta - to_pose.theta).abs() < 1e-5);
    }

    #[test]
    fn test_ground_truth_generation() {
        // Simple straight-line trajectory
        let trajectory: Vec<Pose2D> = (0..10)
            .map(|i| Pose2D::new(i as f32 * 0.5, 0.0, 0.0))
            .collect();

        let config = GroundTruthConfig {
            min_covered_distance: 0.5,
            node_sampling_interval: 1,
            ..Default::default()
        };

        let ground_truth = GroundTruthRelations::generate(&trajectory, &[], config);

        assert!(ground_truth.len() > 0);
        assert_eq!(ground_truth.num_nodes, 10);
        assert!((ground_truth.trajectory_length - 4.5).abs() < 1e-5);
    }

    #[test]
    fn test_ground_truth_with_loop_closures() {
        // Square trajectory
        let trajectory = vec![
            Pose2D::new(0.0, 0.0, 0.0),
            Pose2D::new(1.0, 0.0, 0.0),
            Pose2D::new(1.0, 1.0, FRAC_PI_2),
            Pose2D::new(0.0, 1.0, std::f32::consts::PI),
            Pose2D::new(0.0, 0.0, -FRAC_PI_2), // Back to start
        ];

        let loop_closures = vec![(0, 4)]; // Loop closure between first and last

        let config = GroundTruthConfig {
            min_covered_distance: 0.5,
            ..Default::default()
        };

        let ground_truth = GroundTruthRelations::generate(&trajectory, &loop_closures, config);

        // Should have relations including the loop closure
        assert!(!ground_truth.is_empty());
        assert!(
            ground_truth
                .relations
                .iter()
                .any(|r| (r.from_node == 0 && r.to_node == 4)
                    || (r.from_node == 4 && r.to_node == 0))
        );
    }

    #[test]
    fn test_empty_trajectory() {
        let config = GroundTruthConfig::default();
        let ground_truth = GroundTruthRelations::generate(&[], &[], config);

        assert!(ground_truth.is_empty());
        assert_eq!(ground_truth.trajectory_length, 0.0);
    }
}
