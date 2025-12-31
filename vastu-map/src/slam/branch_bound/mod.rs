//! Branch-and-bound scan matcher with precomputed multi-resolution grids.
//!
//! Implements Cartographer-style branch-and-bound search for efficient scan matching
//! over large search windows. Uses a precomputed grid hierarchy where each coarse cell
//! stores the maximum probability of all finer cells within it.
//!
//! This allows pruning: if the upper bound score at a coarse level is worse than
//! the current best, the entire subtree can be skipped.
//!
//! ## Grid Hierarchy
//!
//! ```text
//! Level 0 (1x):   [0.8][0.2][0.9][0.1][0.7][0.3][0.8][0.2]  <- original resolution
//!                   \   /       \   /       \   /       \   /
//! Level 1 (4x):     [0.8]       [0.9]       [0.7]       [0.8]   <- max of 4 cells
//!                     \           /           \           /
//! Level 2 (16x):        [0.9]                   [0.8]
//!                           \                   /
//! Level 3 (64x):              [0.9]                          <- coarsest level
//! ```
//!
//! At each coarse level, the value is the maximum of all finer cells below.
//! This provides an upper bound: the true score cannot exceed the coarse score.

mod algorithm;
mod config;
mod grids;
mod search_node;

pub use algorithm::{
    branch_and_bound_match, branch_and_bound_match_scan, branch_and_bound_match_simd,
};
pub use config::{BranchBoundConfig, BranchBoundResult, NUM_LEVELS};
pub use grids::PrecomputedGrids;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{CellType, GridCoord, Pose2D};
    use crate::grid::GridStorage;

    fn create_test_storage() -> GridStorage {
        let mut storage = GridStorage::centered(100, 100, 0.05);

        // Create a wall at x=1m
        for y in 40..60 {
            storage.set_type(GridCoord::new(70, y), CellType::Wall);
        }

        storage.recompute_distance_field();
        storage
    }

    #[test]
    fn test_precomputed_grids_creation() {
        let storage = create_test_storage();
        let grids = PrecomputedGrids::from_storage(&storage);

        // Check dimensions at each level
        assert_eq!(grids.dimensions(), (100, 100));
    }

    #[test]
    fn test_coarse_level_is_upper_bound() {
        let storage = create_test_storage();
        let grids = PrecomputedGrids::from_storage(&storage);

        // Sample some random points and check that coarse scores >= fine scores
        let points = vec![(1.0, 0.0), (1.0, 0.1), (1.0, -0.1)];
        let pose = Pose2D::new(0.0, 0.0, 0.0);
        let offset = (0.0, 0.0);

        let fine_score = grids.score_scan_at_level(&points, pose, 0, offset);

        for level in 1..NUM_LEVELS {
            let coarse_score = grids.score_scan_at_level(&points, pose, level, offset);
            assert!(
                coarse_score >= fine_score - 0.01,
                "Coarse level {} score {} should be >= fine score {}",
                level,
                coarse_score,
                fine_score
            );
        }
    }

    #[test]
    fn test_branch_bound_match() {
        let storage = create_test_storage();
        let grids = PrecomputedGrids::from_storage(&storage);

        // Create scan points that should match the wall at x=1m
        let points: Vec<(f32, f32)> = (-10..=10).map(|i| (1.0, i as f32 * 0.05)).collect();

        let prior = Pose2D::new(0.0, 0.0, 0.0);
        let config = BranchBoundConfig {
            search_x: 0.5,
            search_y: 0.5,
            search_theta: 0.3,
            ..Default::default()
        };

        let result = branch_and_bound_match(&points, &grids, prior, &config);

        // Should find a match
        assert!(result.score > 0.0);
        assert!(result.nodes_expanded > 0);
    }

    #[test]
    fn test_empty_points() {
        let storage = create_test_storage();
        let grids = PrecomputedGrids::from_storage(&storage);

        let prior = Pose2D::new(0.0, 0.0, 0.0);
        let config = BranchBoundConfig::default();

        let result = branch_and_bound_match(&[], &grids, prior, &config);

        assert_eq!(result.score, 0.0);
        assert!(!result.converged);
    }
}
