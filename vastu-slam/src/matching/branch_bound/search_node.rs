//! Search node for branch-and-bound algorithm.

use crate::core::Pose2D;

use super::grids::PrecomputedGrids;

/// A node in the branch-and-bound search tree.
#[derive(Clone, Debug)]
pub(super) struct SearchNode {
    /// Resolution level (0 = finest, NUM_LEVELS-1 = coarsest).
    pub level: usize,
    /// X position in grid coordinates at this level.
    pub x: i32,
    /// Y position in grid coordinates at this level.
    pub y: i32,
    /// Theta bin index (for angular discretization).
    pub theta_bin: i32,
    /// Upper bound score for this node.
    pub upper_bound: f32,
}

impl SearchNode {
    /// Get the center pose for this search node.
    pub fn center_pose(&self, grids: &PrecomputedGrids, theta_step: f32) -> Pose2D {
        let res = grids.level_resolution(self.level);
        Pose2D::new(
            grids.origin().x + (self.x as f32 + 0.5) * res,
            grids.origin().y + (self.y as f32 + 0.5) * res,
            self.theta_bin as f32 * theta_step,
        )
    }

    /// Generate child nodes (subdivide into finer level).
    pub fn subdivide(&self, grids: &PrecomputedGrids, num_theta_bins: i32) -> Vec<SearchNode> {
        if self.level == 0 {
            return Vec::new(); // Already at finest level
        }

        let new_level = self.level - 1;
        let mut children = Vec::with_capacity(8); // 2x2x2 = 8 children

        // Subdivide in x, y (each cell splits into 2x2)
        for dy in 0..2 {
            for dx in 0..2 {
                let new_x = self.x * 2 + dx;
                let new_y = self.y * 2 + dy;

                // Check bounds
                if new_x < grids.widths[new_level] as i32 && new_y < grids.heights[new_level] as i32
                {
                    // For theta, we can keep the same bin or split if needed
                    // For simplicity, keep same theta bin at this level
                    children.push(SearchNode {
                        level: new_level,
                        x: new_x,
                        y: new_y,
                        theta_bin: self.theta_bin,
                        upper_bound: 0.0, // Will be computed by caller
                    });
                }
            }
        }

        // Also subdivide angular bins if at coarse level
        if self.level >= 2 {
            // Add adjacent theta bins at the new level
            let base_children: Vec<_> = children.clone();
            for child in base_children {
                let theta_minus = (child.theta_bin - 1).rem_euclid(num_theta_bins);
                let theta_plus = (child.theta_bin + 1).rem_euclid(num_theta_bins);

                children.push(SearchNode {
                    theta_bin: theta_minus,
                    ..child.clone()
                });
                children.push(SearchNode {
                    theta_bin: theta_plus,
                    ..child
                });
            }
        }

        children
    }
}
