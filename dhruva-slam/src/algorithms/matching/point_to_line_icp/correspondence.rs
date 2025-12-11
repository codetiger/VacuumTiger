//! Correspondence finding for point-to-line ICP.

use kiddo::{KdTree, SquaredEuclidean};

use super::config::PointToLineIcpConfig;
use super::line2d::Line2D;
use crate::core::types::{Point2D, PointCloud2D};

/// Correspondence between a source point and a target line.
#[derive(Debug)]
pub struct Correspondence {
    /// Index in source cloud
    pub source_idx: usize,
    /// Target line (fitted from neighbors)
    pub target_line: Line2D,
    /// Squared distance to line
    pub distance_sq: f32,
}

/// Find correspondences with line fitting.
///
/// For each point in the transformed source cloud, finds k nearest neighbors
/// in the target and fits a line through them. Only correspondences with
/// good line quality are kept.
///
/// # Arguments
/// * `config` - ICP configuration
/// * `transformed_source` - Pre-transformed source cloud
/// * `target` - Target point cloud
/// * `target_tree` - K-d tree built from target
/// * `output` - Buffer to store correspondences (cleared before filling)
pub fn find_correspondences_into(
    config: &PointToLineIcpConfig,
    transformed_source: &PointCloud2D,
    target: &PointCloud2D,
    target_tree: &KdTree<f32, 2>,
    output: &mut Vec<Correspondence>,
) {
    output.clear();
    let max_dist_sq = config.max_correspondence_distance.powi(2);

    for i in 0..transformed_source.len() {
        // Use pre-transformed source point
        let tx = transformed_source.xs[i];
        let ty = transformed_source.ys[i];
        let transformed = Point2D::new(tx, ty);

        // Find k nearest neighbors in target
        let neighbors = target_tree.nearest_n::<SquaredEuclidean>(&[tx, ty], config.line_neighbors);

        if neighbors.is_empty() {
            continue;
        }

        // Check if closest is within range
        let closest_dist_sq = neighbors[0].distance;
        if closest_dist_sq > max_dist_sq {
            continue;
        }

        // Gather neighbor points for line fitting using stack array
        const MAX_LINE_NEIGHBORS: usize = 16;
        let mut neighbor_points: [Point2D; MAX_LINE_NEIGHBORS] =
            [Point2D::default(); MAX_LINE_NEIGHBORS];
        let mut neighbor_count = 0;

        for n in neighbors.iter() {
            if n.distance <= max_dist_sq * 4.0 && neighbor_count < MAX_LINE_NEIGHBORS {
                neighbor_points[neighbor_count] = target.point_at(n.item as usize);
                neighbor_count += 1;
            }
        }

        if neighbor_count < 2 {
            continue;
        }

        // Fit line through neighbors
        if let Some(line) = Line2D::fit(&neighbor_points[..neighbor_count]) {
            if line.quality < config.min_line_quality {
                continue;
            }

            // Compute point-to-line distance
            let dist_to_line = line.distance(&transformed);
            let dist_sq = dist_to_line * dist_to_line;

            output.push(Correspondence {
                source_idx: i,
                target_line: line,
                distance_sq: dist_sq,
            });
        }
    }

    // Apply outlier rejection
    if config.outlier_ratio > 0.0 && !output.is_empty() {
        output.sort_by(|a, b| a.distance_sq.partial_cmp(&b.distance_sq).unwrap());

        let keep_count = ((1.0 - config.outlier_ratio) * output.len() as f32) as usize;
        output.truncate(keep_count.max(config.min_correspondences));
    }
}
