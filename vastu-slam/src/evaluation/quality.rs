//! Map and loop closure quality metrics.
//!
//! This module provides quality metrics that extend beyond accuracy and performance:
//!
//! - **Loop Closure Quality**: Precision/Recall for loop closure detection (RTAB-Map style)
//! - **Map Quality**: Consistency, coverage, and sharpness metrics

use crate::{CellCounts, GridStorage, Pose2D};
use serde::{Deserialize, Serialize};

/// Loop closure precision/recall metrics.
///
/// Evaluates the quality of loop closure detection by comparing against ground truth.
/// Based on RTAB-Map's benchmark methodology.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LoopClosurePR {
    /// True positives: correctly detected loop closures.
    pub true_positives: usize,

    /// False positives: incorrectly detected loop closures.
    pub false_positives: usize,

    /// False negatives: missed loop closures.
    pub false_negatives: usize,

    /// Precision: TP / (TP + FP).
    pub precision: f32,

    /// Recall: TP / (TP + FN).
    pub recall: f32,

    /// F1 score: harmonic mean of precision and recall.
    pub f1_score: f32,

    /// Maximum recall at 100% precision (key benchmark metric).
    pub max_recall_at_100_precision: f32,
}

impl LoopClosurePR {
    /// Compute precision/recall from detections and ground truth.
    ///
    /// # Arguments
    ///
    /// * `detections` - Detected loop closures: (from_node, to_node, score)
    /// * `ground_truth` - Ground truth loop closures
    /// * `tolerance` - Position tolerance for matching (meters)
    pub fn compute(
        detections: &[(usize, usize, f32)],
        ground_truth: &LoopClosureGroundTruth,
        tolerance: f32,
    ) -> Self {
        if ground_truth.closures.is_empty() && detections.is_empty() {
            return Self {
                precision: 1.0,
                recall: 1.0,
                f1_score: 1.0,
                max_recall_at_100_precision: 1.0,
                ..Default::default()
            };
        }

        // Sort detections by score (descending) for PR curve
        let mut sorted_detections: Vec<_> = detections.to_vec();
        sorted_detections
            .sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));

        let mut tp = 0usize;
        let mut fp = 0usize;
        let mut matched_gt: Vec<bool> = vec![false; ground_truth.closures.len()];

        // Match detections to ground truth
        for (from, to, _score) in &sorted_detections {
            let mut is_tp = false;

            // Check if this detection matches any ground truth
            for (i, (gt_from, gt_to)) in ground_truth.closures.iter().enumerate() {
                if matched_gt[i] {
                    continue;
                }

                // Check if nodes match (with some tolerance for node indices)
                let from_matches = (*from as i32 - *gt_from as i32).abs() <= 1;
                let to_matches = (*to as i32 - *gt_to as i32).abs() <= 1;

                // If we have poses, also check spatial proximity
                let spatially_close =
                    if let (Some(poses), _) = (&ground_truth.poses, tolerance > 0.0) {
                        if *from < poses.len() && *gt_from < poses.len() {
                            poses[*from].distance(&poses[*gt_from]) < tolerance
                        } else {
                            true
                        }
                    } else {
                        true
                    };

                if (from_matches && to_matches) || spatially_close {
                    matched_gt[i] = true;
                    is_tp = true;
                    break;
                }
            }

            if is_tp {
                tp += 1;
            } else {
                fp += 1;
            }
        }

        let fn_count = matched_gt.iter().filter(|&&m| !m).count();

        let precision = if tp + fp > 0 {
            tp as f32 / (tp + fp) as f32
        } else {
            1.0
        };

        let recall = if tp + fn_count > 0 {
            tp as f32 / (tp + fn_count) as f32
        } else {
            1.0
        };

        let f1_score = if precision + recall > 0.0 {
            2.0 * precision * recall / (precision + recall)
        } else {
            0.0
        };

        // Compute max recall at 100% precision
        // Walk through sorted detections until we hit a false positive
        let max_recall_at_100_precision = {
            let mut running_tp = 0usize;
            let running_fp = 0usize;
            let mut best_recall = 0.0f32;
            let mut matched: Vec<bool> = vec![false; ground_truth.closures.len()];

            for (from, to, _score) in &sorted_detections {
                let mut is_tp = false;

                for (i, (gt_from, gt_to)) in ground_truth.closures.iter().enumerate() {
                    if matched[i] {
                        continue;
                    }
                    let from_matches = (*from as i32 - *gt_from as i32).abs() <= 1;
                    let to_matches = (*to as i32 - *gt_to as i32).abs() <= 1;

                    if from_matches && to_matches {
                        matched[i] = true;
                        is_tp = true;
                        break;
                    }
                }

                if is_tp {
                    running_tp += 1;
                    let current_recall = running_tp as f32 / ground_truth.closures.len() as f32;
                    let current_precision = running_tp as f32 / (running_tp + running_fp) as f32;
                    if current_precision >= 0.9999 {
                        best_recall = current_recall;
                    }
                } else {
                    // Once we hit a FP, precision < 100%, so stop
                    break;
                }
            }

            best_recall
        };

        Self {
            true_positives: tp,
            false_positives: fp,
            false_negatives: fn_count,
            precision,
            recall,
            f1_score,
            max_recall_at_100_precision,
        }
    }

    /// Print metrics.
    pub fn print(&self) {
        println!("=== Loop Closure Quality ===");
        println!("True Positives:  {}", self.true_positives);
        println!("False Positives: {}", self.false_positives);
        println!("False Negatives: {}", self.false_negatives);
        println!("Precision:       {:.2}%", self.precision * 100.0);
        println!("Recall:          {:.2}%", self.recall * 100.0);
        println!("F1 Score:        {:.2}%", self.f1_score * 100.0);
        println!(
            "Max Recall @100% Precision: {:.2}%",
            self.max_recall_at_100_precision * 100.0
        );
    }
}

/// Ground truth for loop closure evaluation.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LoopClosureGroundTruth {
    /// List of (from_node, to_node) pairs that should match.
    pub closures: Vec<(usize, usize)>,

    /// Optional: poses for spatial matching.
    #[serde(skip)]
    pub poses: Option<Vec<Pose2D>>,
}

impl LoopClosureGroundTruth {
    /// Create from a list of closure pairs.
    pub fn from_pairs(closures: Vec<(usize, usize)>) -> Self {
        Self {
            closures,
            poses: None,
        }
    }

    /// Create with poses for spatial matching.
    pub fn with_poses(closures: Vec<(usize, usize)>, poses: Vec<Pose2D>) -> Self {
        Self {
            closures,
            poses: Some(poses),
        }
    }

    /// Generate ground truth from trajectory based on spatial proximity.
    ///
    /// Two nodes form a loop closure if:
    /// 1. They are at least `min_separation` nodes apart in the sequence
    /// 2. They are within `max_distance` meters of each other
    pub fn from_trajectory(poses: &[Pose2D], min_separation: usize, max_distance: f32) -> Self {
        let mut closures = Vec::new();

        for i in 0..poses.len() {
            for j in (i + min_separation)..poses.len() {
                if poses[i].distance(&poses[j]) < max_distance {
                    closures.push((i, j));
                }
            }
        }

        Self {
            closures,
            poses: Some(poses.to_vec()),
        }
    }
}

/// Map quality metrics.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct MapQualityMetrics {
    /// Explored area in square meters.
    pub coverage_m2: f32,

    /// Wall sharpness (0-1, higher is better).
    /// Measures how well-defined wall boundaries are.
    pub wall_sharpness: f32,

    /// Submap consistency (0-1, higher is better).
    /// Measures agreement at submap boundaries.
    pub submap_consistency: f32,

    /// Cell type distribution.
    pub cell_counts: CellCounts,

    /// Percentage of cells that are known.
    pub known_percentage: f32,

    /// Average obstacle thickness (cells).
    pub avg_wall_thickness: f32,
}

impl MapQualityMetrics {
    /// Compute quality metrics from a grid.
    pub fn compute(grid: &GridStorage) -> Self {
        let cell_counts = grid.count_by_type();
        let resolution = grid.resolution();

        let total_cells = cell_counts.total();
        let known_cells = cell_counts.known();

        let known_percentage = if total_cells > 0 {
            known_cells as f32 / total_cells as f32 * 100.0
        } else {
            0.0
        };

        let coverage_m2 = known_cells as f32 * resolution * resolution;

        // Compute wall sharpness
        let wall_sharpness = Self::compute_wall_sharpness(grid);

        // Compute average wall thickness
        let avg_wall_thickness = Self::compute_avg_wall_thickness(grid);

        Self {
            coverage_m2,
            wall_sharpness,
            submap_consistency: 1.0, // Placeholder - would need submap data
            cell_counts,
            known_percentage,
            avg_wall_thickness,
        }
    }

    /// Compute wall sharpness by looking at transitions between wall and floor.
    fn compute_wall_sharpness(grid: &GridStorage) -> f32 {
        use crate::CellType;

        let width = grid.width();
        let height = grid.height();

        if width < 3 || height < 3 {
            return 1.0;
        }

        let mut sharp_transitions = 0usize;
        let mut total_wall_cells = 0usize;

        for y in 1..(height - 1) {
            for x in 1..(width - 1) {
                let coord = crate::GridCoord::new(x as i32, y as i32);
                let cell = grid.get(coord);

                if let Some(c) = cell
                    && c.cell_type == CellType::Wall
                {
                    total_wall_cells += 1;

                    // Check 4-connected neighbors
                    let neighbors = [
                        crate::GridCoord::new(x as i32 - 1, y as i32),
                        crate::GridCoord::new(x as i32 + 1, y as i32),
                        crate::GridCoord::new(x as i32, y as i32 - 1),
                        crate::GridCoord::new(x as i32, y as i32 + 1),
                    ];

                    let floor_neighbors = neighbors
                        .iter()
                        .filter_map(|&nc| grid.get(nc))
                        .filter(|nc| nc.cell_type == CellType::Floor)
                        .count();

                    // Wall cell with floor neighbor = sharp boundary
                    if floor_neighbors > 0 && floor_neighbors < 4 {
                        sharp_transitions += 1;
                    }
                }
            }
        }

        if total_wall_cells == 0 {
            return 1.0;
        }

        // Ratio of wall cells that form sharp boundaries
        sharp_transitions as f32 / total_wall_cells as f32
    }

    /// Compute average wall thickness.
    fn compute_avg_wall_thickness(grid: &GridStorage) -> f32 {
        use crate::CellType;

        let width = grid.width();
        let height = grid.height();

        let mut thicknesses = Vec::new();

        // Scan horizontally
        for y in 0..height {
            let mut wall_run = 0;
            for x in 0..width {
                let coord = crate::GridCoord::new(x as i32, y as i32);
                let is_wall = grid
                    .get(coord)
                    .map(|c| c.cell_type == CellType::Wall)
                    .unwrap_or(false);

                if is_wall {
                    wall_run += 1;
                } else if wall_run > 0 {
                    thicknesses.push(wall_run as f32);
                    wall_run = 0;
                }
            }
            if wall_run > 0 {
                thicknesses.push(wall_run as f32);
            }
        }

        // Scan vertically
        for x in 0..width {
            let mut wall_run = 0;
            for y in 0..height {
                let coord = crate::GridCoord::new(x as i32, y as i32);
                let is_wall = grid
                    .get(coord)
                    .map(|c| c.cell_type == CellType::Wall)
                    .unwrap_or(false);

                if is_wall {
                    wall_run += 1;
                } else if wall_run > 0 {
                    thicknesses.push(wall_run as f32);
                    wall_run = 0;
                }
            }
            if wall_run > 0 {
                thicknesses.push(wall_run as f32);
            }
        }

        if thicknesses.is_empty() {
            return 0.0;
        }

        thicknesses.iter().sum::<f32>() / thicknesses.len() as f32
    }

    /// Print metrics.
    pub fn print(&self) {
        println!("=== Map Quality Metrics ===");
        println!("Coverage:        {:.2} m²", self.coverage_m2);
        println!("Known cells:     {:.1}%", self.known_percentage);
        println!("Wall sharpness:  {:.2}", self.wall_sharpness);
        println!("Avg wall thickness: {:.2} cells", self.avg_wall_thickness);
        println!();
        println!("Cell distribution:");
        println!("  Floor:   {}", self.cell_counts.floor);
        println!("  Wall:    {}", self.cell_counts.wall);
        println!("  Cliff:   {}", self.cell_counts.cliff);
        println!("  Bump:    {}", self.cell_counts.bump);
        println!("  Unknown: {}", self.cell_counts.unknown);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_loop_closure_pr_perfect() {
        let ground_truth = LoopClosureGroundTruth::from_pairs(vec![(0, 10), (5, 15)]);
        let detections = vec![(0, 10, 0.9), (5, 15, 0.8)];

        let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);

        assert_eq!(pr.true_positives, 2);
        assert_eq!(pr.false_positives, 0);
        assert_eq!(pr.false_negatives, 0);
        assert!((pr.precision - 1.0).abs() < 1e-5);
        assert!((pr.recall - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_loop_closure_pr_with_fp() {
        let ground_truth = LoopClosureGroundTruth::from_pairs(vec![(0, 10)]);
        let detections = vec![(0, 10, 0.9), (1, 20, 0.5)]; // Second is FP

        let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);

        assert_eq!(pr.true_positives, 1);
        assert_eq!(pr.false_positives, 1);
        assert!((pr.precision - 0.5).abs() < 1e-5);
        assert!((pr.recall - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_loop_closure_pr_with_fn() {
        let ground_truth = LoopClosureGroundTruth::from_pairs(vec![(0, 10), (5, 15)]);
        let detections = vec![(0, 10, 0.9)]; // Missing second closure

        let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);

        assert_eq!(pr.true_positives, 1);
        assert_eq!(pr.false_negatives, 1);
        assert!((pr.precision - 1.0).abs() < 1e-5);
        assert!((pr.recall - 0.5).abs() < 1e-5);
    }

    #[test]
    fn test_ground_truth_from_trajectory() {
        let poses = vec![
            Pose2D::new(0.0, 0.0, 0.0),
            Pose2D::new(1.0, 0.0, 0.0),
            Pose2D::new(2.0, 0.0, 0.0),
            Pose2D::new(2.0, 1.0, 0.0),
            Pose2D::new(1.0, 1.0, 0.0),
            Pose2D::new(0.0, 1.0, 0.0),
            Pose2D::new(0.0, 0.5, 0.0), // Close to pose[0]
        ];

        let gt = LoopClosureGroundTruth::from_trajectory(&poses, 3, 1.0);

        // Should detect closure between pose[0] and pose[6]
        assert!(!gt.closures.is_empty());
        assert!(gt.closures.iter().any(|&(a, b)| a == 0 && b == 6));
    }

    #[test]
    fn test_empty_metrics() {
        let ground_truth = LoopClosureGroundTruth::from_pairs(vec![]);
        let detections: Vec<(usize, usize, f32)> = vec![];

        let pr = LoopClosurePR::compute(&detections, &ground_truth, 1.0);

        assert!((pr.precision - 1.0).abs() < 1e-5);
        assert!((pr.recall - 1.0).abs() < 1e-5);
    }
}
