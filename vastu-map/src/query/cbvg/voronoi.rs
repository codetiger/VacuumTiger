//! Voronoi diagram approximation for medial axis computation.
//!
//! Uses point sampling along walls and computes Voronoi diagram
//! to approximate the medial axis of free space.

use crate::core::{Bounds, Point2D};
use crate::features::Line2D;

use super::clearance::{min_distance_to_walls, sample_line};
use super::config::CBVGConfig;
use super::node::CBVGNode;

/// A Voronoi edge connecting two points in free space.
#[derive(Clone, Debug)]
pub struct VoronoiEdge {
    /// Start point of the edge.
    pub start: Point2D,
    /// End point of the edge.
    pub end: Point2D,
    /// Indices of the two generating wall points.
    pub generators: (usize, usize),
}

impl VoronoiEdge {
    /// Create a new Voronoi edge.
    pub fn new(start: Point2D, end: Point2D, gen1: usize, gen2: usize) -> Self {
        Self {
            start,
            end,
            generators: (gen1, gen2),
        }
    }

    /// Get the length of this edge.
    #[inline]
    pub fn length(&self) -> f32 {
        self.start.distance(self.end)
    }

    /// Get a point along the edge at parameter t (0=start, 1=end).
    #[inline]
    pub fn point_at(&self, t: f32) -> Point2D {
        self.start.lerp(self.end, t)
    }
}

/// Sample points along all wall segments.
///
/// # Arguments
/// * `lines` - Wall segments
/// * `sample_step` - Distance between samples
///
/// # Returns
/// Vector of sampled wall points.
pub fn sample_wall_points(lines: &[Line2D], sample_step: f32) -> Vec<Point2D> {
    lines
        .iter()
        .flat_map(|line| sample_line(line, sample_step))
        .collect()
}

/// Compute approximate Voronoi edges using distance field approach.
///
/// This is a simplified implementation that:
/// 1. Samples points along walls
/// 2. Finds points equidistant from two walls (approximate Voronoi vertices)
/// 3. Connects them to form Voronoi edges
///
/// # Arguments
/// * `lines` - Wall segments
/// * `bounds` - Map bounds for grid sampling
/// * `config` - CBVG configuration
///
/// # Returns
/// Vector of Voronoi edges in free space.
pub fn compute_voronoi_edges(
    lines: &[Line2D],
    bounds: Option<&Bounds>,
    config: &CBVGConfig,
) -> Vec<VoronoiEdge> {
    if lines.is_empty() {
        return Vec::new();
    }

    // Get bounds for sampling
    let bounds = match bounds {
        Some(b) => *b,
        None => compute_bounds_from_lines(lines),
    };

    // Expand bounds slightly to ensure coverage
    let margin = config.min_clearance * 2.0;
    let expanded_bounds = Bounds {
        min: Point2D::new(bounds.min.x - margin, bounds.min.y - margin),
        max: Point2D::new(bounds.max.x + margin, bounds.max.y + margin),
    };

    // Use grid-based approach to find medial axis points
    let step = config.voronoi_sample_step;
    let mut medial_points: Vec<Point2D> = Vec::new();

    let x_steps = ((expanded_bounds.max.x - expanded_bounds.min.x) / step).ceil() as usize;
    let y_steps = ((expanded_bounds.max.y - expanded_bounds.min.y) / step).ceil() as usize;

    for ix in 0..=x_steps {
        for iy in 0..=y_steps {
            let x = expanded_bounds.min.x + ix as f32 * step;
            let y = expanded_bounds.min.y + iy as f32 * step;
            let point = Point2D::new(x, y);

            // Check if this point is on the medial axis
            if is_near_medial_axis(point, lines, config) {
                let clearance = min_distance_to_walls(point, lines);
                if clearance >= config.min_clearance_narrow {
                    medial_points.push(point);
                }
            }
        }
    }

    // Connect nearby medial points to form edges
    connect_medial_points(&medial_points, lines, config)
}

/// Check if a point is near the medial axis (equidistant from multiple walls).
fn is_near_medial_axis(point: Point2D, lines: &[Line2D], config: &CBVGConfig) -> bool {
    if lines.len() < 2 {
        return false;
    }

    let mut distances: Vec<(f32, usize)> = lines
        .iter()
        .enumerate()
        .map(|(idx, line)| (line.distance_to_point_segment(point), idx))
        .collect();

    distances.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    if distances.len() < 2 {
        return false;
    }

    let d1 = distances[0].0;
    let d2 = distances[1].0;

    if d1 < config.min_clearance_narrow {
        return false;
    }

    let tolerance = config.voronoi_sample_step * 1.5;
    let diff = (d1 - d2).abs();
    diff < tolerance
}

/// Connect medial axis points to form edges.
fn connect_medial_points(
    points: &[Point2D],
    lines: &[Line2D],
    config: &CBVGConfig,
) -> Vec<VoronoiEdge> {
    let mut edges = Vec::new();
    let max_connect_dist = config.voronoi_sample_step * 2.0;

    for i in 0..points.len() {
        for j in (i + 1)..points.len() {
            let dist = points[i].distance(points[j]);

            // Only connect nearby points
            if dist <= max_connect_dist {
                // Verify the connection is valid (no walls in between)
                let midpoint = points[i].lerp(points[j], 0.5);
                let mid_clearance = min_distance_to_walls(midpoint, lines);

                if mid_clearance >= config.min_clearance_narrow {
                    edges.push(VoronoiEdge::new(points[i], points[j], i, j));
                }
            }
        }
    }

    edges
}

/// Sample nodes along Voronoi edges (medial axis).
///
/// # Arguments
/// * `edges` - Voronoi edges
/// * `lines` - Wall segments (for clearance computation)
/// * `config` - CBVG configuration
///
/// # Returns
/// Vector of CBVG nodes sampled along the medial axis.
pub fn sample_medial_axis_nodes(
    edges: &[VoronoiEdge],
    lines: &[Line2D],
    config: &CBVGConfig,
) -> Vec<CBVGNode> {
    let mut nodes = Vec::new();
    let mut seen_positions: Vec<Point2D> = Vec::new();

    for edge in edges {
        let edge_length = edge.length();
        let num_samples = (edge_length / config.voronoi_sample_step).ceil() as usize;
        let num_samples = num_samples.max(1);

        for i in 0..=num_samples {
            let t = i as f32 / num_samples as f32;
            let position = edge.point_at(t);

            // Skip if too close to existing node
            let is_duplicate = seen_positions
                .iter()
                .any(|p| p.distance(position) < config.node_merge_distance);

            if is_duplicate {
                continue;
            }

            let clearance = min_distance_to_walls(position, lines);

            // Skip if insufficient clearance
            if clearance < config.min_clearance_narrow {
                continue;
            }

            let is_narrow = clearance < config.min_clearance;

            nodes.push(CBVGNode::medial_axis(position, clearance, is_narrow));
            seen_positions.push(position);

            // Limit total nodes
            if nodes.len() >= config.max_nodes {
                return nodes;
            }
        }
    }

    nodes
}

/// Generate nodes on a uniform grid with clearance checks.
///
/// This replaces medial axis generation with simple grid sampling:
/// 1. Sample points on a uniform grid (coverage_node_interval spacing)
/// 2. Keep only points with sufficient clearance from walls
/// 3. Apply deterministic jitter for better distribution
///
/// The graph's optimize_nodes() should be called after to reduce density
/// by merging visibility cliques into centroids.
///
/// # Arguments
/// * `lines` - Wall segments
/// * `bounds` - Map bounds
/// * `config` - CBVG configuration
///
/// # Returns
/// Vector of CBVG nodes uniformly distributed in free space.
pub fn generate_grid_nodes(
    lines: &[Line2D],
    bounds: Option<&Bounds>,
    config: &CBVGConfig,
) -> Vec<CBVGNode> {
    if lines.is_empty() {
        return Vec::new();
    }

    let bounds = match bounds {
        Some(b) => *b,
        None => compute_bounds_from_lines(lines),
    };

    // Expand bounds slightly to ensure edge coverage
    let margin = config.min_clearance;
    let expanded_bounds = Bounds {
        min: Point2D::new(bounds.min.x - margin, bounds.min.y - margin),
        max: Point2D::new(bounds.max.x + margin, bounds.max.y + margin),
    };

    let step = config.coverage_node_interval;
    let jitter_range = step * 0.3; // ±30% jitter for better distribution
    let mut nodes: Vec<CBVGNode> = Vec::new();

    let x_steps = ((expanded_bounds.max.x - expanded_bounds.min.x) / step).ceil() as usize;
    let y_steps = ((expanded_bounds.max.y - expanded_bounds.min.y) / step).ceil() as usize;

    for ix in 0..=x_steps {
        for iy in 0..=y_steps {
            // Base grid position with deterministic jitter
            let base_x = expanded_bounds.min.x + ix as f32 * step;
            let base_y = expanded_bounds.min.y + iy as f32 * step;
            let (jitter_x, jitter_y) = deterministic_jitter(ix, iy, jitter_range);
            let point = Point2D::new(base_x + jitter_x, base_y + jitter_y);

            // Check clearance from walls
            let clearance = min_distance_to_walls(point, lines);
            if clearance < config.min_clearance_narrow {
                continue;
            }

            // Skip duplicates
            let is_duplicate = nodes
                .iter()
                .any(|n| n.position.distance(point) < config.node_merge_distance);

            if is_duplicate {
                continue;
            }

            let is_narrow = clearance < config.min_clearance;
            nodes.push(CBVGNode::medial_axis(point, clearance, is_narrow));

            if nodes.len() >= config.max_nodes {
                return nodes;
            }
        }
    }

    nodes
}

/// Compute bounds from line segments.
fn compute_bounds_from_lines(lines: &[Line2D]) -> Bounds {
    if lines.is_empty() {
        return Bounds::empty();
    }

    let mut min = Point2D::new(f32::INFINITY, f32::INFINITY);
    let mut max = Point2D::new(f32::NEG_INFINITY, f32::NEG_INFINITY);

    for line in lines {
        min = min.min(line.start).min(line.end);
        max = max.max(line.start).max(line.end);
    }

    Bounds { min, max }
}

/// Generate deterministic jitter for a grid cell.
///
/// Uses a simple hash function to create pseudo-random but reproducible
/// offsets for each grid cell. This ensures the same grid always produces
/// the same node positions.
///
/// # Arguments
/// * `ix` - Grid x index
/// * `iy` - Grid y index
/// * `range` - Maximum jitter magnitude (±range)
///
/// # Returns
/// Tuple of (x_offset, y_offset) in range [-range, +range]
pub fn deterministic_jitter(ix: usize, iy: usize, range: f32) -> (f32, f32) {
    // Simple hash combining grid indices
    // Using prime multipliers for better distribution
    let hash1 = ((ix.wrapping_mul(73856093)) ^ (iy.wrapping_mul(19349663))) as u32;
    let hash2 = ((ix.wrapping_mul(83492791)) ^ (iy.wrapping_mul(41729867))) as u32;

    // Convert to [-1, 1] range using the hash bits
    let norm1 = (hash1 % 10000) as f32 / 5000.0 - 1.0;
    let norm2 = (hash2 % 10000) as f32 / 5000.0 - 1.0;

    (norm1 * range, norm2 * range)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_corridor() -> Vec<Line2D> {
        // Simple corridor: two parallel walls
        vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0)), // Bottom wall
            Line2D::new(Point2D::new(0.0, 1.0), Point2D::new(5.0, 1.0)), // Top wall
        ]
    }

    fn make_room() -> Vec<Line2D> {
        // Simple room: 4 walls
        vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(4.0, 0.0)), // Bottom
            Line2D::new(Point2D::new(4.0, 0.0), Point2D::new(4.0, 3.0)), // Right
            Line2D::new(Point2D::new(4.0, 3.0), Point2D::new(0.0, 3.0)), // Top
            Line2D::new(Point2D::new(0.0, 3.0), Point2D::new(0.0, 0.0)), // Left
        ]
    }

    #[test]
    fn test_deterministic_jitter() {
        let range = 0.15; // ±0.15m jitter

        // Test that jitter is deterministic (same input = same output)
        let (j1x, j1y) = deterministic_jitter(5, 10, range);
        let (j2x, j2y) = deterministic_jitter(5, 10, range);
        assert_eq!(j1x, j2x);
        assert_eq!(j1y, j2y);

        // Test that different cells get different jitter
        let (j3x, j3y) = deterministic_jitter(5, 11, range);
        assert!(
            j1x != j3x || j1y != j3y,
            "Adjacent cells should have different jitter"
        );

        // Test that jitter is within bounds
        for ix in 0..20 {
            for iy in 0..20 {
                let (jx, jy) = deterministic_jitter(ix, iy, range);
                assert!(jx.abs() <= range, "X jitter {} exceeds range {}", jx, range);
                assert!(jy.abs() <= range, "Y jitter {} exceeds range {}", jy, range);
            }
        }
    }

    #[test]
    fn test_sample_wall_points() {
        let walls = make_corridor();
        let points = sample_wall_points(&walls, 1.0);

        // Should have points from both walls
        assert!(!points.is_empty());
        assert!(points.len() >= 10); // At least 5 per wall
    }

    #[test]
    fn test_generate_grid_nodes() {
        let walls = make_room();
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.3)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18);

        let nodes = generate_grid_nodes(&walls, None, &config);

        // Should have nodes in the room
        assert!(!nodes.is_empty());

        // All nodes should have sufficient clearance
        for node in &nodes {
            assert!(node.clearance >= config.min_clearance_narrow);
        }

        // Should have nodes near the center of the room
        let center = Point2D::new(2.0, 1.5);
        let near_center: usize = nodes
            .iter()
            .filter(|n| n.position.distance(center) < 1.5)
            .count();
        assert!(near_center > 0);
    }

    #[test]
    fn test_compute_bounds_from_lines() {
        let walls = make_room();
        let bounds = compute_bounds_from_lines(&walls);

        assert!((bounds.min.x - 0.0).abs() < 0.01);
        assert!((bounds.min.y - 0.0).abs() < 0.01);
        assert!((bounds.max.x - 4.0).abs() < 0.01);
        assert!((bounds.max.y - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_empty_walls() {
        let config = CBVGConfig::default();
        let nodes = generate_grid_nodes(&[], None, &config);
        assert!(nodes.is_empty());
    }
}
