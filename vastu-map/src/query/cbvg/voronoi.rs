//! Voronoi diagram approximation for medial axis computation.
//!
//! Uses point sampling along walls and computes Voronoi diagram
//! to approximate the medial axis of free space.

use crate::core::{Bounds, Point2D};
use crate::features::Line2D;

use super::clearance::{min_distance_to_walls, sample_line};
use super::config::CBVGConfig;
use super::node::{CBVGNode, NodeType};
use super::visibility::VisibilityRegion;

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
///
/// A point is on the medial axis if it's approximately equidistant from
/// its two nearest walls.
fn is_near_medial_axis(point: Point2D, lines: &[Line2D], config: &CBVGConfig) -> bool {
    if lines.len() < 2 {
        return false;
    }

    // Find distances to all walls
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

    // Point must have minimum clearance
    if d1 < config.min_clearance_narrow {
        return false;
    }

    // Check if approximately equidistant (within tolerance)
    let tolerance = config.voronoi_sample_step * 1.5;
    let diff = (d1 - d2).abs();

    // On medial axis if distances are similar
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

/// Generate nodes directly from distance field without explicit Voronoi computation.
///
/// This is a simpler alternative that samples the medial axis directly.
///
/// # Arguments
/// * `lines` - Wall segments
/// * `bounds` - Map bounds
/// * `config` - CBVG configuration
///
/// # Returns
/// Vector of CBVG nodes on the medial axis.
pub fn generate_medial_axis_nodes(
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

    // Expand bounds
    let margin = config.min_clearance;
    let expanded_bounds = Bounds {
        min: Point2D::new(bounds.min.x - margin, bounds.min.y - margin),
        max: Point2D::new(bounds.max.x + margin, bounds.max.y + margin),
    };

    let step = config.voronoi_sample_step;
    let mut nodes: Vec<CBVGNode> = Vec::new();

    let x_steps = ((expanded_bounds.max.x - expanded_bounds.min.x) / step).ceil() as usize;
    let y_steps = ((expanded_bounds.max.y - expanded_bounds.min.y) / step).ceil() as usize;

    for ix in 0..=x_steps {
        for iy in 0..=y_steps {
            let x = expanded_bounds.min.x + ix as f32 * step;
            let y = expanded_bounds.min.y + iy as f32 * step;
            let point = Point2D::new(x, y);

            // Check if on medial axis with sufficient clearance
            if is_near_medial_axis(point, lines, config) {
                let clearance = min_distance_to_walls(point, lines);

                if clearance >= config.min_clearance_narrow {
                    let is_narrow = clearance < config.min_clearance;

                    // Check for duplicates
                    let is_duplicate = nodes
                        .iter()
                        .any(|n| n.position.distance(point) < config.node_merge_distance);

                    if !is_duplicate {
                        nodes.push(CBVGNode::medial_axis(point, clearance, is_narrow));

                        if nodes.len() >= config.max_nodes {
                            return nodes;
                        }
                    }
                }
            }
        }
    }

    // Add junction nodes at corridor intersections
    add_junction_nodes(&mut nodes, lines, config);

    nodes
}

/// Add junction nodes where multiple corridors meet.
fn add_junction_nodes(nodes: &mut [CBVGNode], lines: &[Line2D], config: &CBVGConfig) {
    // Find points where 3+ walls are approximately equidistant
    // These are corridor junctions

    if lines.len() < 3 {
        return;
    }

    // For now, detect junctions as points where medial axis changes direction significantly
    // This is a simplified approach; a full implementation would use Voronoi vertices

    let mut junction_candidates: Vec<Point2D> = Vec::new();

    for node in nodes.iter() {
        // Count nearby walls at similar distances
        let clearance = node.clearance;
        let tolerance = config.voronoi_sample_step;

        let nearby_walls: usize = lines
            .iter()
            .filter(|line| {
                let dist = line.distance_to_point_segment(node.position);
                (dist - clearance).abs() < tolerance
            })
            .count();

        // Junction if equidistant from 3+ walls
        if nearby_walls >= 3 {
            junction_candidates.push(node.position);
        }
    }

    // Mark existing nodes as junctions or add new junction nodes
    for candidate in junction_candidates {
        // Find if there's an existing node nearby
        let existing = nodes
            .iter_mut()
            .find(|n| n.position.distance(candidate) < config.node_merge_distance);

        if let Some(node) = existing {
            node.node_type = NodeType::Junction;
        }
    }
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

/// Generate nodes from lidar visibility region.
///
/// Only creates nodes in areas the lidar has actually observed, with proper
/// wall clearance. Uses a grid-based approach with the coverage_node_interval
/// to ensure uniform coverage in open areas.
///
/// # Arguments
/// * `visibility` - Visibility region tracking scanned areas
/// * `lines` - Wall segments for clearance computation
/// * `config` - CBVG configuration
///
/// # Returns
/// Vector of CBVG nodes in visible areas with safe clearance.
pub fn generate_nodes_from_visibility(
    visibility: &VisibilityRegion,
    lines: &[Line2D],
    config: &CBVGConfig,
) -> Vec<CBVGNode> {
    if visibility.is_empty() || lines.is_empty() {
        return Vec::new();
    }

    let bounds = visibility.visible_bounds();
    if bounds.is_empty() {
        return Vec::new();
    }

    let step = config.coverage_node_interval;
    let mut nodes: Vec<CBVGNode> = Vec::new();

    // Jitter parameters: nodes are offset by ±30% of step size
    // This prevents systematic gaps near walls due to grid alignment
    let jitter_range = step * 0.3;

    // Grid sample within visible bounds
    let x_start = bounds.min.x;
    let x_end = bounds.max.x;
    let y_start = bounds.min.y;
    let y_end = bounds.max.y;

    let x_steps = ((x_end - x_start) / step).ceil() as usize;
    let y_steps = ((y_end - y_start) / step).ceil() as usize;

    for ix in 0..=x_steps {
        for iy in 0..=y_steps {
            // Base grid position
            let base_x = x_start + ix as f32 * step;
            let base_y = y_start + iy as f32 * step;

            // Apply deterministic jitter based on grid cell
            // Uses a simple hash to create pseudo-random but reproducible offsets
            let (jitter_x, jitter_y) = deterministic_jitter(ix, iy, jitter_range);
            let x = base_x + jitter_x;
            let y = base_y + jitter_y;
            let point = Point2D::new(x, y);

            // Must be in visible area (scanned by lidar)
            if !visibility.is_visible(point) {
                continue;
            }

            // Must have clearance from walls
            let clearance = min_distance_to_walls(point, lines);
            if clearance < config.min_clearance_narrow {
                continue;
            }

            // Check for duplicates
            let is_duplicate = nodes
                .iter()
                .any(|n| n.position.distance(point) < config.node_merge_distance);

            if is_duplicate {
                continue;
            }

            // Determine node type based on clearance
            let is_narrow = clearance < config.min_clearance;
            nodes.push(CBVGNode::medial_axis(point, clearance, is_narrow));

            if nodes.len() >= config.max_nodes {
                return nodes;
            }
        }
    }

    // Add junction nodes at corridor intersections
    add_junction_nodes(&mut nodes, lines, config);

    nodes
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
fn deterministic_jitter(ix: usize, iy: usize, range: f32) -> (f32, f32) {
    // Simple hash combining grid indices
    // Using prime multipliers for better distribution
    let hash1 = ((ix.wrapping_mul(73856093)) ^ (iy.wrapping_mul(19349663))) as u32;
    let hash2 = ((ix.wrapping_mul(83492791)) ^ (iy.wrapping_mul(41729867))) as u32;

    // Convert to [-1, 1] range using the hash bits
    let norm1 = (hash1 % 10000) as f32 / 5000.0 - 1.0;
    let norm2 = (hash2 % 10000) as f32 / 5000.0 - 1.0;

    (norm1 * range, norm2 * range)
}

/// Generate nodes combining visibility-aware and medial axis approaches.
///
/// This hybrid approach:
/// 1. Places coverage nodes uniformly in visible areas
/// 2. Adds medial axis nodes where they don't already exist
/// 3. Ensures good connectivity in narrow passages
///
/// # Arguments
/// * `visibility` - Visibility region tracking scanned areas
/// * `lines` - Wall segments
/// * `config` - CBVG configuration
///
/// # Returns
/// Vector of CBVG nodes optimized for path planning.
pub fn generate_nodes_hybrid(
    visibility: &VisibilityRegion,
    lines: &[Line2D],
    config: &CBVGConfig,
) -> Vec<CBVGNode> {
    // Start with visibility-based coverage nodes
    let mut nodes = generate_nodes_from_visibility(visibility, lines, config);

    if nodes.len() >= config.max_nodes {
        return nodes;
    }

    // Add medial axis nodes for better coverage in corridors
    // Use smaller step for more precision in narrow areas
    let corridor_config = CBVGConfig {
        voronoi_sample_step: config.voronoi_sample_step,
        ..config.clone()
    };

    let bounds = visibility.visible_bounds();
    for ix in 0..((bounds.width() / corridor_config.voronoi_sample_step).ceil() as usize) {
        for iy in 0..((bounds.height() / corridor_config.voronoi_sample_step).ceil() as usize) {
            let x = bounds.min.x + ix as f32 * corridor_config.voronoi_sample_step;
            let y = bounds.min.y + iy as f32 * corridor_config.voronoi_sample_step;
            let point = Point2D::new(x, y);

            // Only add in visible areas
            if !visibility.is_visible(point) {
                continue;
            }

            // Only add if on medial axis
            if !is_near_medial_axis(point, lines, &corridor_config) {
                continue;
            }

            let clearance = min_distance_to_walls(point, lines);
            if clearance < config.min_clearance_narrow {
                continue;
            }

            // Skip if already covered
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
    fn test_is_near_medial_axis() {
        let walls = make_corridor();
        let config = CBVGConfig::default().with_voronoi_sample_step(0.1);

        // Point in the middle of corridor should be on medial axis
        let center = Point2D::new(2.5, 0.5);
        assert!(is_near_medial_axis(center, &walls, &config));

        // Point close to a wall should not be on medial axis
        let near_wall = Point2D::new(2.5, 0.1);
        assert!(!is_near_medial_axis(near_wall, &walls, &config));
    }

    #[test]
    fn test_generate_medial_axis_nodes() {
        let walls = make_room();
        let config = CBVGConfig::default()
            .with_voronoi_sample_step(0.2)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18);

        let nodes = generate_medial_axis_nodes(&walls, None, &config);

        // Should have nodes in the room
        assert!(!nodes.is_empty());

        // All nodes should have sufficient clearance
        for node in &nodes {
            assert!(node.clearance >= config.min_clearance_narrow);
        }

        // Most nodes should be near the center of the room
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
        let nodes = generate_medial_axis_nodes(&[], None, &config);
        assert!(nodes.is_empty());
    }

    #[test]
    fn test_generate_nodes_from_visibility() {
        use crate::core::Pose2D;

        let walls = make_room();
        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.5)
            .with_min_clearance(0.3)
            .with_min_clearance_narrow(0.18);

        // Create visibility region with scan from center of room
        let mut visibility = VisibilityRegion::new(0.1);
        let robot_pose = Pose2D::new(2.0, 1.5, 0.0);

        // Simulate a circular scan around the robot
        let scan_points: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                let dist = 1.5; // 1.5m range
                robot_pose.transform_point(Point2D::from_polar(angle, dist))
            })
            .collect();

        visibility.update_from_scan(robot_pose, &scan_points);

        let nodes = generate_nodes_from_visibility(&visibility, &walls, &config);

        // Should have nodes
        assert!(!nodes.is_empty());

        // All nodes should be in visible area
        for node in &nodes {
            assert!(visibility.is_visible(node.position));
        }

        // All nodes should have sufficient clearance
        for node in &nodes {
            assert!(node.clearance >= config.min_clearance_narrow);
        }
    }

    #[test]
    fn test_visibility_nodes_only_in_scanned_area() {
        use crate::core::Pose2D;

        // Create a large room
        let walls = vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(10.0, 0.0)),
            Line2D::new(Point2D::new(10.0, 0.0), Point2D::new(10.0, 10.0)),
            Line2D::new(Point2D::new(10.0, 10.0), Point2D::new(0.0, 10.0)),
            Line2D::new(Point2D::new(0.0, 10.0), Point2D::new(0.0, 0.0)),
        ];

        let config = CBVGConfig::default()
            .with_coverage_node_interval(0.5)
            .with_min_clearance(0.3);

        // Scan only the bottom-left corner
        let mut visibility = VisibilityRegion::new(0.1);
        let robot_pose = Pose2D::new(2.0, 2.0, 0.0);

        let scan_points: Vec<Point2D> = (0..36)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::PI / 18.0;
                let dist = 1.5;
                robot_pose.transform_point(Point2D::from_polar(angle, dist))
            })
            .collect();

        visibility.update_from_scan(robot_pose, &scan_points);

        let nodes = generate_nodes_from_visibility(&visibility, &walls, &config);

        // All nodes should be near the robot (not in unexplored far corner)
        for node in &nodes {
            assert!(
                node.position.distance(robot_pose.position()) < 3.0,
                "Node at {:?} is too far from scanned area",
                node.position
            );
        }
    }
}
