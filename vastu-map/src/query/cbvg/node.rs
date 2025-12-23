//! Node types for Clearance-Based Visibility Graph.

use crate::core::Point2D;

/// Type of CBVG node for debugging and visualization.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeType {
    /// Node on Voronoi edge (medial axis of free space).
    MedialAxis,
    /// Node at Voronoi vertex (corridor junction).
    Junction,
    /// Temporary node for robot's current position.
    Robot,
    /// Temporary node for goal position.
    Goal,
}

/// A node in the clearance-based visibility graph.
///
/// Each node represents a safe waypoint that the robot can travel through,
/// with guaranteed minimum clearance from all obstacles.
#[derive(Clone, Debug)]
pub struct CBVGNode {
    /// Position in map coordinates (meters).
    pub position: Point2D,

    /// Clearance from nearest wall (meters).
    /// This is the distance to the closest obstacle.
    pub clearance: f32,

    /// Node type for debugging and visualization.
    pub node_type: NodeType,

    /// Whether this node is in a narrow passage.
    /// True if clearance < min_clearance but >= min_clearance_narrow.
    pub is_narrow_passage: bool,
}

impl CBVGNode {
    /// Create a new CBVG node.
    #[inline]
    pub fn new(position: Point2D, clearance: f32, node_type: NodeType) -> Self {
        Self {
            position,
            clearance,
            node_type,
            is_narrow_passage: false,
        }
    }

    /// Create a medial axis node.
    #[inline]
    pub fn medial_axis(position: Point2D, clearance: f32, is_narrow: bool) -> Self {
        Self {
            position,
            clearance,
            node_type: NodeType::MedialAxis,
            is_narrow_passage: is_narrow,
        }
    }

    /// Create a junction node.
    #[inline]
    pub fn junction(position: Point2D, clearance: f32) -> Self {
        Self {
            position,
            clearance,
            node_type: NodeType::Junction,
            is_narrow_passage: false,
        }
    }

    /// Create a robot position node.
    #[inline]
    pub fn robot(position: Point2D, clearance: f32) -> Self {
        Self {
            position,
            clearance,
            node_type: NodeType::Robot,
            is_narrow_passage: false,
        }
    }

    /// Create a goal position node.
    #[inline]
    pub fn goal(position: Point2D, clearance: f32) -> Self {
        Self {
            position,
            clearance,
            node_type: NodeType::Goal,
            is_narrow_passage: false,
        }
    }

    /// Check if this is a temporary node (robot or goal).
    #[inline]
    pub fn is_temporary(&self) -> bool {
        matches!(self.node_type, NodeType::Robot | NodeType::Goal)
    }

    /// Get the color for SVG visualization.
    pub fn svg_color(&self) -> &'static str {
        match self.node_type {
            NodeType::MedialAxis => {
                if self.is_narrow_passage {
                    "orange"
                } else {
                    "blue"
                }
            }
            NodeType::Junction => "green",
            NodeType::Robot => "red",
            NodeType::Goal => "purple",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_creation() {
        let node = CBVGNode::new(Point2D::new(1.0, 2.0), 0.5, NodeType::MedialAxis);
        assert_eq!(node.position.x, 1.0);
        assert_eq!(node.position.y, 2.0);
        assert_eq!(node.clearance, 0.5);
        assert_eq!(node.node_type, NodeType::MedialAxis);
        assert!(!node.is_narrow_passage);
    }

    #[test]
    fn test_medial_axis_node() {
        let node = CBVGNode::medial_axis(Point2D::new(0.0, 0.0), 0.2, true);
        assert_eq!(node.node_type, NodeType::MedialAxis);
        assert!(node.is_narrow_passage);
    }

    #[test]
    fn test_temporary_nodes() {
        let robot = CBVGNode::robot(Point2D::zero(), 0.3);
        let goal = CBVGNode::goal(Point2D::zero(), 0.3);
        let axis = CBVGNode::medial_axis(Point2D::zero(), 0.3, false);

        assert!(robot.is_temporary());
        assert!(goal.is_temporary());
        assert!(!axis.is_temporary());
    }

    #[test]
    fn test_svg_colors() {
        assert_eq!(
            CBVGNode::medial_axis(Point2D::zero(), 0.3, false).svg_color(),
            "blue"
        );
        assert_eq!(
            CBVGNode::medial_axis(Point2D::zero(), 0.2, true).svg_color(),
            "orange"
        );
        assert_eq!(
            CBVGNode::junction(Point2D::zero(), 0.3).svg_color(),
            "green"
        );
        assert_eq!(CBVGNode::robot(Point2D::zero(), 0.3).svg_color(), "red");
        assert_eq!(CBVGNode::goal(Point2D::zero(), 0.3).svg_color(), "purple");
    }
}
