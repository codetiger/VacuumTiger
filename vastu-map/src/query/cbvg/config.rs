//! Configuration for Clearance-Based Visibility Graph.

use serde::{Deserialize, Serialize};

/// Configuration for clearance-based visibility graph path planning.
///
/// The CBVG places nodes along the medial axis of free space, ensuring
/// all waypoints are at a safe distance from obstacles.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CBVGConfig {
    /// Minimum clearance from walls in open areas (meters).
    /// Nodes will be placed at least this distance from any wall.
    /// Default: 0.3
    pub min_clearance: f32,

    /// Minimum clearance in narrow passages (meters).
    /// Used when corridor is too narrow for full clearance.
    /// Default: 0.18 (robot_radius + 0.02)
    pub min_clearance_narrow: f32,

    /// Sample resolution along Voronoi edges (meters).
    /// Controls density of nodes along the medial axis.
    /// Default: 0.1
    pub voronoi_sample_step: f32,

    /// Maximum edge length between nodes (meters).
    /// Longer edges are split with intermediate nodes.
    /// Default: 0.8
    pub max_edge_length: f32,

    /// Interval for placing coverage nodes in open areas (meters).
    /// Ensures nodes exist even in areas without medial axis.
    /// Default: 0.5
    pub coverage_node_interval: f32,

    /// Distance threshold for merging nearby nodes (meters).
    /// Nodes closer than this are merged to reduce graph size.
    /// Default: 0.15
    pub node_merge_distance: f32,

    /// Maximum nodes in the graph.
    /// Limits computation for very complex maps.
    /// Default: 500
    pub max_nodes: usize,

    /// Clearance sample step for edge validation (meters).
    /// Points are sampled along edges to verify clearance.
    /// Default: 0.05
    pub edge_clearance_step: f32,

    /// Wall sampling step for Voronoi approximation (meters).
    /// Points are sampled along walls for Voronoi computation.
    /// Default: 0.1
    pub wall_sample_step: f32,
}

impl Default for CBVGConfig {
    fn default() -> Self {
        Self {
            min_clearance: 0.3,
            min_clearance_narrow: 0.18,
            voronoi_sample_step: 0.1,
            max_edge_length: 0.8,
            coverage_node_interval: 0.35,
            node_merge_distance: 0.15,
            max_nodes: 500,
            edge_clearance_step: 0.05,
            wall_sample_step: 0.1,
        }
    }
}

impl CBVGConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for minimum clearance.
    pub fn with_min_clearance(mut self, clearance: f32) -> Self {
        self.min_clearance = clearance;
        self
    }

    /// Builder-style setter for narrow passage clearance.
    pub fn with_min_clearance_narrow(mut self, clearance: f32) -> Self {
        self.min_clearance_narrow = clearance;
        self
    }

    /// Builder-style setter for Voronoi sample step.
    pub fn with_voronoi_sample_step(mut self, step: f32) -> Self {
        self.voronoi_sample_step = step;
        self
    }

    /// Builder-style setter for maximum edge length.
    pub fn with_max_edge_length(mut self, length: f32) -> Self {
        self.max_edge_length = length;
        self
    }

    /// Builder-style setter for coverage node interval.
    pub fn with_coverage_node_interval(mut self, interval: f32) -> Self {
        self.coverage_node_interval = interval;
        self
    }

    /// Builder-style setter for node merge distance.
    pub fn with_node_merge_distance(mut self, distance: f32) -> Self {
        self.node_merge_distance = distance;
        self
    }

    /// Builder-style setter for maximum nodes.
    pub fn with_max_nodes(mut self, max: usize) -> Self {
        self.max_nodes = max;
        self
    }

    /// Builder-style setter for edge clearance step.
    pub fn with_edge_clearance_step(mut self, step: f32) -> Self {
        self.edge_clearance_step = step;
        self
    }

    /// Builder-style setter for wall sample step.
    pub fn with_wall_sample_step(mut self, step: f32) -> Self {
        self.wall_sample_step = step;
        self
    }

    /// Get the effective clearance for a given measured clearance.
    ///
    /// Returns `min_clearance` for open areas, or `min_clearance_narrow`
    /// for narrow passages where full clearance isn't achievable.
    #[inline]
    pub fn effective_clearance(&self, measured_clearance: f32) -> f32 {
        if measured_clearance >= self.min_clearance {
            self.min_clearance
        } else {
            self.min_clearance_narrow
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = CBVGConfig::default();
        assert_eq!(config.min_clearance, 0.3);
        assert_eq!(config.min_clearance_narrow, 0.18);
        assert_eq!(config.voronoi_sample_step, 0.1);
        assert_eq!(config.max_edge_length, 0.8);
        assert_eq!(config.coverage_node_interval, 0.35);
        assert_eq!(config.node_merge_distance, 0.15);
        assert_eq!(config.max_nodes, 500);
        assert_eq!(config.edge_clearance_step, 0.05);
        assert_eq!(config.wall_sample_step, 0.1);
    }

    #[test]
    fn test_builder_pattern() {
        let config = CBVGConfig::new()
            .with_min_clearance(0.25)
            .with_max_nodes(1000);

        assert_eq!(config.min_clearance, 0.25);
        assert_eq!(config.max_nodes, 1000);
    }

    #[test]
    fn test_effective_clearance() {
        let config = CBVGConfig::default();

        // Open area - use full clearance
        assert_eq!(config.effective_clearance(0.5), 0.3);

        // Narrow passage - use reduced clearance
        assert_eq!(config.effective_clearance(0.2), 0.18);
    }
}
