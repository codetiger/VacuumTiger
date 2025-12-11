//! Configuration for Point-to-Line ICP.

/// Configuration for Point-to-Line ICP.
#[derive(Debug, Clone)]
pub struct PointToLineIcpConfig {
    /// Maximum number of iterations.
    pub max_iterations: u32,

    /// Convergence threshold for translation (meters).
    pub translation_epsilon: f32,

    /// Convergence threshold for rotation (radians).
    pub rotation_epsilon: f32,

    /// Maximum correspondence distance (meters).
    pub max_correspondence_distance: f32,

    /// Minimum number of valid correspondences required.
    pub min_correspondences: usize,

    /// Outlier rejection ratio (0.0 to 1.0).
    pub outlier_ratio: f32,

    /// Number of neighbors to use for line fitting.
    ///
    /// More neighbors = smoother lines but less local detail.
    /// Typically 5-10.
    pub line_neighbors: usize,

    /// Minimum line fit quality (R² value).
    ///
    /// Correspondences with poor line fits are rejected.
    pub min_line_quality: f32,
}

impl Default for PointToLineIcpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            translation_epsilon: 0.001,
            rotation_epsilon: 0.001,
            max_correspondence_distance: 0.5,
            min_correspondences: 10,
            outlier_ratio: 0.1,
            line_neighbors: 5,
            min_line_quality: 0.8,
        }
    }
}
