//! Path traversability checking result.

use crate::core::WorldPoint;

/// Result of path traversability check.
#[derive(Clone, Debug)]
pub struct PathCheckResult {
    /// Is the entire path clear?
    pub is_clear: bool,
    /// Position where the path is blocked (if blocked).
    pub blocked_at: Option<WorldPoint>,
    /// Furthest safe position along the path.
    pub furthest_safe: Option<WorldPoint>,
    /// Distance that is traversable.
    pub distance_traversable: f32,
}

impl PathCheckResult {
    /// Percentage of path that is traversable (0.0 to 1.0).
    pub fn traversable_ratio(&self, total_length: f32) -> f32 {
        if total_length <= 0.0 {
            return 0.0;
        }
        (self.distance_traversable / total_length).min(1.0)
    }
}
