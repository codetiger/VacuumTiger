//! Loop closure detection for pose graph SLAM.
//!
//! Detects when the robot revisits a previously mapped area by comparing
//! current scan data with historical keyframes.
//!
//! # Detection Strategy
//!
//! 1. **Distance-based trigger**: Only check for loops near previous poses
//! 2. **Descriptor matching**: Use LiDAR-IRIS (default) or ScanContext for place recognition
//! 3. **Geometric verification**: Validate matches with scan matching
//!
//! # Descriptor Options
//!
//! | Descriptor | Size | Matching Speed | Memory (1000 KF) |
//! |------------|------|----------------|------------------|
//! | LiDAR-IRIS | 80 bytes | ~0.5ms (Hamming) | 80 KB |
//! | ScanContext | 4.8 KB | ~30ms (cosine) | 4.8 MB |

use crate::algorithms::descriptors::{LidarIris, LidarIrisConfig};
use crate::algorithms::matching::{IcpConfig, PointToPointIcp, ScanMatcher};
use crate::core::types::{PointCloud2D, Pose2D};
use crate::engine::slam::keyframe::Keyframe;

use super::pose_graph::Information2D;

/// A potential loop closure between two poses.
#[derive(Debug, Clone)]
pub struct LoopClosureCandidate {
    /// ID of the query keyframe (current).
    pub query_id: u64,

    /// ID of the matching keyframe (historical).
    pub match_id: u64,

    /// Relative pose from query to match.
    pub relative_pose: Pose2D,

    /// Information matrix for the constraint.
    pub information: Information2D,

    /// Confidence score (0-1).
    pub confidence: f32,
}

/// Configuration for loop detection.
#[derive(Debug, Clone)]
pub struct LoopDetectorConfig {
    /// Minimum distance (in graph nodes) before considering loop closure.
    ///
    /// Prevents detecting loops with recent poses.
    pub min_node_distance: usize,

    /// Maximum Euclidean distance (meters) to search for loop candidates.
    pub max_search_distance: f32,

    /// Maximum Hamming distance threshold for IRIS matching (out of 640 bits).
    ///
    /// Lower = stricter matching. Default 100 = ~15% bit difference.
    pub max_hamming_distance: u32,

    /// Minimum scan match score to accept a loop closure.
    pub min_match_score: f32,

    /// Maximum number of candidates to verify per detection.
    pub max_candidates: usize,

    /// ICP configuration for verification.
    pub icp_config: IcpConfig,

    /// Information matrix scale for loop closure constraints.
    ///
    /// Lower = less confident = wider covariance.
    pub information_scale: f32,

    /// LiDAR-IRIS configuration.
    pub iris_config: LidarIrisConfig,
}

impl Default for LoopDetectorConfig {
    fn default() -> Self {
        Self {
            min_node_distance: 20,
            max_search_distance: 10.0,
            max_hamming_distance: 100,
            min_match_score: 0.5,
            max_candidates: 5,
            icp_config: IcpConfig {
                max_iterations: 30,
                translation_epsilon: 0.001,
                rotation_epsilon: 0.001,
                max_correspondence_distance: 1.0,
                ..Default::default()
            },
            information_scale: 50.0,
            iris_config: LidarIrisConfig::default(),
        }
    }
}

/// Internal descriptor storage.
enum Descriptor {
    Iris(LidarIris),
}

/// Loop closure detector using LiDAR-IRIS or ScanContext with geometric verification.
pub struct LoopDetector {
    config: LoopDetectorConfig,

    /// Scan matcher for geometric verification.
    matcher: PointToPointIcp,

    /// Descriptor database (keyframe_id -> descriptor).
    descriptors: Vec<(u64, Descriptor)>,
}

impl LoopDetector {
    /// Create a new loop detector.
    pub fn new(config: LoopDetectorConfig) -> Self {
        let matcher = PointToPointIcp::new(config.icp_config.clone());

        Self {
            config,
            matcher,
            descriptors: Vec::new(),
        }
    }

    /// Add a keyframe to the database.
    pub fn add_keyframe(&mut self, keyframe_id: u64, scan: &PointCloud2D) {
        let descriptor = Descriptor::Iris(LidarIris::from_scan(scan, &self.config.iris_config));
        self.descriptors.push((keyframe_id, descriptor));
    }

    /// Clear all keyframes.
    pub fn clear(&mut self) {
        self.descriptors.clear();
    }

    /// Detect loop closures for a new keyframe.
    ///
    /// # Arguments
    ///
    /// * `query_id` - ID of the query keyframe
    /// * `query_scan` - Point cloud for the query keyframe
    /// * `query_pose` - Current pose estimate for the query
    /// * `keyframes` - All keyframes for geometric verification
    ///
    /// # Returns
    ///
    /// Vector of verified loop closure candidates.
    pub fn detect(
        &mut self,
        query_id: u64,
        query_scan: &PointCloud2D,
        query_pose: &Pose2D,
        keyframes: &[Keyframe],
    ) -> Vec<LoopClosureCandidate> {
        // Skip if not enough keyframes in database
        if self.descriptors.len() < self.config.min_node_distance {
            return Vec::new();
        }

        self.detect_iris(query_id, query_scan, query_pose, keyframes)
    }

    /// Detect using LiDAR-IRIS descriptors.
    fn detect_iris(
        &mut self,
        query_id: u64,
        query_scan: &PointCloud2D,
        query_pose: &Pose2D,
        keyframes: &[Keyframe],
    ) -> Vec<LoopClosureCandidate> {
        let mut candidates = Vec::new();

        // Compute IRIS descriptor for query
        let query_iris = LidarIris::from_scan(query_scan, &self.config.iris_config);

        // Find candidate matches using Hamming distance
        let mut potential_matches: Vec<(u64, u32, usize)> = Vec::new(); // (id, distance, rotation)

        for (kf_id, descriptor) in &self.descriptors {
            // Skip recent keyframes
            if query_id.saturating_sub(*kf_id) < self.config.min_node_distance as u64 {
                continue;
            }

            let Descriptor::Iris(iris) = descriptor;
            let (distance, rotation) = query_iris.match_with_rotation(iris);

            if distance <= self.config.max_hamming_distance {
                potential_matches.push((*kf_id, distance, rotation));
            }
        }

        // Sort by distance (ascending - lower is better)
        potential_matches.sort_by_key(|(_, d, _)| *d);

        // Limit candidates
        potential_matches.truncate(self.config.max_candidates);

        // Geometric verification
        for (match_id, hamming_distance, _rotation) in potential_matches {
            // Find the matching keyframe
            let match_kf = match keyframes.iter().find(|kf| kf.id == match_id) {
                Some(kf) => kf,
                None => continue,
            };

            // Check spatial distance
            let dx = query_pose.x - match_kf.pose.x;
            let dy = query_pose.y - match_kf.pose.y;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance > self.config.max_search_distance {
                continue;
            }

            // Convert Hamming distance to similarity score (0-1)
            let descriptor_similarity =
                1.0 - (hamming_distance as f32 / self.config.iris_config.signature_bits as f32);

            // Geometric verification using scan matching
            if let Some(candidate) = self.verify_candidate(
                query_id,
                query_scan,
                query_pose,
                match_kf,
                descriptor_similarity,
            ) {
                candidates.push(candidate);
            }
        }

        candidates
    }

    /// Verify a loop closure candidate using scan matching.
    fn verify_candidate(
        &mut self,
        query_id: u64,
        query_scan: &PointCloud2D,
        query_pose: &Pose2D,
        match_kf: &Keyframe,
        descriptor_similarity: f32,
    ) -> Option<LoopClosureCandidate> {
        // Compute initial guess for relative pose
        let initial_guess = query_pose.inverse().compose(&match_kf.pose);

        // Run scan matching
        let result = self
            .matcher
            .match_scans(query_scan, &match_kf.scan, &initial_guess);

        if !result.converged || result.score < self.config.min_match_score {
            return None;
        }

        // Compute confidence from both descriptor similarity and ICP score
        let confidence = (descriptor_similarity * result.score).sqrt();

        // Compute information matrix (scaled by confidence)
        let scale = self.config.information_scale * confidence;
        let information = Information2D::from_std_dev(
            0.1 / scale.sqrt(), // ~10cm / sqrt(scale)
            0.1 / scale.sqrt(),
            0.05 / scale.sqrt(), // ~3 degrees / sqrt(scale)
        );

        Some(LoopClosureCandidate {
            query_id,
            match_id: match_kf.id,
            relative_pose: result.transform,
            information,
            confidence,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;

    fn create_wall_scan(offset_x: f32, offset_y: f32) -> PointCloud2D {
        let mut scan = PointCloud2D::new();
        // Create a distinctive L-shaped pattern
        for i in 0..50 {
            let x = offset_x + i as f32 * 0.1;
            scan.push(Point2D::new(x, offset_y + 2.0)); // Horizontal wall
        }
        for i in 0..30 {
            let y = offset_y + i as f32 * 0.1;
            scan.push(Point2D::new(offset_x, y)); // Vertical wall
        }
        scan
    }

    fn create_different_scan() -> PointCloud2D {
        let mut scan = PointCloud2D::new();
        // Create a completely different pattern (circle)
        for i in 0..50 {
            let angle = i as f32 * 0.126; // ~2π/50
            scan.push(Point2D::new(3.0 * angle.cos(), 3.0 * angle.sin()));
        }
        scan
    }

    #[test]
    fn test_loop_detector_creation() {
        let config = LoopDetectorConfig::default();
        let _detector = LoopDetector::new(config);
    }

    #[test]
    fn test_add_keyframe() {
        let config = LoopDetectorConfig::default();
        let mut detector = LoopDetector::new(config);

        let scan = create_wall_scan(0.0, 0.0);
        detector.add_keyframe(0, &scan);
    }

    #[test]
    fn test_no_loop_with_few_keyframes() {
        let config = LoopDetectorConfig {
            min_node_distance: 10,
            ..Default::default()
        };
        let mut detector = LoopDetector::new(config);

        // Add just a few keyframes
        for i in 0..5 {
            let scan = create_wall_scan(i as f32, 0.0);
            detector.add_keyframe(i, &scan);
        }

        // Should not detect loops with too few keyframes
        let query_scan = create_wall_scan(0.0, 0.0);
        let keyframes: Vec<Keyframe> = Vec::new();
        let candidates = detector.detect(10, &query_scan, &Pose2D::identity(), &keyframes);

        assert!(candidates.is_empty());
    }

    #[test]
    fn test_iris_similarity() {
        let config = LidarIrisConfig::default();

        let scan1 = create_wall_scan(0.0, 0.0);
        let scan2 = create_wall_scan(0.0, 0.0); // Same pattern
        let scan3 = create_different_scan(); // Different pattern

        let iris1 = LidarIris::from_scan(&scan1, &config);
        let iris2 = LidarIris::from_scan(&scan2, &config);
        let iris3 = LidarIris::from_scan(&scan3, &config);

        // Same scan should have zero distance
        let dist_same = iris1.distance(&iris2);
        assert_eq!(dist_same, 0, "Same scan should have distance 0");

        // Different scan should have higher distance
        let dist_diff = iris1.distance(&iris3);
        assert!(
            dist_diff > dist_same,
            "Different scan should have higher distance"
        );
    }

    #[test]
    fn test_loop_closure_candidate() {
        let candidate = LoopClosureCandidate {
            query_id: 100,
            match_id: 10,
            relative_pose: Pose2D::new(1.0, 2.0, 0.1),
            information: Information2D::default(),
            confidence: 0.8,
        };

        assert_eq!(candidate.query_id, 100);
        assert_eq!(candidate.match_id, 10);
        assert!(candidate.confidence > 0.7);
    }

    #[test]
    fn test_config_defaults() {
        let config = LoopDetectorConfig::default();

        assert!(config.min_node_distance > 0);
        assert!(config.max_search_distance > 0.0);
        assert!(config.max_hamming_distance > 0);
        assert!(config.min_match_score > 0.0);
        assert!(config.min_match_score < 1.0);
    }
}
