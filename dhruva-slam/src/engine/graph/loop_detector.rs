//! Loop closure detection for pose graph SLAM.
//!
//! Detects when the robot revisits a previously mapped area by comparing
//! current scan data with historical keyframes.
//!
//! # Detection Strategy
//!
//! 1. **Distance-based trigger**: Only check for loops near previous poses
//! 2. **Scan context matching**: Use global descriptor for place recognition
//! 3. **Geometric verification**: Validate matches with scan matching

use crate::algorithms::matching::{IcpConfig, PointToPointIcp, ScanMatcher};
use crate::core::types::{PointCloud2D, Pose2D};
use crate::engine::slam::keyframe::{Keyframe, ScanContext};

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

    /// Scan match score.
    pub match_score: f32,
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

    /// Minimum scan context similarity to consider a match.
    pub min_scan_context_similarity: f32,

    /// Minimum scan match score to accept a loop closure.
    pub min_match_score: f32,

    /// Ring key distance threshold for quick rejection.
    pub ring_key_threshold: f32,

    /// Maximum number of candidates to verify per detection.
    pub max_candidates: usize,

    /// ICP configuration for verification.
    pub icp_config: IcpConfig,

    /// Information matrix scale for loop closure constraints.
    ///
    /// Lower = less confident = wider covariance.
    pub information_scale: f32,
}

impl Default for LoopDetectorConfig {
    fn default() -> Self {
        Self {
            min_node_distance: 20,
            max_search_distance: 10.0,
            min_scan_context_similarity: 0.7,
            min_match_score: 0.5,
            ring_key_threshold: 0.5,
            max_candidates: 5,
            icp_config: IcpConfig {
                max_iterations: 30,
                translation_epsilon: 0.001,
                rotation_epsilon: 0.001,
                max_correspondence_distance: 1.0,
                ..Default::default()
            },
            information_scale: 50.0, // Less confident than odometry
        }
    }
}

/// Loop closure detector using scan context and geometric verification.
pub struct LoopDetector {
    config: LoopDetectorConfig,

    /// Scan matcher for geometric verification.
    matcher: PointToPointIcp,

    /// Scan context database (keyframe_id -> context).
    /// In a production system, this would use more efficient indexing.
    scan_contexts: Vec<(u64, ScanContext)>,
}

impl LoopDetector {
    /// Create a new loop detector.
    pub fn new(config: LoopDetectorConfig) -> Self {
        let matcher = PointToPointIcp::new(config.icp_config.clone());

        Self {
            config,
            matcher,
            scan_contexts: Vec::new(),
        }
    }

    /// Add a keyframe to the database.
    pub fn add_keyframe(&mut self, keyframe_id: u64, scan: &PointCloud2D) {
        let context = ScanContext::from_scan(scan);
        self.scan_contexts.push((keyframe_id, context));
    }

    /// Remove a keyframe from the database.
    pub fn remove_keyframe(&mut self, keyframe_id: u64) {
        self.scan_contexts.retain(|(id, _)| *id != keyframe_id);
    }

    /// Clear all keyframes.
    pub fn clear(&mut self) {
        self.scan_contexts.clear();
    }

    /// Get number of keyframes in database.
    pub fn num_keyframes(&self) -> usize {
        self.scan_contexts.len()
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
        let mut candidates = Vec::new();

        // Skip if not enough keyframes in database
        if self.scan_contexts.len() < self.config.min_node_distance {
            return candidates;
        }

        // Compute scan context for query
        let query_context = ScanContext::from_scan(query_scan);

        // Find candidate matches
        let mut potential_matches: Vec<(u64, f32)> = Vec::new();

        for (kf_id, context) in &self.scan_contexts {
            // Skip recent keyframes (must be at least min_node_distance away)
            if query_id.saturating_sub(*kf_id) < self.config.min_node_distance as u64 {
                continue;
            }

            // Quick rejection using ring key
            if !query_context.quick_match(context, self.config.ring_key_threshold) {
                continue;
            }

            // Full similarity computation
            let similarity = query_context.similarity(context);
            if similarity >= self.config.min_scan_context_similarity {
                potential_matches.push((*kf_id, similarity));
            }
        }

        // Sort by similarity (descending)
        potential_matches
            .sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Limit candidates
        potential_matches.truncate(self.config.max_candidates);

        // Geometric verification
        for (match_id, context_similarity) in potential_matches {
            // Find the matching keyframe
            let match_kf = keyframes.iter().find(|kf| kf.id == match_id);
            if match_kf.is_none() {
                continue;
            }
            let match_kf = match_kf.unwrap();

            // Check spatial distance
            let dx = query_pose.x - match_kf.pose.x;
            let dy = query_pose.y - match_kf.pose.y;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance > self.config.max_search_distance {
                continue;
            }

            // Geometric verification using scan matching
            if let Some(candidate) = self.verify_candidate(
                query_id,
                query_scan,
                query_pose,
                match_kf,
                context_similarity,
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
        context_similarity: f32,
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

        // Compute confidence from both scan context and ICP score
        let confidence = (context_similarity * result.score).sqrt();

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
            match_score: result.score,
        })
    }

    /// Get configuration.
    pub fn config(&self) -> &LoopDetectorConfig {
        &self.config
    }
}

/// Trait for loop detection strategies.
#[allow(dead_code)]
pub trait LoopDetection {
    /// Detect loop closures for a new keyframe.
    fn detect_loops(
        &mut self,
        query_id: u64,
        query_scan: &PointCloud2D,
        query_pose: &Pose2D,
        keyframes: &[Keyframe],
    ) -> Vec<LoopClosureCandidate>;

    /// Add a keyframe to the detection database.
    fn add_to_database(&mut self, keyframe_id: u64, scan: &PointCloud2D);
}

impl LoopDetection for LoopDetector {
    fn detect_loops(
        &mut self,
        query_id: u64,
        query_scan: &PointCloud2D,
        query_pose: &Pose2D,
        keyframes: &[Keyframe],
    ) -> Vec<LoopClosureCandidate> {
        self.detect(query_id, query_scan, query_pose, keyframes)
    }

    fn add_to_database(&mut self, keyframe_id: u64, scan: &PointCloud2D) {
        self.add_keyframe(keyframe_id, scan);
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
        let detector = LoopDetector::new(config);

        assert_eq!(detector.num_keyframes(), 0);
    }

    #[test]
    fn test_add_keyframe() {
        let config = LoopDetectorConfig::default();
        let mut detector = LoopDetector::new(config);

        let scan = create_wall_scan(0.0, 0.0);
        detector.add_keyframe(0, &scan);

        assert_eq!(detector.num_keyframes(), 1);
    }

    #[test]
    fn test_remove_keyframe() {
        let config = LoopDetectorConfig::default();
        let mut detector = LoopDetector::new(config);

        let scan = create_wall_scan(0.0, 0.0);
        detector.add_keyframe(0, &scan);
        detector.add_keyframe(1, &scan);

        assert_eq!(detector.num_keyframes(), 2);

        detector.remove_keyframe(0);
        assert_eq!(detector.num_keyframes(), 1);
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
    fn test_scan_context_similarity() {
        let scan1 = create_wall_scan(0.0, 0.0);
        let scan2 = create_wall_scan(0.0, 0.0); // Same pattern
        let scan3 = create_different_scan(); // Different pattern

        let ctx1 = ScanContext::from_scan(&scan1);
        let ctx2 = ScanContext::from_scan(&scan2);
        let ctx3 = ScanContext::from_scan(&scan3);

        // Same scan should have high similarity
        let sim_same = ctx1.similarity(&ctx2);
        assert!(sim_same > 0.9, "Same scan similarity: {}", sim_same);

        // Different scan should have lower similarity
        let sim_diff = ctx1.similarity(&ctx3);
        assert!(sim_diff < sim_same, "Different scan should be less similar");
    }

    #[test]
    fn test_loop_closure_candidate() {
        let candidate = LoopClosureCandidate {
            query_id: 100,
            match_id: 10,
            relative_pose: Pose2D::new(1.0, 2.0, 0.1),
            information: Information2D::default(),
            confidence: 0.8,
            match_score: 0.85,
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
        assert!(config.min_scan_context_similarity > 0.0);
        assert!(config.min_scan_context_similarity < 1.0);
    }
}
