//! Loop closure detection using LiDAR-IRIS descriptors.
//!
//! Two-stage detection (like Cartographer):
//! 1. Fast candidate detection using LiDAR-IRIS descriptors
//! 2. Verification via scan matching (correlative or branch-and-bound)

use serde::{Deserialize, Serialize};

use crate::core::simd::PointCloud;
use crate::core::{LidarScan, Pose2D, normalize_angle};
use crate::grid::GridStorage;

use super::descriptor::LidarIris;
use crate::slam::{
    BranchBoundConfig, CorrelativeMatcher, CorrelativeMatcherConfig, PrecomputedGrids,
    branch_and_bound_match_simd,
};

/// Configuration for loop closure detection.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoopClosureConfig {
    /// Minimum distance traveled between keyframes (meters).
    #[serde(default = "default_keyframe_distance")]
    pub keyframe_distance: f32,

    /// Minimum rotation between keyframes (radians).
    #[serde(default = "default_keyframe_rotation")]
    pub keyframe_rotation: f32,

    /// Maximum descriptor distance for loop closure candidate.
    /// Lower = stricter matching (0-640).
    #[serde(default = "default_max_descriptor_distance")]
    pub max_descriptor_distance: u32,

    /// Minimum pose graph index gap for loop closure.
    /// Prevents detecting "loops" between consecutive scans.
    #[serde(default = "default_min_loop_gap")]
    pub min_loop_gap: usize,

    /// Maximum distance between poses for loop closure (meters).
    /// Prevents matching places that are too far apart.
    #[serde(default = "default_max_loop_distance")]
    pub max_loop_distance: f32,

    /// Whether loop closure is enabled.
    #[serde(default = "default_true")]
    pub enabled: bool,

    // === Scan-Matching Verification ===
    /// Minimum scan matching score for loop closure verification.
    /// Candidates below this score are rejected as false positives.
    /// Range: 0.0 to 1.0 (0.6 = 60% of points match walls)
    #[serde(default = "default_min_verification_score")]
    pub min_verification_score: f32,

    /// Search window for verification scan matching (meters).
    /// Should be small since descriptor already gives approximate alignment.
    #[serde(default = "default_verification_search")]
    pub verification_search_window: f32,

    /// Angular search window for verification (radians).
    #[serde(default = "default_verification_angular")]
    pub verification_angular_window: f32,

    /// Use branch-and-bound matching instead of correlative matcher.
    /// More efficient for large search windows (>30cm).
    /// Enabled by default (Cartographer-style).
    #[serde(default = "default_true")]
    pub use_branch_bound: bool,
}

fn default_keyframe_distance() -> f32 {
    0.5 // 50cm between keyframes
}

fn default_keyframe_rotation() -> f32 {
    0.5 // ~28 degrees
}

fn default_max_descriptor_distance() -> u32 {
    100 // Out of 640 bits
}

fn default_min_loop_gap() -> usize {
    20 // At least 20 poses between loop endpoints
}

fn default_max_loop_distance() -> f32 {
    5.0 // 5 meters
}

fn default_true() -> bool {
    true
}

fn default_min_verification_score() -> f32 {
    0.6 // 60% of points must match walls
}

fn default_verification_search() -> f32 {
    0.2 // 20cm search window (descriptor gives rough alignment)
}

fn default_verification_angular() -> f32 {
    0.2 // ~11 degrees angular search
}

impl Default for LoopClosureConfig {
    fn default() -> Self {
        Self {
            keyframe_distance: default_keyframe_distance(),
            keyframe_rotation: default_keyframe_rotation(),
            max_descriptor_distance: default_max_descriptor_distance(),
            min_loop_gap: default_min_loop_gap(),
            max_loop_distance: default_max_loop_distance(),
            enabled: true,
            min_verification_score: default_min_verification_score(),
            verification_search_window: default_verification_search(),
            verification_angular_window: default_verification_angular(),
            use_branch_bound: true, // Cartographer-style branch-and-bound
        }
    }
}

/// A detected loop closure.
#[derive(Clone, Debug)]
pub struct LoopClosure {
    /// Index of the current pose (loop start).
    pub from_idx: usize,
    /// Index of the matched keyframe pose (loop end).
    pub to_idx: usize,
    /// Relative pose from `from_idx` to `to_idx`.
    /// This is the constraint to add to the pose graph.
    pub relative_pose: Pose2D,
    /// Confidence in the loop closure (0.0 to 1.0).
    pub confidence: f32,
    /// Descriptor distance (lower = better match).
    pub descriptor_distance: u32,
    /// Rotation offset used for best match.
    pub rotation_offset: i32,
}

/// Keyframe data stored for loop closure detection.
#[derive(Clone, Debug)]
struct Keyframe {
    /// Pose graph index.
    pose_idx: usize,
    /// World pose at this keyframe.
    pose: Pose2D,
    /// LiDAR-IRIS descriptor.
    descriptor: LidarIris,
}

/// Result of loop closure verification.
#[derive(Clone, Debug)]
pub struct VerifiedLoopClosure {
    /// The original loop closure candidate.
    pub closure: LoopClosure,
    /// Refined relative pose from scan matching.
    pub refined_relative_pose: Pose2D,
    /// Scan matching score (0.0 to 1.0).
    pub match_score: f32,
    /// Whether verification passed.
    pub verified: bool,
}

/// Loop closure detector.
///
/// Maintains a database of keyframes and detects when the robot
/// revisits a previously seen location.
///
/// Two-stage detection (Cartographer-style):
/// 1. Fast candidate detection using LiDAR-IRIS descriptors
/// 2. Verification via correlative scan matching with Gauss-Newton refinement
pub struct LoopClosureDetector {
    /// Configuration.
    config: LoopClosureConfig,
    /// Stored keyframes.
    keyframes: Vec<Keyframe>,
    /// Last keyframe pose (for distance check).
    last_keyframe_pose: Option<Pose2D>,
    /// Current pose index.
    current_idx: usize,
    /// Matcher for verification.
    verification_matcher: CorrelativeMatcher,
}

impl LoopClosureDetector {
    /// Create a new loop closure detector.
    pub fn new(config: LoopClosureConfig) -> Self {
        // Create verification matcher with tight search window
        let matcher_config = CorrelativeMatcherConfig {
            search_x: config.verification_search_window,
            search_y: config.verification_search_window,
            search_theta: config.verification_angular_window,
            linear_resolution: 0.01, // 1cm fine resolution
            angular_resolution: 0.01,
            multi_resolution: true,
            ..CorrelativeMatcherConfig::default()
        };
        let verification_matcher = CorrelativeMatcher::new(matcher_config);

        Self {
            config,
            keyframes: Vec::new(),
            last_keyframe_pose: None,
            current_idx: 0,
            verification_matcher,
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(LoopClosureConfig::default())
    }

    /// Get configuration.
    pub fn config(&self) -> &LoopClosureConfig {
        &self.config
    }

    /// Get number of stored keyframes.
    pub fn keyframe_count(&self) -> usize {
        self.keyframes.len()
    }

    /// Process a new scan and pose.
    ///
    /// Returns a loop closure if detected, otherwise None.
    ///
    /// # Arguments
    /// * `scan` - The current lidar scan
    /// * `pose` - The current robot pose in world frame
    ///
    /// # Returns
    /// Some(LoopClosure) if a loop is detected, None otherwise.
    pub fn add_scan(&mut self, scan: &LidarScan, pose: Pose2D) -> Option<LoopClosure> {
        if !self.config.enabled {
            return None;
        }

        let pose_idx = self.current_idx;
        self.current_idx += 1;

        // Check if we should create a keyframe
        if !self.should_create_keyframe(pose) {
            return None;
        }

        // Create descriptor for current scan
        let descriptor = LidarIris::from_scan(scan);

        // Skip sparse descriptors (e.g., in open areas)
        if descriptor.is_sparse() {
            return None;
        }

        // Search for loop closure candidates
        let loop_closure = self.find_loop_closure(pose_idx, pose, &descriptor);

        // Add as new keyframe
        self.keyframes.push(Keyframe {
            pose_idx,
            pose,
            descriptor,
        });
        self.last_keyframe_pose = Some(pose);

        loop_closure
    }

    /// Verify a loop closure candidate using scan matching.
    ///
    /// This is the second stage of Cartographer-style loop closure:
    /// after LiDAR-IRIS finds a candidate, verify it with actual scan matching.
    ///
    /// # Arguments
    /// * `candidate` - The loop closure candidate from `add_scan`
    /// * `current_scan` - The current lidar scan
    /// * `storage` - The occupancy grid map
    ///
    /// # Returns
    /// Verification result with refined pose and match score.
    pub fn verify_loop_closure(
        &self,
        candidate: &LoopClosure,
        current_scan: &LidarScan,
        storage: &GridStorage,
    ) -> VerifiedLoopClosure {
        // Find the target keyframe
        let target_keyframe = self
            .keyframes
            .iter()
            .find(|kf| kf.pose_idx == candidate.to_idx);

        let (refined_pose, match_score) = match target_keyframe {
            Some(kf) => {
                // Use the candidate's estimated pose as prior for scan matching
                // The target keyframe pose transformed by the relative pose
                let estimated_current_pose = Pose2D::new(
                    kf.pose.x - candidate.relative_pose.x,
                    kf.pose.y - candidate.relative_pose.y,
                    normalize_angle(kf.pose.theta - candidate.relative_pose.theta),
                );

                // Run scan matching to verify and refine
                let match_result = self.verification_matcher.match_scan(
                    current_scan,
                    estimated_current_pose,
                    storage,
                );

                // Compute refined relative pose from match result
                let refined_relative = Pose2D::new(
                    kf.pose.x - match_result.pose.x,
                    kf.pose.y - match_result.pose.y,
                    normalize_angle(kf.pose.theta - match_result.pose.theta),
                );

                (refined_relative, match_result.score)
            }
            None => {
                // Keyframe not found, return original with zero score
                (candidate.relative_pose, 0.0)
            }
        };

        let verified = match_score >= self.config.min_verification_score;

        VerifiedLoopClosure {
            closure: candidate.clone(),
            refined_relative_pose: refined_pose,
            match_score,
            verified,
        }
    }

    /// Verify a loop closure candidate using branch-and-bound matching.
    ///
    /// This variant uses precomputed multi-resolution grids for more efficient
    /// matching over larger search windows (Cartographer-style).
    ///
    /// # Arguments
    /// * `candidate` - The loop closure candidate from `add_scan`
    /// * `current_scan` - The current lidar scan
    /// * `precomputed_grids` - Precomputed grids from finalized submap
    /// * `target_pose` - The pose of the target keyframe in world frame
    /// * `sensor_offset` - Lidar sensor offset (x, y) in robot frame
    ///
    /// # Returns
    /// Verification result with refined pose and match score.
    pub fn verify_loop_closure_branch_bound(
        &self,
        candidate: &LoopClosure,
        current_scan: &LidarScan,
        precomputed_grids: &PrecomputedGrids,
        target_pose: Pose2D,
        sensor_offset: (f32, f32),
    ) -> VerifiedLoopClosure {
        // Convert scan to PointCloud for SIMD-optimized matching
        let points = PointCloud::from_scan(current_scan);

        // Use the candidate's estimated pose as prior
        let estimated_current_pose = Pose2D::new(
            target_pose.x - candidate.relative_pose.x,
            target_pose.y - candidate.relative_pose.y,
            normalize_angle(target_pose.theta - candidate.relative_pose.theta),
        );

        // Configure branch-and-bound search
        let bb_config = BranchBoundConfig {
            search_x: self.config.verification_search_window,
            search_y: self.config.verification_search_window,
            search_theta: self.config.verification_angular_window,
            angular_resolution: 0.02, // ~1.15 degrees
            min_score: 0.0,
            sensor_offset,
        };

        // Run branch-and-bound matching
        let result = branch_and_bound_match_simd(
            &points,
            precomputed_grids,
            estimated_current_pose,
            &bb_config,
        );

        // Normalize score to 0.0-1.0 range (based on number of points)
        let max_possible_score = points.len() as f32;
        let normalized_score = if max_possible_score > 0.0 {
            result.score / max_possible_score
        } else {
            0.0
        };

        // Compute refined relative pose
        let refined_relative = Pose2D::new(
            target_pose.x - result.pose.x,
            target_pose.y - result.pose.y,
            normalize_angle(target_pose.theta - result.pose.theta),
        );

        let verified = result.converged && normalized_score >= self.config.min_verification_score;

        VerifiedLoopClosure {
            closure: candidate.clone(),
            refined_relative_pose: refined_relative,
            match_score: normalized_score,
            verified,
        }
    }

    /// Get the target keyframe pose for a loop closure candidate.
    pub fn get_keyframe_pose(&self, pose_idx: usize) -> Option<Pose2D> {
        self.keyframes
            .iter()
            .find(|kf| kf.pose_idx == pose_idx)
            .map(|kf| kf.pose)
    }

    /// Check if we should create a keyframe at current pose.
    fn should_create_keyframe(&self, pose: Pose2D) -> bool {
        match &self.last_keyframe_pose {
            None => true, // First keyframe
            Some(last) => {
                let distance = pose.position().distance(&last.position());
                let rotation = (pose.theta - last.theta).abs();

                distance >= self.config.keyframe_distance
                    || rotation >= self.config.keyframe_rotation
            }
        }
    }

    /// Search for loop closure candidates.
    fn find_loop_closure(
        &self,
        current_idx: usize,
        current_pose: Pose2D,
        current_descriptor: &LidarIris,
    ) -> Option<LoopClosure> {
        let mut best_candidate: Option<LoopClosure> = None;
        let mut best_distance = self.config.max_descriptor_distance;

        for keyframe in &self.keyframes {
            // Check pose index gap
            if current_idx.saturating_sub(keyframe.pose_idx) < self.config.min_loop_gap {
                continue;
            }

            // Check spatial distance
            let spatial_distance = current_pose.position().distance(&keyframe.pose.position());
            if spatial_distance > self.config.max_loop_distance {
                continue;
            }

            // Compare descriptors with rotation-invariant matching
            let (desc_distance, rotation_offset) =
                current_descriptor.best_match(&keyframe.descriptor);

            if desc_distance < best_distance {
                best_distance = desc_distance;

                // Calculate relative pose (simple approximation)
                // In a full implementation, you'd use ICP or similar for refinement
                let dx = keyframe.pose.x - current_pose.x;
                let dy = keyframe.pose.y - current_pose.y;
                let dtheta = keyframe.pose.theta - current_pose.theta;

                // Convert rotation offset to radians
                let rotation_rad = rotation_offset as f32 * std::f32::consts::TAU / 80.0;

                let relative_pose = Pose2D::new(dx, dy, dtheta + rotation_rad);

                // Confidence based on descriptor similarity
                let confidence = 1.0 - (desc_distance as f32 / 640.0);

                best_candidate = Some(LoopClosure {
                    from_idx: current_idx,
                    to_idx: keyframe.pose_idx,
                    relative_pose,
                    confidence,
                    descriptor_distance: desc_distance,
                    rotation_offset,
                });
            }
        }

        best_candidate
    }

    /// Clear all stored keyframes.
    pub fn reset(&mut self) {
        self.keyframes.clear();
        self.last_keyframe_pose = None;
        self.current_idx = 0;
    }

    /// Get all keyframe poses (for visualization).
    pub fn keyframe_poses(&self) -> Vec<Pose2D> {
        self.keyframes.iter().map(|kf| kf.pose).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_scan(ranges: &[f32]) -> LidarScan {
        let num_points = ranges.len();
        let angles: Vec<f32> = (0..num_points)
            .map(|i| -std::f32::consts::PI + i as f32 * std::f32::consts::TAU / num_points as f32)
            .collect();
        LidarScan::new(ranges.to_vec(), angles, 0.1, 10.0)
    }

    #[test]
    fn test_detector_creation() {
        let detector = LoopClosureDetector::with_defaults();
        assert_eq!(detector.keyframe_count(), 0);
    }

    #[test]
    fn test_add_first_keyframe() {
        let mut detector = LoopClosureDetector::with_defaults();
        let scan = create_test_scan(&vec![2.0; 360]);
        let pose = Pose2D::new(0.0, 0.0, 0.0);

        let result = detector.add_scan(&scan, pose);
        assert!(result.is_none()); // No loop on first scan
        assert_eq!(detector.keyframe_count(), 1);
    }

    #[test]
    fn test_keyframe_distance() {
        let mut detector = LoopClosureDetector::with_defaults();
        let scan = create_test_scan(&vec![2.0; 360]);

        // First keyframe
        detector.add_scan(&scan, Pose2D::new(0.0, 0.0, 0.0));
        assert_eq!(detector.keyframe_count(), 1);

        // Too close - should not create keyframe
        detector.add_scan(&scan, Pose2D::new(0.1, 0.0, 0.0));
        assert_eq!(detector.keyframe_count(), 1);

        // Far enough - should create keyframe
        detector.add_scan(&scan, Pose2D::new(1.0, 0.0, 0.0));
        assert_eq!(detector.keyframe_count(), 2);
    }

    #[test]
    fn test_disabled_detector() {
        let config = LoopClosureConfig {
            enabled: false,
            ..Default::default()
        };
        let mut detector = LoopClosureDetector::new(config);
        let scan = create_test_scan(&vec![2.0; 360]);

        let result = detector.add_scan(&scan, Pose2D::new(0.0, 0.0, 0.0));
        assert!(result.is_none());
        assert_eq!(detector.keyframe_count(), 0);
    }

    #[test]
    fn test_reset() {
        let mut detector = LoopClosureDetector::with_defaults();
        let scan = create_test_scan(&vec![2.0; 360]);

        detector.add_scan(&scan, Pose2D::new(0.0, 0.0, 0.0));
        detector.add_scan(&scan, Pose2D::new(1.0, 0.0, 0.0));
        assert_eq!(detector.keyframe_count(), 2);

        detector.reset();
        assert_eq!(detector.keyframe_count(), 0);
    }

    #[test]
    fn test_loop_closure_config() {
        let config = LoopClosureConfig::default();
        assert!((config.keyframe_distance - 0.5).abs() < 0.001);
        assert!(config.enabled);
    }
}
