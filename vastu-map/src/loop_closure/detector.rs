//! Loop closure detector implementation.

use crate::core::{Point2D, Pose2D};
use crate::features::{Corner2D, Line2D, ScanDescriptor};
use crate::matching::match_scan;

use super::config::LoopClosureConfig;
use super::types::{Keyframe, LoopClosure};

/// Loop closure detector using keyframe-based matching.
pub struct LoopClosureDetector {
    /// Stored keyframes.
    keyframes: Vec<Keyframe>,

    /// Configuration.
    config: LoopClosureConfig,

    /// Last pose where a keyframe was created.
    last_keyframe_pose: Option<Pose2D>,

    /// Total distance traveled.
    total_distance: f32,

    /// Next keyframe ID.
    next_keyframe_id: usize,
}

impl LoopClosureDetector {
    /// Create a new loop closure detector.
    pub fn new(config: LoopClosureConfig) -> Self {
        Self {
            keyframes: Vec::new(),
            config,
            last_keyframe_pose: None,
            total_distance: 0.0,
            next_keyframe_id: 0,
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &LoopClosureConfig {
        &self.config
    }

    /// Get all keyframes.
    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// Get the number of keyframes.
    pub fn keyframe_count(&self) -> usize {
        self.keyframes.len()
    }

    /// Get the total distance traveled.
    pub fn total_distance(&self) -> f32 {
        self.total_distance
    }

    /// Clear all keyframes and reset state.
    pub fn clear(&mut self) {
        self.keyframes.clear();
        self.last_keyframe_pose = None;
        self.total_distance = 0.0;
        self.next_keyframe_id = 0;
    }

    /// Process a new observation and check for loop closures.
    ///
    /// # Arguments
    /// * `current_pose` - Current robot pose in world frame
    /// * `lines` - Extracted lines in world frame
    /// * `corners` - Extracted corners in world frame
    /// * `points` - Scan points in robot frame (for ICP verification)
    ///
    /// # Returns
    /// A loop closure if one is detected, None otherwise.
    pub fn process(
        &mut self,
        current_pose: Pose2D,
        lines: &[Line2D],
        corners: &[Corner2D],
        points: &[Point2D],
    ) -> Option<LoopClosure> {
        // Update distance traveled
        if let Some(last_pose) = self.last_keyframe_pose {
            let dx = current_pose.x - last_pose.x;
            let dy = current_pose.y - last_pose.y;
            self.total_distance += (dx * dx + dy * dy).sqrt();
        }

        // Check if we need to create a keyframe
        let should_create_keyframe = self.should_create_keyframe(current_pose, lines, corners);

        let mut loop_closure = None;

        if should_create_keyframe {
            // First check for loop closure candidates before adding keyframe
            loop_closure = self.find_loop_closure(current_pose, lines, corners, points);

            // Create and store keyframe
            let keyframe = Keyframe::new(
                self.next_keyframe_id,
                current_pose,
                lines.to_vec(),
                corners.to_vec(),
                points.to_vec(),
                self.total_distance,
                self.config.descriptor_radius,
            );

            self.keyframes.push(keyframe);
            self.next_keyframe_id += 1;
            self.last_keyframe_pose = Some(current_pose);
        }

        loop_closure
    }

    /// Check if a new keyframe should be created.
    fn should_create_keyframe(
        &self,
        current_pose: Pose2D,
        lines: &[Line2D],
        corners: &[Corner2D],
    ) -> bool {
        // Need minimum features
        if lines.len() < self.config.min_lines_for_keyframe {
            return false;
        }
        if corners.len() < self.config.min_corners_for_keyframe {
            return false;
        }

        // First keyframe
        if self.last_keyframe_pose.is_none() {
            return true;
        }

        // Check distance since last keyframe
        let last_pose = self.last_keyframe_pose.unwrap();
        let dx = current_pose.x - last_pose.x;
        let dy = current_pose.y - last_pose.y;
        let distance = (dx * dx + dy * dy).sqrt();

        distance >= self.config.keyframe_interval
    }

    /// Find loop closure candidates and verify with ICP.
    fn find_loop_closure(
        &self,
        current_pose: Pose2D,
        lines: &[Line2D],
        corners: &[Corner2D],
        points: &[Point2D],
    ) -> Option<LoopClosure> {
        // Need enough keyframes for loop closure
        if self.keyframes.is_empty() {
            return None;
        }

        // Compute descriptor for current observation
        let current_descriptor =
            ScanDescriptor::compute(lines, corners, self.config.descriptor_radius);

        if !current_descriptor.is_valid_for_matching() {
            return None;
        }

        // Find candidates using descriptor distance
        let mut candidates: Vec<(usize, f32)> = self
            .keyframes
            .iter()
            .enumerate()
            .filter_map(|(idx, kf)| {
                // Skip recent keyframes (min travel distance)
                let travel_since_kf = self.total_distance - kf.distance_traveled;
                if travel_since_kf < self.config.min_travel_distance {
                    return None;
                }

                // Check descriptor validity
                if !kf.descriptor.is_valid_for_matching() {
                    return None;
                }

                // Quick signature pre-filter
                let sig_diff = (current_descriptor.quick_signature()
                    ^ kf.descriptor.quick_signature())
                .count_ones();
                if sig_diff > 16 {
                    return None; // Too different
                }

                // Full descriptor distance
                let dist = current_descriptor.distance(&kf.descriptor);
                if dist <= self.config.max_descriptor_distance {
                    Some((idx, dist))
                } else {
                    None
                }
            })
            .collect();

        // Sort by descriptor distance (best first)
        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // Verify top candidates with ICP
        for (kf_idx, descriptor_dist) in
            candidates.iter().take(self.config.max_candidates_to_verify)
        {
            let keyframe = &self.keyframes[*kf_idx];

            // Compute initial guess: relative pose from keyframe to current
            let initial_guess = keyframe.pose.inverse().compose(current_pose);

            // Transform current points to keyframe frame
            let transformed_points: Vec<Point2D> = points
                .iter()
                .map(|p| {
                    // Transform from current robot frame to world, then to keyframe frame
                    let world = current_pose.transform_point(*p);
                    keyframe.pose.inverse().transform_point(world)
                })
                .collect();

            // Run ICP verification
            let match_result = match_scan(&transformed_points, &keyframe.lines, initial_guess);

            if match_result.converged
                && match_result.confidence >= self.config.min_verification_confidence
            {
                // Found a valid loop closure
                let relative_pose = keyframe.pose.inverse().compose(current_pose);

                return Some(LoopClosure {
                    from_keyframe: keyframe.id,
                    to_keyframe: self.next_keyframe_id,
                    relative_pose,
                    covariance: match_result.covariance,
                    confidence: match_result.confidence,
                    descriptor_distance: *descriptor_dist,
                });
            }
        }

        None
    }

    /// Force add a keyframe at the current position.
    /// Useful for initialization or when loop closure should be checked
    /// regardless of distance traveled.
    pub fn force_keyframe(
        &mut self,
        pose: Pose2D,
        lines: &[Line2D],
        corners: &[Corner2D],
        points: &[Point2D],
    ) -> usize {
        let keyframe = Keyframe::new(
            self.next_keyframe_id,
            pose,
            lines.to_vec(),
            corners.to_vec(),
            points.to_vec(),
            self.total_distance,
            self.config.descriptor_radius,
        );

        let id = self.next_keyframe_id;
        self.keyframes.push(keyframe);
        self.next_keyframe_id += 1;
        self.last_keyframe_pose = Some(pose);

        id
    }

    /// Get a keyframe by ID.
    pub fn get_keyframe(&self, id: usize) -> Option<&Keyframe> {
        self.keyframes.iter().find(|kf| kf.id == id)
    }

    /// Check if loop closure detection is possible.
    /// Returns true if there are enough keyframes and sufficient travel distance.
    pub fn can_detect_loop(&self) -> bool {
        if self.keyframes.is_empty() {
            return false;
        }

        // Check if we've traveled enough since any keyframe
        self.keyframes.iter().any(|kf| {
            let travel_since = self.total_distance - kf.distance_traveled;
            travel_since >= self.config.min_travel_distance
        })
    }
}

impl Default for LoopClosureDetector {
    fn default() -> Self {
        Self::new(LoopClosureConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_simple_lines() -> Vec<Line2D> {
        vec![
            Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(2.0, 0.0)),
            Line2D::new(Point2D::new(2.0, 0.0), Point2D::new(2.0, 2.0)),
            Line2D::new(Point2D::new(2.0, 2.0), Point2D::new(0.0, 2.0)),
            Line2D::new(Point2D::new(0.0, 2.0), Point2D::new(0.0, 0.0)),
        ]
    }

    fn make_simple_corners() -> Vec<Corner2D> {
        vec![
            Corner2D::new(Point2D::new(0.0, 0.0), 3, 0, std::f32::consts::FRAC_PI_2),
            Corner2D::new(Point2D::new(2.0, 0.0), 0, 1, std::f32::consts::FRAC_PI_2),
            Corner2D::new(Point2D::new(2.0, 2.0), 1, 2, std::f32::consts::FRAC_PI_2),
            Corner2D::new(Point2D::new(0.0, 2.0), 2, 3, std::f32::consts::FRAC_PI_2),
        ]
    }

    fn make_simple_points() -> Vec<Point2D> {
        let mut points = Vec::new();
        // Wall points
        for i in 0..20 {
            let t = i as f32 / 19.0 * 2.0;
            points.push(Point2D::new(t, 0.0));
            points.push(Point2D::new(t, 2.0));
            points.push(Point2D::new(0.0, t));
            points.push(Point2D::new(2.0, t));
        }
        points
    }

    #[test]
    fn test_detector_new() {
        let detector = LoopClosureDetector::default();
        assert_eq!(detector.keyframe_count(), 0);
        assert_eq!(detector.total_distance(), 0.0);
        assert!(!detector.can_detect_loop());
    }

    #[test]
    fn test_first_keyframe_created() {
        let mut detector = LoopClosureDetector::default();
        let lines = make_simple_lines();
        let corners = make_simple_corners();
        let points = make_simple_points();

        let result = detector.process(Pose2D::identity(), &lines, &corners, &points);

        assert!(result.is_none()); // No loop on first observation
        assert_eq!(detector.keyframe_count(), 1);
    }

    #[test]
    fn test_keyframe_interval() {
        let config = LoopClosureConfig {
            keyframe_interval: 1.0,
            ..Default::default()
        };
        let mut detector = LoopClosureDetector::new(config);
        let lines = make_simple_lines();
        let corners = make_simple_corners();
        let points = make_simple_points();

        // First keyframe
        detector.process(Pose2D::identity(), &lines, &corners, &points);
        assert_eq!(detector.keyframe_count(), 1);

        // Too close - no new keyframe
        detector.process(Pose2D::new(0.5, 0.0, 0.0), &lines, &corners, &points);
        assert_eq!(detector.keyframe_count(), 1);

        // Far enough - new keyframe
        detector.process(Pose2D::new(1.5, 0.0, 0.0), &lines, &corners, &points);
        assert_eq!(detector.keyframe_count(), 2);
    }

    #[test]
    fn test_insufficient_features_no_keyframe() {
        let mut detector = LoopClosureDetector::default();
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0))]; // Only 1 line
        let corners = vec![]; // No corners
        let points = make_simple_points();

        detector.process(Pose2D::identity(), &lines, &corners, &points);

        assert_eq!(detector.keyframe_count(), 0); // Not enough features
    }

    #[test]
    fn test_force_keyframe() {
        let mut detector = LoopClosureDetector::default();
        let lines = vec![Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(1.0, 0.0))];
        let corners = vec![];
        let points = make_simple_points();

        let id = detector.force_keyframe(Pose2D::identity(), &lines, &corners, &points);

        assert_eq!(id, 0);
        assert_eq!(detector.keyframe_count(), 1);
    }

    #[test]
    fn test_get_keyframe() {
        let mut detector = LoopClosureDetector::default();
        let lines = make_simple_lines();
        let corners = make_simple_corners();
        let points = make_simple_points();

        detector.force_keyframe(Pose2D::new(1.0, 2.0, 0.5), &lines, &corners, &points);

        let kf = detector.get_keyframe(0);
        assert!(kf.is_some());
        assert_eq!(kf.unwrap().pose.x, 1.0);
        assert_eq!(kf.unwrap().pose.y, 2.0);

        assert!(detector.get_keyframe(999).is_none());
    }

    #[test]
    fn test_clear() {
        let mut detector = LoopClosureDetector::default();
        let lines = make_simple_lines();
        let corners = make_simple_corners();
        let points = make_simple_points();

        detector.force_keyframe(Pose2D::identity(), &lines, &corners, &points);
        assert_eq!(detector.keyframe_count(), 1);

        detector.clear();
        assert_eq!(detector.keyframe_count(), 0);
        assert_eq!(detector.total_distance(), 0.0);
    }

    #[test]
    fn test_can_detect_loop() {
        let config = LoopClosureConfig {
            min_travel_distance: 2.0,
            keyframe_interval: 1.0,
            ..Default::default()
        };
        let mut detector = LoopClosureDetector::new(config);
        let lines = make_simple_lines();
        let corners = make_simple_corners();
        let points = make_simple_points();

        assert!(!detector.can_detect_loop());

        // Add first keyframe
        detector.process(Pose2D::identity(), &lines, &corners, &points);
        assert!(!detector.can_detect_loop()); // Not traveled enough

        // Move far enough
        detector.process(Pose2D::new(3.0, 0.0, 0.0), &lines, &corners, &points);
        assert!(detector.can_detect_loop());
    }

    #[test]
    fn test_total_distance_tracking() {
        let config = LoopClosureConfig {
            keyframe_interval: 1.0,
            ..Default::default()
        };
        let mut detector = LoopClosureDetector::new(config);
        let lines = make_simple_lines();
        let corners = make_simple_corners();
        let points = make_simple_points();

        detector.process(Pose2D::identity(), &lines, &corners, &points);
        assert_eq!(detector.total_distance(), 0.0);

        // Move 1 meter
        detector.process(Pose2D::new(1.0, 0.0, 0.0), &lines, &corners, &points);
        assert!((detector.total_distance() - 1.0).abs() < 0.01);

        // Move 1 more meter diagonally
        detector.process(Pose2D::new(2.0, 1.0, 0.0), &lines, &corners, &points);
        let expected = 1.0 + (2.0_f32).sqrt();
        assert!((detector.total_distance() - expected).abs() < 0.01);
    }

    #[test]
    fn test_keyframe_descriptor() {
        let mut detector = LoopClosureDetector::default();
        let lines = make_simple_lines();
        let corners = make_simple_corners();
        let points = make_simple_points();

        detector.force_keyframe(Pose2D::identity(), &lines, &corners, &points);

        let kf = detector.get_keyframe(0).unwrap();
        assert!(kf.descriptor.is_valid_for_matching());
        assert_eq!(kf.descriptor.num_lines, 4);
        assert_eq!(kf.descriptor.num_corners, 4);
    }
}
