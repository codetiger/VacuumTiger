//! Core types for loop closure detection.

use crate::core::{Point2D, Pose2D};
use crate::features::{Corner2D, Line2D, ScanDescriptor};

/// A keyframe stores a snapshot of the map at a particular pose.
#[derive(Clone, Debug)]
pub struct Keyframe {
    /// Unique keyframe identifier.
    pub id: usize,

    /// Robot pose when keyframe was created.
    pub pose: Pose2D,

    /// Scan descriptor for quick matching.
    pub descriptor: ScanDescriptor,

    /// Line features in world frame.
    pub lines: Vec<Line2D>,

    /// Corner features in world frame.
    pub corners: Vec<Corner2D>,

    /// Points in robot frame (for ICP verification).
    pub points: Vec<Point2D>,

    /// Distance traveled when keyframe was created.
    pub distance_traveled: f32,
}

impl Keyframe {
    /// Create a new keyframe.
    pub fn new(
        id: usize,
        pose: Pose2D,
        lines: Vec<Line2D>,
        corners: Vec<Corner2D>,
        points: Vec<Point2D>,
        distance_traveled: f32,
        descriptor_radius: f32,
    ) -> Self {
        let descriptor = ScanDescriptor::compute(&lines, &corners, descriptor_radius);
        Self {
            id,
            pose,
            descriptor,
            lines,
            corners,
            points,
            distance_traveled,
        }
    }
}

/// A detected loop closure constraint.
#[derive(Clone, Debug)]
pub struct LoopClosure {
    /// Index of the earlier keyframe.
    pub from_keyframe: usize,

    /// Index of the later keyframe (current).
    pub to_keyframe: usize,

    /// Relative pose from `from_keyframe` to `to_keyframe`.
    pub relative_pose: Pose2D,

    /// Covariance matrix of the relative pose estimate.
    /// Format: [[σxx, σxy, σxθ], [σxy, σyy, σyθ], [σxθ, σyθ, σθθ]]
    pub covariance: [[f32; 3]; 3],

    /// Confidence of the loop closure (0.0 to 1.0).
    pub confidence: f32,

    /// Descriptor distance used for candidate retrieval.
    pub descriptor_distance: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_loop_closure_struct() {
        let closure = LoopClosure {
            from_keyframe: 0,
            to_keyframe: 5,
            relative_pose: Pose2D::new(1.0, 2.0, 0.5),
            covariance: [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.001]],
            confidence: 0.85,
            descriptor_distance: 0.3,
        };

        assert_eq!(closure.from_keyframe, 0);
        assert_eq!(closure.to_keyframe, 5);
        assert_eq!(closure.confidence, 0.85);
    }
}
