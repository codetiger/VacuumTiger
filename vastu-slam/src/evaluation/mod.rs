//! Cartographer-style evaluation framework for VastuSLAM.
//!
//! This module provides tools for evaluating SLAM accuracy, performance, and quality
//! following the approach used by Google Cartographer.
//!
//! ## Overview
//!
//! The evaluation framework is built on three pillars:
//!
//! 1. **Accuracy** - Pose relations metric, ATE, RPE
//! 2. **Performance** - Latency benchmarks, resource usage
//! 3. **Quality** - Map consistency, loop closure precision/recall
//!
//! ## Cartographer-style Evaluation
//!
//! The core idea is to evaluate using **relative poses** between trajectory nodes,
//! which works without external ground truth (GPS, motion capture).
//!
//! ### Two-step process:
//!
//! 1. **Generate ground truth**: Extract pose relations from an optimized trajectory
//!    (with loop closures applied)
//! 2. **Evaluate**: Compare a test trajectory against these ground truth relations
//!
//! ## Example
//!
//! ```rust,ignore
//! use vastu_slam::evaluation::{GroundTruthRelations, RelationsMetrics, GroundTruthConfig};
//!
//! // Step 1: Generate ground truth from optimized trajectory
//! let config = GroundTruthConfig::default();
//! let ground_truth = GroundTruthRelations::generate(
//!     &optimized_poses,
//!     &loop_closures,
//!     config,
//! );
//!
//! // Step 2: Evaluate test trajectory
//! let metrics = RelationsMetrics::compute(&test_poses, &ground_truth);
//! metrics.print();
//! // Output:
//! // Abs translational error 0.00832 +/- 0.00421 m
//! // Abs rotational error 0.15234 +/- 0.08123 deg
//! ```

mod accuracy;
mod ground_truth;
mod performance;
mod quality;
mod relations_metrics;

pub use accuracy::{AbsoluteTrajectoryError, AccuracyMetrics, RelativePoseError, TrajectoryError};
pub use ground_truth::{GroundTruthConfig, GroundTruthRelations, PoseRelation};
pub use performance::{OperationTiming, PerformanceMetrics, PerformanceTracker};
pub use quality::{LoopClosureGroundTruth, LoopClosurePR, MapQualityMetrics};
pub use relations_metrics::{RelationError, RelationsMetrics};
