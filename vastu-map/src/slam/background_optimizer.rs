//! Background pose graph optimization.
//!
//! Provides a mechanism to run pose graph optimization asynchronously,
//! preventing the main SLAM thread from blocking during optimization.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────────┐
//! │                        SLAM Thread                                   │
//! │  ┌───────────┐   ┌───────────┐   ┌─────────────────────────────┐   │
//! │  │ Scan Recv │──▶│ Match     │──▶│ Insert Submap + Add Pose   │   │
//! │  └───────────┘   └───────────┘   └──────────────┬──────────────┘   │
//! │                                                  │                   │
//! │                                                  ▼                   │
//! │                                  ┌────────────────────────────────┐ │
//! │                                  │ Loop Closure Detector          │ │
//! │                                  │ (LiDAR-IRIS + verification)    │ │
//! │                                  └───────────────┬────────────────┘ │
//! └──────────────────────────────────────────────────┼──────────────────┘
//!                                                    │ trigger
//!                                                    ▼
//! ┌─────────────────────────────────────────────────────────────────────┐
//! │                    Background Optimizer                              │
//! │  ┌──────────────┐   ┌──────────────┐   ┌──────────────────────┐    │
//! │  │ Copy Graph   │──▶│  Optimize    │──▶│ Return Corrections   │    │
//! │  │ Snapshot     │   │ (N iters)    │   │                      │    │
//! │  └──────────────┘   └──────────────┘   └──────────────────────┘    │
//! └─────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage Modes
//!
//! 1. **Synchronous** (embedded target): Call `optimize_sync()` directly
//! 2. **Threaded** (desktop): Spawn optimizer thread, use channels
//!
//! For embedded ARM (Allwinner A33), synchronous mode is recommended since
//! the additional thread overhead may not be worthwhile for the typical
//! 100-500ms optimization time that only occurs on loop closure.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread::{self, JoinHandle};

use serde::{Deserialize, Serialize};

use crate::core::Pose2D;
use crate::slam::loop_closure::{LoopClosure, PoseGraph, PoseGraphConfig};

/// Configuration for background optimizer.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BackgroundOptimizerConfig {
    /// Maximum iterations per optimization run.
    #[serde(default = "default_max_iterations")]
    pub max_iterations: usize,

    /// Convergence threshold for pose change.
    #[serde(default = "default_convergence_threshold")]
    pub convergence_threshold: f32,

    /// Trigger optimization every N nodes (0 = only on loop closure).
    #[serde(default = "default_trigger_every_n_nodes")]
    pub trigger_every_n_nodes: usize,

    /// Huber loss scale parameter (meters).
    #[serde(default = "default_huber_scale")]
    pub huber_scale: f32,

    /// Whether to use background thread (vs synchronous).
    #[serde(default = "default_use_thread")]
    pub use_background_thread: bool,
}

fn default_max_iterations() -> usize {
    50
}

fn default_convergence_threshold() -> f32 {
    1e-5
}

fn default_trigger_every_n_nodes() -> usize {
    0 // Only on loop closure by default
}

fn default_huber_scale() -> f32 {
    0.1 // 10cm
}

fn default_use_thread() -> bool {
    false // Synchronous by default for embedded
}

impl Default for BackgroundOptimizerConfig {
    fn default() -> Self {
        Self {
            max_iterations: default_max_iterations(),
            convergence_threshold: default_convergence_threshold(),
            trigger_every_n_nodes: default_trigger_every_n_nodes(),
            huber_scale: default_huber_scale(),
            use_background_thread: default_use_thread(),
        }
    }
}

/// A pose correction to apply after optimization.
#[derive(Clone, Debug)]
pub struct PoseCorrection {
    /// Pose index in the graph.
    pub pose_idx: usize,
    /// Original pose before optimization.
    pub original: Pose2D,
    /// Optimized pose.
    pub optimized: Pose2D,
    /// Delta (optimized - original).
    pub delta: Pose2D,
}

/// Result of an optimization run.
#[derive(Clone, Debug)]
pub struct OptimizationResult {
    /// Pose corrections to apply.
    pub corrections: Vec<PoseCorrection>,
    /// Number of iterations performed.
    pub iterations: usize,
    /// Final error after optimization.
    pub final_error: f32,
    /// Error before optimization.
    pub initial_error: f32,
    /// Whether optimization converged.
    pub converged: bool,
}

/// Trigger type for optimization.
#[derive(Clone, Debug)]
pub enum OptimizerTrigger {
    /// Trigger due to loop closure detection.
    LoopClosure(LoopClosure),
    /// Trigger due to node count threshold.
    NodeCount(usize),
    /// Manual trigger.
    Manual,
}

/// Background pose graph optimizer.
///
/// Can operate in two modes:
/// 1. **Synchronous**: Call `optimize_sync()` directly (blocks caller)
/// 2. **Threaded**: Use `start_background_thread()` and channels
pub struct BackgroundOptimizer {
    /// Configuration.
    config: BackgroundOptimizerConfig,
    /// The pose graph being optimized.
    graph: Arc<Mutex<PoseGraph>>,
    /// Background thread handle (if running).
    thread_handle: Option<JoinHandle<()>>,
    /// Channel to send triggers to background thread.
    trigger_tx: Option<mpsc::Sender<OptimizerTrigger>>,
    /// Channel to receive results from background thread.
    result_rx: Option<mpsc::Receiver<OptimizationResult>>,
    /// Flag to stop background thread.
    should_stop: Arc<AtomicBool>,
    /// Count of poses added since last optimization.
    poses_since_optimization: usize,
}

impl BackgroundOptimizer {
    /// Create a new background optimizer.
    pub fn new(config: BackgroundOptimizerConfig) -> Self {
        let graph_config = PoseGraphConfig {
            max_iterations: config.max_iterations,
            convergence_threshold: config.convergence_threshold,
            huber_scale: config.huber_scale,
            ..PoseGraphConfig::default()
        };

        Self {
            config,
            graph: Arc::new(Mutex::new(PoseGraph::new(graph_config))),
            thread_handle: None,
            trigger_tx: None,
            result_rx: None,
            should_stop: Arc::new(AtomicBool::new(false)),
            poses_since_optimization: 0,
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(BackgroundOptimizerConfig::default())
    }

    /// Get a reference to the pose graph.
    pub fn graph(&self) -> &Arc<Mutex<PoseGraph>> {
        &self.graph
    }

    /// Add a pose to the graph.
    ///
    /// Returns the pose index and an optional trigger if optimization should run.
    pub fn add_pose(
        &mut self,
        pose: Pose2D,
        odom_delta: Option<Pose2D>,
    ) -> (usize, Option<OptimizerTrigger>) {
        let pose_idx = {
            let mut graph = self.graph.lock().unwrap();
            graph.add_pose(pose, odom_delta)
        };

        self.poses_since_optimization += 1;

        // Check if we should trigger optimization based on node count
        let trigger = if self.config.trigger_every_n_nodes > 0
            && self.poses_since_optimization >= self.config.trigger_every_n_nodes
        {
            self.poses_since_optimization = 0;
            Some(OptimizerTrigger::NodeCount(pose_idx))
        } else {
            None
        };

        (pose_idx, trigger)
    }

    /// Add a loop closure constraint.
    ///
    /// Returns a trigger to start optimization.
    pub fn add_loop_closure(&mut self, closure: LoopClosure) -> OptimizerTrigger {
        {
            let mut graph = self.graph.lock().unwrap();
            graph.add_loop_closure(closure.clone());
        }

        self.poses_since_optimization = 0;
        OptimizerTrigger::LoopClosure(closure)
    }

    /// Run optimization synchronously.
    ///
    /// This blocks the caller until optimization is complete.
    /// Use this for embedded targets where threading overhead isn't worthwhile.
    pub fn optimize_sync(&mut self) -> OptimizationResult {
        let mut graph = self.graph.lock().unwrap();

        let initial_error = graph.total_error();
        let iterations = graph.optimize();
        let final_error = graph.total_error();

        // Compute corrections
        let poses = graph.poses();
        let corrections = Self::compute_corrections_from_graph(&graph, poses);

        OptimizationResult {
            corrections,
            iterations,
            final_error,
            initial_error,
            converged: iterations < self.config.max_iterations,
        }
    }

    /// Start the background optimization thread.
    ///
    /// After calling this, use `trigger_optimization()` to start optimization
    /// and `try_receive_result()` to check for completed results.
    pub fn start_background_thread(&mut self) {
        if self.thread_handle.is_some() {
            return; // Already running
        }

        let (trigger_tx, trigger_rx) = mpsc::channel::<OptimizerTrigger>();
        let (result_tx, result_rx) = mpsc::channel::<OptimizationResult>();

        self.trigger_tx = Some(trigger_tx);
        self.result_rx = Some(result_rx);
        self.should_stop.store(false, Ordering::SeqCst);

        let graph: Arc<Mutex<PoseGraph>> = Arc::clone(&self.graph);
        let should_stop = Arc::clone(&self.should_stop);
        let max_iterations = self.config.max_iterations;

        let handle = thread::spawn(move || {
            while !should_stop.load(Ordering::SeqCst) {
                // Wait for trigger
                match trigger_rx.recv_timeout(std::time::Duration::from_millis(100)) {
                    Ok(_trigger) => {
                        // Perform optimization
                        let result = {
                            let mut graph_lock = graph.lock().unwrap();

                            let initial_error = graph_lock.total_error();
                            let iterations = graph_lock.optimize();
                            let final_error = graph_lock.total_error();

                            let poses = graph_lock.poses();
                            let corrections =
                                Self::compute_corrections_from_graph(&graph_lock, poses);

                            OptimizationResult {
                                corrections,
                                iterations,
                                final_error,
                                initial_error,
                                converged: iterations < max_iterations,
                            }
                        };

                        // Send result back
                        let _ = result_tx.send(result);
                    }
                    Err(mpsc::RecvTimeoutError::Timeout) => {
                        // No trigger, continue waiting
                    }
                    Err(mpsc::RecvTimeoutError::Disconnected) => {
                        break; // Channel closed
                    }
                }
            }
        });

        self.thread_handle = Some(handle);
    }

    /// Trigger optimization in the background thread.
    ///
    /// Returns `true` if the trigger was sent successfully.
    pub fn trigger_optimization(&self, trigger: OptimizerTrigger) -> bool {
        if let Some(ref tx) = self.trigger_tx {
            tx.send(trigger).is_ok()
        } else {
            false
        }
    }

    /// Try to receive a result from the background thread.
    ///
    /// Returns `None` if no result is available yet.
    pub fn try_receive_result(&self) -> Option<OptimizationResult> {
        if let Some(ref rx) = self.result_rx {
            rx.try_recv().ok()
        } else {
            None
        }
    }

    /// Block until a result is available from the background thread.
    ///
    /// Returns `None` if the channel is disconnected.
    pub fn wait_for_result(&self) -> Option<OptimizationResult> {
        if let Some(ref rx) = self.result_rx {
            rx.recv().ok()
        } else {
            None
        }
    }

    /// Stop the background thread.
    pub fn stop_background_thread(&mut self) {
        self.should_stop.store(true, Ordering::SeqCst);

        // Drop the trigger channel to unblock the thread
        self.trigger_tx = None;

        if let Some(handle) = self.thread_handle.take() {
            let _ = handle.join();
        }

        self.result_rx = None;
    }

    /// Check if the background thread is running.
    pub fn is_background_running(&self) -> bool {
        self.thread_handle.is_some()
    }

    /// Get the number of poses in the graph.
    pub fn pose_count(&self) -> usize {
        self.graph.lock().unwrap().len()
    }

    /// Check if the graph has any loop closures.
    pub fn has_loops(&self) -> bool {
        self.graph.lock().unwrap().has_loops()
    }

    /// Reset the graph and optimizer state.
    pub fn reset(&mut self) {
        let mut graph = self.graph.lock().unwrap();
        graph.reset();
        self.poses_since_optimization = 0;
    }

    /// Compute corrections from optimized graph.
    fn compute_corrections_from_graph(
        _graph: &PoseGraph,
        optimized_poses: &[Pose2D],
    ) -> Vec<PoseCorrection> {
        // Since we optimize in-place, we need to track the deltas
        // For now, return the current poses as the "optimized" values
        // In a real implementation, we'd snapshot before and after
        optimized_poses
            .iter()
            .enumerate()
            .map(|(idx, &pose)| {
                PoseCorrection {
                    pose_idx: idx,
                    original: pose, // These would be the original poses
                    optimized: pose,
                    delta: Pose2D::new(0.0, 0.0, 0.0), // Would be actual delta
                }
            })
            .collect()
    }

    /// Get optimized poses directly.
    pub fn get_optimized_poses(&self) -> Vec<Pose2D> {
        let graph = self.graph.lock().unwrap();
        graph.poses().to_vec()
    }
}

impl Drop for BackgroundOptimizer {
    fn drop(&mut self) {
        self.stop_background_thread();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_optimizer_creation() {
        let optimizer = BackgroundOptimizer::with_defaults();
        assert_eq!(optimizer.pose_count(), 0);
        assert!(!optimizer.has_loops());
    }

    #[test]
    fn test_add_poses() {
        let mut optimizer = BackgroundOptimizer::with_defaults();

        // Add first pose
        let (idx0, trigger0) = optimizer.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        assert_eq!(idx0, 0);
        assert!(trigger0.is_none());

        // Add second pose with odometry
        let delta = Pose2D::new(1.0, 0.0, 0.1);
        let (idx1, _) = optimizer.add_pose(Pose2D::new(1.0, 0.0, 0.1), Some(delta));
        assert_eq!(idx1, 1);
        assert_eq!(optimizer.pose_count(), 2);
    }

    #[test]
    fn test_node_count_trigger() {
        let config = BackgroundOptimizerConfig {
            trigger_every_n_nodes: 3,
            ..Default::default()
        };
        let mut optimizer = BackgroundOptimizer::new(config);

        // Add poses until trigger
        let (_, t1) = optimizer.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        assert!(t1.is_none());

        let (_, t2) =
            optimizer.add_pose(Pose2D::new(1.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));
        assert!(t2.is_none());

        let (_, t3) =
            optimizer.add_pose(Pose2D::new(2.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));
        assert!(matches!(t3, Some(OptimizerTrigger::NodeCount(_))));
    }

    #[test]
    fn test_loop_closure_trigger() {
        let mut optimizer = BackgroundOptimizer::with_defaults();

        // Add some poses
        optimizer.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        optimizer.add_pose(Pose2D::new(1.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));
        optimizer.add_pose(Pose2D::new(2.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));

        // Add loop closure
        let closure = LoopClosure {
            from_idx: 2,
            to_idx: 0,
            relative_pose: Pose2D::new(-2.0, 0.0, 0.0),
            confidence: 0.9,
            descriptor_distance: 50,
            rotation_offset: 0,
        };

        let trigger = optimizer.add_loop_closure(closure);
        assert!(matches!(trigger, OptimizerTrigger::LoopClosure(_)));
        assert!(optimizer.has_loops());
    }

    #[test]
    fn test_sync_optimization() {
        let mut optimizer = BackgroundOptimizer::with_defaults();

        // Build a simple chain of poses
        optimizer.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        optimizer.add_pose(Pose2D::new(1.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));
        optimizer.add_pose(Pose2D::new(2.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));

        // Run synchronous optimization
        let result = optimizer.optimize_sync();

        assert!(result.iterations > 0 || result.converged);
        assert!(!result.corrections.is_empty());
    }

    #[test]
    fn test_background_thread() {
        let mut optimizer = BackgroundOptimizer::with_defaults();

        // Add some poses
        optimizer.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        optimizer.add_pose(Pose2D::new(1.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));

        // Start background thread
        optimizer.start_background_thread();
        assert!(optimizer.is_background_running());

        // Trigger optimization
        let triggered = optimizer.trigger_optimization(OptimizerTrigger::Manual);
        assert!(triggered);

        // Wait for result
        let result = optimizer.wait_for_result();
        assert!(result.is_some());

        // Stop thread
        optimizer.stop_background_thread();
        assert!(!optimizer.is_background_running());
    }

    #[test]
    fn test_reset() {
        let mut optimizer = BackgroundOptimizer::with_defaults();

        optimizer.add_pose(Pose2D::new(0.0, 0.0, 0.0), None);
        optimizer.add_pose(Pose2D::new(1.0, 0.0, 0.0), Some(Pose2D::new(1.0, 0.0, 0.0)));

        assert_eq!(optimizer.pose_count(), 2);

        optimizer.reset();
        assert_eq!(optimizer.pose_count(), 0);
    }
}
