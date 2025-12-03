//! Keyframe management for SLAM.
//!
//! Keyframes are pose-stamped scans that are stored for loop closure detection
//! and map optimization. Not every scan becomes a keyframe - only scans that
//! represent sufficient motion or rotation from the previous keyframe.

use crate::core::types::{Pose2D, PointCloud2D};

/// A keyframe containing a pose and associated scan data.
#[derive(Debug, Clone)]
pub struct Keyframe {
    /// Unique identifier for this keyframe.
    pub id: u64,

    /// Global pose of the robot when this keyframe was captured.
    pub pose: Pose2D,

    /// Point cloud in robot-local frame.
    pub scan: PointCloud2D,

    /// Timestamp when keyframe was captured (microseconds).
    pub timestamp_us: u64,

    /// Index of the submap this keyframe belongs to.
    pub submap_id: u64,

    /// Scan context descriptor for loop closure (computed lazily).
    scan_context: Option<ScanContext>,
}

impl Keyframe {
    /// Create a new keyframe.
    pub fn new(
        id: u64,
        pose: Pose2D,
        scan: PointCloud2D,
        timestamp_us: u64,
        submap_id: u64,
    ) -> Self {
        Self {
            id,
            pose,
            scan,
            timestamp_us,
            submap_id,
            scan_context: None,
        }
    }

    /// Get or compute the scan context descriptor.
    pub fn scan_context(&mut self) -> &ScanContext {
        if self.scan_context.is_none() {
            self.scan_context = Some(ScanContext::from_scan(&self.scan));
        }
        self.scan_context.as_ref().unwrap()
    }

    /// Check if scan context has been computed.
    pub fn has_scan_context(&self) -> bool {
        self.scan_context.is_some()
    }

    /// Transform the scan to global frame.
    pub fn global_scan(&self) -> PointCloud2D {
        self.scan.transform(&self.pose)
    }
}

/// Scan context descriptor for place recognition.
///
/// Based on the Scan Context paper, this creates a rotation-invariant
/// descriptor of the local environment by dividing the scan into
/// sectors and rings, storing the maximum height (distance) in each bin.
#[derive(Debug, Clone)]
pub struct ScanContext {
    /// Descriptor matrix (sectors × rings).
    /// Each element is the max range in that bin.
    descriptor: Vec<f32>,

    /// Number of angular sectors.
    num_sectors: usize,

    /// Number of radial rings.
    num_rings: usize,

    /// Ring key for fast candidate retrieval.
    ring_key: Vec<f32>,
}

impl ScanContext {
    /// Default number of sectors (angular divisions).
    pub const DEFAULT_SECTORS: usize = 60;

    /// Default number of rings (radial divisions).
    pub const DEFAULT_RINGS: usize = 20;

    /// Maximum range to consider (meters).
    pub const MAX_RANGE: f32 = 8.0;

    /// Create a scan context descriptor from a point cloud.
    pub fn from_scan(scan: &PointCloud2D) -> Self {
        Self::from_scan_with_params(scan, Self::DEFAULT_SECTORS, Self::DEFAULT_RINGS)
    }

    /// Create with custom parameters.
    pub fn from_scan_with_params(
        scan: &PointCloud2D,
        num_sectors: usize,
        num_rings: usize,
    ) -> Self {
        let mut descriptor = vec![0.0f32; num_sectors * num_rings];

        let sector_size = std::f32::consts::TAU / num_sectors as f32;
        let ring_size = Self::MAX_RANGE / num_rings as f32;

        for point in scan.iter() {
            let range = (point.x * point.x + point.y * point.y).sqrt();
            if range > Self::MAX_RANGE || range < 0.1 {
                continue;
            }

            let angle = point.y.atan2(point.x);
            let angle_normalized = if angle < 0.0 {
                angle + std::f32::consts::TAU
            } else {
                angle
            };

            let sector = ((angle_normalized / sector_size) as usize).min(num_sectors - 1);
            let ring = ((range / ring_size) as usize).min(num_rings - 1);

            let idx = sector * num_rings + ring;
            descriptor[idx] = descriptor[idx].max(range);
        }

        // Compute ring key (sum of each ring across all sectors)
        let mut ring_key = vec![0.0f32; num_rings];
        for ring in 0..num_rings {
            for sector in 0..num_sectors {
                ring_key[ring] += descriptor[sector * num_rings + ring];
            }
        }

        Self {
            descriptor,
            num_sectors,
            num_rings,
            ring_key,
        }
    }

    /// Compute similarity score with another scan context.
    ///
    /// Returns a value between 0.0 (no match) and 1.0 (perfect match).
    /// Uses column-shifted cosine similarity for rotation invariance.
    pub fn similarity(&self, other: &ScanContext) -> f32 {
        if self.num_sectors != other.num_sectors || self.num_rings != other.num_rings {
            return 0.0;
        }

        let mut best_score = 0.0f32;

        // Try all rotations (column shifts)
        for shift in 0..self.num_sectors {
            let score = self.cosine_similarity_shifted(other, shift);
            best_score = best_score.max(score);
        }

        best_score
    }

    /// Fast similarity check using ring key.
    ///
    /// Returns true if the ring keys are similar enough to warrant
    /// full similarity computation.
    pub fn quick_match(&self, other: &ScanContext, threshold: f32) -> bool {
        if self.num_rings != other.num_rings {
            return false;
        }

        // Compute L2 distance of ring keys
        let mut dist_sq = 0.0f32;
        let mut norm_a = 0.0f32;
        let mut norm_b = 0.0f32;

        for i in 0..self.num_rings {
            dist_sq += (self.ring_key[i] - other.ring_key[i]).powi(2);
            norm_a += self.ring_key[i].powi(2);
            norm_b += other.ring_key[i].powi(2);
        }

        let norm = (norm_a * norm_b).sqrt();
        if norm < 1e-6 {
            return false;
        }

        // Normalized distance
        let normalized_dist = dist_sq.sqrt() / norm;
        normalized_dist < threshold
    }

    /// Compute cosine similarity with a column shift.
    fn cosine_similarity_shifted(&self, other: &ScanContext, shift: usize) -> f32 {
        let mut dot = 0.0f32;
        let mut norm_a = 0.0f32;
        let mut norm_b = 0.0f32;

        for sector in 0..self.num_sectors {
            let shifted_sector = (sector + shift) % self.num_sectors;

            for ring in 0..self.num_rings {
                let a = self.descriptor[sector * self.num_rings + ring];
                let b = other.descriptor[shifted_sector * self.num_rings + ring];

                dot += a * b;
                norm_a += a * a;
                norm_b += b * b;
            }
        }

        let norm = (norm_a * norm_b).sqrt();
        if norm < 1e-6 {
            return 0.0;
        }

        dot / norm
    }

    /// Get the descriptor as a slice.
    pub fn descriptor(&self) -> &[f32] {
        &self.descriptor
    }

    /// Get the ring key.
    pub fn ring_key(&self) -> &[f32] {
        &self.ring_key
    }
}

/// Configuration for keyframe selection.
#[derive(Debug, Clone)]
pub struct KeyframeManagerConfig {
    /// Minimum translation (meters) before creating a new keyframe.
    pub min_translation: f32,

    /// Minimum rotation (radians) before creating a new keyframe.
    pub min_rotation: f32,

    /// Maximum number of keyframes to keep in memory.
    ///
    /// Older keyframes may be removed to save memory, but their
    /// poses are kept in the pose graph.
    pub max_keyframes: usize,

    /// Minimum time between keyframes (microseconds).
    pub min_interval_us: u64,
}

impl Default for KeyframeManagerConfig {
    fn default() -> Self {
        Self {
            min_translation: 0.5,  // 50cm
            min_rotation: 0.5,     // ~30 degrees
            max_keyframes: 1000,
            min_interval_us: 500_000, // 500ms minimum
        }
    }
}

/// Manages keyframe creation and storage.
pub struct KeyframeManager {
    config: KeyframeManagerConfig,

    /// All keyframes in chronological order.
    keyframes: Vec<Keyframe>,

    /// Pose of last keyframe (for motion threshold check).
    last_keyframe_pose: Option<Pose2D>,

    /// Timestamp of last keyframe.
    last_keyframe_time: u64,

    /// Next keyframe ID.
    next_id: u64,
}

impl KeyframeManager {
    /// Create a new keyframe manager.
    pub fn new(config: KeyframeManagerConfig) -> Self {
        Self {
            config,
            keyframes: Vec::new(),
            last_keyframe_pose: None,
            last_keyframe_time: 0,
            next_id: 0,
        }
    }

    /// Check if a new keyframe should be created.
    ///
    /// Returns true if the motion since the last keyframe exceeds
    /// the configured thresholds.
    pub fn should_create_keyframe(
        &self,
        current_pose: &Pose2D,
        timestamp_us: u64,
    ) -> bool {
        // Always create first keyframe
        let last_pose = match &self.last_keyframe_pose {
            Some(pose) => pose,
            None => return true,
        };

        // Check time threshold
        if timestamp_us < self.last_keyframe_time + self.config.min_interval_us {
            return false;
        }

        // Check translation threshold
        let dx = current_pose.x - last_pose.x;
        let dy = current_pose.y - last_pose.y;
        let translation = (dx * dx + dy * dy).sqrt();

        if translation >= self.config.min_translation {
            return true;
        }

        // Check rotation threshold
        let rotation = crate::core::math::angle_diff(last_pose.theta, current_pose.theta).abs();
        if rotation >= self.config.min_rotation {
            return true;
        }

        false
    }

    /// Create a new keyframe.
    ///
    /// # Arguments
    ///
    /// * `pose` - Global pose at keyframe time
    /// * `scan` - Point cloud in robot frame
    /// * `timestamp_us` - Timestamp
    /// * `submap_id` - ID of the submap this keyframe belongs to
    ///
    /// # Returns
    ///
    /// Reference to the newly created keyframe.
    pub fn create_keyframe(
        &mut self,
        pose: Pose2D,
        scan: PointCloud2D,
        timestamp_us: u64,
        submap_id: u64,
    ) -> &Keyframe {
        let id = self.next_id;
        self.next_id += 1;

        let keyframe = Keyframe::new(id, pose, scan, timestamp_us, submap_id);
        self.keyframes.push(keyframe);

        self.last_keyframe_pose = Some(pose);
        self.last_keyframe_time = timestamp_us;

        // Trim if over limit
        if self.keyframes.len() > self.config.max_keyframes {
            // Remove oldest keyframe (but keep its pose in graph)
            self.keyframes.remove(0);
        }

        self.keyframes.last().unwrap()
    }

    /// Get all keyframes.
    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// Get a keyframe by ID.
    pub fn get(&self, id: u64) -> Option<&Keyframe> {
        self.keyframes.iter().find(|kf| kf.id == id)
    }

    /// Get mutable keyframe by ID.
    pub fn get_mut(&mut self, id: u64) -> Option<&mut Keyframe> {
        self.keyframes.iter_mut().find(|kf| kf.id == id)
    }

    /// Get the most recent keyframe.
    pub fn latest(&self) -> Option<&Keyframe> {
        self.keyframes.last()
    }

    /// Get number of keyframes.
    pub fn len(&self) -> usize {
        self.keyframes.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.keyframes.is_empty()
    }

    /// Get keyframes in a spatial region.
    pub fn keyframes_near(&self, pose: &Pose2D, max_distance: f32) -> Vec<&Keyframe> {
        let max_dist_sq = max_distance * max_distance;

        self.keyframes
            .iter()
            .filter(|kf| {
                let dx = kf.pose.x - pose.x;
                let dy = kf.pose.y - pose.y;
                dx * dx + dy * dy <= max_dist_sq
            })
            .collect()
    }

    /// Clear all keyframes.
    pub fn clear(&mut self) {
        self.keyframes.clear();
        self.last_keyframe_pose = None;
        self.last_keyframe_time = 0;
        self.next_id = 0;
    }

    /// Get configuration.
    pub fn config(&self) -> &KeyframeManagerConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;

    fn create_test_scan() -> PointCloud2D {
        let mut scan = PointCloud2D::new();
        // Create a simple square pattern
        for i in 0..10 {
            let angle = i as f32 * std::f32::consts::TAU / 10.0;
            let x = 2.0 * angle.cos();
            let y = 2.0 * angle.sin();
            scan.push(Point2D::new(x, y));
        }
        scan
    }

    #[test]
    fn test_keyframe_creation() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let scan = create_test_scan();
        let keyframe = Keyframe::new(0, pose, scan, 1000, 0);

        assert_eq!(keyframe.id, 0);
        assert_eq!(keyframe.pose.x, 1.0);
        assert_eq!(keyframe.submap_id, 0);
    }

    #[test]
    fn test_scan_context() {
        let scan = create_test_scan();
        let context = ScanContext::from_scan(&scan);

        assert_eq!(context.descriptor.len(), ScanContext::DEFAULT_SECTORS * ScanContext::DEFAULT_RINGS);
        assert_eq!(context.ring_key.len(), ScanContext::DEFAULT_RINGS);
    }

    #[test]
    fn test_scan_context_self_similarity() {
        let scan = create_test_scan();
        let context = ScanContext::from_scan(&scan);

        let similarity = context.similarity(&context);
        assert!(similarity > 0.99, "Self-similarity should be ~1.0: {}", similarity);
    }

    #[test]
    fn test_keyframe_manager_first_keyframe() {
        let config = KeyframeManagerConfig::default();
        let manager = KeyframeManager::new(config);

        // Should always create first keyframe
        assert!(manager.should_create_keyframe(&Pose2D::identity(), 0));
    }

    #[test]
    fn test_keyframe_manager_motion_threshold() {
        let config = KeyframeManagerConfig {
            min_translation: 0.5,
            min_rotation: 0.5,
            min_interval_us: 0, // Disable time threshold for test
            ..Default::default()
        };
        let mut manager = KeyframeManager::new(config);

        // Create first keyframe
        let scan = create_test_scan();
        manager.create_keyframe(Pose2D::identity(), scan.clone(), 0, 0);

        // Small motion - should not create
        let small_motion = Pose2D::new(0.1, 0.0, 0.0);
        assert!(!manager.should_create_keyframe(&small_motion, 1000));

        // Large translation - should create
        let large_translation = Pose2D::new(1.0, 0.0, 0.0);
        assert!(manager.should_create_keyframe(&large_translation, 1000));

        // Large rotation - should create
        let large_rotation = Pose2D::new(0.0, 0.0, 1.0);
        assert!(manager.should_create_keyframe(&large_rotation, 1000));
    }

    #[test]
    fn test_keyframe_manager_near_query() {
        let config = KeyframeManagerConfig {
            min_translation: 0.1,
            min_interval_us: 0,
            ..Default::default()
        };
        let mut manager = KeyframeManager::new(config);

        let scan = create_test_scan();

        // Create keyframes at different positions
        manager.create_keyframe(Pose2D::new(0.0, 0.0, 0.0), scan.clone(), 0, 0);
        manager.create_keyframe(Pose2D::new(1.0, 0.0, 0.0), scan.clone(), 1000, 0);
        manager.create_keyframe(Pose2D::new(5.0, 0.0, 0.0), scan.clone(), 2000, 0);

        // Query near origin
        let near = manager.keyframes_near(&Pose2D::identity(), 2.0);
        assert_eq!(near.len(), 2); // Should find first two

        // Query near far point
        let near_far = manager.keyframes_near(&Pose2D::new(5.0, 0.0, 0.0), 1.0);
        assert_eq!(near_far.len(), 1);
    }
}
