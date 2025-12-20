# Loop Closure Module

Keyframe-based loop closure detection using shape context descriptors.

## Overview

Loop closure detection identifies when the robot revisits a previously mapped area. This enables:

- **Drift correction**: Correct accumulated odometry errors
- **Global consistency**: Maintain accurate global map
- **Pose graph optimization**: Provide constraints for optimization

## Components

| Type | Description |
|------|-------------|
| `LoopClosureDetector` | Main detector with keyframe storage |
| `LoopClosureConfig` | Configuration parameters |
| `Keyframe` | Snapshot of map state at a pose |
| `LoopClosure` | Detected loop closure constraint |
| `LoopDetector` | Trait for extensibility |

## Pipeline

```
New Observation
        │
        ▼
┌───────────────────┐
│ Should Create     │ ← Check distance from last keyframe
│ Keyframe?         │
└────────┬──────────┘
         │ Yes
         ▼
┌───────────────────┐
│ Compute Scan      │ ← Shape context descriptor
│ Descriptor        │
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ Find Candidates   │ ← Descriptor similarity + min travel distance
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ Verify with ICP   │ ← Point-to-line matching
└────────┬──────────┘
         │
         ▼
    Loop Closure
    (if verified)
```

## Usage

```rust
use vastu_map::loop_closure::{LoopClosureDetector, LoopClosureConfig};
use vastu_map::core::Pose2D;

let config = LoopClosureConfig::default();
let mut detector = LoopClosureDetector::new(config);

// Process observations
for (pose, lines, corners, points) in observations {
    if let Some(closure) = detector.process(pose, &lines, &corners, &points) {
        println!("Loop closure: {} -> {}",
            closure.from_keyframe, closure.to_keyframe);
        println!("Confidence: {:.2}", closure.confidence);

        // Use closure for pose graph optimization
        optimize_pose_graph(&closure);
    }
}
```

## Configuration

```rust
let config = LoopClosureConfig {
    // Keyframe creation
    keyframe_interval: 2.0,           // Distance between keyframes (m)
    min_lines_for_keyframe: 3,        // Min lines required
    min_corners_for_keyframe: 2,      // Min corners required

    // Candidate selection
    min_travel_distance: 5.0,         // Min travel before loop check (m)
    max_descriptor_distance: 0.5,     // Max descriptor difference
    max_candidates_to_verify: 3,      // Top candidates to verify

    // Verification
    min_verification_confidence: 0.6, // Min ICP confidence
    descriptor_radius: 2.0,           // Radius for descriptor (m)

    // ICP for verification
    icp_config: IcpConfig::default(),
};
```

## Keyframe

Stores a snapshot for later matching:

```rust
pub struct Keyframe {
    pub id: usize,                    // Unique identifier
    pub pose: Pose2D,                 // Robot pose when created
    pub descriptor: ScanDescriptor,   // Shape context descriptor
    pub lines: Vec<Line2D>,           // Line features
    pub corners: Vec<Corner2D>,       // Corner features
    pub points: Vec<Point2D>,         // Raw points (for ICP)
    pub distance_traveled: f32,       // Total travel distance
}
```

## Loop Closure Result

```rust
pub struct LoopClosure {
    pub from_keyframe: usize,         // Earlier keyframe ID
    pub to_keyframe: usize,           // Current keyframe ID
    pub relative_pose: Pose2D,        // Relative transform
    pub covariance: [[f32; 3]; 3],    // Pose covariance
    pub confidence: f32,              // Match confidence (0-1)
    pub descriptor_distance: f32,     // Descriptor similarity
}
```

## Descriptor Matching

Two-stage matching for efficiency:

1. **Quick Signature**: 64-bit hash for fast pre-filtering
2. **Full Descriptor**: Shape context distance for final matching

```rust
// Quick pre-filter
let sig_diff = (desc_a.quick_signature() ^ desc_b.quick_signature()).count_ones();
if sig_diff > 16 {
    return false; // Too different
}

// Full comparison
let distance = desc_a.distance(&desc_b);
if distance <= max_descriptor_distance {
    // Candidate for ICP verification
}
```

## Extensibility

Implement `LoopDetector` trait for custom algorithms:

```rust
pub trait LoopDetector: Send + Sync {
    fn detect(
        &mut self,
        pose: Pose2D,
        lines: &[Line2D],
        corners: &[Corner2D],
        points: &[Point2D],
    ) -> Option<LoopClosure>;

    fn clear(&mut self);
    fn can_detect(&self) -> bool;
    fn keyframe_count(&self) -> usize;
}
```

## Best Practices

1. **Tune min_travel_distance** to avoid false positives in revisited areas
2. **Verify with ICP** - descriptor matching alone is not sufficient
3. **Use loop closures** for pose graph optimization, not direct correction
4. **Monitor keyframe count** to manage memory usage
