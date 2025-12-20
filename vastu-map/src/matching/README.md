# Matching Module

Scan-to-map matching algorithms for robot localization.

## Components

| File | Description |
|------|-------------|
| `correspondence.rs` | Point-to-line correspondence types and match results |
| `nearest_neighbor.rs` | Find correspondences by distance |
| `ransac.rs` | Robust pose estimation with outlier rejection |
| `gauss_newton.rs` | Nonlinear least squares optimization |
| `icp/` | Point-to-Line ICP implementation |
| `scratch.rs` | Pre-allocated buffers for zero-allocation ICP |
| `traits.rs` | `ScanMatcher` trait for extensibility |

## Algorithms

### Point-to-Line ICP

The primary matching algorithm. Aligns scan points to map lines:

```rust
let result = match_scan(&points, &lines, initial_pose);

if result.converged {
    println!("Pose: ({:.3}, {:.3}, {:.3})",
        result.pose.x, result.pose.y, result.pose.theta);
    println!("Confidence: {:.2}", result.confidence);
}
```

With configuration:

```rust
let config = IcpConfig {
    max_iterations: 20,
    convergence_threshold: 0.001,
    max_correspondence_distance: 0.5,
    outlier_rejection: OutlierRejection::MedianAbsoluteDeviation { k: 2.0 },
    coarse_search: Some(CoarseSearchConfig::default()),
    ..Default::default()
};

let result = match_scan_with_config(&points, &lines, initial_pose, &config);
```

### Zero-Allocation Matching

For hot paths, reuse scratch space:

```rust
let mut scratch = IcpScratchSpace::default_capacity();
let icp = PointToLineIcp::new();

for scan in scans {
    let result = icp.match_scan_with_scratch(&scan, &map, pose, &mut scratch);
}
```

### RANSAC Pose Estimation

Robust estimation with outlier rejection:

```rust
let config = RansacConfig {
    max_iterations: 100,
    inlier_threshold: 0.1,
    min_inliers: 10,
    confidence: 0.99,
};

let result = estimate_pose_ransac(&correspondences, &config);
```

### Gauss-Newton Optimization

Nonlinear least squares for pose refinement:

```rust
let config = GaussNewtonConfig {
    max_iterations: 10,
    convergence_threshold: 1e-6,
};

let result = optimize_pose_fast(&correspondences, initial_pose, &config);
```

## Correspondence Finding

```rust
// Basic nearest neighbor
let correspondences = find_correspondences(&points, &lines, max_distance);

// With spatial index (faster for large maps)
let correspondences = find_correspondences_spatial(&points, &lines, &index, max_distance);

// Batch processing (SIMD-friendly)
let correspondences = find_correspondences_batch(&points, &lines, max_distance);

// Weighted by confidence
let correspondences = find_correspondences_weighted(&points, &lines, max_distance, weights);

// With angle constraint
let correspondences = find_correspondences_with_angle(
    &points, &lines, max_distance, max_angle_diff
);
```

## Extensibility

Implement `ScanMatcher` trait for custom matching:

```rust
pub trait ScanMatcher: Send + Sync {
    fn match_scan(
        &self,
        points: &[Point2D],
        lines: &[Line2D],
        initial_pose: Pose2D,
    ) -> MatchResult;
}
```

## ICP Pipeline

```
Initial Pose (from odometry)
        │
        ▼
┌───────────────────┐
│ Coarse Search     │ ← Grid search around initial pose
│ (optional)        │
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ Find              │ ← Nearest neighbor in map lines
│ Correspondences   │
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ Outlier Rejection │ ← MAD or distance threshold
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ Gauss-Newton      │ ← Minimize point-to-line distance
│ Optimization      │
└────────┬──────────┘
         │
         ▼
    Converged? ────No──┐
         │             │
        Yes            │
         │             │
         ▼             │
    Matched Pose  ◄────┘
```

## Confidence Computation

Match confidence is based on:
- Inlier ratio (percentage of points with good correspondences)
- Mean residual error (average point-to-line distance)
- Number of correspondences

```rust
// Confidence ranges from 0.0 to 1.0
// 0.0 = no match (fallback to odometry)
// 1.0 = perfect match
```
