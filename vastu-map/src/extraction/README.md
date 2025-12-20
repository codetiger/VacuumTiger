# Extraction Module

Feature extraction algorithms for converting point clouds to geometric features.

## Components

| File | Description |
|------|-------------|
| `line_fitting.rs` | Total Least Squares (TLS) line fitting with weighted variants |
| `split_merge/` | Split-and-Merge algorithm for line extraction |
| `corner_detection.rs` | Corner detection at line intersections |
| `traits.rs` | `LineExtractor` trait for extensibility |

## Algorithms

### Line Fitting (Total Least Squares)

Orthogonal regression minimizes perpendicular distance to the line:

```rust
// Fit line to points
let line = fit_line(&points);

// Weighted fitting (accounts for range-based uncertainty)
let weights = compute_range_weights(&points, sensor_pos);
let line = fit_line_weighted(&points, &weights);

// Fit from sensor position (auto-computes weights)
let line = fit_line_from_sensor(&points, sensor_pos, None);
```

### Split-and-Merge

Recursive algorithm for extracting lines from ordered point sequences:

1. **Split**: Find point with maximum distance from line
2. **Recurse**: If distance > threshold, split at that point
3. **Merge**: Combine collinear adjacent segments

```rust
let config = SplitMergeConfig::default();
let lines = extract_lines(&points, &config);

// With sensor position for range weighting
let lines = extract_lines_from_sensor(&points, sensor_pos, &config);

// Adaptive threshold based on point density
let threshold = adaptive_split_threshold(&points, base_threshold);
```

Configuration options:
- `split_threshold`: Max perpendicular distance before split (meters)
- `min_segment_points`: Minimum points per segment
- `min_segment_length`: Minimum segment length (meters)
- `merge_max_angle`: Max angle difference for merging (radians)
- `merge_max_gap`: Max gap between segments for merging (meters)

### Corner Detection

Find corners at line intersections:

```rust
let config = CornerConfig::default();
let corners = detect_corners(&lines, &config);

// Curvature-based detection (from point cloud)
let corners = detect_curvature_corners(&points, &config);

// Hybrid: intersection + curvature
let corners = detect_hybrid_corners(&lines, &points, &config);

// Remove duplicate corners
let unique = deduplicate_corners(&corners, min_distance);
```

## Extensibility

Implement `LineExtractor` trait for custom extraction algorithms:

```rust
pub trait LineExtractor: Send + Sync {
    fn extract(&self, points: &[Point2D]) -> Vec<Line2D>;
    fn extract_with_pose(&self, points: &[Point2D], sensor_pose: Pose2D) -> Vec<Line2D>;
}
```

Built-in implementation:
- `SplitMergeExtractor`: Default split-merge algorithm

## Pipeline

Typical extraction pipeline:

```
PolarScan → PointCloud2D → Split-Merge → Lines → Corner Detection → Corners
    │                           │                        │
    └─ to_cartesian()           └─ extract_lines()       └─ detect_corners()
```
