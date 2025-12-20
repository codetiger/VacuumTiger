# Features Module

Geometric feature types for VectorMap SLAM.

## Components

| File | Description |
|------|-------------|
| `line.rs` | `Line2D` - Line segment defined by endpoints |
| `line_collection.rs` | `LineCollection` - SoA collection for SIMD batch operations |
| `corner.rs` | `Corner2D` and `CornerCollection` - Corner features at line intersections |
| `feature_set.rs` | `FeatureSet` - Combined container for lines and corners |
| `descriptors.rs` | `CornerDescriptor` and `ScanDescriptor` - Shape context descriptors for loop closure |

## Design Rationale

### Why Lines Instead of Points?

Indoor environments are dominated by straight edges (walls, furniture, doorframes). Using line segments provides:

- **Compact representation**: One line replaces dozens of points
- **Robust matching**: Lines survive partial occlusion and noise
- **Meaningful geometry**: Lines encode structure, not just shape
- **Efficient ICP**: Point-to-line ICP converges faster than point-to-point

### Why Endpoints, Not Parametric Form?

Lines use endpoint representation `(start, end)` instead of parametric `(rho, theta)`:

- **Simpler transforms**: Just rotate+translate two points
- **SIMD-friendly**: 4 floats per line, no trig during transform
- **No angle wrapping**: No discontinuity at +/-pi
- **Built-in bounds**: Segment extent is implicit in endpoints

### Why Corners?

Corners (line intersections) provide:

- **Reliable landmarks**: Corners are stable across viewpoints
- **Loop closure signatures**: Corner patterns are distinctive
- **Pose constraints**: Two corners fully constrain 2D pose

## Key Types

### Line2D

```rust
let line = Line2D::new(Point2D::new(0.0, 0.0), Point2D::new(5.0, 0.0));

line.length();                    // Segment length
line.midpoint();                  // Center point
line.direction();                 // Unit direction vector
line.normal();                    // Perpendicular unit vector
line.distance_to_point(point);    // Perpendicular distance
line.project_point(point);        // Closest point on line
```

### LineCollection (SoA Layout)

```rust
let mut collection = LineCollection::new();
collection.push(&line);

// Batch transform (SIMD-friendly)
let transformed = collection.transform(&pose);

// Access raw arrays
let start_xs: &[f32] = collection.start_xs();
```

### Corner2D

```rust
let corner = Corner2D::new(
    point,          // Corner position
    line1_idx,      // First line index
    line2_idx,      // Second line index
    angle,          // Interior angle
);
```

### ScanDescriptor

Shape context descriptor for loop closure:

```rust
let descriptor = ScanDescriptor::compute(&lines, &corners, radius);

// Quick pre-filter with bit signature
let sig_diff = (desc_a.quick_signature() ^ desc_b.quick_signature()).count_ones();

// Full descriptor distance
let distance = desc_a.distance(&desc_b);
```

## Data Layout

Feature collections use Struct-of-Arrays for SIMD efficiency:

```
AoS (Array of Structs):         SoA (Struct of Arrays):
[Line{x1,y1,x2,y2}]             start_xs: [x1, x1, x1, ...]
[Line{x1,y1,x2,y2}]    →        start_ys: [y1, y1, y1, ...]
[Line{x1,y1,x2,y2}]             end_xs:   [x2, x2, x2, ...]
                                end_ys:   [y2, y2, y2, ...]
```

SoA enables processing 4 lines per SIMD instruction.
