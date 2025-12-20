# Integration Module

Map integration for combining scan features with the map.

## Components

| File | Description |
|------|-------------|
| `spatial_index.rs` | R-tree spatial index for efficient queries |
| `association.rs` | Line association (matching scan lines to map lines) |
| `merger.rs` | Feature merging (updating map lines with observations) |

## Integration Pipeline

```
Scan Features (Lines)
        │
        ▼
┌───────────────────┐
│ 1. Associate      │ ← Find correspondences to map lines
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ 2. Merge          │ ← Update matched map lines
└────────┬──────────┘
         │
         ▼
┌───────────────────┐
│ 3. Add            │ ← Insert unmatched lines as new features
└────────┬──────────┘
         │
         ▼
    Updated Map
```

## Components

### Spatial Index (R-Tree)

Efficient spatial queries using rstar R-tree:

```rust
let index = SpatialIndex::new(&lines);

// Query lines near a point
let candidates = index.query_near(point, radius);

// Query lines in a bounding box
let candidates = index.query_bounds(&bounds);
```

### Line Association

Find correspondences between scan and map lines:

```rust
let config = AssociationConfig {
    max_distance: 0.3,           // Max perpendicular distance
    max_angle_diff: 0.15,        // Max angle difference (radians)
    min_overlap_ratio: 0.3,      // Minimum segment overlap
};

// Find all associations
let associations = find_associations(&scan_lines, &map_lines, Some(&index), &config);

// Find unique associations (one-to-one mapping)
let unique = find_unique_associations(&scan_lines, &map_lines, Some(&index), &config);

// Find unmatched scan lines (for adding to map)
let unmatched = find_unmatched_scan_lines(&scan_lines, &map_lines, Some(&index), &config);
```

### Line Merging

Update map lines with new observations:

```rust
let config = MergerConfig {
    position_weight: 0.3,        // Weight for new observation
    extend_endpoints: true,      // Allow extending line endpoints
    max_extension: 0.5,          // Max extension distance
    min_observation_count: 3,    // Min observations for stable line
};

// Merge single line
let result = merge_lines(&scan_line, &mut map_line, &config);

// Batch merge
batch_merge(&scan_lines, &mut map_lines, &associations, &config);

// Create new line from unmatched scan line
let new_line = create_new_line(&scan_line);

// Merge collinear lines in the map
merge_collinear_lines(&mut map_lines, max_gap, max_angle);
```

## Association Criteria

Lines are associated when:

1. **Distance**: Perpendicular distance < `max_distance`
2. **Angle**: Angular difference < `max_angle_diff`
3. **Overlap**: Segments overlap by at least `min_overlap_ratio`

```
         Map Line
    ←────────────────→
              ╲
               ╲ angle_diff
                ╲
    ←──────────→
     Scan Line
        ↑
        │ perpendicular distance
```

## Merge Strategy

When merging lines:

1. **Endpoint Extension**: Extend map line to cover scan line extent
2. **Position Averaging**: Weighted average of line positions
3. **Observation Counting**: Track observation count for stability

```
Before:   Map:  ════════════
          Scan:       ══════════

After:    Map:  ═══════════════════
                      (extended + averaged)
```

## Best Practices

1. **Use spatial index** for maps with > 100 lines
2. **Tune association thresholds** based on sensor noise
3. **Merge collinear lines** periodically to reduce map size
4. **Track observation counts** to filter unstable features
