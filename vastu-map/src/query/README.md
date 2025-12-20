# Query Module

Map query operations for navigation and exploration.

## Components

| File | Description |
|------|-------------|
| `raycast.rs` | Ray casting to find obstacles |
| `occupancy.rs` | Point occupancy queries (free/occupied/unknown) |
| `frontier.rs` | Frontier detection for exploration |

## Raycast

Find distance to obstacles in a given direction:

```rust
// Basic raycast
let distance = raycast(origin, direction, max_range, &lines);

// With spatial index (faster)
let distance = raycast_indexed(origin, direction, max_range, &index);

// Detailed result with hit information
let result = raycast_detailed(origin, direction, max_range, &lines);
if let Some(hit) = result.hit {
    println!("Hit at {:?}, line index: {}", hit.point, hit.line_idx);
}

// Batch raycast (multiple rays at once)
let distances = raycast_batch(&origins, &directions, max_range, &lines);

// 360-degree sweep
let distances = raycast_360(origin, num_rays, max_range, &lines);

// Sweep over angle range
let distances = raycast_sweep(origin, start_angle, end_angle, num_rays, max_range, &lines);
```

## Occupancy

Query whether points are free, occupied, or unknown:

```rust
let config = OccupancyConfig {
    robot_radius: 0.15,          // Robot footprint radius
    clearance: 0.05,             // Extra safety margin
    unknown_threshold: 0.5,      // Distance threshold for "unknown"
};

// Single point query
let occupancy = query_occupancy(point, &lines, Some(&index), &config);

match occupancy {
    Occupancy::Free => println!("Navigable space"),
    Occupancy::Occupied => println!("Obstacle"),
    Occupancy::Unknown => println!("Not yet mapped"),
}

// Batch query (SIMD-friendly)
let results = query_occupancy_batch(&points, &lines, Some(&index), &config);

// Path clearance check
let clear = is_path_clear(from, to, &lines, &config);

// Region clearance check
let clear = is_region_clear(center, radius, &lines, &config);
```

## Frontier Detection

Find unexplored boundaries for exploration:

```rust
let config = FrontierConfig {
    min_length: 0.3,             // Min frontier length
    cluster_distance: 0.5,       // Distance for clustering
    min_cluster_size: 3,         // Min points per cluster
};

// Detect all frontiers
let frontiers = detect_frontiers(&lines, &config);

// Detect frontiers relative to robot position
let frontiers = detect_frontiers_from_robot(&lines, robot_pose, &config);

// Cluster nearby frontiers
let clusters = cluster_frontiers(&frontiers, cluster_distance);

// Get cluster centroid
let centroid = cluster_centroid(&cluster);

// Rank frontiers by exploration value
let ranked = rank_frontiers(&frontiers, robot_pose, &config);

// Get best frontier for exploration
if let Some(best) = get_best_frontier(&frontiers, robot_pose, &config) {
    println!("Explore: {:?}", best.point);
}
```

## Frontier Types

Frontiers are line endpoints that don't connect to other lines:

```
         ═══════════════╗
                        ║
    Frontier →  •       ║  ← Connected
                        ║
         ═══════════════╝

    Unconnected endpoints indicate unexplored areas
```

## Use Cases

### Navigation

```rust
// Check if goal is reachable
if is_path_clear(robot_pos, goal, &lines, &config) {
    // Plan direct path
}

// Virtual lidar for local planning
let distances = raycast_360(robot_pos, 360, 3.0, &lines);
```

### Exploration

```rust
// Find next exploration target
let frontiers = detect_frontiers(&lines, &config);
let ranked = rank_frontiers(&frontiers, robot_pose, &config);

if let Some(target) = ranked.first() {
    println!("Next target: {:?}", target.point);
}
```

### Collision Checking

```rust
// Check robot footprint
let clear = is_region_clear(robot_pos, robot_radius, &lines, &config);

// Check planned path
let waypoints = [p1, p2, p3, p4];
let all_clear = waypoints.windows(2).all(|pair| {
    is_path_clear(pair[0], pair[1], &lines, &config)
});
```
