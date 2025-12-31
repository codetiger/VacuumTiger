# Modes Module

SLAM operation modes for different use cases.

## Module Structure

```
modes/
├── mod.rs          # Module exports
└── localization.rs # Localization-only mode (no map updates)
```

## Operation Modes

VastuSLAM supports different operational modes depending on the use case:

| Mode | Map Updates | Use Case |
|------|-------------|----------|
| **Full SLAM** | Yes | Initial mapping, exploration |
| **Localization** | No | Navigation on known map |

## Localization Mode

When the environment is already mapped, use localization mode for efficient pose tracking without modifying the map.

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                     LOCALIZATION PIPELINE                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  LidarScan + Odometry Delta                                         │
│       │                                                              │
│       ▼                                                              │
│  ┌────────────────────────────────────────┐                         │
│  │        ODOMETRY PREDICTION             │                         │
│  │  predicted_pose = current ⊕ delta      │                         │
│  └───────────────────┬────────────────────┘                         │
│                      │                                               │
│                      ▼                                               │
│  ┌────────────────────────────────────────┐                         │
│  │         SCAN MATCHING                  │                         │
│  │  Match scan against known map          │                         │
│  │  (CorrelativeMatcher)                  │                         │
│  └───────────────────┬────────────────────┘                         │
│                      │                                               │
│                      ▼                                               │
│  ┌────────────────────────────────────────┐                         │
│  │       DEVIATION CHECK                  │                         │
│  │  Is matched pose within bounds?        │                         │
│  │  linear ≤ 0.5m, angular ≤ 0.5 rad     │                         │
│  └───────────────────┬────────────────────┘                         │
│                      │                                               │
│           ┌──────────┴──────────┐                                   │
│           │                     │                                    │
│     converged=true        converged=false                           │
│           │                     │                                    │
│           ▼                     ▼                                    │
│    Use matched pose      Use predicted pose                         │
│    (scan-corrected)      (odometry fallback)                        │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Basic Usage

```rust
use vastu_slam::modes::{Localizer, LocalizerConfig};
use vastu_slam::io::load_vastu;
use vastu_slam::core::{Pose2D, LidarScan};
use std::path::Path;

// Load a pre-built map
let map = load_vastu(Path::new("map.vastu"))?;

// Create localizer with default config
let config = LocalizerConfig::default();
let mut localizer = Localizer::new(map, config);

// Set initial pose (from user input, QR code, or dock position)
localizer.set_initial_pose(Pose2D::new(1.0, 2.0, 0.0));

// Main localization loop
loop {
    let scan = get_lidar_scan();
    let odom_delta = get_odometry_delta();

    let result = localizer.localize(&scan, Some(odom_delta));

    if result.converged {
        println!("Localized at: ({:.2}, {:.2}) θ={:.1}°",
            result.pose.x, result.pose.y,
            result.pose.theta.to_degrees());
        println!("Confidence: {:.1}%", result.score * 100.0);
    } else {
        println!("Localization uncertain, using odometry");
    }
}
```

### Configuration

```rust
use vastu_slam::modes::LocalizerConfig;
use vastu_slam::matching::CorrelativeMatcherConfig;

let config = LocalizerConfig {
    // Scan matcher settings
    matcher: CorrelativeMatcherConfig {
        search_x: 0.20,           // Smaller search for localization
        search_y: 0.20,
        search_theta: 0.10,
        multi_resolution: true,
        ..Default::default()
    },

    // Acceptance thresholds
    min_score: 0.5,               // Minimum match quality (0.0-1.0)
    max_linear_deviation: 0.5,    // Max distance from prediction (m)
    max_angular_deviation: 0.5,   // Max rotation from prediction (rad)
};
```

### Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_score` | 0.5 | Minimum score to accept match |
| `max_linear_deviation` | 0.5m | Max translation from odometry prediction |
| `max_angular_deviation` | 0.5 rad | Max rotation from odometry prediction |

### Localization Result

```rust
pub struct LocalizationResult {
    pub pose: Pose2D,              // Estimated robot pose
    pub score: f32,                // Match score (0.0 - 1.0)
    pub converged: bool,           // Overall success
    pub within_bounds: bool,       // Passed deviation check
    pub match_result: ScanMatchResult, // Raw matcher output
}
```

### Initialization

The localizer requires an initial pose estimate:

```rust
// Option 1: Known starting position (dock, charger)
localizer.set_initial_pose(Pose2D::new(0.0, 0.0, 0.0));

// Option 2: User-provided position
localizer.set_initial_pose(user_selected_pose);

// Option 3: Global localization (not yet implemented)
// Would use particle filter or multi-hypothesis matching
```

### Failure Modes

```rust
let result = localizer.localize(&scan, Some(odom));

if !result.converged {
    // Possible causes:

    if !result.match_result.converged {
        // Scan matcher failed to find good match
        // - Robot in unmapped area
        // - Map changed significantly
        // - Sensor malfunction
    }

    if result.score < config.min_score {
        // Match quality too low
        // - Partial occlusion
        // - Dynamic obstacles
    }

    if !result.within_bounds {
        // Matched pose too far from prediction
        // - Wheel slip / kidnapped robot
        // - Odometry error
    }
}
```

### Robot Kidnapping Detection

When the matched pose deviates significantly from odometry prediction, the robot may have been moved:

```rust
let result = localizer.localize(&scan, Some(odom));

if result.match_result.converged && result.score >= 0.7 && !result.within_bounds {
    // High-confidence match but large deviation
    // Robot was likely moved (kidnapped)

    // Option 1: Accept new position
    localizer.set_initial_pose(result.match_result.pose);

    // Option 2: Alert and require confirmation
    alert_user("Robot position changed unexpectedly");
}
```

## Localization vs Full SLAM

| Aspect | Localization Mode | Full SLAM |
|--------|-------------------|-----------|
| Map updates | Never | Continuous |
| Memory usage | Fixed | Growing |
| CPU usage | Lower | Higher |
| Handles changes | No | Yes |
| Loop closure | N/A | Active |
| Best for | Known, static environments | Exploration, dynamic environments |

### When to Use Localization Mode

- Robot is navigating a previously mapped area
- Map is known to be accurate and current
- Lower CPU/memory usage needed
- Reproducible paths required

### When to Use Full SLAM

- First-time mapping
- Environment has changed
- Exploring new areas
- Long-duration operation with potential drift

## Thread Safety

| Type | Thread Safety |
|------|---------------|
| `Localizer` | `Send` (single owner) |
| `LocalizerConfig` | `Clone + Send + Sync` |
| `LocalizationResult` | `Clone + Send` |

Typical pattern: main thread owns localizer, results can be sent to other threads.

## Performance

| Operation | Typical Time | Notes |
|-----------|--------------|-------|
| `localize()` | 5-15ms | Depends on scan size and search window |
| Initial pose set | <1μs | Just assignment |
| Map loading | 10-50ms | One-time startup cost |

The localizer uses the same correlative matcher as full SLAM but with smaller search windows for faster operation.
