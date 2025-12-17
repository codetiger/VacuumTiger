# DhruvaSLAM

Production-ready 2D SLAM (Simultaneous Localization and Mapping) implementation in Rust for robotic vacuum cleaners, targeting the Allwinner A33 embedded platform.

## Overview

DhruvaSLAM is a standalone SLAM daemon that:
1. Connects to [SangamIO](../sangam-io/README.md) daemon for sensor data (encoders, gyro, lidar)
2. Computes odometry from wheel encoders with IMU fusion
3. Performs scan-to-map matching for pose correction
4. Builds and maintains an occupancy grid map
5. Publishes corrected poses and maps over TCP for visualization

```
┌─────────────────────────────────────────────────────────────┐
│                    DhruvaSLAM Daemon                         │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────────┐ │
│  │  Odometry   │  │    SLAM     │  │   TCP Publisher      │ │
│  │  Pipeline   │  │   Engine    │  │   (Port 5557)        │ │
│  │             │  │             │  │                      │ │
│  │ Encoders ──►│  │ Lidar ─────►│  │ ──► Pose @ 50Hz      │ │
│  │ Gyro ──────►│  │ Odometry ──►│  │ ──► Map @ 1Hz        │ │
│  │             │  │             │  │ ──► Scan @ 5Hz       │ │
│  └─────────────┘  └─────────────┘  └──────────────────────┘ │
└────────────────────────┬────────────────────────────────────┘
                         │ TCP 5555 (Protobuf)
              ┌──────────▼──────────┐
              │   SangamIO Daemon   │
              │   (Robot Hardware)  │
              └─────────────────────┘
```

### Key Features

- **Configurable Odometry** - Wheel-only, Complementary, ESKF, or Mahony fusion
- **Multiple Scan Matchers** - ICP, P2L-ICP, Correlative, Multi-Resolution, Hybrid
- **Occupancy Grid Mapping** - Log-odds representation with Bresenham ray tracing
- **Feature Extraction** - Line and corner detection from occupancy maps
- **Loop Closure** - LiDAR-IRIS binary descriptors with sparse CG optimization
- **Bag Recording/Playback** - Record sensor data for offline development

### Specifications

| Property | Value |
|----------|-------|
| Language | Rust 2024 edition |
| Target | ARM Cortex-A7 (Allwinner A33) |
| Input | 110Hz sensors, 5Hz lidar |
| Output | 50Hz pose, 1Hz map, 5Hz scan |
| Memory | < 100MB for SLAM |

## Architecture

DhruvaSLAM is organized as a binary crate (not a library) with 5 logical layers:

```
┌─────────────────────────────────────────────────────┐
│                      main.rs                         │  ← Entry point
└─────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────┐
│                      io/                             │  ← Infrastructure
│         (sangam_client, bag, streaming)              │
└─────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────┐
│                    engine/                           │  ← Orchestration
│              (slam, graph optimization)              │
└─────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────┐
│                  algorithms/                         │  ← Core algorithms
│      (matching, mapping, localization, descriptors)  │
└─────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────┐
│                   sensors/                           │  ← Sensor processing
│         (odometry, preprocessing, calibration)       │
└─────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────┐
│                     core/                            │  ← Foundation
│              (types, math, simd)                     │
└─────────────────────────────────────────────────────┘
```

See individual module READMEs for details:
- [core/](src/core/README.md) - Foundation types (Pose2D, PointCloud2D, LaserScan) and SIMD
- [sensors/](src/sensors/README.md) - Odometry fusion and lidar preprocessing
- [algorithms/](src/algorithms/README.md) - Matching, mapping, localization, descriptors
- [engine/](src/engine/README.md) - SLAM orchestration, keyframes, pose graph
- [io/](src/io/README.md) - SangamIO client, bag files, TCP streaming

## Building

```bash
# Build for development
cargo build

# Build optimized release
cargo build --release

# Run tests
cargo test

# Check without building
cargo check
```

### Cross-Compilation (ARM)

```bash
# Add ARM target
rustup target add armv7-unknown-linux-musleabihf

# Build for robot
cargo build --release --target armv7-unknown-linux-musleabihf
```

## Usage

### Running the SLAM Daemon

```bash
# With default config (looks for dhruva-slam.toml)
cargo run --release

# With explicit config file
cargo run --release -- --config dhruva-slam.toml

# With command-line overrides
cargo run --release -- --sangam 192.168.68.101:5555 --port 5557

# With debug logging
RUST_LOG=debug cargo run --release

# Offline playback from bag file
cargo run --release -- --bag bags/recording.bag

# Loop bag file playback
cargo run --release -- --bag bags/recording.bag --loop
```

**Arguments:**
| Argument | Description | Default |
|----------|-------------|---------|
| `-c, --config` | Configuration file | `dhruva-slam.toml` |
| `-s, --sangam` | SangamIO address | `192.168.68.101:5555` |
| `-p, --port` | TCP publish port | `5557` |
| `-b, --bag` | Bag file for offline playback | (none) |
| `-l, --loop` | Loop bag file playback | false |

## Configuration

Edit `dhruva-slam.toml`:

```toml
[source]
sangam_address = "192.168.68.101:5555"

[output]
bind_port = 5557
odometry_rate_hz = 50.0
map_rate_hz = 1.0
feature_rate_hz = 0.2

[slam]
min_range = 0.15              # Lidar min range (m)
max_range = 8.0               # Lidar max range (m)

[lidar]
mounting_x = -0.0936          # Lidar X offset from robot center (m)
mounting_y = 0.0
optical_offset = 0.0258       # Optical center offset (m)
angle_offset = 0.0            # Angular calibration (rad)

[odometry]
algorithm = "mahony"          # wheel, complementary, eskf, mahony
ticks_per_meter = 4464.0      # Encoder calibration
wheel_base = 0.233            # Distance between wheels (m)

[matcher]
algorithm = "multi_res"       # icp, p2l, correlative, multi_res, hybrid_icp, hybrid_p2l

[loop_closure]
enabled = true
detection_interval = 5        # Check every N keyframes
min_keyframe_gap = 20         # Skip recent poses
optimization_threshold = 3    # Optimize after N loop closures

[optimization]
solver = "sparse_cg"          # dense_cholesky, sparse_cg
max_iterations = 50
tolerance = 1e-6
```

### Key Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `odometry.algorithm` | Fusion algorithm | `mahony` (recommended) |
| `odometry.ticks_per_meter` | Encoder ticks per meter | 4464 (CRL-200S) |
| `odometry.wheel_base` | Distance between wheel centers | 0.233m |
| `matcher.algorithm` | Scan matching algorithm | `multi_res` (recommended) |
| `slam.min_range` | Minimum valid lidar range | 0.15m |
| `slam.max_range` | Maximum valid lidar range | 8.0m |

## TCP Output Protocol

The daemon publishes data on the configured port (default 5557) using length-prefixed Protobuf:

```
┌──────────────────┬─────────────────────┐
│ Length (4 bytes) │ Protobuf Payload    │
│ Big-endian u32   │ (binary)            │
└──────────────────┴─────────────────────┘
```

See `proto/dhruva.proto` for the complete schema.

### Published Messages

| Message Type | Rate | Description |
|--------------|------|-------------|
| `OdometryMessage` | 50Hz | Corrected pose (x, y, theta) |
| `SlamMapMessage` | 1Hz | Occupancy grid map (base64) |
| `SlamScanMessage` | 5Hz | Processed lidar scan with pose |
| `FeatureMessage` | 0.2Hz | Extracted lines and corners |

## Module Structure

```
dhruva-slam/
├── src/
│   ├── main.rs               # Entry point, config, main loop
│   │
│   ├── core/                 # Foundation layer
│   │   ├── types/            # Pose2D, Point2D, PointCloud2D, LaserScan
│   │   ├── math.rs           # Angle normalization, interpolation
│   │   └── simd/             # SIMD vector types (Float4)
│   │
│   ├── sensors/              # Sensor processing
│   │   ├── calibration.rs    # Gyro bias estimation
│   │   ├── odometry/         # Wheel, Complementary, ESKF, Mahony
│   │   └── preprocessing/    # Range filter, downsampler, outlier removal
│   │
│   ├── algorithms/           # Core algorithms
│   │   ├── matching/         # ICP, P2L-ICP, Correlative, Multi-Resolution
│   │   ├── mapping/          # OccupancyGrid, RayTracer, FeatureExtractor
│   │   ├── localization/     # SensorModel, MotionModel (MCL)
│   │   └── descriptors/      # LiDAR-IRIS for loop closure
│   │
│   ├── engine/               # SLAM orchestration
│   │   ├── slam/             # OnlineSlam, Keyframe, Submap, Recovery
│   │   └── graph/            # PoseGraph, LoopDetector, SparseCG Optimizer
│   │
│   ├── io/                   # I/O infrastructure
│   │   ├── sangam_client.rs  # SangamIO TCP client
│   │   ├── bag/              # BagRecorder, BagPlayer, SimulatedClient
│   │   └── streaming/        # OdometryPublisher, message types
│   │
│   └── metrics/              # Quality metrics
│       ├── scan_match_error.rs
│       └── map_noise.rs
│
├── proto/                    # Protobuf schemas
│   └── dhruva.proto          # Output message definitions
│
├── bags/                     # Recorded sensor data
├── results/                  # Benchmark results
├── docs/                     # Design documents
│
├── Cargo.toml                # Dependencies
└── dhruva-slam.toml          # Runtime configuration
```

## Data Flow

```
SangamIO (110Hz sensors, 5Hz lidar)
    │
    ▼
┌───────────────────────────────────────┐
│           SangamClient                │
│  • TCP connection (Protobuf)          │
│  • Message deserialization            │
└───────────────────┬───────────────────┘
                    │
        ┌───────────┴───────────┐
        ▼                       ▼
┌───────────────┐      ┌───────────────┐
│   Odometry    │      │  Preprocess   │
│   Pipeline    │      │   + SLAM      │
│               │      │               │
│ • Wheel odom  │      │ • Range filter│
│ • IMU fusion  │─────►│ • Downsample  │
│ (Mahony/ESKF) │      │ • Match to map│
│               │      │ • Update map  │
└───────┬───────┘      └───────┬───────┘
        │                      │
        └──────────┬───────────┘
                   ▼
        ┌──────────────────┐
        │  TCP Publisher   │
        │  • Pose @ 50Hz   │
        │  • Map @ 1Hz     │
        │  • Scan @ 5Hz    │
        │  • Features      │
        └──────────────────┘
                   │
                   ▼
          Visualization Client
              (Drishti)
```

## Testing

```bash
# Run all tests
cargo test

# Run specific test
cargo test test_wheel_odometry

# Run with output
cargo test -- --nocapture
```

## Algorithm Selection Guide

### Odometry Algorithms

| Algorithm | Use Case | CPU |
|-----------|----------|-----|
| `wheel` | Testing, baseline | Very Low |
| `complementary` | Simple fusion | Low |
| `eskf` | Production with uncertainty | Medium |
| `mahony` | **Recommended** - auto gyro calibration | Low |

### Scan Matchers

| Algorithm | Use Case | CPU | Accuracy |
|-----------|----------|-----|----------|
| `icp` | Small displacements | Low | Good |
| `p2l` | Structured environments (walls) | Medium | Very Good |
| `correlative` | Large initial errors | High | Good |
| `multi_res` | **Recommended** - balanced | Medium | Good |
| `hybrid_icp` | Correlative + ICP refinement | High | Very Good |
| `hybrid_p2l` | Correlative + P2L refinement | High | Excellent |

## Dependencies

| Crate | Purpose |
|-------|---------|
| `prost` | Protobuf serialization |
| `serde` | Configuration serialization |
| `postcard` | Bag file format |
| `kiddo` | K-d tree for nearest neighbor |
| `basic_toml` | Config parsing |
| `env_logger` | Logging |
| `ctrlc` | Signal handling |
| `approx` | Floating-point comparison in tests |

## Design Notes

### Design Principles

| Principle | Description |
|-----------|-------------|
| **Binary, Not Library** | Standalone daemon, not imported as a library |
| **Runtime Configuration** | Algorithm selection via TOML config |
| **Embedded-First** | Optimized for Allwinner A33 (ARM Cortex-A7) |
| **Testability** | Components unit-testable with bag file playback |

### Platform Considerations (Allwinner A33)

**Target specs:** Quad-core ARM Cortex-A7 @ 1.2 GHz, 512MB RAM, NEON SIMD

**Performance Budget (5 Hz SLAM target):**

| Component | Budget | Notes |
|-----------|--------|-------|
| Preprocessing | 5 ms | Range filter + downsample to 180 pts |
| Odometry Fusion | 1 ms | Mahony AHRS |
| Scan Matching | 50 ms | Multi-resolution correlative |
| Map Update | 10 ms | Log-odds with ray tracing |
| Feature Extraction | 20 ms | Line/corner detection |

**Memory Budget (< 100 MB):**

| Component | Allocation |
|-----------|------------|
| Occupancy Grid | 40 MB |
| Scan Buffers | 10 MB |
| K-d Trees | 10 MB |
| Working Memory | 40 MB |

### Known Limitations

- Loop closure detection is implemented but not fully integrated into pose correction
- No multi-session mapping (map not persisted between runs)
- Single-threaded (no background optimization thread yet)

### Future Work

See [IMPROVEMENTS.md](IMPROVEMENTS.md) for detailed roadmap:
- Full loop closure integration with pose graph correction
- Background optimization thread
- Multi-resolution map representation
- GICP scan matching

## Related Documents

- [IMPROVEMENTS.md](IMPROVEMENTS.md) - Technical improvement roadmap
- [docs/LOOP_CLOSURE_PROPOSAL.md](docs/LOOP_CLOSURE_PROPOSAL.md) - LiDAR-IRIS implementation plan

## License

See the root [LICENSE](../LICENSE) file.
