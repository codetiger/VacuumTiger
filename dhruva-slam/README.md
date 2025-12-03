# DhruvaSLAM

Modular SLAM (Simultaneous Localization and Mapping) implementation in Rust for robotic vacuum cleaners.

## Overview

DhruvaSLAM computes odometry from wheel encoders and gyroscopes, then performs online SLAM using laser scans from a 2D lidar to build maps and refine pose estimates in real-time. It connects to a [SangamIO](../sangam-io/README.md) daemon running on the robot.

```
┌─────────────────────────────────────────────────────────────┐
│                    DhruvaSLAM Node                          │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────────┐ │
│  │  Odometry   │  │    SLAM     │  │   TCP Publisher      │ │
│  │  Pipeline   │  │   Engine    │  │   (Port 5557)        │ │
│  │             │  │             │  │                      │ │
│  │ Encoders ──►│  │ Lidar ─────►│  │ ──► Pose             │ │
│  │ Gyro ──────►│  │ Odometry ──►│  │ ──► Map              │ │
│  │             │  │             │  │ ──► Status           │ │
│  └─────────────┘  └─────────────┘  └──────────────────────┘ │
└────────────────────────┬────────────────────────────────────┘
                         │ TCP 5555
              ┌──────────▼──────────┐
              │   SangamIO Daemon   │
              │   (Robot Hardware)  │
              └─────────────────────┘
```

### Key Features

- **Wheel Odometry** - Encoder-based dead reckoning with gyro fusion
- **Scan Matching** - Point-to-point ICP, point-to-line ICP, correlative matching
- **Occupancy Grid Mapping** - Log-odds 2D mapping with ray tracing
- **Online SLAM** - Keyframe selection, submap management, loop closure
- **Pose Graph Optimization** - Global consistency via graph optimization
- **Bag Recording/Playback** - Record sensor data for offline testing

### Specifications

| Property | Value |
|----------|-------|
| Language | Rust 2021 edition |
| Version | 0.1.0 |
| Input Rate | 500Hz (sensors), 5Hz (lidar) |
| Output Rate | Configurable (default 50Hz odometry) |

## Architecture

DhruvaSLAM uses a layered architecture with clear separation of concerns:

```
Layer 5: I/O          sangam_client, bag, streaming
Layer 4: Engine       slam, graph optimization
Layer 3: Algorithms   matching, mapping, localization
Layer 2: Sensors      odometry, preprocessing
Layer 1: Core         types, math primitives
```

See individual module READMEs for details:
- [core/](src/core/README.md) - Foundation types and math
- [sensors/](src/sensors/README.md) - Odometry and scan preprocessing
- [algorithms/](src/algorithms/README.md) - Matching, mapping, localization
- [engine/](src/engine/README.md) - SLAM orchestration and pose graph
- [io/](src/io/README.md) - SangamIO client, bag files, TCP streaming

## Building

```bash
# Build
cargo build --release

# Run tests
cargo test

# Run benchmarks
cargo bench
```

## Usage

### Running the SLAM Node

```bash
# With configuration file
cargo run --bin dhruva-slam-node -- --config dhruva-slam.toml

# With command-line arguments
cargo run --bin dhruva-slam-node -- --sangam 192.168.68.101:5555 --port 5557

# With debug logging
RUST_LOG=debug cargo run --bin dhruva-slam-node -- --sangam 192.168.68.101:5555
```

**Arguments:**
| Argument | Description | Default |
|----------|-------------|---------|
| `-c, --config` | Configuration file | `dhruva-slam.toml` |
| `-s, --sangam` | SangamIO address | `192.168.68.101:5555` |
| `-p, --port` | TCP publish port | `5557` |

### Recording Sensor Data

```bash
# Record for 60 seconds
cargo run --bin bag-record -- \
  --sangam 192.168.68.101:5555 \
  --output recording.bag \
  --duration 60

# Record until Ctrl-C
cargo run --bin bag-record -- \
  --sangam 192.168.68.101:5555 \
  --output recording.bag
```

### Inspecting Bag Files

```bash
# Show bag info
cargo run --bin bag-info -- recording.bag

# Verbose output with message counts
cargo run --bin bag-info -- --verbose --count recording.bag
```

## Configuration

Edit `dhruva-slam.toml`:

```toml
[source]
sangam_address = "192.168.68.101:5555"

[output]
bind_port = 5557
odometry_rate_hz = 50.0
slam_status_rate_hz = 5.0
slam_map_rate_hz = 1.0

[odometry]
ticks_per_meter = 4464.0      # Encoder calibration
wheel_base = 0.233            # Distance between wheels (m)
alpha = 0.8                   # Gyro weight in complementary filter
gyro_scale = 0.0001745        # Raw gyro to rad/s
gyro_bias_z = 0.0             # Auto-calibrated at startup

[slam]
enabled = true
initial_mode = "Mapping"      # Mapping, Localization, or Idle
min_range = 0.15              # Lidar min range (m)
max_range = 8.0               # Lidar max range (m)
```

### Key Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `ticks_per_meter` | Encoder ticks per meter traveled | 4464 (CRL-200S) |
| `wheel_base` | Distance between wheel centers | 0.233m |
| `alpha` | Gyro weight (0=encoders only, 1=gyro only) | 0.8 |
| `gyro_scale` | Raw gyro units to rad/s | 0.0001745 |

## TCP Output Protocol

The node publishes data on the configured port (default 5557) using length-prefixed JSON:

```
┌──────────────────┬─────────────────────┐
│ Length (4 bytes) │ JSON Payload        │
│ Big-endian u32   │                     │
└──────────────────┴─────────────────────┘
```

### Published Topics

| Topic | Rate | Description |
|-------|------|-------------|
| `odometry` | 50Hz | Current pose (x, y, theta) |
| `slam/status` | 5Hz | SLAM status (mode, keyframes, etc.) |
| `slam/map` | 1Hz | Occupancy grid map |
| `slam/scan` | 5Hz | Processed lidar scan |
| `slam/diagnostics` | 5Hz | Timing and statistics |

### Example Odometry Message

```json
{
  "topic": "odometry",
  "timestamp_us": 1701612345000000,
  "pose": {
    "x": 1.234,
    "y": 0.567,
    "theta": 0.123
  }
}
```

### Example SLAM Status Message

```json
{
  "topic": "slam/status",
  "timestamp_us": 1701612345000000,
  "mode": "Mapping",
  "num_scans": 1234,
  "num_keyframes": 45,
  "num_submaps": 3,
  "last_match_score": 0.95,
  "is_lost": false
}
```

## Module Structure

```
dhruva-slam/
├── src/
│   ├── lib.rs                 # Library root
│   ├── bin/
│   │   ├── dhruva_slam_node.rs   # Main SLAM daemon
│   │   ├── bag_record.rs         # Recording tool
│   │   └── bag_info.rs           # Bag inspection tool
│   │
│   ├── core/                  # Foundation layer
│   │   ├── types/             # Point2D, Pose2D, LaserScan, etc.
│   │   └── math.rs            # Angle normalization, interpolation
│   │
│   ├── sensors/               # Sensor processing
│   │   ├── odometry/          # Wheel odometry, filters
│   │   └── preprocessing/     # Lidar scan filtering
│   │
│   ├── algorithms/            # Core algorithms
│   │   ├── matching/          # ICP, correlative matching
│   │   ├── mapping/           # Occupancy grid, ray tracing
│   │   └── localization/      # Particle filter (MCL)
│   │
│   ├── engine/                # SLAM orchestration
│   │   ├── slam/              # Online SLAM, keyframes, submaps
│   │   └── graph/             # Pose graph, loop closure
│   │
│   └── io/                    # I/O infrastructure
│       ├── sangam_client.rs   # SangamIO TCP client
│       ├── bag/               # Recording and playback
│       └── streaming/         # TCP publishing
│
├── tests/                     # Integration tests
│   ├── dead_reckoning.rs      # Odometry tests
│   └── sangam_integration.rs  # End-to-end tests
│
├── benches/                   # Performance benchmarks
│   └── milestone1.rs          # Core operations
│
├── Cargo.toml                 # Dependencies
└── dhruva-slam.toml           # Runtime configuration
```

## Data Flow

```
SangamIO (500Hz sensors, 5Hz lidar)
    │
    ▼
┌───────────────────────────────────────┐
│           SangamClient                │
│  • TCP connection to robot            │
│  • Message deserialization            │
└───────────────────┬───────────────────┘
                    │
        ┌───────────┴───────────┐
        ▼                       ▼
┌───────────────┐      ┌───────────────┐
│   Odometry    │      │    SLAM       │
│   Pipeline    │      │   Engine      │
│               │      │               │
│ • Wheel odom  │      │ • Preprocess  │
│ • Gyro fusion │─────►│ • Match scans │
│ • Decimation  │      │ • Update map  │
│               │      │ • Keyframes   │
└───────┬───────┘      └───────┬───────┘
        │                      │
        └──────────┬───────────┘
                   ▼
        ┌──────────────────┐
        │  TCP Publisher   │
        │  • Pose stream   │
        │  • Map updates   │
        │  • Diagnostics   │
        └──────────────────┘
                   │
                   ▼
          Visualization Client
              (Drishti)
```

## Testing

```bash
# Unit tests
cargo test

# Integration tests (requires bag file or live robot)
cargo test --test sangam_integration -- --ignored --nocapture

# Specific test
cargo test test_wheel_odometry
```

## Benchmarks

```bash
# Run all benchmarks
cargo bench

# View HTML reports
open target/criterion/report/index.html
```

Benchmarked operations:
- Math primitives (angle normalization, interpolation)
- Pose operations (compose, inverse, transform)
- Wheel odometry processing
- Complementary filter fusion
- Scan preprocessing pipeline
- ICP scan matching

## Dependencies

| Crate | Purpose |
|-------|---------|
| `serde` | Serialization framework |
| `serde_json` | JSON wire format |
| `postcard` | Binary wire format |
| `kiddo` | K-d tree for nearest neighbor |
| `thiserror` | Error handling |
| `log` / `env_logger` | Logging |
| `ctrlc` | Signal handling |
| `toml` | Configuration parsing |

## Design Notes

### Design Goals

| Goal | Description |
|------|-------------|
| **Modularity** | Each component (sensor fusion, scan matching, mapping, localization) independently replaceable |
| **Performance** | Real-time operation on embedded ARM (Allwinner A33 quad-core Cortex-A7) |
| **Testability** | Components unit-testable in isolation with mock/recorded data |
| **Portability** | No platform-specific dependencies in core SLAM logic |

### Platform Considerations (Allwinner A33)

**Target specs:** Quad-core ARM Cortex-A7 @ 1.2 GHz, 512MB RAM, NEON SIMD

**Algorithm Selection for A33:**

| Component | Recommended | Alternative | Avoid |
|-----------|-------------|-------------|-------|
| Sensor Fusion | ESKF | Complementary | UKF |
| Scan Matching | Multi-Res Correlative + P2L ICP | Correlative only | GICP, NDT |
| Map | Probabilistic Grid + Submaps | Binary Grid | SDF |
| Localization | MCL (300-500 particles) | Scan match only | KLD-MCL |
| Loop Closure | Distance + Scan Context | Distance only | Full BoW |
| Optimization | Gauss-Newton | - | Full LM every time |

**Performance Budget (10 Hz target):**

| Component | Budget | Notes |
|-----------|--------|-------|
| Preprocessing | 5 ms | Downsample to ~180 points |
| Odometry Fusion | 5 ms | ESKF predict + update |
| Scan Matching | 40 ms | Multi-resolution + ICP |
| Map Update | 15 ms | Log-odds with ray tracing |
| Loop Check | 10 ms | Background when possible |

**Memory Budget (256 MB for SLAM):**

| Component | Allocation |
|-----------|------------|
| Active Submaps (3) | 30 MB |
| Compressed Submaps | 50 MB |
| Pose Graph | 10 MB |
| Scan Buffers | 20 MB |
| Keyframes | 50 MB |
| Particle Filter | 20 MB |
| Working Memory | 76 MB |

### Algorithm Selection Guide

**By Environment Type:**

| Environment | Recommended Matcher | Notes |
|-------------|---------------------|-------|
| Structured (offices) | Point-to-Line ICP | Walls provide good line features |
| Cluttered (homes) | Multi-Res Correlative + ICP | Handles irregular obstacles |
| Large open spaces | Multi-Res Correlative | Needs global matching |
| Dynamic environments | Any + temporal filtering | Filter moving objects |

**By Accuracy Requirements:**

| Accuracy | Recommended Stack |
|----------|-------------------|
| Basic (±10cm) | Complementary + Correlative + MCL |
| Standard (±5cm) | ESKF + Multi-Res + ICP + MCL |
| High (±2cm) | ESKF + Preintegration + GICP + Graph opt |

### Future Extensibility

Not yet implemented but designed for:
- UKF sensor fusion (higher accuracy, more compute)
- NDT scan matching (alternative to ICP)
- IMU preintegration (tightly-coupled fusion)
- KLD-sampling MCL (adaptive particle count)
- Submap compression (memory optimization for long-term operation)

## License

See the root [LICENSE](../LICENSE) file.
