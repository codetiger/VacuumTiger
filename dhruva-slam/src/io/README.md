# I/O Module

External communication and data persistence infrastructure.

## Overview

The I/O module provides:
- **SangamClient** - TCP client for SangamIO daemon
- **Bag** - Recording and playback of sensor data
- **Streaming** - TCP publishing for visualization clients

## SangamClient

TCP client that connects to the SangamIO daemon running on the robot.

### Connection

```rust
use dhruva_slam::io::sangam_client::SangamClient;

// Connect to SangamIO (Protobuf wire format)
let mut client = SangamClient::connect("192.168.68.101:5555")?;

// Set read timeout
client.set_timeout(Some(Duration::from_millis(500)));
```

### Receiving Messages

```rust
use dhruva_slam::io::sangam_client::Message;

loop {
    match client.recv() {
        Ok(msg) => {
            match msg.topic.as_str() {
                "sensors/sensor_status" => {
                    // Extract sensor values
                    let values = &msg.payload.values;
                    let left_ticks = values.get("wheel_left")
                        .and_then(|v| v.as_u16());
                }
                "sensors/lidar" => {
                    // Extract lidar scan
                    let scan = extract_lidar_scan(&msg.payload);
                }
                _ => {}
            }
        }
        Err(ClientError::Timeout) => continue,
        Err(e) => break,
    }
}
```

### Message Structure

```rust
pub struct Message {
    pub topic: String,       // e.g., "sensors/sensor_status"
    pub payload: Payload,
}

pub struct Payload {
    pub group_id: String,
    pub timestamp_us: u64,
    pub values: HashMap<String, SensorValue>,
}
```

### Protocol

Length-prefixed Protobuf framing over TCP:

```
┌──────────────────┬─────────────────────┐
│ Length (4 bytes) │ Protobuf Payload    │
│ Big-endian u32   │ (binary)            │
└──────────────────┴─────────────────────┘
```

## Bag Recording and Playback

Record sensor data from the robot for offline testing and development.

### File Format

```
┌─────────────────────────────────────────────────────────────┐
│ Header (64 bytes)                                           │
│  • Magic: "DBAG"                                            │
│  • Version: 1                                               │
│  • Flags                                                    │
│  • Start timestamp (μs)                                     │
│  • End timestamp (μs)                                       │
│  • Message count                                            │
│  • Index offset                                             │
│  • Reserved                                                 │
├─────────────────────────────────────────────────────────────┤
│ Messages                                                    │
│  ┌──────────────┬─────────────────────────────────────────┐ │
│  │ Length (4B)  │ Postcard-encoded BagMessage             │ │
│  └──────────────┴─────────────────────────────────────────┘ │
│  ...                                                        │
└─────────────────────────────────────────────────────────────┘
```

### BagRecorder

Record live sensor data to file.

```rust
use dhruva_slam::io::bag::BagRecorder;

// Record for specific duration
let recorder = BagRecorder::connect(
    "192.168.68.101:5555",
    "recording.bag",
)?;

let info = recorder.record_duration(Duration::from_secs(60));
println!("Recorded {} messages", info.message_count);

// Or record until stopped
let recorder = BagRecorder::connect(address, path)?;

// In signal handler or another thread:
recorder.stop();
```

### BagPlayer

Read recorded bag files.

```rust
use dhruva_slam::io::bag::BagPlayer;

let player = BagPlayer::open("recording.bag")?;

// Get metadata
let info = player.info();
println!("Duration: {}s", info.duration_secs());
println!("Messages: {}", info.message_count);

// Iterate messages
for msg in player.messages() {
    let msg = msg?;
    println!("[{}] {}", msg.timestamp_us, msg.topic);
}
```

### SimulatedClient

Drop-in replacement for SangamClient that reads from bag file.

```rust
use dhruva_slam::io::bag::SimulatedClient;

// Same API as SangamClient
let mut client = SimulatedClient::open("recording.bag")?;

loop {
    match client.recv() {
        Ok(msg) => process_message(&msg),
        Err(ClientError::EndOfFile) => break,
        Err(e) => return Err(e),
    }
}
```

### Bag Tools

**Recording:**
```bash
cargo run --bin bag-record -- \
  --sangam 192.168.68.101:5555 \
  --output recording.bag \
  --duration 60
```

**Inspection:**
```bash
cargo run --bin bag-info -- recording.bag
cargo run --bin bag-info -- --verbose --count recording.bag
```

## Streaming

TCP publishing for visualization clients (Drishti).

### OdometryPublisher

TCP server that broadcasts SLAM data to connected clients.

```rust
use dhruva_slam::io::streaming::OdometryPublisher;

let publisher = OdometryPublisher::new("0.0.0.0:5557")?;

// In main loop
publisher.publish_pose(&pose, timestamp_us);
publisher.publish_slam_status(&status);
publisher.publish_slam_map(&map);
publisher.publish_slam_diagnostics(&diagnostics);
```

### Published Message Types

**OdometryMessage:**
```rust
pub struct OdometryMessage {
    pub timestamp_us: u64,
    pub pose: Pose2D,
    pub velocity: Option<Twist2D>,
    pub covariance: Option<Covariance2D>,
}
```

**SlamStatusMessage:**
```rust
pub struct SlamStatusMessage {
    pub timestamp_us: u64,
    pub mode: String,
    pub num_scans: u64,
    pub num_keyframes: usize,
    pub num_submaps: usize,
    pub last_match_score: f32,
    pub is_lost: bool,
}
```

**SlamMapMessage:**
```rust
pub struct SlamMapMessage {
    pub timestamp_us: u64,
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub data: String,  // Base64-encoded occupancy data
}
```

**SlamScanMessage:**
```rust
pub struct SlamScanMessage {
    pub timestamp_us: u64,
    pub pose: Pose2D,
    pub points: Vec<(f32, f32)>,  // (x, y) in robot frame
}
```

**SlamDiagnosticsMessage:**
```rust
pub struct SlamDiagnosticsMessage {
    pub timestamp_us: u64,
    pub timing_breakdown: TimingBreakdown,
    pub scan_match_stats: ScanMatchStats,
    pub mapping_stats: MappingStats,
    pub loop_closure_stats: LoopClosureStats,
}
```

### OdometryPipeline

Fuses raw sensor data into odometry estimates.

```rust
use dhruva_slam::io::streaming::OdometryPipeline;

let config = OdometryPipelineConfig {
    ticks_per_meter: 4464.0,
    wheel_base: 0.233,
    alpha: 0.8,
    gyro_scale: 0.0001745,
    output_rate_hz: 50.0,
    gyro_calibration_samples: 1500,
};

let mut pipeline = OdometryPipeline::new(config);

// Process each sensor update (110Hz)
if let Some(pose) = pipeline.process(left_ticks, right_ticks, gyro_z, timestamp_us) {
    // Decimated output at configured rate
    publish_pose(&pose, timestamp_us);
}

// Get diagnostics
if let Some(diag) = pipeline.diagnostics(timestamp_us) {
    publish_diagnostics(&diag);
}

// Current pose estimate
let current = pipeline.pose();
```

**Features:**
- Auto-calibrates gyro bias on startup (1500 samples)
- Decimates 110Hz input to configurable output rate
- Provides timing and quality diagnostics

## File Structure

```
io/
├── mod.rs              # Module exports
├── sangam_client.rs    # SangamIO TCP client
├── bag/
│   ├── mod.rs          # Bag exports
│   ├── format.rs       # File format constants
│   ├── recorder.rs     # BagRecorder
│   ├── player.rs       # BagPlayer
│   └── simulated.rs    # SimulatedClient
└── streaming/
    ├── mod.rs          # Streaming exports
    ├── publisher.rs    # OdometryPublisher
    ├── pipeline.rs     # OdometryPipeline
    └── messages.rs     # Message types
```

## Usage Examples

### Complete Recording Session

```rust
use dhruva_slam::io::bag::BagRecorder;

fn record_session(address: &str, output: &str, duration_secs: u64) -> Result<()> {
    println!("Recording from {} to {}", address, output);

    let recorder = BagRecorder::connect(address, output)?;
    let info = recorder.record_duration(Duration::from_secs(duration_secs));

    println!("Recording complete:");
    println!("  Messages: {}", info.message_count);
    println!("  Duration: {:.1}s", info.duration_secs());
    println!("  Size: {} bytes", info.file_size);

    Ok(())
}
```

### Offline Processing

```rust
use dhruva_slam::io::bag::SimulatedClient;
use dhruva_slam::io::streaming::OdometryPipeline;

fn process_bag(bag_path: &str) -> Result<()> {
    let mut client = SimulatedClient::open(bag_path)?;
    let mut pipeline = OdometryPipeline::new(config);

    let mut poses = Vec::new();

    loop {
        match client.recv() {
            Ok(msg) => {
                if msg.topic == "sensors/sensor_status" {
                    let (left, right, gyro) = extract_sensor_data(&msg);
                    if let Some(pose) = pipeline.process(left, right, gyro, msg.timestamp_us) {
                        poses.push(pose);
                    }
                }
            }
            Err(ClientError::EndOfFile) => break,
            Err(e) => return Err(e.into()),
        }
    }

    println!("Processed {} poses", poses.len());
    Ok(())
}
```

### Live SLAM with Publishing

```rust
use dhruva_slam::io::sangam_client::SangamClient;
use dhruva_slam::io::streaming::{OdometryPublisher, OdometryPipeline};
use dhruva_slam::engine::slam::OnlineSlam;

fn run_slam_node(sangam_addr: &str, publish_port: &str) -> Result<()> {
    let mut client = SangamClient::connect(sangam_addr)?;
    let publisher = OdometryPublisher::new(publish_port)?;
    let mut pipeline = OdometryPipeline::new(odom_config);
    let mut slam = OnlineSlam::new(slam_config);

    loop {
        let msg = client.recv()?;

        match msg.topic.as_str() {
            "sensors/sensor_status" => {
                let (left, right, gyro) = extract_sensors(&msg);
                if let Some(pose) = pipeline.process(left, right, gyro, msg.timestamp_us) {
                    publisher.publish_pose(&pose, msg.timestamp_us);
                }
            }
            "sensors/lidar" => {
                let scan = extract_lidar(&msg);
                let odom_delta = pipeline.delta_since_last_scan();
                let result = slam.process_scan(&scan, &odom_delta, msg.timestamp_us);

                publisher.publish_slam_scan(&scan, &result.pose, msg.timestamp_us);

                if result.keyframe_created {
                    publisher.publish_slam_status(&slam.status());
                    publisher.publish_slam_map(slam.map());
                }
            }
            _ => {}
        }
    }
}
```

## Performance

| Operation | Time | Notes |
|-----------|------|-------|
| SangamClient.recv | <1ms | Network-bound |
| BagPlayer iteration | ~10μs | Per message |
| OdometryPipeline.process | <10μs | Per sample |
| Publisher.publish_pose | <100μs | Per client |
| Publisher.publish_map | ~5ms | Depends on map size |
