# I/O Module

External communication and data persistence infrastructure.

## Overview

The I/O module provides:
- **SangamClient** - TCP client for the SangamIO daemon
- **Streaming** - TCP publishing for visualization clients

## Module Structure

```
io/
├── mod.rs              # Module exports
├── sangam_client.rs    # SangamIO TCP client
├── sangam_udp_receiver.rs  # UDP sensor data receiver
├── map_manager.rs      # Map persistence
├── motion_controller.rs # Velocity commands
└── streaming/
    ├── mod.rs          # Streaming exports
    ├── publisher.rs    # OdometryPublisher
    ├── pipeline.rs     # OdometryPipeline
    └── messages.rs     # Message types
```

## SangamClient

TCP client that connects to the SangamIO daemon running on the robot.

### Connection

```rust
use crate::io::sangam_client::{SangamClient, ClientError};

// Connect to SangamIO (Protobuf wire format)
let mut client = SangamClient::connect("192.168.68.101:5555")?;

// Set read timeout
client.set_timeout(Some(Duration::from_millis(500)));
```

### Receiving Messages

```rust
use crate::io::sangam_client::Message;

loop {
    match client.recv() {
        Ok(msg) => {
            match msg.topic.as_str() {
                "sensors/sensor_status" => {
                    // Sensor data at 110Hz
                    let values = &msg.payload.values;
                    let left_ticks = values.get("wheel_left")
                        .and_then(|v| v.as_u16());
                    let gyro_z = values.get("gyro_z")
                        .and_then(|v| v.as_i16());
                }
                "sensors/lidar" => {
                    // Lidar scan at 5Hz
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

### LidarScan Type

```rust
pub struct LidarScan {
    pub timestamp_us: u64,
    pub points: Vec<LidarPoint>,
}

pub struct LidarPoint {
    pub angle_deg: f32,     // 0-360 degrees
    pub distance_mm: u16,   // Distance in mm
    pub quality: u8,        // Signal quality
}
```

### Wire Protocol

Length-prefixed Protobuf framing over TCP:

```
┌──────────────────┬─────────────────────┐
│ Length (4 bytes) │ Protobuf Payload    │
│ Big-endian u32   │ (binary)            │
└──────────────────┴─────────────────────┘
```

## Streaming

TCP publishing for visualization clients (Drishti).

### OdometryPublisher

TCP server that broadcasts SLAM data to connected clients:

```rust
use crate::io::streaming::OdometryPublisher;

let publisher = OdometryPublisher::new("0.0.0.0:5557")?;

// In main loop
publisher.publish_pose(&pose, timestamp_us);
publisher.publish_map(&map, timestamp_us);
publisher.publish_scan(&scan, &pose, timestamp_us);
publisher.publish_features(&features, timestamp_us);
```

### Published Message Types

**OdometryMessage** (50Hz):
```rust
pub struct OdometryMessage {
    pub timestamp_us: u64,
    pub pose: Pose2D,           // Robot pose
    pub velocity: Option<Twist2D>,
    pub covariance: Option<Covariance2D>,
}
```

**SlamMapMessage** (1Hz):
```rust
pub struct SlamMapMessage {
    pub timestamp_us: u64,
    pub resolution: f32,        // Meters per cell
    pub width: u32,
    pub height: u32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub data: String,           // Base64-encoded occupancy
}
```

**SlamScanMessage** (5Hz):
```rust
pub struct SlamScanMessage {
    pub timestamp_us: u64,
    pub pose: Pose2D,           // Scan pose
    pub points: Vec<(f32, f32)>, // (x, y) in robot frame
}
```

**FeatureMessage** (0.2Hz):
```rust
pub struct FeatureMessage {
    pub timestamp_us: u64,
    pub lines: Vec<LineSegment>,
    pub corners: Vec<Point2D>,
}
```

### OdometryPipeline

Fuses raw sensor data into odometry estimates with rate limiting:

```rust
use crate::io::streaming::{OdometryPipeline, OdometryPipelineConfig};

let config = OdometryPipelineConfig {
    ticks_per_meter: 4464.0,
    wheel_base: 0.233,
    output_rate_hz: 50.0,
    gyro_calibration_samples: 1500,
};

let mut pipeline = OdometryPipeline::new(config);

// Process each sensor update (110Hz input)
if let Some(pose) = pipeline.process(left_ticks, right_ticks, gyro_z, timestamp_us) {
    // Decimated output at configured rate (50Hz)
    publisher.publish_pose(&pose, timestamp_us);
}

// Current pose (always available)
let current = pipeline.pose();
```

**Features:**
- Auto-calibrates gyro bias on startup (1500 samples)
- Decimates 110Hz input to configurable output rate
- Tracks pose delta for SLAM integration

## Usage Examples

### Live SLAM with Publishing

```rust
use crate::io::sangam_client::SangamClient;
use crate::io::streaming::OdometryPublisher;

fn run_slam_node(sangam_addr: &str, publish_port: &str) -> Result<()> {
    let mut client = SangamClient::connect(sangam_addr)?;
    let publisher = OdometryPublisher::new(publish_port)?;
    let mut odom_pipeline = OdometryPipeline::new(odom_config);
    let mut slam = OnlineSlam::new(slam_config);

    loop {
        let msg = client.recv()?;

        match msg.topic.as_str() {
            "sensors/sensor_status" => {
                let (left, right, gyro) = extract_sensors(&msg);
                if let Some(pose) = odom_pipeline.process(left, right, gyro, msg.timestamp_us) {
                    publisher.publish_pose(&pose, msg.timestamp_us);
                }
            }
            "sensors/lidar" => {
                let scan = preprocess(&extract_lidar(&msg));
                let odom_delta = odom_pipeline.delta_since_last_scan();
                let result = slam.process_scan(&scan, &odom_delta, msg.timestamp_us);

                publisher.publish_scan(&scan, &result.pose, msg.timestamp_us);

                if result.keyframe_created {
                    publisher.publish_map(slam.map(), msg.timestamp_us);
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
| OdometryPipeline.process | <10μs | Per sample |
| Publisher.publish_pose | <100μs | Per client |
| Publisher.publish_map | ~5ms | Depends on map size |
