# SangamIO TCP Protocol Specification

## Overview

SangamIO uses a TCP-based streaming protocol with MessagePack serialization. The daemon acts as a server, accepting connections on port 5000 (configurable).

## Message Framing

All messages use a simple length-prefixed format:

```
+------------------+-------------+------+-----------------------+
| Length (4 bytes) | Topic (str) | NULL | Payload (MessagePack) |
+------------------+-------------+------+-----------------------+
```

- **Length**: 32-bit big-endian integer (includes topic string + null byte + payload)
- **Topic**: Null-terminated string identifying message type (e.g., "telemetry", "lidar", "command")
- **NULL**: Single null byte (0x00) marking end of topic string
- **Payload**: MessagePack-encoded data structure

## Message Topics

### Outbound (Daemon → Client)

#### Topic: "telemetry" (SensorUpdate)
Sent at 20Hz with current sensor readings.

```rust
{
    "timestamp_ms": u64,
    "battery_level": u8,           // 0-100%
    "battery_voltage": f32,         // Volts
    "battery_current": f32,         // Amps
    "battery_temperature": f32,     // Celsius
    "bumper_left": bool,
    "bumper_right": bool,
    "cliff_left": bool,
    "cliff_front_left": bool,
    "cliff_front_right": bool,
    "cliff_right": bool,
    "wheel_drop_left": bool,
    "wheel_drop_right": bool,
    "button_power": bool,
    "button_home": bool,
    "odometry": {
        "x": f64,                   // meters
        "y": f64,                   // meters
        "theta": f64                // radians
    },
    "velocity": {
        "linear": f32,              // m/s
        "angular": f32              // rad/s
    }
}
```

#### Topic: "lidar" (LidarScan)
Sent at 5Hz with complete 360° scan data.

```rust
{
    "timestamp_ms": u64,
    "scan_time_ms": u32,           // Time to complete scan
    "points": [
        {
            "angle": f32,           // radians (0 to 2π)
            "distance": f32,        // meters
            "quality": u8           // 0-255
        },
        // ... ~360 points
    ]
}
```

#### Topic: "quality" (ConnectionQuality) - TODO
*Note: Not yet implemented. Reserved for future use.*

Planned to send every 5 seconds with link statistics:
```rust
{
    "clients_connected": u32,
    "messages_sent": u64,
    "bytes_sent": u64,
    "messages_dropped": u32,       // Queue overflow
    "uptime_seconds": u64,
    "control_frequency_hz": f32,
    "heartbeat_latency_ms": f32
}
```

#### Topic: "error" (ErrorReport) - TODO
*Note: Not yet implemented. Reserved for future use.*

Planned to send on hardware errors or warnings:
```rust
{
    "timestamp_ms": u64,
    "severity": "warning" | "error" | "critical",
    "source": "gd32" | "lidar" | "system",
    "code": u32,
    "message": String
}
```

### Inbound (Client → Daemon)

#### Topic: "command" (RobotCommand)
Commands are MessagePack-encoded with a type discriminator:

```rust
// SetVelocity
{
    "type": "SetVelocity",
    "linear": f32,                 // m/s (-0.5 to 0.5)
    "angular": f32                 // rad/s (-2.0 to 2.0)
}

// Stop (Emergency stop)
{
    "type": "Stop"
}

// MoveDistance
{
    "type": "MoveDistance",
    "distance": f32,               // meters
    "max_speed": f32               // m/s (optional)
}

// Rotate
{
    "type": "Rotate",
    "angle": f32,                  // radians
    "max_speed": f32               // rad/s (optional)
}

// SetLidarPower
{
    "type": "SetLidarPower",
    "enabled": bool
}

// ResetOdometry
{
    "type": "ResetOdometry",
    "x": f32,                      // meters (optional, default 0)
    "y": f32,                      // meters (optional, default 0)
    "theta": f32                   // radians (optional, default 0)
}
```

## Client Implementation Examples

### Python Client

```python
import socket
import struct
import msgpack
import threading

class SangamClient:
    def __init__(self, host='localhost', port=5000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.running = True

        # Start receiver thread
        self.receiver_thread = threading.Thread(target=self._receive_loop)
        self.receiver_thread.start()

    def _receive_loop(self):
        while self.running:
            # Read length prefix
            length_bytes = self._recv_exactly(4)
            if not length_bytes:
                break
            length = struct.unpack('>I', length_bytes)[0]

            # Read frame (topic + null + payload)
            frame = self._recv_exactly(length)
            if not frame:
                break

            # Parse topic and payload
            null_idx = frame.index(0)
            topic = frame[:null_idx].decode('utf-8')
            payload = frame[null_idx + 1:]
            data = msgpack.unpackb(payload)

            # Dispatch
            if topic == "telemetry":
                self.on_sensor_update(data)
            elif topic == "lidar":
                self.on_lidar_scan(data)

    def _recv_exactly(self, n):
        data = b''
        while len(data) < n:
            chunk = self.sock.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def send_command(self, command):
        payload = msgpack.packb(command)
        frame = b'command\x00' + payload
        header = struct.pack('>I', len(frame))
        self.sock.send(header + frame)

    def set_velocity(self, linear, angular):
        self.send_command({
            'type': 'SetVelocity',
            'linear': linear,
            'angular': angular
        })

    def on_sensor_update(self, data):
        # Override in subclass
        pass

    def on_lidar_scan(self, data):
        # Override in subclass
        pass
```

### Rust Client

```rust
use std::net::TcpStream;
use std::io::{Read, Write};
use rmp_serde::{Deserializer, Serializer};
use serde::{Deserialize, Serialize};

#[derive(Serialize)]
#[serde(tag = "type")]
enum RobotCommand {
    SetVelocity { linear: f32, angular: f32 },
    Stop,
    MoveDistance { distance: f32, max_speed: Option<f32> },
}

#[derive(Deserialize)]
struct SensorUpdate {
    timestamp_ms: u64,
    battery_level: u8,
    odometry: Odometry,
    // ... other fields
}

struct SangamClient {
    stream: TcpStream,
}

impl SangamClient {
    fn connect(addr: &str) -> std::io::Result<Self> {
        let stream = TcpStream::connect(addr)?;
        Ok(Self { stream })
    }

    fn send_command(&mut self, cmd: RobotCommand) -> std::io::Result<()> {
        let mut payload = Vec::new();
        cmd.serialize(&mut Serializer::new(&mut payload)).unwrap();

        // Build frame: "command" + null + payload
        let mut frame = Vec::new();
        frame.extend_from_slice(b"command\x00");
        frame.extend_from_slice(&payload);

        // Write length prefix and frame
        let length = frame.len() as u32;
        self.stream.write_all(&length.to_be_bytes())?;
        self.stream.write_all(&frame)?;
        Ok(())
    }

    fn read_message(&mut self) -> std::io::Result<(String, Vec<u8>)> {
        // Read length prefix
        let mut length_bytes = [0u8; 4];
        self.stream.read_exact(&mut length_bytes)?;
        let length = u32::from_be_bytes(length_bytes);

        // Read frame
        let mut frame = vec![0u8; length as usize];
        self.stream.read_exact(&mut frame)?;

        // Parse topic and payload
        let null_pos = frame.iter().position(|&b| b == 0).unwrap();
        let topic = String::from_utf8(frame[..null_pos].to_vec()).unwrap();
        let payload = frame[null_pos + 1..].to_vec();

        Ok((topic, payload))
    }
}
```

## Connection Management

- **TCP Keep-Alive**: Enabled with 30-second interval
- **Client Timeout**: Disconnects after 60 seconds of no commands
- **Reconnection**: Clients should implement exponential backoff
- **Multiple Clients**: Supported (all receive same telemetry)

## Error Handling

- **Invalid Commands**: Logged but connection maintained
- **Deserialization Errors**: Connection closed
- **Queue Overflow**: Old messages dropped, counter incremented
- **Hardware Errors**: Reported via ErrorReport messages

## Performance Considerations

- **Message Rates**:
  - SensorUpdate: 20Hz (50ms)
  - LidarScan: 5Hz (200ms)
  - Commands: Process immediately (< 25ms latency)
- **Buffer Sizes**:
  - Receive: 64KB
  - Send: 256KB (for lidar data)
- **Queue Depth**: 100 messages per topic

## Version Compatibility

Protocol version is not explicitly tracked. Clients should handle:
- Unknown message topics (ignore)
- Missing fields (use defaults)
- Extra fields (ignore)

This allows forward and backward compatibility within reason.