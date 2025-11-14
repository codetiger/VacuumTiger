# Drishti - Real-time Robot Status Visualization System

**Drishti** is a Python client for monitoring and controlling VacuumTiger robots running the SangamIO firmware. It connects to the robot via TCP sockets to receive real-time telemetry and send commands.

## Features

- **Real-time Telemetry Monitoring**: Subscribe to sensor data (battery, encoders, IR sensors, bumper, etc.)
- **Connection Quality Metrics**: Monitor GD32 communication health
- **Lidar Data Streaming**: Receive point cloud scans
- **Robot Control**: Send motion and actuator commands
- **Lidar Control**: Enable/disable lidar with PWM speed control
- **Pure Python**: No Rust dependencies, runs on any platform

## Architecture

```
┌─────────────────────┐                    ┌─────────────────────┐
│   Drishti (Mac)     │                    │  SangamIO (Robot)   │
│                     │                    │                     │
│  ┌──────────────┐   │  TCP (5555)        │  ┌──────────────┐   │
│  │ Telemetry    │◄──┼────────────────────┼──┤ Publisher    │   │
│  │ Receiver     │   │  "telemetry"       │  │ (Telemetry + │   │
│  └──────────────┘   │  "lidar"           │  │  Lidar)      │   │
│                     │                    │  └──────────────┘   │
│  ┌──────────────┐   │  TCP (5556)        │  ┌──────────────┐   │
│  │ Command      │──►┼────────────────────┼──►  Command     │   │
│  │ Sender       │   │  "command"         │  │  Receiver    │   │
│  └──────────────┘   │                    │  └──────────────┘   │
└─────────────────────┘                    └─────────────────────┘
```

## Installation

### Prerequisites

- Python 3.8 or later
- Network access to the robot

### Install Dependencies

```bash
cd drishti
pip install -r requirements.txt
```

Or install manually:

```bash
pip install msgpack
```

## Usage

### Basic Demo: Side Brush Test

The default demo starts the side brush at 50% speed, monitors telemetry for 5 seconds, then stops:

```bash
python drishti.py
```

**With verbose logging:**

```bash
python drishti.py --verbose
```

**Custom robot hostname/IP:**

```bash
python drishti.py --robot 192.168.1.100
```

### Example Output

```
12:34:56 [INFO] Connecting to robot telemetry: vacuum:5555
12:34:56 [INFO] ✓ Connected to telemetry stream
12:34:56 [INFO] Connecting to robot command channel: vacuum:5556
12:34:56 [INFO] ✓ Connected to command channel
12:34:56 [INFO] ✓ Connected to robot
12:34:56 [INFO] ============================================================
12:34:56 [INFO] DEMO: Side Brush Test (5 seconds)
12:34:56 [INFO] ============================================================
12:34:56 [INFO] Starting side brush at 50% speed...
12:34:56 [INFO] Side brush speed set to 50%
12:34:56 [INFO] Monitoring telemetry (press Ctrl+C to stop early)...
12:34:56 [INFO]   Battery: 85%
12:34:57 [INFO]   Connection: 98.5% success (RX=1000, TX=1015)
12:34:58 [INFO]   Battery: 84%
12:35:01 [INFO] ✓ Received 247 telemetry messages in 5 seconds
12:35:01 [INFO] Stopping side brush...
12:35:01 [INFO] Side brush speed set to 0%
12:35:01 [INFO] ============================================================
12:35:01 [INFO] ✓ Demo complete
12:35:01 [INFO] ============================================================
12:35:01 [INFO] Disconnected from robot
```

## API Reference

### RobotClient

Main client class for robot communication.

#### Initialization

```python
from drishti import RobotClient

client = RobotClient(robot_ip="vacuum", pub_port=5555, cmd_port=5556)
```

**Parameters:**
- `robot_ip` (str): Robot hostname or IP address (default: "vacuum")
- `pub_port` (int): Robot's telemetry TCP port (default: 5555)
- `cmd_port` (int): Robot's command TCP port (default: 5556)

#### Receiving Telemetry

```python
result = client.receive_telemetry(timeout_ms=1000)
if result:
    topic, message = result

    if topic == "telemetry":
        if "SensorUpdate" in message:
            sensor_data = message["SensorUpdate"]
            print(f"Battery: {sensor_data['battery_level']}%")
            print(f"Encoders: L={sensor_data['encoder_left']}, R={sensor_data['encoder_right']}")

        elif "ConnectionQuality" in message:
            quality = message["ConnectionQuality"]
            print(f"Success rate: {quality['success_rate']*100:.1f}%")

    elif topic == "lidar":
        scan = message
        print(f"Lidar scan: {len(scan['points'])} points")
```

#### Sending Commands

**Set wheel velocities:**
```python
client.set_wheel_velocity(left_ticks=1000.0, right_ticks=1000.0)
```

**Control actuators:**
```python
client.set_side_brush_speed(50)      # 0-100%
client.set_main_brush_speed(75)      # 0-100%
client.set_air_pump_speed(100)       # 0-100%
```

**Control lidar:**
```python
client.enable_lidar(pwm=80)          # Enable at 80% PWM
client.set_lidar_pwm(pwm=60)         # Adjust to 60% PWM
client.disable_lidar()               # Power off
```

**Emergency stop:**
```python
client.emergency_stop()
```

**Close connection:**
```python
client.close()
```

## Message Formats

### TCP Protocol

All messages use length-prefixed framing:
```
[4-byte length (big-endian)][topic\0][MessagePack payload]
```

### Telemetry Messages

#### SensorUpdate (Published at ~500 Hz)

```python
{
    "SensorUpdate": {
        "timestamp": 1234567890000,      # Microseconds since epoch
        "battery_level": 85,              # 0-100%
        "is_charging": False,             # Charging flag
        "ir_sensor_1": 100,               # Front IR sensor (ADC value)
        "start_button_ir": 50,            # Start button IR (ADC value)
        "dock_button_ir": 60,             # Dock button IR (ADC value)
        "bumper_pressed": False,          # Bumper state
        "cliff_sensors": [10, 20, 30, 40],# Cliff sensor ADC values (×4)
        "error_code": 0,                  # GD32 error code
        "encoder_left": 12345,            # Left wheel encoder ticks (cumulative)
        "encoder_right": 12340            # Right wheel encoder ticks (cumulative)
    }
}
```

#### ConnectionQuality (Published at ~1 Hz)

```python
{
    "ConnectionQuality": {
        "timestamp": 1234567890000,
        "rx_packets": 1000,
        "tx_packets": 1015,
        "success_rate": 0.985,
        "telemetry_fresh": True
    }
}
```

#### LidarScan (Published at ~5 Hz)

```python
{
    "timestamp": 1234567890000,
    "scan_number": 42,
    "points": [
        {
            "angle": 0.0,        # Radians (0 to 2π)
            "distance": 1.5,     # Meters
            "quality": 200       # Signal strength (0-255)
        },
        # ... more points
    ]
}
```

### Command Messages

All commands are sent on the "command" topic and serialized with MessagePack.

#### SetWheelVelocity

```python
{
    "SetWheelVelocity": {
        "left": 1000.0,   # ticks/sec (positive = forward)
        "right": 1000.0   # ticks/sec (positive = forward)
    }
}
```

#### SetSideBrushSpeed

```python
{
    "SetSideBrushSpeed": {
        "speed": 50  # 0-100%
    }
}
```

#### SetMainBrushSpeed

```python
{
    "SetMainBrushSpeed": {
        "speed": 75  # 0-100%
    }
}
```

#### SetAirPumpSpeed

```python
{
    "SetAirPumpSpeed": {
        "speed": 100  # 0-100%
    }
}
```

#### EnableLidar

```python
{
    "EnableLidar": {
        "pwm": 80  # 0-100% PWM duty cycle
    }
}
```

**Note:** This performs full initialization (prep + power + PWM) via GD32.

#### DisableLidar

```python
"DisableLidar"
```

#### SetLidarPWM

```python
{
    "SetLidarPWM": {
        "pwm": 60  # 0-100% (lidar must already be enabled)
    }
}
```

#### EmergencyStopAll

```python
"EmergencyStopAll"
```

Stops all motors including lidar.

## Robot Configuration

The robot must be running SangamIO with TCP streaming enabled. Default configuration (`sangamio.toml`):

```toml
[streaming]
tcp_pub_address = "0.0.0.0:5555"  # Telemetry and lidar
tcp_cmd_address = "0.0.0.0:5556"  # Commands
```

## Network Setup

Ensure your Mac can reach the robot on the network:

```bash
# Test connectivity
ping vacuum

# Or use IP address
ping 192.168.1.100
```

If using hostname "vacuum", add to `/etc/hosts`:

```
192.168.1.100  vacuum
```

## Troubleshooting

### Connection Timeout

**Symptom:** `receive_telemetry()` always returns `None`

**Causes:**
1. Robot is not running SangamIO
2. Network connectivity issues
3. Firewall blocking TCP ports

**Solutions:**
```bash
# Check robot is running
ssh root@vacuum "ps aux | grep sangam"

# Check ports are listening
ssh root@vacuum "netstat -tuln | grep 5555"

# Check firewall (if applicable)
ssh root@vacuum "iptables -L"
```

### MessagePack Deserialization Errors

**Symptom:** `Error receiving telemetry: unpack failed`

**Cause:** Version mismatch between Rust `rmp-serde` and Python `msgpack`

**Solution:** Ensure both sides use compatible MessagePack versions (current: Rust rmp-serde 1.1, Python msgpack 1.0+)

### Command Not Executing

**Symptom:** Commands sent but robot doesn't respond

**Causes:**
1. Robot's command receiver not running
2. Wrong endpoint/port
3. Command format incorrect

**Solutions:**
```bash
# Check robot logs
ssh root@vacuum "journalctl -u sangamio -f"

# Verify endpoints match
# Robot config: 0.0.0.0:5556
# Drishti: vacuum:5556
```

## Development

### Custom Control Script

```python
#!/usr/bin/env python3
from drishti import RobotClient
import time

client = RobotClient(robot_ip="vacuum")

try:
    # Enable lidar
    client.enable_lidar(pwm=80)

    # Start vacuum cleaning
    client.set_main_brush_speed(100)
    client.set_side_brush_speed(75)
    client.set_air_pump_speed(100)

    # Move forward
    client.set_wheel_velocity(1000.0, 1000.0)

    # Run for 10 seconds
    time.sleep(10)

    # Stop
    client.set_wheel_velocity(0.0, 0.0)
    client.set_main_brush_speed(0)
    client.set_side_brush_speed(0)
    client.set_air_pump_speed(0)
    client.disable_lidar()

finally:
    client.close()
```

### Monitoring Loop

```python
from drishti import RobotClient
import time

client = RobotClient(robot_ip="vacuum")

try:
    while True:
        result = client.receive_telemetry(timeout_ms=100)

        if result:
            topic, message = result

            if topic == "telemetry" and "SensorUpdate" in message:
                data = message["SensorUpdate"]

                # Check battery
                if data["battery_level"] and data["battery_level"] < 20:
                    print("⚠️  Low battery!")

                # Check bumper
                if data["bumper_pressed"]:
                    print("⚠️  Bumper pressed - reversing")
                    client.set_wheel_velocity(-500.0, -500.0)
                    time.sleep(1)
                    client.set_wheel_velocity(0.0, 0.0)

finally:
    client.close()
```

## License

Same as VacuumTiger/SangamIO project.

## Support

For issues, see the main VacuumTiger repository.
