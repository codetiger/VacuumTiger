# Drishti - Robot Diagnostic Tool

Simple monitoring tool for testing sensor diagnosis and TCP protocol communication with SangamIO.

## Installation

```bash
cd drishti
pip install -r requirements.txt
```

## Usage

### GUI Application

```bash
python drishti_ui.py --robot 192.168.68.101
```

Options:
- `--robot` - Robot IP address (default: 192.168.68.101)
- `--port` - TCP port (default: 5555)
- `--verbose` - Enable debug logging

### Console Demo

```bash
python drishti.py --robot 192.168.68.101
```

## Features

- Full-screen robot diagram with sensor overlays
- Real-time telemetry monitoring (500Hz)
- JSON protocol over TCP
- Visual indicators for all 13 sensors

![Drishti UI](drishti.png)

## TCP Protocol

Connects to SangamIO on port 5555 for telemetry stream.

**Message format:** Length-prefixed JSON
```
[4-byte length][topic\0][JSON payload]
```

**Example sensor data:**
```json
{
  "SensorUpdate": {
    "battery_level": {"U8": 85},
    "encoder_left": {"I32": 12345},
    "bumper_left": {"Bool": false},
    "cliff_sensors": [{"U16": 100}, {"U16": 110}, {"U16": 105}, {"U16": 98}]
  }
}
```

## Troubleshooting

**No data received:**
- Verify SangamIO is running on robot
- Check network connectivity: `ping 192.168.68.101`
- Verify port 5555 is accessible

**Connection refused:**
- Robot may not be running SangamIO
- Check with: `ssh root@vacuum "ps aux | grep sangam"`
