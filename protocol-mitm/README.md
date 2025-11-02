# Protocol MITM - GD32 Protocol Reverse Engineering Tools

Man-in-the-Middle (MITM) toolset for reverse-engineering proprietary serial protocols on embedded devices. This project was created to intercept and analyze the communication between the AuxCtrl firmware and the GD32F103 motor controller in CRL-200S vacuum robots.

## Overview

This toolset enables transparent serial port interception using PTY-based forwarding, allowing capture of bidirectional communication without modifying the original firmware or hardware. The captured protocol was successfully reverse-engineered and implemented in the [SangamIO library](../sangam-io/).

### Key Features

- **Transparent PTY-based Serial Forwarding** - Creates virtual serial ports that applications connect to normally
- **Bidirectional Packet Logging** - Captures TX/RX with microsecond-precision timestamps
- **GPIO State Monitoring** - Tracks hardware control lines in parallel with serial data
- **Protocol-Aware Decoding** - Automatically identifies and annotates known GD32 commands
- **Run Counter Integration** - Supports multi-session capture campaigns
- **Graceful Shutdown** - Clean signal handling with statistics reporting

## Project Structure

```
protocol-mitm/
├── src/
│   └── main.rs           # Serial MITM logger binary
├── scripts/              # Robot-side automation scripts
│   ├── mitm_boot_setup.sh    # Initial MITM configuration
│   ├── mitm_cleanup.sh       # Remove MITM setup
│   ├── mitm_disable.sh       # Temporarily disable MITM
│   ├── mitm_enable.sh        # Re-enable MITM
│   ├── mitm_init.sh          # Initialize MITM for new session
│   └── mitm_stop.sh          # Stop MITM cleanly
├── tools/                # Development machine tools
│   ├── mitm_deploy_all.sh    # Deploy MITM to robot
│   ├── mitm_disable.sh       # Remote disable
│   ├── mitm_enable.sh        # Remote enable
│   ├── mitm_finish_run.sh    # Complete capture session
│   ├── mitm_reboot_robot.sh  # Remote reboot
│   ├── mitm_retrieve_logs.sh # Download logs
│   └── README.md
├── logs/                 # Captured protocol logs
└── docs/                 # Documentation
    ├── MITM_LOGGING_GUIDE.md
    └── LIDAR_INITIALIZATION_SEQUENCE.md
```

## Quick Start

### 1. Build for Target Platform

```bash
cargo build --release --target armv7-unknown-linux-musleabihf
```

### 2. Deploy to Robot

```bash
cd tools
./mitm_deploy_all.sh
```

This will:
- Build the MITM binary
- Copy it to the robot
- Deploy all automation scripts
- Initialize the MITM setup

### 3. Enable MITM

```bash
./mitm_enable.sh
```

The robot will reboot with MITM active. AuxCtrl will transparently communicate through the MITM logger.

### 4. Capture Protocol Data

Perform robot operations (navigation, cleaning, return to dock) while MITM logs all serial traffic to `/tmp/mitm_capture_run<N>_<timestamp>.log`.

### 5. Retrieve Logs

```bash
./mitm_retrieve_logs.sh
```

Logs will be downloaded to `logs/mitm_captures_<timestamp>/` on your development machine.

### 6. Finish Session

```bash
./mitm_finish_run.sh
```

This disables MITM, restores normal operation, and optionally retrieves logs.

## How It Works

### PTY-Based Forwarding

1. **Move Real Serial Port**: `/dev/ttyS3` → `/dev/ttyS3_hardware`
2. **Create Virtual Port**: MITM creates PTY at `/tmp/ttyS3_tap`
3. **Symlink**: `/dev/ttyS3` → `/tmp/ttyS3_tap`
4. **Proxy Loop**: Bidirectional forwarding with logging

```
AuxCtrl → /dev/ttyS3 (symlink) → /tmp/ttyS3_tap (PTY slave)
                                          ↓
                                    MITM Process
                                          ↓
                              /dev/ttyS3_hardware → GD32
```

### Log Format

```
[timestamp] TX <bytes> bytes
  HEX: FA FB 07 A2 10 0E 00 00 B0 10
  PKT: CMD=0xA2 LEN=7
       (LIDAR_PREP - Preparation command)

[timestamp] GPIO_233: 0 -> 1 (Lidar Power ON)

[timestamp] RX <bytes> bytes
  HEX: FA FB 63 15 01 00 00 00 04 00 ...
  PKT: CMD=0x15 LEN=99
       (STATUS - Sensor/IMU/Battery data)
```

## Results

This MITM approach successfully captured the complete GD32 protocol, including:

- **Lidar Activation Sequence** (CMD=0x65, CMD=0xA2, CMD=0x97, CMD=0x71)
- **Heartbeat Protocol** (CMD=0x66 every 20ms)
- **Status Packets** (CMD=0x15 with 96 bytes of sensor data)
- **Motor Control** (CMD=0x66 with speed parameters)
- **Checksum Algorithm** (16-bit big-endian word sum)

See [LIDAR_INITIALIZATION_SEQUENCE.md](docs/LIDAR_INITIALIZATION_SEQUENCE.md) for detailed findings.

## Requirements

### Robot (CRL-200S)
- Allwinner A33 running Tina Linux
- Root access via SSH
- GD32F103 motor controller on `/dev/ttyS3`
- GPIO 233 for lidar power control

### Development Machine
- Rust 1.70+ with ARM cross-compilation support
- `armv7-unknown-linux-musleabihf` target installed
- SSH access to robot

## Safety Notes

⚠️ **MITM Mode Disables Original Firmware** - AuxCtrl cannot control the robot while MITM is active. Always restore normal operation before leaving the robot unattended.

⚠️ **Boot-Persistent Changes** - MITM setup modifies system files (`/etc/init.d/rcS`). Use `mitm_cleanup.sh` to fully remove.

⚠️ **GPIO Monitoring** - The MITM logger monitors GPIO 233 for lidar power state. Ensure GPIO is exported and accessible.

## Troubleshooting

### "Permission denied" on /dev/ttyS3_hardware
The real serial port wasn't moved. Run `mitm_boot_setup.sh` on the robot.

### AuxCtrl crashes immediately
MITM binary may not be running. Check: `ps aux | grep serial_mitm`

### No log file created
MITM may have failed to start. Check `/tmp/mitm.log` for errors.

### Robot doesn't boot after MITM
MITM setup may have corrupted boot scripts. Use serial console to restore `/etc/init.d/rcS`.

## Related Projects

- **[SangamIO](../sangam-io/)** - Rust library implementing the reverse-engineered GD32 protocol
- **[VacuumTiger](../)** - Complete robotic vacuum control system

## License

Apache-2.0

## Authors

Created by the VacuumTiger project team.
