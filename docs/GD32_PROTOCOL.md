# GD32F103 Communication Protocol Specification

**Status**: ✅ VERIFIED
**Source**: Reverse engineered from 3irobotix CRL-200S vacuum robot
**Last Updated**: 2025-10-30

This document specifies the communication protocol between a main processor (originally Allwinner A33) and the GD32F103 microcontroller for robotic vacuum control.

## Table of Contents

- [Physical Layer](#physical-layer)
- [Protocol Layer](#protocol-layer)
- [Initialization Sequence](#initialization-sequence)
- [Command Reference](#command-reference)
- [Status Packet Format](#status-packet-format)
- [Timing Requirements](#timing-requirements)
- [Error Handling](#error-handling)
- [Implementation Notes](#implementation-notes)

## Physical Layer

### Serial Configuration

| Parameter | Value |
|-----------|-------|
| **Port** | `/dev/ttyS3` (device-specific) |
| **Baud Rate** | 115200 |
| **Data Format** | 8N1 (8 data bits, no parity, 1 stop bit) |
| **Flow Control** | None |
| **Mode** | Raw, non-blocking |

## Protocol Layer

### Packet Structure

```
┌──────┬──────┬────────┬────────┬─────────────┬──────┐
│SYNC1 │SYNC2 │ LENGTH │ CMD_ID │   PAYLOAD   │ CRC  │
├──────┼──────┼────────┼────────┼─────────────┼──────┤
│ 0xFA │ 0xFB │ 1 byte │ 1 byte │ 0-254 bytes │1 byte│
└──────┴──────┴────────┴────────┴─────────────┴──────┘
```

**Field Descriptions**:
- **SYNC1, SYNC2**: Fixed synchronization bytes (0xFA, 0xFB)
- **LENGTH**: Total length of remaining packet (CMD_ID + PAYLOAD + CRC)
- **CMD_ID**: Command identifier (see Command Reference)
- **PAYLOAD**: Variable-length data (0-254 bytes)
- **CRC**: Simple XOR checksum calculated as: `CMD_ID ⊕ all PAYLOAD bytes ⊕ CMD_ID`

### CRC Algorithm

The checksum is a simple XOR of the command ID with itself and all payload bytes:

```python
def calculate_crc(cmd_id, payload):
    crc = cmd_id
    for byte in payload:
        crc ^= byte
    crc ^= cmd_id
    return crc & 0xFF
```

**Example**:
```
Packet: FA FB 03 06 00 06
        │  │  │  │  │  └─ CRC: 0x06 ⊕ 0x00 ⊕ 0x06 = 0x06 ✓
        │  │  │  │  └──── PAYLOAD: 0x00
        │  │  │  └─────── CMD_ID: 0x06
        │  │  └────────── LENGTH: 3 (CMD + PAYLOAD + CRC)
        │  └───────────── SYNC2: 0xFB
        └──────────────── SYNC1: 0xFA
```

### Communication Model

**Bidirectional UART Communication**:
- **TX (CPU→GD32)**: Commands for control and queries
- **RX (GD32→CPU)**: Status packets (CMD=0x15) containing sensor data

**Communication Pattern**:
1. **Initialization Phase**: Send CMD=0x08 repeatedly (200ms intervals) for up to 5 seconds until GD32 responds
2. **Operational Phase**: Maintain heartbeat (CMD=0x66) every 20-50ms
3. **Status Updates**: GD32 autonomously sends CMD=0x15 packets (~1-2 Hz)

## Initialization Sequence

The GD32 requires a specific initialization sequence before it will respond to commands:

### Phase 1: Wake-up Loop (up to 5 seconds)

Send CMD=0x08 repeatedly every 200ms until GD32 responds with CMD=0x15:

```
TX: FA FB 63 08 20 08 08 20 08 08 20 08 08 ... (99 bytes total)
    └─ Repeating pattern: 0x20 0x08 0x08 (32 times)
```

### Phase 2: Version Request (Optional)

```
TX: FA FB 03 07 00 07
RX: [Version string, e.g., "2.0.1_19082728"]
```

### Phase 3: Enable Command

```
TX: FA FB 03 06 00 06  (Wake/enable motors)
```

### Phase 4: Control Mode

```
TX: FA FB 04 8D 01 8D 01  (Set control mode)
```

### Phase 5: Heartbeat Loop

Send CMD=0x66 every 20-50ms:

```
TX: FA FB 0B 66 00 00 00 00 00 00 00 00 66 00
RX: FA FB 63 15 ... (99-byte status packet)
```

## Command Reference

### System Control Commands

| CMD ID | Direction | Name | Payload | Description |
|--------|-----------|------|---------|-------------|
| 0x04 | CPU→GD32 | STM32_SLEEP | 0 bytes | Put GD32 into sleep mode |
| 0x05 | CPU→GD32 | WAKEUP_ACK | 0 bytes | Acknowledge wakeup signal |
| 0x06 | CPU→GD32 | WAKE | 0 bytes | Wake/enable motors |
| 0x07 | CPU→GD32 | GET_VERSION | 0 bytes | Request firmware version |
| 0x08 | CPU→GD32 | INITIALIZE | 96 bytes | Initialization/wake-up sequence |
| 0x0A | CPU→GD32 | RESET_ERROR | 0 bytes | Clear error codes |
| 0x0D | CPU→GD32 | STATUS_REQUEST | 0 bytes | Request status (may not trigger immediate response) |
| 0x9A | CPU→GD32 | RESTART_R16 | 0 bytes | Restart GD32 MCU |

### Motor Control Commands

| CMD ID | Direction | Name | Payload | Description |
|--------|-----------|------|---------|-------------|
| 0x65 | CPU→GD32 | MOTOR_CONTROL_TYPE | 1 byte | Set motor control mode |
| 0x66 | CPU→GD32 | HEARTBEAT | 8 bytes | Heartbeat/keep-alive (critical!) |
| 0x67 | CPU→GD32 | MOTOR_SPEED | 8 bytes | Set left/right motor speeds (2× int32) |

### Actuator Control Commands

| CMD ID | Direction | Name | Payload | Description |
|--------|-----------|------|---------|-------------|
| 0x68 | CPU→GD32 | BLOWER_SPEED | 2 bytes | Set vacuum blower speed (uint16) |
| 0x69 | CPU→GD32 | BRUSH_SPEED | 1 byte | Set side brush speed (uint8) |
| 0x6A | CPU→GD32 | ROLLING_SPEED | 1 byte | Set rolling brush speed (uint8) |

### Sensor Control Commands

| CMD ID | Direction | Name | Payload | Description |
|--------|-----------|------|---------|-------------|
| 0x78 | CPU→GD32 | CLIFF_IR_CONTROL | 1 byte | Enable/disable cliff sensors (bool) |
| 0x79 | CPU→GD32 | CLIFF_IR_DIRECTION | 1 byte | Set cliff sensor direction (bool) |

### Peripheral Control Commands

| CMD ID | Direction | Name | Payload | Description |
|--------|-----------|------|---------|-------------|
| 0x8D | CPU→GD32 | BUTTON_LED_STATE | 1 byte | Control button LEDs/mode |
| 0x97 | CPU→GD32 | LIDAR_POWER | 1 byte | Control lidar power (bool) |
| 0x99 | CPU→GD32 | R16_POWER | 1 byte | Control GD32 power (bool) |
| 0x9B | CPU→GD32 | CHARGER_POWER | 1 byte | Control charging (bool) |

### IMU Calibration Commands

| CMD ID | Direction | Name | Payload | Description |
|--------|-----------|------|---------|-------------|
| 0xA1 | CPU→GD32 | IMU_FACTORY_CALIBRATE | 0 bytes | Factory IMU calibration |
| 0xA2 | CPU→GD32 | IMU_CALIBRATE_STATE | 0 bytes | Get IMU calibration state |
| 0xA3 | CPU→GD32 | GEO_MAGNETISM_CALIBRATE | 0 bytes | Calibrate magnetometer |
| 0xA4 | CPU→GD32 | GEO_MAGNETISM_STATE | 0 bytes | Get magnetometer calibration state |

### Response Commands

| CMD ID | Direction | Name | Payload | Description |
|--------|-----------|------|---------|-------------|
| 0x15 | GD32→CPU | STATUS_DATA | 96 bytes | Sensor data packet (autonomous, ~1-2 Hz) |

## Status Packet Format (CMD=0x15)

The 99-byte status packet (96 bytes payload + 3 bytes header) contains:

- **IMU Data**: Accelerometer, gyroscope, magnetometer readings
- **Wheel Encoders**: Left and right wheel encoder counts for odometry
- **Battery Status**: Voltage, current, charge level
- **Bumper Sensors**: Collision detection states
- **Cliff Sensors**: Drop-off detection readings
- **Button States**: User interface button status
- **Error/Fault Codes**: System diagnostics

**Note**: Detailed byte-level mapping requires further analysis of captured packets.

## Timing Requirements

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Initialization Timeout** | 5 seconds | Retry CMD=0x08 every 200ms |
| **Heartbeat Interval** | 20-50ms | Typically 20ms, critical for motor control |
| **Packet Timeout** | 50ms | Per-packet read timeout |
| **Status Packet Frequency** | 1-2 Hz | Autonomous from GD32 |

## Error Handling

### Missing Heartbeat

If heartbeat (CMD=0x66) stops for >50ms:
- GD32 may enter error state
- Red LED indication possible
- Motors will stop for safety
- Requires re-initialization sequence (CMD=0x08 loop)

### CRC Errors

- Invalid CRC will cause packet rejection
- GD32 will log error internally
- No acknowledgment sent for invalid packets
- Sender should retry or continue with next command

### Initialization Failures

- If no CMD=0x15 response after 5 seconds of CMD=0x08, check:
  - Serial port configuration (115200 baud, 8N1)
  - Correct serial device path
  - GD32 power supply
  - Physical UART connection

## Implementation Notes

### Critical Details

1. **Port Selection**: The serial port is hardware-specific. On the original device, `/dev/ttyS3` was used, NOT `/dev/ttyS1` as initially believed.

2. **Initialization is Mandatory**: The GD32 will NOT respond to commands without the CMD=0x08 initialization sequence. This is not optional.

3. **Heartbeat is Critical**: The CMD=0x66 heartbeat must be sent continuously. Missing heartbeats will cause the GD32 to enter a safety/error state.

4. **Bidirectional Communication**: Unlike initial assumptions, the GD32 DOES send response packets (CMD=0x15). Implementations must handle incoming data.

### Example Code Structure

```rust
// Pseudo-code for implementation
fn communicate_with_gd32() -> Result<()> {
    let mut serial = open_serial("/dev/ttyS3", 115200)?;

    // Phase 1: Initialize
    let start = Instant::now();
    loop {
        send_packet(&serial, cmd_initialize())?;
        if let Some(response) = read_packet_timeout(&serial, 200ms)? {
            if response.cmd_id == 0x15 {
                break; // GD32 is awake!
            }
        }
        if start.elapsed() > Duration::from_secs(5) {
            return Err("Initialization timeout");
        }
    }

    // Phase 2: Setup
    send_packet(&serial, cmd_get_version())?;
    send_packet(&serial, cmd_wake())?;
    send_packet(&serial, cmd_button_led_state(1))?;

    // Phase 3: Operational loop
    loop {
        send_packet(&serial, cmd_heartbeat())?;

        // Process any incoming status packets
        while let Some(packet) = read_packet_nonblocking(&serial)? {
            if packet.cmd_id == 0x15 {
                process_status_data(&packet.payload);
            }
        }

        // Send motor commands as needed
        send_packet(&serial, cmd_motor_speed(left, right))?;

        sleep(Duration::from_millis(20)); // 50Hz heartbeat
    }
}
```

## References

### Research Sources

This protocol was reverse-engineered from the 3irobotix CRL-200S vacuum robot through:
1. Binary decompilation of the `/usr/sbin/AuxCtrl` process (Ghidra)
2. System call tracing with `strace`
3. Serial MITM capture using pseudo-terminal (PTY) interception
4. Empirical testing and validation

### Related Documentation

For the complete research journey and additional details, see the **[VacuumRobot](https://github.com/codetiger/VacuumRobot)** repository:
- Hardware analysis and component identification
- Original firmware analysis
- Protocol discovery evolution
- Serial MITM methodology
- Test results and validation

## License

This specification is released under Apache 2.0 license for educational and research purposes.

---

**Implementation Status**: ✅ Verified and tested with working Rust implementation
