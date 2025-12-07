# GD32 Command Reference

This document lists all known GD32 commands discovered from reverse-engineering the AuxCtrl binary, captured logs, and current SangamIO implementation.

**Source Reference:** `src/devices/crl200s/constants.rs`, `src/devices/crl200s/gd32/packet.rs`, `src/devices/crl200s/gd32/commands.rs`

## Command Format

All commands use the packet format: `[0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC_H] [CRC_L]`

- **Sync bytes**: 0xFA 0xFB
- **LEN**: Length of CMD + PAYLOAD + CRC (minimum 3)
- **CMD**: Command byte
- **PAYLOAD**: Variable length data
- **CRC**: 16-bit big-endian word sum checksum (except 0x08)

### CRC Calculation (from `packet.rs`)

```rust
fn checksum(data: &[u8]) -> u16 {
    let mut sum: u16 = 0;
    let mut i = 0;
    // Sum 16-bit big-endian words
    while i + 1 < data.len() {
        let word = ((data[i] as u16) << 8) | (data[i + 1] as u16);
        sum = sum.wrapping_add(word);
        i += 2;
    }
    // XOR with odd trailing byte
    if i < data.len() {
        sum ^= data[i] as u16;
    }
    sum
}
```

---

## Master Command Table

### Confidence Levels
- **HIGH**: Verified through multiple sources (code + logs + working implementation)
- **MEDIUM**: Found in reverse-engineered code OR observed in logs, but not fully tested
- **LOW**: Single source, purpose inferred from context

| Hex | Name | Payload | Implemented | MITM Usage | Confidence | Reason |
|-----|------|---------|-------------|------------|------------|--------|
| **Heartbeat/System** |
| 0x06 | Heartbeat | None | ✅ | 1264x | HIGH | Working in SangamIO, matches AuxCtrl `packetHeartBeat` |
| 0x07 | Version Request | None | ✅ | 2x | HIGH | Working in SangamIO, matches AuxCtrl `packetRequireSystemVersion` |
| 0x08 | Initialize/IMU Zero | None | ✅ | 1x | HIGH | Working in SangamIO, matches AuxCtrl `packetSetIMUZero`, no CRC |
| 0x04 | MCU Sleep | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "mcu", action: Disable }` |
| 0x05 | Wakeup Ack | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "mcu", action: Enable }` |
| 0x0A | Reset Error Code | None | ✅ | 1x | HIGH | `ComponentControl { id: "mcu", action: Reset }` |
| 0x0C | Protocol Sync | 1 byte | ✅ | 1x | HIGH | First cmd at boot, wakes GD32, payload 0x01 |
| 0x0D | Request STM32 Data | None | ✅ | 86x | HIGH | Internal driver polling every ~3s in heartbeat loop |
| **Motor Control** |
| 0x65 | Motor Mode | 1 byte | ✅ | 5x | HIGH | Working in SangamIO, 0x00=idle, 0x02=nav mode |
| 0x66 | Motor Velocity | 8 bytes | ✅ | 11473x | HIGH | Working in SangamIO, primary motion control |
| 0x67 | Motor Speed | 4 bytes | ✅ | 0 | MEDIUM | Implemented but not used, direct wheel control |
| 0x68 | Air Pump (Vacuum) | 2 bytes | ✅ | 1x | HIGH | Working in SangamIO, matches AuxCtrl `packetBlowerSpeed` |
| 0x69 | Side Brush | 1 byte | ✅ | 32x | HIGH | Working in SangamIO, matches AuxCtrl `packetBrushSpeed` |
| 0x6A | Main Brush | 1 byte | ✅ | 31x | HIGH | Working in SangamIO, matches AuxCtrl `packetRollingSpeed` |
| 0x6B | Water Pump / Motor Init | 1 byte | ✅ | 95x | HIGH | Dual-use: boot handshake (2s pulse) + water pump for 2-in-1 mop box (0-100%) |
| **Lidar Control** |
| 0x17 | Lidar Config | 4 bytes | ❌ | 1x | LOW | Init: `[0x01, 0xF0, 0xDF, 0xFA]`, response: `[0x01]` |
| 0x18 | Lidar Query | None/8 bytes | ❌ | 1x | LOW | Request: none, Response: `[0x04, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00]` |
| 0x19 | Lidar Enable | 1 byte | ❌ | 1x | LOW | Init sequence, payload 0x01 |
| 0x71 | Lidar PWM | 4 bytes | ✅ | 6971x | HIGH | Working in SangamIO, matches AuxCtrl `controlLidarPwm` |
| 0x7C | Unknown Lidar | 3 bytes | ❌ | 1x | LOW | Init: `[0x00, 0xFF, 0xFF]`, purpose unknown |
| 0x97 | Lidar Power | 1 byte | ✅ | 3x | HIGH | Working in SangamIO, matches AuxCtrl `packetLidarPower` |
| **Sensor Control** |
| 0x78 | Cliff IR Control | 1 byte | ✅ | 1x | HIGH | `ComponentControl { id: "cliff_ir", action: Enable/Disable }` |
| 0x79 | Cliff IR Direction | 1 byte | ✅ | 1x | MEDIUM | `ComponentControl { id: "cliff_ir", action: Configure { direction } }` |
| 0x86 | Dock IR Sensor | 1 byte | ❌ | 3x | MEDIUM | Payloads: 0x00=off, 0x41=on, dock detection sensor |
| 0x9D | Unknown Sensor | 1 byte | ❌ | 2x | LOW | Payload 0x01, sent during init |
| **LED/UI** |
| 0x8D | Button LED State | 1 byte | ✅ | 20x | HIGH | Working in SangamIO, 19 modes discovered (see LED section) |
| **Power Management** |
| 0x99 | Main Board Power | 1 byte | ✅ | 0 | MEDIUM | `ComponentControl { id: "main_board", action: Enable/Disable }` |
| 0x9A | Main Board Restart | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "main_board", action: Reset }` |
| 0x9B | Charger Power | 1 byte | ✅ | 0 | MEDIUM | `ComponentControl { id: "charger", action: Enable/Disable }` |
| **Calibration** |
| 0xA1 | IMU Factory Calibrate | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "imu", action: Reset }` |
| 0xA2 | IMU Calibrate State | 0 or 4 bytes | ✅ | 3x | HIGH | `ComponentControl { id: "imu", action: Enable }` |
| 0xA3 | Compass Calibrate | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "compass", action: Reset }` |
| 0xA4 | Compass Cal State | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "compass", action: Enable }` |

### Response Commands (GD32 → Host)

| Hex | Name | Payload | MITM Count | Confidence | Reason |
|-----|------|---------|------------|------------|--------|
| 0x06 | Heartbeat Ack | None | 130x | HIGH | Echo acknowledgment |
| 0x07 | Version Response | Variable | 2x | HIGH | Response to version request |
| 0x08 | Init Ack | None | 1x | HIGH | Echo acknowledgment |
| 0x0C | Protocol Sync Ack | 1 byte | 1x | HIGH | Echo of 0x0C command (~270ms delay) |
| 0x15 | Status Packet | 96 bytes | 12993x | HIGH | Continuous sensor data @ ~500Hz |
| 0x17 | Lidar Config Ack | 1 byte | 1x | LOW | Response `[0x01]` |
| 0x18 | Lidar Query Response | 8 bytes | 1x | LOW | Lidar state data |
| 0xA2 | IMU Cal Response | 1 byte | 3x | HIGH | Response `[0x01]` |

---

### LED States (0x8D) - Complete Mode Table

The LED command uses a lookup table in GD32 firmware (not bit-field encoded).
Values 0-18 have distinct behaviors; values 19+ default to Orange Stable.

| Value | Color | Animation | Likely Purpose |
|-------|-------|-----------|----------------|
| 0 | OFF | - | Off |
| 1 | Blue | Stable | Idle/Standby |
| 2 | Blue | Stable | (duplicate of 1) |
| 3 | Orange | Stable | Warning |
| 4 | Orange | Slow Wobble | Charging (breathing) |
| 5 | Orange | Fast Blink → OFF | Transition |
| 6 | Orange | Medium Blink | Charging active |
| 7 | Red | Stable | Error |
| 8 | Red | Medium Blink | Error (attention) |
| 9 | Red | Stable | (duplicate of 7) |
| 10 | Orange/Red | Alternating Blink | Critical warning |
| 11 | Blue | Medium Blink | Processing/Active |
| 12 | Blue | Slow Wobble | Standby (breathing) |
| 13 | Blue | Stable | (duplicate of 1) |
| 14 | Blue | Fast Blink → ON | Boot/Init sequence |
| 15 | Red→Blue | Sequence | State transition |
| 16 | Red | Medium Blink | (duplicate of 8) |
| 17 | Purple | Stable | Special/Factory mode |
| 18 | Orange | Stable | (duplicate of 3) |
| 19+ | Orange | Stable | Default fallback |

**Usage**: `ComponentControl { id: "led", action: Configure { config: { "state": U8(N) } } }` where N is 0-18

---

## Lidar Auto-Tuning

The lidar motor requires PWM control (0x71) to spin at the correct speed for scanning. SangamIO implements **automatic PWM discovery** that finds the maximum stable PWM value - no user configuration needed.

### How It Works

When lidar is enabled via `ComponentControl { id: "lidar", action: Enable }`:

1. **Start at 100% PWM** - Motor begins at full speed
2. **Adaptive step ramping** - PWM decreases with large steps (20%) initially
3. **Direction reversal on state change** - When scans start/stop, direction reverses and step halves
4. **Convergence** - Settles at maximum stable PWM when step size reaches 2%
5. **Stop sending commands** - Once stable, no more PWM commands are sent

### Settling Detection

After each PWM change, the driver waits before evaluating success:
- **250ms timeout** OR **3 consecutive health packets (0xAE)** - whichever comes first
- This allows the motor to stabilize before checking if measurement scans (0xAD) arrive

### State Machine

```
┌─────────────────┐
│ RAMPING         │  PWM starts at 100%, step starts at 20%
│ (finding range) │  Step halves on each direction reversal
└────────┬────────┘
         │ step_size <= 2% and scans stable
         ▼
┌─────────────────┐
│ STABLE          │  Maximum stable PWM found
│ (optimal found) │  Stop sending PWM commands (0x71)
└────────┬────────┘
         │ scans stop unexpectedly
         ▼
┌─────────────────┐
│ RECOVERY        │  Increase PWM by 5% until scans resume
└─────────────────┘  Return to STABLE when recovered
```

### Example Tuning Sequence

Starting at 100% with 20% step size:

```
100% → 80% (step=20, no scans yet, decrease)
 80% → 60% (step=20, no scans, decrease)
 60%       (scans start! reverse direction, step halves to 10%)
 60% → 70% (step=10, scans working, increase)
 70% → 80% (step=10, scans working, increase)
 80%       (scans stop! reverse direction, step halves to 5%)
 80% → 75% (step=5, no scans, decrease)
 75%       (scans start! reverse direction, step halves to 2%)
 75% → 73% (step=2, scans stable, step <= MIN_STEP)
         → STABLE at 73% (stop sending PWM commands)
```

Total tuning time: ~400-800ms (depends on motor characteristics)

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `SCAN_TIMEOUT_MS` | 200 | No scan within this time = "not receiving" |
| `SETTLING_TIMEOUT_MS` | 250 | Wait after PWM change before evaluating |
| `SETTLING_HEALTH_COUNT` | 3 | OR wait for this many health packets |
| `INITIAL_STEP` | 20% | Starting step size |
| `MIN_STEP` | 2% | Convergence threshold |
| `MIN_PWM` | 30% | Never go below this |
| `MAX_PWM` | 100% | Never exceed this |
| `RECOVERY_STEP` | 5% | Fixed step for recovery mode |

### TCP API

**Enable lidar (starts auto-tuning):**
```protobuf
RobotCommand {
  command: ComponentControl {
    id: "lidar",
    action: Enable {}
  }
}
```

**Disable lidar (stops motor, resets tuning state):**
```protobuf
RobotCommand {
  command: ComponentControl {
    id: "lidar",
    action: Disable {}
  }
}
```

**Note:** Manual PWM configuration is no longer supported. The `Configure` action for lidar is ignored.

