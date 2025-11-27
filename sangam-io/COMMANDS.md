# GD32 Command Reference

This document lists all known GD32 commands discovered from reverse-engineering the AuxCtrl binary, captured logs, and current SangamIO implementation.

## Command Format

All commands use the packet format: `[0xFA 0xFB] [LEN] [CMD] [PAYLOAD] [CRC_H] [CRC_L]`

- **Sync bytes**: 0xFA 0xFB
- **LEN**: Length of CMD + PAYLOAD + CRC (minimum 3)
- **CMD**: Command byte
- **PAYLOAD**: Variable length data
- **CRC**: 16-bit big-endian word sum checksum (except 0x08)

---

## Master Command Table

### Confidence Levels
- **HIGH**: Verified through multiple sources (code + logs + working implementation)
- **MEDIUM**: Found in reverse-engineered code OR observed in logs, but not fully tested
- **LOW**: Single source, purpose inferred from context

| Hex | Name | Payload | Implemented | R2D Usage | Confidence | Reason |
|-----|------|---------|-------------|-----------|------------|--------|
| **Heartbeat/System** |
| 0x06 | Heartbeat | None | ✅ | 606x | HIGH | Working in SangamIO, matches AuxCtrl `packetHeartBeat` |
| 0x07 | Version Request | None | ✅ | 3x | HIGH | Working in SangamIO, matches AuxCtrl `packetRequireSystemVersion` |
| 0x08 | Initialize/IMU Zero | None | ✅ | 13x | HIGH | Working in SangamIO, matches AuxCtrl `packetSetIMUZero`, no CRC |
| 0x04 | MCU Sleep | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "mcu", action: Disable }` |
| 0x05 | Wakeup Ack | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "mcu", action: Enable }` |
| 0x0A | Reset Error Code | None | ✅ | 1x | HIGH | `ComponentControl { id: "mcu", action: Reset }` |
| 0x0D | Request STM32 Data | None | ✅ | 98x | HIGH | Internal driver polling every ~3s in heartbeat loop |
| **Motor Control** |
| 0x65 | Motor Mode | 1 byte | ✅ | 4x | HIGH | Working in SangamIO, 0x00=idle, 0x02=nav mode |
| 0x66 | Motor Velocity | 8 bytes | ✅ | 6627x | HIGH | Working in SangamIO, primary motion control |
| 0x67 | Motor Speed | 4 bytes | ✅ | 0 | MEDIUM | Implemented but not used in R2D, direct wheel control |
| 0x68 | Air Pump (Vacuum) | 2 bytes | ✅ | 0 | HIGH | Working in SangamIO, matches AuxCtrl `packetBlowerSpeed` |
| 0x69 | Side Brush | 1 byte | ✅ | 15x | HIGH | Working in SangamIO, matches AuxCtrl `packetBrushSpeed` |
| 0x6A | Main Brush | 1 byte | ✅ | 18x | HIGH | Working in SangamIO, matches AuxCtrl `packetRollingSpeed` |
| 0x6B | Unknown Actuator | 1 byte | ❌ | 2x | LOW | Only observed in R2D log with payload 0x00, purpose unknown |
| **Lidar Control** |
| 0x71 | Lidar PWM | 4 bytes | ✅ | 5073x | HIGH | Working in SangamIO, matches AuxCtrl `controlLidarPwm` |
| 0x97 | Lidar Power | 1 byte | ✅ | 6x | HIGH | Working in SangamIO, matches AuxCtrl `packetLidarPower` |
| **Sensor Control** |
| 0x78 | Cliff IR Control | 1 byte | ✅ | 0 | MEDIUM | `ComponentControl { id: "cliff_ir", action: Enable/Disable }` |
| 0x79 | Cliff IR Direction | 1 byte | ✅ | 0 | MEDIUM | `ComponentControl { id: "cliff_ir", action: Configure { direction } }` |
| 0x86 | Unknown Sensor | 1 byte | ❌ | 2x | LOW | Only in R2D log, payload 0x10→0x00, possibly dock IR |
| 0x9D | Unknown | 1 byte | ❌ | 1x | LOW | Only in R2D log, payload 0x01, purpose unknown |
| **LED/UI** |
| 0x8D | Button LED State | 1 byte | ✅ | 7x | HIGH | Working in SangamIO, 19 modes discovered (see LED section) |
| **Power Management** |
| 0x99 | Main Board Power | 1 byte | ✅ | 0 | MEDIUM | `ComponentControl { id: "main_board", action: Enable/Disable }` |
| 0x9A | Main Board Restart | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "main_board", action: Reset }` |
| 0x9B | Charger Power | 1 byte | ✅ | 0 | MEDIUM | `ComponentControl { id: "charger", action: Enable/Disable }` |
| **Calibration** |
| 0xA1 | IMU Factory Calibrate | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "imu", action: Reset }` |
| 0xA2 | IMU Calibrate State | 0 or 4 bytes | ✅ | 2x | HIGH | `ComponentControl { id: "imu", action: Enable }` |
| 0xA3 | Compass Calibrate | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "compass", action: Reset }` |
| 0xA4 | Compass Cal State | None | ✅ | 0 | MEDIUM | `ComponentControl { id: "compass", action: Enable }` |

### Response Commands (GD32 → Host)

| Hex | Name | Payload | Confidence | Reason |
|-----|------|---------|------------|--------|
| 0x15 | Status Packet | 96 bytes | HIGH | Continuous sensor data, fully decoded in SangamIO |
| 0x07 | Version Response | Variable | HIGH | Response to version request |

---

## Return-to-Dock Command Sequence

From captured ~160 second operation:

```
Phase 1: Startup
  0x08 (13x) → Initialize/wake MCU
  0x07 (3x)  → Request version
  0x8D (4x)  → LED state 0x01 (standby)

Phase 2: Navigation Active
  0x65       → Motor mode 0x02 (navigation)
  0xA2       → Lidar prep [0x10, 0x0E, 0x00, 0x00]
  0x97       → Lidar power on
  0x71       → Lidar PWM (continuous, 5073x total)
  0x66       → Motor velocity (continuous, 6627x total)
  0x8D       → LED state 0x06 (active)
  0x0D       → Request STM32 data (every ~3s)
  0x69/0x6A  → Brush activation (brief)

Phase 3: Dock Approach
  0x86       → Unknown (0x10)
  0x6B       → Unknown (0x00)
  0x9D       → Unknown (0x01)

Phase 4: Shutdown
  0x65       → Motor mode 0x00 (idle)
  0x97       → Lidar power off
  0x86       → Unknown (0x00)
  0x0A       → Reset error codes
```

---

## Status Packet (0x15) Structure

96-byte packet containing all sensor data:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0x01 | 1 | Bumper flags | Bit 1: right, Bit 2: left |
| 0x03 | 1 | Cliff flags | Bits 0-3: LS, LF, RF, RS |
| 0x04 | 1 | Dustbox flags | Bit 2: attached |
| 0x07 | 1 | Charging flags | Bit 0: battery, Bit 1: charging |
| 0x10 | 8 | Right encoder | u64 LE tick count |
| 0x18 | 8 | Left encoder | u64 LE tick count |
| 0x3A | 4 | Start button | u32 LE press state |
| 0x3E | 4 | Dock button | u32 LE press state |

---

## Implementation Status

| Status | Count | Commands |
|--------|-------|----------|
| ✅ Implemented | 25 | 0x04, 0x05, 0x06, 0x07, 0x08, 0x0A, 0x0D, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x71, 0x78, 0x79, 0x8D, 0x97, 0x99, 0x9A, 0x9B, 0xA1, 0xA2, 0xA3, 0xA4 |
| ❌ Not Implemented | 3 | 0x6B, 0x86, 0x9D |

---

## Implementation Priority

### Medium Priority (Used in R2D, low confidence)
| Hex | Name | Reason |
|-----|------|--------|
| 0x86 | Unknown Sensor | Used during dock approach, may be dock IR |
| 0x6B | Unknown Actuator | Brief use, may be related to docking |
| 0x9D | Unknown | Single use, purpose unclear |


---

## Notes

### Timing Requirements
- Heartbeat (0x06) or velocity (0x66) must be sent every 20-50ms
- Mode switch (0x65) requires 100ms delay before actuator commands
- Initialize (0x08) may need repeated sends for up to 5 seconds

### Command 0xA2 Dual Purpose
Observed with two different usages:
1. **No payload**: Query IMU calibration state (from AuxCtrl code)
2. **Payload [0x10, 0x0E, 0x00, 0x00]**: Sent before lidar power-on (from R2D log)

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

### Brush Naming
- 0x69 = Side brush (AuxCtrl: "BrushSpeed")
- 0x6A = Main/rolling brush (AuxCtrl: "RollingSpeed")

---

## TCP API - ComponentControl

All sensors and actuators are controlled via the unified `ComponentControl` command:

```json
{
  "type": "ComponentControl",
  "id": "<component_id>",
  "action": { "type": "<action_type>", ... }
}
```

### Component Actions

| Action | Description | Example |
|--------|-------------|---------|
| `Enable` | One-time activation (optional config) | `{ "type": "Enable" }` or `{ "type": "Enable", "config": {...} }` |
| `Disable` | One-time deactivation (optional config) | `{ "type": "Disable" }` |
| `Reset` | Factory reset / emergency stop (optional config) | `{ "type": "Reset" }` |
| `Configure` | Continuous updates (velocity, speed) | `{ "type": "Configure", "config": { "speed": {"U8": 50} } }` |

**Design:**
- `Enable/Disable/Reset` - one-time setup/teardown with optional config (e.g., mode)
- `Configure` - continuous updates called repeatedly (e.g., velocity, brush speed)

### Supported Components

| Component ID | Enable | Disable | Reset | Configure |
|--------------|--------|---------|-------|-----------|
| `drive` | Mode 0x02 (or `{mode: U8}`) | Stop + Mode 0x00 | Emergency stop | Velocity or Tank (see below) |
| `vacuum` | 100% speed | 0% speed | - | `{ "speed": U8 }` |
| `main_brush` | 100% speed | 0% speed | - | `{ "speed": U8 }` |
| `side_brush` | 100% speed | 0% speed | - | `{ "speed": U8 }` |
| `led` | - | - | - | `{ "state": U8 }` |
| `lidar` | Power on + PWM | Power off | - | - |
| `imu` | Query state (0xA2) | - | Factory calibrate (0xA1) | - |
| `compass` | Query state (0xA4) | - | Start calibration (0xA3) | - |
| `cliff_ir` | Enable (0x78) | Disable (0x78) | - | `{ "direction": U8 }` (0x79) |
| `main_board` | Power on (0x99) | Power off (0x99) ⚠️ | Restart (0x9A) ⚠️ | - |
| `charger` | Enable (0x9B) | Disable (0x9B) | - | - |
| `mcu` | Wakeup ack (0x05) | Sleep (0x04) | Reset error code (0x0A) | - |

### Drive Configuration Modes

The `drive` component supports two configuration modes:

**Velocity Mode** (differential drive):
- `linear`: F32 - Linear velocity in m/s (positive = forward)
- `angular`: F32 - Angular velocity in rad/s (positive = counter-clockwise)

**Tank Drive Mode** (direct wheel control):
- `left`: F32 - Left wheel velocity in m/s
- `right`: F32 - Right wheel velocity in m/s

### TCP JSON Examples

**Drive forward at 0.2 m/s:**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Configure", "config": {"linear": {"F32": 0.2}, "angular": {"F32": 0.0}}}}
```

**Turn in place (0.5 rad/s):**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Configure", "config": {"linear": {"F32": 0.0}, "angular": {"F32": 0.5}}}}
```

**Tank drive (left=0.1, right=0.2):**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Configure", "config": {"left": {"F32": 0.1}, "right": {"F32": 0.2}}}}
```

**Stop (graceful):**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Disable"}}
```

**Emergency stop (immediate halt):**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Reset"}}
```

**Enable drive (default nav mode 0x02):**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Enable"}}
```

**Enable drive with specific mode:**
```json
{"type": "ComponentControl", "id": "drive", "action": {"type": "Enable", "config": {"mode": {"U8": 2}}}}
```

**Set vacuum to 75% speed:**
```json
{"type": "ComponentControl", "id": "vacuum", "action": {"type": "Configure", "config": {"speed": {"U8": 75}}}}
```

**Trigger IMU factory calibration:**
```json
{"type": "ComponentControl", "id": "imu", "action": {"type": "Reset"}}
```

**Set LED to charging animation (mode 4):**
```json
{"type": "ComponentControl", "id": "led", "action": {"type": "Configure", "config": {"state": {"U8": 4}}}}
```

**Power on main board (A33):**
```json
{"type": "ComponentControl", "id": "main_board", "action": {"type": "Enable"}}
```

**Restart main board (⚠️ daemon will terminate!):**
```json
{"type": "ComponentControl", "id": "main_board", "action": {"type": "Reset"}}
```

**Enable charger power:**
```json
{"type": "ComponentControl", "id": "charger", "action": {"type": "Enable"}}
```

**Disable charger power:**
```json
{"type": "ComponentControl", "id": "charger", "action": {"type": "Disable"}}
```

**Put MCU to sleep:**
```json
{"type": "ComponentControl", "id": "mcu", "action": {"type": "Disable"}}
```

**Wake MCU (acknowledge wakeup):**
```json
{"type": "ComponentControl", "id": "mcu", "action": {"type": "Enable"}}
```

**Reset MCU error codes:**
```json
{"type": "ComponentControl", "id": "mcu", "action": {"type": "Reset"}}
```
