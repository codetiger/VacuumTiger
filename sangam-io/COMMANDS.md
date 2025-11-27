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
| 0x6B | Motor Controller Init | 1 byte | ❌ | 95x | HIGH | Boot handshake: 0x00=off, 0x64=ready. Pulsed for 2s at startup |
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

## AuxCtrl Initialization Sequence (MITM Captured)

From MITM capture run10 - boot sequence analyzed in 5 phases:

### Phase 1: System Init (immediate)
```
Step  CMD   Packet                              Purpose
────  ────  ──────────────────────────────────  ─────────────────────────
  1   0x0C  FA FB 04 0C 01 0C 01                System init / protocol sync
  2   0x07  FA FB 03 07 00 07                   Request version
  3   0x8D  FA FB 04 8D 01 8D 01                LED state 0x01 (standby blue)
  4   0x0A  FA FB 03 0A 00 0A                   Reset error codes
```

### Phase 2: Hardware Config (within 50ms)
```
  5   0x07  FA FB 03 07 00 07                   Request version (retry)
  6   0x17  FA FB 07 17 01 F0 DF FA 07 1A      Lidar motor config
  7   0x18  FA FB 03 18 00 18                   Lidar query
  8   0x97  FA FB 04 97 00 97 00                Lidar power OFF
  9   0x9D  FA FB 04 9D 01 9D 01                Unknown sensor enable
 10   0x71  FA FB 07 71 00 00 00 00 71 00      Lidar PWM = 0
```

### Phase 3: Actuator Init (immediate)
```
 11   0x65  FA FB 04 65 00 65 00                Motor mode 0x00 (idle)
 12   0x68  FA FB 05 68 00 00 68 00             Vacuum 0%
 13   0x69  FA FB 04 69 00 69 00                Side brush 0%
 14   0x6A  FA FB 04 6A 00 6A 00                Main brush 0%
 15   0x6B  FA FB 04 6B 00 6B 00                Motor controller init = OFF
 16   0x08  FA FB 03 08 00 08                   Initialize/IMU zero
 17   0x19  FA FB 04 19 01 19 01                Lidar enable
 18   0x7C  FA FB 06 7C 00 FF FF 7B FF          Unknown lidar config
```

### Phase 4: Motor Controller Handshake (~2 seconds)
```
 19   0x8D  FA FB 04 8D 01 8D 01                LED state refresh
 ...  0x6B  FA FB 04 6B 64 6B 64                Motor ctrl init = 100% (every ~22ms)
 ...  0x66  FA FB 0B 66 00 00 00 00 ...         Velocity = 0 (interleaved)
```
**Note:** GD32 requires 2 seconds of 0x6B [64] pulses before accepting motion commands.
Robot does NOT move during this phase (encoders unchanged).

### Phase 5: Normal Operation
```
 ...  0x66  FA FB 0B 66 00 00 00 00 ...         Motor velocity (heartbeat continues)
 ...  0x71  FA FB 07 71 ...                     Lidar PWM control
```
**Note:** 0x6B stops after handshake. Only 0x66 continues as heartbeat.

## Return-to-Dock Command Sequence (Legacy)

From earlier captured ~160 second operation:

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
  0x86       → Dock IR (0x10)
  0x6B       → Mop motor (0x00)
  0x9D       → Unknown (0x01)

Phase 4: Shutdown
  0x65       → Motor mode 0x00 (idle)
  0x97       → Lidar power off
  0x86       → Dock IR off (0x00)
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
| ✅ Implemented | 26 | 0x04, 0x05, 0x06, 0x07, 0x08, 0x0A, 0x0C, 0x0D, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x71, 0x78, 0x79, 0x8D, 0x97, 0x99, 0x9A, 0x9B, 0xA1, 0xA2, 0xA3, 0xA4 |
| ❌ Not Implemented | 9 | 0x17, 0x18, 0x19, 0x6B, 0x7C, 0x86, 0x9D |

---

## Implementation Priority

### High Priority (Boot sequence, well understood)
| Hex | Name | MITM Usage | Reason |
|-----|------|------------|--------|
| 0x0C | Protocol Sync | 1x | ✅ DONE - First boot command, wakes GD32 |
| 0x6B | Motor Controller Init | 95x | 2-second boot handshake required before motion commands work |

### Medium Priority (Init sequence, partially understood)
| Hex | Name | MITM Usage | Reason |
|-----|------|------------|--------|
| 0x86 | Dock IR Sensor | 3x | Dock detection, 0x00=off, 0x41=on |
| 0x17 | Lidar Config | 1x | Lidar init, payload `[0x01, 0xF0, 0xDF, 0xFA]` |
| 0x18 | Lidar Query | 1x | Lidar state query, response has 8 bytes |
| 0x19 | Lidar Enable | 1x | Lidar init, payload 0x01 |

### Low Priority (Single use, unknown purpose)
| Hex | Name | MITM Usage | Reason |
|-----|------|------------|--------|
| 0x7C | Unknown Lidar | 1x | Init, payload `[0x00, 0xFF, 0xFF]` |
| 0x9D | Unknown Sensor | 2x | Init, payload 0x01 |


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

### Protocol Commands

These are standalone commands not using ComponentControl:

**Protocol Sync (wake GD32 at boot):**
```json
{"type": "ProtocolSync"}
```
This is a fire-and-forget command. GD32 echoes it back after ~270ms. Typically sent once at boot before any other commands.
