# CRL-200S GD32 Sensor Status Packet (0x15)

> **Device-specific documentation for the CRL-200S robotic vacuum platform.**
> This document describes the 96-byte sensor status packet sent by the GD32 motor controller.

## Packet Structure

```
Packet Format: [0xFA 0xFB] [LEN] [0x15] [PAYLOAD (96 bytes)] [CRC_H] [CRC_L]
Total Size: 102 bytes (6 header/CRC + 96 payload)
Frequency: ~110 Hz (every ~9ms, limited by 115200 baud)
```

## Payload Byte Map

### Quick Reference Table

| Offset (Hex) | Offset (Dec) | Size | Field | Type | Description |
|--------------|--------------|------|-------|------|-------------|
| 0x00 | 0 | 1 | Reserved | - | Unknown purpose |
| 0x01 | 1 | 1 | Bumper Flags | u8 | Bit-field for bumper sensors |
| 0x02 | 2 | 1 | Reserved | - | Unknown purpose |
| 0x03 | 3 | 1 | Cliff Flags | u8 | Bit-field for cliff sensors |
| 0x04 | 4 | 1 | Dustbox Flags | u8 | Bit-field for dustbox detection |
| 0x05-0x06 | 5-6 | 2 | Reserved | - | Unknown purpose |
| 0x07 | 7 | 1 | Charging Flags | u8 | Bit-field for charging/dock status |
| 0x08 | 8 | 1 | Battery Voltage | u8 | Raw voltage (value / 10 = volts) |
| 0x09-0x0F | 9-15 | 7 | Reserved | - | Unknown purpose |
| 0x10-0x11 | 16-17 | 2 | Left Wheel Encoder | u16 LE | Encoder tick count (lower 16 bits) |
| 0x12-0x17 | 18-23 | 6 | Reserved | - | May be upper bytes of encoder or unused |
| 0x18-0x19 | 24-25 | 2 | Right Wheel Encoder | u16 LE | Encoder tick count (lower 16 bits) |
| 0x1A-0x1F | 26-31 | 6 | Reserved | - | May be upper bytes of encoder or unused |
| 0x20-0x27 | 32-39 | 8 | Reserved | - | Unknown purpose |
| 0x28-0x29 | 40-41 | 2 | Gyro X (Yaw) | i16 LE | Raw gyroscope yaw rate |
| 0x2A-0x2B | 42-43 | 2 | Accel X | i16 LE | Raw accelerometer X |
| 0x2C-0x2D | 44-45 | 2 | Gyro Y (Pitch) | i16 LE | Raw gyroscope pitch rate |
| 0x2E-0x2F | 46-47 | 2 | Accel Y | i16 LE | Raw accelerometer Y |
| 0x30-0x31 | 48-49 | 2 | Gyro Z (Roll) | i16 LE | Raw gyroscope roll rate |
| 0x32-0x33 | 50-51 | 2 | Accel Z | i16 LE | Raw accelerometer Z |
| 0x34-0x35 | 52-53 | 2 | Tilt X | i16 LE | LP-filtered gravity vector X |
| 0x36-0x37 | 54-55 | 2 | Tilt Y | i16 LE | LP-filtered gravity vector Y |
| 0x38-0x39 | 56-57 | 2 | Tilt Z | i16 LE | LP-filtered gravity vector Z |
| 0x3A-0x3B | 58-59 | 2 | Start Button | u16 LE | Button press state |
| 0x3C-0x3D | 60-61 | 2 | Reserved | - | Unknown (may be part of button state) |
| 0x3E-0x3F | 62-63 | 2 | Dock Button | u16 LE | Button press state |
| 0x40-0x45 | 64-69 | 6 | Reserved | - | Unknown purpose |
| 0x46 | 70 | 1 | Water Tank Level | u8 | 0=empty, 100=full (2-in-1 mop box) |
| 0x47-0x4F | 71-79 | 9 | Reserved | - | Unknown purpose |
| 0x50-0x5F | 80-95 | 16 | Reserved | - | Padding to 96 bytes |

---

## Detailed Field Descriptions

### Byte 0x01: Bumper Flags

| Bit | Mask | Field | Description |
|-----|------|-------|-------------|
| 0 | 0x01 | Reserved | - |
| 1 | 0x02 | Bumper Right | 1 = right bumper triggered |
| 2 | 0x04 | Bumper Left | 1 = left bumper triggered |
| 3-7 | - | Reserved | - |

**Code Reference:** [`constants.rs`](constants.rs) lines 97-98

### Byte 0x03: Cliff Flags

| Bit | Mask | Field | Description |
|-----|------|-------|-------------|
| 0 | 0x01 | Cliff Left Side | 1 = drop detected |
| 1 | 0x02 | Cliff Left Front | 1 = drop detected |
| 2 | 0x04 | Cliff Right Front | 1 = drop detected |
| 3 | 0x08 | Cliff Right Side | 1 = drop detected |
| 4-7 | - | Reserved | - |

**Code Reference:** [`constants.rs`](constants.rs) lines 99-102

### Byte 0x04: Dustbox Flags

| Bit | Mask | Field | Description |
|-----|------|-------|-------------|
| 0-1 | - | Reserved | - |
| 2 | 0x04 | Dustbox Attached | 1 = dustbox is present |
| 3-7 | - | Reserved | - |

**Note:** This byte may also indicate box type (0x00 = normal dustbox, 0x04 = 2-in-1 mop box) based on MITM observations.

**Code Reference:** [`constants.rs`](constants.rs) line 103

### Byte 0x07: Charging Flags

| Bit | Mask | Field | Description |
|-----|------|-------|-------------|
| 0 | 0x01 | Dock Connected | 1 = on charging dock |
| 1 | 0x02 | Charging | 1 = actively charging |
| 2-7 | - | Reserved | - |

**Code Reference:** [`constants.rs`](constants.rs) lines 95-96

### Byte 0x08: Battery Voltage

- **Type:** u8
- **Conversion:** `voltage_volts = raw_value / 10.0`
- **Example:** Raw value 155 = 15.5V

**Battery Level Calculation:**
```
BATTERY_VOLTAGE_MIN = 13.5V  (0%)
BATTERY_VOLTAGE_MAX = 15.5V  (100%)
percentage = (voltage - 13.5) / (15.5 - 13.5) * 100
```

**Code Reference:** [`constants.rs`](constants.rs) lines 91-92

### Bytes 0x10-0x11: Left Wheel Encoder

- **Type:** u16 little-endian
- **Description:** Encoder tick count for left wheel
- **Implementation:** Only lower 2 bytes are read by [`gd32/reader.rs`](gd32/reader.rs)
- **Note:** Bytes 0x12-0x17 may contain upper bytes of a larger counter, but are currently unused

**Code Reference:** [`constants.rs`](constants.rs) line 67

### Bytes 0x18-0x19: Right Wheel Encoder

- **Type:** u16 little-endian
- **Description:** Encoder tick count for right wheel
- **Implementation:** Only lower 2 bytes are read by [`gd32/reader.rs`](gd32/reader.rs)
- **Note:** Bytes 0x1A-0x1F may contain upper bytes of a larger counter, but are currently unused

**Code Reference:** [`constants.rs`](constants.rs) line 68

### IMU Data (0x28-0x33)

The IMU data is interleaved: Gyro-Accel pairs for each axis.

**Raw Hardware Axis Mapping (Before Transform):**
- Gyro at 0x28 = Hardware "X" (actually Yaw rate - most active during flat rotation)
- Gyro at 0x2C = Hardware "Y" (actually Pitch rate - most active during nose up/down)
- Gyro at 0x30 = Hardware "Z" (actually Roll rate - most active during left/right tilt)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0x28-0x29 | 2 | Gyro X (raw) | i16 LE - Angular velocity (Yaw in hardware coords) |
| 0x2A-0x2B | 2 | Accel X | i16 LE - Linear acceleration X |
| 0x2C-0x2D | 2 | Gyro Y (raw) | i16 LE - Angular velocity (Pitch in hardware coords) |
| 0x2E-0x2F | 2 | Accel Y | i16 LE - Linear acceleration Y |
| 0x30-0x31 | 2 | Gyro Z (raw) | i16 LE - Angular velocity (Roll in hardware coords) |
| 0x32-0x33 | 2 | Accel Z | i16 LE - Linear acceleration Z |

**Coordinate Frame Transform:**

SangamIO applies a configurable axis transform (via `sangamio.toml`) to convert raw hardware axes to **ROS REP-103** standard:

```toml
[device.hardware.frame_transforms.imu_gyro]
x = [2, 1]   # output_x (Roll) = input_z * 1
y = [1, 1]   # output_y (Pitch) = input_y * 1
z = [0, -1]  # output_z (Yaw) = input_x * -1 (sign flip)
```

**Output (After Transform):**
- `gyro_x` = Roll rate (X axis rotation, left/right tilt)
- `gyro_y` = Pitch rate (Y axis rotation, nose up/down)
- `gyro_z` = Yaw rate (Z axis rotation, CCW positive)

**Code Reference:** [`constants.rs`](constants.rs) lines 73-88, [`gd32/reader.rs`](gd32/reader.rs) lines 313-323

### LP-Filtered Tilt Vector (0x34-0x39)

Low-pass filtered gravity vector for tilt correction. Useful for determining robot orientation relative to gravity.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0x34-0x35 | 2 | Tilt X | i16 LE - LP gravity vector X |
| 0x36-0x37 | 2 | Tilt Y | i16 LE - LP gravity vector Y |
| 0x38-0x39 | 2 | Tilt Z | i16 LE - LP gravity vector Z |

**Code Reference:** [`constants.rs`](constants.rs) lines 85-88, [`gd32/reader.rs`](gd32/reader.rs) lines 334-355

### Bytes 0x3A-0x3B: Start Button

- **Type:** u16 little-endian
- **Description:** Start button press state
- **Implementation:** Read as u16 from bytes 0x3A-0x3B

**Code Reference:** [`constants.rs`](constants.rs) line 70, [`gd32/reader.rs`](gd32/reader.rs) lines 223-229

### Bytes 0x3E-0x3F: Dock Button

- **Type:** u16 little-endian
- **Description:** Dock/home button press state
- **Implementation:** Read as u16 from bytes 0x3E-0x3F

**Code Reference:** [`constants.rs`](constants.rs) line 71, [`gd32/reader.rs`](gd32/reader.rs) lines 230-236

### Byte 0x46: Water Tank Level

- **Type:** u8
- **Range:** 0-100
- **Description:** Water tank fill level for 2-in-1 mop box
- **Values:**
  - 0 = Empty or pump off
  - 100 = Full / water present
- **Note:** Only relevant when 2-in-1 mop box is attached

---

## Visual Byte Layout

```
Offset:  00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
         ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ──
    0x00  ?? BP ?? CF DB ?? ?? CH BV ?? ?? ?? ?? ?? ?? ??
    0x10  WL WL ?? ?? ?? ?? ?? ?? WR WR ?? ?? ?? ?? ?? ??
    0x20  ?? ?? ?? ?? ?? ?? ?? ?? GX GX AX AX GY GY AY AY
    0x30  GZ GZ AZ AZ TX TX TY TY TZ TZ SB SB ?? ?? HB HB
    0x40  ?? ?? ?? ?? ?? ?? WT ?? ?? ?? ?? ?? ?? ?? ?? ??
    0x50  ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ??

Legend:
  BP = Bumper Flags (1 byte)     CF = Cliff Flags (1 byte)
  DB = Dustbox Flags (1 byte)    CH = Charging Flags (1 byte)
  BV = Battery Voltage (1 byte)  WT = Water Tank Level (1 byte)
  WL = Left Wheel Encoder (2 bytes, u16 LE)
  WR = Right Wheel Encoder (2 bytes, u16 LE)
  GX/GY/GZ = Gyroscope (2 bytes each, i16 LE)
  AX/AY/AZ = Accelerometer (2 bytes each, i16 LE)
  TX/TY/TZ = Tilt Vector (2 bytes each, i16 LE)
  SB = Start Button (2 bytes, u16 LE)
  HB = Home/Dock Button (2 bytes, u16 LE)
  ?? = Unknown/Reserved
```

---

## Implementation Notes

### Minimum Payload Size

The reader requires a minimum payload size of 80 bytes (`STATUS_PAYLOAD_MIN_SIZE`). Packets smaller than this are discarded.

**Code Reference:** [`constants.rs`](constants.rs) line 59

### Data Endianness

All multi-byte fields use **little-endian** encoding:
- u16: `[low_byte, high_byte]`
- i16: `[low_byte, high_byte]` (signed)

### Actual Bytes Read by reader.rs

| Field | Offset | Bytes Read | Type |
|-------|--------|------------|------|
| Charging Flags | 0x07 | 1 | u8 (bit-field) |
| Battery Voltage | 0x08 | 1 | u8 |
| Bumper Flags | 0x01 | 1 | u8 (bit-field) |
| Cliff Flags | 0x03 | 1 | u8 (bit-field) |
| Dustbox Flags | 0x04 | 1 | u8 (bit-field) |
| Left Wheel Encoder | 0x10-0x11 | 2 | u16 LE |
| Right Wheel Encoder | 0x18-0x19 | 2 | u16 LE |
| Gyro X | 0x28-0x29 | 2 | i16 LE |
| Accel X | 0x2A-0x2B | 2 | i16 LE |
| Gyro Y | 0x2C-0x2D | 2 | i16 LE |
| Accel Y | 0x2E-0x2F | 2 | i16 LE |
| Gyro Z | 0x30-0x31 | 2 | i16 LE |
| Accel Z | 0x32-0x33 | 2 | i16 LE |
| Tilt X | 0x34-0x35 | 2 | i16 LE |
| Tilt Y | 0x36-0x37 | 2 | i16 LE |
| Tilt Z | 0x38-0x39 | 2 | i16 LE |
| Start Button | 0x3A-0x3B | 2 | u16 LE |
| Dock Button | 0x3E-0x3F | 2 | u16 LE |

### Sensor Update Frequency

- **Status Packets:** ~110 Hz (every ~9ms)
- **All sensors updated:** Per packet (real-time)
- **Encoder Resolution:** Incremental ticks (wrapping u16 counters)

---

## Unknown/Reserved Bytes

The following byte ranges have not been fully characterized:

| Offset Range | Size | Notes |
|--------------|------|-------|
| 0x00 | 1 | First byte, unknown |
| 0x02 | 1 | Between bumper and cliff |
| 0x05-0x06 | 2 | Between dustbox and charging |
| 0x09-0x0F | 7 | After battery voltage |
| 0x12-0x17 | 6 | After right encoder (may be upper bytes) |
| 0x1A-0x1F | 6 | After left encoder (may be upper bytes) |
| 0x20-0x27 | 8 | Before IMU data |
| 0x3C-0x3D | 2 | After start button |
| 0x40-0x45 | 6 | After dock button |
| 0x47-0x4F | 9 | After water tank level |
| 0x50-0x5F | 16 | End padding |

These may contain additional sensor data, error codes, or internal state. Further reverse engineering via MITM capture may reveal their purpose.

---

## Related Documentation

- [COMMANDS.md](COMMANDS.md) - GD32 command reference
- [`constants.rs`](constants.rs) - Offset and flag constants
- [`gd32/reader.rs`](gd32/reader.rs) - Status packet parsing logic
- [`gd32/protocol.rs`](gd32/protocol.rs) - Packet framing and CRC

---

## Changelog

- **2024-12-15:** Moved to `src/devices/crl200s/` directory, updated relative paths
- **2024-12-07:** Added coordinate frame transform documentation for IMU data (ROS REP-103 conversion)
- **2024-11-30:** Fixed encoder and button sizes to match actual reader.rs implementation (u16, not u64/u32)
- **2024-11-30:** Initial documentation based on reverse-engineered constants and MITM captures
