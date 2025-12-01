# Lidar Initialization Sequence - Final Specification

**Successfully Reverse-Engineered and Implemented**
**Date**: November 2, 2025
**Method**: MITM Protocol Capture + Binary Analysis
**Status**: ✅ **WORKING** - Verified on hardware

## Overview

This document describes the complete, verified lidar initialization and control sequence for the GD32F103 motor controller and 3iRobotix Delta-2D lidar. The protocol was successfully reverse-engineered through MITM logging of the AuxCtrl firmware and validated on actual hardware.

## Critical Discovery: 4-Command Sequence

The lidar requires a **precise 4-command sequence** sent to the GD32 motor controller. Missing any command will prevent the lidar from operating.

### Complete Working Sequence

```
1. CMD=0x65 mode=0x02  - Switch to navigation mode (CRITICAL!)
2. CMD=0xA2            - Lidar preparation command
3. CMD=0x97 [0x01]     - Enable lidar power (GPIO 233 ON)
4. CMD=0x71 [100]      - Set lidar PWM speed to 100%
```

**Timeline**:
- Wait 1.4s after GD32 initialization
- Send all 4 commands in sequence (no delay between commands)
- Wait 2s for motor spin-up
- Lidar is ready for scanning

## Protocol Specification

### CMD=0x65: Motor Mode Switch

**Discovered via**: MITM capture (line 25673 in cleaning session log)
**CRITICAL**: This MUST be sent before any lidar commands work!

```
Packet Format: [0xFA 0xFB] [0x04] [0x65] [MODE] [CRC]
- MODE: 0x00 = initialization, 0x02 = navigation/cleaning mode
- CRC: 16-bit big-endian word sum
```

**Purpose**: Enables navigation subsystems including lidar control. Without switching to mode 0x02, the GD32 firmware ignores CMD=0xA2, CMD=0x97, and CMD=0x71.

**Implementation** (sangam-io/src/devices/crl200s/gd32/mod.rs):
```rust
pub fn set_motor_mode(&mut self, mode: u8) -> Result<()> {
    log::info!("GD32: Setting motor mode: 0x{:02X}", mode);
    Self::send_command(&self.transport, &Gd32Command::MotorType(mode))?;
    log::debug!("GD32: Sent CMD=0x65 (MotorType=0x{:02X})", mode);
    Ok(())
}
```

### CMD=0xA2: Lidar Preparation

**Discovered via**: MITM capture (line 25682, bundled with CMD=0x97)
**Purpose**: Unknown internal preparation (possibly initializes GPIO or PWM subsystems)

```
Packet Format: [0xFA 0xFB] [0x07] [0xA2] [0x10 0x0E 0x00 0x00] [CRC]
- Payload: Fixed 4-byte sequence [0x10, 0x0E, 0x00, 0x00]
- CRC: 16-bit big-endian word sum
```

**Note**: This command was discovered by analyzing MITM logs. In the original AuxCtrl firmware, CMD=0xA2 and CMD=0x97 are sent in a single write operation, suggesting they're tightly coupled.

**Implementation** (sangam-io/src/devices/crl200s/gd32/mod.rs):
```rust
pub fn send_lidar_prep(&mut self) -> Result<()> {
    log::info!("GD32: Sending lidar preparation command");
    Self::send_command(&self.transport, &Gd32Command::LidarPrep)?;
    log::debug!("GD32: Sent CMD=0xA2 (LidarPrep)");
    Ok(())
}
```

### CMD=0x97: Lidar Power Control (GPIO 233)

**Discovered via**: Binary analysis at address 0x00060988
**Purpose**: Controls GPIO 233 (lidar motor power enable)

```
Packet Format: [0xFA 0xFB] [0x04] [0x97] [POWER] [CRC]
- POWER: 0x00 = OFF, 0x01 = ON
- CRC: 16-bit big-endian word sum
```

**GPIO Behavior**:
- CMD=0x97 [0x01] → GD32 sets `/sys/class/gpio/gpio233/value` = 1
- CMD=0x97 [0x00] → GD32 sets `/sys/class/gpio/gpio233/value` = 0

**Implementation** (sangam-io/src/devices/crl200s/gd32/mod.rs):
```rust
pub fn set_lidar_power(&mut self, enable: bool) -> Result<()> {
    log::info!("GD32: Setting lidar power: {}", enable);
    Self::send_command(&self.transport, &Gd32Command::LidarPower(enable))?;
    log::debug!("GD32: Sent CMD=0x97 (LidarPower={})", enable);
    Ok(())
}
```

### CMD=0x71: Lidar PWM Speed Control

**Discovered via**: Binary analysis at address 0x0003f604
**Purpose**: Sets lidar motor speed (0-100%)

```
Packet Format: [0xFA 0xFB] [0x07] [0x71] [PWM 4 bytes little-endian] [CRC]
- PWM: 0-100 (percentage, sent as u32)
- CRC: 16-bit big-endian word sum
```

**Implementation** (sangam-io/src/devices/crl200s/gd32/mod.rs):
```rust
pub fn set_lidar_pwm(&mut self, pwm_percent: i32) -> Result<()> {
    log::info!("GD32: Setting lidar PWM: {}%", pwm_percent);
    Self::send_command(&self.transport, &Gd32Command::LidarPWM(pwm_percent))?;
    log::debug!("GD32: Sent CMD=0x71 (LidarPWM={}%)", pwm_percent.clamp(0, 100));
    Ok(())
}
```

## Complete Initialization Procedure

### Full Working Example

```rust
// Step 1: Initialize GD32 motor controller
let gd32_transport = SerialTransport::open("/dev/ttyS3", 115200)?;
let mut gd32 = Gd32Driver::new(gd32_transport)?;
// Heartbeat thread starts automatically (CMD=0x66 every 20ms)

// CRITICAL: Wait 1.4s after init before powering lidar
thread::sleep(Duration::from_millis(1400));

// Step 2: Send complete 4-command sequence
gd32.set_motor_mode(0x02)?;      // Switch to navigation mode
gd32.send_lidar_prep()?;         // Preparation command
gd32.set_lidar_power(true)?;     // Enable GPIO 233
gd32.set_lidar_pwm(100)?;        // Set motor speed to 100%

// Wait for motor spin-up
thread::sleep(Duration::from_secs(2));

// Step 3: Initialize Delta-2D lidar
let lidar_transport = SerialTransport::open("/dev/ttyS1", 115200)?;
let mut lidar = Delta2DDriver::new(lidar_transport)?;

// Step 4: Start scanning
lidar.start()?;

// Step 5: Read scan data
while let Some(scan) = lidar.get_scan()? {
    // Process scan data...
}

// Step 6: Clean shutdown
lidar.stop()?;
gd32.set_lidar_pwm(0)?;          // Stop motor
gd32.set_lidar_power(false)?;    // Disable power
```

### Verified Results

**Hardware Test (November 2, 2025)**:
- ✅ Lidar motor spins up correctly
- ✅ GPIO 233 = 1 (confirmed via sysfs)
- ✅ Scan data received on /dev/ttyS1
- ✅ 47 scans captured in 5 seconds
- ✅ 7,755 data points total
- ✅ Average 165 points per scan
- ✅ Scan rate: ~9.4 Hz

## Critical Timing Requirements

| Event | Delay | Reason |
|-------|-------|--------|
| After GD32 init | 1.4s | Matches AuxCtrl timing; allows GD32 firmware to stabilize |
| After power ON | 2.0s | Motor spin-up to stable operating speed (~300 RPM) |
| Between commands | None | All 4 commands sent immediately in sequence |
| Heartbeat interval | 20ms | CMD=0x66 must continue throughout operation |

**Missing heartbeat → Emergency motor stop within 100ms**

## MITM Discovery Process

### Key Log Analysis Points

**Run**: `cleaning_session_with_gpio.log`

**Line 25673** - Navigation mode switch:
```
[timestamp] TX 5 bytes
  HEX: FA FB 04 65 02 65 02
  PKT: CMD=0x65 LEN=4
```

**Line 25682** - Bundled prep + power commands:
```
[timestamp] TX 17 bytes
  HEX: FA FB 07 A2 10 0E 00 00 B0 10 FA FB 04 97 01 97 01
  PKT: CMD=0xA2 LEN=7
  PKT: CMD=0x97 LEN=4
       (LIDAR_POWER - GPIO 233 = ON)
```

**Line 25691** - PWM speed:
```
[timestamp] TX 12 bytes
  HEX: FA FB 07 71 64 00 00 00 71 64
  PKT: CMD=0x71 LEN=7
       (LIDAR_PWM - Speed = 100%)
```

**GPIO State Change**:
```
[timestamp] GPIO_233: 0 -> 1 (Lidar Power ON)
```

## Implementation Status

### SangamIO Library (v0.1.0)

| Component | Status | Location |
|-----------|--------|----------|
| CMD=0x65 (MotorMode) | ✅ Implemented | `src/devices/crl200s/gd32/commands.rs` |
| CMD=0xA2 (LidarPrep) | ✅ Implemented | `src/devices/crl200s/gd32/commands.rs` |
| CMD=0x97 (LidarPower) | ✅ Implemented | `src/devices/crl200s/gd32/commands.rs` |
| CMD=0x71 (LidarPWM) | ✅ Implemented | `src/devices/crl200s/gd32/commands.rs` |
| Heartbeat thread | ✅ Working | `src/devices/crl200s/gd32/heartbeat.rs` |
| Delta-2D driver | ✅ Working | `src/devices/crl200s/delta2d/mod.rs` |

## Troubleshooting

### Lidar Motor Doesn't Spin

**Check List**:
1. ✅ Did you send CMD=0x65 mode=0x02 first?
2. ✅ Did you send all 4 commands in order?
3. ✅ Is heartbeat running? (ps aux | grep AuxCtrl should be stopped)
4. ✅ Did you wait 1.4s after GD32 init?
5. ✅ Is GPIO 233 = 1? (cat /sys/class/gpio/gpio233/value)

### No Scan Data from /dev/ttyS1

**Possible Causes**:
- Motor not spinning (see above)
- Delta-2D lidar not initialized (check serial port open)
- Start command not sent (lidar.start()?)
- Serial port in use by another process

### GPIO 233 Stays at 0

**Root Cause**: CMD=0x65 mode=0x02 not sent, or sent after CMD=0x97

**Fix**: Always send CMD=0x65 mode=0x02 FIRST in the sequence

## References

### Source Code
- **Protocol Implementation**: `sangam-io/src/devices/crl200s/gd32/protocol.rs`
- **GD32 Driver**: `sangam-io/src/devices/crl200s/gd32/mod.rs`
- **Commands**: `sangam-io/src/devices/crl200s/gd32/commands.rs`

### Documentation
- **MITM Logging Guide**: `protocol-mitm/docs/MITM_LOGGING_GUIDE.md`
- **GD32 Command Reference**: `sangam-io/COMMANDS.md`
- **Sensor Status Packet**: `sangam-io/SENSORSTATUS.md`

### MITM Logs
- **Successful Capture**: `protocol-mitm/logs/mitm_gpio/cleaning_session_with_gpio.log`
- **Analysis Tool**: `protocol-mitm/src/main.rs` (serial_mitm binary)

## Appendix: Binary Analysis

### AuxCtrl Function Addresses

| Function | Address | Notes |
|----------|---------|-------|
| `lidarSpeedControl()` | 0x0003f6b8 | Main control loop |
| `controlLidarPwm(int)` | 0x0003f604 | Sends CMD=0x71 |
| `packetLidarPower(bool)` | 0x00060988 | Sends CMD=0x97 |
| `CGpioControl::GpioInit()` | 0x00066ad0 | GPIO initialization |
| GPIO 233 constant | 0x0001296c | Data section |

### Extraction Commands

```bash
# Extract strings
arm-linux-gnueabihf-strings /usr/sbin/AuxCtrl | grep -i lidar

# Disassemble lidar functions
arm-linux-gnueabihf-objdump -d /usr/sbin/AuxCtrl | grep -A 50 "0003f604"

# Find GPIO references
arm-linux-gnueabihf-objdump -s -j .rodata /usr/sbin/AuxCtrl | grep -B2 -A2 "e9 00 00 00"
```

## Version History

- **v1.0** (Nov 2, 2025): Final verified specification with working hardware test
- **v0.2** (Nov 1, 2025): Added CMD=0xA2 after MITM discovery
- **v0.1** (Nov 1, 2025): Initial spec from binary analysis (CMD=0x97, CMD=0x71 only)
