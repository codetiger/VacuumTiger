# SangamIO Examples

Working examples demonstrating how to use SangamIO.

## test_all_components

Comprehensive integration test for all robot components: GD32 motor controller, Delta-2D Lidar, wheel motors, brushes, vacuum, and sensors.

### What It Does

This example performs a complete 15-step test sequence:

1. **Initialize GD32 motor controller** (automatic wake-up and heartbeat)
2. **Power on Lidar motor** via GD32 (complete AuxCtrl sequence: mode switch, prep, power, PWM)
3. **Initialize Delta-2D Lidar** driver
4. **Start Lidar scanning**
5. **Read Lidar data** for 5 seconds (logs scan statistics)
6. **Stop Lidar scanning**
7. **Test forward movement** (6 inches at slow speed with encoder readout)
8. **Test clockwise rotation** (360° in place, 3 seconds)
9. **Test anticlockwise rotation** (360° in place, 3 seconds)
10. **Test suction motor** (100% power for 2 seconds)
11. **Test rolling brush** (100% speed for 2 seconds)
12. **Test edge/side brush** (100% speed for 2 seconds)
13. **Read all sensor data** (battery, encoders, errors, status flags, IR sensors, packet stats)
14. **Power off Lidar motor** (PWM to 0%, then disable power)
15. **Clean shutdown** (automatic cleanup via Drop)

### Building

**For ARM target (robot):**
```bash
cd sangam-io
cargo build --example test_all_components --release \
    --target armv7-unknown-linux-musleabihf \
    --features="std,gd32,lidar"
```

**For local testing (without hardware):**
```bash
cargo build --example test_all_components --features="std,gd32,lidar"
```

Binary location: `target/armv7-unknown-linux-musleabihf/release/examples/test_all_components`

### Running on Hardware

**IMPORTANT**: The robot's monitoring system will auto-restart AuxCtrl. You must rename the binary to prevent this.

**Method 1: Deploy and run manually**
```bash
# 1. Build
cd sangam-io
cargo build --release --example test_all_components \
    --target armv7-unknown-linux-musleabihf \
    --features="std,gd32,lidar"

# 2. Deploy (cat method - scp doesn't work)
cat ../target/armv7-unknown-linux-musleabihf/release/examples/test_all_components | \
  sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "cat > /tmp/test && chmod +x /tmp/test"

# 3. Disable AuxCtrl (rename to prevent auto-restart)
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum \
  "mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && killall -9 AuxCtrl"

# 4. Run test with logging
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "RUST_LOG=debug /tmp/test"

# 5. CRITICAL: Restore AuxCtrl
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum \
  "mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl"
```

**Method 2: One-command test (with auto-restore)**
```bash
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && \
  killall -9 AuxCtrl 2>/dev/null; \
  RUST_LOG=debug /tmp/test; \
  EXIT_CODE=\$?; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl; \
  exit \$EXIT_CODE
"
```

### Expected Output

When running successfully, you'll see output like:

```
=== SangamIO Comprehensive Component Test ===

Step 1: Initializing GD32 motor controller...
✓ GD32 initialized successfully
  - Heartbeat running at 20ms intervals
  - Initialization sequence completed

Waiting 1.4 seconds for motor controller stabilization...

Step 2: Powering on Lidar motor...
✓ GD32 switched to navigation mode (CMD=0x65 mode=0x02)
✓ Lidar prep command sent (CMD=0xA2)
✓ Lidar power enabled (CMD=0x97)
✓ Lidar PWM set to 100% (CMD=0x71)
  - Waiting 2 seconds for motor spin-up...

Step 3: Initializing Delta-2D Lidar...
✓ Lidar driver initialized

Step 4: Starting Lidar scan...
✓ Lidar scanning started

Step 5: Reading Lidar data for 5 seconds...
Scan #1: 360 points, timestamp: Some(1234567890)
  Sample points:
    Point 1: angle=0.00°, distance=2.350m, quality=255
    Point 2: angle=1.00°, distance=2.348m, quality=254
    ...
Scan #2: 360 points, timestamp: Some(1234567891)
...

✓ Scan complete:
  - Total scans received: 25
  - Total points: 9000
  - Average points per scan: 360

Step 6: Stopping Lidar scan...
✓ Lidar scanning stopped

TESTING: Staying in mode 0x02 (navigation mode) for all components
✓ GD32 remaining in navigation mode (CMD=0x65 mode=0x02)

Step 7: Testing forward movement (6 inches)...
  - Both wheels forward at 2000 ticks (slow speed)
  - Duration: 1 second
  - Starting encoders: left=0, right=0
  - Ending encoders: left=2000, right=2000
  - Delta: left=2000, right=2000
✓ Forward movement complete

Step 8: Testing rotation - 360° clockwise (in place)...
  - Left wheel forward, right wheel backward
  - Speed: 3000 ticks for 3 seconds
  - Starting encoders: left=2000, right=2000
  - Ending encoders: left=5000, right=-1000
  - Delta: left=3000, right=-3000
  ✓ Both wheels rotated!
✓ Clockwise rotation complete

Step 9: Testing rotation - 360° anticlockwise (in place)...
  - Left wheel backward, right wheel forward
  - Speed: 3000 ticks for 3 seconds
  - Starting encoders: left=5000, right=-1000
  - Ending encoders: left=2000, right=2000
  - Delta: left=-3000, right=3000
  ✓ Both wheels rotated!
✓ Anticlockwise rotation complete

Step 10: Testing suction motor...
  - Turning ON suction at 100% power
  - Turning OFF suction
✓ Suction motor test complete

Step 11: Testing rolling brush...
  - Turning ON rolling brush at 100% speed
  - Turning OFF rolling brush
✓ Rolling brush test complete

Step 12: Testing edge/side brush...
  - Turning ON side brush at 100% speed
  - Turning OFF side brush
✓ Side brush test complete

Step 13: Reading all sensor data from GD32...

Battery Status:
  - Voltage: 14.80 V
  - Current: 1.23 A
  - Level: 85%

Encoder Counts:
  - Left wheel: 2000 ticks
  - Right wheel: 2000 ticks

Error Status:
  - Error code: 0x00
  - Status: No errors

Status Flags:
  - Status flag: 0x01
  - Charging: false
  - Battery state flag: 2
  - Percent value: 85.00

IR Proximity Sensors:
  - IR sensor 1: 123
  - Point button IR: 45
  - Dock button IR: 67

Communication Statistics:
  - Packets received: 850
  - Packets transmitted: 425
  - Lost packets: 2

NOTE: 24 of 88 STATUS_DATA bytes decoded (27% coverage)
      Remaining fields: IMU data, cliff sensors, bumpers, wall sensor
      See source code in src/devices/gd32/protocol.rs for field mapping details

Step 14: Powering off Lidar motor...
✓ Lidar PWM set to 0% (CMD=0x71)
✓ Lidar power disabled (CMD=0x97)

Step 15: Clean exit...
  - GD32 heartbeat thread will stop
  - Motors will stop

=== All component tests completed successfully ===
```

### Physical Observations

When the example runs, you should observe:

**Lidar (Steps 2-6)**:
- Lidar motor spins up (audible)
- Red laser visible during scanning
- Motor stops after Step 14

**Movement (Steps 7-9)**:
- Robot moves forward ~6 inches (Step 7)
- Robot rotates clockwise in place (Step 8)
- Robot rotates anticlockwise in place (Step 9)
- **CRITICAL**: Put robot on blocks or in open space!

**Vacuum & Brushes (Steps 10-12)**:
- Suction motor runs for 2 seconds (audible)
- Main rolling brush spins for 2 seconds
- Side brush spins for 2 seconds

**No errors**:
- No beeps or error indicators
- All steps complete successfully

### Safety Warnings

⚠️ **IMPORTANT SAFETY CONSIDERATIONS**:

1. **Clear Space**: Run in open area away from obstacles and ledges
2. **Elevated Testing**: Consider putting robot on blocks for wheel tests
3. **Emergency Stop**: Keep physical power disconnect accessible
4. **Low Speeds**: Example uses slow speeds (2000-3000 ticks) for safety
5. **Monitor Battery**: Low battery causes erratic behavior
6. **Never Leave Unattended**: Always supervise hardware tests

### Code Walkthrough

The example demonstrates key SangamIO patterns:

**Automatic Initialization:**
```rust
let gd32_transport = SerialTransport::open("/dev/ttyS3", 115200)?;
let mut gd32 = Gd32Driver::new(gd32_transport)?;
// Heartbeat thread starts automatically!
```

**Complete Lidar Power-On Sequence:**
```rust
// CRITICAL: Complete AuxCtrl-verified sequence
gd32.set_motor_mode(0x02)?;          // Navigation mode
gd32.send_lidar_prep()?;             // Prep command
gd32.set_lidar_power(true)?;         // Enable GPIO 233
gd32.set_lidar_pwm(100)?;            // Set motor speed
```

**Motor Control:**
```rust
// Forward movement
gd32.set_raw_motor_speeds(2000, 2000)?;

// Clockwise rotation (in place)
gd32.set_raw_motor_speeds(3000, -3000)?;

// Read encoders
let (left, right) = gd32.get_encoder_counts();
```

**Actuator Control:**
```rust
gd32.set_vacuum(100)?;      // Suction: 0-100%
gd32.set_main_brush(100)?;  // Main brush: 0-100%
gd32.set_side_brush(100)?;  // Side brush: 0-100%
```

**Sensor Reading:**
```rust
let (voltage, current, level) = gd32.get_battery_info();
let (left_enc, right_enc) = gd32.get_encoder_counts();
let error_code = gd32.get_error_code();
let (status_flag, charging, battery_state, percent_val) = gd32.get_status_flags();
let (ir1, point_btn, dock_btn) = gd32.get_ir_sensors();
let (rx_packets, tx_packets, lost_packets) = gd32.get_packet_stats();
```

**Lidar Scanning:**
```rust
let lidar_transport = SerialTransport::open("/dev/ttyS1", 115200)?;
let mut lidar = Delta2DDriver::new(lidar_transport)?;
lidar.start()?;

while let Ok(Some(scan)) = lidar.get_scan() {
    println!("Got {} points", scan.points.len());
    for point in &scan.points {
        println!("  angle={:.2}°, distance={:.3}m, quality={}",
            point.angle.to_degrees(), point.distance, point.quality);
    }
}
```

**Automatic Cleanup:**
```rust
// Drop trait automatically:
// - Stops heartbeat thread
// - Stops motors
// - Cleans up resources
drop(gd32);
drop(lidar);
```

### Serial Port Configuration

- **GD32 Motor Controller**: `/dev/ttyS3` @ 115200 baud
- **Delta-2D Lidar**: `/dev/ttyS1` @ 115200 baud

Both ports must be available (AuxCtrl must be stopped).

### Using as a Template

Copy this example as a starting point for your robot application:

```bash
cp examples/test_all_components.rs examples/my_robot.rs
```

Then modify to implement your robot logic. Key sections to customize:

1. **Motion Control** (Steps 7-9): Replace with your navigation logic
2. **Lidar Processing** (Step 5): Add obstacle detection, mapping, etc.
3. **Sensor Monitoring** (Step 13): Use sensor data for decision making
4. **Actuator Control** (Steps 10-12): Integrate cleaning patterns

### Protocol Insights

This example implements the **verified AuxCtrl initialization sequence** discovered via MITM analysis:

1. **CMD=0x65 mode=0x02**: Switch to navigation mode (enables lidar control)
2. **CMD=0xA2**: Lidar preparation command
3. **CMD=0x97**: Enable lidar power (GPIO 233)
4. **CMD=0x71**: Set lidar PWM speed

**Without this exact sequence, the lidar will not work!**

See `src/devices/gd32/protocol.rs` in the source code for complete protocol specification.

### Sensor Coverage

Currently decoding **24 of 88 bytes** (27%) from the GD32 STATUS_DATA packet:

**Decoded:**
- Battery voltage, current, percentage
- Wheel encoder counts (left/right)
- Error codes
- Status flags (charging, battery state)
- IR proximity sensors (3 sensors)
- Communication statistics

**Not Yet Decoded:**
- IMU data (accelerometer, gyroscope)
- Cliff sensors (4 sensors)
- Bumpers (front/side)
- Wall-following sensor
- Additional actuator feedback

See `parse_status_packet()` function in `src/devices/gd32/protocol.rs` for implementation details of decoded fields.

## Adding More Examples

To create a new example:

1. Create `examples/your_example.rs`
2. Add to `Cargo.toml`:
   ```toml
   [[example]]
   name = "your_example"
   required-features = ["std", "gd32", "lidar"]
   ```
3. Build: `cargo build --example your_example --features="std,gd32,lidar"`

## Common Issues

**If the example doesn't run**, see the [Troubleshooting](../GUIDE.md#troubleshooting) section in GUIDE.md.

**Quick checks**:
- Is AuxCtrl stopped **and renamed**? `ssh root@vacuum "ps | grep AuxCtrl"`
- Do serial ports exist? `ssh root@vacuum "ls -l /dev/ttyS*"`
- Is the robot powered on and battery charged?
- Is robot connected to WiFi/network?

**Common problems**:
- **Robot doesn't move**: Check battery level, verify AuxCtrl is stopped
- **Lidar doesn't spin**: Verify complete power-on sequence was executed
- **Serial port busy**: Another process has the port open, restart robot
- **Robot beeps error**: Low battery or hardware fault, check error code

## Learn More

- **[GUIDE.md](../GUIDE.md)** - Complete deployment and development guide
- **[REFERENCE.md](../REFERENCE.md)** - Architecture and API documentation
- **Protocol Specification**: See source code comments in `src/devices/gd32/protocol.rs`
- **API Docs**: Run `cargo doc --open` in sangam-io directory
