# SangamIO Examples

Working examples demonstrating how to use SangamIO.

## test_lidar_scenario

Complete integration test for GD32 motor controller and Delta-2D Lidar.

### What It Does

1. Initialize GD32 motor controller (automatic wake-up and heartbeat)
2. Initialize Delta-2D Lidar driver
3. Power on Lidar motor via GD32 (CMD=0x97)
4. Read and log Lidar scan data for 5 seconds
5. Power off Lidar motor
6. Clean shutdown (automatic cleanup)

### Building

```bash
cargo build --example test_lidar_scenario --release \
    --features="std,gd32,lidar"
```

Binary location: `target/armv7-unknown-linux-musleabihf/release/examples/test_lidar_scenario`

### Running on Hardware

```bash
# Deploy and run
cat target/armv7-unknown-linux-musleabihf/release/examples/test_lidar_scenario | \
  sshpass -p "<password>" ssh root@vacuum \
  "killall -9 AuxCtrl 2>/dev/null; cat > /tmp/test && chmod +x /tmp/test && /tmp/test"
```

### Expected Output

When running successfully, you'll see:

```
=== SangamIO Lidar Test Scenario ===

Step 1: Initializing GD32 motor controller...
GD32: Starting initialization sequence
GD32: Sent initialization command
GD32: Device initialized successfully
GD32: Sending wake command
GD32: Setting control mode
GD32: Starting heartbeat thread
GD32: Heartbeat thread started
✓ GD32 initialized successfully
  - Heartbeat running at 20ms intervals

Step 2: Initializing Delta-2D Lidar...
Delta2D: Driver initialized
✓ Lidar driver initialized

Step 3: Powering on Lidar motor...
GD32: Setting lidar power: true
✓ Lidar motor powered on
  - Waiting 2 seconds for motor spin-up...

Step 4: Starting Lidar scan...
✓ Lidar scanning started

Step 5: Reading Lidar data for 5 seconds...
Scan #1: 360 points, timestamp: Some(1730293812345)
  Sample points:
    Point 1: angle=0.00°, distance=2.350m, quality=255
    Point 2: angle=1.00°, distance=2.348m, quality=254
    Point 3: angle=2.00°, distance=2.345m, quality=253
    Point 4: angle=3.00°, distance=2.342m, quality=252
    Point 5: angle=4.00°, distance=2.340m, quality=251
Scan #2: 360 points, timestamp: Some(1730293812545)
Scan #3: 360 points, timestamp: Some(1730293812745)
Scan #4: 360 points, timestamp: Some(1730293812945)
Scan #5: 360 points, timestamp: Some(1730293813145)
...

✓ Scan complete:
  - Total scans received: 25
  - Total points: 9000
  - Average points per scan: 360

Step 6: Stopping Lidar scan...
✓ Lidar scanning stopped

Step 7: Powering off Lidar motor...
GD32: Setting lidar power: false
✓ Lidar motor powered off

Step 8: Clean exit...
  - GD32 heartbeat thread will stop
  - Motors will stop

GD32: Shutting down driver
GD32: Heartbeat thread shutting down
GD32: Heartbeat thread stopped
GD32: Driver shutdown complete
=== Test scenario completed successfully ===
```

### Physical Observations

When the example runs, you should observe:

- **Lidar motor**: Spins up after Step 3 (you'll hear it)
- **Red laser**: Visible spinning pattern during Step 5
- **GD32 LEDs**: May blink (device-specific)
- **No error sounds**: No beeps or error indicators

### Code Walkthrough

The example demonstrates key SangamIO patterns:

```rust
// Automatic initialization
let gd32_transport = SerialTransport::open("/dev/ttyS3", 115200)?;
let mut gd32 = Gd32Driver::new(gd32_transport)?;
// Heartbeat thread starts automatically!

// Control peripherals
gd32.set_lidar_power(true)?;

// Read sensor data
let lidar_transport = SerialTransport::open("/dev/ttyS2", 115200)?;
let mut lidar = Delta2DDriver::new(lidar_transport)?;
lidar.start()?;

while let Ok(Some(scan)) = lidar.get_scan() {
    println!("Got {} points", scan.points.len());
}

// Automatic cleanup via Drop trait
// Heartbeat stops, motors stop, everything cleaned up
```

### Using as a Template

Copy this example as a starting point for your robot application:

```bash
cp examples/test_lidar_scenario.rs examples/my_robot.rs
```

Then modify the main loop to implement your robot logic.

## Adding More Examples

To create a new example:

1. Create `examples/your_example.rs`
2. Add to `Cargo.toml`:
   ```toml
   [[example]]
   name = "your_example"
   required-features = ["std", "gd32"]
   ```
3. Build: `cargo build --example your_example --features="..."`

## Common Issues

**If the example doesn't run**, see the [Troubleshooting](../GUIDE.md#troubleshooting) section in GUIDE.md.

**Quick checks**:
- Is AuxCtrl stopped? `ssh root@vacuum "ps | grep AuxCtrl"`
- Do serial ports exist? `ssh root@vacuum "ls -l /dev/ttyS{2,3}"`
- Is the robot powered on and connected to WiFi?

## Learn More

- **[GUIDE.md](../GUIDE.md)** - Complete deployment guide
- **[REFERENCE.md](../REFERENCE.md)** - Architecture and API documentation
- **API Docs**: Run `cargo doc --open`
