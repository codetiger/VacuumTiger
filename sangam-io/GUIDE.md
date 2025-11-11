# SangamIO Guide

Complete guide to building, deploying, and developing with SangamIO on your vacuum robot.

## Prerequisites

### Development Tools

You'll need these installed on your development machine:

```bash
# Rust toolchain (1.70+)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# ARM cross-compilation target
rustup target add armv7-unknown-linux-musleabihf

# ARM GNU toolchain (macOS only)
brew tap messense/macos-cross-toolchains
brew install armv7-unknown-linux-musleabihf

# Deployment tool
brew install hudochenkov/sshpass/sshpass  # macOS
sudo apt-get install sshpass               # Linux
```

### Hardware Requirements

- **Robot**: 3irobotix CRL-200S or compatible
- **Main CPU**: Allwinner A33 running Tina Linux
- **Motor Controller**: GD32F103 on `/dev/ttyS3` @ 115200 baud
- **Lidar**: 3iRobotix Delta-2D on `/dev/ttyS1` @ 115200 baud

### Network Setup

Your robot needs SSH access. Connect via USB first:

```bash
# Enable debug mode (you have ~5 seconds after connecting)
adb shell touch /mnt/UDISK/debug_mode

# Verify it worked
adb shell ls -l /mnt/UDISK/debug_mode
```

Once debug mode is active, configure WiFi and SSH. Then you can disconnect USB and work remotely.

Add this to `~/.ssh/config` for convenience:
```
Host vacuum
    HostName 192.168.1.XXX  # Your robot's IP
    User root
```

## First Deployment

### Step 1: Build for ARM

```bash
cd sangam-io

# Build the quick demo example
cargo build --example quick_demo --release \
    --target armv7-unknown-linux-musleabihf
```

The binary will be at `target/armv7-unknown-linux-musleabihf/release/examples/quick_demo` (~500KB).

### Step 2: Stop Original Firmware

SangamIO needs exclusive access to the serial ports:

```bash
ssh root@vacuum "killall -9 AuxCtrl"
```

⚠️ **Warning**: The robot won't respond to its app until you reboot or restart AuxCtrl.

### Step 3: Deploy to Robot

```bash
# One-line deploy
cat target/armv7-unknown-linux-musleabihf/release/examples/quick_demo | \
  sshpass -p "vacuum@123" ssh root@vacuum \
  "cat > /tmp/test && chmod +x /tmp/test"
```

**Note**: The device lacks sftp-server, so we use `cat` over SSH instead of `scp`.

### Step 4: Run the Test

```bash
ssh root@vacuum "RUST_LOG=info /tmp/test"
```

You should see the hardware test sequence execute (~20 seconds).

**What to observe**:
- ✅ GD32 initializes within 5 seconds
- ✅ Lidar motor spins up (audible)
- ✅ Lidar emits red laser light
- ✅ Robot moves forward briefly
- ✅ Robot rotates clockwise then counter-clockwise
- ✅ Clean shutdown with no errors

### Step 5: Verify It Worked

If successful, you'll see:
```
INFO  quick_demo] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
INFO  quick_demo] 🤖 SangamIO Quick Hardware Demo
INFO  quick_demo] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
...
INFO  quick_demo] ✅ All tests completed successfully!
```

If something went wrong, see [Troubleshooting](#troubleshooting) below.

### Step 6: Restore Original Firmware

When you're done testing:

```bash
# Reboot to restore everything
ssh root@vacuum "reboot"

# Or manually restart AuxCtrl
ssh root@vacuum "/usr/sbin/AuxCtrl &"
```

## Development Workflow

### Iterative Development

Here's the fastest edit-build-deploy cycle:

```bash
# Edit your code
vim src/sangam.rs

# Build, deploy, and run in one command
cargo build --example quick_demo --release \
  --target armv7-unknown-linux-musleabihf && \
  cat target/armv7-unknown-linux-musleabihf/release/examples/quick_demo | \
  sshpass -p "vacuum@123" ssh root@vacuum \
  "killall -9 AuxCtrl 2>/dev/null; cat > /tmp/test && chmod +x /tmp/test && RUST_LOG=debug /tmp/test"
```

### Enable Verbose Logging

Control log output with `RUST_LOG`:

```bash
# Debug logging
ssh root@vacuum "RUST_LOG=debug /tmp/test"

# Trace logging (very verbose, includes heartbeat timing)
ssh root@vacuum "RUST_LOG=trace /tmp/test"

# Specific module logging
ssh root@vacuum "RUST_LOG=sangam_io::devices::gd32=trace /tmp/test"
```

### Monitor Serial Traffic

Watch what's happening on the serial ports:

```bash
# Monitor GD32 communication
ssh root@vacuum "cat /dev/ttyS3 | hexdump -C"

# Monitor Lidar communication
ssh root@vacuum "cat /dev/ttyS1 | hexdump -C"
```

### Check System Status

```bash
# See what's running
ssh root@vacuum "ps | grep -E '(AuxCtrl|test)'"

# Monitor kernel logs
ssh root@vacuum "dmesg | tail -20"

# Watch logs in real-time
ssh root@vacuum "dmesg -w"
```

## Building Your Own Application

### Start from the Example

```bash
# Copy the example as a template
cp examples/quick_demo.rs examples/my_robot.rs
```

Add it to `Cargo.toml`:
```toml
[[example]]
name = "my_robot"
```

### Basic Robot Control

```rust
use sangam_io::SangamIO;
use std::time::Duration;
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize hardware (CRL-200S configuration)
    let mut sangam = SangamIO::crl200s("/dev/ttyS3", "/dev/ttyS1")?;

    // Enable cleaning components
    sangam.set_blower_speed(75)?;   // Vacuum at 75%
    sangam.set_brush_speed(50)?;    // Side brush at 50%

    // Move forward at 0.3 m/s for 2 seconds
    sangam.set_velocity(0.3, 0.0)?;
    thread::sleep(Duration::from_secs(2));

    // Get odometry delta for SLAM
    let delta = sangam.get_odometry_delta()?;
    println!("Traveled: Δx={:.3}m, Δy={:.3}m", delta.delta_x, delta.delta_y);

    // Scan for obstacles
    if let Some(scan) = sangam.get_scan()? {
        println!("Scanned {} points", scan.points.len());
        // Process scan data for obstacle avoidance
    }

    // Check sensors
    if let Some(battery) = sangam.get_battery_level() {
        println!("Battery: {}%", battery);
    }

    // Stop robot
    sangam.stop()?;

    Ok(())
}
```

### Build and Deploy

```bash
cargo build --example my_robot --release --target armv7-unknown-linux-musleabihf
# Deploy using same method as quick_demo
```

## Troubleshooting

### GD32 Initialization Timeout

**Symptom**: "GD32 initialization timeout (no response after 5 seconds)"

**Common causes**:
1. Original AuxCtrl still running
2. Wrong serial port
3. GD32 not powered

**Solutions**:
```bash
# Check if AuxCtrl is running
ssh root@vacuum "ps | grep AuxCtrl"
# Kill it: ssh root@vacuum "killall -9 AuxCtrl"

# Verify serial port exists
ssh root@vacuum "ls -l /dev/ttyS3"

# Check kernel logs for errors
ssh root@vacuum "dmesg | tail"
```

### No Lidar Data

**Symptom**: "Total scans received: 0"

**Solutions**:
- Listen for the lidar motor spinning (you should hear it)
- Check if the red laser is visible
- Verify lidar serial port: `ssh root@vacuum "ls -l /dev/ttyS1"`
- Ensure lidar power command succeeded (check logs)

### Serial Port Busy

**Symptom**: "Device or resource busy"

**Solution**: Another process is using the port
```bash
# Find what's using it
ssh root@vacuum "lsof | grep ttyS"

# Kill the process
ssh root@vacuum "killall -9 <process-name>"
```

### Binary Won't Run

**Symptom**: "/tmp/test_lidar: not found"

**Cause**: Wrong architecture or missing dependencies

**Check**:
```bash
# Verify binary is ARM
file target/armv7-unknown-linux-musleabihf/release/examples/test_all_components
# Should show: ARM, EABI5, statically linked

# Check on device
ssh root@vacuum "file /tmp/test"
```

### Permission Denied

**Symptom**: "Permission denied" accessing /dev/ttyS3

**Solution**: Already root on the robot, but verify:
```bash
ssh root@vacuum "whoami"  # Should be 'root'
ssh root@vacuum "ls -l /dev/ttyS3"  # Check permissions
```

### Can't Connect to Robot

**Solutions**:
```bash
# Find robot IP
nmap -sn 192.168.1.0/24 | grep -B 2 vacuum

# Test SSH connection
ssh root@vacuum "echo 'Connected!'"

# Verify debug mode is still active
ssh root@vacuum "ls /mnt/UDISK/debug_mode"
```

## Hardware Calibration

Before production use, you may need to calibrate physical parameters in `src/config.rs`:

**Wheel geometry** (in `SangamConfig::crl200s_defaults()`):
```rust
wheel_base: 0.235,      // Distance between wheels (meters)
wheel_radius: 0.0325,   // Wheel radius (meters)
ticks_per_revolution: 1560.0,  // Encoder ticks per revolution
```

**Motion constraints**:
```rust
max_linear_velocity: 0.5,    // Maximum forward speed (m/s)
max_angular_velocity: 2.0,   // Maximum rotation speed (rad/s)
linear_acceleration: 0.3,    // Acceleration limit (m/s²)
```

**Calibration procedure**:
1. Command robot to move 1 meter forward: `sangam.move_forward(1.0)?`
2. Measure actual distance traveled
3. Adjust `wheel_radius` or `ticks_per_revolution` accordingly
4. Repeat until odometry is accurate within 2-3%

## Safety Notes

⚠️ **Before running on hardware**:

- Test in open space away from obstacles
- Have a way to quickly power off the robot
- Monitor the first few runs closely
- Start with low speeds for testing
- Check battery levels (low battery causes erratic behavior)

## Quick Reference

| Command | Purpose |
|---------|---------|
| `killall -9 AuxCtrl` | Stop original firmware |
| `ps \| grep AuxCtrl` | Check if AuxCtrl is running |
| `dmesg \| tail` | Check kernel logs |
| `ls -l /dev/ttyS3` | Verify GD32 serial port |
| `ls -l /dev/ttyS1` | Verify Lidar serial port |
| `reboot` | Restore original firmware |

## Next Steps

- **Learn the architecture**: See [REFERENCE.md](REFERENCE.md) for system design details
- **Explore the API**: Run `cargo doc --open` for complete API documentation
- **See more examples**: Check [examples/README.md](examples/README.md)
- **Add new hardware**: See [REFERENCE.md](REFERENCE.md#extending-sangamio) for extension guide

## Getting Help

- **Found a bug?** Open an issue at https://github.com/codetiger/VacuumTiger/issues
- **Need clarification?** Check [REFERENCE.md](REFERENCE.md) for technical details
- **Want to contribute?** See the architecture section in [REFERENCE.md](REFERENCE.md)
