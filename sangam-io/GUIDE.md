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

# Build the test example
cargo build --example test_all_components --release \
    --target armv7-unknown-linux-musleabihf \
    --features="std,gd32,lidar"
```

The binary will be at `target/armv7-unknown-linux-musleabihf/release/examples/test_all_components` (~500KB).

### Step 2: Stop Original Firmware

SangamIO needs exclusive access to the serial ports:

```bash
ssh root@vacuum "killall -9 AuxCtrl"
```

⚠️ **Warning**: The robot won't respond to its app until you reboot or restart AuxCtrl.

### Step 3: Deploy to Robot

```bash
# One-line deploy (replace $ROBOT_PASSWORD with your robot's root password)
cat target/armv7-unknown-linux-musleabihf/release/examples/test_all_components | \
  sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum \
  "cat > /tmp/test && chmod +x /tmp/test"
```

**Alternative** (if you have SSH keys set up):
```bash
scp target/armv7-unknown-linux-musleabihf/release/examples/test_all_components \
    root@vacuum:/tmp/test
ssh root@vacuum "chmod +x /tmp/test"
```

### Step 4: Run the Test

```bash
ssh root@vacuum "/tmp/test"
```

You should see the 15-step test scenario execute. Check [examples/README.md](examples/README.md#expected-output) for the full expected output.

**What to observe**:
- ✅ GD32 initializes within 5 seconds
- ✅ Lidar motor spins up (you'll hear it)
- ✅ Lidar emits red laser light
- ✅ Scan data appears in console
- ✅ Clean shutdown with no errors

### Step 5: Verify It Worked

If successful, you'll see:
```
=== SangamIO Comprehensive Component Test ===
Step 1: Initializing GD32 motor controller...
✓ GD32 initialized successfully
...
=== All component tests completed successfully ===
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
vim src/devices/gd32/mod.rs

# Build, deploy, and run in one command
cargo build --example test_all_components --release \
  --target armv7-unknown-linux-musleabihf --features="std,gd32,lidar" && \
  cat target/armv7-unknown-linux-musleabihf/release/examples/test_all_components | \
  sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum \
  "killall -9 AuxCtrl 2>/dev/null; cat > /tmp/test && chmod +x /tmp/test && /tmp/test"
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
cp examples/test_all_components.rs examples/my_robot.rs
```

Add it to `Cargo.toml`:
```toml
[[example]]
name = "my_robot"
required-features = ["std", "gd32", "lidar"]
```

### Basic Robot Control

```rust
use sangam_io::devices::{Gd32Driver, Delta2DDriver};
use sangam_io::drivers::{MotorDriver, LidarDriver};
use sangam_io::transport::SerialTransport;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize motor controller
    let gd32_transport = SerialTransport::open("/dev/ttyS3", 115200)?;
    let mut gd32 = Gd32Driver::new(gd32_transport)?;

    // Move forward at 0.5 m/s
    gd32.set_velocity(0.5, 0.0)?;

    // Get odometry
    let odom = gd32.get_odometry()?;
    println!("Position: ({:.2}, {:.2})", odom.x, odom.y);

    // Power on lidar and scan
    gd32.set_lidar_power(true)?;
    let lidar_transport = SerialTransport::open("/dev/ttyS1", 115200)?;
    let mut lidar = Delta2DDriver::new(lidar_transport)?;
    lidar.start()?;

    if let Ok(Some(scan)) = lidar.get_scan() {
        println!("Got {} points", scan.points.len());
    }

    Ok(())
}
```

### Build and Deploy

```bash
cargo build --example my_robot --release --features="std,gd32,lidar"
# Deploy using same method as test scenario
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

Before production use, you'll need to calibrate these constants in `src/devices/gd32/mod.rs`:

**Wheel geometry** (in `set_velocity()`):
```rust
const WHEEL_BASE: f32 = 0.3;      // Measure your robot's wheel base (meters)
const WHEEL_RADIUS: f32 = 0.05;   // Measure wheel radius (meters)
```

**Encoder conversion** (in `set_wheel_velocity()` and `get_odometry()`):
```rust
const TICKS_PER_RADIAN: f32 = 100.0;        // Calibrate by testing
const TICKS_PER_REVOLUTION: f32 = 1000.0;   // Calibrate by testing
```

**Calibration procedure**:
1. Command robot to move 1 meter forward
2. Measure actual distance traveled
3. Calculate correction factor
4. Update constants and repeat until accurate

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
