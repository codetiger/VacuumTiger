# SangamIO Development Workflow

This document outlines the complete workflow for building, deploying, testing, and debugging the SangamIO firmware on the CRL-200S robot.

## Prerequisites

- **Host Machine**: macOS/Linux with Rust installed
- **Target Device**: CRL-200S robot (IP: 192.168.68.101, SSH: root@vacuum)
- **SSH Password**: vacuum@123
- **Cross-compilation target**: armv7-unknown-linux-musleabihf

## 1. Build Process

### Development Build (Host Testing)
```bash
cd sangam-io
cargo build
cargo test
```

### Production Build (ARM Target)
```bash
cd sangam-io
cargo build --release \
  --target armv7-unknown-linux-musleabihf \
  --example sangam_slam_demo
```

### Build Output Location
```
# The binary is built in the parent directory's target folder:
../target/armv7-unknown-linux-musleabihf/release/examples/sangam_slam_demo
```

## 2. Deployment Process

### Standard Deployment (Using cat over SSH)
The robot lacks sftp-server, so we use cat over SSH:

```bash
# Set password variable (optional)
export ROBOT_PASSWORD="vacuum@123"

# Deploy binary (from sangam-io directory)
cat ../target/armv7-unknown-linux-musleabihf/release/examples/sangam_slam_demo | \
  sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "cat > /tmp/sangam_slam_demo && chmod +x /tmp/sangam_slam_demo"
```

### Why cat over SSH?
- Device lacks sftp-server, causing standard `scp` to fail
- This method pipes the binary through SSH stdin
- More reliable than attempting scp

## 3. Testing Process

### IMPORTANT: AuxCtrl Management
The robot's monitoring system auto-restarts AuxCtrl if killed. You MUST rename it to prevent auto-restart:

### Complete Test Workflow

```bash
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "
  # Backup and stop original firmware
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak 2>/dev/null; \
  killall -9 AuxCtrl 2>/dev/null; \

  # Run test with logging
  RUST_LOG=info /tmp/sangam_slam_demo; \

  # CRITICAL: Always restore AuxCtrl
  EXIT_CODE=\$?; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null; \
  exit \$EXIT_CODE
"
```

### Test with Automatic Restore on Failure
```bash
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak 2>/dev/null; \
  killall -9 AuxCtrl 2>/dev/null; \
  RUST_LOG=debug /tmp/sangam_slam_demo; \
  EXIT_CODE=\$?; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null; \
  exit \$EXIT_CODE
"
```

### Test with Timeout (Note: timeout command not available on device)
```bash
# Note: The device doesn't have 'timeout' command, so tests run until completion
# Use Ctrl+C to interrupt if needed, then manually restore AuxCtrl
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak 2>/dev/null; \
  killall -9 AuxCtrl 2>/dev/null; \
  RUST_LOG=info /tmp/sangam_slam_demo 2>&1; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null
"
```

## 4. Logging and Debugging

### Log Levels
- `RUST_LOG=error` - Only errors
- `RUST_LOG=warn` - Warnings and errors
- `RUST_LOG=info` - Info messages (recommended for testing)
- `RUST_LOG=debug` - Detailed debug output
- `RUST_LOG=trace` - Everything (very verbose)

### Capture Logs to File
```bash
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak 2>/dev/null; \
  killall -9 AuxCtrl 2>/dev/null; \
  RUST_LOG=debug /tmp/sangam_slam_demo > /tmp/sangam_slam_demo.log 2>&1; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null
"

# Retrieve log file
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "cat /tmp/sangam_slam_demo.log" > local_test.log
```

### Live Monitoring
```bash
# Run test and watch output live
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak 2>/dev/null; \
  killall -9 AuxCtrl 2>/dev/null; \
  RUST_LOG=info /tmp/sangam_slam_demo 2>&1; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null
"
```

## 5. Log Analysis

### Key Log Patterns to Check

#### Successful Initialization
```
GD32: Initialization complete - ready for operations
Delta2D: Driver initialized
SangamIO: Control loop started at 50Hz
```

#### Motor Movement
```
MotionController: Velocity command - linear=X.XXXm/s
GD32: Motor command sent - left=XXX, right=XXX
```

#### Component Activation
```
GD32: Setting vacuum power: XX%
GD32: Setting main/rolling brush speed: XX%
GD32: Setting side brush speed: XX%
```

#### Proper Shutdown
```
GD32: Sending shutdown commands for all components
GD32: Vacuum stop command sent
GD32: Main brush stop command sent
GD32: Side brush stop command sent
GD32: Driver shutdown complete
```

### Common Issues and Solutions

#### Issue: Robot doesn't move
Check for motor speed values:
```bash
grep "Motor command sent" /tmp/test.log
# Should show non-zero values like: left=800, right=800
```

#### Issue: Components don't stop on exit
Check shutdown sequence:
```bash
grep "shutdown\|stop" /tmp/test.log | tail -20
# Should show stop commands BEFORE thread shutdown
```

#### Issue: Serial port access denied
```bash
# Ensure AuxCtrl is stopped
ps | grep AuxCtrl
killall -9 AuxCtrl
```

## 6. Quick Commands Reference

### One-Line Build & Deploy
```bash
cd sangam-io && \
cargo build --release --target armv7-unknown-linux-musleabihf --example sangam_slam_demo && \
cat ../target/armv7-unknown-linux-musleabihf/release/examples/sangam_slam_demo | \
  sshpass -p "vacuum@123" ssh root@vacuum "cat > /tmp/sangam_slam_demo && chmod +x /tmp/sangam_slam_demo"
```

### One-Line Test with Auto-Restore
```bash
sshpass -p "vacuum@123" ssh root@vacuum "
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak 2>/dev/null; killall -9 AuxCtrl 2>/dev/null; \
  RUST_LOG=info /tmp/sangam_slam_demo 2>&1; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null
"
```

### Check Robot Status
```bash
sshpass -p "vacuum@123" ssh root@vacuum "
  echo 'Processes:' && ps | grep -E 'AuxCtrl|sangam|test' && \
  echo -e '\nSerial Ports:' && ls -la /dev/ttyS* | head -5
"
```

### Emergency Recovery
```bash
# If something goes wrong, restore AuxCtrl
sshpass -p "vacuum@123" ssh root@vacuum "
  killall -9 sangam_slam_demo 2>/dev/null; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null; \
  /usr/sbin/AuxCtrl &
"
```

## 7. Safety Guidelines

1. **Always test in open space** - Robot will move
2. **Have physical access** - For emergency power-off
3. **Start with low speeds** - Use conservative velocity values initially
4. **Monitor battery** - Low battery causes erratic behavior
5. **Always restore AuxCtrl** - Robot won't function properly without it
6. **Use timeouts** - Prevent runaway tests with `timeout` command

## 8. Development Tips

### Iterative Testing
1. Make small changes
2. Test with short timeout first (10-15 seconds)
3. Check logs for expected behavior
4. Increase test duration once verified

### Debug Specific Components
```bash
# Test only motors
RUST_LOG=sangam_io::devices::gd32=debug,info /tmp/sangam_slam_demo

# Test only lidar
RUST_LOG=sangam_io::devices::delta2d=debug,info /tmp/sangam_slam_demo

# Test motion controller
RUST_LOG=sangam_io::motion=debug,info /tmp/sangam_slam_demo
```

### Performance Monitoring
```bash
# Check CPU usage during test
sshpass -p "vacuum@123" ssh root@vacuum "top -b -n 1 | head -20"

# Check memory usage
sshpass -p "vacuum@123" ssh root@vacuum "free -h"
```

## 9. Troubleshooting Checklist

- [ ] Is AuxCtrl stopped and renamed?
- [ ] Are serial ports accessible (/dev/ttyS3, /dev/ttyS1)?
- [ ] Is the binary executable (chmod +x)?
- [ ] Is RUST_LOG set for desired verbosity?
- [ ] Is there enough free space in /tmp?
- [ ] Is the battery charged (>20%)?
- [ ] Are all old test processes killed?
- [ ] Is AuxCtrl restored after testing?

## 10. Example Full Session

```bash
# 1. Build
cd ~/Development/VacuumTiger/sangam-io
cargo build --release --target armv7-unknown-linux-musleabihf \
  --example sangam_slam_demo

# 2. Deploy
cat ../target/armv7-unknown-linux-musleabihf/release/examples/sangam_slam_demo | \
  sshpass -p "vacuum@123" ssh root@vacuum "cat > /tmp/sangam_slam_demo && chmod +x /tmp/sangam_slam_demo"

# 3. Test with logging
sshpass -p "vacuum@123" ssh root@vacuum "
  echo '=== Starting test ===' && \
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak 2>/dev/null && \
  killall -9 AuxCtrl 2>/dev/null && \
  echo 'Running sangam_slam_demo...' && \
  RUST_LOG=info /tmp/sangam_slam_demo 2>&1 | tee /tmp/sangam_slam_demo.log && \
  echo 'Test complete, restoring AuxCtrl...' && \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl 2>/dev/null && \
  echo 'Done!'
"

# 4. Analyze results
sshpass -p "vacuum@123" ssh root@vacuum "
  echo '=== Test Summary ===' && \
  echo 'Total lines:' && wc -l /tmp/sangam_slam_demo.log && \
  echo -e '\nInitialization:' && \
  grep 'Initialization complete' /tmp/sangam_slam_demo.log && \
  echo -e '\nMotor commands:' && \
  grep 'Motor command' /tmp/sangam_slam_demo.log | head -3 && \
  echo -e '\nShutdown:' && \
  grep 'shutdown complete' /tmp/sangam_slam_demo.log
"
```

---

Remember: The robot is physical hardware. Always prioritize safety and have a way to physically disconnect power if needed.