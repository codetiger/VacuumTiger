# MITM Logging Guide - Full Vacuum Cycle Capture

Complete guide for capturing AuxCtrl ↔ GD32 communication during vacuum cleaning operations using the automated MITM logging system.

## Overview

This system enables systematic capture of serial protocol data during multiple full vacuum cleaning cycles. It uses a PTY-based MITM proxy that transparently intercepts communication between the robot's AuxCtrl software and the GD32F103 motor controller.

**Key Features:**
- Transparent to AuxCtrl and Monitor watchdog
- Automatic setup on boot
- Run counter for tracking progress (target: 3-5 runs)
- Manual control with helper scripts
- Graceful shutdown with statistics
- Archives logs for analysis

## Architecture

```
Boot Sequence:
  1. Robot boots → procd starts
  2. _root.sh runs → mitm_boot_setup.sh executes
  3. /dev/ttyS3 moved to /dev/ttyS3_hardware
  4. serial_mitm starts, creates /tmp/ttyS3_tap (PTY)
  5. /dev/ttyS3 symlinked to /tmp/ttyS3_tap
  6. Monitor starts → forks AuxCtrl
  7. AuxCtrl opens /dev/ttyS3 → transparently uses MITM

Communication Flow:
  AuxCtrl → /dev/ttyS3 (symlink) → /tmp/ttyS3_tap (PTY)
    → serial_mitm (logs TX)
    → /dev/ttyS3_hardware (real port)
    → GD32 MCU

  GD32 MCU → /dev/ttyS3_hardware
    → serial_mitm (logs RX)
    → /tmp/ttyS3_tap (PTY)
    → AuxCtrl
```

## Prerequisites

### Development Machine

- Rust toolchain with ARM cross-compilation support
- `sshpass` for automated SSH/SCP
- SSH access to robot configured

Install requirements:
```bash
# macOS
brew install sshpass

# Add ARM target
rustup target add armv7-unknown-linux-musleabihf
```

### Robot

- SSH root access with password or key-based authentication
  - Set `ROBOT_PASSWORD` environment variable before running scripts if using password auth
  - Example: `export ROBOT_PASSWORD=your_password`
- `/mnt/UDISK` partition with ~100MB free space
- `/mnt/UDISK/_root.sh` modified to call `mitm_boot_setup.sh`

## Quick Start (3-5 Capture Runs)

### 1. Deploy Everything

From `protocol-mitm/` directory:

```bash
# Build and deploy binary + scripts to robot
./tools/mitm_deploy_all.sh
```

This will:
- Build `serial_mitm` for ARM
- Deploy binary to `/mnt/UDISK/binary/serial_mitm`
- Deploy all robot-side scripts to `/mnt/UDISK/`
- Verify robot connectivity

### 2. Enable MITM Mode

```bash
./tools/mitm_enable.sh
```

This creates the flag file that activates MITM on next boot.

### 3. Perform Capture Runs (Repeat 3-5 times)

For each run:

```bash
# Step A: Reboot robot
./tools/mitm_reboot_robot.sh
# (Waits for boot, verifies MITM started)

# Step B: Start vacuum cleaning
# → Press physical button on robot
# → Wait for robot to clean small room
# → Wait for robot to return to dock

# Step C: Finish run
./tools/mitm_finish_run.sh
# (Stops logging, increments counter, shows progress)
```

Repeat Steps A-C until you have 3-5 complete runs.

### 4. Retrieve Logs

After all runs are complete:

```bash
./tools/mitm_retrieve_logs.sh
# Downloads all logs to ./logs/mitm_captures_TIMESTAMP/
```

### 5. Disable MITM and Restore

```bash
./tools/mitm_disable.sh
# (Cleans up, archives logs on robot, offers to reboot)
```

Robot is now back to normal operation.

## Detailed Workflow

### Initial Setup (One Time)

#### 1. Verify SSH Configuration

Ensure `~/.ssh/config` contains:

```
Host vacuum
    HostName 192.168.68.101  # Your robot's IP
    User root
    HostKeyAlgorithms +ssh-rsa
    PubkeyAcceptedKeyTypes +ssh-rsa
    KexAlgorithms +diffie-hellman-group1-sha1
```

Test connection:
```bash
ssh root@vacuum "echo OK"
# Should print "OK" without prompting for password
```

#### 2. Integrate with _root.sh

SSH to robot and edit `/mnt/UDISK/_root.sh`:

```bash
ssh root@vacuum
vi /mnt/UDISK/_root.sh
```

Add this line at the beginning:

```bash
[ -f /mnt/UDISK/mitm_boot_setup.sh ] && /mnt/UDISK/mitm_boot_setup.sh
```

This ensures MITM setup runs early in boot sequence, before Monitor starts.

#### 3. Deploy MITM Infrastructure

```bash
cd protocol-mitm/
./tools/mitm_deploy_all.sh
```

Expected output:
```
Building serial_mitm binary for ARM...
✓ Binary built successfully (520K)
✓ Robot is reachable
✓ Directories created
✓ Binary deployed to /mnt/UDISK/binary/serial_mitm
✓ All scripts deployed
```

### Capture Session

#### Enable MITM Mode

```bash
./tools/mitm_enable.sh 5  # Target 5 runs (default)
```

This creates `/mnt/UDISK/mitm_enabled` and resets run counter to 1.

#### Run 1-5 Capture Loop

**For Run 1:**

```bash
# Reboot to start MITM
./tools/mitm_reboot_robot.sh
```

Expected boot sequence on robot:
```
[MITM Boot] MITM mode ENABLED. Starting setup...
[MITM Boot] Current run: 1
[MITM Boot] Starting MITM proxy (run 1)...
[MITM Boot] ✓ MITM setup complete!
[MITM Boot] Ready for vacuum cleaning cycle.
```

**Start Cleaning:**

- Press the physical CLEAN button on the robot
- Robot should start cleaning normally
- Let it clean a small room (5-10 minutes)
- Wait for robot to return to dock

**Finish Run:**

```bash
./tools/mitm_finish_run.sh
```

Expected output:
```
Stopping MITM logger on robot...
✓ MITM proxy stopped

Capture Statistics:
  Run: 1
  Log file: mitm_capture_run1_20231101_143022.log
  Size: 4.2M
  TX packets: 15234
  RX packets: 15189

Progress: 1/5 runs complete
```

**Repeat for Runs 2-5:**

Just run the loop again:
```bash
./tools/mitm_reboot_robot.sh  # Run 2 starts
# ... press button, wait for dock ...
./tools/mitm_finish_run.sh

./tools/mitm_reboot_robot.sh  # Run 3 starts
# ... and so on ...
```

After Run 5:
```
✅ TARGET ACHIEVED: 5 runs completed!

Next steps:
  1. Retrieve all logs: ./tools/mitm_retrieve_logs.sh
  2. Disable MITM mode: ./tools/mitm_disable.sh
```

#### Retrieve Logs

```bash
./tools/mitm_retrieve_logs.sh
```

Output:
```
Found 5 log file(s)

Downloading logs to: ./logs/mitm_captures_20231101_150322

  mitm_capture_run1_20231101_143022.log ... ✓ (4.2M)
  mitm_capture_run2_20231101_144156.log ... ✓ (4.3M)
  mitm_capture_run3_20231101_145301.log ... ✓ (4.1M)
  mitm_capture_run4_20231101_150412.log ... ✓ (4.4M)
  mitm_capture_run5_20231101_151523.log ... ✓ (4.2M)

Summary:
  Files downloaded: 5
  Total size: 21M
  Location: ./logs/mitm_captures_20231101_150322
```

#### Disable MITM

```bash
./tools/mitm_disable.sh
```

This will:
- Run cleanup script (kill proxy, restore /dev/ttyS3)
- Remove flag file
- Archive logs on robot
- Offer to reboot

Choose 'y' to reboot and restore normal operation.

## Log Format

Each log file contains timestamped bidirectional packet captures:

```
=== MITM Session Started ===
Run Number: 1
Timestamp: 2023-11-01T14:30:22+00:00
Real port: /dev/ttyS3_hardware
Virtual port: /tmp/ttyS3_tap
===

[1698850222.946188] TX 14 bytes
  HEX: FA FB 0B 66 00 00 00 00 00 00 00 00 66 00
  PKT: CMD=0x66 LEN=11
       (HEARTBEAT - Keepalive packet)

[1698850222.946523] RX 102 bytes
  HEX: FA FB 63 15 01 06 00 0F 04 00 00 01 A6 21 ...
  PKT: CMD=0x15 LEN=99
       (STATUS - Sensor/IMU/Battery data)

...

=== MITM Session Ended ===
End Timestamp: 2023-11-01T14:40:35+00:00
TX Packets: 15234
RX Packets: 15189
TX Bytes: 213276
RX Bytes: 1503711
===
```

**Packet Annotations:**
- **TX**: AuxCtrl → GD32 (commands)
- **RX**: GD32 → AuxCtrl (status/responses)
- **Timestamps**: Microsecond precision
- **CMD annotations**: Common command types decoded

## Troubleshooting

### MITM Proxy Not Starting

**Symptom:** After reboot, `serial_mitm` is not running

**Checks:**
```bash
ssh root@vacuum

# 1. Is flag file present?
ls -l /mnt/UDISK/mitm_enabled

# 2. Is binary executable?
ls -l /mnt/UDISK/binary/serial_mitm
# Should show: -rwxr-xr-x

# 3. Does _root.sh call boot script?
grep mitm_boot_setup /mnt/UDISK/_root.sh

# 4. Check boot log
cat /tmp/serial_mitm.log 2>/dev/null || dmesg | grep MITM
```

**Fix:** Redeploy with `./tools/mitm_deploy_all.sh`

### AuxCtrl Not Responding After MITM Setup

**Symptom:** Robot doesn't respond to button presses, motors don't work

**Cause:** MITM proxy may have crashed, breaking the serial connection

**Recovery:**
```bash
ssh root@vacuum

# Check if proxy is running
ps | grep serial_mitm

# If not running, cleanup and reboot
/mnt/UDISK/mitm_cleanup.sh
reboot
```

### Device Node Already Exists Error

**Symptom:** Boot setup fails with "cannot move /dev/ttyS3: File exists"

**Cause:** Previous MITM session didn't clean up properly

**Fix:**
```bash
ssh root@vacuum
/mnt/UDISK/mitm_cleanup.sh
reboot
```

### Logs Not Being Created

**Symptom:** MITM proxy runs but no log files appear

**Checks:**
```bash
ssh root@vacuum

# Check log directory
ls -lh /mnt/UDISK/log/

# Check disk space
df -h /mnt/UDISK

# Check proxy status
ps aux | grep serial_mitm
```

**Fix:** Ensure `/mnt/UDISK/log/` exists and has space. Run `mitm_deploy_all.sh` to recreate directories.

### Robot Becomes Unreachable

**Symptom:** SSH connection times out, robot doesn't respond

**Cause:** Network issue or robot crashed

**Recovery:**
1. Power cycle robot (unplug, wait 10s, plug back in)
2. Wait 60s for boot
3. Run `./tools/mitm_reboot_robot.sh` to test connectivity
4. If still unreachable, disable MITM physically:
   - Power off robot
   - On next boot, immediately SSH and run `/mnt/UDISK/mitm_disable.sh`

## File Structure Reference

### On Robot (`/mnt/UDISK/`)

```
/mnt/UDISK/
├── binary/
│   └── serial_mitm              # MITM proxy binary (520KB)
├── log/
│   ├── mitm_capture_run1_*.log  # Capture logs
│   ├── mitm_capture_run2_*.log
│   └── archive/                 # Archived logs
│       └── capture_TIMESTAMP/
├── mitm_enabled                 # Flag file (presence = MITM active)
├── mitm_run_counter             # Current run number (1-5)
├── mitm_boot_setup.sh           # Boot-time setup script
├── mitm_stop.sh                 # Stop logging script
├── mitm_cleanup.sh              # Restore normal operation
├── mitm_enable.sh               # Enable MITM mode
├── mitm_disable.sh              # Disable MITM mode
└── _root.sh                     # User boot script (calls boot_setup)
```

### On Dev Machine (`protocol-mitm/`)

```
protocol-mitm/
├── src/
│   └── main.rs                  # MITM proxy source (serial_mitm binary)
├── scripts/                     # Robot-side scripts (deployed)
│   ├── mitm_boot_setup.sh
│   ├── mitm_stop.sh
│   ├── mitm_cleanup.sh
│   ├── mitm_enable.sh
│   ├── mitm_disable.sh
│   └── mitm_init.sh
├── tools/                       # Dev-side control scripts
│   ├── mitm_deploy_all.sh       # Build + deploy everything
│   ├── mitm_enable.sh           # Remote enable
│   ├── mitm_reboot_robot.sh     # Reboot + wait
│   ├── mitm_finish_run.sh       # Stop current run
│   ├── mitm_retrieve_logs.sh    # Download logs
│   ├── mitm_disable.sh          # Remote disable
│   └── README.md
├── logs/                        # Downloaded logs (local)
│   └── mitm_captures_*/
├── docs/
│   ├── MITM_LOGGING_GUIDE.md    # This file
│   └── LIDAR_INITIALIZATION_SEQUENCE.md
├── Cargo.toml
└── README.md
```

## Analysis Tips

### Extract Initialization Sequence

```bash
cd logs/mitm_captures_*/

# Find first init packets (CMD=0x08)
grep "CMD=0x08" mitm_capture_run1_*.log | head -20

# Count heartbeat frequency
grep "CMD=0x66" mitm_capture_run1_*.log | head -100

# Extract all unique command types
grep "PKT: CMD=" mitm_capture_run1_*.log | \
  awk '{print $3}' | sort | uniq -c
```

### Compare Runs for Consistency

```bash
# Extract command sequences from each run
for log in mitm_capture_run*.log; do
    echo "=== $log ==="
    grep "PKT: CMD=" "$log" | awk '{print $3}' | uniq -c
done
```

### Timing Analysis

```bash
# Calculate heartbeat intervals
grep "CMD=0x66" mitm_capture_run1_*.log | \
  awk -F'[\\[\\]]' '{print $2}' | \
  head -100 > heartbeat_timestamps.txt

# Use a script to calculate intervals
python3 -c "
import sys
times = [float(line.strip()) for line in open('heartbeat_timestamps.txt')]
intervals = [times[i+1] - times[i] for i in range(len(times)-1)]
print(f'Avg interval: {sum(intervals)/len(intervals):.6f}s')
print(f'Min: {min(intervals):.6f}s, Max: {max(intervals):.6f}s')
"
```

## Best Practices

1. **Small Room First**: Test with a small room (5-10 min cleaning) to validate setup before longer runs

2. **Check Logs Immediately**: After each run, check log file size before rebooting for next run

3. **Monitor Disk Space**: MITM logs can be 3-5MB each; ensure sufficient space before starting

4. **Retrieve Frequently**: Download logs after 2-3 runs to avoid losing data if robot crashes

5. **Document Anomalies**: Note any unusual robot behavior (stuttering, errors) in log filenames or separate notes

6. **Compare Runs**: Look for consistency across runs to validate protocol understanding

## Advanced Usage

### Change Target Run Count

```bash
./tools/mitm_enable.sh 10  # Capture 10 runs instead of 5
```

### Manual Control (Robot-side)

If you're already SSHed into the robot:

```bash
# Enable manually
/mnt/UDISK/mitm_enable.sh
reboot

# After cleaning
/mnt/UDISK/mitm_stop.sh

# Disable manually
/mnt/UDISK/mitm_disable.sh
```

### Log to External Storage

Modify `MITM_BINARY` path in `mitm_boot_setup.sh` to log to USB drive if mounted at `/mnt/SNN/`:

```bash
LOG_DIR="/mnt/SNN/mitm_logs"
```

### Custom Run Numbers

Start from a specific run:

```bash
ssh root@vacuum "echo 3 > /mnt/UDISK/mitm_run_counter"
```

## Safety Considerations

- **Monitor Watchdog Active**: The Monitor watchdog continues to supervise AuxCtrl. If proxy crashes, motors stop safely
- **Log Rotation**: MITM logs don't auto-rotate; monitor disk space
- **Network Dependency**: Dev-side scripts require robot network access
- **Reboot Required**: Always reboot after disabling MITM to ensure clean state

## Related Documentation

- [LIDAR_INITIALIZATION_SEQUENCE.md](LIDAR_INITIALIZATION_SEQUENCE.md) - Complete lidar protocol specification
- [Protocol-MITM README](../README.md) - Project overview and quick start
- [SangamIO Guide](../../sangam-io/GUIDE.md) - Implementation guide for the protocol library
- GD32 Protocol Spec - See `sangam-io/src/devices/gd32/protocol.rs` in source code

## Support

If you encounter issues:

1. Check this guide's Troubleshooting section
2. Review `src/main.rs` source code for MITM proxy behavior details
3. Check robot logs: `ssh root@vacuum "dmesg | tail -50"`
4. Verify network connectivity: `ping vacuum`

For persistent issues, collect:
- Output of `./tools/mitm_deploy_all.sh`
- Robot boot log: `ssh root@vacuum "dmesg"`
- Process list: `ssh root@vacuum "ps aux | grep -E 'serial_mitm|AuxCtrl|Monitor'"`
- MITM build log: Check `cargo build` output from protocol-mitm directory
