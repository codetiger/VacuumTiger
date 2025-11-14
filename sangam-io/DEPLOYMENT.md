# SangamIO Production Deployment Guide

## Prerequisites

- Target device: Allwinner A33 running Tina Linux
- Cross-compilation host: Ubuntu/macOS with Rust toolchain
- SSH access to robot (root@vacuum, password: vacuum@123)

## Cross-Compilation Setup

### Install ARM Toolchain

```bash
# Add ARM target to Rust
rustup target add armv7-unknown-linux-musleabihf

# Install cross-linker (Ubuntu/Debian)
sudo apt-get install gcc-arm-linux-gnueabihf

# For macOS, use homebrew
brew install arm-linux-gnueabihf-binutils
```

### Configure Cargo

Create `.cargo/config.toml` in project root:

```toml
[target.armv7-unknown-linux-musleabihf]
linker = "arm-linux-gnueabihf-gcc"
```

## Building for Production

```bash
cd sangam-io

# Build optimized binary
cargo build --release --target armv7-unknown-linux-musleabihf

# Strip debug symbols (reduces size by ~40%)
arm-linux-gnueabihf-strip target/armv7-unknown-linux-musleabihf/release/sangamio

# Verify binary
file target/armv7-unknown-linux-musleabihf/release/sangamio
# Output: ELF 32-bit LSB executable, ARM, EABI5 version 1
```

## Deployment

### 1. Stop Original Firmware

The robot's monitor service auto-restarts killed processes. We must rename the original binary:

```bash
# Connect to robot
ssh root@vacuum

# Backup and disable AuxCtrl
mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.original
killall -9 AuxCtrl 2>/dev/null
```

### 2. Deploy SangamIO

Since the device lacks sftp-server, use cat over SSH:

```bash
# From host machine
cat target/armv7-unknown-linux-musleabihf/release/sangamio | \
  sshpass -p "vacuum@123" ssh root@vacuum "cat > /usr/sbin/sangamio && chmod +x /usr/sbin/sangamio"

# Deploy configuration
sshpass -p "vacuum@123" ssh root@vacuum "cat > /etc/sangamio.toml" < sangamio.toml
```

### 3. Create Systemd Service

Create `/etc/systemd/system/sangamio.service`:

```ini
[Unit]
Description=SangamIO Hardware Abstraction Daemon
After=network.target

[Service]
Type=simple
ExecStart=/usr/sbin/sangamio
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal
Environment="RUST_LOG=info"

# Resource limits
LimitNOFILE=1024
MemoryMax=50M
CPUQuota=10%

# Security
PrivateTmp=true
ProtectHome=true
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
```

### 4. Enable Service

```bash
# Reload systemd
systemctl daemon-reload

# Enable auto-start
systemctl enable sangamio

# Start service
systemctl start sangamio

# Check status
systemctl status sangamio
journalctl -u sangamio -f
```

## Configuration

Edit `/etc/sangamio.toml`:

```toml
[robot]
type = "crl200s"
name = "vacuum-01"

[hardware]
gd32_port = "/dev/ttyS3"
lidar_port = "/dev/ttyS1"
gd32_baud = 115200
lidar_baud = 230400

[network]
tcp_port = 5000
tcp_bind = "0.0.0.0"
udp_discovery = true
udp_port = 5001

[motion]
# Physical parameters
wheel_base = 0.235              # meters
wheel_radius = 0.0325           # meters
ticks_per_revolution = 1560.0

# Safety limits
max_linear_velocity = 0.5       # m/s
max_angular_velocity = 2.0      # rad/s
linear_acceleration = 0.3       # m/s²
angular_acceleration = 1.0      # rad/s²

[control]
heartbeat_interval_ms = 20
control_frequency_hz = 50
odometry_update_hz = 20

[logging]
level = "info"
file = "/var/log/sangamio.log"
max_size_mb = 10
max_files = 3
```

## Monitoring

### Health Check Script

Create `/usr/local/bin/sangamio-health`:

```bash
#!/bin/bash

# Check if daemon is running
if ! pgrep -x sangamio > /dev/null; then
    echo "ERROR: SangamIO not running"
    exit 1
fi

# Check TCP port
if ! netstat -tln | grep -q ':5000'; then
    echo "ERROR: TCP port 5000 not listening"
    exit 1
fi

# Check serial ports
if [ ! -c /dev/ttyS3 ]; then
    echo "ERROR: GD32 serial port missing"
    exit 1
fi

if [ ! -c /dev/ttyS1 ]; then
    echo "ERROR: Lidar serial port missing"
    exit 1
fi

echo "OK: SangamIO healthy"
exit 0
```

### Monitoring with Cron

Add to root's crontab:

```bash
# Check every minute, restart if needed
* * * * * /usr/local/bin/sangamio-health || systemctl restart sangamio
```

## Troubleshooting

### Common Issues

#### 1. Serial Port Permission Denied
```bash
# Add user to dialout group
usermod -a -G dialout root

# Or run as root (default)
```

#### 2. Original Firmware Interfering
```bash
# Check if AuxCtrl is running
ps aux | grep AuxCtrl

# Ensure it's renamed
ls -la /usr/sbin/AuxCtrl*
```

#### 3. TCP Connection Refused
```bash
# Check firewall
iptables -L

# Verify binding
netstat -tln | grep 5000

# Test locally
telnet localhost 5000
```

#### 4. High CPU Usage
```bash
# Check log spam
journalctl -u sangamio | tail -100

# Reduce log level
export RUST_LOG=warn
```

### Debug Mode

For development/debugging:

```bash
# Stop service
systemctl stop sangamio

# Run manually with debug output
RUST_LOG=debug /usr/sbin/sangamio
```

### Performance Tuning

```bash
# Monitor resource usage
top -p $(pgrep sangamio)

# Check thread count
ls /proc/$(pgrep sangamio)/task | wc -l

# Measure latency
time echo '{"type":"Stop"}' | nc localhost 5000
```

## Rollback Procedure

If issues arise, restore original firmware:

```bash
# Stop SangamIO
systemctl stop sangamio
systemctl disable sangamio

# Restore AuxCtrl
mv /usr/sbin/AuxCtrl.original /usr/sbin/AuxCtrl

# Reboot to ensure clean state
reboot
```

## Security Considerations

1. **Network Security**:
   - SangamIO binds to all interfaces by default
   - Consider firewall rules or bind to localhost only
   - No authentication built-in (add proxy if needed)

2. **File Permissions**:
   ```bash
   chmod 755 /usr/sbin/sangamio
   chmod 644 /etc/sangamio.toml
   ```

3. **Resource Limits**:
   - Memory limited to 50MB
   - CPU limited to 10%
   - Prevents runaway resource usage

## Updates

To update SangamIO:

```bash
# Build new version
cargo build --release --target armv7-unknown-linux-musleabihf

# Deploy atomically
cat new_binary | ssh root@vacuum "cat > /usr/sbin/sangamio.new && \
  chmod +x /usr/sbin/sangamio.new && \
  mv /usr/sbin/sangamio.new /usr/sbin/sangamio"

# Restart service
ssh root@vacuum "systemctl restart sangamio"
```

## Logs and Diagnostics

```bash
# View recent logs
journalctl -u sangamio -n 100

# Follow logs
journalctl -u sangamio -f

# Export logs
journalctl -u sangamio --since="1 hour ago" > sangamio.log

# Check core dumps
ls /var/core/
```