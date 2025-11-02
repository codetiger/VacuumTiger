# Deployment Notes

## Key Deployment Steps for CRL-200S Robot

### 1. File Transfer Method

**Problem**: SCP doesn't work on the device
```bash
scp binary root@vacuum:/tmp/test
# Error: ash: /usr/libexec/sftp-server: not found
```

**Solution**: Use `cat` over SSH instead
```bash
cat binary | sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "cat > /tmp/test && chmod +x /tmp/test"
```

This works because:
- Doesn't require sftp-server (which is missing on minimal embedded systems)
- Uses standard input/output redirection
- Works on any system with SSH

### 2. Preventing AuxCtrl Auto-Restart

**Problem**: `killall -9 AuxCtrl` stops the process but it auto-restarts immediately
- Managed by a monitor/watchdog process
- Only gives ~2-3 seconds before it comes back
- Not enough time for proper testing

**Solution**: Temporarily rename the binary
```bash
# Before testing - prevent restart
ssh root@vacuum "mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak"

# Kill any running instance
ssh root@vacuum "killall -9 AuxCtrl"

# Now run your test
ssh root@vacuum "RUST_LOG=debug /tmp/test_lidar"

# After testing - restore original firmware
ssh root@vacuum "mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl"
```

### 3. Complete Deployment Workflow

```bash
# Step 1: Build for ARM
cd sangam-io
cargo build --release --example test_lidar_scenario --features="std,gd32,lidar"

# Step 2: Deploy binary
cat ../target/armv7-unknown-linux-musleabihf/release/examples/test_lidar_scenario | \
  sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "cat > /tmp/test_lidar && chmod +x /tmp/test_lidar"

# Step 3: Disable AuxCtrl
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && killall -9 AuxCtrl"

# Step 4: Run test
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "RUST_LOG=debug /tmp/test_lidar"

# Step 5: Restore original firmware
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl"
```

### 4. Alternative: One-Command Test (with auto-restore)

```bash
sshpass -p "$ROBOT_PASSWORD" ssh root@vacuum "
  mv /usr/sbin/AuxCtrl /usr/sbin/AuxCtrl.bak && \
  killall -9 AuxCtrl 2>/dev/null; \
  RUST_LOG=debug /tmp/test_lidar; \
  EXIT_CODE=\$?; \
  mv /usr/sbin/AuxCtrl.bak /usr/sbin/AuxCtrl; \
  exit \$EXIT_CODE
"
```

This ensures AuxCtrl is restored even if your test crashes.

## Important Notes

- **Always restore AuxCtrl** after testing - robot won't function without it
- The rename method prevents the watchdog from restarting it
- If you forget to restore, just SSH in and rename it back
- Original firmware will start automatically on next reboot even if not restored manually
