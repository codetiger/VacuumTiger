#!/bin/sh
#
# MITM Boot Setup Script
# Runs on robot boot to set up serial port MITM proxy BEFORE Monitor/AuxCtrl start
#
# Installation:
#   1. Copy to /mnt/UDISK/mitm_boot_setup.sh
#   2. Add to /mnt/UDISK/_root.sh:
#      [ -f /mnt/UDISK/mitm_boot_setup.sh ] && /mnt/UDISK/mitm_boot_setup.sh
#
# This script should be called early in boot sequence, before robotManager starts

ENABLED_FLAG="/mnt/UDISK/mitm_enabled"
MITM_BINARY="/mnt/UDISK/binary/serial_mitm"
RUN_COUNTER_FILE="/mnt/UDISK/mitm_run_counter"
REAL_PORT="/dev/ttyS3"
HARDWARE_PORT="/dev/ttyS3_hardware"
VIRTUAL_PORT="/tmp/ttyS3_tap"
PID_FILE="/tmp/mitm.pid"
LOG_DIR="/mnt/UDISK/log"

echo "[MITM Boot] Checking if MITM mode is enabled..."

# Check if MITM mode is enabled
if [ ! -f "$ENABLED_FLAG" ]; then
    echo "[MITM Boot] MITM mode disabled (no flag file). Skipping setup."
    exit 0
fi

echo "[MITM Boot] MITM mode ENABLED. Starting setup..."

# Check if binary exists
if [ ! -f "$MITM_BINARY" ]; then
    echo "[MITM Boot] ERROR: MITM binary not found at $MITM_BINARY"
    echo "[MITM Boot] Please run mitm_deploy_all.sh from dev machine first"
    exit 1
fi

# Create log directory if needed
mkdir -p "$LOG_DIR"

# Read current run counter
if [ -f "$RUN_COUNTER_FILE" ]; then
    RUN_NUM=$(cat "$RUN_COUNTER_FILE")
else
    RUN_NUM=1
fi

echo "[MITM Boot] Current run: $RUN_NUM"

# Check if real serial port exists
if [ ! -c "$REAL_PORT" ]; then
    echo "[MITM Boot] ERROR: Serial port $REAL_PORT not found!"
    exit 1
fi

# Check if already set up (in case of restart)
if [ -c "$HARDWARE_PORT" ]; then
    echo "[MITM Boot] MITM already set up (found $HARDWARE_PORT)"
else
    # Move real port to backup location
    echo "[MITM Boot] Moving $REAL_PORT -> $HARDWARE_PORT"
    mv "$REAL_PORT" "$HARDWARE_PORT"

    if [ $? -ne 0 ]; then
        echo "[MITM Boot] ERROR: Failed to move serial port"
        exit 1
    fi
fi

# Start MITM proxy in background
echo "[MITM Boot] Starting MITM proxy (run $RUN_NUM)..."
$MITM_BINARY $RUN_NUM >/dev/null 2>&1 &
MITM_PID=$!

# Save PID for later cleanup
echo $MITM_PID > "$PID_FILE"

# Wait a moment for PTY to be created
sleep 1

# Check if virtual port was created
if [ ! -L "$VIRTUAL_PORT" ]; then
    echo "[MITM Boot] ERROR: Virtual port $VIRTUAL_PORT not created!"
    kill $MITM_PID 2>/dev/null
    mv "$HARDWARE_PORT" "$REAL_PORT"
    exit 1
fi

# Create symlink /dev/ttyS3 -> /tmp/ttyS3_tap
echo "[MITM Boot] Creating symlink $REAL_PORT -> $VIRTUAL_PORT"
ln -sf "$VIRTUAL_PORT" "$REAL_PORT"

if [ $? -ne 0 ]; then
    echo "[MITM Boot] ERROR: Failed to create symlink"
    kill $MITM_PID 2>/dev/null
    mv "$HARDWARE_PORT" "$REAL_PORT"
    exit 1
fi

echo "[MITM Boot] ✓ MITM setup complete!"
echo "[MITM Boot]   - MITM proxy running (PID: $MITM_PID)"
echo "[MITM Boot]   - Real port: $HARDWARE_PORT"
echo "[MITM Boot]   - Virtual port: $VIRTUAL_PORT (via PTY)"
echo "[MITM Boot]   - Symlink: $REAL_PORT -> $VIRTUAL_PORT"
echo "[MITM Boot]   - AuxCtrl will transparently use MITM proxy"
echo "[MITM Boot]   - Run number: $RUN_NUM"
echo ""
echo "[MITM Boot] Ready for vacuum cleaning cycle."
echo "[MITM Boot] Press robot button to start cleaning."
echo "[MITM Boot] Run 'mitm_stop.sh' when complete."
echo ""

exit 0
