#!/bin/sh
#
# MITM Cleanup Script
# Restores robot to normal operation (without MITM)
#
# Usage: /mnt/UDISK/mitm_cleanup.sh
#
# WARNING: This will require a reboot to restart AuxCtrl cleanly

REAL_PORT="/dev/ttyS3"
HARDWARE_PORT="/dev/ttyS3_hardware"
VIRTUAL_PORT="/tmp/ttyS3_tap"
PID_FILE="/tmp/mitm.pid"

echo "========================================="
echo "  MITM Cleanup - Restore Normal Mode"
echo "========================================="
echo ""

# Stop MITM proxy if running
if [ -f "$PID_FILE" ]; then
    MITM_PID=$(cat "$PID_FILE")
    if kill -0 $MITM_PID 2>/dev/null; then
        echo "Stopping MITM proxy (PID $MITM_PID)..."
        kill -TERM $MITM_PID 2>/dev/null
        sleep 2
        kill -9 $MITM_PID 2>/dev/null
    fi
    rm "$PID_FILE"
    echo "✓ MITM proxy stopped"
fi

# Remove symlink
if [ -L "$REAL_PORT" ]; then
    echo "Removing symlink $REAL_PORT..."
    rm "$REAL_PORT"
    echo "✓ Symlink removed"
fi

# Remove virtual port
if [ -L "$VIRTUAL_PORT" ]; then
    echo "Removing virtual port $VIRTUAL_PORT..."
    rm "$VIRTUAL_PORT"
    echo "✓ Virtual port removed"
fi

# Restore hardware port
if [ -c "$HARDWARE_PORT" ]; then
    echo "Restoring $HARDWARE_PORT -> $REAL_PORT..."
    mv "$HARDWARE_PORT" "$REAL_PORT"
    echo "✓ Real port restored"
else
    echo "WARNING: Hardware port $HARDWARE_PORT not found!"
    echo "This is expected if cleanup was already run or MITM was never set up."
fi

echo ""
echo "========================================="
echo "  Cleanup Complete"
echo "========================================="
echo ""
echo "IMPORTANT: You must reboot the robot for AuxCtrl to work properly."
echo ""
echo "To reboot:"
echo "  reboot"
echo ""
echo "Or from dev machine:"
echo "  ./tools/mitm_reboot_robot.sh"
echo ""
