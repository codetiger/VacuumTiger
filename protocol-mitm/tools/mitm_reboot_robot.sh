#!/bin/bash
#
# MITM Reboot Robot Script
# Reboots the robot and waits for it to come back online
#
# Usage: ./tools/mitm_reboot_robot.sh

ROBOT_HOST="vacuum"
ROBOT_PASS="${ROBOT_PASSWORD:-}"

echo "========================================="
echo "  MITM - Reboot Robot"
echo "========================================="
echo ""

echo "Rebooting robot..."
sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST "sync && reboot" 2>/dev/null || true

echo "Waiting for robot to shut down..."
sleep 5

# Wait for robot to come back online
echo -n "Waiting for robot to boot up"
MAX_WAIT=60
WAIT_COUNT=0

while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    if sshpass -p "$ROBOT_PASS" ssh -o ConnectTimeout=2 root@$ROBOT_HOST "echo OK" &>/dev/null; then
        echo " ✓"
        break
    fi
    echo -n "."
    sleep 2
    WAIT_COUNT=$((WAIT_COUNT + 1))
done

if [ $WAIT_COUNT -ge $MAX_WAIT ]; then
    echo " TIMEOUT"
    echo ""
    echo "Robot did not come back online within ${MAX_WAIT} seconds"
    echo "Please check robot status manually"
    exit 1
fi

# Give MITM proxy time to start
echo "Waiting for MITM proxy to initialize..."
sleep 3

# Check MITM status
echo ""
echo "Checking MITM status..."
MITM_RUNNING=$(sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST \
  "ps | grep serial_mitm | grep -v grep" || echo "")

if [ -n "$MITM_RUNNING" ]; then
    echo "✓ MITM proxy is running"
    RUN_NUM=$(sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST \
      "cat /mnt/UDISK/mitm_run_counter 2>/dev/null" || echo "?")
    echo "  Current run: $RUN_NUM"
else
    echo "⚠ MITM proxy is NOT running"
    echo "  Check if MITM mode is enabled:"
    echo "    ssh root@$ROBOT_HOST 'ls -l /mnt/UDISK/mitm_enabled'"
fi

echo ""
echo "========================================="
echo "  Robot Ready"
echo "========================================="
echo ""
echo "Next steps:"
echo "  1. Press physical button on robot to start cleaning"
echo "  2. Wait for robot to return to dock"
echo "  3. Stop logging: ./tools/mitm_finish_run.sh"
echo ""
