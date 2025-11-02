#!/bin/bash
#
# MITM Enable Script (Dev-side)
# Remotely enables MITM mode on robot
#
# Usage: ./tools/mitm_enable.sh [target_runs]

ROBOT_HOST="vacuum"
ROBOT_PASS="${ROBOT_PASSWORD:-}"
TARGET_RUNS="${1:-5}"

echo "========================================="
echo "  MITM - Enable Mode"
echo "========================================="
echo ""

echo "Enabling MITM mode on robot..."
sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST "/mnt/UDISK/mitm_enable.sh $TARGET_RUNS"

echo ""
echo "========================================="
echo "  MITM Mode Enabled"
echo "========================================="
echo ""
echo "Configuration:"
echo "  Target runs: $TARGET_RUNS"
echo ""
echo "Next steps:"
echo "  1. Reboot robot to start Run 1:"
echo "     ./tools/mitm_reboot_robot.sh"
echo ""
echo "  2. Press physical button to start cleaning"
echo ""
echo "  3. When done, finish the run:"
echo "     ./tools/mitm_finish_run.sh"
echo ""
echo "  4. Repeat steps 1-3 for $TARGET_RUNS total runs"
echo ""
