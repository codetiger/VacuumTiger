#!/bin/bash
#
# MITM Finish Run Script
# Stops MITM logging after a vacuum cleaning cycle completes
#
# Usage: ./tools/mitm_finish_run.sh

ROBOT_HOST="vacuum"
ROBOT_PASS="${ROBOT_PASSWORD:-}"

echo "========================================="
echo "  MITM - Finish Current Run"
echo "========================================="
echo ""

echo "Stopping MITM logger on robot..."
sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST "/mnt/UDISK/mitm_stop.sh"

echo ""
echo "========================================="
echo "  Run Finished"
echo "========================================="
echo ""

# Get run status
CURRENT_RUN=$(sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST \
  "cat /mnt/UDISK/mitm_run_counter 2>/dev/null" || echo "?")

if [ "$CURRENT_RUN" != "?" ]; then
    COMPLETED_RUN=$((CURRENT_RUN - 1))
    echo "Completed: Run $COMPLETED_RUN"
    echo "Next: Run $CURRENT_RUN"
    echo ""

    if [ $COMPLETED_RUN -ge 5 ]; then
        echo "🎉 All runs complete! (Target: 5)"
        echo ""
        echo "Next steps:"
        echo "  1. Retrieve all logs:"
        echo "     ./tools/mitm_retrieve_logs.sh"
        echo ""
        echo "  2. Disable MITM mode:"
        echo "     ./tools/mitm_disable.sh"
        echo ""
        echo "  3. Reboot to restore normal operation:"
        echo "     ./tools/mitm_reboot_robot.sh"
    else
        echo "Progress: $COMPLETED_RUN/5 runs complete"
        echo ""
        echo "Next steps for Run $CURRENT_RUN:"
        echo "  1. Reboot robot:"
        echo "     ./tools/mitm_reboot_robot.sh"
        echo ""
        echo "  2. Press button to start cleaning"
        echo ""
        echo "  3. Run this script again when done"
    fi
fi

echo ""
