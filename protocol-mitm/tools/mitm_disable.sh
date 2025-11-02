#!/bin/bash
#
# MITM Disable Script (Dev-side)
# Remotely disables MITM mode on robot
#
# Usage: ./tools/mitm_disable.sh

ROBOT_HOST="vacuum"
ROBOT_PASS="${ROBOT_PASSWORD:-}"

echo "========================================="
echo "  MITM - Disable Mode"
echo "========================================="
echo ""

echo "Disabling MITM mode on robot..."
sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST "/mnt/UDISK/mitm_disable.sh"

echo ""
echo "========================================="
echo "  MITM Mode Disabled"
echo "========================================="
echo ""
echo "The robot requires a reboot to restore normal operation."
echo ""
echo "Reboot now? [y/N]: "
read -r RESPONSE

if [ "$RESPONSE" = "y" ] || [ "$RESPONSE" = "Y" ]; then
    ./tools/mitm_reboot_robot.sh
else
    echo ""
    echo "Skipping reboot. To reboot later:"
    echo "  ./tools/mitm_reboot_robot.sh"
    echo ""
fi

echo "To retrieve archived logs:"
echo "  ./tools/mitm_retrieve_logs.sh"
echo ""
