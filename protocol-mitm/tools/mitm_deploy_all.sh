#!/bin/bash
#
# MITM Deploy All Script
# Builds serial_mitm binary and deploys all scripts to robot
#
# Usage: ./tools/mitm_deploy_all.sh
#
# Requirements:
#   - ARM cross-compilation target installed (armv7-unknown-linux-musleabihf)
#   - sshpass installed
#   - SSH access to robot configured

set -e  # Exit on error

ROBOT_HOST="vacuum"
ROBOT_PASS="${ROBOT_PASSWORD:-}"
SCRIPT_DIR="scripts"
TARGET_DIR="/mnt/UDISK"
BINARY_DIR="/mnt/UDISK/binary"

echo "========================================="
echo "  MITM Deploy - Build and Deploy"
echo "========================================="
echo ""

# Check if we're in the right directory
if [ ! -f "Cargo.toml" ]; then
    echo "ERROR: Must run from sangam-io root directory"
    exit 1
fi

# Check if sshpass is available
if ! command -v sshpass &> /dev/null; then
    echo "ERROR: sshpass not found. Install with: brew install sshpass"
    exit 1
fi

# Check if ARM target is installed
echo "Checking ARM cross-compilation target..."
if ! rustup target list | grep -q "armv7-unknown-linux-musleabihf (installed)"; then
    echo "Installing ARM target..."
    rustup target add armv7-unknown-linux-musleabihf
fi
echo "✓ ARM target available"
echo ""

# Build serial_mitm for ARM
echo "Building serial_mitm binary for ARM..."
cargo build --release --bin serial_mitm --target armv7-unknown-linux-musleabihf

if [ ! -f "../target/armv7-unknown-linux-musleabihf/release/serial_mitm" ]; then
    echo "ERROR: Build failed - binary not found"
    exit 1
fi

BINARY_SIZE=$(du -h ../target/armv7-unknown-linux-musleabihf/release/serial_mitm | awk '{print $1}')
echo "✓ Binary built successfully ($BINARY_SIZE)"
echo ""

# Test robot connectivity
echo "Testing robot connectivity..."
if ! sshpass -p "$ROBOT_PASS" ssh -o ConnectTimeout=5 root@$ROBOT_HOST "echo OK" &>/dev/null; then
    echo "ERROR: Cannot connect to robot at $ROBOT_HOST"
    echo "Check network connection and SSH config"
    exit 1
fi
echo "✓ Robot is reachable"
echo ""

# Create target directories on robot
echo "Creating directories on robot..."
sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST "mkdir -p $BINARY_DIR $TARGET_DIR/log"
echo "✓ Directories created"
echo ""

# Deploy binary
echo "Deploying serial_mitm binary..."
cat ../target/armv7-unknown-linux-musleabihf/release/serial_mitm | \
  sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST \
  "cat > $BINARY_DIR/serial_mitm && chmod +x $BINARY_DIR/serial_mitm"
echo "✓ Binary deployed to $BINARY_DIR/serial_mitm"
echo ""

# Deploy scripts
echo "Deploying robot-side scripts..."
for script in $SCRIPT_DIR/*.sh; do
    SCRIPT_NAME=$(basename "$script")
    echo "  - $SCRIPT_NAME"
    cat "$script" | \
      sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST \
      "cat > $TARGET_DIR/$SCRIPT_NAME && chmod +x $TARGET_DIR/$SCRIPT_NAME"
done
echo "✓ All scripts deployed"
echo ""

# Check if _root.sh exists and has MITM boot hook
echo "Checking _root.sh integration..."
HAS_HOOK=$(sshpass -p "$ROBOT_PASS" ssh root@$ROBOT_HOST \
  "grep -q 'mitm_boot_setup' /mnt/UDISK/_root.sh 2>/dev/null && echo yes || echo no")

if [ "$HAS_HOOK" = "yes" ]; then
    echo "✓ MITM boot hook already present in _root.sh"
else
    echo "⚠ _root.sh does not call mitm_boot_setup.sh"
    echo "  Add this line to /mnt/UDISK/_root.sh:"
    echo "  [ -f /mnt/UDISK/mitm_boot_setup.sh ] && /mnt/UDISK/mitm_boot_setup.sh"
fi
echo ""

echo "========================================="
echo "  Deployment Complete!"
echo "========================================="
echo ""
echo "Deployed files:"
echo "  Binary: $BINARY_DIR/serial_mitm ($BINARY_SIZE)"
echo "  Scripts: $TARGET_DIR/*.sh"
echo ""
echo "Next steps:"
echo "  1. Enable MITM mode:"
echo "     ./tools/mitm_enable.sh"
echo ""
echo "  2. Reboot robot to start first capture:"
echo "     ./tools/mitm_reboot_robot.sh"
echo ""
echo "  3. Press physical button to start cleaning"
echo ""
echo "  4. When done, finish the run:"
echo "     ./tools/mitm_finish_run.sh"
echo ""
echo "  5. Repeat steps 2-4 for 3-5 runs"
echo ""
