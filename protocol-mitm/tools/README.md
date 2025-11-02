# MITM Logging Tools

Helper scripts for automated GD32 protocol capture during vacuum cleaning operations.

## Quick Start

```bash
# 1. Deploy everything to robot
./mitm_deploy_all.sh

# 2. Enable MITM mode
./mitm_enable.sh

# 3. For each of 3-5 runs:
./mitm_reboot_robot.sh          # Reboot & start MITM
# → Press button on robot to start cleaning
# → Wait for robot to dock
./mitm_finish_run.sh            # Stop logging

# 4. Retrieve all logs
./mitm_retrieve_logs.sh

# 5. Disable MITM and restore
./mitm_disable.sh
```

## Available Scripts

| Script | Purpose |
|--------|---------|
| `mitm_deploy_all.sh` | Build serial_mitm binary and deploy all scripts to robot |
| `mitm_enable.sh` | Enable MITM mode (takes effect on next boot) |
| `mitm_disable.sh` | Disable MITM mode and restore normal operation |
| `mitm_reboot_robot.sh` | Reboot robot and wait for MITM to start |
| `mitm_finish_run.sh` | Stop current capture run and show progress |
| `mitm_retrieve_logs.sh` | Download all capture logs from robot |

## Documentation

See [../docs/MITM_LOGGING_GUIDE.md](../docs/MITM_LOGGING_GUIDE.md) for complete documentation.
