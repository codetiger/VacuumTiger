#!/usr/bin/env python3
"""
Drishti UI - CRL-200S Robot Visualization

Two modes:
- sensor: Connect to SangamIO for raw sensor visualization (robot view)
- slam: Connect to DhruvaSLAM for SLAM visualization (map, trajectory, controls)
"""

import sys
import argparse
import logging
from PyQt5.QtWidgets import QApplication

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger(__name__)


def run_sensor_mode(args):
    """Run sensor mode - connects to SangamIO for raw sensor data."""
    from ui.main_window import MainWindow

    logger.info("Drishti - Sensor Mode")
    logger.info(f"Connecting to SangamIO at {args.robot}:{args.port}")
    if args.log_raw_packets:
        logger.info("Raw packet logging ENABLED")

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    try:
        window = MainWindow(
            robot_ip=args.robot,
            port=args.port,
            log_raw_packets=args.log_raw_packets
        )
        window.show()
        logger.info("Sensor mode started")
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


def run_slam_mode(args):
    """Run SLAM mode - connects to DhruvaSLAM for SLAM visualization."""
    from ui.slam_main_window import SlamMainWindow

    logger.info("Drishti - SLAM Mode")
    logger.info(f"Connecting to DhruvaSLAM at {args.host}:{args.port}")
    if args.command_port:
        logger.info(f"Command port: {args.command_port}")

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    try:
        window = SlamMainWindow(
            slam_host=args.host,
            slam_port=args.port,
            command_port=args.command_port
        )
        window.show()
        logger.info("SLAM mode started")
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Drishti - CRL-200S Robot Visualization"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose logging"
    )

    subparsers = parser.add_subparsers(dest="mode", help="Operating mode")

    # Sensor mode subcommand
    sensor_parser = subparsers.add_parser(
        "sensor",
        help="Sensor mode - connect to SangamIO for raw sensor visualization"
    )
    sensor_parser.add_argument(
        "--robot",
        default="192.168.68.101",
        help="SangamIO hostname or IP address (default: 192.168.68.101)"
    )
    sensor_parser.add_argument(
        "--port",
        type=int,
        default=5555,
        help="SangamIO TCP port (default: 5555)"
    )
    sensor_parser.add_argument(
        "--log-raw-packets",
        action="store_true",
        help="Log raw GD32 status packets to file for debugging"
    )

    # SLAM mode subcommand
    slam_parser = subparsers.add_parser(
        "slam",
        help="SLAM mode - connect to DhruvaSLAM for map visualization"
    )
    slam_parser.add_argument(
        "--host",
        default="localhost",
        help="DhruvaSLAM hostname (default: localhost)"
    )
    slam_parser.add_argument(
        "--port",
        type=int,
        default=5557,
        help="DhruvaSLAM port for TCP commands+maps and UDP status (default: 5557)"
    )
    slam_parser.add_argument(
        "--command-port",
        type=int,
        default=None,
        help="DhruvaSLAM command port (default: same as --port)"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    if args.mode == "sensor":
        run_sensor_mode(args)
    elif args.mode == "slam":
        run_slam_mode(args)
    else:
        # Default: show help
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
