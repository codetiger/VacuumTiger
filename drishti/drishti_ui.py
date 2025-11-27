#!/usr/bin/env python3
"""
Drishti UI - CRL-200S Robot Visualization

Full-screen robot diagram with sensor overlays.
"""

import sys
import argparse
import logging
from PyQt5.QtWidgets import QApplication
from ui.main_window import MainWindow

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger(__name__)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Drishti - CRL-200S Robot Visualization"
    )
    parser.add_argument(
        "--robot",
        default="192.168.68.101",
        help="Robot hostname or IP address (default: 192.168.68.101)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5555,
        help="TCP port (default: 5555)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose logging"
    )
    parser.add_argument(
        "--log-raw-packets",
        action="store_true",
        help="Log raw GD32 status packets to file for debugging"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    logger.info("Drishti - CRL-200S Robot Visualization")
    logger.info(f"Connecting to {args.robot}:{args.port}")
    if args.log_raw_packets:
        logger.info("Raw packet logging ENABLED")

    # Create Qt application
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Create and show main window
    try:
        window = MainWindow(
            robot_ip=args.robot,
            port=args.port,
            log_raw_packets=args.log_raw_packets
        )
        window.show()

        logger.info("Application started")
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
