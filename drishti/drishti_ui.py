#!/usr/bin/env python3
"""
Drishti UI - Robot Vacuum Visualization Application

Main entry point for the graphical user interface.
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
    """Main entry point"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Drishti - Robot Vacuum Visualization System"
    )
    parser.add_argument(
        "--robot",
        default="192.168.68.101",
        help="Robot hostname or IP address (default: 192.168.68.101)"
    )
    parser.add_argument(
        "--pub-port",
        type=int,
        default=5555,
        help="Telemetry publisher port (default: 5555)"
    )
    parser.add_argument(
        "--cmd-port",
        type=int,
        default=5556,
        help="Command receiver port (default: 5556)"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose logging"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    logger.info("=" * 60)
    logger.info("Drishti - Robot Vacuum Visualization System")
    logger.info("=" * 60)
    logger.info(f"Robot: {args.robot}")
    logger.info(f"Telemetry port: {args.pub_port}")
    logger.info(f"Command port: {args.cmd_port}")
    logger.info("=" * 60)

    # Create Qt application
    app = QApplication(sys.argv)

    # Set application style
    app.setStyle('Fusion')

    # Create and show main window
    try:
        window = MainWindow(
            robot_ip=args.robot,
            pub_port=args.pub_port,
            cmd_port=args.cmd_port
        )
        window.show()

        logger.info("Application started successfully")
        logger.info("Close the window or press Ctrl+C to exit")

        # Run application event loop
        sys.exit(app.exec_())

    except KeyboardInterrupt:
        logger.info("\nApplication interrupted by user")
        sys.exit(0)

    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
