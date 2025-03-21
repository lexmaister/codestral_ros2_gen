"""
Network Scanner CLI

TIP - executable creation:
    - Install ufx: sudo apt install ufx
    - Install PyInstaller: pip install pyinstaller
    - Create an executable:
    pyinstaller --onefile --clean --upx-dir=/usr/bin --distpath scripts/ ./src/codestral_ros2_gen/network_scan/nscan.py
"""

import argparse
import sys
import logging
import os
import json

from codestral_ros2_gen.network_scan.network_scanner import NetworkScanner

# Set up logging independent of codestral_ros2_gen package
logger = logging.getLogger("nscan")
logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)


def main():
    parser = argparse.ArgumentParser(
        description="""Network Scanner CLI

Examples of target specifications:
  - CIDR notation: 192.168.1.0/24
  - IP range: 192.168.1.1-192.168.1.254
  - Single IP: 192.168.1.1
  - Octet ranges: 192.168.10-20.1-254
  - Wildcard: 192.168.1.*""",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "target", help="Network target specification (see examples above)"
    )
    parser.add_argument(
        "-a",
        "--show-all",
        action="store_true",
        help="Display all hosts, not just active ones",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=5.0,
        metavar="SEC",
        help="Scan timeout in seconds",
    )
    # New option for output path
    parser.add_argument(
        "-o",
        "--out_path",
        metavar="DIR",
        help="Output directory where JSON results (nscan_results.json) will be saved; by default, /tmp directory",
        default=None,
    )
    args = parser.parse_args()

    # Set default out_path if not provided
    if args.out_path is None:
        args.out_path = os.path.join("/tmp", "nscan_results.json")
    else:
        # If a directory is provided, append the JSON filename
        if os.path.isdir(args.out_path):
            args.out_path = os.path.join(args.out_path, "nscan_results.json")

    scanner = NetworkScanner(timeout=args.timeout, logger=logger)
    results = scanner.scan(args.target)
    output = scanner.format_results(results, show_all=args.show_all)
    scanner.logger.info(f"Scan results:\n{output}")

    # Save results as JSON
    with open(args.out_path, "w") as f:
        json.dump(results, f, indent=4)
    logger.info(f"JSON results saved to {args.out_path}")


if __name__ == "__main__":
    sys.exit(main())
