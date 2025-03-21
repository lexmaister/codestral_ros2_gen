import argparse
import sys
import logging

from .network_scanner import NetworkScanner


logger = logging.getLogger("nscan")


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
        metavar="[s]",
        help="Scan timeout in seconds",
    )
    args = parser.parse_args()

    scanner = NetworkScanner(timeout=args.timeout, logger=logger)
    results = scanner.scan(args.target)
    output = scanner.format_results(results, show_all=args.show_all)
    print(output)


if __name__ == "__main__":
    sys.exit(main())
