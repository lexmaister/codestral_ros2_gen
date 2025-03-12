#!/home/alex/Projects/ros2/.venv/bin/python3
"""
simple_scanner.py - Minimal example of using ScanOperation context manager
"""

import asyncio
import sys
from codestral_ros2_gen.network_scanner.scan_operation import ScanOperation, HostState


async def run_scan(target):
    """Run a simple network scan using ScanOperation as context manager"""
    print(f"Scanning network: {target}")

    try:
        # Use ScanOperation as an async context manager
        async with ScanOperation(targets=target, timeout=5) as scan_op:
            # Execute the scan
            await scan_op.execute()

            # Print results
            print(f"\nScanned {len(scan_op.hosts)} hosts")
            print(
                f"Found {len([h for h in scan_op.hosts.values() if h.state == HostState.RESPONDED])} responding hosts\n"
            )

    except Exception as e:
        print(f"\n\n{e}\n")
        return 1

    return 0


if __name__ == "__main__":
    # Get target from command line if provided, otherwise use default
    target = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.0/24"

    try:
        exit_code = asyncio.run(run_scan(target))
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\nScan interrupted by user")
        sys.exit(130)
