#!/home/alex/Projects/ros2/.venv/bin/python3

import asyncio
import time

from codestral_ros2_gen import logger
from codestral_ros2_gen.network_scanner.network_scanner import NetworkScanner


async def main():
    try:
        # Create scanner instance
        scanner = NetworkScanner(timeout=1.0)

        # Example targets (adjust for your network)
        # targets = "192.168.0.1-192.168.0.10"  # Scan first 10 IPs
        # Alternative target formats:
        targets = "192.168.0.0/24"  # CIDR notation
        # targets = "192.168.1.1, 192.168.1.100"  # Comma-separated IPs

        logger.info(f"\nStarting scan for targets: {targets}")
        start_time = time.time()

        # Run scan
        results = await scanner.scan(targets)

        # logger.info results
        logger.info("\nScan Results:")
        logger.info("-" * 60)
        for ip, data in results.items():
            status = "✓ ALIVE" if data["is_alive"] else "✗ DOWN"
            response = (
                f"{data['response_time']:.3f}ms" if data["response_time"] else "N/A"
            )
            if data["is_alive"]:
                logger.info(f"Host: {ip:16} Status: {status:8} Response: {response:10}")
            # if data["error"]:
            #     logger.info(f"  Error: {data['error']}")

        end_time = time.time()
        logger.info(f"\nScan completed in {end_time - start_time:.2f} seconds")

    except PermissionError:
        logger.info(
            "Error: This script requires root privileges. Please run with sudo."
        )
    except Exception as e:
        logger.info(f"Error during scan: {e}")


if __name__ == "__main__":
    asyncio.run(main())
