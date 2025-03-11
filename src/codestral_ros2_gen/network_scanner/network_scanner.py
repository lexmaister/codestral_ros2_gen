#!/usr/bin/env python3
import asyncio
import socket
import logging
import time
import select
import random
import os
import struct
from typing import Dict, Set, Optional, Any, List, Union

from .network_host import NetworkHost, HostState, HostResult
from .network_parser import parse_network_targets


class ScanOperation:
    """Context manager for a single network scan operation"""

    def __init__(
        self,
        targets: str,
        scanner=None,  # Optional scanner
        timeout: float = 1.0,
        packet_size: int = 64,
        logger: Optional[Any] = None,
    ):
        self.targets = targets
        self.scanner = scanner

        # Use scanner values if provided, otherwise use params
        self.timeout = getattr(scanner, "timeout", timeout)
        self.packet_size = getattr(scanner, "packet_size", packet_size)
        self.logger = getattr(
            scanner, "logger", logger or logging.getLogger("scan_operation")
        )

        self.sock = None
        self.scan_id = random.randint(0, 0xFFFF)
        self.hosts = set()
        self.host_map = {}

    async def __aenter__(self):
        """Initialize socket and host data for this scan operation"""
        # Create the raw socket
        try:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP
            )
            self.sock.setblocking(False)

            # the socket receive buffer size to 64KB (65536 bytes) - prevents packets loss
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            # enables nanosecond precision timestamps for received packets if available
            if hasattr(socket, "SO_TIMESTAMPNS"):
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_TIMESTAMPNS, 1)

            self.logger.debug(f"Socket created for scan operation (ID: {self.scan_id})")

            # Initialize hosts to scan
            self._create_hosts(self.targets)
            return self

        except Exception as e:
            self.logger.error(f"Failed to create socket: {e}")
            if self.sock:
                self.sock.close()
                self.sock = None
            raise

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Clean up resources when scan operation completes"""
        if self.sock:
            try:
                self.sock.setblocking(True)
                self.sock.settimeout(0.1)
                self.sock.close()
            except Exception as e:
                self.logger.warning(f"Error closing socket: {e}")
            finally:
                self.sock = None
                self.logger.debug("Socket closed for scan operation")

        # Clean up other resources
        self.hosts.clear()
        self.host_map.clear()

        return False  # Don't suppress exceptions

    def _create_hosts(self, targets: str) -> None:
        """Create NetworkHost objects for the given targets"""
        host_ips = parse_network_targets(targets)
        self.logger.info(f"Scanning {len(host_ips)} targets")

        # Create hosts with the same scan ID but unique sequence numbers
        for idx, ip in enumerate(host_ips, 1):
            host = NetworkHost(
                ip,
                timeout_sec=self.scanner.timeout,
                packet_size=self.scanner.packet_size,
                logger=self.logger,
                icmp_id=self.scan_id,  # Same ID for all hosts in this scan
                icmp_seq=idx,  # Unique sequence per host
            )
            self.hosts.add(host)
            self.host_map[ip] = host

    async def execute(self) -> Dict[str, dict]:
        """Execute the scan operation and return results"""
        if not self.sock:
            raise RuntimeError("Socket not initialized. Use async with pattern.")

        # Send ICMP packets to all hosts
        self._send_packets()

        # Collect responses with timeout
        await self._collect_responses()

        # Prepare and return results
        return self._prepare_results()

    def _send_packets(self) -> None:
        """Send ICMP echo request packets to all hosts"""
        send_time = time.time()

        for host in self.hosts:
            try:
                # Create the ICMP packet
                packet = host.create_icmp_packet()

                # Send the packet
                self.sock.sendto(packet, (host.ip_address, 0))
                host.state = HostState.SENT
                host.send_time = send_time

            except Exception as e:
                self.logger.debug(f"Failed to send to {host.ip_address}: {e}")
                host.state = HostState.ERROR
                host.error = str(e)

    async def _collect_responses(self) -> None:
        """Collect ICMP responses with timeout"""
        end_time = time.time() + self.scanner.timeout

        # Set up for select() polling
        recv_socket = self.sock.fileno()

        while True:
            # Check timeout
            if time.time() >= end_time:
                self.logger.debug("Scan timeout reached")
                break

            # Check if all hosts are processed
            if await self._is_scanning_complete():
                self.logger.debug("All hosts processed")
                break

            # Wait for responses with appropriate timeout
            remaining_time = max(0, end_time - time.time())
            await asyncio.sleep(0)  # Yield to event loop

            try:
                # Use select with remaining timeout
                r, _, _ = select.select([recv_socket], [], [], min(0.1, remaining_time))

                if not r:
                    # No data available, loop again
                    continue

                # Process ICMP response
                packet, addr = self.sock.recvfrom(65536)
                recv_time = time.time()
                src_ip = addr[0]

                # Find the host that this response belongs to
                matching_host = self.host_map.get(src_ip)

                if matching_host and matching_host.state == HostState.SENT:
                    # Process the response
                    if matching_host.process_response(packet, recv_time):
                        self.logger.debug(
                            f"Response from {src_ip} in {matching_host.response_time_ms:.2f}ms"
                        )

            except (socket.timeout, BlockingIOError):
                # No data available yet
                continue

            except Exception as e:
                self.logger.error(f"Error receiving packets: {e}")
                # Brief pause before continuing
                await asyncio.sleep(0.01)

    async def _is_scanning_complete(self) -> bool:
        """Check if scanning should continue"""
        # Give control back to event loop periodically during check
        await asyncio.sleep(0)
        return not any(h.state == HostState.SENT for h in self.hosts)

    def _prepare_results(self) -> Dict[str, dict]:
        """Prepare the scan results"""
        results = {}

        # Mark any remaining hosts as unresponsive
        for host in self.hosts:
            if host.state == HostState.SENT:
                host.state = HostState.INACTIVE

            # Format the result
            results[host.ip_address] = {
                "state": "up" if host.state == HostState.ACTIVE else "down",
                "response_time_ms": host.response_time_ms,
                "error": host.error,
            }

        return results


class NetworkScanner:
    """Asynchronous network scanner using ICMP echo requests"""

    def __init__(
        self,
        timeout: float = 2.0,
        packet_size: int = 64,
        logger: Optional[Any] = None,
    ):
        self.timeout = timeout
        self.packet_size = packet_size
        self.logger = logger or logging.getLogger("network_scanner")
        self.logger.info(f"NetworkScanner initialized with timeout={timeout} seconds")

    async def scan(self, targets: str) -> ScanOperation:
        """Create a new scan operation for the specified targets"""
        return ScanOperation(self, targets)


async def run_network_monitoring(targets: str, interval: float = 5.0):
    """Example function to run periodic network scans"""
    scanner = NetworkScanner(timeout=1.0)
    logger = logging.getLogger("network_monitor")

    try:
        while True:
            start_time = time.time()

            try:
                # Use the scan operation as a context manager
                async with await scanner.scan(targets) as operation:
                    results = await operation.execute()

                    # Process results
                    active_hosts = sum(
                        1 for ip, data in results.items() if data["state"] == "up"
                    )
                    logger.info(
                        f"Scan complete: {active_hosts}/{len(results)} hosts active"
                    )

                    # Additional result processing...
                    for ip, data in results.items():
                        if data["state"] == "up":
                            logger.debug(f"{ip}: {data['response_time_ms']:.2f}ms")

            except Exception as e:
                logger.error(f"Scan failed: {e}")

            # Calculate time until next scan
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            await asyncio.sleep(sleep_time)

    except asyncio.CancelledError:
        logger.info("Network monitoring stopped")


if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    # Run example monitoring for local network
    try:
        asyncio.run(run_network_monitoring("192.168.1.0/24", interval=7.0))
    except KeyboardInterrupt:
        print("Monitoring stopped by user")
