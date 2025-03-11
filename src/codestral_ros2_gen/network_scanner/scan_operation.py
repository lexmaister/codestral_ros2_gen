#!/usr/bin/env python3
import asyncio
import socket
import logging
import time
import random
from typing import Dict, Optional
import logging

from .utils import get_codestral_ros2_gen_logger
from .network_host import NetworkHost, HostState
from .network_parser import parse_network_targets


crg_logger = get_codestral_ros2_gen_logger()


class ScanOperation:
    """Context manager for a single network scan operation"""

    def __init__(
        self,
        targets: str,
        scanner=None,  # Optional scanner
        timeout: float = 1.0,
        packet_size: int = 64,
        logger: Optional[logging.Logger] = None,
    ):
        self.targets = targets
        self.scanner = scanner

        # Use scanner values if provided, otherwise use params
        self.timeout = getattr(scanner, "timeout", timeout)
        self.packet_size = getattr(scanner, "packet_size", packet_size)

        if hasattr(scanner, "logger"):
            self.logger = getattr(scanner, "logger")
        elif logger is not None:
            self.logger = logger
        elif crg_logger is not None:
            self.logger = crg_logger
        else:
            raise RuntimeError("No logger provided and default logger is not set")

        self.sock = None
        self.scan_id = random.randint(0, 0xFFFF)
        self.hosts: Dict[str, NetworkHost] = {}

        self.logger.debug(f"ScanOperation initialized with targets: {targets}")

    async def __aenter__(self):
        """Initialize socket and host data for this scan operation"""
        try:
            # Create the raw socket
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP
            )
            self.sock.setblocking(False)

            # the socket receive buffer size to 64KB (65536 bytes) - prevents packets loss
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            # enables nanosecond precision timestamps for received packets if available
            if hasattr(socket, "SO_TIMESTAMPNS"):
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_TIMESTAMPNS, 1)
            # Don't receive outgoing packets (Linux only)
            if hasattr(socket, "IP_RECVERR"):
                self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_RECVERR, 1)

            self.logger.debug(f"Socket created for scan operation (ID: {self.scan_id})")

            # Initialize hosts to scan
            self._create_hosts(self.targets)
            return self

        except Exception as e:
            self._close_socket()
            raise RuntimeError(f"Failed to create socket: {e}")

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Clean up resources when scan operation completes"""
        self._close_socket()
        # Clean up other resources
        self.hosts.clear()

        return False  # Don't suppress exceptions

    def _close_socket(self) -> None:
        """Properly clean up and close socket"""
        self.logger.info("Cleaning up and closing socket")
        if self.sock:
            try:
                # 1. Set short timeout for cleanup operations
                self.sock.settimeout(0.1)

                # 2. Clear socket buffer
                try:
                    while True:
                        self.sock.recv(4096)
                except (BlockingIOError, socket.timeout):
                    pass  # Buffer is empty or timeout occurred

                # 3. Shutdown socket if supported
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except (OSError, socket.error):
                    pass  # Shutdown not supported or socket already closed

                # 4. Close socket
                self.sock.close()
                self.logger.debug("Socket properly cleaned and closed")

            except Exception as e:
                self.logger.warning(f"Error during socket cleanup: {str(e)}")

            finally:
                self.sock = None
                self.logger.info("Socket closed for scan operation")

        else:
            self.logger.debug("No socket to close")

    def _create_hosts(self, targets: str) -> None:
        """Create NetworkHost objects for the given targets"""
        host_ips = parse_network_targets(targets)
        self.logger.info(f"Found {len(host_ips)} hosts to scan")

        # Create hosts with the same scan ID and default sequence numbers
        for ip in host_ips:
            host = NetworkHost(
                ip_address=ip,
                icmp_id=self.scan_id,  # Same ID for all hosts in this scan
                icmp_seq=1,  # Use default sequence numbers
                packet_size=self.packet_size,
                logger=self.logger,
            )
            self.hosts[ip] = host

    async def execute(self) -> Dict[str, dict]:
        """Execute the scan operation and return results"""
        self.logger.info("Start of scanning")
        if not self.sock:
            raise RuntimeError("Socket not initialized. Cannot execute scan.")

        # Send ICMP packets to all hosts
        self._send_packets()

        self.logger.info("Waiting for initial responses to arrive...")
        await asyncio.sleep(0.5)  # Give time for first responses to arrive

        # Collect responses with timeout
        await self._collect_responses()

        self.logger.info("Scan completed")

        return self._prepare_results()

    def _send_packets(self) -> None:
        """Send ICMP echo request packets to all hosts"""
        self.logger.info("Sending ICMP echo request packets to all hosts")
        for ip, host in self.hosts.items():
            self.logger.debug(f"Sending packet to {ip}")
            try:
                self.sock.sendto(host.packet, (host.ip_address, 0))
                host.mark_sent()
                time.sleep(0.01)

            except Exception as e:
                host.mark_error(str(e))

        self.logger.info("All packets sent")

    async def _collect_responses(self) -> None:
        """
        Asynchronously collect ICMP responses with a timeout.
        Uses asyncio to achieve non-blocking behavior.
        """
        end_time = time.time() + self.timeout
        self.logger.info(f"Collecting responses until {end_time}")

        pending_hosts = set(self.hosts.keys())
        while True:
            # Check if all hosts have been processed
            if not pending_hosts:
                self.logger.info("All hosts processed: stopping response collection")
                break

            # Check timeout
            if time.time() >= end_time:
                self.logger.info(
                    "Timeout reached: marking all remaining hosts as unresponsive"
                )
                for ip in pending_hosts:
                    host = self.hosts.get(ip)
                    host.mark_timeout()
                break

            # Calculate remaining time for the loop iteration
            remaining_time = max(0.001, end_time - time.time())

            try:
                # Wait for a packet to be available with asyncio's event loop
                packet, addr = await asyncio.wait_for(
                    self._receive_packet(sock=self.sock, size=65536),
                    timeout=remaining_time,
                )
                src_ip = addr[0]  # Extract source IP

                # Find the corresponding host based on src_ip
                matching_host = self.hosts.get(src_ip)

                if matching_host and matching_host.state == HostState.SENT:
                    matching_host.handle_response(packet=packet)
                    pending_hosts.remove(src_ip)

            except asyncio.TimeoutError:
                # No packets received during the wait time
                continue

            except Exception as e:
                # Catch-all for unexpected errors
                raise RuntimeError(f"Error in response collection: {e}") from e

    async def _receive_packet(self, sock: socket.socket, size):
        """Helper to get both packet and address with asyncio"""
        future = asyncio.Future()

        def _read_callback():
            try:
                data, addr = sock.recvfrom(size)
                if not future.done():
                    future.set_result((data, addr))
            except (BlockingIOError, ConnectionRefusedError):
                # Will retry on next readable event
                pass
            except Exception as exc:
                if not future.done():
                    future.set_exception(exc)

        loop = asyncio.get_running_loop()
        loop.add_reader(sock.fileno(), _read_callback)
        try:
            return await future
        finally:
            loop.remove_reader(sock.fileno())

    def _prepare_results(self) -> Dict[str, dict]:
        """Prepare the scan results"""
        results = {}

        # Mark any remaining hosts as unresponsive
        for ip, host in self.hosts.items():
            if host.state == HostState.SENT:
                host.mark_timeout()

            # Format the result
            results[ip] = {
                "state": "UP" if host.state == HostState.RESPONDED else "DOWN",
                "response_time_ms": host.rtt_ms,
                "error": host.error_message,
            }

        return results
