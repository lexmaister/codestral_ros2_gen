import asyncio
import socket
import logging
from typing import List, Dict, Set
import os
import time
import select

from codestral_ros2_gen import logger_main
from codestral_ros2_gen.network_scanner.network_host import NetworkHost, HostState
from codestral_ros2_gen.network_scanner.network_parser import parse_network_targets


logger = logging.getLogger(f"{logger_main}.{__name__.split('.')[-1]}")


class NetworkScanner:
    """Asynchronous network scanner using ICMP echo requests"""

    def __init__(self, timeout: float = 2.0):
        self.timeout = timeout
        self.hosts: List[NetworkHost] = []

        # Check privileges
        if os.name != "nt" and os.geteuid() != 0:
            raise PermissionError(
                "Raw socket operations require root privileges. Run with sudo."
            )

        logger.info(f"NetworkScanner initialized with timeout={timeout}s")

    def _create_socket(self) -> socket.socket:
        """Create and configure raw socket for ICMP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
        sock.setblocking(False)
        logger.info("Raw socket created for ICMP")
        return sock

    def _send_packets_to_hosts(
        self, sock: socket.socket, hosts: Set[NetworkHost]
    ) -> None:
        """Send ICMP packets to all hosts"""
        logger.info(f"Sending packets to {len(hosts)} hosts")
        for host in hosts:
            try:
                logger.debug(f"Sending packet to {host.ip_address}")
                sock.sendto(host.packet, (host.ip_address, 0))
                host.mark_sent()
            except Exception as e:
                logger.error(f"Error sending to {host.ip_address}: {str(e)}")
                host.mark_error(str(e))

        logger.info("All packets sent")

    def _close_socket(self, sock: socket.socket) -> None:
        """Properly clean up and close socket"""
        try:
            logger.info("Cleaning up and closing socket")
            # 1. Set short timeout for cleanup operations
            sock.settimeout(0.1)

            # 2. Clear socket buffer
            try:
                while True:
                    sock.recv(4096)
            except (BlockingIOError, socket.timeout):
                pass  # Buffer is empty or timeout occurred

            # 3. Shutdown socket if supported
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except (OSError, socket.error):
                pass  # Shutdown not supported or socket already closed

            # 4. Close socket
            sock.close()
            logger.debug("Socket properly cleaned and closed")

        except Exception as e:
            logger.warning(f"Error during socket cleanup: {str(e)}")

    async def _receive_responses(
        self, sock: socket.socket, hosts: Set[NetworkHost]
    ) -> None:
        """Receive and process responses with timeout"""

        async def is_scanning_complete() -> bool:
            """Check if scanning should continue"""
            # Give control back to event loop periodically during check
            await asyncio.sleep(0)
            return not any(h.state == HostState.SENT for h in hosts)

        # Use asyncio.get_event_loop().time() instead of time.time()
        loop = asyncio.get_running_loop()
        end_time = loop.time() + self.timeout

        while True:
            # Check timeout
            if loop.time() >= end_time:
                logger.debug("Scan timeout reached")
                break

            # Check if all hosts are processed
            if await is_scanning_complete():
                logger.debug("All hosts processed")
                break

            try:
                ready_to_read, _, _ = select.select([sock], [], [], 0.01)

                if not ready_to_read:
                    await asyncio.sleep(0.01)
                    continue

                data, addr = sock.recvfrom(1024)
                receive_time = (
                    time.time()
                )  # Use absolute time instead of loop time for host

                if len(data) < 20:
                    logger.debug("Received packet too small")
                    continue

                icmp_header = data[20:]
                src_ip = addr[0]

                matching_host = next(
                    (
                        h
                        for h in hosts
                        if h.ip_address == src_ip and h.state == HostState.SENT
                    ),
                    None,
                )

                if matching_host and matching_host.validate_response(
                    icmp_header, receive_time
                ):
                    logger.debug(
                        f"Response from {src_ip} in {matching_host.result.response_time*1000:.2f}ms"
                    )
                    matching_host.mark_responded()

            except BlockingIOError:
                await asyncio.sleep(0.01)
            except Exception as e:
                error_msg = f"Error receiving response: {str(e)}"
                logger.error(error_msg)

                # Determine which host(s) were affected
                if "matching_host" in locals() and matching_host:
                    # If we know which host caused the error, mark just that one
                    logger.debug(
                        f"Marking host {matching_host.ip_address} as error due to: {error_msg}"
                    )
                    matching_host.mark_error(error_msg)
                else:
                    # If we can't determine specific host, mark all pending hosts as error
                    logger.warning(
                        "Marking all pending hosts as error due to receive failure"
                    )
                    for host in hosts:
                        if host.state == HostState.SENT:
                            host.mark_error(error_msg)
                    # Exit the receive loop as we can't continue processing
                    break

        # Mark remaining SENT hosts as timeout
        for host in hosts:
            if host.state == HostState.SENT:
                logger.debug(f"Host {host.ip_address} timed out")
                host.mark_timeout()

    def _collect_results(self) -> Dict[str, dict]:
        """Collect and format scan results"""
        return {
            host.ip_address: {
                "is_alive": host.result.is_alive,
                "response_time": host.result.response_time,
                "error": host.result.error,
                "state": host.state.value,
            }
            for host in self.hosts
        }

    async def scan(self, targets: str) -> Dict[str, dict]:
        """Perform network scan on specified targets"""
        # Parse targets and create host objects
        host_ips = parse_network_targets(targets)
        logger.info(f"Scanning {len(host_ips)} targets")
        self.hosts = [NetworkHost(ip) for ip in host_ips]

        sock = None
        try:
            # Create and initialize socket
            sock = self._create_socket()

            # Prepare hosts for scanning
            hosts_to_scan = set(self.hosts)

            # Send packets to all hosts
            self._send_packets_to_hosts(sock, hosts_to_scan)

            # Wait for and process responses
            await self._receive_responses(sock, hosts_to_scan)

            # Return formatted results
            return self._collect_results()

        except Exception as e:
            raise RuntimeError(f"Scan failled with error:\n\n{str(e)}\n")

        finally:
            if sock:
                self._close_socket(sock)
