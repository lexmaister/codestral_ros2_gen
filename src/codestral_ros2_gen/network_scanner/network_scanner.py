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
        """
        Initialize network scanner

        Args:
            timeout: Seconds to wait for response
        """
        self.timeout = timeout
        self.hosts: List[NetworkHost] = []

        # Check privileges
        if os.name != "nt" and os.geteuid() != 0:
            raise PermissionError(
                "Raw socket operations require root privileges. Run with sudo."
            )

        logger.info(f"NetworkScanner initialized with timeout={timeout}s")

    async def scan(self, targets: str) -> Dict[str, dict]:
        """Perform network scan on specified targets"""
        # Parse targets and create host objects
        host_ips = parse_network_targets(targets)
        logger.info(f"Scanning {len(host_ips)} targets")
        self.hosts = [NetworkHost(ip) for ip in host_ips]

        sock = None
        try:
            # Create socket inside the coroutine
            sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
            sock.setblocking(False)

            # All hosts to scan
            hosts_to_scan = set(self.hosts)

            # Send packets to all hosts
            await self._send_packets(sock, hosts_to_scan)

            # Wait for responses with timeout
            await self._receive_responses(sock, hosts_to_scan)

            # Return results
            return self._collect_results()

        except Exception as e:
            # Make sure we're only logging actual exceptions
            logger.error(f"Scan error: {str(e)}")
            raise

        finally:
            if sock:
                try:
                    # Proper socket cleanup sequence:

                    # 1. Set a short timeout to avoid hanging during cleanup
                    sock.settimeout(0.1)

                    # 2. Clear any pending data from the socket buffer
                    try:
                        # Non-blocking read to clear buffer
                        while True:
                            # Try to empty the buffer
                            sock.recv(4096)
                    except (BlockingIOError, socket.timeout):
                        pass  # Buffer is empty or timeout occurred

                    # 3. Shut down the socket (if supported by platform)
                    try:
                        # SHUT_RDWR = 2 (both sending and receiving)
                        sock.shutdown(socket.SHUT_RDWR)
                    except (OSError, socket.error):
                        # Some socket types or platforms don't support shutdown
                        pass

                    # 4. Finally close the socket
                    sock.close()
                    logger.debug("Socket properly cleaned and closed")

                except Exception as e:
                    logger.warning(f"Error during socket cleanup: {str(e)}")

    async def _send_packets(self, sock: socket.socket, hosts: Set[NetworkHost]) -> None:
        """Send ICMP packets to all specified hosts"""
        for host in hosts:
            try:
                logger.debug(f"Sending packet to {host.ip_address}")
                sock.sendto(host.packet, (host.ip_address, 0))
                host.mark_sent()
            except Exception as e:
                logger.error(f"Error sending to {host.ip_address}: {e}")
                host.mark_error(str(e))

    async def _receive_responses(
        self, sock: socket.socket, hosts: Set[NetworkHost]
    ) -> None:
        """Receive and process responses with timeout"""
        # Calculate end time based on timeout
        end_time = time.time() + self.timeout

        # Continue until timeout or all hosts responded
        while time.time() < end_time and any(h.state == HostState.SENT for h in hosts):
            # Check if there are any responses ready with select
            ready_to_read, _, _ = select.select([sock], [], [], 0.01)

            if not ready_to_read:
                # Nothing to read yet, sleep briefly and continue
                await asyncio.sleep(0.01)
                continue

            try:
                # Receive data (standard blocking call but we know data is ready)
                data, addr = sock.recvfrom(1024)
                receive_time = time.time()

                if len(data) < 20:
                    logger.debug("Received packet too small")
                    continue

                icmp_header = data[20:]
                src_ip = addr[0]

                # Find matching host
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
                        f"Response received from {src_ip} in {matching_host.result.response_time*1000:.2f}ms"
                    )
                    matching_host.mark_responded()

            except BlockingIOError:
                # Just in case, although select should prevent this
                await asyncio.sleep(0.01)
            except Exception as e:
                logger.error(f"Error receiving response: {e}")

        # Mark remaining hosts as timeout
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
