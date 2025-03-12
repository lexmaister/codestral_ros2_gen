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

"""
This module provides the ScanOperation class, which is a context manager for a single network scan operation.
It handles the configuration of sockets, creation of NetworkHost objects, sending of ICMP packets, and collection of responses.
"""


class ScanOperation:
    """
    Context manager for a single network scan operation.

    This class manages the lifecycle of a network scan, including socket configuration, host creation,
    packet sending, and response collection. It uses asyncio for non-blocking operations.
    """

    def __init__(
        self,
        targets: str,
        scanner=None,  # Optional scanner
        timeout: float = 1.0,
        packet_size: int = 64,
        sending_interval: float = 0.05,
        send_buff_size: int = 65536,
        recv_buff_size: int = 131072,
        logger: Optional[logging.Logger] = None,
    ):
        """
        Initialize the ScanOperation.

        Args:
            targets (str): The network targets to scan.
            scanner (Optional[object]): An optional scanner object with configuration attributes.
            timeout (float): The timeout for the scan operation.
            packet_size (int): The size of the ICMP packets to send.
            sending_interval (float): The interval between sending packets.
            send_buff_size (int): The size of the send buffer.
            recv_buff_size (int): The size of the receive buffer.
            logger (Optional[logging.Logger]): The logger to use for logging messages.
        """
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

        self.send_sock: socket.socket = None
        self.recv_sock: socket.socket = None
        self.send_buffer = send_buff_size
        self.recv_buffer = recv_buff_size
        self.sending_interval = sending_interval
        self.scan_id = random.randint(0, 0xFFFF)
        self.hosts: Dict[str, NetworkHost] = {}

        self.logger.debug(f"ScanOperation initialized with targets: {targets}")

    async def __aenter__(self):
        """
        Initialize socket and host data for this scan operation.

        This method is called when entering the context manager.
        It configures the sockets and creates NetworkHost objects for the scan targets.

        Returns:
            ScanOperation: The initialized ScanOperation instance.
        """
        self._configure_sockets()
        self._create_hosts()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Clean up resources when scan operation completes.

        This method is called when exiting the context manager.
        It closes the sockets and clears the hosts dictionary.

        Args:
            exc_type: The type of the exception that caused the context to be exited.
            exc_val: The exception instance that caused the context to be exited.
            exc_tb: The traceback object for the exception that caused the context to be exited.

        Returns:
            bool: False to indicate that exceptions should not be suppressed.
        """
        self._close_sockets()
        # Clean up other resources
        self.hosts.clear()

        return False  # Don't suppress exceptions

    def _configure_sockets(self):
        """Configure send and receive sockets with optimal settings for Linux.

        This method sets up the send and receive sockets with appropriate options for ICMP communication.
        """
        try:
            # Send raw socket configuration
            self.send_sock = socket.socket(
                socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP
            )
            self.send_sock.setblocking(True)
            self.send_sock.setsockopt(socket.SOL_IP, socket.IP_TTL, 64)
            self.send_sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_SNDBUF, self.send_buffer
            )

            # Receive raw socket configuration
            self.recv_sock = socket.socket(
                socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP
            )
            self.recv_sock.setblocking(False)
            self.recv_sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_RCVBUF, self.recv_buffer
            )
            self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            self.logger.info(
                f"Sockets configured for scan operation (ID: {self.scan_id})"
            )
        except socket.error as e:
            self._close_sockets()
            raise RuntimeError(f"Failed to create sockets: {e}")

    def _close_sockets(self) -> None:
        """Properly clean up and close sockets.

        This method ensures that the sockets are closed properly, clearing any remaining data and shutting down the sockets.
        """
        self.logger.info(
            f"Cleaning up and closing sockets for scan operation (ID: {self.scan_id})"
        )
        for sock in ("send_sock", "recv_sock"):
            self.logger.debug(f"Closing socket {sock!r}")
            sock_obj: socket.socket = getattr(self, sock)
            if sock_obj:
                try:
                    # 1. Set short timeout for cleanup operations
                    sock_obj.settimeout(0.1)

                    # 2. Clear socket buffer
                    try:
                        while True:
                            sock_obj.recv(4096)
                    except (BlockingIOError, socket.timeout):
                        pass  # Buffer is empty or timeout occurred

                    # 3. Shutdown socket if supported
                    try:
                        sock_obj.shutdown(socket.SHUT_RDWR)
                    except (OSError, socket.error):
                        pass  # Shutdown not supported or socket already closed

                    # 4. Close socket
                    sock_obj.close()

                except Exception as e:
                    self.logger.warning(f"Error during socket cleanup: {str(e)}")

                finally:
                    setattr(self, sock, None)
                    self.logger.info(f"Socket {sock!r} cleaned up and closed")

            else:
                self.logger.debug(f"No socket {sock!r} to close")

    def _create_hosts(self) -> None:
        """Create NetworkHost objects for the given targets.

        This method parses the network targets and creates NetworkHost objects for each target IP address.
        """
        host_ips = parse_network_targets(self.targets)
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
        """Execute the scan operation and return results.

        This method orchestrates the sending of ICMP packets and the collection of responses.
        It logs the progress and handles any errors that occur during the scan.

        Returns:
            Dict[str, dict]: A dictionary containing the scan results for each target IP address.
        """
        self.logger.info("Start of scanning")
        for sock in ("send_sock", "recv_sock"):
            sock_obj = getattr(self, sock)
            if sock_obj is None:
                raise RuntimeError(
                    f"Socket {sock!r} not initialized. Cannot execute scan."
                )

        # Send ICMP packets to all hosts
        await self._send_packets()

        # Collect responses with timeout
        await self._collect_responses()

        self.logger.info("Scan completed")

        return self._prepare_results()

    async def _send_packets(self) -> None:
        """Send ICMP echo request packets to all hosts.

        This method sends ICMP echo request packets to each target IP address and logs the progress.
        """
        self.logger.info("Sending ICMP echo request packets to all hosts")

        total_hosts = len(self.hosts)
        sent_count = 0

        # Start time for overall progress tracking
        start_time = time.time()

        for ip, host in self.hosts.items():
            self.logger.debug(f"Sending packet to {ip}")
            try:
                # Still using blocking socket.sendto, but in async context
                self.send_sock.sendto(host.packet, (host.ip_address, 0))
                host.mark_sent()

                # Update progress after each successful send
                sent_count += 1
                completion_percentage = (sent_count / total_hosts) * 100

                # Log progress at regular intervals
                if sent_count == total_hosts or sent_count % (total_hosts // 10) == 0:
                    self.logger.info(
                        f"Progress: {sent_count}/{total_hosts} hosts ({round(completion_percentage)}%)"
                    )

                # Use asyncio.sleep instead of time.sleep
                if sent_count < total_hosts:  # Don't sleep after the last packet
                    await asyncio.sleep(self.sending_interval)

            except Exception as e:
                host.mark_error(str(e))
                self.logger.error(f"Failed to send to {ip}: {e}")

        # Final timing statistics
        total_time = time.time() - start_time
        avg_rate = total_hosts / total_time if total_time > 0 else 0

        self.logger.info(
            f"All packets sent ({total_hosts} hosts in {total_time:.2f}s, avg rate: {avg_rate:.2f} packets/sec)"
        )

    async def _collect_responses(self) -> None:
        """
        Asynchronously collect ICMP responses with a timeout.

        This method collects ICMP responses from the target IP addresses until the timeout is reached.
        It uses asyncio to achieve non-blocking behavior.

        Raises:
            RuntimeError: If the response collection fails.
        """
        end_time = time.time() + self.timeout
        self.logger.info(
            f"Collecting responses until {time.strftime('%H:%M:%S', time.localtime(end_time))!r}"
        )

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

            try:
                # Wait for a packet to be available with asyncio's event loop
                packet, addr = await asyncio.wait_for(
                    self._receive_packet(
                        sock=self.recv_sock, size=256
                    ),  # reasonable buffer for ping (typically <100 bytes)
                    timeout=(
                        end_time - time.time()
                    ),  # remaining time for the loop iteration
                )
                src_ip = addr[0]  # Extract source IP

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
        """Helper to get both packet and address with asyncio.

        This method sets up a callback to receive packets asynchronously using asyncio's event loop.

        Args:
            sock (socket.socket): The socket to receive packets from.
            size (int): The size of the buffer to use for receiving packets.

        Returns:
            tuple: A tuple containing the received packet and the address of the sender.
        """
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
        """Prepare the scan results.

        This method prepares the scan results by marking any remaining hosts as unresponsive and formatting the results.

        Returns:
            Dict[str, dict]: A dictionary containing the scan results for each target IP address.
        """
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
