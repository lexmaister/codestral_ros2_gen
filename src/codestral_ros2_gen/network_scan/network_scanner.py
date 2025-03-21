import asyncio
import logging
from typing import Dict, Optional
import time

from .scan_operation import ScanOperation
from . import nscan_logger

if nscan_logger is not None:
    from codestral_ros2_gen.utils.init_pkg_logger import init_pkg_logger


class NetworkScanner:
    """
    NetworkScanner performs a complete network scan by:
        * Setting scan parameters.
        * Utilizing a ScanOperation context manager that configures both send and receive sockets.
        * Sending ICMP packets synchronously and then collecting responses asynchronously.
        * Formatting and reporting the results upon scan completion.

    Example usage:
        >>> from codestral_ros2_gen.network_scan.network_scanner import NetworkScanner
        >>> scanner = NetworkScanner()
        >>> hosts = scanner.scan("192.168.10.0/24")
        >>> print(scanner.format_results(hosts, show_all=False))
    """

    def __init__(
        self,
        timeout: float = 5.0,
        packet_size: int = 64,
        sending_interval: float = 0.05,
        send_buff_size: int = 65536,
        recv_buff_size: int = 131072,
        logger: Optional[logging.Logger] = None,
    ):
        """
        Initialize the network scanner with configurable parameters.

        Args:
            timeout: Timeout for response in seconds.
            packet_size: Size of the ICMP echo request packets.
            sending_interval: Interval between sending packets.
            send_buff_size: Buffer size for sending socket.
            recv_buff_size: Buffer size for receiving socket.
            logger: Optional logger instance.
        """
        self.timeout = timeout
        self.packet_size = packet_size
        self.sending_interval = sending_interval
        self.send_buff_size = send_buff_size
        self.recv_buff_size = recv_buff_size
        self.scan_start = None

        if logger is not None:
            self.logger = logger
        elif nscan_logger is not None:
            self.logger = init_pkg_logger()
            self.logger.name = nscan_logger
        else:
            raise RuntimeError("No logger provided and default logger is not set")

    def scan(self, targets: str) -> Dict[str, dict]:
        """
        Execute a network scan synchronously for the specified targets.

        This method uses asyncio.run to run an asynchronous scan operation which:
         - Configures the scan environment within a ScanOperation context.
         - Synchronously sends ICMP packets via a blocking send socket.
         - Asynchronously gathers responses via a non-blocking receive socket.

        Args:
            targets: Network targets in CIDR format or as comma-separated IP addresses.

        Returns:
            A dictionary mapping host IP addresses to their scan results (state, response time, error).
        """
        try:
            self.scan_start = time.time()
            self.logger.info(f"Starting network scan for {targets}")
            return asyncio.run(self._scan_async(targets))

        except Exception as e:
            self.logger.error(f"Scan failed: {e}")
            # suppress the exception and return an empty results dictionary
            return {}

        finally:
            self.scan_time = time.time() - self.scan_start if self.scan_start else 0
            self.logger.info(f"Scan completed in {self.scan_time:.2f} seconds")
            self.scan_start = None

    async def _scan_async(self, targets: str) -> Dict[str, dict]:
        """
        Asynchronously execute the network scan operation.

        Within the ScanOperation context, this method first configures both send and receive sockets,
        then sends ICMP packets synchronously, and finally initiates an asynchronous loop to collect responses.

        Args:
            targets: Network targets in CIDR or comma-separated IP format.

        Returns:
            A dictionary of scan results for each target host.
        """
        async with ScanOperation(
            targets=targets,
            scanner=self,
            timeout=self.timeout,
            packet_size=self.packet_size,
            sending_interval=self.sending_interval,
            send_buff_size=self.send_buff_size,
            recv_buff_size=self.recv_buff_size,
            logger=self.logger,
        ) as operation:
            return await operation.execute()

    def format_results(self, results: Dict[str, dict], show_all: bool = False) -> str:
        """
        Format the scan results into a human-readable table.

        The table includes headers for IP address, state, response time (ms), and any error messages.
        An appended summary shows the total scanned hosts, counts for hosts that are up, down, with errors,
        and the total scan duration. If show_all is False, only hosts with state "UP" are displayed.

        Args:
            results: The dictionary of scan results returned by scan().
            show_all: Whether to display all hosts or only those with state "UP".

        Returns:
            A formatted string report of the scan results.
        """
        self.logger.info("Formatting scan results")
        out = ""
        # Filter results to show only active hosts ("UP") if `show_all` is False
        filtered_results = (
            results
            if show_all
            else {ip: data for ip, data in results.items() if data["state"] == "UP"}
        )

        if not filtered_results:
            self.logger.info("No hosts to display.")
            return out

        out += " Network Scan Results ".center(80, "=") + "\n"
        out += f"{'IP Address':<16} {'State':<10} {'Response Time (ms)':<20} {'Error':<32}\n"
        out += "-" * 80 + "\n"

        for ip, data in sorted(filtered_results.items()):
            state = data["state"]
            response_time = (
                f"{data['response_time_ms']} ms"
                if data["response_time_ms"] is not None
                else "-"
            )
            error = data["error"] if data["error"] else ""

            out += f"{ip:<16} {state:<10} {response_time:<20} {error:<32}\n"

        out += " Summary ".center(80, "=") + "\n"
        out += f"Total hosts scanned: {len(results)}\n"
        out += f"Hosts up: {sum(1 for data in results.values() if data['state'] == 'UP')}\n"
        out += f"Hosts down: {sum(1 for data in results.values() if data['state'] == 'DOWN')}\n"
        out += f"Hosts with errors: {sum(1 for data in results.values() if data['error'])}\n"
        out += f"Scan duration: {self.scan_time:.2f} seconds\n"
        out += "=" * 80 + "\n"

        return out
