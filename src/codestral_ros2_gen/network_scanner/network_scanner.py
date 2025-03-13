import asyncio
import logging
from typing import Dict, Optional
import time

from .utils import get_codestral_ros2_gen_logger
from .scan_operation import ScanOperation

crg_logger = get_codestral_ros2_gen_logger()


class NetworkScanner:
    """
    Network scanner that performs scans and handles formatted results.
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
        elif crg_logger is not None:
            self.logger = crg_logger
        else:
            raise RuntimeError("No logger provided and default logger is not set")

    def scan(self, targets: str) -> Dict[str, dict]:
        """
        Perform a scan synchronously for the specified targets.

        Args:
            targets: Network targets in CIDR notation or as comma-separated IPs.

        Returns:
            Dict[str, dict]: Processed scan results with fields `state`, `response_time_ms`, and `error`.
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
        Perform an asynchronous scan.

        Args:
            targets: Network targets in CIDR notation or as comma-separated IPs.

        Returns:
            Dict[str, dict]: Processed scan results with fields `state`, `response_time_ms`, and `error`.
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
        Format the scan results in a table format.

        Args:
            results: Scan results returned by `scan()` or `_scan_async()`.
            show_all: Whether to show all hosts or only those with state "UP".

        Returns:
            str: Formatted results as a string.
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
