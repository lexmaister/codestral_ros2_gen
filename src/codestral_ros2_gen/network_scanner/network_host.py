"""
Network Host Module for network scanning operations.

This module provides classes for network host representation and handling
during network scanning operations. It includes the host state tracking
and ICMP packet generation capabilities.
"""

import random
import struct
import time
from enum import Enum, auto
from typing import Optional, Any

from .utils import get_codestral_ros2_gen_logger

crg_logger = get_codestral_ros2_gen_logger()


class HostState(Enum):
    """
    Enumeration of possible network host states during scanning.

    Attributes:
        INIT: Initial state when host is created
        SENT: ICMP packet has been sent to the host
        RESPONDED: Host has responded to the ICMP packet
        TIMEOUT: Host did not respond within the timeout period
        ERROR: An error occurred during scanning
    """

    INIT = auto()
    SENT = auto()
    RESPONDED = auto()
    TIMEOUT = auto()
    ERROR = auto()


class HostResult:
    """
    Class to store the result of a host scan.

    This class encapsulates the result data from a network host scan,
    including response time and error information.

    Attributes:
        rtt_ms (float): Round-trip time in milliseconds - would be rounded to integer, or None if no response
        error (str): Error message if an error occurred, or None
        responded (bool): Whether the host responded successfully
    """

    def __init__(self, rtt_ms: Optional[float] = None, error: Optional[str] = None):
        """
        Initialize a HostResult object.

        Args:
            rtt_ms: Round-trip time in milliseconds, or None if no response
            error: Error message if an error occurred, or None
        """
        self.rtt_ms = round(rtt_ms) if rtt_ms is not None else None
        self.error = error
        self.responded = rtt_ms is not None and error is None

    def __str__(self) -> str:
        """
        Return a string representation of the result.

        Returns:
            str: String description of the result
        """
        if self.responded:
            return f"Response time: {self.rtt_ms} ms"
        elif self.error:
            return f"Error: {self.error}"
        else:
            return "No response (timeout)"


class NetworkHost:
    """
    Single host handler for network scanning operations.

    This class represents a network host during scanning operations,
    tracking its state and managing ICMP packet generation.
    """

    def __init__(
        self,
        ip_address: str,
        icmp_id=None,
        icmp_seq=1,
        timeout_sec: float = 1.0,
        packet_size: int = 64,
        logger: Optional[Any] = None,
    ):
        """
        Initialize a NetworkHost object.

        Args:
            ip_address: IP address of the host
            icmp_id: ICMP identifier (random if None)
            icmp_seq: ICMP sequence number (default: 1)
            timeout_sec: Timeout in seconds to wait for response
            packet_size: Size of the ICMP packet in bytes
            logger: External logger to use (typically a ROS2 logger)
        """
        self._state = HostState.INIT
        self.ip_address = ip_address
        self.timeout_sec = timeout_sec
        self.packet_size = packet_size
        self.send_time = 0.0
        self.recv_time = 0.0
        self.error_message = None
        self.result = None

        # Generate random identifier if none provided
        if icmp_id is None:
            # Generate random 16-bit integer (0-65535)
            self.icmp_id = random.randint(0, 0xFFFF)
        else:
            self.icmp_id = icmp_id

        self.icmp_seq = icmp_seq

        if logger is not None:
            self.logger = logger
        elif crg_logger is not None:
            self.logger = crg_logger
        else:
            raise RuntimeError("No logger provided and default logger is not set")

        self.packet = self._create_icmp_packet()
        self.logger.debug(f"Created NetworkHost for {ip_address}")

    @property
    def state(self) -> HostState:
        """
        Get the current state of the host.

        Returns:
            HostState: Current state of the host
        """
        return self._state

    def _create_icmp_packet(self) -> bytes:
        """
        Create an ICMP echo request packet.

        Returns:
            bytes: The ICMP packet ready to be sent
        """
        self.logger.debug(f"Creating ICMP packet for {self.ip_address}")

        # ICMP type 8 (echo request), code 0
        icmp_type = 8
        icmp_code = 0
        icmp_checksum = 0

        # Create header without checksum
        header = struct.pack(
            "!BBHHH", icmp_type, icmp_code, icmp_checksum, self.icmp_id, self.icmp_seq
        )

        # Create payload (pad to desired size)
        payload_size = max(0, self.packet_size - len(header))
        payload = bytes([i & 0xFF for i in range(payload_size)])

        # Calculate checksum on the header and payload
        full_packet = header + payload
        checksum = self._calculate_checksum(full_packet)

        # Insert checksum into header
        header = struct.pack(
            "!BBHHH", icmp_type, icmp_code, checksum, self.icmp_id, self.icmp_seq
        )
        packet = header + payload

        # Validate the packet
        if len(packet) != self.packet_size:
            self.logger.debug(
                f"Packet size mismatch: expected {self.packet_size}, got {len(packet)}"
            )

        return packet

    def _calculate_checksum(self, data: bytes) -> int:
        """
        Calculate the checksum for an ICMP packet.

        Args:
            data: The data to calculate the checksum for

        Returns:
            int: Calculated checksum value
        """
        sum = 0
        countTo = (len(data) // 2) * 2
        count = 0

        while count < countTo:
            val = data[count + 1] * 256 + data[count]
            sum = sum + val
            sum = sum & 0xFFFFFFFF
            count = count + 2

        if countTo < len(data):
            sum = sum + data[len(data) - 1]
            sum = sum & 0xFFFFFFFF

        sum = (sum >> 16) + (sum & 0xFFFF)
        sum = sum + (sum >> 16)
        answer = ~sum & 0xFFFF

        # Validate the checksum
        if answer == 0:
            self.logger.debug("Warning: Calculated checksum is 0")

        return answer

    def mark_sent(self) -> None:
        """
        Mark the host as having had a packet sent to it.

        Records the current time as the send time and updates the state.
        """
        self._state = HostState.SENT
        self.send_time = time.time()
        self.logger.debug(f"Packet sent to {self.ip_address} at {self.send_time}")

    def mark_responded(self, recv_time: Optional[float] = None) -> None:
        """
        Mark the host as having responded to the packet.

        Args:
            recv_time: Time when response was received, defaults to current time
        """
        self._state = HostState.RESPONDED
        self.recv_time = recv_time or time.time()
        rtt_ms = (self.recv_time - self.send_time) * 1000
        self.result = HostResult(rtt_ms=rtt_ms)
        self.logger.debug(
            f"Response from {self.ip_address} after {self.result.rtt_ms} ms"
        )

    def mark_timeout(self) -> None:
        """
        Mark the host as having timed out (no response received).
        """
        self._state = HostState.TIMEOUT
        self.result = HostResult()
        self.logger.debug(f"Timeout for {self.ip_address} after {self.timeout_sec} s")

    def mark_error(self, error_message: str) -> None:
        """
        Mark the host as having encountered an error.

        Args:
            error_message: Description of the error
        """
        self._state = HostState.ERROR
        self.error_message = error_message
        self.result = HostResult(error=error_message)
        self.logger.debug(f"Error for {self.ip_address}: {error_message}")

    def is_timed_out(self) -> bool:
        """
        Check if the host has timed out.

        Returns:
            bool: True if the host is in SENT state and the timeout period has elapsed
        """
        if self._state != HostState.SENT:
            return False

        elapsed = time.time() - self.send_time
        timed_out = elapsed > self.timeout_sec

        if timed_out:
            self.logger.debug(
                f"Host {self.ip_address} timed out ({elapsed:.2f} > {self.timeout_sec}) s"
            )

        return timed_out

    def __str__(self) -> str:
        """
        Return a string representation of the host.

        Returns:
            str: String description of the host
        """
        return f"Host({self.ip_address}, {self._state.name})"
