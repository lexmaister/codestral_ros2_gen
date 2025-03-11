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
from typing import Optional
import logging

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


class NetworkHost:
    """
    Single host handler for network scanning operations.

    This class represents a network host during scanning operations,
    tracking its state and managing ICMP packet generation and validation.
    """

    def __init__(
        self,
        ip_address: str,
        icmp_id=None,
        icmp_seq=1,
        packet_size: int = 64,
        logger: logging.Logger | None = None,
    ):
        """
        Initialize a NetworkHost object.

        Args:
            ip_address: IP address of the host
            icmp_id: ICMP identifier (random if None)
            icmp_seq: ICMP sequence number (default: 1)
            packet_size: Size of the ICMP packet in bytes (default: 64)
            logger: External logger to use (typically a ROS2 logger)
        """
        self._state = HostState.INIT
        self.ip_address = ip_address
        self.packet_size = packet_size
        self.send_time = 0.0
        self.recv_time = 0.0
        self.rtt_ms: int = None
        self.error_message = None

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
        self.logger.debug(f"Created NetworkHost object for {ip_address}")

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

        Raises:
            RuntimeError: If the generated packet size is not equal to provided packet size
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

        # Placeholder payload data
        payload_data = b"abcdefghijklmnopqrstuvwxyz"

        # Pad the payload to reach the requested packet size
        padding_size = max(0, self.packet_size - len(header) - len(payload_data))
        padding = bytes([i & 0xFF for i in range(padding_size)])

        # Combine the payload
        payload = payload_data + padding

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
            raise RuntimeError(
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
            self.logger.warning("Calculated checksum is 0")

        return answer

    def handle_response(self, packet: bytes) -> None:
        """
        Validate ICMP echo reply packet for this host.

        Args:
            packet (bytes): The received ICMP packet.

        Side Effects:
            Updates the host state and response time based on the received packet.
        """
        self.logger.debug(f"Handling response for {self.ip_address}")

        # Skip IP header (length in bytes = first 4 bits * 4)
        ip_header_length = (packet[0] & 0x0F) * 4
        icmp_packet = packet[ip_header_length:]

        if len(icmp_packet) < 8:
            self.mark_error("Received ICMP packet is too short")
            return

        # Parse ICMP Echo Reply header (Type 0, Code 0)
        icmp_type, icmp_code, _, recv_id, recv_seq = struct.unpack(
            "!BBHHH", icmp_packet[:8]
        )

        # Filter out Echo Requests with our ID - these are our own packets
        if icmp_type == 8 and recv_id == self.icmp_id:
            self.logger.debug(
                f"Filtering out our own outgoing packet (seq: {recv_seq})"
            )
            self.mark_error("Received our own outgoing packet")
            return

        if icmp_type != 0 or icmp_code != 0:
            self.mark_error(
                f"Received ICMP packet is not an Echo Reply - type/code: {icmp_type}/{icmp_code}"
            )
            return

        if recv_id != self.icmp_id or recv_seq != self.icmp_seq:
            self.mark_error("Received ICMP packet has incorrect ID or Sequence")
            return

        self.mark_responded()

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

        Sets the state to RESPONDED, records the receive time, and calculates the RTT.

        Args:
            recv_time: Time when response was received, defaults to current time
        """
        self._state = HostState.RESPONDED
        self.recv_time = recv_time or time.time()
        self.rtt_ms = round((self.recv_time - self.send_time) * 1000)
        self.logger.debug(f"Response from {self.ip_address} after {self.rtt_ms} ms")

    def mark_timeout(self) -> None:
        """
        Mark the host as having timed out (no response received).
        """
        self._state = HostState.TIMEOUT
        self.logger.debug(f"Timeout for {self.ip_address}")

    def mark_error(self, error_message: str) -> None:
        """
        Mark the host as having encountered an error.

        Args:
            error_message: Description of the error
        """
        self._state = HostState.ERROR
        self.error_message = error_message
        self.logger.error(f"Error for {self.ip_address}: {error_message}")

    def __str__(self) -> str:
        """
        Return a string representation of the host.

        Returns:
            str: String description of the host
        """
        return f"Host({self.ip_address}, {self._state.name})"
