from enum import Enum
import struct
from dataclasses import dataclass
from typing import Optional
import time
from logging import getLogger


logger = getLogger(__name__)


class HostState(Enum):
    INIT = "init"
    SENT = "sent"
    RESPONDED = "responded"
    TIMEOUT = "timeout"
    ERROR = "error"


@dataclass
class HostResult:
    is_alive: bool
    response_time: Optional[float] = None
    error: Optional[str] = None


class NetworkHost:
    """Single host handler for network scanning operations"""

    def __init__(self, ip_address: str):
        self.ip_address: str = ip_address
        self.state: HostState = HostState.INIT
        self.sequence: int = 0
        self.packet: Optional[bytes] = None
        self.send_time: Optional[float] = None
        self.result: HostResult = HostResult(is_alive=False)

        # Pre-calculate packet
        self._prepare_packet()

    def _prepare_packet(self) -> None:
        """Pre-calculate ICMP packet for this host"""
        self.sequence = int(time.time() * 1000) & 0xFFFF
        header = struct.pack("!BBHHH", 8, 0, 0, id(self) & 0xFFFF, self.sequence)
        data = b"codestral_scan"
        checksum = self._calculate_checksum(header + data)
        self.packet = (
            struct.pack("!BBHHH", 8, 0, checksum, id(self) & 0xFFFF, self.sequence)
            + data
        )

    def _calculate_checksum(self, data: bytes) -> int:
        """Calculate ICMP checksum"""
        if len(data) % 2 == 1:
            data += b"\0"
        words = struct.unpack("!%dH" % (len(data) // 2), data)
        checksum = sum(words)
        checksum = (checksum >> 16) + (checksum & 0xFFFF)
        checksum += checksum >> 16
        return ~checksum & 0xFFFF

    def validate_response(self, icmp_header: bytes, receive_time: float) -> bool:
        """Validate received ICMP response"""
        if len(icmp_header) < 8:
            return False

        type, code, _, p_id, sequence = struct.unpack("!BBHHH", icmp_header[:8])

        if p_id != (id(self) & 0xFFFF) or sequence != self.sequence:
            return False

        if type != 0 or code != 0:
            return False

        if self.send_time:
            self.result.response_time = receive_time - self.send_time
            self.result.is_alive = True

        return True

    def mark_sent(self) -> None:
        """Mark host as waiting for response"""
        self.send_time = time.time()
        self.state = HostState.SENT

    def mark_responded(self) -> None:
        """Mark host as responded"""
        self.state = HostState.RESPONDED

    def mark_timeout(self) -> None:
        """Mark host as timeout"""
        self.state = HostState.TIMEOUT

    def mark_error(self, error_msg: str) -> None:
        """Mark host as errored with specific error message"""
        self.state = HostState.ERROR
        self.result.error = error_msg
        self.result.is_alive = False
        self.result.response_time = None
        logger.debug(f"Host {self.ip_address} marked as error: {error_msg}")
