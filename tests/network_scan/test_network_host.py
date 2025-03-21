import pytest
import time
import struct
import logging
from unittest.mock import patch
from typing import Dict, Any

# Import the module under test
from codestral_ros2_gen.network_scan.network_host import (
    NetworkHost,
    HostState,
)


class TestHostState:
    """Tests for the HostState enum."""

    def test_enum_values(self):
        """Test that all expected states are defined with unique values."""
        states = [
            HostState.INIT,
            HostState.SENT,
            HostState.RESPONDED,
            HostState.TIMEOUT,
            HostState.ERROR,
        ]

        # Check for uniqueness
        assert len(set(state.value for state in states)) == len(states)

        # Check specific states exist
        assert hasattr(HostState, "INIT")
        assert hasattr(HostState, "SENT")
        assert hasattr(HostState, "RESPONDED")
        assert hasattr(HostState, "TIMEOUT")
        assert hasattr(HostState, "ERROR")


@pytest.fixture
def host_params() -> Dict[str, Any]:
    """Fixture providing standard parameters for host creation."""
    return {
        "ip_address": "192.168.1.1",
        "icmp_id": 12345,
        "icmp_seq": 1,
        "packet_size": 64,
    }


class TestHostInitialization:
    """Tests for the NetworkHost class initialization."""

    def test_initialization_codestral_ros2_gen(self, host_params):
        """Test host initialization with various parameters with default codestral logger"""
        host = NetworkHost(**host_params)

        # Check instance attributes are set correctly
        assert host.ip_address == "192.168.1.1"
        assert host.packet_size == 64
        assert host.send_time == 0.0
        assert host.recv_time == 0.0
        assert host.error_message is None
        assert host.state == HostState.INIT
        assert host.icmp_id == 12345
        assert host.icmp_seq == 1

        # Check logger is set correctly
        assert hasattr(host, "logger")
        assert host.logger is not None
        assert host.logger.name == "root.nscan.host"

        # Verify packet was created
        assert host.packet is not None
        assert isinstance(host.packet, bytes)
        assert len(host.packet) == 64

    def test_initialization_ros2_logger(self, host_params, mock_ros2_logger):
        """Test host initialization with various parameters with external ROS2 logger"""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)

        # Check logger is set correctly
        assert hasattr(host, "logger")
        assert host.logger is not None
        assert host.logger.get_name() == "ros2_logger"

    def test_initialization_without_logger(self, host_params):
        """Test host initialization without any logger"""
        with patch("codestral_ros2_gen.network_scan.network_host.nscan_logger", None):
            with pytest.raises(
                RuntimeError, match="No logger provided and default logger is not set"
            ):
                host = NetworkHost(**host_params)

    def test_packet_creation(self, host_params, mock_ros2_logger):
        """Test ICMP packet creation."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)

        # Get the packet
        packet = host.packet

        # Verify packet size
        assert len(packet) == host_params["packet_size"]

        # Verify ICMP header fields
        header = packet[:8]  # First 8 bytes contain the header
        icmp_type, icmp_code, checksum, icmp_id, icmp_seq = struct.unpack(
            "!BBHHH", header
        )

        assert icmp_type == 8  # Echo request
        assert icmp_code == 0
        assert checksum != 0  # Checksum should be calculated
        assert icmp_id == 12345  # Our identifier
        assert icmp_seq == 1  # Sequence number

    def test_packet_size_custom(self, host_params, mock_ros2_logger):
        """Test creating a packet with custom size."""
        params = host_params.copy()
        params["packet_size"] = 128

        host = NetworkHost(**params, logger=mock_ros2_logger)
        assert len(host.packet) == 128

    def test_calculate_checksum(self, host_params, mock_ros2_logger):
        """Test the checksum calculation method."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)

        # Test with known values
        test_data = b"\x08\x00\x00\x00\x12\x34\x56\x78"  # Example ICMP header
        checksum = host._calculate_checksum(test_data)

        # The checksum should be a 16-bit value
        assert 0 <= checksum <= 0xFFFF

        # We could also verify against a pre-calculated expected value
        # This would require manual calculation of the expected value

        # Test with empty data
        empty_checksum = host._calculate_checksum(b"")
        assert empty_checksum == 0xFFFF  # Checksum of empty data is all 1's


class TestMarks:
    """Test marking states of the NetworkHost."""

    def test_mark_sent(self, host_params, mock_ros2_logger):
        """Test marking a host as having had a packet sent."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)

        # Record time before and after to bound the send_time
        before = time.time()
        host.mark_sent()
        after = time.time()

        # Check state changed
        assert host.state == HostState.SENT

        # Check send_time was set correctly (between before and after)
        assert before <= host.send_time <= after

        # Verify debug message was logged
        mock_ros2_logger.debug.assert_any_call(
            f"Packet sent to {host.ip_address} at {host.send_time}"
        )

    def test_mark_responded(self, host_params, mock_ros2_logger):
        """Test marking a host as having responded."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)

        # Set send time to a known value
        host.mark_sent()
        send_time = host.send_time

        # Wait a small amount to ensure time difference
        time.sleep(0.01)

        # Mark as responded
        host.mark_responded()

        # Check state changed
        assert host.state == HostState.RESPONDED

        # Check recv_time was set
        assert host.recv_time > send_time

        assert host.rtt_ms > 0  # Should be positive

        # Verify debug message was logged
        mock_ros2_logger.debug.assert_any_call(
            f"Round transfer time for {host.ip_address}: {host.rtt_ms} ms"
        )

    def test_mark_responded_with_explicit_time(self, host_params, mock_ros2_logger):
        """Test marking a host as responded with explicit receive time."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)

        # Set send time
        host.mark_sent()
        send_time = host.send_time

        # Set explicit receive time
        recv_time = send_time + 0.1  # 100ms later
        host.mark_responded(recv_time)

        # Check recv_time was set correctly
        assert host.recv_time == recv_time

        # Check RTT calculation
        assert host.rtt_ms == 100.0  # (0.1 seconds = 100ms)

    def test_mark_timeout(self, host_params, mock_ros2_logger):
        """Test marking a host as having timed out."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)
        host.mark_timeout()

        # Check state changed
        assert host.state == HostState.TIMEOUT

        assert host.rtt_ms is None

        # Verify debug message was logged
        mock_ros2_logger.debug.assert_any_call(f"Timeout for {host.ip_address}")

    def test_mark_error(self, host_params, mock_ros2_logger):
        """Test marking a host as having encountered an error."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)
        error_msg = "Permission denied"
        host.mark_error(error_msg)

        # Check state changed
        assert host.state == HostState.ERROR

        # Check error message was set
        assert host.error_message == error_msg

        # Verify debug message was logged
        mock_ros2_logger.error.assert_any_call(
            f"Error for {host.ip_address}: {error_msg}"
        )


class TestHandlingResponce:
    """Test handling ICMP response packets."""

    @staticmethod
    def _create_icmp_packet(
        icmp_type: int = 0,
        icmp_code: int = 0,
        icmp_id: int = 12345,
        icmp_seq: int = 1,
        data: bytes = b"PING",
    ) -> bytes:
        """Create a mock ICMP packet."""
        icmp_header = struct.pack("!BBHHH", icmp_type, icmp_code, 0, icmp_id, icmp_seq)
        return icmp_header + data

    def test_handle_response_valid_packet(self, host_params):
        """Test handling a valid ICMP response packet."""
        host = NetworkHost(**host_params)
        valid_packet = self._create_icmp_packet(
            icmp_id=host.icmp_id, icmp_seq=host.icmp_seq
        )
        host.mark_sent()
        time.sleep(1.1)

        host.handle_response(valid_packet)
        print(str(host))

        assert host.state == HostState.RESPONDED
        assert host.rtt_ms is not None
        assert host.rtt_ms > 1000  # Ensure RTT is greater than 1 second

    def test_handle_response_invalid_size(self, host_params, mock_ros2_logger):
        """Test handling a packet with incorrect size."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)
        invalid_packet = b"X" * 32  # Too small

        host.handle_response(invalid_packet)

        assert host.state == HostState.ERROR
        mock_ros2_logger.error.assert_called_with(
            f"Error for {host.ip_address}: Received ICMP packet is too short"
        )

    def test_handle_response_malformed_packet(self, host_params, mock_ros2_logger):
        """Test handling a malformed packet with correct size but invalid content."""
        host = NetworkHost(**host_params, logger=mock_ros2_logger)
        # Create a packet with correct size but invalid ICMP structure
        malformed_packet = b"Y" * 64

        host.handle_response(malformed_packet)

        assert host.state == HostState.ERROR
        assert (
            host.error_message
            == "Received ICMP packet is not an Echo Reply - type/code: 89/89"
        )
        mock_ros2_logger.error.assert_called_with(
            f"Error for {host.ip_address}: {host.error_message}"
        )


class TestCheckSumCalc:
    """Test the checksum calculation."""

    @pytest.fixture
    def sample_ping_packet(self):
        """Return a real ICMP echo request packet with known checksum."""
        # Values from Wireshark capture
        return {
            "type": 8,  # Echo request
            "code": 0,
            "checksum": 0x8730,
            "identifier": 0x0F51,
            "sequence": 1,
        }

    @pytest.fixture
    def network_host(self):
        """Create a NetworkHost instance with proper configuration."""
        host = NetworkHost("192.168.10.253")
        return host

    def test_checksum_calculation(self, sample_ping_packet):
        """Test that our checksum calculation matches Wireshark's value."""
        # Configure host with values from the captured packet
        host = NetworkHost(
            "192.168.10.253",
            icmp_id=sample_ping_packet["identifier"],
            icmp_seq=sample_ping_packet["sequence"],
        )
        packet = host._create_icmp_packet()

        # Extract calculated checksum from positions 2-3
        calculated_checksum = (packet[2] << 8) | packet[3]

        # Assert checksum matches Wireshark value
        assert (
            calculated_checksum == sample_ping_packet["checksum"]
        ), f"Checksum mismatch: got 0x{calculated_checksum:04x}, expected 0x{sample_ping_packet['checksum']:04x}"
