import pytest
import time
import struct
from unittest.mock import patch
from typing import Dict, Any

# Import the module under test
from codestral_ros2_gen.network_scanner.network_host import (
    NetworkHost,
    HostState,
    HostResult,
)


class TestHostResult:
    """Tests for the HostResult class."""

    def test_init_with_rtt(self):
        """Test initialization with RTT value."""
        result = HostResult(rtt_ms=10.5)
        assert result.rtt_ms == 10
        assert result.error is None
        assert result.responded is True

    def test_init_with_error(self):
        """Test initialization with error."""
        result = HostResult(error="Connection failed")
        assert result.rtt_ms is None
        assert result.error == "Connection failed"
        assert result.responded is False

    def test_init_with_no_params(self):
        """Test initialization with no parameters (timeout case)."""
        result = HostResult()
        assert result.rtt_ms is None
        assert result.error is None
        assert result.responded is False

    def test_string_representation(self):
        """Test string conversion for different result types."""
        # Test success case
        success = HostResult(rtt_ms=10.5)
        assert str(success) == "Response time: 10 ms"

        # Test error case
        error = HostResult(error="Connection failed")
        assert str(error) == "Error: Connection failed"

        # Test timeout case
        timeout = HostResult()
        assert str(timeout) == "No response (timeout)"


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
        "timeout_sec": 2.0,
        "packet_size": 64,
    }


def test_initialization_codestral_ros2_gen(host_params):
    """Test host initialization with various parameters with default codestral logger"""
    host = NetworkHost(**host_params)

    # Check instance attributes are set correctly
    assert host.ip_address == "192.168.1.1"
    assert host.timeout_sec == 2.0
    assert host.packet_size == 64
    assert host.send_time == 0.0
    assert host.recv_time == 0.0
    assert host.error_message is None
    assert host.result is None
    assert host.state == HostState.INIT
    assert host.icmp_id == 12345
    assert host.icmp_seq == 1

    # Check logger is set correctly
    assert hasattr(host, "logger")
    assert host.logger is not None
    assert host.logger.name == "root.network_host"

    # Verify packet was created
    assert host.packet is not None
    assert isinstance(host.packet, bytes)
    assert len(host.packet) == 64


def test_initialization_ros2_logger(host_params, mock_ros2_logger):
    """Test host initialization with various parameters with external ROS2 logger"""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)

    # Check logger is set correctly
    assert hasattr(host, "logger")
    assert host.logger is not None
    assert host.logger.get_name() == "ros2_logger"


def test_initialization_without_logger(host_params):
    """Test host initialization without any logger"""
    with patch("codestral_ros2_gen.network_scanner.network_host.default_logger", None):
        with pytest.raises(
            RuntimeError, match="No logger provided and default logger is not set"
        ):
            host = NetworkHost(**host_params)


def test_packet_creation(host_params, mock_ros2_logger):
    """Test ICMP packet creation."""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)

    # Get the packet
    packet = host.packet

    # Verify packet size
    assert len(packet) == host_params["packet_size"]

    # Verify ICMP header fields
    header = packet[:8]  # First 8 bytes contain the header
    icmp_type, icmp_code, checksum, icmp_id, icmp_seq = struct.unpack("!BBHHH", header)

    assert icmp_type == 8  # Echo request
    assert icmp_code == 0
    assert checksum != 0  # Checksum should be calculated
    assert icmp_id == 12345  # Our identifier
    assert icmp_seq == 1  # Sequence number


def test_packet_size_custom(host_params, mock_ros2_logger):
    """Test creating a packet with custom size."""
    params = host_params.copy()
    params["packet_size"] = 128

    host = NetworkHost(**params, logger=mock_ros2_logger)
    assert len(host.packet) == 128


def test_mark_sent(host_params, mock_ros2_logger):
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


def test_mark_responded(host_params, mock_ros2_logger):
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

    # Check result was created
    assert host.result is not None
    assert host.result.responded is True
    assert host.result.rtt_ms is not None
    assert host.result.rtt_ms > 0  # Should be positive

    # Verify debug message was logged
    mock_ros2_logger.debug.assert_any_call(
        f"Response from {host.ip_address} after {host.result.rtt_ms} ms"
    )


def test_mark_responded_with_explicit_time(host_params, mock_ros2_logger):
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
    assert host.result.rtt_ms == 100.0  # (0.1 seconds = 100ms)


def test_mark_timeout(host_params, mock_ros2_logger):
    """Test marking a host as having timed out."""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)
    host.mark_timeout()

    # Check state changed
    assert host.state == HostState.TIMEOUT

    # Check result was created
    assert host.result is not None
    assert host.result.responded is False
    assert host.result.rtt_ms is None
    assert host.result.error is None

    # Verify debug message was logged
    mock_ros2_logger.debug.assert_any_call(
        f"Timeout for {host.ip_address} after {host.timeout_sec} s"
    )


def test_mark_error(host_params, mock_ros2_logger):
    """Test marking a host as having encountered an error."""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)
    error_msg = "Permission denied"
    host.mark_error(error_msg)

    # Check state changed
    assert host.state == HostState.ERROR

    # Check error message was set
    assert host.error_message == error_msg

    # Check result was created
    assert host.result is not None
    assert host.result.responded is False
    assert host.result.rtt_ms is None
    assert host.result.error == error_msg

    # Verify debug message was logged
    mock_ros2_logger.debug.assert_any_call(f"Error for {host.ip_address}: {error_msg}")


def test_is_timed_out_when_not_sent(host_params, mock_ros2_logger):
    """Test timeout detection when packet hasn't been sent."""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)
    # Host starts in INIT state
    assert host.is_timed_out() is False


def test_is_timed_out_when_recently_sent(host_params, mock_ros2_logger):
    """Test timeout detection shortly after sending."""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)
    host.mark_sent()
    # Packet just sent, should not have timed out yet
    assert host.is_timed_out() is False


def test_is_timed_out_after_timeout(host_params, mock_ros2_logger):
    """Test timeout detection after timeout period."""
    # Use a very short timeout for testing
    params = host_params.copy()
    params["timeout_sec"] = 0.1

    host = NetworkHost(**params, logger=mock_ros2_logger)
    host.mark_sent()

    # Wait longer than the timeout period
    time.sleep(0.2)

    # Now should report as timed out
    assert host.is_timed_out() is True
    mock_ros2_logger.debug.assert_any_call(
        f"Host {host.ip_address} timed out (0.20 > 0.1) s"
    )


def test_is_timed_out_when_responded(host_params, mock_ros2_logger):
    """Test timeout detection after host has already responded."""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)
    host.mark_sent()
    host.mark_responded()

    # Host has responded, should not report as timed out
    assert host.is_timed_out() is False


def test_is_timed_out_when_error(host_params, mock_ros2_logger):
    """Test timeout detection after host has encountered an error."""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)
    host.mark_sent()
    host.mark_error("Connection refused")

    # Host had an error, should not report as timed out
    assert host.is_timed_out() is False


def test_string_representation(host_params, mock_ros2_logger):
    """Test string conversion of NetworkHost object."""
    host = NetworkHost(**host_params, logger=mock_ros2_logger)
    assert str(host) == "Host(192.168.1.1, INIT)"

    # Test with different states
    host.mark_sent()
    assert str(host) == "Host(192.168.1.1, SENT)"

    host.mark_responded()
    assert str(host) == "Host(192.168.1.1, RESPONDED)"


def test_calculate_checksum(host_params, mock_ros2_logger):
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
