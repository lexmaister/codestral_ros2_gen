import pytest
import socket
from unittest.mock import Mock, patch
import time

from codestral_ros2_gen.network_scanner.scan_operation import ScanOperation
from codestral_ros2_gen.network_scanner.network_host import (
    NetworkHost,
    HostState,
)


@pytest.fixture
def basic_scan_params():
    """Fixture providing basic parameters for scan operations."""
    return {
        "targets": "192.168.1.0/24",
        "timeout": 1.0,
        "packet_size": 64,
        "sending_interval": 0.05,
        "send_buff_size": 65536,
        "recv_buff_size": 131072,
    }


@pytest.fixture
def mock_socket():
    """Fixture providing a mock socket."""
    mock = Mock(spec=socket.socket)
    mock.settimeout = Mock()
    mock.setsockopt = Mock()
    mock.close = Mock()
    return mock


class TestScanOperationInit:
    """Test cases for ScanOperation initialization."""

    def test_init_with_default_params(self, basic_scan_params, mock_ros2_logger):
        """Test initialization with default parameters."""
        scan_op = ScanOperation(
            targets=basic_scan_params["targets"], logger=mock_ros2_logger
        )

        assert scan_op.targets == basic_scan_params["targets"]
        assert scan_op.timeout == 1.0  # Default value
        assert scan_op.packet_size == 64  # Default value
        assert scan_op.logger == mock_ros2_logger

    def test_init_with_custom_params(self, basic_scan_params, mock_ros2_logger):
        """Test initialization with custom parameters."""
        custom_timeout = 2.0
        custom_packet_size = 128

        scan_op = ScanOperation(
            targets=basic_scan_params["targets"],
            timeout=custom_timeout,
            packet_size=custom_packet_size,
            logger=mock_ros2_logger,
        )

        assert scan_op.targets == basic_scan_params["targets"]
        assert scan_op.timeout == custom_timeout
        assert scan_op.packet_size == custom_packet_size

    def test_init_with_scanner_object(self, basic_scan_params, mock_ros2_logger):
        """Test initialization with a scanner object that provides configuration."""
        mock_scanner = Mock()
        mock_scanner.timeout = 3.0
        mock_scanner.packet_size = 256

        scan_op = ScanOperation(
            targets=basic_scan_params["targets"],
            scanner=mock_scanner,
            logger=mock_ros2_logger,
        )

        assert scan_op.scanner == mock_scanner
        # Would need to check if scanner attributes are used properly in the real implementation


@pytest.mark.asyncio
class TestScanOperationContextManager:
    """Test cases for ScanOperation as an async context manager."""

    @patch.object(ScanOperation, "_configure_sockets")
    @patch.object(ScanOperation, "_create_hosts")
    async def test_async_enter(
        self,
        mock_create_hosts,
        mock_configure_sockets,
        basic_scan_params,
        mock_ros2_logger,
    ):
        """Test the __aenter__ method sets up the scan operation correctly."""
        scan_op = ScanOperation(
            targets=basic_scan_params["targets"], logger=mock_ros2_logger
        )

        result = await scan_op.__aenter__()

        mock_configure_sockets.assert_called_once()
        mock_create_hosts.assert_called_once()
        assert result == scan_op

    @patch.object(ScanOperation, "_close_sockets")
    async def test_async_exit(
        self, mock_close_socket, basic_scan_params, mock_ros2_logger
    ):
        """Test the __aexit__ method cleans up resources."""
        scan_op = ScanOperation(
            targets=basic_scan_params["targets"], logger=mock_ros2_logger
        )

        # Mock the socket attributes
        scan_op.send_sock = Mock(spec=socket.socket)
        scan_op.recv_sock = Mock(spec=socket.socket)
        scan_op.hosts = {"192.168.1.1": Mock()}

        await scan_op.__aexit__(None, None, None)

        # Verify _close_socket was called
        mock_close_socket.assert_called_once()

        # Verify hosts dictionary was cleared
        assert len(scan_op.hosts) == 0


class TestScanOperationSockets:
    """Test cases for socket configuration and handling."""

    @patch("socket.socket")
    def test_configure_sockets(
        self, mock_socket_class, basic_scan_params, mock_ros2_logger
    ):
        """Test socket configuration."""
        mock_socket_instance = Mock(spec=socket.socket)
        mock_socket_instance.setblocking = Mock()
        mock_socket_instance.setsockopt = Mock()
        mock_socket_class.return_value = mock_socket_instance

        scan_op = ScanOperation(
            targets=basic_scan_params["targets"],
            send_buff_size=basic_scan_params["send_buff_size"],
            recv_buff_size=basic_scan_params["recv_buff_size"],
            logger=mock_ros2_logger,
        )

        scan_op._configure_sockets()

        # Check socket creation and configuration
        assert mock_socket_class.call_count == 2  # two sockets created
        assert mock_socket_instance.setsockopt.call_count > 0
        assert scan_op.send_sock is not None
        assert scan_op.recv_sock is not None

    @patch("socket.socket")
    def test_configure_sockets_error(
        self, mock_socket_class, basic_scan_params, mock_ros2_logger
    ):
        """Test error handling in socket configuration."""
        mock_socket_class.side_effect = socket.error("Socket creation error")

        scan_op = ScanOperation(
            targets=basic_scan_params["targets"], logger=mock_ros2_logger
        )

        with pytest.raises(RuntimeError) as exc_info:
            scan_op._configure_sockets()

        assert "Failed to create sockets" in str(exc_info.value)


class TestScanOperationHosts:
    """Test cases for host creation and management."""

    def test_create_hosts(self, mock_ros2_logger):
        """Test host creation for a network range."""
        # Setup mock IP addresses
        mock_addresses = "192.168.1.1, 192.168.1.2, 192.168.1.3"

        scan_op = ScanOperation(targets=mock_addresses, logger=mock_ros2_logger)

        scan_op._create_hosts()

        # Verify hosts were created
        assert len(scan_op.hosts) == 3
        assert all(isinstance(host, NetworkHost) for ip, host in scan_op.hosts.items())
        assert all(host.ip_address in mock_addresses for host in scan_op.hosts.values())


@pytest.mark.asyncio
class TestScanOperationExecution:
    """Tests for ScanOperation.execute method."""

    @patch.object(ScanOperation, "_send_packets")
    @patch.object(ScanOperation, "_collect_responses")  # Use the correct method name
    async def test_execute(
        self, mock_collect_responses, mock_send_packets, mock_ros2_logger
    ):
        """Test the execute method performs the scan correctly."""
        # Create scan operation
        scan_op = ScanOperation(targets="192.168.1.1", logger=mock_ros2_logger)

        # Set up mocked sockets
        scan_op.send_sock = Mock(spec=socket.socket)
        scan_op.recv_sock = Mock(spec=socket.socket)

        # Set up mock host
        mock_host = Mock()
        mock_host.state = HostState.RESPONDED
        mock_host.ip_address = "192.168.1.1"
        mock_host.rtt_ms = 10
        scan_op.hosts = {"192.168.1.1": mock_host}

        # Execute the scan
        results = await scan_op.execute()

        # Verify methods were called
        mock_send_packets.assert_called_once()
        mock_collect_responses.assert_called_once()

        # Verify results
        assert "192.168.1.1" in results
        assert results["192.168.1.1"]["state"] == "UP"

    @pytest.mark.asyncio
    async def test_execute_with_send_error(self, mock_ros2_logger):
        """Test that execute properly handles errors during packet sending."""
        # Create scan operation
        scan_op = ScanOperation(targets="192.168.1.1", logger=mock_ros2_logger)

        # Set up mocked sockets to avoid the 'socket not initialized' error
        scan_op.send_sock = Mock(spec=socket.socket)
        scan_op.recv_sock = Mock(spec=socket.socket)

        # Mock the _send_packets method to raise an error
        with patch.object(
            scan_op, "_send_packets", side_effect=RuntimeError("Sending error")
        ):

            # Execute the scan and expect exception
            with pytest.raises(RuntimeError) as excinfo:
                await scan_op.execute()

            # Verify the error message
            assert "Sending error" in str(excinfo.value)

    @pytest.mark.asyncio
    async def test_timeout_handling(self, mock_ros2_logger):
        """Test handling of timeouts during scan."""
        scan_op = ScanOperation(targets="192.168.1.1", logger=mock_ros2_logger)

        # Set up mocks to simulate timeout
        scan_op.send_sock = Mock(spec=socket.socket)
        scan_op.recv_sock = Mock(spec=socket.socket)
        scan_op.recv_sock.fileno.return_value = 1

        # Create host that never gets a response
        host = NetworkHost(ip_address="192.168.1.1")
        host.mark_timeout()
        scan_op.hosts = {"192.168.1.1": host}

        async def mock_collect_responses():
            # Simulate timeout by not changing host state
            pass

        # Patch both the socket initialization and the response collection
        with (
            patch.object(scan_op, "_configure_sockets"),
            patch.object(scan_op, "_send_packets"),
            patch.object(scan_op, "_collect_responses", new=mock_collect_responses),
            patch.object(scan_op, "_close_sockets"),
        ):

            # Execute scan
            results = await scan_op.execute()

        # Verify timeout was handled correctly
        assert "192.168.1.1" in results
        assert results["192.168.1.1"]["state"] == "DOWN"


@pytest.mark.asyncio
class TestScanOperationIntegration:
    """Integration tests for ScanOperation."""

    async def test_context_manager_usage(self, mock_ros2_logger):
        """Test basic context manager functionality with minimal mocking."""

        # Create a scan operation and fully patch its network methods
        scan_op = ScanOperation(targets="192.168.1.1", logger=mock_ros2_logger)

        # Patch internal methods to prevent any actual network operations
        with (
            patch.object(scan_op, "_configure_sockets"),
            patch.object(scan_op, "_create_hosts"),
            patch.object(scan_op, "_send_packets"),
            patch.object(scan_op, "_collect_responses"),
        ):

            # Use the context manager
            async with scan_op:
                scan_op.send_sock = Mock(spec=["close"])
                scan_op.recv_sock = Mock(spec=["close"])

                # Setup mock host and result
                mock_host = Mock()
                mock_host.state = (
                    Mock()
                )  # Add state attribute that execute() might need
                mock_host.rtt_ms = 10.5
                mock_host.error_message = None
                scan_op.hosts = {"192.168.1.1": mock_host}

                # Run the scan with fully mocked operations
                results = await scan_op.execute()

                # The execute method is now mocked, so just check it returns some dict
                assert isinstance(results, dict)
                mock_ros2_logger.info.assert_called_with("Scan completed")

            # After context exit, verify cleanup happened
            mock_ros2_logger.info.assert_called_with(
                "Socket 'recv_sock' cleaned up and closed"
            )

        assert scan_op.send_sock is None
        assert scan_op.recv_sock is None
