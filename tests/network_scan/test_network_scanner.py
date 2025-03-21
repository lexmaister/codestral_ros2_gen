import pytest
from unittest.mock import Mock, patch, AsyncMock
import socket
import asyncio

from codestral_ros2_gen.network_scan.network_scanner import NetworkScanner
from codestral_ros2_gen.network_scan.network_host import NetworkHost
from codestral_ros2_gen.network_scan.scan_operation import ScanOperation


@pytest.fixture
def default_scanner(mock_ros2_logger):
    """Fixture providing a default NetworkScanner instance."""
    return NetworkScanner(logger=mock_ros2_logger)


@pytest.fixture
def configured_scanner(mock_ros2_logger):
    """Fixture providing a configured NetworkScanner instance."""
    return NetworkScanner(
        timeout=2.0,
        packet_size=128,
        sending_interval=0.05,
        logger=mock_ros2_logger,
    )


@pytest.fixture
def mock_scan_operation():
    """Fixture providing a mock ScanOperation."""
    mock_op = Mock(spec=ScanOperation)
    mock_op.execute.return_value = [
        Mock(spec=NetworkHost, ip="192.168.1.1", is_up=True, rtt=5.0),
        Mock(spec=NetworkHost, ip="192.168.1.2", is_up=False, rtt=None),
    ]
    return mock_op


class TestNetworkScanner:
    """Test suite for the NetworkScanner class."""

    def test_initialization_default(self, default_scanner, mock_ros2_logger):
        """Test initialization with default parameters."""
        assert default_scanner.logger == mock_ros2_logger
        assert hasattr(default_scanner, "timeout")
        assert hasattr(default_scanner, "packet_size")

    def test_initialization_configured(self, configured_scanner, mock_ros2_logger):
        """Test initialization with specific configuration."""
        assert configured_scanner.logger == mock_ros2_logger
        assert configured_scanner.timeout == 2.0
        assert configured_scanner.packet_size == 128
        assert configured_scanner.sending_interval == 0.05

    @pytest.mark.asyncio
    async def test_scan_method(self, default_scanner):
        """Test the scan method creates and executes a ScanOperation."""
        # Create a mock for the ScanOperation context manager
        mock_scan_op = AsyncMock()
        mock_scan_op.__aenter__.return_value = mock_scan_op
        mock_result = [Mock(ip="192.168.1.1", is_up=True, rtt=5.0)]
        mock_scan_op.execute.return_value = mock_result

        # Patch the ScanOperation class to return our async mock
        with patch(
            "codestral_ros2_gen.network_scan.network_scanner.ScanOperation",
            return_value=mock_scan_op,
        ):
            # Execute scan
            results = await default_scanner._scan_async("192.168.1.1")
            # Or if scan() is a wrapper that calls asyncio.run():
            # results = default_scanner.scan()

            # Verify behavior
            mock_scan_op.__aenter__.assert_called_once()
            mock_scan_op.execute.assert_called_once()
            assert results == mock_result

    def test_scan_with_no_targets(self, default_scanner):
        """Test scan behavior with no targets set."""
        assert default_scanner.scan("xxx") == {}

    @pytest.mark.asyncio
    async def test_get_scan_results(self, configured_scanner):
        """Test retrieving scan results."""
        # Create mock scan results in the format returned by ScanOperation
        mock_results = {
            "192.168.1.1": {"state": "UP", "response_time_ms": 5, "error": None},
            "192.168.1.2": {"state": "DOWN", "response_time_ms": None, "error": None},
        }

        # Create a mock async scan operation
        mock_scan_op = AsyncMock()
        mock_scan_op.__aenter__.return_value = mock_scan_op
        mock_scan_op.execute.return_value = mock_results

        # Patch ScanOperation to return our async mock
        with patch(
            "codestral_ros2_gen.network_scan.network_scanner.ScanOperation",
            return_value=mock_scan_op,
        ):
            # Execute scan - the scan method should store results internally
            results = await configured_scanner._scan_async("192.168.1.1, 192.168.1.2")

            assert len(results) == 2
            assert "192.168.1.1" in results
            assert "192.168.1.2" in results

            # Verify our mock was used correctly
            mock_scan_op.__aenter__.assert_called_once()
            mock_scan_op.execute.assert_called_once()

    def test_format_results(self, configured_scanner, mock_ros2_logger):
        """Test the format_results method with different input configurations."""
        # Create test scan results
        test_results = {
            "192.168.1.1": {"state": "UP", "response_time_ms": 5, "error": None},
            "192.168.1.2": {"state": "DOWN", "response_time_ms": None, "error": None},
            "192.168.1.3": {
                "state": "DOWN",
                "response_time_ms": None,
                "error": "Connection timeout",
            },
            "192.168.1.4": {"state": "UP", "response_time_ms": 10, "error": None},
        }

        # Set a scan time for the summary section
        configured_scanner.scan_time = 1.5

        # Case 1: Test with show_all=True
        formatted_all = configured_scanner.format_results(test_results, show_all=True)

        # Verify structure and content
        assert "Network Scan Results" in formatted_all
        assert "IP Address" in formatted_all
        assert "State" in formatted_all
        assert "Response Time (ms)" in formatted_all

        # Verify all IPs are included
        for ip in test_results.keys():
            assert ip in formatted_all

        # Verify summary statistics
        assert "Total hosts scanned: 4" in formatted_all
        assert "Hosts up: 2" in formatted_all
        assert "Hosts down: 2" in formatted_all
        assert "Hosts with errors: 1" in formatted_all
        assert "Scan duration: 1.50 seconds" in formatted_all

        # Case 2: Test with show_all=False (only UP hosts)
        formatted_up_only = configured_scanner.format_results(
            test_results, show_all=False
        )

        # Verify only UP hosts are included
        assert "192.168.1.1" in formatted_up_only
        assert "192.168.1.4" in formatted_up_only
        assert "192.168.1.2" not in formatted_up_only
        assert "192.168.1.3" not in formatted_up_only

        # Summary should still show statistics for all hosts
        assert "Total hosts scanned: 4" in formatted_up_only

        # Case 3: Test with empty results
        empty_formatted = configured_scanner.format_results({}, show_all=True)
        assert empty_formatted == ""
        assert mock_ros2_logger.info.call_args_list[-1][0][0] == "No hosts to display."

        # Case 4: Test with only DOWN hosts and show_all=False
        down_only_results = {
            "192.168.1.2": {"state": "DOWN", "response_time_ms": None, "error": None},
            "192.168.1.3": {
                "state": "DOWN",
                "response_time_ms": None,
                "error": "Connection timeout",
            },
        }

        down_only_formatted = configured_scanner.format_results(
            down_only_results, show_all=False
        )
        assert down_only_formatted == ""
        assert mock_ros2_logger.info.call_args_list[-1][0][0] == "No hosts to display."

        # Case 5: Verify response time formatting
        results_with_times = {
            "192.168.1.1": {"state": "UP", "response_time_ms": 5, "error": None},
            "192.168.1.2": {"state": "DOWN", "response_time_ms": None, "error": None},
        }

        formatted_times = configured_scanner.format_results(
            results_with_times, show_all=True
        )
        assert "5 ms" in formatted_times
        assert "-" in formatted_times
