import pytest
import subprocess
from unittest.mock import patch, Mock, call

from codestral_ros2_gen.utils.ros2_runner import ROS2Runner


@pytest.fixture
def mock_popen(monkeypatch):
    """Fixture to mock subprocess.Popen"""
    mock_process = Mock()
    mock_process.stdout = Mock()
    mock_process.stderr = Mock()
    mock_process.returncode = 0
    mock_process.pid = 12345

    # Make poll() indicate process is running
    mock_process.poll.return_value = None

    # Make communicate() return empty output by default
    mock_process.communicate.return_value = ("", "")

    mock_popen = Mock(return_value=mock_process)
    monkeypatch.setattr(subprocess, "Popen", mock_popen)

    return mock_popen, mock_process


def test_init_default_values():
    """Test initialization with default values"""
    runner = ROS2Runner(node_command="ros2 run pkg node", test_command="pytest")
    assert runner.node_command == "ros2 run pkg node"
    assert runner.test_command == "pytest"
    assert runner.test_timeout == 30  # assuming default timeout is 30
    assert runner.node_process is None
    assert runner.test_output == ""


def test_start_node(mock_popen):
    """Test starting a node successfully"""
    mock_process_cls, mock_process_instance = mock_popen

    # Configure mock to show process is running
    mock_process_instance.poll.return_value = None

    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node", test_command="pytest"
    )

    runner.start_node()

    # Updated assertion to match actual implementation
    mock_process_cls.assert_called_with(
        "ros2 run test_pkg test_node",
        shell=True,
        executable="/bin/bash",
        stderr=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    assert runner.node_process == mock_process_instance


def test_start_node_immediate_termination(mock_popen):
    """Test starting a node that terminates immediately"""
    mock_process_cls, mock_process_instance = mock_popen

    # Configure mock to show process terminated
    mock_process_instance.poll.return_value = 1
    mock_process_instance.communicate.return_value = ("", "Node crashed")

    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node", test_command="pytest"
    )

    with pytest.raises(RuntimeError) as excinfo:
        runner.start_node()

    assert "Node process terminated immediately" in str(excinfo.value)


def test_kill_node_not_running():
    """Test killing a node that isn't running"""
    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node", test_command="pytest"
    )

    # Should not raise any exception
    runner.kill_node()
    assert runner.node_process is None


def test_kill_node_graceful(mock_popen):
    """Test killing a node gracefully"""
    mock_process_cls, mock_process_instance = mock_popen

    # Configure mock for successful startup and shutdown
    mock_process_instance.poll.return_value = None
    mock_process_instance.wait.return_value = 0

    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node", test_command="pytest"
    )

    runner.start_node()
    runner.kill_node()

    mock_process_instance.terminate.assert_called_once()
    mock_process_instance.wait.assert_called_once()
    assert runner.node_process is None


def test_run_tests_success(mock_popen):
    """Test successful test execution"""
    mock_process_cls, mock_process_instance = mock_popen

    # Configure mock for successful test run
    mock_process_instance.communicate.return_value = ("Tests passed", "")
    mock_process_instance.returncode = 0

    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node", test_command="pytest"
    )

    result = runner.run_tests()

    assert result == 0
    assert "Tests passed" in runner.test_output


def test_run_tests_with_node(mock_popen):
    """Test running tests with a node running"""
    mock_process_cls, mock_process_instance = mock_popen

    # Configure mock for successful node startup and test run
    mock_process_instance.poll.return_value = None
    mock_process_instance.communicate.return_value = ("Test output", "")
    mock_process_instance.returncode = 0

    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node", test_command="pytest"
    )

    runner.start_node()
    result = runner.run_tests()

    assert result == 0
    assert "Test output" in runner.test_output


def test_run_tests_failure(mock_popen):
    """Test handling test failures"""
    mock_process_cls, mock_process_instance = mock_popen

    # Configure mock for failed test run
    mock_process_instance.communicate.return_value = ("Test failed", "Error occurred")
    mock_process_instance.returncode = 1

    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node", test_command="pytest"
    )

    result = runner.run_tests()

    assert result == 1
    assert "Test failed" in runner.test_output
    assert "Error occurred" in runner.test_output


def test_run_tests_timeout(mock_popen):
    """Test handling timeout during test execution"""
    mock_process_cls, mock_process_instance = mock_popen

    # Configure mock for timeout with some partial output
    timeout_exc = subprocess.TimeoutExpired(
        cmd="pytest",
        timeout=30,
        output="Partial test output",
        stderr="Test timeout occurred",
    )
    mock_process_instance.communicate.side_effect = timeout_exc

    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node",
        test_command="pytest",
        test_timeout=30,
    )

    result = runner.run_tests()

    # Verify process was killed after timeout
    mock_process_instance.kill.assert_called_once()
    # Check return code indicates failure
    assert result == 1
    # Verify timeout message is in output
    assert "Test process timed out after 30 seconds" in runner.test_output
    assert "Partial test output" in runner.test_output


def test_get_tests_stat():
    """Test parsing test output for statistics"""
    runner = ROS2Runner(
        node_command="ros2 run test_pkg test_node",
        test_command="pytest",
        test_timeout=30,
    )
    runner.test_output = """
    ============================= short test summary info ==============================
    FAILED test_pkg/test_node.py::test_failed - AssertionError: assert False
    PASSED test_pkg/test_node.py::test_passed
    SKIPPED test_pkg/test_node.py::test_skipped
    ====================== 1 failed, 1 passed, 1 skipped in 0.12s =======================
    """
    runner._get_tests_stat()
    assert runner.tests_passed == 1
    assert runner.tests_failed == 1
    assert runner.tests_skipped == 1
