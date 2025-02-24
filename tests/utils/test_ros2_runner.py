# test_ros2_runner_no_ros.py

import pytest
import subprocess
from unittest.mock import patch, Mock
from codestral_ros2_gen.utils.ros2_runner import ROS2Runner


@pytest.fixture
def mock_popen():
    """
    A pytest fixture to patch subprocess.Popen so no real process is created.
    """
    with patch("subprocess.Popen") as mock_process_cls:
        mock_process_instance = Mock()
        # By default, pretend the node process started successfully and is still running
        mock_process_instance.poll.return_value = None
        mock_process_instance.communicate.return_value = ("", "")
        mock_process_cls.return_value = mock_process_instance
        yield mock_process_cls, mock_process_instance


def test_start_node_success(mock_popen):
    """
    Test that start_node() behaves correctly when the node appears to start successfully.
    """
    mock_process_cls, mock_process_instance = mock_popen

    runner = ROS2Runner(
        node_command="echo Starting Node", test_command="echo Running Tests"
    )
    runner.start_node()

    # Assert Popen was called to start the node process
    mock_process_cls.assert_called_once_with(
        runner.node_command,
        shell=True,
        executable="/bin/bash",
        stderr=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    # Assert the process is assumed running (poll returns None)
    assert runner.node_process.poll() is None


def test_start_node_failure(mock_popen):
    """
    Test that start_node() raises a RuntimeError if the node terminates immediately.
    """
    mock_process_cls, mock_process_instance = mock_popen
    # Make poll() indicate the process has ended
    mock_process_instance.poll.return_value = 1
    # Provide error message in stderr
    mock_process_instance.communicate.return_value = ("", "Error: Node crashed")

    runner = ROS2Runner(
        node_command="echo Starting Node", test_command="echo Running Tests"
    )
    with pytest.raises(RuntimeError) as excinfo:
        runner.start_node()

    # Assert we caught an immediate termination
    assert "terminated immediately" in str(excinfo.value)


def test_run_tests_success(mock_popen):
    """
    Test that run_tests() captures output and returns 0 if the subprocess exits with code 0.
    """
    mock_process_cls, mock_process_instance = mock_popen
    # Indicate test command completes successfully
    mock_process_instance.wait.return_value = 0
    mock_process_instance.stdout.readline.side_effect = ["Test passed\n", ""]
    mock_process_instance.stderr.readline.side_effect = ["", ""]

    runner = ROS2Runner(
        node_command="echo Starting Node", test_command="echo Running Tests"
    )
    exit_code = runner.run_tests()

    # Make sure the correct command was used
    mock_process_cls.assert_called_with(
        runner.test_command,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    # Assert the exit code is 0
    assert exit_code == 0
    # Check captured output
    assert "Test passed" in runner.test_output


def test_run_tests_failure(mock_popen):
    """
    Test that run_tests() captures output and returns a non-zero exit code if tests fail.
    """
    mock_process_cls, mock_process_instance = mock_popen
    # Indicate the test command fails
    mock_process_instance.wait.return_value = 1
    mock_process_instance.stdout.readline.side_effect = ["Test failed\n", ""]
    mock_process_instance.stderr.readline.side_effect = ["Error details\n", ""]

    runner = ROS2Runner(
        node_command="echo Starting Node", test_command="echo Running Tests"
    )
    exit_code = runner.run_tests()

    # Assert the process exit code was 1
    assert exit_code == 1
    # Check captured output
    assert "Test failed" in runner.test_output
    assert "Error details" in runner.test_output


def test_kill_node(mock_popen):
    """
    Test that kill_node() terminates the process (either gracefully or by force).
    """
    mock_process_cls, mock_process_instance = mock_popen
    runner = ROS2Runner(
        node_command="echo Starting Node", test_command="echo Running Tests"
    )

    # Start the node process
    runner.start_node()

    # Now kill the node
    runner.kill_node()
    mock_process_instance.terminate.assert_called_once()
