import pytest
from unittest.mock import patch, MagicMock
import subprocess
from codestral_ros2_gen.utils.ros2_runner import ROS2Runner


@pytest.fixture
def mock_subprocess_popen():
    with patch("subprocess.Popen") as mock_popen:
        yield mock_popen


@pytest.fixture
def mock_psutil_process():
    with patch("psutil.Process") as mock_process:
        yield mock_process


def test_start_node_success(mock_subprocess_popen):
    mock_process = MagicMock()
    mock_process.poll.return_value = None
    mock_subprocess_popen.return_value = mock_process

    runner = ROS2Runner(node_command="ros2 run my_node", test_command="pytest")
    runner.start_node()

    mock_subprocess_popen.assert_called_once_with(
        "ros2 run my_node",
        shell=True,
        executable="/bin/bash",
        stderr=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    assert runner.node_process == mock_process


def test_start_node_failure(mock_subprocess_popen):
    mock_process = MagicMock()
    mock_process.poll.return_value = 1
    mock_process.communicate.return_value = ("stdout", "stderr")
    mock_subprocess_popen.return_value = mock_process

    runner = ROS2Runner(node_command="ros2 run my_node", test_command="pytest")

    with pytest.raises(RuntimeError, match="Node process terminated immediately"):
        runner.start_node()


def test_run_tests_success(mock_subprocess_popen):
    mock_process = MagicMock()
    mock_process.communicate.return_value = ("stdout", "stderr")
    mock_process.returncode = 0
    mock_subprocess_popen.return_value = mock_process

    runner = ROS2Runner(node_command="ros2 run my_node", test_command="pytest")
    return_code = runner.run_tests()

    mock_subprocess_popen.assert_called_once_with(
        "pytest",
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert return_code == 0
    assert runner.test_output == "stdout:\nstdout\nstderr:\nstderr"


def test_run_tests_timeout(mock_subprocess_popen):
    mock_process = MagicMock()
    mock_process.communicate.side_effect = subprocess.TimeoutExpired(
        cmd="pytest", timeout=30, output=("stdout", "stderr")
    )
    mock_subprocess_popen.return_value = mock_process

    runner = ROS2Runner(
        node_command="ros2 run my_node", test_command="pytest", test_timeout=30
    )
    return_code = runner.run_tests()

    mock_process.kill.assert_called_once()
    assert return_code == 1
    assert "Test process timed out after 30 seconds" in runner.test_output


def test_kill_node_success(mock_psutil_process):
    mock_shell_process = MagicMock()
    mock_child_process = MagicMock()
    mock_child_process.cmdline.return_value = ["ros2", "run", "my_node"]
    mock_child_process.pid = 5678
    mock_shell_process.children.return_value = [mock_child_process]
    mock_shell_process.is_running.return_value = True
    mock_psutil_process.return_value = mock_shell_process

    runner = ROS2Runner(node_command="ros2 run my_node", test_command="pytest")
    runner.node_process = MagicMock(pid=1234)

    runner.kill_node()

    mock_child_process.terminate.assert_called_once()
    mock_shell_process.terminate.assert_called_once()
    assert runner.node_process is None


def test_get_tests_stat():
    runner = ROS2Runner(node_command="ros2 run my_node", test_command="pytest")
    runner.test_output = "stdout:\n10 passed in 0.12s\n10 failed in 0.12s\n10 skipped in 0.12s\nstderr:\n"

    runner._get_tests_stat()

    assert runner.tests_passed == 10
    assert runner.tests_failed == 10
    assert runner.tests_skipped == 10


@pytest.mark.parametrize("return_code, succeed", [(1, 0), (0, 1)])
def test_run_success(mock_subprocess_popen, mock_psutil_process, return_code, succeed):
    mock_test_process = MagicMock()
    mock_test_process.poll.return_value = None
    mock_test_process.communicate.return_value = (
        "stdout 1 passed, 2 failed, 3 skipped in 10 s",
        "stderr",
    )
    mock_test_process.returncode = return_code
    mock_subprocess_popen.return_value = mock_test_process

    mock_node_process = MagicMock()
    mock_node_process.poll.return_value = None
    mock_node_process.children.return_value = []
    mock_node_process.is_running.return_value = False
    mock_psutil_process.return_value = mock_node_process

    runner = ROS2Runner(node_command="ros2 run my_node", test_command="pytest")
    success, _, stats = runner.run()

    assert success == succeed
    assert stats == (1, 2, 3)
