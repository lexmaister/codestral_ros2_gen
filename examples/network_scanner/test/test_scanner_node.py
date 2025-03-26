import rclpy
import time
import pytest
import os
import psutil
import subprocess
import json
import codestral_ros2_gen.utils.ros2_runner as ros2_runner
from network_scanner.msg import NetworkStatus


@pytest.fixture(scope="function")
def network_scanner_node(request, tmp_path, monkeypatch):
    """Fixture that sets up a network scanner node with the provided parameters."""
    if not rclpy.ok():
        rclpy.init()

    # Default parameters
    network = "4.2.2.2"
    scan_period = 3
    mock_nscan = False
    mock_nscan_text = """#!/bin/bash
    echo "NSCAN"
    """
    if hasattr(request, "param"):
        params = request.param
        network = params.get("network", network)
        scan_period = params.get("scan_period", scan_period)
        mock_nscan = params.get("mock_nscan", mock_nscan)
        mock_nscan_text = params.get("mock_nscan_text", mock_nscan_text)

    if mock_nscan:
        print("Using mock nscan binary")
        nscan_path = os.path.join(tmp_path, "nscan")
        with open(nscan_path, "w") as f:
            f.write(mock_nscan_text)
        os.chmod(nscan_path, 0o755)

        monkeypatch.setenv("PATH", f"{tmp_path}:{os.environ.get('PATH')}")
        print(f"nscan path: {nscan_path}")

    runner = ros2_runner.ROS2Runner(
        node_command=f"ros2 run network_scanner scanner_node --ros-args -p network:={network!r} -p scan_period:={scan_period}",
        test_command="",
    )
    runner.start_node()

    shell_process = psutil.Process(runner.node_process.pid)
    child_process = shell_process.children(recursive=False)[0]
    print(f"Child process PID: {child_process.pid}, command: {child_process.cmdline()}")

    yield child_process

    if runner.node_process:
        runner.kill_node()
        time.sleep(2)  # Allow time to shutdown


@pytest.fixture(scope="function")
def network_status_client():
    # Initialize RCL if needed
    if not rclpy.ok():
        rclpy.init()

    # Create a fresh client node for each test
    client_node = rclpy.create_node("network_status_client")
    received_msgs = []

    def callback(msg):
        received_msgs.append(msg)

    # Create subscription
    subscription = client_node.create_subscription(
        NetworkStatus, "/network_status", callback, 10
    )

    # Give some time for subscription to be established
    time.sleep(0.5)

    # Yield both the node and message buffer to the test
    yield client_node, received_msgs

    # Cleanup
    client_node.destroy_subscription(subscription)
    client_node.destroy_node()


def wait_for_messages(node, messages, count=2, timeout=30):
    """Helper function to wait for messages"""
    start_time = time.time()
    while time.time() - start_time < timeout and len(messages) < count:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Print received messages for debugging
    print(f"Received {len(messages)} messages:")
    for i, msg in enumerate(messages):
        print(f"Message {i}:")
        for addr in msg.addresses:
            print(f"  IP: {addr.ip_address}, Status: {addr.status}")

    return len(messages) >= count


def validate_message(msg, expected_ips):
    """Helper function to validate message content"""
    found_ips = [addr.ip_address for addr in msg.addresses]
    for expected_ip in expected_ips:
        assert (
            expected_ip in found_ips
        ), f"Expected IP '{expected_ip}' not found. Found: {found_ips}"

    for addr in msg.addresses:
        assert addr.status == "UP", f"IP '{addr.ip_address}' is not UP: {addr.status}"


# ======= Tests =======


class TestBasic:
    """Test only the basic functionality of the network scanner node."""

    @pytest.mark.order(1)
    def test_nscan_app_start(self, tmpdir):
        # Start the network scanner app
        process = subprocess.Popen(
            ["nscan", "1.2.3.4", "-t", "1", "-o", tmpdir],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        process.wait()
        assert process.returncode == 0

    @pytest.mark.order(2)
    def test_node_running(self, network_scanner_node):
        # Verify the node process is running.
        assert network_scanner_node.is_running()

    @pytest.mark.order(3)
    @pytest.mark.parametrize(
        "network_scanner_node,expected",
        [
            (
                {"network": "8.8.8.8", "scan_period": 3},
                {"expected_ip": ["8.8.8.8"], "expected_interval": 3},
            ),
            (
                {"network": "1.1.1.1, 8.8.4.4", "scan_period": 6},
                {"expected_ip": ["1.1.1.1", "8.8.4.4"], "expected_interval": 6},
            ),
        ],
        indirect=["network_scanner_node"],
    )
    def test_network_scanner_parameters(
        self, network_scanner_node, network_status_client, expected
    ):
        # Verify the node process is running
        assert network_scanner_node.is_running(), "Network scanner node is not running"

        client_node, received_msgs = network_status_client

        # Wait for at least 2 messages
        assert wait_for_messages(
            client_node, received_msgs, count=2, timeout=20
        ), f"Timeout waiting for messages. Received: {len(received_msgs)}"

        # Validate the content of the first two messages
        for msg in received_msgs[:2]:
            validate_message(msg, expected["expected_ip"])

        # Calculate the interval using ROS timestamps
        first_msg = received_msgs[0]
        second_msg = received_msgs[1]

        interval = (second_msg.stamp.sec - first_msg.stamp.sec) + (
            (second_msg.stamp.nanosec - first_msg.stamp.nanosec) * 1e-9
        )

        # Allow 1 second of tolerance for the interval check
        assert (
            abs(interval - expected["expected_interval"]) < 1
        ), f"Interval {interval:.2f} sec, expected ~{expected['expected_interval']} sec"

    @pytest.mark.order(4)
    def test_invalid_parameters(self):
        """
        Test that the network scanner node handles invalid parameters.
        When an empty network string or a non-positive scan_period is provided,
        the node should log an error and exit.
        """
        # Provide invalid parameters: empty network string and non-positive scan_period.
        invalid_network = "1.2.3.4.5"
        invalid_scan_period = 0
        runner = ros2_runner.ROS2Runner(
            node_command=f"ros2 run network_scanner scanner_node --ros-args -p network:={invalid_network!r} -p scan_period:={invalid_scan_period}",
            test_command="",
        )
        try:
            with pytest.raises(
                RuntimeError, match="Node process terminated immediately"
            ):
                runner.start_node()

        finally:
            runner.kill_node()

    @pytest.mark.order(5)
    def test_no_messages_without_publisher(self, network_status_client):
        """
        Test that no messages are received when there is no publisher on the network status topic.
        """
        client_node, received_msgs = network_status_client
        assert not wait_for_messages(
            client_node, received_msgs, count=1, timeout=5
        ), "Expected no messages, but received one."

    @pytest.mark.order(6)
    @pytest.mark.parametrize(
        "network_scanner_node",
        [
            {
                "network": "1.0.0.1",
                "scan_period": 1,
                "mock_nscan": True,
                "mock_nscan_text": f"""#!/usr/bin/env python3
import time
with open("/tmp/nscan_invocations.log", "a") as f:
    f.write(str(time.time()) + "\\n")
time.sleep(3)
exit(0)
""",
            },
        ],
        indirect=["network_scanner_node"],
    )
    def test_no_overlapping_scans(self, network_scanner_node):
        """
        Test that scans do not overlap when each scan takes longer than the scan_period.
        Uses a stub nscan binary that sleeps for 3 seconds, while scan_period is set to 1 second.
        """

        # Create a temporary file to record nscan invocations
        log_file = "/tmp/nscan_invocations.log"
        with open(log_file, "w") as f:
            f.write("")

        time.sleep(10)  # Wait to allow multiple scans to complete

        # Read the log file and verify that each invocation happened at least 3 seconds apart
        with open(log_file, "r") as log:
            # Read the times from the log file
            lines = log.readlines()
            if lines:
                times = [float(line.strip()) for line in lines]
                assert len(times) > 1, "Only one scan invocation recorded."
                times = sorted(times)
                # Check that each scan started at least 3 seconds after the previous one
                for t1, t2 in zip(times, times[1:]):
                    assert (
                        t2 - t1
                    ) >= 3, f"Overlap detected: Scan started after {t2-t1:.2f} sec, expected at least 3 sec between scans."

            else:
                pytest.fail("No scan invocations recorded.")


class TestAdditional:
    """
    Additional tests for the network scanner node - react on specific situations.
    """

    @pytest.mark.parametrize(
        "network_scanner_node",
        [
            {
                "network": "1.2.3.4",
                "scan_period": 2,
                "mock_nscan": True,
            },
        ],
        indirect=["network_scanner_node"],
    )
    def test_invalid_json_format(self, network_scanner_node, network_status_client):
        """
        Test that when the JSON file format isn't valid.
        """
        # Expected path for the JSON file produced by nscan
        json_file_path = "/tmp/nscan_results.json"
        if os.path.exists(json_file_path):
            os.remove(json_file_path)

        with open(json_file_path, "w") as f:
            f.buffer.write(b"{malformed json content")

        client_node, received_msgs = network_status_client
        # timeout 4 to wait for process shuts down
        assert not wait_for_messages(
            client_node, received_msgs, count=1, timeout=4
        ), "Expected no messages, but received one."

        ros_node = network_scanner_node
        with pytest.raises(psutil.NoSuchProcess):
            print("Node process status:", ros_node.status())

    @pytest.mark.parametrize(
        "network_scanner_node",
        [
            {
                "network": "1.2.3.4",
                "scan_period": 2,
                "mock_nscan": True,
            },
        ],
        indirect=["network_scanner_node"],
    )
    def test_outdated_json_results(self, network_scanner_node, network_status_client):
        """
        Test that when the JSON result file is older than 20 seconds,
        the scanner node publishes a message with an empty addresses list.
        """
        # Expected path for the JSON file produced by nscan
        json_file_path = "/tmp/nscan_results.json"
        if os.path.exists(json_file_path):
            os.remove(json_file_path)

        # Create outdated JSON file - will check from the first scan
        data = {"7.7.7.7": {"state": "UP", "response_time_ms": 15, "error": None}}
        with open(json_file_path, "w") as f:
            json.dump(data, f)

        # Backdate the modification time by 20 seconds
        old_time = time.time() - 20
        os.utime(json_file_path, (old_time, old_time))

        client_node, received_msgs = network_status_client
        assert wait_for_messages(
            client_node, received_msgs, count=3, timeout=10
        ), "Did not receive at least 3 messages in time"

        assert all(msg.addresses == [] for msg in received_msgs)


class TestFull(TestBasic, TestAdditional):
    """
    Complete test suite that runs all tests for the network scanner node,
    including both basic functionality and additional edge cases.
    """

    pass
