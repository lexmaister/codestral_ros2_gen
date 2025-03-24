from network_scanner.msg import NetworkStatus
import rclpy
import subprocess
import time
import pytest
import os
import json
from codestral_ros2_gen.utils.ros2_runner import ROS2Runner


@pytest.fixture(scope="function")
def network_scanner_node(request):
    if not rclpy.ok():
        rclpy.init()

    # Default parameters
    network = "8.8.8.8"
    scan_period = 10
    if hasattr(request, "param"):
        params = request.param
        network = params.get("network", network)
        scan_period = params.get("scan_period", scan_period)

    runner = ROS2Runner(
        node_command=f"ros2 run network_scanner scanner_node --ros-args -p network:={network!r} -p scan_period:={scan_period}",
        test_command="",
    )
    runner.start_node()
    yield runner.node_process
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


def test_nscan_app_start(tmpdir):
    # Start the network scanner app
    process = subprocess.Popen(
        ["nscan", "8.8.8.8", "-t", "1", "-o", tmpdir],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    process.wait()
    assert process.returncode == 0


def test_node_running(network_scanner_node):
    # Verify the node process is running.
    assert network_scanner_node.poll() is None


@pytest.mark.parametrize(
    "network_scanner_node,expected",
    [
        (
            {"network": "8.8.8.8", "scan_period": 10},
            {"expected_ip": ["8.8.8.8"], "expected_interval": 10},
        ),
        (
            {"network": "1.1.1.1, 8.8.4.4", "scan_period": 5},
            {"expected_ip": ["1.1.1.1", "8.8.4.4"], "expected_interval": 5},
        ),
    ],
    indirect=["network_scanner_node"],
)
def test_network_scanner_parameters(
    network_scanner_node, network_status_client, expected
):
    # Helper function to wait for messages
    def wait_for_messages(node, messages, count=2, timeout=30):
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

    # Helper function to validate message content
    def validate_message(msg, expected_ips):
        found_ips = [addr.ip_address for addr in msg.addresses]
        for expected_ip in expected_ips:
            assert (
                expected_ip in found_ips
            ), f"Expected IP '{expected_ip}' not found. Found: {found_ips}"

        for addr in msg.addresses:
            assert (
                addr.status == "UP"
            ), f"IP '{addr.ip_address}' is not UP: {addr.status}"

    # Verify the node process is running
    assert network_scanner_node.poll() is None, "Network scanner node is not running"

    client_node, received_msgs = network_status_client

    # Wait for at least 2 messages
    assert wait_for_messages(
        client_node, received_msgs, count=2
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


def test_invalid_parameters():
    """
    Test that the network scanner node handles invalid parameters.
    When an empty network string or a non-positive scan_period is provided,
    the node should log an error and exit.
    """
    # Provide invalid parameters: empty network string and non-positive scan_period.
    invalid_network = "1.2.3.4.5"
    invalid_scan_period = 0
    runner = ROS2Runner(
        node_command=f"ros2 run network_scanner scanner_node --ros-args -p network:={invalid_network!r} -p scan_period:={invalid_scan_period}",
        test_command="",
    )
    with pytest.raises(RuntimeError, match="Node process terminated immediately"):
        runner.start_node()

    runner.kill_node()


def test_no_messages_without_publisher(network_status_client):
    """
    Test that no messages are received when there is no publisher on the network status topic.
    """
    client_node, received_msgs = network_status_client
    # Wait for a short duration, shorter than usually expected for messages.
    start_time = time.time()
    while time.time() - start_time < 5:
        rclpy.spin_once(client_node, timeout_sec=0.1)
    # Expect no messages have been received
    assert (
        len(received_msgs) == 0
    ), f"Expected no messages, but received {len(received_msgs)}"


def test_no_overlapping_scans(tmp_path, monkeypatch):
    """
    Test that scans do not overlap when each scan takes longer than the scan_period.
    Uses a stub nscan binary that sleeps for 3 seconds, while scan_period is set to 1 second.
    """

    # Create a temporary file to record nscan invocations
    log_file = tmp_path / "nscan_invocations.log"
    log_file.write_text("")

    # Create a stub nscan binary that logs its invocation time and sleeps for 3 seconds
    stub_nscan = tmp_path / "nscan"
    stub_nscan.write_text(
        f"""#!/usr/bin/env python3
import time
with open("{str(log_file)}", "a") as f:
    f.write(str(time.time()) + "\\n")
time.sleep(3)
exit(0)
"""
    )
    stub_nscan.chmod(0o755)

    # Prepend the directory containing our stub nscan to the PATH
    original_path = os.environ.get("PATH", "")
    monkeypatch.setenv(
        "PATH", f"{tmp_path}{':' + original_path if original_path else ''}"
    )

    # Start the scanner node with a short scan_period (1 second)
    from codestral_ros2_gen.utils.ros2_runner import ROS2Runner

    runner = ROS2Runner(
        node_command="ros2 run network_scanner scanner_node --ros-args -p network:='8.8.8.8' -p scan_period:=1",
        test_command="",
    )
    runner.start_node()

    # Let the node run long enough to allow multiple scan invocations
    time.sleep(8)
    runner.kill_node()

    # Read the log file and verify that each invocation happened at least 3 seconds apart
    times = [
        float(line.strip())
        for line in log_file.read_text().splitlines()
        if line.strip()
    ]
    for t1, t2 in zip(times, times[1:]):
        assert (
            t2 - t1
        ) >= 3, f"Overlap detected: Scan started after {t2-t1:.2f} sec, expected at least 3 sec between scans."


def test_outdated_json_results(tmp_path, monkeypatch, network_status_client):
    """
    Test that when the JSON result file is older than 10 seconds,
    the scanner node publishes a message with an empty addresses list.
    """
    # Expected path for the JSON file produced by nscan
    json_file_path = "/tmp/nscan_results.json"
    if os.path.exists(json_file_path):
        os.remove(json_file_path)

    # Create a stub nscan binary that writes a JSON file with valid data,
    # then backdates its modification time by 20 seconds.
    stub_nscan = tmp_path / "nscan"
    stub_nscan.write_text(
        f"""#!/usr/bin/env python3
import time, os, json
json_file = "{json_file_path}"
data = {{"8.8.8.8": {{"state": "UP", "response_time_ms": 15, "error": None}}}}
with open(json_file, "w") as f:
    json.dump(data, f)
# Backdate the modification time by 20 seconds
old_time = time.time() - 20
os.utime(json_file, (old_time, old_time))
exit(0)
"""
    )
    stub_nscan.chmod(0o755)

    # Prepend our stub nscan directory to the PATH so that the node picks it up.
    original_path = os.environ.get("PATH", "")
    monkeypatch.setenv(
        "PATH", f"{tmp_path}{':' + original_path if original_path else ''}"
    )

    # Start the scanner node with a short scan_period (2 seconds) to trigger the scan.
    from codestral_ros2_gen.utils.ros2_runner import ROS2Runner

    runner = ROS2Runner(
        node_command="ros2 run network_scanner scanner_node --ros-args -p network:='8.8.8.8' -p scan_period:=2",
        test_command="",
    )
    runner.start_node()

    # Wait for the node to publish a message (using the existing ROS2 status client).
    client_node, received_msgs = network_status_client
    start_time = time.time()
    while time.time() - start_time < 10 and not received_msgs:
        import rclpy

        rclpy.spin_once(client_node, timeout_sec=0.1)

    runner.kill_node()

    assert received_msgs, "No message received within timeout."
    # Assert that the message's addresses list is empty due to the outdated JSON.
    msg = received_msgs[0]
    assert (
        len(msg.addresses) == 0
    ), "Expected empty addresses list when JSON file is outdated."
