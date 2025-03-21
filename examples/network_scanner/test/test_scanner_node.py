from network_scanner.msg import NetworkStatus
import rclpy
import subprocess
import time
import pytest


@pytest.fixture(scope="module")
def network_scanner_node():
    # Start node similarly to "ros2 run network_scanner scanner_node"
    process = subprocess.Popen(
        ["ros2", "run", "network_scanner", "scanner_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    time.sleep(2)  # Allow node time to initialize
    yield process
    process.terminate()
    process.wait()


@pytest.fixture(scope="module")
def network_status_client():
    if not rclpy.ok():
        rclpy.init()
    client_node = rclpy.create_node("network_status_client")
    received_msgs = []

    def callback(msg):
        received_msgs.append(msg)

    subscription = client_node.create_subscription(
        NetworkStatus, "/network_status", callback, 10
    )
    yield client_node, received_msgs
    client_node.destroy_subscription(subscription)
    client_node.destroy_node()


def test_node_running(network_scanner_node):
    # Verify the node process is still running.
    assert network_scanner_node.poll() is None


def test_default_parameters(network_scanner_node, network_status_client):
    client_node, received_msgs = network_status_client
    start_time = time.time()
    timeout = 30  # seconds
    while time.time() - start_time < timeout and len(received_msgs) < 2:
        rclpy.spin_once(client_node, timeout_sec=1)
    assert len(received_msgs) >= 2, "Didn't receive at least two messages"
    first_msg = received_msgs[0]
    second_msg = received_msgs[1]
    # Verify at least one IPStatus entry in the first message has ip_address "8.8.8.8"
    assert any(
        ip.ip_address == "8.8.8.8" for ip in first_msg.addresses
    ), "Default IP '8.8.8.8' not found in first message"
    # Compute the interval from ROS timestamps (with nanosec correction)
    interval = (second_msg.stamp.sec - first_msg.stamp.sec) + (
        (second_msg.stamp.nanosec - first_msg.stamp.nanosec) * 1e-9
    )
    assert (
        abs(interval - 10) < 2
    ), f"Interval between messages is {interval} sec, expected ~10 seconds"
