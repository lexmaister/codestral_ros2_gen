#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from ai_gen_interfaces.srv import ObjectHeight
import time


# Default parameters
DEFAULT_PARAMS = {
    "focal_length": 35.0,
    "image_height": 1152,
    "pixel_size": 3.45,
    "object_distance": 6.5,
}


@pytest.fixture
def service_client():
    rclpy.init()
    node = Node("test_node")
    client = node.create_client(ObjectHeight, "calculate_object_height")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
    yield client, node
    node.destroy_node()
    rclpy.shutdown()


def call_service(
    client: rclpy.client.Client,
    node: Node,
    focal_length: float,
    image_height: int,
    pixel_size: float,
    object_distance: float,
    timeout_sec: float = 1.0,
) -> ObjectHeight.Response:
    """Helper function to call service"""
    request = ObjectHeight.Request()
    request.focal_length = focal_length
    request.image_height = image_height
    request.pixel_size = pixel_size
    request.object_distance = object_distance

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if not future.done():
        raise TimeoutError("Service call timed out")

    try:
        response = future.result()
        return response
    except Exception as e:
        raise RuntimeError(f"Service call failed: {str(e)}")


def test_default_parameters(service_client):
    """Test service with default parameters"""
    client, node = service_client
    response = call_service(
        client,
        node,
        DEFAULT_PARAMS["focal_length"],
        DEFAULT_PARAMS["image_height"],
        DEFAULT_PARAMS["pixel_size"],
        DEFAULT_PARAMS["object_distance"],
    )
    assert response.success
    assert response.object_height > 0
    assert isinstance(response.object_height, float)


@pytest.mark.parametrize(
    "focal_length,image_height,pixel_size,distance,expected",
    [
        (35.0, 1152, 3.45, 6.5, 738.1),
        (35.0, 1152, 3.45, 5.9, 670.0),
        (50.0, 1152, 3.45, 6.5, 516.7),
    ],
)
def test_valid_parameters(
    service_client, focal_length, image_height, pixel_size, distance, expected
):
    """Test service with various valid parameter combinations"""
    client, node = service_client
    response = call_service(
        client, node, focal_length, image_height, pixel_size, distance
    )
    assert response.success
    assert abs(response.object_height - expected) < 0.1


@pytest.mark.parametrize(
    "focal_length,image_height,pixel_size,distance",
    [
        (-35.0, 1152, 3.45, 6.5),  # Negative focal length
        (35.0, -1152, 3.45, 6.5),  # Negative image height
        (35.0, 1152, -3.45, 6.5),  # Negative pixel size
        (35.0, 1152, 3.45, -6.5),  # Negative distance
        (0.0, 1152, 3.45, 6.5),  # Zero focal length
    ],
)
def test_invalid_parameters(
    service_client, focal_length, image_height, pixel_size, distance
):
    """Test service with invalid parameters"""
    client, node = service_client
    response = call_service(
        client, node, focal_length, image_height, pixel_size, distance
    )
    assert not response.success
    assert "Invalid parameter:" in response.feedback


@pytest.mark.parametrize(
    "focal_length,image_height,pixel_size,distance,expected",
    [
        (0.001, 1, 0.001, 0.001, 0.001),  # Minimum values
        (10000.0, 10000, 10000.0, 10000.0, 100000000.0),  # Maximum values
    ],
)
def test_edge_cases(
    service_client, focal_length, image_height, pixel_size, distance, expected
):
    """Test service with edge case parameters"""
    client, node = service_client
    response = call_service(
        client, node, focal_length, image_height, pixel_size, distance
    )
    assert response.success
    assert abs(response.object_height - expected) < 0.1


def test_concurrent_requests(service_client):
    """Test service with concurrent requests"""
    client, node = service_client
    futures = []
    for _ in range(10):
        request = ObjectHeight.Request()
        request.focal_length = DEFAULT_PARAMS["focal_length"]
        request.image_height = DEFAULT_PARAMS["image_height"]
        request.pixel_size = DEFAULT_PARAMS["pixel_size"]
        request.object_distance = DEFAULT_PARAMS["object_distance"]
        futures.append(client.call_async(request))

    for future in futures:
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        assert response.success


def test_service_recovery(service_client):
    """Test service recovery after failure"""
    client, node = service_client
    response = call_service(client, node, -35.0, 1152, 3.45, 6.5)
    assert not response.success

    response = call_service(
        client,
        node,
        DEFAULT_PARAMS["focal_length"],
        DEFAULT_PARAMS["image_height"],
        DEFAULT_PARAMS["pixel_size"],
        DEFAULT_PARAMS["object_distance"],
    )
    assert response.success
    assert response.object_height > 0


def test_performance(service_client):
    """Test service performance under load"""
    client, node = service_client
    start_time = time.time()
    for _ in range(100):
        response = call_service(
            client,
            node,
            DEFAULT_PARAMS["focal_length"],
            DEFAULT_PARAMS["image_height"],
            DEFAULT_PARAMS["pixel_size"],
            DEFAULT_PARAMS["object_distance"],
        )
        assert response.success
    end_time = time.time()
    assert (end_time - start_time) < 5.0  # Ensure it completes in under 5 seconds


if __name__ == "__main__":
    pytest.main([__file__])
