#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ObjectHeightService(Node):
    """Template for object height service node."""

    def __init__(self):
        super().__init__("object_height_service")
        self.status_publisher = self.create_publisher(String, "service_status", 10)
        self.timer = self.create_timer(2.0, self.publish_status)
        self.get_logger().info("Object Height Service template initialized")

    def publish_status(self):
        msg = String()
        msg.data = "Ready to calculate object height"
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    service = ObjectHeightService()
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
