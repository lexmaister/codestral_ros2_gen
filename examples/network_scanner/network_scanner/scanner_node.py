#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random

from network_scanner.msg import NetworkStatus, IPStatus


class NetworkScannerNode(Node):
    def __init__(self):
        super().__init__("network_scanner_node")
        self.publisher_ = self.create_publisher(NetworkStatus, "network_status", 10)
        self.timer = self.create_timer(5.0, self.publish_status)
        self.get_logger().info(
            "Network scanner node started. Publishing every 5 seconds."
        )

    def publish_status(self):
        msg = NetworkStatus()
        # Set the current time using the node's clock.
        msg.stamp = self.get_clock().now().to_msg()

        # Create a single IPStatus message for 192.168.1.1
        ip_status = IPStatus()
        ip_status.ip_address = "192.168.1.1"
        ip_status.status = random.choice(["online", "offline"])

        # Add the IPStatus message to the addresses array.
        msg.addresses = [ip_status]

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published status for 192.168.1.1 as {ip_status.status}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NetworkScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
