from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="object_height",
                executable="object_height_service",
                name="object_height_service",
                output="screen",
                emulate_tty=True,
            )
        ]
    )
