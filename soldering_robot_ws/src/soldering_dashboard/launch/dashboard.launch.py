"""Launch only the read-mostly operator dashboard.

The dashboard never starts a hardware bridge or motor stack by itself.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="soldering_dashboard",
                executable="dashboard",
                name="soldering_dashboard",
                output="screen",
            )
        ]
    )
