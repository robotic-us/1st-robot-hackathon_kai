"""Bind the two installed CAN motors to physical ROS observation nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bindings = ((1, 0), (2, 7))
    nodes = [
        Node(
            package="soldering_control",
            executable="physical_motor_node",
            namespace="motors",
            name=f"motor_{motor_id}",
            output="screen",
            parameters=[
                {
                    "motor_id": motor_id,
                    "axis_index": axis_index,
                    "telemetry_rate_hz": 100.0,
                    "state_rate_hz": 20.0,
                }
            ],
        )
        for motor_id, axis_index in bindings
    ]
    return LaunchDescription(nodes)
