"""Launch six independent simulated motor controller processes."""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    axis_indices = (0, 7, -1, -1, -1, -1)
    motor_nodes = [
        Node(
            package="soldering_control",
            executable="motor_axis_node",
            namespace="motors",
            name=f"motor_{motor_id}",
            output="screen",
            parameters=[
                {
                    "motor_id": motor_id,
                    "axis_index": axis_indices[motor_id - 1],
                    "simulation": True,
                    "control_rate_hz": 100.0,
                    "state_rate_hz": 20.0,
                    "trajectory_duration_s": 5.0,
                }
            ],
        )
        for motor_id in range(1, 7)
    ]
    coordinator = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="soldering_control",
                executable="six_motor_coordinator",
                name="six_motor_coordinator",
                output="screen",
                parameters=[{"segment_s": 5.0}],
            )
        ],
    )
    master_mcu = Node(
        package="soldering_control",
        executable="master_mcu_node",
        name="master_mcu",
        output="screen",
        parameters=[{"stale_timeout_s": 0.10}],
    )
    return LaunchDescription(
        [*motor_nodes, master_mcu, coordinator]
    )
