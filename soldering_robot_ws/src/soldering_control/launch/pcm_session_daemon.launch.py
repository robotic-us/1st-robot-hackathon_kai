from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="soldering_control",
                executable="pcm_session_daemon",
                name="pcm_session_daemon",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/ttyACM0",
                        "media_device": "/dev/sda1",
                        "default_axes": [7],
                        "queue_capacity": 16,
                        "connect_on_start": True,
                        "sdo_timeout_s": 1.5,
                    }
                ],
            )
        ]
    )
