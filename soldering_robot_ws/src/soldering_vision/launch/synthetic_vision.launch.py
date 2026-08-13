from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="soldering_vision",
                executable="synthetic_scene",
                name="synthetic_scene",
                output="screen",
            ),
            Node(
                package="soldering_vision",
                executable="vision_observer",
                name="vision_observer",
                output="screen",
                parameters=[
                    {
                        "detector_backend": "heuristic",
                        "confidence_min": 0.5,
                        "inference_hz": 10.0,
                        "homography_valid": True,
                        "homography": [
                            0.1,
                            0.0,
                            -32.0,
                            0.0,
                            0.1,
                            -24.0,
                            0.0,
                            0.0,
                            1.0,
                        ],
                    }
                ],
            ),
        ]
    )
