from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("soldering_vision")
    config = os.path.join(package_share, "config", "vision.yaml")
    camera_device = LaunchConfiguration("camera_device")
    detector_backend = LaunchConfiguration("detector_backend")
    yolo_model = LaunchConfiguration("yolo_model")
    convnext_model = LaunchConfiguration("convnext_model")
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("detector_backend", default_value="heuristic"),
            DeclareLaunchArgument("yolo_model", default_value=""),
            DeclareLaunchArgument("convnext_model", default_value=""),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="camera",
                output="screen",
                parameters=[config, {"video_device": camera_device}],
            ),
            Node(
                package="soldering_vision",
                executable="vision_observer",
                name="vision_observer",
                output="screen",
                parameters=[
                    config,
                    {
                        "detector_backend": detector_backend,
                        "yolo_model": yolo_model,
                        "convnext_model": convnext_model,
                    },
                ],
            ),
        ]
    )
