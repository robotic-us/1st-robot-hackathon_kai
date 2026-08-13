"""One-command real bringup for the two currently installed motors."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    monitor = Node(
        package="agx_phorce_bridge",
        executable="phorce_monitor",
        name="phorce_monitor",
        output="screen",
        parameters=[
            {
                "nic": "eno1",
                "mode": "op_idle",
                # Native bridge two-pass detection: observe a stable non-zero
                # PCM oper mask, close that session, then reopen OP-IDLE with
                # the detected mask as the explicit expected-axis contract.
                "axes": "auto",
                "mbx_enabled": True,
                "motion_poll_hz": 2.0,
                "rt_cpu": 8,
                "priority": 80,
            }
        ],
    )
    motor_nodes = Node(
        package="soldering_control",
        executable="auto_motor_nodes",
        name="auto_motor_nodes",
        output="screen",
        parameters=[{"max_axes": 6, "stable_samples": 25}],
    )
    setup = Node(
        package="soldering_control",
        executable="hardware_setup_node",
        name="hardware_setup",
        output="screen",
        parameters=[
            {
                "auto_axes": True,
                "max_axes": 6,
                "discovery_stable_samples": 25,
                "stop_velocity_rad_s": 0.02,
                "stable_duration_s": 1.0,
                "feedback_timeout_s": 0.10,
                "window_timeout_s": 1.50,
            }
        ],
    )
    action_server = Node(
        package="agx_motion_slot",
        executable="motion_action_server",
        name="motion_action_server",
        output="screen",
        parameters=[{"backend": "ecat"}],
    )
    stop_if_monitor_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="phorce_monitor exited; hardware setup is fail-closed"
                    )
                )
            ],
        )
    )
    delayed_hardware_stack = TimerAction(
        period=2.0,
        actions=[motor_nodes, setup, action_server],
    )
    return LaunchDescription(
        [stop_if_monitor_exits, monitor, delayed_hardware_stack]
    )
