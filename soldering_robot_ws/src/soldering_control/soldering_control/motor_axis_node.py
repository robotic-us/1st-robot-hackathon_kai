#!/usr/bin/env python3
"""ROS 2 node that owns the control state of one motor."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from soldering_interfaces.msg import MotorControlCommand, MotorTelemetry
from std_msgs.msg import Float64, Float64MultiArray

from soldering_control.motor_axis_core import MotorAxisCore


TARGET_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
TELEMETRY_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
CONTROL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class MotorAxisNode(Node):
    """One node, one motor, one independent controller state."""

    def __init__(self) -> None:
        super().__init__("motor_axis")
        motor_id = int(self.declare_parameter("motor_id", 1).value)
        axis_index = int(self.declare_parameter("axis_index", -1).value)
        simulation = bool(self.declare_parameter("simulation", True).value)
        control_rate_hz = float(
            self.declare_parameter("control_rate_hz", 100.0).value
        )
        state_rate_hz = float(
            self.declare_parameter("state_rate_hz", 20.0).value
        )
        trajectory_duration_s = float(
            self.declare_parameter("trajectory_duration_s", 5.0).value
        )
        if not simulation:
            raise RuntimeError(
                "real motor output is disabled until a PCM/P-Vector adapter "
                "is configured"
            )
        if motor_id < 1 or control_rate_hz <= 0.0 or state_rate_hz <= 0.0:
            raise ValueError("invalid motor id or node rate")

        self._motor_id = motor_id
        self._axis_index = axis_index
        self._dt_s = 1.0 / control_rate_hz
        self._publish_every = max(1, round(control_rate_hz / state_rate_hz))
        self._ticks = 0
        self._sequence = 0
        self._acceleration_command_deg_s2 = 0.0
        self._last_command_ns = 0
        self._has_received_command = False
        self._command_timeout_ns = int(0.10 * 1.0e9)
        self._core = MotorAxisCore(
            trajectory_duration_s=trajectory_duration_s,
            random_seed=20260806 + motor_id,
        )
        self._state_publisher = self.create_publisher(
            JointState, "~/state", 10
        )
        self._debug_publisher = self.create_publisher(
            Float64MultiArray, "~/control_state", 10
        )
        self._command_publisher = self.create_publisher(
            Float64, "~/desired_deg", 10
        )
        self._telemetry_publisher = self.create_publisher(
            MotorTelemetry, "~/telemetry", TELEMETRY_QOS
        )
        self.create_subscription(
            Float64, "~/target_deg", self._on_target, TARGET_QOS
        )
        self.create_subscription(
            MotorControlCommand,
            "~/control_command",
            self._on_control_command,
            CONTROL_QOS,
        )
        self.create_timer(self._dt_s, self._on_control_tick)
        self.get_logger().info(
            f"motor {motor_id} ready: axis_index={axis_index}, "
            f"simulation=true, rate={control_rate_hz:.1f}Hz"
        )

    def _on_target(self, msg: Float64) -> None:
        if not math.isfinite(msg.data):
            self.get_logger().error("rejected non-finite target")
            return
        if self._core.set_target(msg.data):
            self.get_logger().info(f"new target={msg.data:+.3f}deg")

    def _on_control_command(self, msg: MotorControlCommand) -> None:
        if msg.motor_id != self._motor_id:
            self.get_logger().error(
                f"rejected command for motor_id={msg.motor_id}"
            )
            return
        if msg.mode != MotorControlCommand.MODE_ACCELERATION:
            self._acceleration_command_deg_s2 = 0.0
            return
        if not math.isfinite(msg.acceleration_command_rad_s2):
            self.get_logger().error("rejected non-finite control command")
            return
        self._acceleration_command_deg_s2 = math.degrees(
            msg.acceleration_command_rad_s2
        )
        self._last_command_ns = self.get_clock().now().nanoseconds
        if not self._has_received_command:
            self.get_logger().info("first master-MCU command received")
            self._has_received_command = True

    def _on_control_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        command_is_stale = (
            self._last_command_ns == 0
            or now_ns - self._last_command_ns > self._command_timeout_ns
        )
        acceleration_command_deg_s2 = (
            0.0
            if command_is_stale
            else self._acceleration_command_deg_s2
        )
        state = self._core.step(
            self._dt_s, acceleration_command_deg_s2
        )
        self._sequence += 1
        command = Float64()
        command.data = state.desired_position_deg
        self._command_publisher.publish(command)

        stamp = self.get_clock().now().to_msg()
        telemetry = MotorTelemetry()
        telemetry.header.stamp = stamp
        telemetry.header.frame_id = f"motor_{self._motor_id}"
        telemetry.motor_id = self._motor_id
        telemetry.axis_index = self._axis_index
        telemetry.connected = self._axis_index >= 0
        telemetry.valid = True
        telemetry.oper = True
        telemetry.fault = False
        telemetry.stale = False
        telemetry.sequence = self._sequence
        telemetry.target_position_rad = math.radians(state.target_deg)
        telemetry.desired_position_rad = math.radians(
            state.desired_position_deg
        )
        telemetry.desired_velocity_rad_s = math.radians(
            state.desired_velocity_deg_s
        )
        telemetry.desired_acceleration_rad_s2 = math.radians(
            state.desired_acceleration_deg_s2
        )
        telemetry.measured_position_rad = math.radians(
            state.measured_position_deg
        )
        telemetry.filtered_position_rad = math.radians(
            state.filtered_position_deg
        )
        telemetry.estimated_velocity_rad_s = math.radians(
            state.velocity_deg_s
        )
        telemetry.estimated_acceleration_rad_s2 = math.radians(
            state.acceleration_deg_s2
        )
        telemetry.load_state = MotorTelemetry.LOAD_UNKNOWN
        telemetry.holding_load = False
        telemetry.load_evidence_a = 0.0
        telemetry.release_permitted = False
        telemetry.position_error_rad = math.radians(state.error_deg)
        telemetry.velocity_error_rad_s = math.radians(
            state.velocity_error_deg_s
        )
        telemetry.control_acceleration_rad_s2 = math.radians(
            state.acceleration_command_deg_s2
        )
        self._telemetry_publisher.publish(telemetry)

        self._ticks += 1
        if self._ticks % self._publish_every:
            return
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [f"motor_{self._motor_id}"]
        joint_state.position = [math.radians(state.filtered_position_deg)]
        joint_state.velocity = [math.radians(state.velocity_deg_s)]
        self._state_publisher.publish(joint_state)

        debug = Float64MultiArray()
        debug.data = [
            state.target_deg,
            state.desired_position_deg,
            state.filtered_position_deg,
            state.velocity_deg_s,
            state.error_deg,
            state.acceleration_command_deg_s2,
            float(self._axis_index),
        ]
        self._debug_publisher.publish(debug)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotorAxisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
