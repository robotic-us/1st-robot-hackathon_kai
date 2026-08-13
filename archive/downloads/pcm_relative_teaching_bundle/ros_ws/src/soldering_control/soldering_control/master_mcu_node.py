#!/usr/bin/env python3
"""Master-MCU control node: observe six motors and return commands."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from soldering_interfaces.msg import (
    MotorControlCommand,
    MotorTelemetry,
    MotorTelemetryArray,
)

from soldering_control.master_mcu_core import (
    AxisObservation,
    MasterAxisController,
)
from soldering_control.motor_axis_node import CONTROL_QOS, TELEMETRY_QOS


class MasterMcuNode(Node):
    """ROS representation of the master MCU's six-axis control loop."""

    def __init__(self) -> None:
        super().__init__("master_mcu")
        stale_timeout_s = float(
            self.declare_parameter("stale_timeout_s", 0.10).value
        )
        if stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive")
        self._stale_timeout_ns = int(stale_timeout_s * 1.0e9)
        self._latest: list[MotorTelemetry | None] = [None] * 6
        self._received_ns = [0] * 6
        self._last_received_mask = -1
        self._controllers = [MasterAxisController() for _ in range(6)]
        self._command_publishers = [
            self.create_publisher(
                MotorControlCommand,
                f"/motors/motor_{motor_id}/control_command",
                CONTROL_QOS,
            )
            for motor_id in range(1, 7)
        ]
        self._observation_publisher = self.create_publisher(
            MotorTelemetryArray,
            "/master_mcu/observations",
            TELEMETRY_QOS,
        )
        self._subscriptions = [
            self.create_subscription(
                MotorTelemetry,
                f"/motors/motor_{motor_id}/telemetry",
                self._callback_for(motor_id),
                TELEMETRY_QOS,
            )
            for motor_id in range(1, 7)
        ]
        self.create_timer(0.01, self._publish_observation_frame)

    def _callback_for(self, motor_id: int):
        def callback(msg: MotorTelemetry) -> None:
            if msg.motor_id != motor_id:
                self.get_logger().error(
                    f"topic motor_{motor_id} carried motor_id={msg.motor_id}"
                )
                return
            values = (
                msg.desired_position_rad,
                msg.desired_velocity_rad_s,
                msg.desired_acceleration_rad_s2,
                msg.measured_position_rad,
                msg.estimated_velocity_rad_s,
            )
            if not all(math.isfinite(value) for value in values):
                self.get_logger().error(
                    f"motor {motor_id}: rejected non-finite observation"
                )
                return
            index = motor_id - 1
            observation = AxisObservation(*values)
            acceleration = self._controllers[index].update(observation)
            command = MotorControlCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = "master_mcu"
            command.motor_id = motor_id
            command.observation_sequence = msg.sequence
            command.mode = MotorControlCommand.MODE_ACCELERATION
            command.acceleration_command_rad_s2 = acceleration
            self._command_publishers[index].publish(command)
            self._latest[index] = msg
            self._received_ns[index] = self.get_clock().now().nanoseconds

        return callback

    def _publish_observation_frame(self) -> None:
        now = self.get_clock().now()
        frame = MotorTelemetryArray()
        frame.header.stamp = now.to_msg()
        frame.header.frame_id = "master_mcu"
        motors = []
        received_mask = 0
        for index, latest in enumerate(self._latest):
            if latest is None:
                missing = MotorTelemetry()
                missing.motor_id = index + 1
                missing.axis_index = -1
                missing.stale = True
                motors.append(missing)
                continue
            age_ns = now.nanoseconds - self._received_ns[index]
            latest.stale = age_ns > self._stale_timeout_ns
            if not latest.stale:
                received_mask |= 1 << index
            motors.append(latest)
        frame.received_mask = received_mask
        frame.motors = motors
        self._observation_publisher.publish(frame)
        if received_mask != self._last_received_mask:
            self.get_logger().info(
                f"observed motor mask=0x{received_mask:02x}"
            )
            self._last_received_mask = received_mask


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MasterMcuNode()
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
