#!/usr/bin/env python3
"""Create exactly one physical ROS node for every auto-detected PCM axis."""

from __future__ import annotations

from collections.abc import Callable

from agx_msgs.msg import PhorceFeedback
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from soldering_control.axis_discovery_core import (
    StableAxisDiscovery,
    motor_bindings_from_mask,
)
from soldering_control.physical_motor_node import PhysicalMotorNode


class MotorNodeAllocator(Node):
    """Latch the session mask, then request one-time node allocation."""

    def __init__(
        self,
        on_detected: Callable[[list[tuple[int, int]]], None],
    ) -> None:
        super().__init__("motor_node_allocator")
        max_axes = int(self.declare_parameter("max_axes", 6).value)
        stable_samples = int(
            self.declare_parameter("stable_samples", 25).value
        )
        self._discovery = StableAxisDiscovery(
            stable_samples=stable_samples,
            max_axes=max_axes,
        )
        self._on_detected = on_detected
        self.create_subscription(
            PhorceFeedback,
            "/phorce/feedback",
            self._on_feedback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "waiting for the bridge's auto-detected PCM axis mask"
        )

    def _on_feedback(self, msg: PhorceFeedback) -> None:
        if not self._discovery.update(
            valid_mask=msg.axis_valid_mask,
            oper_mask=msg.axis_oper_mask,
            fault_mask=msg.axis_fault_mask,
            stale_mask=msg.axis_stale_mask,
        ):
            return
        bindings = motor_bindings_from_mask(self._discovery.mask)
        self.get_logger().info(
            f"allocating motor nodes: mask=0x{self._discovery.mask:03x} "
            f"bindings={bindings}"
        )
        self._on_detected(bindings)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    motor_nodes: list[PhysicalMotorNode] = []

    def allocate(bindings: list[tuple[int, int]]) -> None:
        if motor_nodes:
            return
        for motor_id, axis_index in bindings:
            node = PhysicalMotorNode(
                motor_id=motor_id,
                axis_index=axis_index,
                node_name=f"motor_{motor_id}",
                namespace="/motors",
            )
            motor_nodes.append(node)
            executor.add_node(node)

    allocator = MotorNodeAllocator(allocate)
    executor.add_node(allocator)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in motor_nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.remove_node(allocator)
        allocator.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
