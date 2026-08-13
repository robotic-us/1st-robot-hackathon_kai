#!/usr/bin/env python3
"""Automatically validate hardware and publish a fail-closed arm state."""

from __future__ import annotations

from agx_msgs.msg import MotionWindowStatus, PhorceFeedback
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String, UInt16

from soldering_control.axis_discovery_core import StableAxisDiscovery
from soldering_control.hardware_setup_core import (
    AxisReadiness,
    readiness_issues,
)


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class HardwareSetupNode(Node):
    """Arm only after stable feedback and positive PCM readiness."""

    def __init__(self) -> None:
        super().__init__("hardware_setup")
        self._axis_indices = list(
            self.declare_parameter("axis_indices", [0, 7]).value
        )
        self._auto_axes = bool(
            self.declare_parameter("auto_axes", False).value
        )
        max_axes = int(self.declare_parameter("max_axes", 6).value)
        discovery_stable_samples = int(
            self.declare_parameter("discovery_stable_samples", 25).value
        )
        self._stop_velocity_rad_s = float(
            self.declare_parameter("stop_velocity_rad_s", 0.02).value
        )
        stable_duration_s = float(
            self.declare_parameter("stable_duration_s", 1.0).value
        )
        self._feedback_timeout_ns = int(
            float(self.declare_parameter("feedback_timeout_s", 0.10).value)
            * 1.0e9
        )
        self._window_timeout_ns = int(
            float(self.declare_parameter("window_timeout_s", 1.50).value)
            * 1.0e9
        )
        if (not self._auto_axes and not self._axis_indices) or any(
            index < 0 or index >= 12 for index in self._axis_indices
        ):
            raise ValueError("axis_indices must contain values in 0..11")
        if self._stop_velocity_rad_s <= 0.0 or stable_duration_s <= 0.0:
            raise ValueError("velocity and duration limits must be positive")

        self._stable_ticks_required = max(1, round(stable_duration_s * 10))
        self._stable_ticks = 0
        self._feedback: PhorceFeedback | None = None
        self._feedback_received_ns = 0
        self._window: MotionWindowStatus | None = None
        self._window_received_ns = 0
        self._discovery = StableAxisDiscovery(
            stable_samples=discovery_stable_samples,
            max_axes=max_axes,
        )
        self._armed = False
        self._last_state = ""
        self._arm_publisher = self.create_publisher(
            Bool, "/soldering/software_armed", LATCHED_QOS
        )
        self._state_publisher = self.create_publisher(
            String, "/soldering/setup_state", LATCHED_QOS
        )
        self._axis_mask_publisher = self.create_publisher(
            UInt16, "/soldering/detected_axis_mask", LATCHED_QOS
        )
        self.create_subscription(
            PhorceFeedback,
            "/phorce/feedback",
            self._on_feedback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            MotionWindowStatus,
            "/phorce/motion_window",
            self._on_motion_window,
            LATCHED_QOS,
        )
        self.create_timer(0.1, self._evaluate)
        if self._auto_axes:
            self._axis_indices = []
            self.get_logger().info("hardware setup waiting for auto-detected axes")
        else:
            mask = sum(1 << index for index in self._axis_indices)
            self.get_logger().info(
                f"hardware setup waiting for axes={self._axis_indices} "
                f"mask=0x{mask:04x}"
            )

    def _on_feedback(self, msg: PhorceFeedback) -> None:
        self._feedback = msg
        self._feedback_received_ns = self.get_clock().now().nanoseconds
        if self._auto_axes and not self._discovery.finalized:
            if self._discovery.update(
                valid_mask=msg.axis_valid_mask,
                oper_mask=msg.axis_oper_mask,
                fault_mask=msg.axis_fault_mask,
                stale_mask=msg.axis_stale_mask,
            ):
                self._axis_indices = list(self._discovery.axes)
                mask_msg = UInt16()
                mask_msg.data = self._discovery.mask
                self._axis_mask_publisher.publish(mask_msg)
                self.get_logger().info(
                    f"detected axes={self._axis_indices} "
                    f"mask=0x{self._discovery.mask:04x}"
                )

    def _on_motion_window(self, msg: MotionWindowStatus) -> None:
        self._window = msg
        self._window_received_ns = self.get_clock().now().nanoseconds

    def _evaluate(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        issues = []
        if self._auto_axes and not self._discovery.finalized:
            issues.append(self._discovery.issue)
        if (
            self._feedback is None
            or now_ns - self._feedback_received_ns > self._feedback_timeout_ns
        ):
            issues.append("feedback_unavailable")
        else:
            if self._auto_axes and self._discovery.finalized:
                observed_mask = self._feedback.axis_oper_mask & 0x0FFF
                if observed_mask != self._discovery.mask:
                    issues.append(
                        "axis_mask_changed:"
                        f"0x{self._discovery.mask:03x}->0x{observed_mask:03x}"
                    )
            axes = [
                AxisReadiness(
                    axis_index=index,
                    valid=self._feedback.axis[index].valid,
                    oper=self._feedback.axis[index].oper,
                    fault=self._feedback.axis[index].fault,
                    stale=self._feedback.axis[index].stale,
                    absolute_encoder_valid=bool(
                        self._feedback.axis[index].abs_valid
                    ),
                    velocity_rad_s=self._feedback.axis[index].velocity_rad_s,
                )
                for index in self._axis_indices
            ]
            window_fresh = (
                self._window is not None
                and now_ns - self._window_received_ns
                <= self._window_timeout_ns
            )
            window_ready = bool(
                window_fresh
                and self._window.window_present
                and self._window.status_valid
            )
            physical_idle = bool(
                window_ready and self._window.physical_idle
            )
            issues.extend(
                readiness_issues(
                    axes,
                    stop_velocity_rad_s=self._stop_velocity_rad_s,
                    motion_window_ready=window_ready,
                    physical_idle=physical_idle,
                )
            )

        if issues:
            self._stable_ticks = 0
            armed = False
            state = "WAITING: " + ", ".join(issues)
        else:
            self._stable_ticks += 1
            armed = self._stable_ticks >= self._stable_ticks_required
            state = (
                "ARMED"
                if armed
                else "STABILIZING: all readiness checks passed"
            )
        self._armed = armed
        arm_msg = Bool()
        arm_msg.data = armed
        self._arm_publisher.publish(arm_msg)
        state_msg = String()
        state_msg.data = state
        self._state_publisher.publish(state_msg)
        if state != self._last_state:
            if armed:
                self.get_logger().info("SOFTWARE ARMED")
            elif self._last_state == "ARMED":
                self.get_logger().error(f"SOFTWARE DISARMED: {state}")
            else:
                self.get_logger().info(state)
            self._last_state = state


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HardwareSetupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
