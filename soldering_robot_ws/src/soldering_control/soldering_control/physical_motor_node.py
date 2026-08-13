#!/usr/bin/env python3
"""Feedback-only ROS node for one CAN-connected PhACT axis."""

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
from sensor_msgs.msg import JointState
from soldering_interfaces.msg import MotorTelemetry

from soldering_control.axis_discovery_core import StableAxisDiscovery
from soldering_control.load_holding_core import LoadHoldingClassifier, LoadState
from soldering_control.motor_axis_node import TELEMETRY_QOS
from soldering_control.physical_axis_core import PhysicalAxisEstimator


MOTION_WINDOW_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class PhysicalMotorNode(Node):
    """Route exactly one real feedback axis into one motor node."""

    def __init__(
        self,
        *,
        motor_id: int | None = None,
        axis_index: int | None = None,
        node_name: str = "physical_motor",
        namespace: str | None = None,
    ) -> None:
        dynamic_instance = motor_id is not None or axis_index is not None
        super().__init__(
            node_name,
            namespace=namespace,
            use_global_arguments=not dynamic_instance,
        )
        motor_id_default = 1 if motor_id is None else motor_id
        axis_index_default = -1 if axis_index is None else axis_index
        self._motor_id = int(
            self.declare_parameter("motor_id", motor_id_default).value
        )
        self._axis_index = int(
            self.declare_parameter("axis_index", axis_index_default).value
        )
        self._auto_bind = bool(
            self.declare_parameter("auto_bind", False).value
        )
        max_axes = int(self.declare_parameter("max_axes", 6).value)
        bind_stable_samples = int(
            self.declare_parameter("bind_stable_samples", 25).value
        )
        telemetry_rate_hz = float(
            self.declare_parameter("telemetry_rate_hz", 100.0).value
        )
        state_rate_hz = float(
            self.declare_parameter("state_rate_hz", 20.0).value
        )
        holding_enter_current_a = float(
            self.declare_parameter("holding_enter_current_a", 0.025).value
        )
        holding_exit_current_a = float(
            self.declare_parameter("holding_exit_current_a", 0.015).value
        )
        holding_exit_samples = int(
            self.declare_parameter("holding_exit_samples", 25).value
        )
        if not 1 <= self._motor_id <= 6:
            raise ValueError("motor_id must be in 1..6")
        if not self._auto_bind and not 0 <= self._axis_index < 12:
            raise ValueError("axis_index must be in 0..11")
        if self._auto_bind and self._axis_index != -1:
            raise ValueError("auto_bind requires axis_index=-1")
        if telemetry_rate_hz <= 0.0 or state_rate_hz <= 0.0:
            raise ValueError("publish rates must be positive")

        self._telemetry_period_ns = int(1.0e9 / telemetry_rate_hz)
        self._joint_state_period_ns = int(1.0e9 / state_rate_hz)
        self._last_feedback_ns = 0
        self._last_telemetry_ns = 0
        self._last_joint_state_ns = 0
        self._motion_busy = False
        self._first_valid_seen = False
        self._last_fault = False
        self._last_load_state = LoadState.UNKNOWN
        self._binding = StableAxisDiscovery(
            stable_samples=bind_stable_samples,
            max_axes=max_axes,
        )
        self._estimator = PhysicalAxisEstimator()
        self._load_classifier = LoadHoldingClassifier(
            enter_current_a=holding_enter_current_a,
            exit_current_a=holding_exit_current_a,
            exit_samples=holding_exit_samples,
        )
        self._telemetry_publisher = self.create_publisher(
            MotorTelemetry, "~/telemetry", TELEMETRY_QOS
        )
        self._state_publisher = self.create_publisher(
            JointState, "~/state", 10
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
            MOTION_WINDOW_QOS,
        )
        if self._auto_bind:
            self.get_logger().info(
                f"motor {self._motor_id} waiting for auto-detected axis "
                "(feedback only; CAN is terminated by PCM)"
            )
        else:
            self.get_logger().info(
                f"motor {self._motor_id} bound to real axis[{self._axis_index}] "
                "(feedback only; CAN is terminated by PCM)"
            )

    def _on_motion_window(self, msg: MotionWindowStatus) -> None:
        self._motion_busy = bool(msg.status_valid and msg.busy)

    def _on_feedback(self, msg: PhorceFeedback) -> None:
        if self._auto_bind and self._axis_index < 0:
            newly_finalized = self._binding.update(
                valid_mask=msg.axis_valid_mask,
                oper_mask=msg.axis_oper_mask,
                fault_mask=msg.axis_fault_mask,
                stale_mask=msg.axis_stale_mask,
            )
            if not newly_finalized:
                return
            detected = self._binding.axis_for_motor(self._motor_id)
            if detected is None:
                self.get_logger().info(
                    f"motor {self._motor_id} inactive: detected axes="
                    f"{self._binding.axes} mask=0x{self._binding.mask:03x}"
                )
                return
            self._axis_index = detected
            self.get_logger().info(
                f"motor {self._motor_id} auto-bound to axis[{detected}] "
                f"from mask=0x{self._binding.mask:03x}"
            )
        if self._axis_index < 0:
            return
        axis = msg.axis[self._axis_index]
        feedback_ns = int(msg.recv_monotonic_ns)
        if self._last_feedback_ns:
            dt_s = (feedback_ns - self._last_feedback_ns) / 1.0e9
            if not 0.0001 <= dt_s <= 0.1:
                dt_s = 0.001
        else:
            dt_s = 0.001
        self._last_feedback_ns = feedback_ns

        estimate = self._estimator.update(
            position_rad=axis.position_rad,
            velocity_rad_s=axis.velocity_rad_s,
            reference_rad=axis.pos_ref_echo_rad,
            dt_s=dt_s,
            motion_busy=self._motion_busy,
        )
        load = self._load_classifier.update(
            current_a=axis.current_a,
            disturbance_current_a=axis.dob_a,
            valid=axis.valid,
            oper=axis.oper,
            fault=axis.fault,
            stale=axis.stale,
            motion_busy=self._motion_busy,
        )
        if load.state != self._last_load_state:
            self.get_logger().info(
                f"load state={load.state.name} evidence={load.evidence_a:.3f}A "
                f"holding={load.holding} (not torque-off permission)"
            )
            self._last_load_state = load.state
        if axis.valid and not self._first_valid_seen:
            self.get_logger().info(
                f"first valid feedback: position={axis.position_rad:+.4f}rad "
                f"bus={axis.bus_v:.2f}V temp={axis.temp_c:.1f}C"
            )
            self._first_valid_seen = True
        if axis.fault != self._last_fault:
            log = (
                self.get_logger().error
                if axis.fault
                else self.get_logger().info
            )
            log(f"fault changed to {axis.fault}")
            self._last_fault = axis.fault

        if feedback_ns - self._last_telemetry_ns >= self._telemetry_period_ns:
            self._publish_telemetry(msg, axis, estimate, load)
            self._last_telemetry_ns = feedback_ns
        joint_state_due = (
            feedback_ns - self._last_joint_state_ns
            >= self._joint_state_period_ns
        )
        if joint_state_due:
            self._publish_joint_state(msg, axis)
            self._last_joint_state_ns = feedback_ns

    def _publish_telemetry(self, msg, axis, estimate, load) -> None:
        telemetry = MotorTelemetry()
        telemetry.header.stamp = msg.stamp
        telemetry.header.frame_id = f"motor_{self._motor_id}"
        telemetry.motor_id = self._motor_id
        telemetry.axis_index = self._axis_index
        telemetry.connected = bool(axis.valid or axis.oper)
        telemetry.valid = axis.valid
        telemetry.oper = axis.oper
        telemetry.fault = axis.fault
        telemetry.stale = axis.stale
        telemetry.sequence = msg.tx_cycle_seq
        telemetry.target_position_rad = estimate.desired_position_rad
        telemetry.desired_position_rad = estimate.desired_position_rad
        telemetry.desired_velocity_rad_s = estimate.desired_velocity_rad_s
        telemetry.desired_acceleration_rad_s2 = (
            estimate.desired_acceleration_rad_s2
        )
        telemetry.measured_position_rad = axis.position_rad
        telemetry.filtered_position_rad = estimate.filtered_position_rad
        telemetry.estimated_velocity_rad_s = axis.velocity_rad_s
        telemetry.estimated_acceleration_rad_s2 = (
            estimate.estimated_acceleration_rad_s2
        )
        telemetry.current_a = axis.current_a
        telemetry.disturbance_current_a = axis.dob_a
        telemetry.load_state = int(load.state)
        telemetry.holding_load = load.holding
        telemetry.load_evidence_a = load.evidence_a
        telemetry.release_permitted = load.release_permitted
        telemetry.bus_voltage_v = axis.bus_v
        telemetry.temperature_c = axis.temp_c
        telemetry.absolute_encoder_valid = bool(axis.abs_valid)
        telemetry.position_error_rad = estimate.position_error_rad
        telemetry.velocity_error_rad_s = estimate.velocity_error_rad_s
        telemetry.control_acceleration_rad_s2 = 0.0
        self._telemetry_publisher.publish(telemetry)

    def _publish_joint_state(self, msg, axis) -> None:
        state = JointState()
        state.header.stamp = msg.stamp
        state.name = [f"motor_{self._motor_id}"]
        state.position = [axis.position_rad]
        state.velocity = [axis.velocity_rad_s]
        state.effort = [axis.current_a]
        self._state_publisher.publish(state)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PhysicalMotorNode()
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
