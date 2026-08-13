"""ROS subscriptions and service calls isolated from the Qt GUI thread."""

from __future__ import annotations

import queue
import threading
import time
from typing import Any

from PyQt5 import QtCore

from agx_msgs.msg import MotionWindowStatus, PhorceFeedback, PhorceStatus
from cv_bridge import CvBridge
from rcl_interfaces.msg import Log
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from soldering_interfaces.msg import GeometryObservation, MotorTelemetry
from soldering_interfaces.srv import PcmCommand
from std_msgs.msg import Bool, String

from .dashboard_core import (
    SlidingRate,
    TopicHealth,
    classify_topic,
    parse_pcm_status,
)


TELEMETRY_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
ROSOUT_QOS = QoSProfile(
    depth=1000,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class DashboardRosNode(Node):
    """Read robot state and serialize deliberate PCM service requests."""

    def __init__(self, worker: "RosWorker", context: Context):
        super().__init__("soldering_dashboard", context=context)
        self._worker = worker
        self._bridge = CvBridge()
        self._last_image_emit = 0.0
        self._last_motor_emit = [0.0] * 6
        self._pending_futures: set[Any] = set()
        self._command_queue = worker.command_queue

        self._thresholds = {
            "/phorce/feedback": 0.10,
            "/phorce/status": 0.50,
            "/phorce/motion_window": 2.0,
            "/soldering/software_armed": 0.50,
            "/soldering/setup_state": 0.50,
            "/pcm_session_daemon/status": 10.0,
            "/soldering/geometry_observation": 1.0,
            "/soldering/vision/annotated": 1.0,
            "/rosout": 10.0,
        }
        for motor_id in range(1, 7):
            self._thresholds[
                f"/motors/motor_{motor_id}/telemetry"
            ] = 0.25
        self._rates = {
            topic: SlidingRate(window_s=5.0) for topic in self._thresholds
        }
        self._dashboard_subscriptions = []

        self._dashboard_subscriptions.append(
            self.create_subscription(
                PhorceFeedback,
                "/phorce/feedback",
                self._on_feedback,
                qos_profile_sensor_data,
            )
        )
        self._dashboard_subscriptions.append(
            self.create_subscription(
                PhorceStatus, "/phorce/status", self._on_phorce_status, 10
            )
        )
        self._dashboard_subscriptions.append(
            self.create_subscription(
                MotionWindowStatus,
                "/phorce/motion_window",
                self._on_motion_window,
                LATCHED_QOS,
            )
        )
        self._dashboard_subscriptions.append(
            self.create_subscription(
                Bool,
                "/soldering/software_armed",
                self._on_software_armed,
                LATCHED_QOS,
            )
        )
        self._dashboard_subscriptions.append(
            self.create_subscription(
                String,
                "/soldering/setup_state",
                self._on_setup_state,
                LATCHED_QOS,
            )
        )
        self._dashboard_subscriptions.append(
            self.create_subscription(
                String,
                "/pcm_session_daemon/status",
                self._on_pcm_status,
                LATCHED_QOS,
            )
        )
        self._dashboard_subscriptions.append(
            self.create_subscription(
                GeometryObservation,
                "/soldering/geometry_observation",
                self._on_geometry,
                10,
            )
        )
        self._dashboard_subscriptions.append(
            self.create_subscription(
                Image,
                "/soldering/vision/annotated",
                self._on_image,
                qos_profile_sensor_data,
            )
        )
        self._dashboard_subscriptions.append(
            self.create_subscription(
                Log, "/rosout", self._on_rosout, ROSOUT_QOS
            )
        )
        for motor_id in range(1, 7):
            topic = f"/motors/motor_{motor_id}/telemetry"
            self._dashboard_subscriptions.append(
                self.create_subscription(
                    MotorTelemetry,
                    topic,
                    self._motor_callback(motor_id, topic),
                    TELEMETRY_QOS,
                )
            )

        self._pcm_client = self.create_client(
            PcmCommand, "/pcm_session_daemon/command"
        )
        self.create_timer(0.05, self._drain_commands)
        self.create_timer(1.0, self._publish_topic_health)
        self.get_logger().info("dashboard ROS monitor ready (no command sent)")

    def _tick(self, topic: str) -> None:
        self._rates[topic].tick()

    def _on_feedback(self, _msg: PhorceFeedback) -> None:
        self._tick("/phorce/feedback")

    def _on_phorce_status(self, msg: PhorceStatus) -> None:
        self._tick("/phorce/status")
        self._worker.bridge_status.emit(
            {
                "mode": msg.mode,
                "feedback_rate_hz": float(msg.feedback_rate_hz),
                "ethercat_operational": bool(msg.ethercat_operational),
                "estop_active": bool(msg.estop_active),
                "master_state": int(msg.master_state),
                "axis_oper_mask": int(msg.axis_oper_mask),
                "axis_fault_mask": int(msg.axis_fault_mask),
                "jitter_max_us": float(msg.jitter_max_us),
                "deadline_skips": int(msg.deadline_skips),
                "mbx_lane_up": bool(msg.mbx_lane_up),
            }
        )

    def _on_motion_window(self, msg: MotionWindowStatus) -> None:
        self._tick("/phorce/motion_window")
        self._worker.motion_window.emit(
            {
                "present": bool(msg.window_present),
                "valid": bool(msg.status_valid),
                "busy": bool(msg.busy),
                "physical_idle": bool(msg.physical_idle),
                "active_slot": int(msg.active_slot),
                "requested_slot": int(msg.requested_slot),
                "loaded_slot_mask": int(msg.loaded_slot_mask),
                "state": int(msg.state),
                "reason": int(msg.reason),
            }
        )

    def _on_software_armed(self, msg: Bool) -> None:
        self._tick("/soldering/software_armed")
        self._worker.armed.emit(bool(msg.data))

    def _on_setup_state(self, msg: String) -> None:
        self._tick("/soldering/setup_state")
        self._worker.setup_state.emit(str(msg.data))

    def _on_pcm_status(self, msg: String) -> None:
        self._tick("/pcm_session_daemon/status")
        self._worker.pcm_status.emit(parse_pcm_status(msg.data))

    def _on_geometry(self, msg: GeometryObservation) -> None:
        self._tick("/soldering/geometry_observation")
        self._worker.vision_status.emit(
            {
                "valid": bool(msg.valid),
                "calibrated": bool(msg.calibrated),
                "backend": str(msg.detector_backend),
                "inference_ms": float(msg.inference_ms),
                "image_age_ms": float(msg.image_age_ms),
                "objects": len(msg.objects),
                "process_class": str(msg.process_class),
                "process_confidence": float(msg.process_confidence),
                "wire_error_x_mm": float(msg.wire_error_x_mm),
                "wire_error_y_mm": float(msg.wire_error_y_mm),
                "iron_to_joint_mm": float(msg.iron_to_joint_mm),
                "solder_to_joint_mm": float(msg.solder_to_joint_mm),
                "occluded": bool(msg.occluded),
            }
        )

    def _on_image(self, msg: Image) -> None:
        self._tick("/soldering/vision/annotated")
        now = time.monotonic()
        if now - self._last_image_emit < 0.10:
            return
        self._last_image_emit = now
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = bgr[:, :, ::-1].copy()
        except Exception as exc:  # noqa: BLE001 - diagnostics must reach UI
            self._worker.log_line.emit(
                {"level": 40, "name": "dashboard", "message": str(exc)}
            )
            return
        self._worker.image_frame.emit(rgb)

    def _on_rosout(self, msg: Log) -> None:
        self._tick("/rosout")
        self._worker.log_line.emit(
            {
                "level": int(msg.level),
                "name": str(msg.name),
                "message": str(msg.msg),
                "stamp": int(msg.stamp.sec) + int(msg.stamp.nanosec) / 1.0e9,
            }
        )

    def _motor_callback(self, motor_id: int, topic: str):
        def callback(msg: MotorTelemetry) -> None:
            self._tick(topic)
            now = time.monotonic()
            if now - self._last_motor_emit[motor_id - 1] < 0.05:
                return
            self._last_motor_emit[motor_id - 1] = now
            self._worker.motor_telemetry.emit(
                {
                    "received_monotonic": now,
                    "motor_id": int(msg.motor_id),
                    "axis_index": int(msg.axis_index),
                    "connected": bool(msg.connected),
                    "valid": bool(msg.valid),
                    "oper": bool(msg.oper),
                    "fault": bool(msg.fault),
                    "stale": bool(msg.stale),
                    "sequence": int(msg.sequence),
                    "target_position_rad": float(msg.target_position_rad),
                    "desired_position_rad": float(msg.desired_position_rad),
                    "measured_position_rad": float(msg.measured_position_rad),
                    "filtered_position_rad": float(msg.filtered_position_rad),
                    "velocity_rad_s": float(msg.estimated_velocity_rad_s),
                    "acceleration_rad_s2": float(
                        msg.estimated_acceleration_rad_s2
                    ),
                    "current_a": float(msg.current_a),
                    "disturbance_current_a": float(
                        msg.disturbance_current_a
                    ),
                    "load_state": int(msg.load_state),
                    "holding_load": bool(msg.holding_load),
                    "load_evidence_a": float(msg.load_evidence_a),
                    "release_permitted": bool(msg.release_permitted),
                    "temperature_c": float(msg.temperature_c),
                    "bus_voltage_v": float(msg.bus_voltage_v),
                    "position_error_rad": float(msg.position_error_rad),
                }
            )

        return callback

    def _publish_topic_health(self) -> None:
        now = time.monotonic()
        rows = []
        for topic, threshold in self._thresholds.items():
            try:
                publishers = len(self.get_publishers_info_by_topic(topic))
                subscribers = len(self.get_subscriptions_info_by_topic(topic))
            except Exception:  # noqa: BLE001 - graph may change concurrently
                publishers = 0
                subscribers = 0
            rate_hz, age_s = self._rates[topic].snapshot(now)
            rows.append(
                TopicHealth(
                    name=topic,
                    publishers=publishers,
                    subscribers=subscribers,
                    rate_hz=rate_hz,
                    age_s=age_s,
                    state=classify_topic(
                        publishers=publishers,
                        age_s=age_s,
                        stale_after_s=threshold,
                    ),
                )
            )
        self._worker.topic_health.emit(rows)

    def _drain_commands(self) -> None:
        try:
            command = self._command_queue.get_nowait()
        except queue.Empty:
            return
        if not self._pcm_client.service_is_ready():
            self._worker.command_result.emit(
                {
                    "accepted": False,
                    "message": "PCM command service is unavailable",
                }
            )
            return
        request = PcmCommand.Request()
        request.operation = int(command["operation"])
        request.slot_id = int(command.get("slot_id", 0))
        request.axes = [int(axis) for axis in command.get("axes", [])]
        request.repeat = int(command.get("repeat", 0))
        future = self._pcm_client.call_async(request)
        self._pending_futures.add(future)

        def finished(done) -> None:
            self._pending_futures.discard(done)
            try:
                response = done.result()
                result = {
                    "accepted": bool(response.accepted),
                    "job_id": int(response.job_id),
                    "message": str(response.message),
                }
            except Exception as exc:  # noqa: BLE001
                result = {"accepted": False, "message": str(exc)}
            self._worker.command_result.emit(result)

        future.add_done_callback(finished)


class RosWorker(QtCore.QThread):
    """Own one rclpy Context and publish Qt signals from its executor thread."""

    motor_telemetry = QtCore.pyqtSignal(object)
    topic_health = QtCore.pyqtSignal(object)
    armed = QtCore.pyqtSignal(bool)
    setup_state = QtCore.pyqtSignal(str)
    bridge_status = QtCore.pyqtSignal(object)
    motion_window = QtCore.pyqtSignal(object)
    pcm_status = QtCore.pyqtSignal(object)
    vision_status = QtCore.pyqtSignal(object)
    image_frame = QtCore.pyqtSignal(object)
    log_line = QtCore.pyqtSignal(object)
    command_result = QtCore.pyqtSignal(object)
    worker_status = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.command_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=8)
        self._stop_event = threading.Event()

    def enqueue_pcm_command(self, command: dict[str, Any]) -> bool:
        try:
            self.command_queue.put_nowait(dict(command))
        except queue.Full:
            self.command_result.emit(
                {"accepted": False, "message": "dashboard command queue full"}
            )
            return False
        return True

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        context = Context()
        executor = None
        node = None
        try:
            rclpy.init(context=context)
            executor = SingleThreadedExecutor(context=context)
            node = DashboardRosNode(self, context)
            executor.add_node(node)
            self.worker_status.emit("running")
            while not self._stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.05)
        except Exception as exc:  # noqa: BLE001
            self.worker_status.emit(f"error: {type(exc).__name__}: {exc}")
        finally:
            if node is not None and executor is not None:
                executor.remove_node(node)
                node.destroy_node()
            if executor is not None:
                executor.shutdown()
            if context.ok():
                context.shutdown()
            self.worker_status.emit("stopped")
