#!/usr/bin/env python3
"""ROS 2 front end for one persistent PCM USB Studio session."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from soldering_interfaces.srv import PcmCommand
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .pcm_session_core import (
    JobOperation,
    PcmJob,
    PcmSessionManager,
    PersistentPcmSession,
)
from .relative_teaching import RelativeTeachingConfig


STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class PcmSessionDaemon(Node):
    """Serialize ROS requests through one process-lifetime CDC client."""

    def __init__(self) -> None:
        super().__init__("pcm_session_daemon")
        port = str(self.declare_parameter("port", "/dev/ttyACM0").value)
        media_device = str(
            self.declare_parameter("media_device", "/dev/sda1").value
        )
        default_axes = tuple(
            int(axis)
            for axis in self.declare_parameter("default_axes", [7]).value
        )
        queue_capacity = int(
            self.declare_parameter("queue_capacity", 16).value
        )
        connect_on_start = bool(
            self.declare_parameter("connect_on_start", True).value
        )
        sdo_timeout_s = float(
            self.declare_parameter("sdo_timeout_s", 1.5).value
        )
        config = RelativeTeachingConfig(
            port=port,
            media_device=media_device,
            axes=default_axes,
            slot_id=2,
            sdo_timeout_s=sdo_timeout_s,
        )
        self._default_axes = default_axes
        self._manager = PcmSessionManager(
            PersistentPcmSession(config),
            queue_capacity=queue_capacity,
            connect_on_start=connect_on_start,
        )
        self._last_event_sequence = -1
        self._status_publisher = self.create_publisher(
            String, "~/status", STATUS_QOS
        )
        self._command_service = self.create_service(
            PcmCommand, "~/command", self._on_command
        )
        self._status_service = self.create_service(
            Trigger, "~/get_status", self._on_get_status
        )
        self.create_timer(0.1, self._publish_status)
        self._manager.start()
        self.get_logger().info(
            "PCM persistent-session daemon started: "
            f"port={port} media={media_device} axes={list(default_axes)}"
        )

    def _operation(self, value: int) -> JobOperation:
        mapping = {
            PcmCommand.Request.RELATIVE_SLOT: JobOperation.RELATIVE_SLOT,
            PcmCommand.Request.PLAY_SLOT: JobOperation.PLAY_SLOT,
            PcmCommand.Request.ARM: JobOperation.ARM,
            PcmCommand.Request.STOP: JobOperation.STOP,
            PcmCommand.Request.RECONNECT: JobOperation.RECONNECT,
        }
        if value not in mapping:
            raise ValueError(f"unknown PCM operation: {value}")
        return mapping[value]

    def _on_command(
        self,
        request: PcmCommand.Request,
        response: PcmCommand.Response,
    ) -> PcmCommand.Response:
        try:
            operation = self._operation(request.operation)
            axes = tuple(int(axis) for axis in request.axes) or self._default_axes
            repeat = int(request.repeat)
            if operation in (
                JobOperation.RELATIVE_SLOT,
                JobOperation.PLAY_SLOT,
            ):
                if repeat < 1:
                    raise ValueError("repeat must be a positive integer")
            else:
                # Non-motion commands do not use repeat.  Accept the ROS
                # integer field's default zero without weakening motion input
                # validation.
                repeat = 1
            job = PcmJob(
                operation=operation,
                slot_id=int(request.slot_id),
                axes=axes,
                repeat=repeat,
            )
            job_id = self._manager.submit(job)
        except (ValueError, RuntimeError) as exc:
            response.accepted = False
            response.job_id = 0
            response.message = str(exc)
            return response
        response.accepted = True
        response.job_id = job_id
        response.message = f"queued {operation.value} job {job_id}"
        return response

    def _status_payload(self) -> dict[str, object]:
        snapshot = self._manager.snapshot()
        payload = snapshot.to_dict()
        if snapshot.last_job_id:
            result = self._manager.result(snapshot.last_job_id)
            if result is not None:
                payload["last_result"] = {
                    "job_id": result.job_id,
                    "operation": result.operation,
                    "success": result.success,
                    "message": result.message,
                    "repeat_completed": result.repeat_completed,
                    "duration_s": (
                        result.finished_monotonic - result.started_monotonic
                    ),
                }
        return payload

    def _on_get_status(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success = True
        response.message = json.dumps(
            self._status_payload(), ensure_ascii=False, sort_keys=True
        )
        return response

    def _publish_status(self) -> None:
        snapshot = self._manager.snapshot()
        if snapshot.event_sequence == self._last_event_sequence:
            return
        self._last_event_sequence = snapshot.event_sequence
        msg = String()
        msg.data = json.dumps(
            self._status_payload(), ensure_ascii=False, sort_keys=True
        )
        self._status_publisher.publish(msg)
        if snapshot.state == "fault":
            self.get_logger().error(snapshot.detail)
        else:
            self.get_logger().info(
                f"state={snapshot.state} detail={snapshot.detail}"
            )

    def destroy_node(self) -> bool:
        self._manager.shutdown()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PcmSessionDaemon()
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
