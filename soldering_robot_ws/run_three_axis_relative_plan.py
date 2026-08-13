#!/usr/bin/env python3
"""Run the 0x2/0x8/0xA relative-teaching resolution trial."""

from __future__ import annotations

import json
import time

from soldering_control.relative_teaching import (
    LinuxPcmHost,
    RelativeTeachingConfig,
    arm_from_origin,
    open_pcm_client,
    play_slot,
    set_current_pose_as_origin,
    start_live_session,
    stop_motion_and_hold,
)
from soldering_control.pcm_studio_client import read_status


AXES = (0, 6, 8)
ROUNDS = (
    ("10deg", ((31, 0), (32, 6), (33, 8))),
    ("1deg_1", ((34, 0), (35, 6), (36, 8))),
    ("1deg_2", ((34, 0), (35, 6), (36, 8))),
    ("1deg_3", ((34, 0), (35, 6), (36, 8))),
)


def emit(event: str, value: object | None = None) -> None:
    payload: dict[str, object] = {
        "time": time.strftime("%H:%M:%S"),
        "event": event,
    }
    if value is not None:
        payload["value"] = value
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), flush=True)


def main() -> int:
    config = RelativeTeachingConfig(
        axes=AXES,
        slot_id=31,
        motion_finish_timeout_s=20.0,
    )
    host = LinuxPcmHost(config)
    completed_slots: list[int] = []
    try:
        host.prepare_live_session()
        with open_pcm_client(config) as opened_client:
            client = opened_client
            live = start_live_session(client, on_event=emit)
            emit("plan_ready", {"live": live, "status": read_status(client)})
            try:
                for round_name, steps in ROUNDS:
                    emit(
                        "round_begin",
                        {"round": round_name, "axes": list(AXES)},
                    )
                    origin = set_current_pose_as_origin(client, AXES)
                    emit(
                        "round_origin_saved",
                        {"round": round_name, "origin": origin},
                    )
                    arm = arm_from_origin(client, AXES)
                    emit("round_armed", {"round": round_name, "arm": arm})

                    for slot_id, axis in steps:
                        result = play_slot(
                            client,
                            slot_id,
                            (axis,),
                            label=f"{round_name}_slot{slot_id}",
                            start_timeout_s=config.motion_start_timeout_s,
                            finish_timeout_s=config.motion_finish_timeout_s,
                            settle_s=config.motion_settle_s,
                            on_event=emit,
                        )
                        completed_slots.append(slot_id)
                        emit(
                            "slot_verified",
                            {
                                "round": round_name,
                                "slot": slot_id,
                                "axis": axis,
                                "result": result,
                            },
                        )

                    cleanup = stop_motion_and_hold(client, on_event=emit)
                    emit(
                        "round_complete",
                        {
                            "round": round_name,
                            "cleanup": cleanup,
                            "holding": True,
                        },
                    )
                    if round_name != ROUNDS[-1][0]:
                        emit(
                            "plan_paused_holding",
                            {
                                "reason": "next persistent rebase requires "
                                "servo-off, but load release is not confirmed",
                                "completed_slots": completed_slots,
                            },
                        )
                        return 2

                emit(
                    "plan_success",
                    {
                        "completed_slots": completed_slots,
                        "final_status": read_status(client),
                    },
                )
            except Exception as exc:  # fail closed while CDC is still open
                cleanup = stop_motion_and_hold(client, on_event=emit)
                emit(
                    "plan_failed",
                    {
                        "type": type(exc).__name__,
                        "error": str(exc),
                        "completed_slots": completed_slots,
                        "cleanup": cleanup,
                    },
                )
                return 1
        return 0
    except Exception as exc:  # noqa: BLE001 - fail closed with structured log
        emit(
            "plan_failed",
            {
                "type": type(exc).__name__,
                "error": str(exc),
                "completed_slots": completed_slots,
                "cleanup": None,
            },
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
