"""PCM 아밍·슬롯 실행용 재사용 함수 모음.

이 파일 자체를 실행해도 모터는 움직이지 않는다. 아래 함수를 다른 Python
코드에서 import하여 명시적으로 호출할 때만 아밍/모션이 실행된다.

사용 전 셸 환경::

    source /opt/ros/humble/setup.bash
    source /home/phorce/hackathon/soldering_robot_ws/install/setup.bash
"""

from __future__ import annotations

from typing import Any

from soldering_control.relative_teaching import (
    LinuxPcmHost,
    RelativeTeachingConfig,
    arm_from_origin,
    open_pcm_client,
    play_slot,
    run_relative_slot_from_current,
    run_relative_slots_from_current,
    run_relative_teaching_cycle,
    start_live_session,
    stop_and_servo_off,
)


__all__ = [
    "LinuxPcmHost",
    "RelativeTeachingConfig",
    "arm_from_origin",
    "open_pcm_client",
    "play_slot",
    "run_relative_slot_from_current",
    "run_relative_slots_from_current",
    "run_relative_teaching_cycle",
    "start_live_session",
    "stop_and_servo_off",
    "arm",
    "run_slot",
    "arm_and_run_slot",
]


def arm(client: Any, axes: tuple[int, ...] = (7,)) -> dict[str, object]:
    """현재 저장된 사용자 원점을 기준으로 소프트웨어 아밍한다."""
    return arm_from_origin(client, axes)


def run_slot(
    client: Any,
    slot_id: int = 2,
    axes: tuple[int, ...] = (7,),
) -> dict[str, object]:
    """지정 슬롯을 한 번 실행하고 busy ON→OFF 완료를 기다린다."""
    return play_slot(client, slot_id=slot_id, axes=axes)


def arm_and_run_slot(
    client: Any,
    slot_id: int = 2,
    axes: tuple[int, ...] = (7,),
) -> dict[str, object]:
    """아밍한 뒤 지정 슬롯을 한 번 실행한다."""
    arm_result = arm(client, axes)
    slot_result = run_slot(client, slot_id, axes)
    return {"arm": arm_result, "slot": slot_result}


# 전체 상대교시 사이클 예시(의도치 않은 구동 방지를 위해 자동 실행하지 않음):
# result = run_relative_teaching_cycle(
#     RelativeTeachingConfig(slot_id=2, axes=(7,))
# )
