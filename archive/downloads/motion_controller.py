"""
앉기 / 일어나기 / 눕기 / 줌 상태를 하나의 worker thread에서 제어하는
phorce 모션 제어기.

교시 번호
---------
1 : OFF 기본 위치 -> 앉은 정면
2 : 앉은 정면(0) -> 앉은 왼쪽1(-1)
3 : 앉은 왼쪽1(-1) -> 앉은 왼쪽2(-2)
4 : 앉은 왼쪽2(-2) -> 앉은 왼쪽1(-1)
5 : 앉은 왼쪽1(-1) -> 앉은 정면(0)
6 : 앉은 정면(0) -> 앉은 오른쪽1(+1)
7 : 앉은 오른쪽1(+1) -> 앉은 정면(0)
8 : 앉은 정면 -> OFF 기본 위치
9 : 앉은 정면 -> 일어난 정면
10: 일어난 정면 -> 앉은 정면
11: 앉은 정면 -> 누운 정면
12: 누운 정면(0) -> 누운 왼쪽1(-1)
13: 누운 왼쪽1(-1) -> 누운 정면(0)
14: 누운 정면(0) -> 누운 오른쪽1(+1)
15: 누운 오른쪽1(+1) -> 누운 정면(0)
16: 누운 정면 -> 누운 정면 줌인
17: 누운 정면 줌인 -> 누운 정면
18: 누운 정면 -> 앉은 정면
19: 초기 위치 -> 환영 춤 위치
20: 환영 춤 (끝점도 춤 위치, 시작 시 2회 연속 실행)
21: 환영 춤 위치 -> 초기 위치

핵심 원칙
---------
- 프로그램 시작 직후 19 -> 20 -> 20 -> 21 환영 시퀀스를 한 번 실행한다.
- 환영 시퀀스가 끝나면 WAIT_FOR_H에서 정지하고, H 요청을 받아야 motion 1부터 시작한다.
- robot.play()는 이 클래스의 worker 한 곳에서만 호출한다.
- robot.play()는 실제 모션 완료까지 blocking이다.
- 얼굴 yaw 추적은 한 번에 인접 모션 하나만 실행한다.
- 그 모션 완료 후에는 반드시 새 얼굴 측정이 들어와야 다음 yaw 모션을 허용한다.
- sitting yaw 범위는 -2 ~ +1, lying yaw 범위는 -1 ~ +1이다.
- standing에서는 yaw 추적을 하지 않는다.
- lying zoom 중에도 yaw 추적을 하지 않는다.
- L 전환과 Z 줌은 manual transition lock을 사용해 서로 겹치지 않는다.
- Q 종료는 모든 일반/수동 전환보다 우선한다. 진행 중 play는 중단하지 않고,
  그 play가 실제 끝난 뒤 현재 실제 논리 상태에서 안전 복귀 경로를 다시 계산한다.
- Q 종료 경로는 항상 해당 자세 중심 -> 앉은 중심 -> motion 8 순서다.
- BUSY만 자동 재시도가 가능하다. Rejected/Aborted/Unavailable은 ERROR로 멈춘다.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple


YawTransition = Tuple[int, int]

SITTING_YAW_MIN_STEP = -2
SITTING_YAW_MAX_STEP = 1
LYING_YAW_MIN_STEP = -1
LYING_YAW_MAX_STEP = 1

POSTURE_SITTING = "sitting"
POSTURE_STANDING = "standing"
POSTURE_LYING = "lying"
VALID_POSTURES = frozenset((POSTURE_SITTING, POSTURE_STANDING, POSTURE_LYING))

STARTUP_MOTION_ID = 1
EXIT_MOTION_ID = 8
WELCOME_TO_DANCE_MOTION_ID = 19
WELCOME_DANCE_MOTION_ID = 20
WELCOME_TO_HOME_MOTION_ID = 21
SIT_TO_STAND_MOTION_ID = 9
STAND_TO_SIT_MOTION_ID = 10
SIT_TO_LIE_MOTION_ID = 11
LIE_ZOOM_IN_MOTION_ID = 16
LIE_ZOOM_OUT_MOTION_ID = 17
LIE_TO_SIT_MOTION_ID = 18

REQUIRED_MOTION_IDS = frozenset(range(1, 22))

SITTING_YAW_MOTIONS: Dict[YawTransition, int] = {
    (0, -1): 2,
    (-1, -2): 3,
    (-2, -1): 4,
    (-1, 0): 5,
    (0, 1): 6,
    (1, 0): 7,
}

LYING_YAW_MOTIONS: Dict[YawTransition, int] = {
    (0, -1): 12,
    (-1, 0): 13,
    (0, 1): 14,
    (1, 0): 15,
}


@dataclass(frozen=True)
class MotionSnapshot:
    phase: str
    current_posture: str
    target_posture: str
    current_yaw: int
    target_yaw: int
    lying_zoom: bool
    manual_action: Optional[str]
    busy: bool
    settling: bool
    tracking_enabled: bool
    shutdown_requested: bool
    shutdown_done: bool
    last_motion_id: Optional[int]
    last_error: Optional[str]


class MotionController:
    """sitting/standing/lying/zoom 상태를 단일 worker로 제어한다."""

    def __init__(
        self,
        *,
        yaw_motions: Optional[Mapping[YawTransition, int]] = None,
        lying_yaw_motions: Optional[Mapping[YawTransition, int]] = None,
        startup_motion_id: int = STARTUP_MOTION_ID,
        welcome_to_dance_motion_id: int = WELCOME_TO_DANCE_MOTION_ID,
        welcome_dance_motion_id: int = WELCOME_DANCE_MOTION_ID,
        welcome_to_home_motion_id: int = WELCOME_TO_HOME_MOTION_ID,
        exit_motion_id: int = EXIT_MOTION_ID,
        sit_to_stand_motion_id: int = SIT_TO_STAND_MOTION_ID,
        stand_to_sit_motion_id: int = STAND_TO_SIT_MOTION_ID,
        sit_to_lie_motion_id: int = SIT_TO_LIE_MOTION_ID,
        lie_zoom_in_motion_id: int = LIE_ZOOM_IN_MOTION_ID,
        lie_zoom_out_motion_id: int = LIE_ZOOM_OUT_MOTION_ID,
        lie_to_sit_motion_id: int = LIE_TO_SIT_MOTION_ID,
        dry_run: bool = True,
        settle_time: float = 0.4,
        startup_settle_time: float = 0.8,
        exit_settle_time: float = 0.8,
        dry_run_motion_time: float = 4.0,
        busy_retry_delay: float = 0.25,
        verify_motion_catalog: bool = True,
    ) -> None:
        self._sitting_yaw_motions = dict(
            SITTING_YAW_MOTIONS if yaw_motions is None else yaw_motions
        )
        self._lying_yaw_motions = dict(
            LYING_YAW_MOTIONS
            if lying_yaw_motions is None
            else lying_yaw_motions
        )

        self._startup_motion_id = int(startup_motion_id)
        self._welcome_sequence = (
            int(welcome_to_dance_motion_id),
            int(welcome_dance_motion_id),
            int(welcome_to_home_motion_id),
        )
        self._welcome_index = 0
        self._welcome_abort_requested = False
        self._exit_motion_id = int(exit_motion_id)
        self._sit_to_stand_motion_id = int(sit_to_stand_motion_id)
        self._stand_to_sit_motion_id = int(stand_to_sit_motion_id)
        self._sit_to_lie_motion_id = int(sit_to_lie_motion_id)
        self._lie_zoom_in_motion_id = int(lie_zoom_in_motion_id)
        self._lie_zoom_out_motion_id = int(lie_zoom_out_motion_id)
        self._lie_to_sit_motion_id = int(lie_to_sit_motion_id)

        self._dry_run = bool(dry_run)
        self._settle_time = max(0.0, float(settle_time))
        self._startup_settle_time = max(0.0, float(startup_settle_time))
        self._exit_settle_time = max(0.0, float(exit_settle_time))
        self._dry_run_motion_time = max(0.0, float(dry_run_motion_time))
        self._busy_retry_delay = max(0.05, float(busy_retry_delay))
        self._verify_motion_catalog = bool(verify_motion_catalog)

        self._condition = threading.Condition()
        self._stop_requested = False

        # CONNECTING -> WELCOME -> WAIT_FOR_H -> STARTUP -> WAIT_FOR_ZERO
        # -> TRACKING -> MANUAL_TRANSITION -> TRACKING
        # -> SHUTDOWN_RETURN -> SHUTDOWN_EXIT -> DONE
        self._phase = "CONNECTING" if not self._dry_run else "WELCOME"

        # motion 1의 끝점은 항상 sitting center다.
        self._current_posture = POSTURE_SITTING
        self._target_posture = POSTURE_SITTING
        self._current_yaw = 0
        self._target_yaw = 0
        self._lying_zoom = False

        # None | to_lying | to_sitting | zoom_in | zoom_out
        self._manual_action: Optional[str] = None

        self._busy = False
        self._settling_until = 0.0
        self._tracking_enabled = False
        self._shutdown_requested = False
        self._shutdown_done = False
        self._last_motion_id: Optional[int] = None
        self._last_error: Optional[str] = None

        # 같은 얼굴 측정 하나로 연속 yaw motion이 나가는 것을 막는다.
        self._measurement_version = 0
        self._minimum_version_for_next_tracking_motion = 0

        # motion 1은 환영 시퀀스가 끝난 뒤 H를 눌렀을 때만 예약한다.
        self._startup_pending = False
        self._exit_pending = False

        self._phorce = None

        self._thread = threading.Thread(
            target=self._worker,
            name="sitting-standing-lying-motion-controller",
            daemon=True,
        )
        self._thread.start()

    @staticmethod
    def _clamp_sitting_yaw(value: int) -> int:
        return max(
            SITTING_YAW_MIN_STEP,
            min(SITTING_YAW_MAX_STEP, int(value)),
        )

    @staticmethod
    def _clamp_lying_yaw(value: int) -> int:
        return max(
            LYING_YAW_MIN_STEP,
            min(LYING_YAW_MAX_STEP, int(value)),
        )

    def _clamp_yaw_for_current_posture(self, value: int) -> int:
        if self._current_posture == POSTURE_SITTING:
            return self._clamp_sitting_yaw(value)
        if self._current_posture == POSTURE_LYING:
            return self._clamp_lying_yaw(value)
        return 0

    def _tracking_command_allowed_locked(self) -> bool:
        now = time.monotonic()
        return (
            self._phase == "TRACKING"
            and self._tracking_enabled
            and not self._shutdown_requested
            and self._manual_action is None
            and not self._busy
            and now >= self._settling_until
        )

    def request_start(self) -> bool:
        """
        환영 시퀀스 19 -> 20 -> 20 -> 21 완료 후 H 키 요청.

        WAIT_FOR_H 상태에서만 motion 1을 예약한다. 환영 모션 진행 중에
        미리 누른 H는 기억하지 않으며, motion 21 완료 뒤 다시 눌러야 한다.
        """
        with self._condition:
            now = time.monotonic()
            if self._shutdown_requested or self._phase == "ERROR":
                return False
            if self._phase != "WAIT_FOR_H":
                return False
            if self._busy or now < self._settling_until:
                return False

            self._startup_pending = True
            self._phase = "STARTUP"
            self._condition.notify_all()
            print("[MOTION] H accepted | motion 1 startup requested")
            return True

    def enable_tracking_after_zero(self) -> bool:
        """3손가락 얼굴 영점 보정 완료 후 최초 TRACKING을 시작한다."""
        with self._condition:
            now = time.monotonic()
            if self._shutdown_requested or self._phase == "ERROR":
                return False
            if self._phase != "WAIT_FOR_ZERO":
                return False
            if self._busy or now < self._settling_until:
                return False

            self._tracking_enabled = True
            self._phase = "TRACKING"
            self._current_posture = POSTURE_SITTING
            self._target_posture = POSTURE_SITTING
            self._current_yaw = 0
            self._target_yaw = 0
            self._lying_zoom = False
            self._manual_action = None

            self._measurement_version += 1
            self._minimum_version_for_next_tracking_motion = (
                self._measurement_version
            )

            self._condition.notify_all()
            print(
                "[MOTION] face zero accepted | TRACKING enabled | "
                "posture=sitting yaw=0"
            )
            return True

    def update_yaw_error(self, yaw_error: int) -> None:
        """
        현재 움직이는 카메라 기준 잔여 yaw 단계 오차를 전달한다.

        sitting에서는 -2~+1, lying에서는 -1~+1로 제한한다.
        실제 worker는 언제나 인접 한 단계 모션 하나만 실행한다.
        standing/lying zoom에서는 무시한다.
        """
        with self._condition:
            if not self._tracking_command_allowed_locked():
                return
            if self._target_posture != self._current_posture:
                return
            if self._current_posture not in (POSTURE_SITTING, POSTURE_LYING):
                return
            if self._current_posture == POSTURE_LYING and self._lying_zoom:
                return

            self._target_yaw = self._clamp_yaw_for_current_posture(
                self._current_yaw + int(yaw_error)
            )
            self._measurement_version += 1
            self._condition.notify_all()

    def request_posture(self, posture: str) -> bool:
        """
        얼굴 높이 판정용 sitting <-> standing 자동 전환 요청.

        standing 요청은 오직 sitting center에서만 허용한다.
        standing에서 sitting 요청은 motion 10으로 연결한다.
        lying은 L 키 전용이므로 여기서는 받지 않는다.
        """
        requested = str(posture).strip().lower()
        if requested not in (POSTURE_SITTING, POSTURE_STANDING):
            raise ValueError(f"지원하지 않는 자동 posture: {posture!r}")

        with self._condition:
            if not self._tracking_command_allowed_locked():
                return False
            if self._target_posture != self._current_posture:
                return False
            if requested == self._current_posture:
                return False

            if requested == POSTURE_STANDING:
                if self._current_posture != POSTURE_SITTING:
                    return False
                if self._current_yaw != 0:
                    return False
                self._target_yaw = 0

            elif requested == POSTURE_SITTING:
                if self._current_posture != POSTURE_STANDING:
                    return False

            self._minimum_version_for_next_tracking_motion = (
                self._measurement_version
            )
            self._target_posture = requested
            self._condition.notify_all()
            print(
                f"[MOTION] posture request | "
                f"{self._current_posture} -> {requested}"
            )
            return True

    def request_lying_toggle(self) -> bool:
        """
        L 키 요청.

        sitting 어느 yaw 상태 -> sitting center -> motion 11 -> lying center
        lying 어느 상태(좌/우/zoom 포함) -> lying center -> motion 18 -> sitting center

        standing에서는 무시한다. L/Z 전환 중에는 새 L/Z를 받지 않는다.
        """
        with self._condition:
            if not self._tracking_command_allowed_locked():
                return False
            if self._target_posture != self._current_posture:
                return False

            if self._current_posture == POSTURE_SITTING:
                action = "to_lying"
            elif self._current_posture == POSTURE_LYING:
                action = "to_sitting"
            else:
                return False

            # 이미 받은 얼굴 yaw 목표는 폐기한다.
            self._target_yaw = self._current_yaw
            self._minimum_version_for_next_tracking_motion = (
                self._measurement_version
            )

            self._manual_action = action
            self._tracking_enabled = False
            self._phase = "MANUAL_TRANSITION"
            self._condition.notify_all()
            print(f"[MOTION] L accepted | manual_action={action}")
            return True

    def request_zoom_toggle(self) -> bool:
        """
        Z 키 요청.

        lying center -> motion 16 -> zoom in
        lying zoom   -> motion 17 -> zoom out

        lying left/right, sitting, standing에서는 무시한다.
        """
        with self._condition:
            if not self._tracking_command_allowed_locked():
                return False
            if self._target_posture != self._current_posture:
                return False
            if self._current_posture != POSTURE_LYING:
                return False
            if self._current_yaw != 0:
                return False

            self._target_yaw = 0
            self._minimum_version_for_next_tracking_motion = (
                self._measurement_version
            )

            self._manual_action = (
                "zoom_out" if self._lying_zoom else "zoom_in"
            )
            self._tracking_enabled = False
            self._phase = "MANUAL_TRANSITION"
            self._condition.notify_all()
            print(
                f"[MOTION] Z accepted | manual_action={self._manual_action}"
            )
            return True

    def request_shutdown(self) -> bool:
        """
        Q 종료 요청.

        현재 play()는 중단하지 않는다.

        - WELCOME 중 Q: 현재 play가 끝난 뒤 motion 21로 초기 위치에 복귀하고 종료한다.
        - WAIT_FOR_H 중 Q: 이미 motion 21 끝점(초기 위치)이므로 새 motion 없이 종료한다.
        - H 이후 Q: 기존과 같이 실제 논리 상태에서 center -> motion 8 안전 복귀한다.
        """
        with self._condition:
            if self._shutdown_requested or self._shutdown_done:
                return False
            if self._phase == "ERROR":
                return False

            self._shutdown_requested = True
            self._tracking_enabled = False
            self._manual_action = None

            # 환영 시퀀스가 끝난 뒤 H를 기다리는 동안에는 이미 초기 위치다.
            if self._phase == "WAIT_FOR_H":
                self._shutdown_done = True
                self._phase = "DONE"
                self._condition.notify_all()
                print("[MOTION] shutdown in WAIT_FOR_H | already at initial pose")
                return True

            # 아직 환영 motion 19도 시작하지 않았다면 초기 위치 그대로이므로
            # motion 21을 잘못 보내지 않고 즉시 종료한다.
            if self._phase == "CONNECTING" or (
                self._phase == "WELCOME"
                and self._welcome_index == 0
                and not self._busy
            ):
                self._shutdown_done = True
                self._phase = "DONE"
                self._condition.notify_all()
                print(
                    "[MOTION] shutdown before welcome motion 19 | "
                    "already at initial pose"
                )
                return True

            # 환영 모션이 실제 진행 중/진행된 뒤에는 현재 play를 끊지 않고,
            # 완료 뒤 motion 21로 초기 위치에 복귀한다.
            if self._phase == "WELCOME":
                self._welcome_abort_requested = True
                self._condition.notify_all()
                print(
                    "[MOTION] shutdown during welcome | "
                    "finish current play then return with motion 21"
                )
                return True

            self._target_posture = self._current_posture
            self._target_yaw = self._current_yaw
            self._phase = "SHUTDOWN_RETURN"
            self._condition.notify_all()
            print("[MOTION] shutdown requested | all normal controls locked")
            return True

    def snapshot(self) -> MotionSnapshot:
        with self._condition:
            return MotionSnapshot(
                phase=self._phase,
                current_posture=self._current_posture,
                target_posture=self._target_posture,
                current_yaw=self._current_yaw,
                target_yaw=self._target_yaw,
                lying_zoom=self._lying_zoom,
                manual_action=self._manual_action,
                busy=self._busy,
                settling=time.monotonic() < self._settling_until,
                tracking_enabled=self._tracking_enabled,
                shutdown_requested=self._shutdown_requested,
                shutdown_done=self._shutdown_done,
                last_motion_id=self._last_motion_id,
                last_error=self._last_error,
            )

    def stop(self, timeout: float = 10.0) -> None:
        with self._condition:
            self._stop_requested = True
            self._condition.notify_all()
        self._thread.join(timeout=max(0.0, float(timeout)))

    def _set_fatal_error(self, message: str) -> None:
        with self._condition:
            self._last_error = message
            self._tracking_enabled = False
            self._manual_action = None
            self._phase = "ERROR"
            self._busy = False
            self._condition.notify_all()
        print(f"[MOTION][ERROR] {message}")

    def _check_required_motions(self, robot) -> bool:
        if not self._verify_motion_catalog:
            return True

        try:
            loaded_ids = {int(m.id) for m in robot.motions()}
        except Exception as exc:
            self._set_fatal_error(
                f"motion catalog 확인 실패: {type(exc).__name__}: {exc}"
            )
            return False

        missing = sorted(REQUIRED_MOTION_IDS - loaded_ids)
        if missing:
            self._set_fatal_error(
                "pcm에 필요한 motion ID가 없음: "
                + ", ".join(str(x) for x in missing)
            )
            return False

        print("[MOTION] catalog OK | motion 1~21 loaded")
        return True

    def _yaw_map_for_current_posture(self):
        if self._current_posture == POSTURE_SITTING:
            return self._sitting_yaw_motions
        if self._current_posture == POSTURE_LYING:
            return self._lying_yaw_motions
        return None

    def _next_tracking_yaw_motion(self):
        if self._current_posture not in (POSTURE_SITTING, POSTURE_LYING):
            return None
        if self._target_posture != self._current_posture:
            return None
        if self._current_posture == POSTURE_LYING and self._lying_zoom:
            return None
        if self._current_yaw == self._target_yaw:
            return None

        end = self._current_yaw + (
            1 if self._target_yaw > self._current_yaw else -1
        )
        yaw_map = self._yaw_map_for_current_posture()
        motion_id = yaw_map.get((self._current_yaw, end))

        if motion_id is None:
            self._last_error = (
                f"{self._current_posture} yaw 모션 미등록: "
                f"{self._current_yaw:+d}->{end:+d}"
            )
            return None

        return self._current_yaw, end, int(motion_id)

    def _next_center_yaw_motion(self, *, shutdown: bool):
        """현재 sitting/lying yaw를 해당 자세 center(0)로 한 단계 복귀한다."""
        if self._current_yaw == 0:
            return None

        yaw_map = self._yaw_map_for_current_posture()
        if yaw_map is None:
            return None

        end = self._current_yaw + (-1 if self._current_yaw > 0 else 1)
        motion_id = yaw_map.get((self._current_yaw, end))
        if motion_id is None:
            self._last_error = (
                f"center 복귀 모션 미등록: {self._current_posture} "
                f"{self._current_yaw:+d}->{end:+d}"
            )
            return None

        kind = "shutdown_center_yaw" if shutdown else "manual_center_yaw"
        prefix = "shutdown" if shutdown else "manual"
        return (
            kind,
            self._current_yaw,
            end,
            int(motion_id),
            f"{prefix} {self._current_posture} yaw "
            f"{self._current_yaw:+d}->{end:+d}",
        )

    def _next_manual_motion(self):
        action = self._manual_action

        if action == "to_lying":
            if self._current_posture != POSTURE_SITTING:
                self._last_error = (
                    f"L to_lying 상태 불일치: posture={self._current_posture}"
                )
                return None

            center_motion = self._next_center_yaw_motion(shutdown=False)
            if center_motion is not None:
                return center_motion

            return (
                "manual_sit_to_lie",
                0,
                0,
                self._sit_to_lie_motion_id,
                "manual sitting center -> lying center",
            )

        if action == "to_sitting":
            if self._current_posture != POSTURE_LYING:
                self._last_error = (
                    f"L to_sitting 상태 불일치: posture={self._current_posture}"
                )
                return None

            if self._lying_zoom:
                return (
                    "manual_zoom_out_for_sit",
                    0,
                    0,
                    self._lie_zoom_out_motion_id,
                    "manual lying zoom -> lying center",
                )

            center_motion = self._next_center_yaw_motion(shutdown=False)
            if center_motion is not None:
                return center_motion

            return (
                "manual_lie_to_sit",
                0,
                0,
                self._lie_to_sit_motion_id,
                "manual lying center -> sitting center",
            )

        if action == "zoom_in":
            if (
                self._current_posture != POSTURE_LYING
                or self._current_yaw != 0
                or self._lying_zoom
            ):
                self._last_error = "Z zoom_in 상태 불일치"
                return None
            return (
                "manual_zoom_in",
                0,
                0,
                self._lie_zoom_in_motion_id,
                "manual lying center -> zoom in",
            )

        if action == "zoom_out":
            if (
                self._current_posture != POSTURE_LYING
                or self._current_yaw != 0
                or not self._lying_zoom
            ):
                self._last_error = "Z zoom_out 상태 불일치"
                return None
            return (
                "manual_zoom_out",
                0,
                0,
                self._lie_zoom_out_motion_id,
                "manual lying zoom -> lying center",
            )

        self._last_error = f"알 수 없는 manual_action: {action!r}"
        return None

    def _next_shutdown_motion(self):
        # standing -> sitting center
        if self._current_posture == POSTURE_STANDING:
            return (
                "shutdown_stand_to_sit",
                0,
                0,
                self._stand_to_sit_motion_id,
                "shutdown standing center -> sitting center",
            )

        # lying zoom -> lying center
        if self._current_posture == POSTURE_LYING and self._lying_zoom:
            return (
                "shutdown_zoom_out",
                0,
                0,
                self._lie_zoom_out_motion_id,
                "shutdown lying zoom -> lying center",
            )

        # sitting/lying 모두 먼저 해당 자세의 yaw center로 복귀한다.
        if self._current_posture in (POSTURE_SITTING, POSTURE_LYING):
            center_motion = self._next_center_yaw_motion(shutdown=True)
            if center_motion is not None:
                return center_motion

        # lying center -> sitting center
        if self._current_posture == POSTURE_LYING:
            return (
                "shutdown_lie_to_sit",
                0,
                0,
                self._lie_to_sit_motion_id,
                "shutdown lying center -> sitting center",
            )

        # sitting center이면 이제 motion 8을 실행하면 된다.
        return None

    def _finish_manual_action_locked(self) -> None:
        if self._shutdown_requested:
            # Q가 play 중 들어온 경우 shutdown phase를 유지한다.
            self._manual_action = None
            self._tracking_enabled = False
            self._phase = "SHUTDOWN_RETURN"
            return

        self._manual_action = None
        self._tracking_enabled = True
        self._phase = "TRACKING"
        self._measurement_version += 1
        self._minimum_version_for_next_tracking_motion = (
            self._measurement_version
        )

    def _worker(self) -> None:
        if self._dry_run:
            self._run_control_loop(robot=None)
            return

        try:
            # 장비 없는 PC에서도 import/문법 검사가 가능하도록 실제 모드에서만 import.
            import phorce

            self._phorce = phorce
            print("[MOTION] connecting to phorce target=robot ...")

            with phorce.connect() as robot:
                print("[MOTION] phorce connected")

                if not self._check_required_motions(robot):
                    return

                with self._condition:
                    if self._phase == "CONNECTING":
                        self._phase = "WELCOME"
                    self._condition.notify_all()

                self._run_control_loop(robot)

        except Exception as exc:
            self._set_fatal_error(
                f"phorce 연결/worker 예외: {type(exc).__name__}: {exc}"
            )

    def _run_control_loop(self, robot) -> None:
        while True:
            with self._condition:
                selected = None

                while not self._stop_requested:
                    now = time.monotonic()

                    if self._phase == "ERROR":
                        self._condition.wait(timeout=0.1)
                        continue

                    if self._busy:
                        self._condition.wait(timeout=0.05)
                        continue

                    if now < self._settling_until:
                        self._condition.wait(
                            timeout=min(0.05, self._settling_until - now)
                        )
                        continue

                    # ----------------------------------------------------
                    # 프로그램 최초 1회 환영 시퀀스: 19 -> 20 -> 20 -> 21
                    # ----------------------------------------------------
                    if self._phase == "WELCOME":
                        # Q가 환영 모션 도중 들어오면 현재 play 완료 뒤에는
                        # 남은 춤을 생략하고 motion 21로 초기 위치에 복귀한다.
                        if self._welcome_abort_requested:
                            # 이미 motion 21까지 성공한 상태라면 새 motion 없이 종료.
                            if self._welcome_index >= len(self._welcome_sequence):
                                self._shutdown_done = True
                                self._phase = "DONE"
                                self._condition.notify_all()
                                continue

                            selected = (
                                "welcome_abort_return",
                                0,
                                0,
                                self._welcome_sequence[-1],
                                "welcome abort -> initial pose",
                            )
                            break

                        if self._welcome_index < len(self._welcome_sequence):
                            motion_id = self._welcome_sequence[self._welcome_index]
                            occurrence = self._welcome_index + 1
                            selected = (
                                "welcome",
                                self._welcome_index,
                                occurrence,
                                motion_id,
                                (
                                    f"welcome {occurrence}/3 | "
                                    f"motion {motion_id}"
                                ),
                            )
                            break

                        self._phase = "WAIT_FOR_H"
                        self._condition.notify_all()
                        print(
                            "[MOTION] welcome complete | "
                            "WAIT_FOR_H | press H to start motion 1"
                        )
                        continue

                    # H가 눌린 뒤에만 motion 1을 실행한다.
                    # 시작 모션 1이 시작된 뒤 Q가 들어오면, motion 1을 끝낸 뒤에야
                    # motion 8의 시작점(sitting center)이 보장된다.
                    if self._startup_pending:
                        self._startup_pending = False
                        selected = (
                            "startup",
                            0,
                            0,
                            self._startup_motion_id,
                            "OFF -> sitting center",
                        )
                        break

                    if self._phase == "STARTUP" and not self._shutdown_requested:
                        self._phase = "WAIT_FOR_ZERO"
                        self._condition.notify_all()

                    # Q가 최우선이다.
                    if self._shutdown_requested:
                        if self._phase == "DONE":
                            self._condition.wait(timeout=0.1)
                            continue

                        shutdown_motion = self._next_shutdown_motion()
                        if shutdown_motion is not None:
                            selected = shutdown_motion
                            break

                        if not self._exit_pending and not self._shutdown_done:
                            self._exit_pending = True
                            self._phase = "SHUTDOWN_EXIT"
                            selected = (
                                "exit",
                                0,
                                0,
                                self._exit_motion_id,
                                "sitting center -> OFF pose",
                            )
                            break

                    # L/Z 수동 전환. 추적보다 우선하며 이 동안 추적은 잠겨 있다.
                    if self._manual_action is not None:
                        if self._phase != "MANUAL_TRANSITION":
                            self._phase = "MANUAL_TRANSITION"
                        manual_motion = self._next_manual_motion()
                        if manual_motion is None:
                            self._phase = "ERROR"
                            self._tracking_enabled = False
                            self._manual_action = None
                            self._condition.notify_all()
                            continue
                        selected = manual_motion
                        break

                    if self._phase != "TRACKING":
                        self._condition.wait(timeout=0.1)
                        continue

                    # 자동 sitting <-> standing 전환을 yaw 추적보다 먼저 처리한다.
                    if self._current_posture != self._target_posture:
                        if (
                            self._current_posture == POSTURE_SITTING
                            and self._target_posture == POSTURE_STANDING
                            and self._current_yaw == 0
                        ):
                            selected = (
                                "sit_to_stand",
                                0,
                                0,
                                self._sit_to_stand_motion_id,
                                "sitting center -> standing center",
                            )
                            break

                        if (
                            self._current_posture == POSTURE_STANDING
                            and self._target_posture == POSTURE_SITTING
                        ):
                            selected = (
                                "stand_to_sit",
                                0,
                                0,
                                self._stand_to_sit_motion_id,
                                "standing center -> sitting center",
                            )
                            break

                        self._last_error = (
                            "지원하지 않는 자동 posture 전환: "
                            f"{self._current_posture}->{self._target_posture}"
                        )
                        self._phase = "ERROR"
                        self._tracking_enabled = False
                        self._condition.notify_all()
                        continue

                    # standing 및 lying zoom에서는 yaw 추적하지 않는다.
                    if self._current_posture == POSTURE_STANDING:
                        self._target_yaw = 0
                        self._condition.wait(timeout=0.1)
                        continue

                    if self._current_posture == POSTURE_LYING and self._lying_zoom:
                        self._target_yaw = 0
                        self._condition.wait(timeout=0.1)
                        continue

                    # 한 yaw motion 완료 뒤 반드시 그 이후 새 얼굴 측정이 필요하다.
                    if (
                        self._measurement_version
                        <= self._minimum_version_for_next_tracking_motion
                    ):
                        self._condition.wait(timeout=0.1)
                        continue

                    hop = self._next_tracking_yaw_motion()
                    if hop is None:
                        self._minimum_version_for_next_tracking_motion = (
                            self._measurement_version
                        )
                        self._condition.wait(timeout=0.05)
                        continue

                    start, end, motion_id = hop
                    selected = (
                        "tracking_yaw",
                        start,
                        end,
                        motion_id,
                        f"{self._current_posture} yaw {start:+d}->{end:+d}",
                    )
                    break

                if self._stop_requested:
                    break

                kind, start, end, motion_id, label = selected
                used_version = self._measurement_version
                self._busy = True
                self._last_motion_id = motion_id
                self._last_error = None

            # 오직 이 worker만 play를 호출한다.
            play_status = self._play_motion(robot, motion_id, label)

            with self._condition:
                self._busy = False

                if play_status == "success":
                    settle = self._settle_time

                    if kind == "welcome":
                        # 19 -> 20 -> 20 -> 21은 각 play의 실제 완료 뒤
                        # 곧바로 다음 play를 호출한다. 별도 settle pause는 두지 않는다.
                        self._welcome_index += 1
                        settle = 0.0

                        # Q가 현재 welcome play 중 들어온 경우:
                        # motion 21을 방금 끝냈다면 즉시 종료하고, 아니면 다음 루프에서
                        # welcome_abort_return(motion 21)을 선택한다.
                        if self._welcome_abort_requested:
                            if motion_id == self._welcome_sequence[-1]:
                                self._welcome_index = len(self._welcome_sequence)
                                self._shutdown_done = True
                                self._phase = "DONE"
                        elif self._welcome_index >= len(self._welcome_sequence):
                            self._phase = "WAIT_FOR_H"
                            print(
                                "[MOTION] welcome complete | "
                                "WAIT_FOR_H | press H to start motion 1"
                            )

                    elif kind == "welcome_abort_return":
                        # motion 21 끝점은 초기 위치. motion 8을 보내지 않고 종료한다.
                        self._welcome_index = len(self._welcome_sequence)
                        self._shutdown_done = True
                        self._phase = "DONE"
                        settle = 0.0

                    elif kind == "startup":
                        self._current_posture = POSTURE_SITTING
                        self._target_posture = POSTURE_SITTING
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._lying_zoom = False
                        self._manual_action = None
                        if self._shutdown_requested:
                            self._phase = "SHUTDOWN_RETURN"
                            self._tracking_enabled = False
                        else:
                            self._phase = "STARTUP"
                        settle = self._startup_settle_time

                    elif kind == "tracking_yaw":
                        self._current_yaw = end
                        self._target_yaw = self._current_yaw
                        self._minimum_version_for_next_tracking_motion = (
                            used_version
                        )

                    elif kind == "sit_to_stand":
                        self._current_posture = POSTURE_STANDING
                        self._target_posture = POSTURE_STANDING
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._lying_zoom = False
                        self._minimum_version_for_next_tracking_motion = (
                            self._measurement_version
                        )
                        if self._shutdown_requested:
                            self._phase = "SHUTDOWN_RETURN"
                            self._tracking_enabled = False

                    elif kind == "stand_to_sit":
                        self._current_posture = POSTURE_SITTING
                        self._target_posture = POSTURE_SITTING
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._lying_zoom = False
                        self._minimum_version_for_next_tracking_motion = (
                            self._measurement_version
                        )
                        if self._shutdown_requested:
                            self._phase = "SHUTDOWN_RETURN"
                            self._tracking_enabled = False

                    elif kind == "manual_center_yaw":
                        self._current_yaw = end
                        self._target_yaw = self._current_yaw
                        # manual_action은 유지되어 다음 center/11/18을 이어 간다.

                    elif kind == "manual_sit_to_lie":
                        self._current_posture = POSTURE_LYING
                        self._target_posture = POSTURE_LYING
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._lying_zoom = False
                        self._finish_manual_action_locked()

                    elif kind == "manual_zoom_out_for_sit":
                        self._lying_zoom = False
                        self._current_yaw = 0
                        self._target_yaw = 0
                        # to_sitting action은 유지되어 다음 motion 18로 간다.

                    elif kind == "manual_lie_to_sit":
                        self._current_posture = POSTURE_SITTING
                        self._target_posture = POSTURE_SITTING
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._lying_zoom = False
                        self._finish_manual_action_locked()

                    elif kind == "manual_zoom_in":
                        self._lying_zoom = True
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._finish_manual_action_locked()

                    elif kind == "manual_zoom_out":
                        self._lying_zoom = False
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._finish_manual_action_locked()

                    elif kind == "shutdown_center_yaw":
                        self._current_yaw = end
                        self._target_yaw = self._current_yaw
                        self._phase = "SHUTDOWN_RETURN"
                        self._tracking_enabled = False

                    elif kind == "shutdown_zoom_out":
                        self._lying_zoom = False
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._phase = "SHUTDOWN_RETURN"
                        self._tracking_enabled = False

                    elif kind == "shutdown_lie_to_sit":
                        self._current_posture = POSTURE_SITTING
                        self._target_posture = POSTURE_SITTING
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._lying_zoom = False
                        self._phase = "SHUTDOWN_RETURN"
                        self._tracking_enabled = False

                    elif kind == "shutdown_stand_to_sit":
                        self._current_posture = POSTURE_SITTING
                        self._target_posture = POSTURE_SITTING
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._lying_zoom = False
                        self._phase = "SHUTDOWN_RETURN"
                        self._tracking_enabled = False

                    elif kind == "exit":
                        self._current_posture = POSTURE_SITTING
                        self._target_posture = POSTURE_SITTING
                        self._current_yaw = 0
                        self._target_yaw = 0
                        self._lying_zoom = False
                        self._manual_action = None
                        self._exit_pending = False
                        self._shutdown_done = True
                        self._tracking_enabled = False
                        self._phase = "DONE"
                        settle = self._exit_settle_time

                    self._settling_until = time.monotonic() + settle

                elif play_status == "busy":
                    # 환영 모션 BUSY는 물리 상태 변화가 없다고 보고 같은 단계에서
                    # retry한다. Q가 걸려 있다면 다음 루프에서 motion 21 복귀만 시도한다.
                    if kind in ("welcome", "welcome_abort_return"):
                        self._phase = "WELCOME"
                        self._settling_until = (
                            time.monotonic() + self._busy_retry_delay
                        )
                        self._condition.notify_all()
                        continue

                    # H 이후 Q가 play 도중 들어왔다면 shutdown 상태를 그대로 둔다.
                    if self._shutdown_requested:
                        self._manual_action = None
                        self._tracking_enabled = False
                        self._phase = "SHUTDOWN_RETURN"

                    elif kind == "tracking_yaw":
                        # 이 얼굴 측정은 stale이므로 자동 재전송하지 않는다.
                        self._target_yaw = self._current_yaw
                        self._minimum_version_for_next_tracking_motion = (
                            used_version
                        )

                    elif kind in ("sit_to_stand", "stand_to_sit"):
                        # 자동 자세 판정 역시 stale 요청으로 자동 재전송하지 않는다.
                        self._target_posture = self._current_posture
                        self._target_yaw = self._current_yaw

                    elif kind == "startup":
                        self._startup_pending = True
                        self._phase = "STARTUP"

                    elif kind == "exit":
                        self._exit_pending = False
                        self._phase = "SHUTDOWN_RETURN"

                    # manual / shutdown은 현재 실제 상태를 기준으로 다음 루프에서
                    # 같은 안전 경로를 다시 계산한다.
                    self._settling_until = (
                        time.monotonic() + self._busy_retry_delay
                    )

                else:  # fatal
                    self._tracking_enabled = False
                    self._manual_action = None
                    self._phase = "ERROR"

                self._condition.notify_all()

    def _play_motion(self, robot, motion_id: int, label: str) -> str:
        """return: 'success' | 'busy' | 'fatal'"""
        if not (1 <= int(motion_id) <= 50):
            self._last_error = f"잘못된 motion id: {motion_id} (허용 1..50)"
            print(f"[MOTION][ERROR] {self._last_error}")
            return "fatal"

        if self._dry_run:
            print(f"[MOTION][DRY RUN] robot.play({motion_id}) | {label}")
            time.sleep(self._dry_run_motion_time)
            print(f"[MOTION][DRY RUN] completed | {label}")
            return "success"

        phorce = self._phorce

        try:
            print(f"[MOTION] robot.play({motion_id}) START | {label}")
            result = robot.play(int(motion_id))

            if not bool(result.ok):
                self._last_error = (
                    f"motion {motion_id} returned ok=False | {label}"
                )
                print(f"[MOTION][ERROR] {self._last_error}")
                return "fatal"

            print(f"[MOTION] robot.play({motion_id}) DONE | {label}")
            return "success"

        except phorce.MotionBusy as exc:
            self._last_error = f"MotionBusy: {exc}"
            print(
                f"[MOTION][BUSY] motion {motion_id} discarded; "
                f"retry after {self._busy_retry_delay:.2f}s"
            )
            return "busy"

        except phorce.MotionRejected as exc:
            self._last_error = (
                f"MotionRejected: {exc} | operator/robot state check required"
            )
            print(f"[MOTION][ERROR] {self._last_error}")
            return "fatal"

        except phorce.MotionAborted as exc:
            self._last_error = f"MotionAborted: {exc}"
            print(f"[MOTION][ERROR] {self._last_error}")
            return "fatal"

        except phorce.PhorceUnavailable as exc:
            self._last_error = f"PhorceUnavailable: {exc}"
            print(f"[MOTION][ERROR] {self._last_error}")
            return "fatal"

        except Exception as exc:
            self._last_error = (
                f"{type(exc).__name__}: {exc} | motion {motion_id} | {label}"
            )
            print(f"[MOTION][ERROR] {self._last_error}")
            return "fatal"