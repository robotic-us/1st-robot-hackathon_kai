#!/usr/bin/env python3
"""Start the official robot stack and run motion slots 4 then 5."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

from .play_motion_4_5 import REAL_CONFIRMATION


ACTION_NAME = "/motion_action_server/play_motion_sequence"
FEEDBACK_TOPIC = "/phorce/feedback"
FEEDBACK_TYPE = "agx_msgs/msg/PhorceFeedback"


def _bridge_command(nic: str, axes: int) -> list[str]:
    return [
        "ros2",
        "run",
        "agx_phorce_bridge",
        "phorce_monitor",
        "--ros-args",
        "-p",
        f"nic:={nic}",
        "-p",
        "mode:=command",
        "-p",
        f"axes:={axes}",
        "-p",
        "mbx_enabled:=true",
    ]


def _server_command() -> list[str]:
    return [
        "ros2",
        "run",
        "agx_motion_slot",
        "motion_action_server",
        "--ros-args",
        "-p",
        "backend:=ecat",
    ]


def _sequence_command(*, execute: bool) -> list[str]:
    command = ["ros2", "run", "soldering_control", "play_motion_4_5"]
    if execute:
        command.extend(
            ["--execute", "--confirm-real", REAL_CONFIRMATION]
        )
    return command


def _start(command: list[str], label: str) -> subprocess.Popen[bytes]:
    print(f"[{label}] 시작", flush=True)
    return subprocess.Popen(command, start_new_session=True)


def _stop(process: subprocess.Popen[bytes], label: str) -> None:
    if process.poll() is not None:
        return
    print(f"[{label}] 종료 중", flush=True)
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.0)


def _action_available() -> bool:
    result = subprocess.run(
        ["ros2", "action", "list"],
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    return result.returncode == 0 and ACTION_NAME in result.stdout.splitlines()


def _wait_for_action(
    server: subprocess.Popen[bytes], timeout_s: float
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        code = server.poll()
        if code is not None:
            raise RuntimeError(f"Action 서버가 종료됐습니다 (exit={code})")
        if _action_available():
            return
        time.sleep(0.5)
    raise RuntimeError(f"{ACTION_NAME} 준비 시간이 초과됐습니다")


def _wait_for_feedback(
    bridge: subprocess.Popen[bytes] | None, timeout_s: float
) -> None:
    probe = subprocess.Popen(
        [
            "ros2",
            "topic",
            "echo",
            FEEDBACK_TOPIC,
            FEEDBACK_TYPE,
            "--qos-profile",
            "sensor_data",
            "--once",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            if bridge is not None and bridge.poll() is not None:
                raise RuntimeError(
                    "공식 EtherCAT 브리지가 종료됐습니다 "
                    f"(exit={bridge.returncode}); 위의 bring_up 오류를 확인하세요"
                )
            code = probe.poll()
            if code == 0:
                return
            if code is not None:
                raise RuntimeError(
                    f"{FEEDBACK_TOPIC} 확인 명령이 실패했습니다 (exit={code})"
                )
            time.sleep(0.2)
        raise RuntimeError(f"{FEEDBACK_TOPIC} 수신 시간이 초과됐습니다")
    finally:
        if probe.poll() is None:
            try:
                os.killpg(probe.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                probe.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(probe.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Start the official EtherCAT bridge and motion server, then "
            "play PCM slots 4 and 5 after the official preflight. Running "
            "this command is the operator's execution confirmation."
        )
    )
    parser.add_argument("--nic", default="eno1")
    parser.add_argument("--axes", type=int, default=2)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.axes <= 0 or args.startup_timeout <= 0.0:
        raise SystemExit("axes and startup-timeout must be positive")
    if not sys.stdin.isatty():
        raise SystemExit("실물 실행은 대화형 터미널에서만 허용됩니다")

    children: list[tuple[subprocess.Popen[bytes], str]] = []
    execution_confirmed = False
    try:
        if _action_available():
            print("[stack] 기존 공식 Action 서버를 사용합니다", flush=True)
            _wait_for_feedback(None, min(args.startup_timeout, 5.0))
        else:
            bridge = _start(_bridge_command(args.nic, args.axes), "bridge")
            children.append((bridge, "bridge"))
            _wait_for_feedback(bridge, args.startup_timeout)

            server = _start(_server_command(), "motion-server")
            children.append((server, "motion-server"))
            _wait_for_action(server, args.startup_timeout)

        print("[preflight] 공식 상태와 슬롯 4·5를 확인합니다", flush=True)
        preflight = subprocess.run(_sequence_command(execute=False), check=False)
        if preflight.returncode != 0:
            raise RuntimeError("사전 점검 실패 — 모션을 전송하지 않았습니다")

        print(
            "\n사전 점검 통과. 이 명령의 실행을 실물 동작 승인으로 간주합니다.\n"
            "PCM 아밍 완료, 작업 반경 비움, 물리 E-Stop 준비가 되어 있어야 합니다.",
            flush=True,
        )
        for remaining in (3, 2, 1):
            print(f"4→5 자동 실행까지 {remaining}초 (취소: Ctrl+C)", flush=True)
            time.sleep(1.0)
        execution_confirmed = True

        result = subprocess.run(_sequence_command(execute=True), check=False)
        if result.returncode == 0:
            input(
                "4→5 완료. PCM 버튼 2를 약 1초 누르고 종료 자세/흰색 LED "
                "절차가 끝난 뒤 Enter를 누르세요: "
            )
        else:
            print(
                "모션 실행이 실패했습니다. 위험하면 물리 E-Stop을 사용하고, "
                "안전한 상태에서 버튼 2로 디스암하세요.",
                flush=True,
            )
            input("로봇이 안전한 상태가 된 것을 확인한 뒤 Enter를 누르세요: ")
        return result.returncode
    except KeyboardInterrupt:
        print(
            "\n중단됨 — Ctrl+C는 E-Stop이 아닙니다. 위험하면 물리 E-Stop을 사용하세요.",
            flush=True,
        )
        return 130
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"통합 실행 실패: {exc}", file=sys.stderr, flush=True)
        if execution_confirmed:
            print(
                "위험하면 물리 E-Stop을 사용하고 안전한 상태에서 버튼 2로 "
                "디스암하세요.",
                file=sys.stderr,
                flush=True,
            )
        return 1
    finally:
        for process, label in reversed(children):
            _stop(process, label)


if __name__ == "__main__":
    raise SystemExit(main())
