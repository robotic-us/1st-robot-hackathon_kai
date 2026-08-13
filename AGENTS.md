# AGENTS.md

## 이 디렉터리의 목적

이 작업 공간은 phorce 해커톤 로봇을 Jetson의 ROS 2 환경에서 제어하기 위한 자료와 산출물을 보관한다. 사용자와의 기본 소통 언어는 한국어이며, 사용자가 선호하는 터미널 프런트엔드는 Ghostty다.

새 작업을 시작할 때는 이 파일의 요약을 기준으로 문서를 선택해서 읽고, 시간에 따라 바뀌는 로봇 연결 상태는 반드시 다시 진단한다.

## 시스템 구조

- Jetson: 참가자 코드와 ROS 2/`phorce` 명령을 실행하는 컴퓨터.
- pcm: Jetson과 관절 모터 사이의 중앙 관리자. 설정과 모션 슬롯의 정본이다.
- phact: 관절별 모터 드라이버이며 최대 12축이다.
- phorce Studio: PCM 설정, 영점, 교시 모션을 미리 저장하는 Windows 프로그램이다. 실시간 참가자 제어용 리모컨이 아니다.
- 참가자 제어는 관절 직접 명령이 아니라 PCM에 미리 적재된 모션 ID `1..50` 중 하나를 재생하는 방식이다. ID `0`은 재생하지 않는다.

## 문서 읽기 순서

HTML과 PDF는 같은 내용이므로 검색과 코드 복사가 쉬운 HTML을 우선한다.

1. 처음 연결하거나 기본 동작 흐름이 필요하면 `hackathon_manuals/01-quickstart.html`.
2. Python/C++ 제어 코드를 작성하면 `hackathon_manuals/02-tutorial.html`.
3. CLI, API, ROS 2 인터페이스, 거절 코드는 `hackathon_manuals/03-manual.html`.
4. LED, 물리 버튼, 부팅/종료, E-Stop은 `hackathon_manuals/pcm-board-guide.html`.
5. 축 구성, 영점, 교시, SD 카드 모션 저장은 `hackathon_manuals/phorce-studio-hackathon-manual.html`.

`자동납땜로봇_ROS2_PVector_제어개발계획서_v2.docx`와 `해커톤_추가공지_p vector.pptx`는 매뉴얼 요약에 포함되지 않은 별도 자료다. PVector나 자동 납땜 로봇 요구사항을 다룰 때는 이 파일들도 직접 확인한다. ZIP 파일은 필요해질 때 목록을 먼저 확인하고, 무작정 덮어쓰거나 작업 공간에 풀지 않는다.

## 현재 환경에 관해 확인된 사실

2026-08-05 점검 당시 Jetson은 Ubuntu 22.04 aarch64, PREEMPT_RT 커널, ROS 2 Humble 환경이었다. 다음 항목이 설치되어 있었다.

- `/opt/ros/humble/bin/ros2`
- `/opt/ros/humble/bin/phorce`
- `/opt/ros/humble/bin/phorce-console`
- `colcon`, Python 3.10, PyQt5, pyqtgraph
- Python `phorce` 패키지와 `/opt/ros/humble/share/phorce/examples/`의 예제 4개

ROS 환경은 `/etc/profile.d/agr-phorce-env.sh`가 Bash 로그인 셸에서 로드한다. 일반 가상환경은 ROS Python 패키지를 가릴 수 있으므로, 추가 패키지가 꼭 필요하면 `python3 -m venv --system-site-packages ...` 방식을 우선 검토한다.

2026-08-06에 Jetson에 ARM64 Ghostty v1.3.1을 stable Snap으로 설치했다. 이 RT 커널에서는 `snap-confine` capability 오류가 발생하므로 `/snap/bin/ghostty`를 직접 사용하지 않는다. 대신 `~/.local/bin/ghostty` 래퍼가 Snap의 현재 버전 바이너리와 리소스를 직접 실행하며, GNOME 메뉴도 사용자 desktop entry를 통해 이 래퍼를 사용한다. Snap 자동 업데이트 후에는 래퍼가 `/snap/ghostty/current`를 따라간다. GNOME 사용자 기본 터미널과 `Ctrl+Alt+T`는 이 Ghostty 래퍼를 사용하며, PATH 기반 `x-terminal-emulator` 호출은 `~/.local/bin/x-terminal-emulator`가 Ghostty로 전달한다. 시스템 전체 alternatives는 다른 사용자에게 영향을 주지 않도록 변경하지 않았다. 사용자 설정은 아직 비어 있어 Ghostty 기본값을 사용한다. Ghostty 설정을 요청받으면 이 로컬 Jetson 설치를 대상으로 하되, 셸은 현재 Bash 기준을 유지한다. `tmux`, `direnv`, `pip3`는 2026-08-05 점검 당시 없었다.

이 디렉터리는 당시 Git 저장소가 아니었다. 변경 전에 현재 상태를 다시 확인하고, 사용자의 기존 파일을 보존한다.

### 프로젝트 개발 의존성

2026-08-06에 자동 납땜 로봇 계획서와 P-Vector 공지를 검토하고 다음 환경을 준비했다.

- 시스템 패키지: `python3-pip`, `python3-venv`, `python3-cffi`, `v4l-utils`, NVIDIA FFmpeg.
- ROS 2 카메라 스택: `usb_cam`, `camera_info_manager`, `camera_calibration`, `image_proc`, `image_transport_plugins`, `diagnostic_updater`와 의존 패키지.
- 프로젝트 가상환경: `/home/phorce/hackathon/.venv`. ROS/Ubuntu 패키지를 가리지 않도록 `python3 -m venv --system-site-packages .venv`로 생성했다.
- Jetson AI: `.venv`에 JetPack 6.2.1/CUDA 12.6용 `torch==2.8.0`, `torchvision==0.23.0`을 설치했다. 재설치 명세는 `requirements-jetson-ai.txt`다.
- 샌드박스 밖 GPU 검증에서 CUDA 12.6, cuDNN 9.3, Jetson Orin 장치와 `convnext_tiny` 단일 추론이 정상 동작했다.

Python/ROS 제어 코드를 개발하거나 실행할 때는 먼저 `source /home/phorce/hackathon/.venv/bin/activate`를 사용한다. 이 가상환경에서 `rclpy`, `phorce`, `agx_msgs`, `cv_bridge`, OpenCV, NumPy, PyTorch를 함께 사용할 수 있다. 2026-08-06 점검 당시 `/dev/video*` 장치는 없었으므로 웹캠 드라이버의 실제 영상 수신과 카메라 보정은 웹캠 연결 후 다시 검증해야 한다.

## 매 작업 시작 시 읽기 전용 점검

로봇 상태는 이전 대화나 이 파일의 기록을 신뢰하지 말고 아래 명령으로 다시 확인한다. 이 명령들은 모션을 실행하지 않는다.

```bash
phorce doctor --json
phorce status --json
phorce list --json
ros2 topic hz /phorce/feedback
ros2 topic echo /phorce/feedback --once
```

`ros2 topic hz`는 필요한 샘플을 얻은 뒤 `Ctrl+C`로 종료한다. 정상 피드백은 대략 1kHz다. 진단 결과, 모션 목록, ROS graph, LED/버튼 상태는 서로 다른 층의 정보이므로 하나만 보고 실물 준비 완료라고 판단하지 않는다.

2026-08-05의 일시적 점검에서는 액션과 목록 endpoint가 응답하고 슬롯 1 하나가 보였지만, `doctor`는 액션 서버 신원과 비어 있는 `motion_dir`/`catalog_manifest` 경고로 `ok=false`였다. 이 값은 현재 상태가 아니므로 그대로 인용하지 말고 재검사한다. 경고가 남아 있으면 실물 재생 전에 원인 확인 또는 운영진의 정상 판정을 받는다.

## 실물 안전 규칙

- 사용자가 실물 동작을 명시적으로 요청하지 않았다면 `phorce play --target robot`이나 raw ROS action goal을 보내지 않는다. 진단·설명·계획 요청은 실물 동작 권한이 아니다.
- 개발과 검증의 기본 대상은 `sim:demo`다. 실물 대상은 명령에 명시적으로 표시하고, 시뮬레이터와 실물용 실행 경로를 분리한다.
- 실물 명령 직전 사람이 로봇 주변, 물리 E-Stop 위치, PCM LED, 준비 버튼 상태를 확인해야 한다.
- 문서에는 기능 버튼 1 유지 시간이 0.6초 이상과 1초로 혼재한다. 현장 기준은 안전하게 약 1초로 잡는다. 버튼 1 이후 3초 경고 및 움직임 가능 상태에서는 물러선다.
- 비상 정지는 물리 E-Stop만 사용한다. 코드의 `cancel()`, GUI의 재생 정지, 서보 끄기는 E-Stop이 아니다.
- E-Stop이 걸리면 스위치만 되돌려서는 복구되지 않으며 전원 재투입이 필요하다.
- 종료는 기능 버튼 2를 약 1초 누르고 종료 자세/흰색 LED 절차가 끝난 다음 전원을 분리한다.
- SD 카드를 물리적으로 빼는 작업은 PCM 전원을 끈 뒤에만 한다.

## 제어 코드의 필수 계약

- `/phorce/feedback` 직접 구독에는 반드시 `qos_profile_sensor_data`를 사용한다. 기본 reliable QoS를 쓰면 오류 없이 메시지가 하나도 안 올 수 있다.
- 축 값을 사용하기 전에 반드시 해당 축의 `valid`가 참인지 검사한다. `!stale`은 대체 조건이 아니다.
- 1kHz 피드백 콜백에서는 최신 상태만 저장한다. 판단과 `play()`는 약 2Hz 수준의 별도 느린 루프에서 수행한다.
- PCM에는 모션 대기열이 없다. 한 요청에 모션 ID 하나만 보내고, 이전 동작의 완료를 확인한 뒤 다음 동작을 보낸다.
- `MotionBusy`/거절 코드 5만 기다렸다 재시도한다.
- `MotionRejected`의 준비 필요 코드 12는 사람이 버튼 1을 눌러야 하고, 복구 필요 코드 13은 버튼 2로 파킹한 뒤 다시 준비해야 한다. 무한 재시도하지 않는다.
- `MotionAborted`는 메시지의 복구 절차를 따르고 자동 반복하지 않는다.
- 참가자 코드에서 raw PDO/SDO, 관절 1kHz 직접 하행 스트리밍, 안전 감시자 우회를 시도하지 않는다.
- C++ 콜백 안에서 future의 `get()`으로 블로킹하지 않는다. 타이머/루프에서 논블로킹으로 확인한다.
- C++ 저수준 프로그램을 다시 빌드한 경우에만 필요한 범위에서 `agr-setcap-ethercat` 권한을 검토한다. 모션 슬롯 API 사용에는 보통 필요 없다.

## 권장 작업 방식

- 새 제어 코드는 먼저 설치된 `03_feedback_to_motion.py` 예제 구조를 참고한다.
- 편의 스크립트를 만들 경우 기본 target을 시뮬레이터로 하고, 실물 실행에는 명시적 플래그와 실행 전 확인 절차를 둔다.
- `phorce list` 결과가 PCM의 실제 적재 모션 정본이다. Jetson의 파일명만 보고 재생 가능 여부를 판단하지 않는다.
- 자동화 로그에는 시각, target, domain ID/namespace, 선택한 모션 ID, `doctor/status/list` 결과, 성공·거절 사유를 남긴다.
- Ghostty에서는 진단, 피드백 관찰, 제어 실행, 로그 확인을 분리된 surface/split로 구성하는 방식을 우선한다. 원격 세션 유지가 필요할 때만 `tmux` 설치를 제안한다.
- 실제 파일 변경을 요청받으면 관련 문서를 먼저 읽고 구현한 뒤, 위험도에 비례해 시뮬레이터 또는 읽기 전용 진단으로 검증한다.
