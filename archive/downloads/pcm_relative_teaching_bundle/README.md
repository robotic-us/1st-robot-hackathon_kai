# PCM 상대교시 배포 묶음

WalkON CM-WH(`0483:5741`)와 PCM 내부 축 7/외부 모터 노드 9를 대상으로 한
Python API, 상대교시 분할 슬롯 CSV, 호스트 권한 설정 파일이다.

## 구성

- `python/pcm_arm_slot_functions.py`: 사용자용 함수 모음
- `python/soldering_control/`: 실제 PCM USB/상대교시 구현
- `python/run_slot2_rebase_repeat.py`: 기존 슬롯 2 호환 실행기
- `python/tests/`: 통신·상대교시 단위 테스트
- `ros_ws/src/`: persistent-session 데몬을 포함한 ROS 2 소스 패키지
- `PCM_SESSION_DAEMON.md`: 데몬 서비스와 상태 토픽 사용법
- `csv/motion_41.csv`~`motion_49.csv`: 축 7 분할 슬롯
- `system/`: USB 권한과 제한된 무암호 USB-reset 설치 파일

Python 코드는 Jetson에서 실행되며 PCM이나 모터 펌웨어에 복사되지 않는다.

## 다른 Jetson에 설치

```bash
cd system
sudo ./install_pcm_permissions.sh
```

설치기는 `sudo`를 실행한 사용자를 자동으로 udev/sudoers 규칙에 넣는다. PCM을
다시 연결한 뒤 `/dev/ttyACM0` 권한을 확인한다.

## CSV 설치

PCM이 USB Storage 모드일 때 기존 `Motions` 폴더를 먼저 백업한다. `csv/`에서
`motion_41.csv`~`motion_49.csv`만 PCM SD의 `Motions/`에 복사한다.
`README.md`와 manifest는 복사하지 않는다. 이후 `sync`, 안전한 마운트 해제,
PCM 전원 재부팅 순서로 적용한다.

## 상대교시 실행

압축을 푼 최상위 폴더에서 다음처럼 실행한다.

```bash
export PYTHONPATH="$PWD/python"
python3 -m soldering_control.relative_teaching \
  --from-current --slot 47 --axes 7
```

위 예시는 `현위치 원점 저장 -> 아밍 -> 슬롯 47(+0.1 deg/2 s) -> 서보 OFF`를
실행한다. 실제 모터가 움직이므로 주변 안전과 비상 정지 수단을 먼저 확보한다.

반복 상대교시는 한 LIVE/CDC 세션을 재사용한다.

```bash
python3 -m soldering_control.relative_teaching \
  --from-current --repeat 5 --slot 43 --axes 7
```

위 명령은 현위치 기준 +10 deg를 5회 수행한다. 구현은 각 회차 사이에 PCM
Flash 저장 완료를 기다리고, CDC 장치 파일만 존재하는 stale 상태에서는 SDO
Hello를 확인한 후 필요할 때만 제한된 USB 데이터 리셋을 수행한다.

슬롯 매핑:

- 41/42: +100/-100 deg
- 43/44: +10/-10 deg
- 45/46: +1/-1 deg
- 47/48: +0.1/-0.1 deg
- 49: 선택적 0 deg 복귀

같은 PCM 기종이라도 CDC/SD 장치명이 다르면 `RelativeTeachingConfig`의
`port`와 `media_device`를 해당 장치명으로 바꾼다.

## ROS 2 persistent-session 데몬

장시간 제어에서는 슬롯 호출마다 Python 프로세스와 USB 세션을 새로 열지 말고
ROS 2 데몬 하나가 PCM 포트를 독점하도록 구성한다.

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select soldering_interfaces soldering_control \
  --symlink-install
source install/setup.bash
ros2 launch soldering_control pcm_session_daemon.launch.py
```

데몬은 작업을 직렬화하고 동일한 LIVE/SDO 세션을 재사용한다. `STOP`은 실행 중
작업뿐 아니라 그 전에 대기 중이던 모션도 취소한다. 호출 예시와 `RECONNECT`,
상태 확인 방법은 `PCM_SESSION_DAEMON.md`에 정리되어 있다.

v4부터는 작업 직전 축 상태 SDO를 검사한다. 유휴 LIVE 세션이 응답하지 않으면
USB 데이터 세션을 자동 복구하고, 복구 직후 같은 작업을 시작한다.
