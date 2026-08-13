# PCM persistent-session ROS 2 daemon

`pcm_session_daemon`은 PCM Studio SDO 연결을 한 프로세스가 독점하고,
모든 모션 요청을 하나의 작업 큐에서 순서대로 실행한다. 각 슬롯 호출마다
USB를 다시 열거나 PCM을 재부팅하지 않는다.

## 빌드와 실행

```bash
cd /home/phorce/hackathon/soldering_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select soldering_interfaces soldering_control --symlink-install
source install/setup.bash
ros2 launch soldering_control pcm_session_daemon.launch.py
```

기본값은 `/dev/ttyACM0`, PCM 미디어 `/dev/sda1`, 축 `[7]`이다. 다른 축을
사용할 때는 launch 파일의 `default_axes`를 실제 PCM 축 번호에 맞춘다.

## 명령 서비스

서비스 이름은 `/pcm_session_daemon/command`이며 타입은
`soldering_interfaces/srv/PcmCommand`이다. 응답의 `accepted=true`는 작업이
큐에 들어갔다는 뜻이고, 모션 완료를 뜻하지 않는다. 완료 여부는 상태에서
확인한다.

현재 위치를 원점으로 잡은 뒤 슬롯 43을 축 7에서 5회 실행:

```bash
ros2 service call /pcm_session_daemon/command \
  soldering_interfaces/srv/PcmCommand \
  "{operation: 1, slot_id: 43, axes: [7], repeat: 5}"
```

저장된 절대 슬롯을 1회 실행:

```bash
ros2 service call /pcm_session_daemon/command \
  soldering_interfaces/srv/PcmCommand \
  "{operation: 2, slot_id: 43, axes: [7], repeat: 1}"
```

현재 원점 기준 아밍:

```bash
ros2 service call /pcm_session_daemon/command \
  soldering_interfaces/srv/PcmCommand \
  "{operation: 3, slot_id: 0, axes: [7], repeat: 0}"
```

소프트웨어 정지 및 서보 OFF:

```bash
ros2 service call /pcm_session_daemon/command \
  soldering_interfaces/srv/PcmCommand \
  "{operation: 4, slot_id: 0, axes: [7], repeat: 0}"
```

`STOP`은 실행 중 작업에 취소를 요청하고, 그보다 먼저 큐에 들어간 미실행
작업도 무효화한다. 새 명령은 `STOP` 이후 다시 제출해야 한다. 이 기능은
SDO를 통한 최선형 소프트웨어 정지이며 물리 비상정지 장치를 대체하지 않는다.

SDO 세션이 끊어진 뒤 USB 데이터 세션을 복구하고 다시 연결:

```bash
ros2 service call /pcm_session_daemon/command \
  soldering_interfaces/srv/PcmCommand \
  "{operation: 5, slot_id: 0, axes: [7], repeat: 0}"
```

모션 명령(`operation` 1, 2)은 `slot_id=1..50`, `repeat>=1`이어야 한다.
축 번호는 중복 없이 `0..11` 범위여야 한다. 빈 `axes`는 launch의
`default_axes`를 사용한다.

## 상태 확인

가장 최근 상태는 latched 토픽과 서비스 양쪽에서 확인할 수 있다.

```bash
ros2 topic echo --once /pcm_session_daemon/status std_msgs/msg/String
ros2 service call /pcm_session_daemon/get_status std_srvs/srv/Trigger "{}"
```

주요 필드는 `state`, `connected`, `queued_jobs`, `active_job_id`,
`repeat_completed`, `last_job_id`, `last_success`, `detail`, `last_result`다.
대표 상태는 `connecting`, `ready`, `rebasing`, `arming`, `playing`,
`parking`, `stopping`, `recovering`, `fault`다.

## 동작 경계

- 데몬 프로세스 하나만 `/dev/ttyACM0`를 연다.
- 슬롯·원점 재설정·아밍은 워커 스레드 하나가 직렬 실행한다.
- 정상 반복 중에는 같은 LIVE/SDO 세션을 계속 사용한다.
- 작업 직전에 축 상태 SDO를 검사하며, 유휴 세션이 끊겼으면 USB 데이터
  세션을 자동 복구한 직후 작업을 시작한다.
- 연결 실패는 `fault`로 남고 자동으로 모션을 재시도하지 않는다.
- 복구는 명시적인 `RECONNECT` 명령으로 수행한다.
- 데몬 종료 시 정지 및 서보 OFF를 최선형으로 요청한 뒤 포트를 닫는다.
