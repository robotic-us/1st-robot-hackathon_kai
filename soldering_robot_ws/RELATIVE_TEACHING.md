# PCM 상대교시 Python API

슬롯 모션, 현재 자세 원점 저장, 소프트웨어 아밍, 두 번째 슬롯 모션은
`soldering_control.relative_teaching`을 호출한다. 응용 코드에서 PCM SDO
인덱스를 직접 쓰지 않는다.

전체 사이클 API는 첫 슬롯 뒤 기본적으로 현재 자세를 HOLD하고 일시정지한다.

```python
from soldering_control.relative_teaching import (
    RelativeTeachingConfig,
    run_relative_teaching_cycle,
)

result = run_relative_teaching_cycle(
    RelativeTeachingConfig(slot_id=2, axes=(7,)),
)
```

반환 모드가 `slot_holding_paused`이면 정상적인 안전 정지다. 다음 영점 저장은
PCM 펌웨어상 서보 OFF가 필요하므로 자동 진행하지 않는다. 장치를 별도 지그로
기계적으로 지지한 경우에만 `mechanically_supported_release=True`를 명시할 수
있다. 저전류 관측만으로는 이 옵션을 켜면 안 된다.

현재 PCM 펌웨어는 슬롯 완료 뒤 Studio SDO 응답을 중단할 수 있다. 전체
함수는 시작 시와 첫 슬롯·원점 저장 사이에 USB **데이터 세션만** 리셋하고,
SD를 마운트 해제한 뒤 새 LIVE 세션을 연다. 따라서 전체 함수를 연속 호출해도
직전 슬롯의 SDO 정지 상태를 이어받지 않는다. PCM 전원 재부팅은 하지 않는다.

단계별 제어가 필요하면 같은 LIVE 세션의 클라이언트에서 다음 함수를 쓴다.

```python
from soldering_control.relative_teaching import (
    arm_from_origin,
    open_pcm_client,
    play_slot,
    RelativeTeachingConfig,
    set_current_pose_as_origin,
    start_live_session,
)

config = RelativeTeachingConfig(slot_id=2, axes=(7,))
with open_pcm_client(config) as client:
    start_live_session(client)
    # 이 시점의 서보가 이미 OFF이고 장치가 지지된 경우에만 영점 저장 가능
    set_current_pose_as_origin(client, axes=(7,))
    arm_from_origin(client, axes=(7,))
    play_slot(client, slot_id=2, axes=(7,))
```

첫 슬롯 뒤 SDO가 끊기는 펌웨어에서는 위 네 호출을 한 핸들에서 연속 수행하지
말고 `run_relative_teaching_cycle()`을 사용한다. 기본 CLI도 첫 슬롯 후
HOLDING으로 멈춘다.

```bash
ros2 run soldering_control pcm_relative_teaching --slot 2 --axes 7
```

실행 결과와 진행 이벤트는 딕셔너리/JSON으로 반환·출력된다. 실제 모션 전에
축 주변의 사람과 공구를 치우고 비상 정지 수단을 확보해야 한다.

`/motors/motor_N/telemetry`는 `current_a`, `disturbance_current_a`(DOB 전류
환산값), `load_state`, `holding_load`, `load_evidence_a`를 제공한다. 기본
HOLDING 진입 임계값은 0.025 A이고 모터별 무부하 노이즈를 측정해
`holding_enter_current_a`로 조정한다. 이 값들은 모터 토크상수 없이 N·m가
아니며, `release_permitted`는 자동으로 참이 되지 않는다.
