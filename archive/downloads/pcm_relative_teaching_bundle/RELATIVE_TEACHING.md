# PCM 상대교시 Python API

슬롯 모션, 현재 자세 원점 저장, 소프트웨어 아밍, 두 번째 슬롯 모션은
`soldering_control.relative_teaching`을 호출한다. 응용 코드에서 PCM SDO
인덱스를 직접 쓰지 않는다.

전체 사이클은 다음 한 번의 호출로 실행한다.

```python
from soldering_control.relative_teaching import (
    RelativeTeachingConfig,
    run_relative_teaching_cycle,
)

result = run_relative_teaching_cycle(
    RelativeTeachingConfig(slot_id=2, axes=(7,)),
)
```

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
    play_slot(client, slot_id=2, axes=(7,))
    set_current_pose_as_origin(client, axes=(7,))
    arm_from_origin(client, axes=(7,))
    play_slot(client, slot_id=2, axes=(7,))
```

첫 슬롯 뒤 SDO가 끊기는 펌웨어에서는 위 네 호출을 한 핸들에서 연속 수행하지
말고 `run_relative_teaching_cycle()`을 사용한다. CLI 호출은 다음과 같다.

```bash
ros2 run soldering_control pcm_relative_teaching --slot 2 --axes 7
```

실행 결과와 진행 이벤트는 딕셔너리/JSON으로 반환·출력된다. 실제 모션 전에
축 주변의 사람과 공구를 치우고 비상 정지 수단을 확보해야 한다.
