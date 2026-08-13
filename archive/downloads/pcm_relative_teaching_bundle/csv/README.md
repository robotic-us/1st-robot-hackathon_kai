# PCM axis-7 resolution slots

외부 노드 9에 대응하는 PCM 내부 축 7의 양방향 분할 측정용 슬롯이다.

- 슬롯 41/42: +100/-100 deg, 10000 ms
- 슬롯 43/44: +10/-10 deg, 2000 ms
- 슬롯 45/46: +1/-1 deg, 2000 ms
- 슬롯 47/48: +0.1/-0.1 deg, 2000 ms
- 슬롯 49: 저장 원점 0 deg 복귀, 10000 ms

CSV의 P-vector 목표는 저장 원점 기준 각도다. 상대교시에서는 **각 슬롯 직전
현위치를 새 원점으로 저장**하므로 이 값이 현위치 기준 상대 이동량이 된다.
반드시 `run_relative_slot_from_current()`로 슬롯을 하나씩 호출한다. 이 호출은
`USB 데이터 복구 -> 현위치 원점 저장 -> 아밍 -> 선택 슬롯 -> 서보 OFF`를
수행한다. 따라서 슬롯 41 다음 슬롯 42를 각각 호출하면 +100 deg와 -100 deg를
각각 100 deg씩 이동한다. 슬롯 49는 상대교시에는 필수가 아닌 보조 슬롯이다.

```python
from soldering_control.relative_teaching import (
    RelativeTeachingConfig,
    run_relative_slot_from_current,
)

run_relative_slot_from_current(
    RelativeTeachingConfig(slot_id=47, axes=(7,))  # 현위치에서 +0.1 deg
)
```

이 폴더는 배포용 파일 묶음이다. 아직 PCM SD에는 복사하지 않았다. PCM에
적용할 때는 기존 `Motions`를 먼저 백업하고 이 파일들을 `Motions/`에 넣은 뒤
`sync`, 안전한 마운트 해제, LIVE 전환/모션 카탈로그 재로딩 순서로 적용한다.
