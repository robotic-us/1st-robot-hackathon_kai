# 행사 종료 아카이브 명세

이 저장소는 2026년 8월 13일 행사 관리자가 카이 팀 컴퓨터를 인계받은 뒤
보존한 작업물입니다. 기존 GitHub README와 최초 커밋 이력은 유지했습니다.

## 포함한 원본

- `/home/phorce/hackathon`의 대회 작업물
- `/home/phorce/participant-guide`의 참가자 가이드
- `/home/phorce/Downloads`의 대회 관련 번들, 스크립트, 문서 및 차트
- 학습 데이터, 학습 결과, 모델 체크포인트 및 장비 설정 백업
- 내부 eMMC의 `/home/phorce/participant-guide`에 남아 있던 구버전 문서
- 실제 로봇 대상 모션 시도의 JSONL 실행 기록

`/home/phorce/hand_landmarker.task`는 Downloads의 같은 파일과 SHA-256이
동일하여 중복으로 한 번 더 넣지 않았습니다.

Jetson 내부 eMMC의 참가자 가이드는 NVMe 홈 디렉터리의 가이드와 파일명이
같지만 체크섬이 다른 이전 버전이므로 `archive/legacy-emmc/participant-guide`에
별도로 보존했습니다.

## 의도적으로 제외한 항목

- Python 가상환경 `.venv`
- ROS 2/colcon의 `build`, `install`, 일반 진단 `log`
- `__pycache__`, `.pytest_cache` 등 재생성 가능한 캐시
- W&B 로컬 실행 캐시
- 빈 `.git`, Codex/에이전트 로컬 상태
- 홈 디렉터리의 인증정보, 브라우저·메일·셸 사용자 설정

이 항목들은 소스 산출물이 아니거나 인증정보 노출 위험이 있어 보존 대상에서
제외했습니다.

단, `soldering_robot_ws/log/motion_sequences`의 실제 동작 시도 기록은 일반
빌드 로그와 달리 실행 이력을 담고 있어 `archive/runtime`에 포함했습니다.

로컬 W&B 실행 기록은 온라인의 다음 완료된 run과 아티팩트 존재를 확인한 뒤
중복 캐시로 제외했습니다.

- `2q9so3hy`: finished, logged artifacts 5개
- `xv9y80av`: finished, logged artifacts 4개

## 외부 의존성 및 대용량 파일

- `vendor/sam2`는 수정되지 않은 외부 프로젝트이므로 전체 복제본 대신 사용
  당시 커밋 `2b90b9f5ceec907a1c18123530e92e794ad901a4`를 가리키는 서브모듈로
  보존했습니다.
- 모델, 데이터셋 이미지, 문서 바이너리, ZIP 및 실행 파일은 Git LFS로
  관리합니다.
- 별도 라이선스가 명시된 외부 자료에는 해당 저작권자의 조건이 적용됩니다.
