# 카이 — 제1회 로봇 해커톤 2026

로보틱어스(Roboticus)가 주최하는 제1회 로봇 해커톤(2026. 8. 5.~8. 8., KAIST) 참가팀 **카이**(KAIST)의 저장소입니다.

- 팀원: 정용환 · 김성은 · 김동혁
- 대회: https://robotic-us.com

## 프로젝트 구성

- `soldering_robot_ws/`: 자동 납땜 로봇 ROS 2 워크스페이스
  - `soldering_control`: PCM 및 모터 제어
  - `soldering_vision`: YOLO/ConvNeXt 기반 비전 파이프라인
  - `soldering_dashboard`: PyQt5 운영 대시보드
  - `soldering_interfaces`: 프로젝트 전용 ROS 메시지와 서비스
- `soldering_robot_ws/data/`: 학습·검증 데이터셋
- `soldering_robot_ws/models/`, `models/`: 학습 모델과 체크포인트
- `pcm_*`: PCM 설정, 모션 슬롯, 실험 백업
- `hackathon_manuals/`, `participant-guide/`: 참가자 및 장비 문서
- `phorce_studio_20260804/`: 대회 당시 제공된 phorce Studio
- `archive/downloads/`: 행사 종료 시점에 남아 있던 배포 번들과 중간 산출물
- `vendor/sam2/`: 당시 사용한 SAM2 버전을 가리키는 Git 서브모듈

자세한 보존 범위와 제외 항목은 `ARCHIVE_MANIFEST.md`를 참고하세요.

## 저장소 받기

대용량 데이터와 모델은 Git LFS로 관리됩니다. Git LFS를 설치한 뒤 서브모듈을
포함해 복제하세요.

```bash
git lfs install
git clone --recurse-submodules https://github.com/robotic-us/1st-robot-hackathon_kai.git
cd 1st-robot-hackathon_kai
git lfs pull
```

Jetson/ROS 2 환경과 실행 방법은 `AGENTS.md` 및 각 패키지의 README를
참고하세요. 실제 로봇을 움직이기 전에는 문서의 E-Stop 및 준비 절차를 반드시
확인해야 합니다.

## 지적재산권

> 본 프로젝트의 지적재산권은 카이 팀(팀원 전원)에게 있으며, 본 대회의 주최 측(로보틱어스)은 아카이브 및 홍보 목적으로만 본 저장소를 활용합니다.

라이선스는 팀이 선택해 `LICENSE` 파일로 추가하세요(MIT 또는 Apache-2.0 권장).
