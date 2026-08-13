# soldering_vision

단일 고정 웹캠으로 자동 납땜 작업면을 관측하는 ROS 2 패키지다. 이 패키지는
모터나 PCM에 명령하지 않고 관측 토픽만 발행한다.

## 런타임 구조

```text
/camera/image_raw
        │
        ▼
YOLO segment/pose
  ├─ fixed_wire / moving_wire 끝점
  ├─ iron_tip / solder_wire 끝점
  └─ copper / insulation / solder_joint mask
        │
        ├─ homography → 작업 평면 x,y(mm)
        ├─ class별 추적 → 안정된 track_id/anchor
        └─ 접합부 interaction crop
                    │
                    ▼
              ConvNeXt-Tiny
       ready/contact_good/solder_good/
       insufficient/excess/misaligned/
       occluded/unsafe
                    │
                    ▼
/soldering/geometry_observation
/soldering/vision/annotated
```

YOLO는 물체와 끝점의 위치를 담당하고 ConvNeXt는 YOLO crop 안의 공정 상태만
분류한다. `PREHEAT`, `FEED` 같은 공정 단계는 이미지 분류 클래스에 넣지 않고
상위 supervisor가 판단한다. 한 대의 카메라에서 임의의 깊이를 추정하지 않으며,
고정 지그의 작업 평면에 대해서만 homography로 mm 좌표를 만든다.

`GeometryObservation.valid`는 다음 조건을 모두 만족할 때만 참이다.

- 카메라/homography 보정 완료
- fixed wire, moving wire, iron tip, solder wire가 신뢰도 기준 이상
- ConvNeXt confidence가 기준 이상이고 결과가 `occluded`/`unsafe`가 아님

따라서 모델이나 보정 파일이 없는 초기 상태는 제어에 사용할 수 없도록
fail-closed로 동작한다.

## 합성 스모크 테스트

AI 모델과 웹캠 없이 ROS 계약을 검사한다.

```bash
cd /home/phorce/hackathon/soldering_robot_ws
source /opt/ros/humble/setup.bash
source /home/phorce/hackathon/.venv/bin/activate
source install/setup.bash
ros2 launch soldering_vision synthetic_vision.launch.py
```

다른 터미널에서:

```bash
ros2 topic echo --once \
  /soldering/geometry_observation \
  soldering_interfaces/msg/GeometryObservation
ros2 run image_view image_view \
  --ros-args -r image:=/soldering/vision/annotated
```

HSV backend는 colored-marker 기준선 및 배선 검증용이다. 실제 납땜 판단에는
사용하지 않는다.

## 실제 웹캠 실행

먼저 `/dev/video0`가 존재하는지 확인하고 카메라 내부 파라미터와 작업면
homography를 보정한다. 그 후 `config/vision.yaml`의 `homography`와
`homography_valid`를 갱신한다.

```bash
ros2 launch soldering_vision vision_pipeline.launch.py \
  camera_device:=/dev/video0 \
  detector_backend:=yolo \
  yolo_model:=/absolute/path/to/best.engine \
  convnext_model:=/absolute/path/to/convnext_process.pt
```

YOLO PyTorch checkpoint(`best.pt`)도 사용할 수 있지만 Jetson 배포에서는
TensorRT engine을 권장한다. TensorRT engine은 생성한 Jetson/TensorRT 버전에
묶이므로 대상 Jetson에서 export한다.

## 1. YOLO 학습 데이터

클래스 정의는 `config/soldering_yolo_dataset.yaml`에 있다.

```text
fixed_wire, moving_wire, iron_tip, solder_wire,
solder_joint, insulation, copper
```

전선과 공구 전체를 instance segmentation polygon으로 표시한다. 접합점 쪽
끝점이 항상 영상 중앙의 작업 ROI를 향하도록 카메라와 지그를 고정한다. 더 높은
끝점 정밀도가 필요하면 동일 class에 첫 keypoint를 끝점으로 둔 pose 모델로
교체할 수 있다.

촬영은 다음 단위로 나눈다.

- 서로 다른 날짜/조명/노출/초점/전선 색상의 session
- 인두기와 납선의 정상 접근, 부분 가림, 반사, 모션 블러
- 물체가 없거나 엉뚱한 공구가 들어온 negative frame
- 납땜 전/가열 중/공급 중/완료 후 장면

동영상은 5~10 fps 이하로 샘플링해 거의 같은 프레임의 중복을 줄인다. 같은
동영상이나 작업 session을 train과 val에 나누면 안 된다. 초기 목표는 상태별
독립 session 10개 이상, 중복 제거 프레임 500장 이상이며 실제 검증 오차를 보고
추가한다.

예시 학습:

```bash
source /home/phorce/hackathon/.venv/bin/activate
python3 -m pip install -r \
  src/soldering_vision/config/requirements-vision-ai.txt
yolo segment train \
  model=yolo11n-seg.pt \
  data=src/soldering_vision/config/soldering_yolo_dataset.yaml \
  imgsz=640 epochs=100 batch=16 device=0
```

Jetson TensorRT FP16 export 예시:

```bash
yolo export model=runs/segment/train/weights/best.pt \
  format=engine imgsz=640 quantize=16 device=0
```

## 2. ConvNeXt crop 데이터

원본 프레임을 다음 구조로 정리한다. 디렉터리의 session 이름이 train/val 분할
단위다.

```text
recordings/
├─ day01_fixture_a/
│  ├─ ready/*.jpg
│  ├─ contact_good/*.jpg
│  ├─ solder_good/*.jpg
│  ├─ insufficient/*.jpg
│  ├─ excess/*.jpg
│  ├─ misaligned/*.jpg
│  ├─ occluded/*.jpg
│  └─ unsafe/*.jpg
└─ day02_fixture_b/...
```

학습된 YOLO로 실제 추론 crop과 위치 흔들림 crop을 생성한다.

```bash
ros2 run soldering_vision build_process_crops \
  --input /data/recordings \
  --output /data/process_crops \
  --yolo-model /models/yolo_soldering_best.pt \
  --jitter-copies 2
```

결과는 `/data/process_crops/train/<class>`와 `val/<class>`에 저장된다. 정답
polygon으로 만든 완벽한 crop만 사용하지 않는 이유는 YOLO 위치 오차가 있는
실전 입력 분포를 ConvNeXt가 학습하게 하기 위해서다.

ConvNeXt-Tiny 학습:

```bash
ros2 run soldering_vision train_convnext \
  --data /data/process_crops \
  --output /models/convnext_process.pt \
  --epochs 25 --batch-size 32 --device cuda
```

학습은 ImageNet ConvNeXt-Tiny weight에서 시작하고 처음 3 epoch는 backbone을
고정한다. 밝기, 약한 회전/scale, blur augmentation을 적용하지만 지그의 좌우
의미가 다를 수 있으므로 horizontal flip은 기본 사용하지 않는다.

### W&B 실험 기록

API key는 코드나 설정 파일에 쓰지 않고 Jetson 터미널에서 한 번만 입력한다.

```bash
source /home/phorce/hackathon/.venv/bin/activate
python3 -m pip install "wandb>=0.22.3,<1"
wandb login --verify
```

그 후 학습 명령에 다음을 추가한다.

```bash
--wandb-project soldering-vision --wandb-run-name convnext-baseline-01
```

trainer는 epoch loss/accuracy/config와 최상 checkpoint artifact를 기록한다.
토큰이 들어가는 `~/.netrc`, 환경변수, `wandb/` 실행 디렉터리는 배포 ZIP과
데이터셋에 포함하지 않는다. 네트워크가 불안정하면 `WANDB_MODE=offline`으로
학습한 뒤 별도로 sync한다.

## 평가 기준

- YOLO: class별 mask mAP와 별도로 끝점 pixel/mm 오차를 측정
- geometry: 고정 자/체커보드 기준 작업면 좌표 오차
- ConvNeXt: class별 precision/recall, confusion matrix
- 안전 class: `occluded`, `unsafe`의 recall과 false-safe 건수를 최우선 확인
- 전체: YOLO crop을 입력으로 한 end-to-end 상태 정확도와 추론 지연

검증을 통과하기 전에는 관측 토픽을 모션 허가 조건으로 사용하지 않는다.

## 향후 상태 추정 연결

이 패키지는 영상 관측만 발행한다. 이후 `state_estimator_node`가
`/soldering/geometry_observation`과 `/master_mcu/observations`의 timestamp를
맞추고, GaP형 뉴럴 Kalman의 observation으로 함께 사용한다. 영상이 가려졌을
때 모터 상태로 짧게 예측할 수는 있지만 `vision valid=false`를 숨기지는 않는다.
