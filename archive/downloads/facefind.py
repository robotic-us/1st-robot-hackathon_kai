# 파일 구성:
#   facefind.py           : 얼굴 및 손 인식 메인 프로그램
#   motion_controllerstand.py : phorce SDK 기반 sitting/standing/lying/zoom 모션 제어 모듈
#
import cv2
import mediapipe as mp
import numpy as np
import time
import importlib.util
import sys

from pathlib import Path
from collections import deque
from cvzone.HandTrackingModule import HandDetector

# 항상 이 facefind.py와 같은 폴더의 motion_controllerstand.py를 직접 로드한다.
# 같은 이름의 예전 모듈이 다른 폴더/PYTHONPATH에 있어도 잘못 import하지 않게 한다.
CURRENT_DIR = Path(__file__).resolve().parent
CONTROLLER_PATH = CURRENT_DIR / "motion_controller.py"

if not CONTROLLER_PATH.is_file():
    raise FileNotFoundError(
        f"motion controller not found next to facefind.py: {CONTROLLER_PATH}"
    )

_controller_spec = importlib.util.spec_from_file_location(
    "robot_motion_controller_local",
    CONTROLLER_PATH,
)
if _controller_spec is None or _controller_spec.loader is None:
    raise ImportError(f"cannot load motion controller: {CONTROLLER_PATH}")

_controller_module = importlib.util.module_from_spec(_controller_spec)
sys.modules[_controller_spec.name] = _controller_module
_controller_spec.loader.exec_module(_controller_module)
MotionController = _controller_module.MotionController

print(f"[CODE] facefind loaded from: {Path(__file__).resolve()}")
print(f"[CODE] motion controller loaded from: {CONTROLLER_PATH.resolve()}")
print("[CODE] expected startup: 19 -> 20 -> 20 -> 21 -> WAIT_FOR_H -> H -> 1")


# ============================================================
# 1. 기본 설정
# ============================================================

MODEL_PATH = str(CURRENT_DIR / "face_landmarker.task")

CAMERA_INDEX = 0          # USB 카메라가 보통 1번이다.
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
MIRROR_IMAGE = True

# True: 카메라 화면 위에 글자, 랜드마크, 선을 표시한다.
# False: 카메라 화면만 표시하고 오버레이는 그리지 않는다.
SHOW_DISPLAY = True

# False: 실제 phorce Python API로 로봇 motion 1~18을 실행한다.
# 로봇 없이 코드 흐름만 시험할 때만 True로 바꾼다.
MOTION_DRY_RUN = False

# ============================================================
# 큰 자세 판정 설정: sitting <-> standing
#
# 전신 포즈 인식기는 사용하지 않고, 3손가락 보정 시 저장한 얼굴 중심 Y를
# 기준으로 자세 변화를 판단한다.
#
# sitting -> standing:
#   얼굴이 기준 위치보다 위로 이동했다는 힌트가 먼저 생기고,
#   그 뒤 얼굴이 실제로 사라진 상태가 일정 시간 유지되어야 확정한다.
#   얼굴이 보이는 동안에는 standing으로 바로 전환하지 않는다.
#
# standing -> sitting:
#   기존 FaceHeight 단계 기반 판정을 그대로 유지한다.
# ============================================================

# sitting center에서 얼굴 중심이 보정 위치보다 화면 높이의 2% 이상 위로
# 올라가면 '일어나는 중일 수 있음'으로만 기억한다.
# 이것만으로 motion 9를 실행하지는 않는다.
STANDING_FACE_LOST_HINT_CHANGE = 0.02

# 위쪽 이동 힌트 이후 얼굴이 이 시간 동안 계속 미검출되어야 standing 확정.
STANDING_FACE_LOST_CONFIRM_TIME = 0.8

# standing 상태에서 얼굴이 이 단계까지 내려가면 sitting으로 확정한다.
# 기존 동작을 그대로 유지한다.
SITTING_VISIBLE_THRESHOLD = -1

# standing -> sitting에서 얼굴이 아래쪽으로 사라질 때 사용할 기존 힌트 단계.
POSTURE_FACE_LOST_HINT_STEP = 1

# standing -> sitting의 기존 face-lost fallback 확정 시간.
POSTURE_FACE_LOST_CONFIRM_TIME = 1.0

# ============================================================
# 2. 얼굴 각도 방향 부호
#
# 목표:
# 오른쪽으로 고개 회전     -> Yaw 양수
# 위로 고개 들기          -> Pitch 양수
# 오른쪽 어깨로 기울이기  -> Roll 양수
#
# 방향이 반대이면 해당 값만 -1.0으로 바꾼다.
# ============================================================

PITCH_SIGN = 1.0
YAW_SIGN = 1.0
ROLL_SIGN = 1.0


# ============================================================
# 3. 얼굴 단계 처리 설정
# ============================================================

MEDIAN_WINDOW = 0.5
STEP_HOLD_TIME = 0.3
# 모션 직후에는 움직이는 카메라에서 얻은 이전 얼굴 단계가 더 이상 유효하지 않으므로
# 새 얼굴을 이 시간 동안 다시 확인한 뒤 일반 얼굴 yaw 추적을 재개한다.
POST_MOTION_REACQUIRE_TIME = STEP_HOLD_TIME
HYSTERESIS = 2.0

CENTER_ANGLE = 13.0
STEP_ANGLE = 28.0
MAX_STEP = 5

CALIBRATION_TIME = 1.0
NO_FACE_HOLD_TIME = 5.0

# 얼굴 중심의 수직 이동을 화면 전체 높이 기준으로 단계화한다.
# ±5% 이내는 0단계, 이후 화면 높이의 20%마다 1단계다.
FACE_HEIGHT_DEADZONE = 0.05
FACE_HEIGHT_PER_STEP = 0.20
FACE_HEIGHT_HYSTERESIS = 0.02
FACE_HEIGHT_MAX_STEP = 5


# ============================================================
# 4. 손가락 직접 yaw / 거리 조정 설정
# ============================================================

# 검지만 편 자세가 이 시간 이상 유지되면 조정 시작
OFFSET_GESTURE_START_TIME = 0.2

# 조정 중 검지 자세가 이 시간 이상 사라지면 값 확정
OFFSET_GESTURE_END_TIME = 0.3

# 한 손 검지는 sitting에서 -1 / 0 / +1의 저장 상태를 한 제스처당 딱 한 칸만 바꾼다.
# 내부 표현은 기존 직접 제어와 호환되도록 -1 / None(=0, FACE) / +1을 사용한다.
#
# 새 검지 제스처에서:
#   왼쪽 100px 이상  : +1 -> 0, 0 -> -1, -1 -> -1
#   오른쪽 100px 이상: -1 -> 0, 0 -> +1, +1 -> +1
#
# 중요한 점: 한 번 검지를 든 동안에는 저장값을 최대 한 번만 바꾼다.
# 손가락을 내렸다가 다시 들어야 다음 한 칸 변경이 가능하다.
# 로봇 motion 출력/경로 계산 로직은 기존 그대로 유지한다.
YAW_GESTURE_ENTER_PX = 100.0
FINGER_YAW_LEFT_TARGET = -1
FINGER_YAW_RIGHT_TARGET = 1

# 두 검지 사이 거리가 화면 너비의 10% 변할 때 m_dist 1단계
M_DIST_STEP_RATIO = 0.10
M_DIST_MIN = -5
M_DIST_MAX = 5

# 검지·중지·약지 3개를 이 시간 동안 유지하면 얼굴 영점 보정을 시작한다.
THREE_FINGER_CALIBRATION_HOLD_TIME = 0.2

# 검지·중지·약지·새끼손가락 4개 제스처는 인식만 유지한다.
# 현재는 종료 동작과 연결하지 않는다.
FOUR_FINGER_QUIT_HOLD_TIME = 0.2


# ============================================================
# 5. MediaPipe Face Landmarker 별칭
# ============================================================

BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
RunningMode = mp.tasks.vision.RunningMode


# ============================================================
# 6. 인식기
# ============================================================

hand_detector = HandDetector(
    staticMode=False,
    maxHands=2,
    detectionCon=0.8,
    minTrackCon=0.5
)


# 손이 없을 때는 3프레임마다 탐색한다.
# 손이 최근 검출된 동안에는 2프레임마다 추적한다.
# 순간적으로 손을 놓쳐도 0.15초 동안 직전 결과를 유지한다.
HAND_SEARCH_INTERVAL = 3
HAND_TRACK_INTERVAL = 2
HAND_LOST_GRACE_TIME = 0.15


# ============================================================
# 7. 손 제스처 상태
# ============================================================

# 네 손가락 종료 제스처 상태
four_finger_since = None

# 세 손가락 얼굴 영점 보정 제스처 상태
three_finger_since = None
three_finger_latched = False

# 검지 직접 yaw 제스처 감지 상태
index_gesture_since = None
index_gesture_last_seen = None

# 실제 한 손 검지 직접 yaw 조정이 진행 중인지
offset_adjusting = False

# 조정 시작 시 검지 끝 좌표. 이 위치가 손가락 중앙 기준이다.
offset_start_finger = None

# 저장된 손가락 yaw 상태.
# -1: 왼쪽 직접 목표, None: 0(중앙/얼굴 추적), +1: 오른쪽 직접 목표.
finger_yaw_target = None

# 한 번 검지를 든 동안 저장값을 이미 한 칸 변경했는지 기억한다.
# True가 되면 손가락을 내릴 때까지 추가 좌/우 변화는 무시한다.
finger_yaw_step_applied = False

# 기존 코드 호환/상태 초기화용. 새 한-제스처-한-칸 로직에서는 중앙 타이머를 쓰지 않는다.
finger_center_since = None

# 중앙 복귀로 강제 제어가 해제되면 얼굴 제어 history를 버리고
# 새 얼굴을 다시 확인하도록 메인 루프에 요청한다.
finger_face_reacquire_requested = False

# pitch_cal은 기존 화면/호환성을 위해 유지한다. 한 손 검지는 더 이상 보정값을 만들지 않는다.
pitch_cal = 0

# 두 손 검지 거리로 조정하는 정수 변수
m_dist = 0

# 두 손 거리 조정 상태
dual_gesture_since = None
dual_gesture_last_seen = None
m_dist_adjusting = False
m_dist_start_distance = None
m_dist_start_value = 0


# ============================================================
# 8. 개인별 얼굴 원점
#
# 얼굴이 사라져도 유지된다.
# C 키로 원점 보정을 완료했을 때만 변경된다.
# ============================================================

calibrated = False

angle_origin = {
    "pitch": 0.0,
    "yaw": 0.0,
    "roll": 0.0
}

face_height_origin = {
    "center_y": 0.0,
    # 얼굴 크기 자체는 보정 정보로만 저장한다.
    # 얼굴 높이 단계 계산에는 사용하지 않는다.
    "height": 1.0
}


# ============================================================
# 9. 얼굴 원점 보정 상태
# ============================================================

calibration_active = False
calibration_start_time = None
calibration_samples = []


# ============================================================
# 10. 상대 얼굴 각도 기록
# ============================================================

angle_history = deque()
face_height_history = deque()


# ============================================================
# 11. 단계 상태
# ============================================================

axis_states = {
    "pitch": {
        "confirmed": 0,
        "candidate": 0,
        "candidate_since": None
    },
    "yaw": {
        "confirmed": 0,
        "candidate": 0,
        "candidate_since": None
    },
    "roll": {
        "confirmed": 0,
        "candidate": 0,
        "candidate_since": None
    },
    "height": {
        "confirmed": 0,
        "candidate": 0,
        "candidate_since": None
    }
}


# ============================================================
# 12. 얼굴 미검출 및 sitting/standing 전환 상태
# ============================================================

last_face_seen_time = None
no_face_printed = False
no_face_timeout_printed = False

# 얼굴이 보이다가 위/아래 화면 밖으로 사라지는 경우를 위한 상태다.
posture_face_lost_candidate = None   # None | "standing" | "sitting"
posture_face_lost_since = None
last_visible_height_step = 0

# sitting center에서 얼굴이 위로 이동했다는 흔적을 기억한다.
# True여도 얼굴이 계속 보이는 동안에는 standing으로 전환하지 않는다.
standing_exit_armed = False



# ============================================================
# 13. 각도 관련 함수
# ============================================================

def normalize_angle(angle):
    """각도를 -180도 이상, +180도 미만 범위로 정규화한다."""
    return (angle + 180.0) % 360.0 - 180.0


def angle_difference(current_angle, origin_angle):
    """현재 각도와 원점 사이의 최단 각도 차이를 반환한다."""
    return normalize_angle(current_angle - origin_angle)


def circular_mean_degrees(angles):
    """각도 배열의 원형 평균을 계산한다."""
    if len(angles) == 0:
        raise ValueError("각도 데이터가 비어 있다.")

    radians = np.radians(angles)

    mean_sin = np.mean(np.sin(radians))
    mean_cos = np.mean(np.cos(radians))

    mean_angle = np.degrees(
        np.arctan2(mean_sin, mean_cos)
    )

    return float(normalize_angle(mean_angle))


def rotation_matrix_to_euler(rotation_matrix):
    """3x3 회전 행렬을 pitch, yaw, roll로 변환한다."""
    r = rotation_matrix

    sy = np.sqrt(
        r[0, 0] ** 2
        + r[1, 0] ** 2
    )

    singular = sy < 1e-6

    if not singular:
        pitch_radian = np.arctan2(
            r[2, 1],
            r[2, 2]
        )

        yaw_radian = np.arctan2(
            -r[2, 0],
            sy
        )

        roll_radian = np.arctan2(
            r[1, 0],
            r[0, 0]
        )

    else:
        pitch_radian = np.arctan2(
            -r[1, 2],
            r[1, 1]
        )

        yaw_radian = np.arctan2(
            -r[2, 0],
            sy
        )

        roll_radian = 0.0

    pitch = normalize_angle(
        np.degrees(pitch_radian) * PITCH_SIGN
    )

    yaw = normalize_angle(
        np.degrees(yaw_radian) * YAW_SIGN
    )

    roll = normalize_angle(
        np.degrees(roll_radian) * ROLL_SIGN
    )

    return pitch, yaw, roll


def get_face_vertical_measurements(landmarks):
    """정규화 랜드마크로 얼굴 중심 Y와 얼굴 높이를 계산한다."""
    y_values = [float(landmark.y) for landmark in landmarks]

    if not y_values:
        return None

    min_y = min(y_values)
    max_y = max(y_values)
    face_height = max_y - min_y

    if face_height <= 1e-6:
        return None

    face_center_y = (min_y + max_y) / 2.0

    return face_center_y, face_height


def get_recent_median_face_height(current_time):
    """최근 MEDIAN_WINDOW초의 화면 기준 얼굴 Y 이동 중앙값을 반환한다."""
    while (
        face_height_history
        and current_time - face_height_history[0][0] > MEDIAN_WINDOW
    ):
        face_height_history.popleft()

    if not face_height_history:
        return None

    values = [sample[1] for sample in face_height_history]
    return float(np.median(values))


def face_height_boundary(lower_step):
    """얼굴 높이 단계 사이의 기본 경계를 반환한다."""
    if lower_step < 0:
        return (
            -FACE_HEIGHT_DEADZONE
            + FACE_HEIGHT_PER_STEP * (lower_step + 1)
        )

    return (
        FACE_HEIGHT_DEADZONE
        + FACE_HEIGHT_PER_STEP * lower_step
    )


def calculate_face_height_step_with_hysteresis(
    height_change,
    confirmed_step
):
    """얼굴 높이 변화 비율에 히스테리시스를 적용한다."""
    new_step = confirmed_step

    while new_step < FACE_HEIGHT_MAX_STEP:
        boundary = face_height_boundary(new_step)

        if height_change > boundary + FACE_HEIGHT_HYSTERESIS:
            new_step += 1
        else:
            break

    while new_step > -FACE_HEIGHT_MAX_STEP:
        boundary = face_height_boundary(new_step - 1)

        if height_change < boundary - FACE_HEIGHT_HYSTERESIS:
            new_step -= 1
        else:
            break

    return new_step


# ============================================================
# 14. 얼굴 각도 중앙값
# ============================================================

def get_recent_median_angles(current_time):
    """최근 MEDIAN_WINDOW초의 상대각도 중앙값을 반환한다."""
    while (
        angle_history
        and current_time - angle_history[0][0] > MEDIAN_WINDOW
    ):
        angle_history.popleft()

    if not angle_history:
        return None

    pitch_values = [sample[1] for sample in angle_history]
    yaw_values = [sample[2] for sample in angle_history]
    roll_values = [sample[3] for sample in angle_history]

    return (
        float(np.median(pitch_values)),
        float(np.median(yaw_values)),
        float(np.median(roll_values))
    )


# ============================================================
# 15. 단계 처리
# ============================================================

def step_boundary(lower_step):
    """
    lower_step과 lower_step + 1 사이의 기본 경계를 반환한다.

    -2 ↔ -1 : -25도
    -1 ↔  0 : -10도
     0 ↔ +1 : +10도
    +1 ↔ +2 : +25도
    """
    if lower_step < 0:
        return (
            -CENTER_ANGLE
            + STEP_ANGLE * (lower_step + 1)
        )

    return (
        CENTER_ANGLE
        + STEP_ANGLE * lower_step
    )


def calculate_step_with_hysteresis(angle, confirmed_step):
    """히스테리시스를 적용해 후보 단계를 계산한다."""
    new_step = confirmed_step

    while new_step < MAX_STEP:
        boundary = step_boundary(new_step)

        if angle > boundary + HYSTERESIS:
            new_step += 1
        else:
            break

    while new_step > -MAX_STEP:
        boundary = step_boundary(new_step - 1)

        if angle < boundary - HYSTERESIS:
            new_step -= 1
        else:
            break

    return new_step


def update_axis_state(axis_name, new_candidate, current_time):
    """후보 단계가 STEP_HOLD_TIME 동안 유지되면 확정한다."""
    state = axis_states[axis_name]

    confirmed = state["confirmed"]
    previous_candidate = state["candidate"]

    if new_candidate == confirmed:
        state["candidate"] = confirmed
        state["candidate_since"] = None
        return False

    if new_candidate != previous_candidate:
        state["candidate"] = new_candidate
        state["candidate_since"] = current_time
        return False

    candidate_since = state["candidate_since"]

    if candidate_since is None:
        state["candidate_since"] = current_time
        return False

    if current_time - candidate_since >= STEP_HOLD_TIME:
        state["confirmed"] = new_candidate
        state["candidate"] = new_candidate
        state["candidate_since"] = None
        return True

    return False


def reset_all_steps():
    """단계 상태만 0으로 초기화한다."""
    for state in axis_states.values():
        state["confirmed"] = 0
        state["candidate"] = 0
        state["candidate_since"] = None


def clear_step_candidates():
    """확정 단계는 유지하고 진행 중인 후보만 취소한다."""
    for state in axis_states.values():
        state["candidate"] = state["confirmed"]
        state["candidate_since"] = None


def reset_posture_transition_detection(reset_last_visible=True):
    """진행 중인 sitting/standing 화면 이탈 후보를 초기화한다."""
    global posture_face_lost_candidate
    global posture_face_lost_since
    global last_visible_height_step
    global standing_exit_armed

    posture_face_lost_candidate = None
    posture_face_lost_since = None
    standing_exit_armed = False

    if reset_last_visible:
        last_visible_height_step = 0




def reset_face_control_after_motion():

    """
    로봇 모션 완료 후 움직이는 카메라 기준 얼굴 제어 상태를 초기화한다.

    모션 중 수집된 yaw/pitch/roll 표본과 후보 상태가 다음 명령에
    섞이지 않도록 한다. 얼굴 원점 보정값 자체는 유지한다.
    """
    angle_history.clear()
    face_height_history.clear()

    for axis_name in ("yaw", "pitch", "roll", "height"):
        state = axis_states[axis_name]
        state["confirmed"] = 0
        state["candidate"] = 0
        state["candidate_since"] = None

    # 카메라 위치가 바뀌었으므로 이전 화면 이탈 힌트도 폐기한다.
    reset_posture_transition_detection(reset_last_visible=True)


def format_finger_yaw_target():
    """화면/로그용 검지 직접 yaw 목표 문자열."""
    if finger_yaw_target is None:
        return "FACE"
    return f"{int(finger_yaw_target):+d}"


def print_step_signal(control_pitch, control_yaw, median_roll):
    """얼굴 단계와 현재 검지 직접 yaw 목표를 출력한다."""
    face_pitch_step = axis_states["pitch"]["confirmed"]
    face_yaw_step = axis_states["yaw"]["confirmed"]
    roll_step = axis_states["roll"]["confirmed"]
    face_height_step = axis_states["height"]["confirmed"]

    final_pitch_step = int(
        np.clip(
            face_pitch_step + pitch_cal,
            -MAX_STEP,
            MAX_STEP
        )
    )

    print(
        f"Yaw:{face_yaw_step:+d} "
        f"(face:{face_yaw_step:+d}, finger:{format_finger_yaw_target()}) | "
        f"Pitch:{final_pitch_step:+d} "
        f"(face:{face_pitch_step:+d}, cal:{pitch_cal:+d}) | "
        f"Roll:{roll_step:+d} | "
        f"FaceHeight:{face_height_step:+d} | "
        f"m_dist:{m_dist:+d}"
    )


# ============================================================
# 16. 얼굴 원점 보정
# ============================================================
# 16. 얼굴 원점 보정
# ============================================================

def start_calibration(current_time):
    """개인별 얼굴 원점 보정을 시작한다."""
    global calibration_active
    global calibration_start_time
    global calibration_samples

    calibration_active = True
    calibration_start_time = current_time
    calibration_samples = []

    angle_history.clear()
    face_height_history.clear()
    clear_step_candidates()

    print(
        "CALIBRATING... "
        "Keep looking naturally at the screen."
    )


def cancel_calibration():
    """보정 중 얼굴을 잃으면 현재 보정을 취소한다."""
    global calibration_active
    global calibration_start_time
    global calibration_samples

    calibration_active = False
    calibration_start_time = None
    calibration_samples = []
    face_height_history.clear()

    print(
        "CALIBRATION FAILED | NO FACE | "
        "PREVIOUS ORIGIN RETAINED"
    )


def complete_calibration():
    """1초 동안 수집한 원시 각도로 새 얼굴 원점을 설정한다."""
    global calibrated
    global calibration_active
    global calibration_start_time
    global calibration_samples

    if not calibration_samples:
        calibration_active = False
        calibration_start_time = None
        calibration_samples = []

        print("CALIBRATION FAILED | NO SAMPLES")
        return

    pitch_values = [sample[0] for sample in calibration_samples]
    yaw_values = [sample[1] for sample in calibration_samples]
    roll_values = [sample[2] for sample in calibration_samples]
    center_y_values = [sample[3] for sample in calibration_samples]
    face_height_values = [sample[4] for sample in calibration_samples]

    angle_origin["pitch"] = circular_mean_degrees(
        pitch_values
    )

    angle_origin["yaw"] = circular_mean_degrees(
        yaw_values
    )

    angle_origin["roll"] = circular_mean_degrees(
        roll_values
    )

    face_height_origin["center_y"] = float(
        np.median(center_y_values)
    )
    face_height_origin["height"] = max(
        float(np.median(face_height_values)),
        1e-6
    )

    calibrated = True

    calibration_active = False
    calibration_start_time = None
    calibration_samples = []

    angle_history.clear()
    face_height_history.clear()
    reset_all_steps()

    print("CALIBRATION COMPLETE")
    print(
        f"Origin | "
        f"Yaw:{angle_origin['yaw']:+.2f} deg | "
        f"Pitch:{angle_origin['pitch']:+.2f} deg | "
        f"Roll:{angle_origin['roll']:+.2f} deg"
    )
    print(
        f"Face screen origin | "
        f"CenterY:{face_height_origin['center_y']:.4f} | "
        f"DetectedHeight:{face_height_origin['height']:.4f}"
    )
    print(
        f"Control | "
        f"FingerYaw:{format_finger_yaw_target()} | "
        f"PitchCal:{pitch_cal:+.2f} deg"
    )


# ============================================================
# 17. 검지 직접 yaw 목표 조정
# ============================================================

def is_index_only_gesture(fingers):
    """
    엄지 상태를 무시하고,
    검지만 펴고 중지·약지·새끼손가락은 접힌 자세인지 확인한다.
    """
    return fingers[1:] == [1, 0, 0, 0]


def is_three_finger_calibration_gesture(fingers):
    """
    엄지 상태를 무시하고,
    검지·중지·약지는 펴고 새끼손가락은 접힌 자세인지 확인한다.
    """
    return fingers[1:] == [1, 1, 1, 0]


def is_four_finger_quit_gesture(fingers):
    """
    엄지 상태를 무시하고,
    검지·중지·약지·새끼손가락을 모두 편 자세인지 확인한다.
    """
    return fingers[1:] == [1, 1, 1, 1]


def set_finger_yaw_target(target):
    """검지 직접 yaw 목표를 설정한다. None이면 얼굴 추적으로 복귀한다."""
    global finger_yaw_target
    global finger_center_since
    global finger_face_reacquire_requested

    normalized = None if target is None else int(target)
    if normalized not in (None, FINGER_YAW_LEFT_TARGET, FINGER_YAW_RIGHT_TARGET):
        raise ValueError(f"invalid finger yaw target: {target!r}")

    if finger_yaw_target == normalized:
        return False

    previous = finger_yaw_target
    finger_yaw_target = normalized
    finger_center_since = None

    if normalized is None:
        # 아직 시작되지 않은 좌/우 직접 목표가 controller에 남아 있다면 즉시 취소한다.
        # 이미 robot.play()가 시작된 경우 update_yaw_error(0)은 busy 때문에 무시되고,
        # 현재 play만 끝난 뒤 새 얼굴 추적으로 복귀한다.
        try:
            state = motion_controller.snapshot()
            if (
                state.phase == "TRACKING"
                and state.tracking_enabled
                and state.current_posture == "sitting"
                and state.target_posture == state.current_posture
            ):
                motion_controller.update_yaw_error(0)
        except NameError:
            # 함수 정의 시점에는 controller가 아직 만들어지지 않았지만, 실제 호출은
            # 메인 루프 시작 뒤에만 일어난다. 방어적으로만 남겨 둔다.
            pass

        # 좌/우 강제 제어 동안의 얼굴 history는 사용하지 않는다.
        # 다음 메인 루프에서 모두 버리고 fresh face 확인을 시작한다.
        finger_face_reacquire_requested = True
        print(
            "FINGER YAW | CENTER -> FACE TRACKING | "
            f"previous:{'FACE' if previous is None else f'{int(previous):+d}'}"
        )
    else:
        side = "LEFT" if normalized < 0 else "RIGHT"
        print(
            f"FINGER YAW | {side} -> ABS TARGET {normalized:+d}"
        )

    return True


def start_offset_adjustment(finger_position, current_time):
    """현재 검지 위치를 기준점으로 저장하고 새 한-칸 제스처를 시작한다."""
    global offset_adjusting
    global offset_start_finger
    global index_gesture_last_seen
    global finger_center_since
    global finger_yaw_step_applied

    offset_adjusting = True
    offset_start_finger = (
        float(finger_position[0]),
        float(finger_position[1])
    )
    index_gesture_last_seen = current_time
    finger_center_since = None
    finger_yaw_step_applied = False

    print(
        "FINGER YAW DIRECT START | "
        f"Saved:{format_finger_yaw_target()} | one change allowed"
    )


def _finger_saved_value():
    """저장 상태를 계산용 -1/0/+1 정수로 반환한다. None은 0이다."""
    if finger_yaw_target is None:
        return 0
    return int(finger_yaw_target)


def _set_finger_saved_value(value):
    """-1/0/+1 저장값을 기존 직접 제어 표현으로 적용한다."""
    value = int(np.clip(int(value), -1, 1))
    target = None if value == 0 else value
    set_finger_yaw_target(target)


def update_offset_adjustment(
    finger_position,
    frame_width,
    frame_height,
    current_time
):
    """
    검지 한 번을 들었을 때 저장된 yaw 값을 최대 한 칸만 바꾼다.

    시작점 기준:
      왼쪽  100px 이상 -> saved = max(-1, saved - 1)
      오른쪽 100px 이상 -> saved = min(+1, saved + 1)

    예:
      saved=-1에서 오른쪽 -> 0까지만 변경, +1로 계속 넘어가지 않음
      saved=+1에서 왼쪽   -> 0까지만 변경, -1로 계속 넘어가지 않음

    한 번 변경된 뒤에는 같은 검지 제스처가 끝날 때까지 추가 변화가 없다.
    frame_width/frame_height 인자는 기존 호출부 호환을 위해 유지한다.
    """
    global index_gesture_last_seen
    global finger_center_since
    global finger_yaw_step_applied

    index_gesture_last_seen = current_time

    # 이번에 검지를 든 뒤 이미 한 번 저장값을 바꿨다면,
    # 손가락을 내릴 때까지 추가 좌/우 입력을 전부 무시한다.
    if finger_yaw_step_applied:
        return

    current_x = float(finger_position[0])
    start_x = float(offset_start_finger[0])
    delta_x = current_x - start_x
    enter = YAW_GESTURE_ENTER_PX

    previous_value = _finger_saved_value()

    if delta_x <= -enter:
        new_value = max(-1, previous_value - 1)
        _set_finger_saved_value(new_value)
        finger_yaw_step_applied = True
        finger_center_since = None
        print(
            "FINGER SAVED STEP | LEFT | "
            f"{previous_value:+d} -> {new_value:+d}"
        )
        return

    if delta_x >= enter:
        new_value = min(1, previous_value + 1)
        _set_finger_saved_value(new_value)
        finger_yaw_step_applied = True
        finger_center_since = None
        print(
            "FINGER SAVED STEP | RIGHT | "
            f"{previous_value:+d} -> {new_value:+d}"
        )
        return

    finger_center_since = None


def finish_offset_adjustment():
    """검지 제스처를 종료한다. 이번에 바뀐 저장값은 그대로 유지한다."""
    global offset_adjusting
    global offset_start_finger
    global index_gesture_since
    global index_gesture_last_seen
    global finger_center_since
    global finger_yaw_step_applied

    offset_adjusting = False
    offset_start_finger = None
    index_gesture_since = None
    index_gesture_last_seen = None
    finger_center_since = None
    finger_yaw_step_applied = False

    print(
        "FINGER YAW DIRECT GESTURE END | "
        f"Target:{format_finger_yaw_target()} | "
        f"m_dist:{m_dist:+d}"
    )

def start_m_dist_adjustment(point1, point2, current_time):
    """두 검지 사이 거리 기반 m_dist 조정을 시작한다."""
    global m_dist_adjusting
    global m_dist_start_distance
    global m_dist_start_value
    global dual_gesture_last_seen

    m_dist_adjusting = True
    m_dist_start_distance = float(
        np.hypot(
            point2[0] - point1[0],
            point2[1] - point1[1]
        )
    )
    m_dist_start_value = m_dist
    dual_gesture_last_seen = current_time

    print(
        "M_DIST ADJUST START | "
        f"m_dist:{m_dist:+d}"
    )


def update_m_dist_adjustment(
    point1,
    point2,
    frame_width,
    current_time
):
    """두 검지 사이 거리 변화로 m_dist를 갱신한다."""
    global m_dist
    global dual_gesture_last_seen

    dual_gesture_last_seen = current_time

    current_distance = float(
        np.hypot(
            point2[0] - point1[0],
            point2[1] - point1[1]
        )
    )

    distance_step = (
        frame_width * M_DIST_STEP_RATIO
    )

    m_dist_delta = int(
        np.trunc(
            (
                current_distance
                - m_dist_start_distance
            )
            / distance_step
        )
    )

    m_dist = int(
        np.clip(
            m_dist_start_value + m_dist_delta,
            M_DIST_MIN,
            M_DIST_MAX
        )
    )


def finish_m_dist_adjustment():
    """m_dist 값을 확정하고 두 손 조정 상태를 종료한다."""
    global m_dist_adjusting
    global m_dist_start_distance
    global dual_gesture_since
    global dual_gesture_last_seen

    m_dist_adjusting = False
    m_dist_start_distance = None
    dual_gesture_since = None
    dual_gesture_last_seen = None

    print(
        "M_DIST SAVED | "
        f"m_dist:{m_dist:+d}"
    )


def cancel_offset_adjustment():
    """현재 검지 제스처 추적만 취소한다. 저장된 yaw 상태는 유지한다."""
    global offset_adjusting
    global offset_start_finger
    global index_gesture_since
    global index_gesture_last_seen
    global finger_center_since
    global finger_yaw_step_applied

    offset_adjusting = False
    offset_start_finger = None
    index_gesture_since = None
    index_gesture_last_seen = None
    finger_center_since = None
    finger_yaw_step_applied = False

    print(
        "FINGER YAW DIRECT CANCELED | "
        f"Target:{format_finger_yaw_target()}"
    )


def reset_finger_yaw_override_and_gesture():
    """L/Q 전환 시 검지 직접 yaw 목표와 관련 제스처 상태를 삭제한다.

    3손가락으로 잡은 angle_origin / face_height_origin은 유지한다.
    """
    global pitch_cal
    global offset_adjusting
    global offset_start_finger
    global index_gesture_since
    global index_gesture_last_seen
    global finger_yaw_target
    global finger_yaw_step_applied
    global finger_center_since
    global finger_face_reacquire_requested

    pitch_cal = 0
    offset_adjusting = False
    offset_start_finger = None
    index_gesture_since = None
    index_gesture_last_seen = None
    finger_yaw_target = None
    finger_yaw_step_applied = False
    finger_center_since = None
    finger_face_reacquire_requested = False

    print("FINGER YAW OVERRIDE RESET | control=FACE")


# ============================================================
# 18. 화면 표시
# ============================================================

def draw_text(
    image,
    text,
    position,
    color=(255, 255, 255),
    scale=0.7,
    thickness=2
):
    if not SHOW_DISPLAY:
        return

    cv2.putText(
        image,
        text,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA
    )


def draw_face_landmarks(image, landmarks):
    """얼굴 랜드마크 일부를 표시한다."""
    if not SHOW_DISPLAY:
        return

    height, width = image.shape[:2]

    for index in range(0, len(landmarks), 5):
        landmark = landmarks[index]

        x = int(landmark.x * width)
        y = int(landmark.y * height)

        cv2.circle(
            image,
            (x, y),
            1,
            (0, 255, 0),
            cv2.FILLED
        )


# ============================================================
# 19. 모델 확인
# ============================================================

if not Path(MODEL_PATH).exists():
    raise FileNotFoundError(
        "face_landmarker.task 파일을 찾을 수 없다.\n"
        f"예상 위치: {MODEL_PATH}"
    )


# ============================================================
# 20. MediaPipe 설정
# ============================================================

face_options = FaceLandmarkerOptions(
    base_options=BaseOptions(
        model_asset_path=MODEL_PATH
    ),
    running_mode=RunningMode.VIDEO,
    num_faces=1,
    min_face_detection_confidence=0.6,
    min_face_presence_confidence=0.6,
    min_tracking_confidence=0.6,
    output_face_blendshapes=False,
    output_facial_transformation_matrixes=True
)


# ============================================================
# 21. 웹캠 설정
# ============================================================

# Windows에서 USB 카메라가 잘 열리지 않으면 CAP_DSHOW가 유용하다.
cap = cv2.VideoCapture(
    CAMERA_INDEX,
    cv2.CAP_V4L2
)

cap.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    FRAME_WIDTH
)

cap.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    FRAME_HEIGHT
)

if not cap.isOpened():
    raise RuntimeError(
        f"카메라 {CAMERA_INDEX}번을 열 수 없다."
    )


program_start_time = time.monotonic()

# MediaPipe VIDEO 모드에 전달하는 timestamp는 호출할 때마다
# 반드시 이전 값보다 커야 한다.
last_face_timestamp_ms = -1

hand_frame_count = 0
cached_hands = []
last_hand_seen_time = None

# sitting / standing / lying / zoom 전체 상태를 제어하는 모션 제어기다.
# 생성 직후 별도 worker thread가 phorce.connect()로 연결하고
# 환영 시퀀스 19 -> 20 -> 20 -> 21을 먼저 실행한다.
# motion 21까지 끝난 뒤 WAIT_FOR_H에서 멈추며, 사용자가 H를 눌러야
# motion 1부터 기존 시작 절차가 진행된다. 실제 robot.play()는 모션 완료까지 blocking이다.
motion_controller = MotionController(
    dry_run=MOTION_DRY_RUN,
    settle_time=0.4,
    startup_settle_time=0.8,
    exit_settle_time=0.8,
)

# 모션 실행/안정화 중 표본을 다음 판단에 재사용하지 않는다.
previous_motion_locked = False

# busy/settling이 막 풀린 바로 그 프레임은 모션 중에 계산되던 값일 수 있으므로
# tracking 명령 전달을 한 프레임 더 건너뛴다. 이후 STEP_HOLD_TIME 동안
# 새 얼굴값이 유지되어야 실제 단계가 다시 확정된다.
skip_tracking_update_this_frame = False

# 모션 후 reset_face_control_after_motion()은 yaw confirmed를 0으로 만든다.
# 모션 후에는 일반 얼굴 yaw 추적을 곧바로 재개하지 않고, 얼굴이 연속으로
# POST_MOTION_REACQUIRE_TIME 동안 보인 뒤에만 다시 허용한다.
# 검지 직접 yaw 목표는 얼굴값과 독립적으로 별도 처리한다.
post_motion_reacquire_required = False
post_motion_face_since = None

# 3손가락 보정이 완료된 순간 TRACKING을 한 번만 켜기 위한 상태다.
tracking_started_from_calibration = False


# ============================================================
# 22. 메인 루프
# ============================================================

with FaceLandmarker.create_from_options(
    face_options
) as face_landmarker:

    while True:
        success, frame = cap.read()

        if not success:
            print("웹캠 프레임을 읽을 수 없다.")
            break

        current_time = time.monotonic()

        # 한 손 검지는 sitting에서만 직접 yaw 목표를 바꾼다.
        # 이미 시작된 yaw motion은 중단하지 않지만, motion 중에도 손가락 위치는 계속
        # 읽어서 목표가 반대로 바뀌면 현재 motion 완료 직후 새 목표를 따라가게 한다.
        # 두 손 m_dist 조정은 기존처럼 실제 motion/settling 중에는 새로 받지 않는다.
        gesture_motion_state = motion_controller.snapshot()
        single_index_control_enabled = (
            gesture_motion_state.phase == "TRACKING"
            and gesture_motion_state.tracking_enabled
            and not gesture_motion_state.shutdown_requested
            and gesture_motion_state.current_posture == "sitting"
            and gesture_motion_state.target_posture
                == gesture_motion_state.current_posture
        )
        dual_index_control_enabled = (
            gesture_motion_state.phase == "TRACKING"
            and gesture_motion_state.tracking_enabled
            and not gesture_motion_state.busy
            and not gesture_motion_state.settling
            and not gesture_motion_state.shutdown_requested
            and not (
                gesture_motion_state.current_posture == "lying"
                and gesture_motion_state.lying_zoom
            )
        )
        index_input_enabled = (
            single_index_control_enabled or dual_index_control_enabled
        )

        if MIRROR_IMAGE:
            frame = cv2.flip(frame, 1)

        frame_height, frame_width = frame.shape[:2]

        # 얼굴 인식에는 손 랜드마크나 글자가 그려지지 않은
        # 깨끗한 원본 프레임을 사용한다.
        face_input_frame = frame.copy()

        # ====================================================
        # 손 인식
        # ====================================================

        hand_frame_count += 1

        hand_recently_seen = (
            last_hand_seen_time is not None
            and current_time - last_hand_seen_time
                <= HAND_LOST_GRACE_TIME
        )

        if hand_recently_seen:
            hand_process_interval = HAND_TRACK_INTERVAL
        else:
            hand_process_interval = HAND_SEARCH_INTERVAL

        should_process_hands = (
            hand_frame_count == 1
            or hand_frame_count % hand_process_interval == 0
        )

        if should_process_hands:
            detected_hands, frame = hand_detector.findHands(
                frame,
                draw=SHOW_DISPLAY
            )

            if detected_hands:
                hands = detected_hands
                cached_hands = detected_hands
                last_hand_seen_time = current_time

            elif hand_recently_seen:
                # 얼굴과 겹치는 등의 이유로 한두 번 놓치더라도
                # 짧은 유예시간 동안 직전 손 결과를 유지한다.
                hands = cached_hands

            else:
                hands = []
                cached_hands = []

        else:
            if hand_recently_seen:
                hands = cached_hands
            else:
                hands = []
                cached_hands = []

        index_gesture_detected = False
        index_finger_position = None
        index_finger_positions = []
        four_finger_gesture_count = 0
        three_finger_gesture_count = 0
        three_finger_calibration_requested = False
        four_finger_quit_requested = False

        if hands:
            for hand in hands:
                lm_list = hand["lmList"]
                fingers = hand_detector.fingersUp(hand)

                if is_four_finger_quit_gesture(fingers):
                    four_finger_gesture_count += 1

                elif is_three_finger_calibration_gesture(fingers):
                    three_finger_gesture_count += 1

                elif is_index_only_gesture(fingers):
                    if index_input_enabled:
                        index_finger_positions.append(
                            lm_list[8][:2]
                        )

        index_count = len(index_finger_positions)

        # 네 손가락 종료 제스처가 가장 높은 우선순위다.
        if (
            len(hands) == 1
            and four_finger_gesture_count == 1
        ):
            if four_finger_since is None:
                four_finger_since = current_time

            elif (
                current_time - four_finger_since
                >= FOUR_FINGER_QUIT_HOLD_TIME
            ):
                four_finger_quit_requested = True

            three_finger_since = None
            three_finger_latched = False
            index_gesture_since = None
            index_gesture_last_seen = None
            dual_gesture_since = None
            dual_gesture_last_seen = None

        else:
            four_finger_since = None

            # 세 손가락 영점 보정 제스처가 다음 우선순위다.
            if (
                len(hands) == 1
                and three_finger_gesture_count == 1
            ):
                if offset_adjusting:
                    finish_offset_adjustment()

                if m_dist_adjusting:
                    finish_m_dist_adjustment()

                index_gesture_since = None
                index_gesture_last_seen = None
                dual_gesture_since = None
                dual_gesture_last_seen = None

                if not three_finger_latched:
                    if three_finger_since is None:
                        three_finger_since = current_time

                    elif (
                        current_time - three_finger_since
                        >= THREE_FINGER_CALIBRATION_HOLD_TIME
                    ):
                        three_finger_calibration_requested = True
                        three_finger_latched = True

            else:
                three_finger_since = None
                three_finger_latched = False

                # 두 손 검지 제스처가 다음 우선순위다.
                if index_count >= 2 and dual_index_control_enabled:
                    point1 = index_finger_positions[0]
                    point2 = index_finger_positions[1]

                    if offset_adjusting:
                        finish_offset_adjustment()

                    index_gesture_since = None
                    index_gesture_last_seen = None
                    dual_gesture_last_seen = current_time

                    if not m_dist_adjusting:
                        if dual_gesture_since is None:
                            dual_gesture_since = current_time

                        elif (
                            current_time - dual_gesture_since
                            >= OFFSET_GESTURE_START_TIME
                        ):
                            start_m_dist_adjustment(
                                point1,
                                point2,
                                current_time
                            )

                    else:
                        update_m_dist_adjustment(
                            point1,
                            point2,
                            frame_width,
                            current_time
                        )

                elif index_count == 1 and single_index_control_enabled:
                    index_gesture_detected = True
                    index_finger_position = index_finger_positions[0]

                    if m_dist_adjusting:
                        if (
                            dual_gesture_last_seen is not None
                            and current_time - dual_gesture_last_seen
                            >= OFFSET_GESTURE_END_TIME
                        ):
                            finish_m_dist_adjustment()

                    else:
                        dual_gesture_since = None
                        index_gesture_last_seen = current_time

                        if not offset_adjusting:
                            if index_gesture_since is None:
                                index_gesture_since = current_time

                            elif (
                                current_time - index_gesture_since
                                >= OFFSET_GESTURE_START_TIME
                            ):
                                start_offset_adjustment(
                                    index_finger_position,
                                    current_time
                                )

                        else:
                            update_offset_adjustment(
                                index_finger_position,
                                frame_width,
                                frame_height,
                                current_time
                            )

                else:
                    if not offset_adjusting:
                        index_gesture_since = None
                    elif (
                        index_gesture_last_seen is not None
                        and current_time - index_gesture_last_seen
                        >= OFFSET_GESTURE_END_TIME
                    ):
                        finish_offset_adjustment()

                    if not m_dist_adjusting:
                        dual_gesture_since = None
                    elif (
                        dual_gesture_last_seen is not None
                        and current_time - dual_gesture_last_seen
                        >= OFFSET_GESTURE_END_TIME
                    ):
                        finish_m_dist_adjustment()

        # 검지가 중앙으로 복귀해 직접 yaw 강제가 해제된 순간에는
        # 강제 제어 중 수집된 얼굴 history를 버리고 fresh face를 다시 확인한다.
        if finger_face_reacquire_requested:
            reset_face_control_after_motion()
            post_motion_reacquire_required = True
            post_motion_face_since = None
            finger_face_reacquire_requested = False
            print(
                "FINGER YAW RELEASE | face history reset | "
                f"reacquire {POST_MOTION_REACQUIRE_TIME:.2f}s"
            )

        # ====================================================
        # 얼굴 인식
        # ====================================================

        raw_timestamp_ms = int(
            (
                current_time
                - program_start_time
            ) * 1000
        )

        # int 변환으로 두 프레임이 같은 밀리초가 되는 경우가 있으므로,
        # 이전 timestamp보다 최소 1ms 크게 보장한다.
        timestamp_ms = max(
            raw_timestamp_ms,
            last_face_timestamp_ms + 1
        )
        last_face_timestamp_ms = timestamp_ms

        rgb_frame = cv2.cvtColor(
            face_input_frame,
            cv2.COLOR_BGR2RGB
        )

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb_frame
        )

        face_result = face_landmarker.detect_for_video(
            mp_image,
            timestamp_ms
        )

        face_detected = False
        raw_angles_available = False

        raw_pitch = 0.0
        raw_yaw = 0.0
        raw_roll = 0.0

        if (
            len(face_result.face_landmarks) > 0
            and len(
                face_result.facial_transformation_matrixes
            ) > 0
        ):
            transformation_matrix = np.array(
                face_result.facial_transformation_matrixes[0],
                dtype=np.float64
            )

            if transformation_matrix.shape == (4, 4):
                face_detected = True
                raw_angles_available = True

                last_face_seen_time = current_time
                no_face_printed = False
                no_face_timeout_printed = False

                face_landmarks = face_result.face_landmarks[0]

                draw_face_landmarks(
                    frame,
                    face_landmarks
                )

                face_vertical = get_face_vertical_measurements(
                    face_landmarks
                )

                current_face_center_y = None
                current_face_height = None

                if face_vertical is not None:
                    (
                        current_face_center_y,
                        current_face_height
                    ) = face_vertical

                rotation_matrix = (
                    transformation_matrix[:3, :3]
                )

                (
                    raw_pitch,
                    raw_yaw,
                    raw_roll
                ) = rotation_matrix_to_euler(
                    rotation_matrix
                )

                # ============================================
                # 얼굴 원점 보정 중
                # ============================================

                if calibration_active:
                    if (
                        current_face_center_y is not None
                        and current_face_height is not None
                    ):
                        calibration_samples.append(
                            (
                                raw_pitch,
                                raw_yaw,
                                raw_roll,
                                current_face_center_y,
                                current_face_height
                            )
                        )

                    calibration_elapsed = (
                        current_time
                        - calibration_start_time
                    )

                    remaining = max(
                        0.0,
                        CALIBRATION_TIME
                        - calibration_elapsed
                    )

                    draw_text(
                        frame,
                        f"CALIBRATING... {remaining:.1f}s",
                        (30, 40),
                        (0, 255, 255),
                        0.9,
                        2
                    )

                    if (
                        calibration_elapsed
                        >= CALIBRATION_TIME
                    ):
                        complete_calibration()

                # ============================================
                # 얼굴 원점 보정 완료
                # ============================================

                elif calibrated:
                    relative_pitch = angle_difference(
                        raw_pitch,
                        angle_origin["pitch"]
                    )

                    relative_yaw = angle_difference(
                        raw_yaw,
                        angle_origin["yaw"]
                    )

                    relative_roll = angle_difference(
                        raw_roll,
                        angle_origin["roll"]
                    )

                    angle_history.append(
                        (
                            current_time,
                            relative_pitch,
                            relative_yaw,
                            relative_roll
                        )
                    )

                    if current_face_center_y is not None:
                        # MediaPipe의 Y 좌표는 화면 높이 전체가 1.0인
                        # 정규화 좌표다. 따라서 0.20은 화면 높이의 20%다.
                        # 얼굴이 화면 위로 올라가면 양수,
                        # 아래로 내려가면 음수가 된다.
                        face_height_change = (
                            face_height_origin["center_y"]
                            - current_face_center_y
                        )

                        face_height_history.append(
                            (
                                current_time,
                                face_height_change
                            )
                        )

                    median_result = get_recent_median_angles(
                        current_time
                    )

                    if median_result is not None:
                        (
                            median_pitch,
                            median_yaw,
                            median_roll
                        ) = median_result

                        # 얼굴 각도는 기존 그대로 사용한다.
                        control_yaw = median_yaw
                        control_pitch = median_pitch
                        control_roll = median_roll

                        pitch_candidate = (
                            calculate_step_with_hysteresis(
                                control_pitch,
                                axis_states[
                                    "pitch"
                                ]["confirmed"]
                            )
                        )

                        yaw_candidate = (
                            calculate_step_with_hysteresis(
                                control_yaw,
                                axis_states[
                                    "yaw"
                                ]["confirmed"]
                            )
                        )

                        roll_candidate = (
                            calculate_step_with_hysteresis(
                                control_roll,
                                axis_states[
                                    "roll"
                                ]["confirmed"]
                            )
                        )

                        pitch_changed = update_axis_state(
                            "pitch",
                            pitch_candidate,
                            current_time
                        )

                        yaw_changed = update_axis_state(
                            "yaw",
                            yaw_candidate,
                            current_time
                        )

                        roll_changed = update_axis_state(
                            "roll",
                            roll_candidate,
                            current_time
                        )

                        median_face_height_change = (
                            get_recent_median_face_height(
                                current_time
                            )
                        )

                        height_changed = False

                        if median_face_height_change is not None:
                            height_candidate = (
                                calculate_face_height_step_with_hysteresis(
                                    median_face_height_change,
                                    axis_states[
                                        "height"
                                    ]["confirmed"]
                                )
                            )

                            height_changed = update_axis_state(
                                "height",
                                height_candidate,
                                current_time
                            )

                        if (
                            pitch_changed
                            or yaw_changed
                            or roll_changed
                            or height_changed
                        ):
                            print_step_signal(
                                control_pitch,
                                control_yaw,
                                control_roll
                            )

                        face_pitch_step = (
                            axis_states[
                                "pitch"
                            ]["confirmed"]
                        )

                        face_yaw_step = (
                            axis_states[
                                "yaw"
                            ]["confirmed"]
                        )

                        roll_step = (
                            axis_states[
                                "roll"
                            ]["confirmed"]
                        )

                        pitch_step = int(
                            np.clip(
                                face_pitch_step + pitch_cal,
                                -MAX_STEP,
                                MAX_STEP
                            )
                        )

                        yaw_step = int(
                            np.clip(
                                face_yaw_step,
                                -MAX_STEP,
                                MAX_STEP
                            )
                        )

                        draw_text(
                            frame,
                            (
                                f"Yaw ctrl: "
                                f"{control_yaw:+.1f}  "
                                f"Step: {yaw_step:+d}"
                            ),
                            (30, 40),
                            (0, 255, 255),
                            0.8,
                            2
                        )

                        draw_text(
                            frame,
                            (
                                f"Pitch ctrl: "
                                f"{control_pitch:+.1f}  "
                                f"Step: {pitch_step:+d}"
                            ),
                            (30, 75),
                            (0, 255, 255),
                            0.8,
                            2
                        )

                        draw_text(
                            frame,
                            (
                                f"Roll: "
                                f"{control_roll:+.1f}  "
                                f"Step: {roll_step:+d}"
                            ),
                            (30, 110),
                            (0, 255, 255),
                            0.8,
                            2
                        )

                        face_height_step = (
                            axis_states[
                                "height"
                            ]["confirmed"]
                        )

                        if median_face_height_change is None:
                            face_height_display = "N/A"
                        else:
                            face_height_display = (
                                f"{median_face_height_change * 100:+.1f}%"
                            )

                        draw_text(
                            frame,
                            (
                                f"Face screen Y: "
                                f"{face_height_display}  "
                                f"Step: {face_height_step:+d}"
                            ),
                            (30, 145),
                            (0, 255, 0),
                            0.7,
                            2
                        )

                        draw_text(
                            frame,
                            (
                                f"Face rel Y/P: "
                                f"{median_yaw:+.1f}, "
                                f"{median_pitch:+.1f}"
                            ),
                            (30, 180),
                            (255, 255, 255),
                            0.55,
                            1
                        )

                        draw_text(
                            frame,
                            (
                                f"FingerYaw: {format_finger_yaw_target()}  "
                                f"PitchCal: {pitch_cal:+d}  "
                                f"Mdist: {m_dist:+d}"
                            ),
                            (30, 210),
                            (255, 255, 0),
                            0.65,
                            2
                        )

                # ============================================
                # 얼굴 원점 보정 전
                # ============================================

                else:
                    draw_text(
                        frame,
                        "NOT CALIBRATED",
                        (30, 40),
                        (0, 0, 255),
                        0.9,
                        2
                    )

                    draw_text(
                        frame,
                        "Look naturally and hold 3 fingers",
                        (30, 75),
                        (0, 255, 255),
                        0.7,
                        2
                    )

                    draw_text(
                        frame,
                        (
                            f"RAW Y/P/R: "
                            f"{raw_yaw:+.1f}, "
                            f"{raw_pitch:+.1f}, "
                            f"{raw_roll:+.1f}"
                        ),
                        (30, 110),
                        (255, 255, 255),
                        0.6,
                        1
                    )

        # ====================================================
        # 얼굴 미검출
        # ====================================================

        if not face_detected:
            angle_history.clear()
            face_height_history.clear()
            clear_step_candidates()

            if calibration_active:
                cancel_calibration()

            if not no_face_printed:
                print(
                    "NO FACE | holding "
                    f"Yaw:"
                    f"{axis_states['yaw']['confirmed']:+d} "
                    f"Pitch:"
                    f"{axis_states['pitch']['confirmed']:+d} "
                    f"Roll:"
                    f"{axis_states['roll']['confirmed']:+d} "
                    f"FaceHeight:"
                    f"{axis_states['height']['confirmed']:+d}"
                )

                no_face_printed = True

            draw_text(
                frame,
                "NO FACE",
                (30, 45),
                (0, 0, 255),
                1.0,
                3
            )

            if calibrated:
                draw_text(
                    frame,
                    "Face origin retained",
                    (30, 85),
                    (0, 255, 255),
                    0.7,
                    2
                )

            if last_face_seen_time is not None:
                no_face_duration = (
                    current_time
                    - last_face_seen_time
                )

                remaining_time = max(
                    0.0,
                    NO_FACE_HOLD_TIME
                    - no_face_duration
                )

                draw_text(
                    frame,
                    (
                        f"Step hold remaining: "
                        f"{remaining_time:.1f}s"
                    ),
                    (30, 120),
                    (0, 0, 255),
                    0.65,
                    2
                )

                if (
                    no_face_duration
                    >= NO_FACE_HOLD_TIME
                    and not no_face_timeout_printed
                ):
                    reset_all_steps()

                    print(
                        "NO FACE TIMEOUT | "
                        "Yaw:+0 Pitch:+0 Roll:+0 | "
                        "FACE ORIGIN / FINGER TARGET RETAINED"
                    )

                    no_face_timeout_printed = True

        # 세 손가락 영점 보정은 환영 시퀀스 완료 -> H -> motion 1 완료 +
        # settling 이후의 WAIT_FOR_ZERO 상태에서만 시작한다. TRACKING 중 재보정은 막는다.
        if three_finger_calibration_requested:
            controller_state_for_zero = motion_controller.snapshot()

            if controller_state_for_zero.phase != "WAIT_FOR_ZERO":
                print(
                    "CALIBRATION IGNORED | "
                    f"controller phase={controller_state_for_zero.phase}"
                )
            elif controller_state_for_zero.busy or controller_state_for_zero.settling:
                print("CALIBRATION IGNORED | MOTION LOCKED")
            elif face_detected and raw_angles_available:
                if not calibration_active:
                    start_calibration(current_time)
            else:
                print(
                    "CALIBRATION CANNOT START | NO FACE"
                )

        # ====================================================
        # 검지 직접 yaw 상태 화면 표시
        # ====================================================

        if offset_adjusting:
            draw_text(
                frame,
                "FINGER YAW DIRECT",
                (frame_width - 330, 40),
                (0, 165, 255),
                0.8,
                2
            )

            draw_text(
                frame,
                f"Target: {format_finger_yaw_target()}",
                (frame_width - 330, 75),
                (0, 165, 255),
                0.7,
                2
            )

            draw_text(
                frame,
                f"PitchCal: {pitch_cal:+d}",
                (frame_width - 330, 110),
                (0, 165, 255),
                0.7,
                2
            )

            if (
                SHOW_DISPLAY
                and index_finger_position is not None
            ):
                current_point = (
                    int(index_finger_position[0]),
                    int(index_finger_position[1])
                )

                start_point = (
                    int(offset_start_finger[0]),
                    int(offset_start_finger[1])
                )

                cv2.circle(
                    frame,
                    start_point,
                    10,
                    (255, 0, 255),
                    2
                )

                cv2.line(
                    frame,
                    start_point,
                    current_point,
                    (255, 0, 255),
                    2
                )

        if m_dist_adjusting:
            draw_text(
                frame,
                "M_DIST ADJUSTING",
                (frame_width - 330, 145),
                (255, 0, 255),
                0.75,
                2
            )

            draw_text(
                frame,
                f"m_dist: {m_dist:+d}",
                (frame_width - 330, 180),
                (255, 0, 255),
                0.7,
                2
            )

            if (
                SHOW_DISPLAY
                and len(index_finger_positions) >= 2
            ):
                p1 = (
                    int(index_finger_positions[0][0]),
                    int(index_finger_positions[0][1])
                )
                p2 = (
                    int(index_finger_positions[1][0]),
                    int(index_finger_positions[1][1])
                )

                cv2.line(
                    frame,
                    p1,
                    p2,
                    (255, 0, 255),
                    2
                )

        elif (
            index_gesture_since is not None
            and index_gesture_detected
        ):
            hold_time = (
                current_time
                - index_gesture_since
            )

            draw_text(
                frame,
                (
                    f"Hold index finger: "
                    f"{min(hold_time, OFFSET_GESTURE_START_TIME):.1f}/"
                    f"{OFFSET_GESTURE_START_TIME:.1f}s"
                ),
                (frame_width - 380, 40),
                (0, 255, 255),
                0.65,
                2
            )

        # ====================================================
        # sitting / standing / lying / zoom 모션 제어
        # ====================================================

        motion_state = motion_controller.snapshot()
        motion_locked = motion_state.busy or motion_state.settling

        # 모션 + settling이 완전히 끝난 순간 이전 카메라 표본을 버린다.
        # 이 프레임 자체도 motion 직전/직후 계산값을 포함할 수 있으므로
        # 바로 motion 판단에는 사용하지 않는다.
        skip_tracking_update_this_frame = False
        if previous_motion_locked and not motion_locked:
            reset_face_control_after_motion()
            skip_tracking_update_this_frame = True
            post_motion_reacquire_required = True
            post_motion_face_since = None

        previous_motion_locked = motion_locked

        # 3손가락 보정이 완료된 최초 순간에만 tracking을 시작한다.
        # motion 1의 끝점은 앉은 정면(sitting, yaw=0)이다.
        if (
            calibrated
            and not calibration_active
            and not tracking_started_from_calibration
            and motion_state.phase == "WAIT_FOR_ZERO"
            and not motion_locked
        ):
            if motion_controller.enable_tracking_after_zero():
                tracking_started_from_calibration = True
                reset_face_control_after_motion()
                post_motion_reacquire_required = True
                post_motion_face_since = None
                motion_state = motion_controller.snapshot()

        # 모션 후에는 새 얼굴값이 실제로 STEP_HOLD_TIME 동안 유지되어
        # axis_states가 다시 확정될 때까지 어떤 자동 자세/yaw 요청도 보내지 않는다.
        # 얼굴이 중간에 사라지면 타이머를 처음부터 다시 시작한다.
        if post_motion_reacquire_required:
            if (
                not motion_locked
                and calibrated
                and not calibration_active
                and face_detected
                and raw_angles_available
            ):
                if post_motion_face_since is None:
                    post_motion_face_since = current_time
                elif (
                    current_time - post_motion_face_since
                    >= POST_MOTION_REACQUIRE_TIME
                ):
                    post_motion_reacquire_required = False
                    post_motion_face_since = None
                    print(
                        "TRACKING REACQUIRED | fresh face held "
                        f"{POST_MOTION_REACQUIRE_TIME:.2f}s"
                    )
            else:
                post_motion_face_since = None

        # 얼굴 인식값은 현재 움직이는 카메라를 기준으로 남아 있는 yaw 오차다.
        # 왼쪽은 음수, 오른쪽은 양수. 검지 직접 제어는 이 값에 보정치를 더하지 않고
        # 별도의 절대 sitting yaw 목표(-1/+1)로 처리한다.
        yaw_error_step = int(
            np.clip(
                axis_states["yaw"]["confirmed"],
                -2,
                2
            )
        )

        face_height_step = int(axis_states["height"]["confirmed"])

        posture_detection_enabled = (
            motion_state.phase == "TRACKING"
            and motion_state.tracking_enabled
            and motion_state.current_posture in ("sitting", "standing")
            and finger_yaw_target is None
            and not motion_locked
            and not post_motion_reacquire_required
            and calibrated
            and not calibration_active
            and not skip_tracking_update_this_frame
        )

        # ----------------------------------------------------
        # sitting <-> standing 판정
        # ----------------------------------------------------
        # sitting -> standing:
        #   1) 오직 앉은 정면(yaw=0)에서 얼굴이 기준보다 위로 2% 이상
        #      올라간 흔적을 먼저 기억한다.
        #   2) 그 뒤 얼굴이 실제로 사라지고 0.8초 동안 계속 미검출되어야
        #      standing으로 확정한다.
        #   3) 얼굴이 계속 보이는 동안에는 FaceHeight가 커져도 motion 9를
        #      실행하지 않는다.
        #
        # standing -> sitting:
        #   기존 동작을 그대로 유지한다.
        #   1) FaceHeight <= -1이면 즉시 sitting 요청
        #   2) 또는 -1 이하까지 내려간 뒤 얼굴이 사라지고 1초 유지
        if posture_detection_enabled:
            if face_detected and raw_angles_available:
                last_visible_height_step = face_height_step

                # 얼굴이 다시 보이면 진행 중이던 face-lost 타이머는 취소한다.
                posture_face_lost_candidate = None
                posture_face_lost_since = None

                # ==================================================
                # sitting -> standing
                # 얼굴이 보이는 동안에는 절대로 standing으로 확정하지 않고,
                # 위쪽으로 올라갔다는 힌트만 저장한다.
                # ==================================================
                if (
                    motion_state.current_posture == "sitting"
                    and motion_state.current_yaw == 0
                ):
                    current_height_change = get_recent_median_face_height(
                        current_time
                    )

                    if current_height_change is not None:
                        if (
                            current_height_change
                            >= STANDING_FACE_LOST_HINT_CHANGE
                        ):
                            if not standing_exit_armed:
                                print(
                                    "POSTURE STANDING ARMED | "
                                    f"FaceHeight:"
                                    f"{current_height_change * 100:+.1f}%"
                                )

                            standing_exit_armed = True

                        # 얼굴이 다시 보정 높이 이하까지 내려오면
                        # '일어나려는 중' 힌트를 취소한다.
                        elif current_height_change <= 0.0:
                            if standing_exit_armed:
                                print(
                                    "POSTURE STANDING DISARMED | "
                                    f"FaceHeight:"
                                    f"{current_height_change * 100:+.1f}%"
                                )

                            standing_exit_armed = False

                else:
                    # sitting center가 아니면 자동 standing 판정 자체를 하지 않는다.
                    standing_exit_armed = False

                # ==================================================
                # standing -> sitting
                # 기존 visible FaceHeight 판정을 그대로 유지한다.
                # ==================================================
                if (
                    motion_state.current_posture == "standing"
                    and face_height_step <= SITTING_VISIBLE_THRESHOLD
                ):
                    if motion_controller.request_posture("sitting"):
                        print(
                            "POSTURE | standing -> sitting center | "
                            f"FaceHeight:{face_height_step:+d}"
                        )

            else:
                desired_posture = None
                required_lost_time = None

                # ==================================================
                # sitting -> standing
                # 위쪽 이동 힌트가 있었고 얼굴이 실제로 사라진 경우에만
                # standing 후보가 된다.
                # ==================================================
                if (
                    motion_state.current_posture == "sitting"
                    and motion_state.current_yaw == 0
                    and standing_exit_armed
                ):
                    desired_posture = "standing"
                    required_lost_time = STANDING_FACE_LOST_CONFIRM_TIME

                # ==================================================
                # standing -> sitting face-lost fallback
                # 기존 동작 그대로 유지한다.
                # ==================================================
                elif (
                    motion_state.current_posture == "standing"
                    and last_visible_height_step
                        <= -POSTURE_FACE_LOST_HINT_STEP
                ):
                    desired_posture = "sitting"
                    required_lost_time = POSTURE_FACE_LOST_CONFIRM_TIME

                if desired_posture is None:
                    posture_face_lost_candidate = None
                    posture_face_lost_since = None

                elif posture_face_lost_candidate != desired_posture:
                    posture_face_lost_candidate = desired_posture
                    posture_face_lost_since = current_time

                    if desired_posture == "standing":
                        print(
                            "POSTURE FACE-LOST CANDIDATE | standing | "
                            "upper-exit armed"
                        )
                    else:
                        print(
                            "POSTURE FACE-LOST CANDIDATE | sitting | "
                            f"last FaceHeight:{last_visible_height_step:+d}"
                        )

                elif (
                    posture_face_lost_since is not None
                    and required_lost_time is not None
                    and current_time - posture_face_lost_since
                        >= required_lost_time
                ):
                    if motion_controller.request_posture(desired_posture):
                        print(
                            "POSTURE FACE-LOST CONFIRMED | "
                            f"request {desired_posture}"
                        )

                    posture_face_lost_candidate = None
                    posture_face_lost_since = None

                    if desired_posture == "standing":
                        standing_exit_armed = False

        else:
            # 모션/settling/보정 중에는 이전 자세 전환 후보를 이어 쓰지 않는다.
            posture_face_lost_candidate = None
            posture_face_lost_since = None
            standing_exit_armed = False

        # 자세 요청이 worker에 들어갔을 수 있으므로 최신 상태를 다시 읽는다.
        motion_state = motion_controller.snapshot()
        motion_locked = motion_state.busy or motion_state.settling

        # yaw 제어 우선순위:
        #   1) sitting에서 저장된 검지 직접 목표(-1/+1)
        #   2) 저장값이 0(None)이면 일반 얼굴 yaw 추적
        # 검지 제스처가 바꾸는 것은 저장값뿐이다. 여기의 motion 출력/경로 계산 방식은
        # 기존 그대로이며, controller가 현재 yaw에서 저장된 목표까지 정상 경로로 이동한다.
        if (
            motion_state.phase == "TRACKING"
            and motion_state.tracking_enabled
            and motion_state.current_posture == "sitting"
            and motion_state.target_posture == motion_state.current_posture
            and finger_yaw_target is not None
            and not motion_locked
            and calibrated
            and not calibration_active
            and not skip_tracking_update_this_frame
        ):
            direct_target = int(finger_yaw_target)
            if (
                motion_state.current_yaw != direct_target
                or motion_state.target_yaw != direct_target
            ):
                motion_controller.update_yaw_error(
                    direct_target - motion_state.current_yaw
                )

        elif (
            motion_state.phase == "TRACKING"
            and motion_state.tracking_enabled
            and motion_state.current_posture in ("sitting", "lying")
            and motion_state.target_posture == motion_state.current_posture
            and finger_yaw_target is None
            and not (
                motion_state.current_posture == "lying"
                and motion_state.lying_zoom
            )
            and not motion_locked
            and not post_motion_reacquire_required
            and calibrated
            and not calibration_active
            and face_detected
            and raw_angles_available
            and not skip_tracking_update_this_frame
        ):
            motion_controller.update_yaw_error(yaw_error_step)

        motion_state = motion_controller.snapshot()

        # ====================================================
        # 모션 상태 화면 표시
        # ====================================================

        if motion_state.phase == "CONNECTING":
            control_status = "CONNECTING | phorce target=robot"
            control_color = (0, 165, 255)
        elif motion_state.phase == "WELCOME":
            if motion_state.busy and motion_state.last_motion_id is not None:
                control_status = (
                    f"WELCOME | MOVING motion {motion_state.last_motion_id} | "
                    "sequence 19 -> 20 -> 20 -> 21"
                )
            else:
                control_status = "WELCOME | sequence 19 -> 20 -> 20 -> 21"
            control_color = (0, 165, 255)
        elif motion_state.phase == "WAIT_FOR_H":
            control_status = "WELCOME COMPLETE | Press H to start motion 1"
            control_color = (0, 255, 255)
        elif motion_state.phase == "STARTUP":
            control_status = "STARTUP | motion 1 -> sitting center"
            control_color = (0, 165, 255)
        elif motion_state.phase == "WAIT_FOR_ZERO":
            control_status = "WAIT FOR ZERO | hold 3 fingers"
            control_color = (0, 255, 255)
        elif motion_state.phase == "TRACKING":
            if motion_state.current_posture == "standing":
                control_status = (
                    "TRACKING | STANDING CENTER | "
                    f"FaceHeight:{face_height_step:+d}"
                )
            elif motion_state.current_posture == "lying":
                if motion_state.lying_zoom:
                    control_status = "TRACKING | LYING ZOOM | yaw tracking locked"
                else:
                    control_status = (
                        f"TRACKING | LYING yaw:{motion_state.current_yaw:+d} | "
                        f"face error:{yaw_error_step:+d}"
                    )
            else:
                if finger_yaw_target is not None:
                    control_status = (
                        f"TRACKING | SITTING yaw:{motion_state.current_yaw:+d} | "
                        f"FINGER TARGET:{int(finger_yaw_target):+d} | "
                        f"FaceHeight:{face_height_step:+d}"
                    )
                else:
                    control_status = (
                        f"TRACKING | SITTING yaw:{motion_state.current_yaw:+d} | "
                        f"face error:{yaw_error_step:+d} | "
                        f"FaceHeight:{face_height_step:+d}"
                    )

            if motion_state.target_posture != motion_state.current_posture:
                control_status += (
                    f" | posture target:{motion_state.target_posture}"
                )
            control_color = (0, 255, 0)

        elif motion_state.phase == "MANUAL_TRANSITION":
            control_status = (
                f"MANUAL | {motion_state.manual_action or 'transition'} | "
                f"{motion_state.current_posture} yaw:{motion_state.current_yaw:+d}"
            )
            if motion_state.current_posture == "lying" and motion_state.lying_zoom:
                control_status += " | ZOOM"
            control_color = (0, 165, 255)

        elif motion_state.phase == "SHUTDOWN_RETURN":
            if motion_state.current_posture == "standing":
                control_status = "SHUTDOWN RETURN | standing -> sitting center"
            elif motion_state.current_posture == "lying":
                if motion_state.lying_zoom:
                    control_status = "SHUTDOWN RETURN | lying zoom -> center -> sitting"
                else:
                    control_status = (
                        f"SHUTDOWN RETURN | lying yaw:"
                        f"{motion_state.current_yaw:+d} -> 0 -> sitting"
                    )
            else:
                control_status = (
                    f"SHUTDOWN RETURN | sitting yaw:"
                    f"{motion_state.current_yaw:+d} -> 0"
                )
            control_color = (0, 165, 255)
        elif motion_state.phase == "SHUTDOWN_EXIT":
            control_status = "SHUTDOWN | motion 8 -> OFF pose"
            control_color = (0, 0, 255)
        elif motion_state.phase == "ERROR":
            control_status = (
                "ERROR | " + (motion_state.last_error or "motion controller error")
            )
            control_color = (0, 0, 255)
        else:
            control_status = "DONE | motion 8 completed"
            control_color = (255, 255, 0)

        if motion_state.busy:
            control_status += f" | MOVING motion {motion_state.last_motion_id}"
        elif motion_state.settling:
            control_status += " | SETTLING"

        draw_text(
            frame,
            control_status,
            (30, frame_height - 55),
            control_color,
            0.55,
            2
        )

        draw_text(
            frame,
            "H: start after welcome   Index drag: sit yaw -1/+1, center=face   3F: zero/start   L: sit<->lie   Z: zoom   Q: shutdown",
            (30, frame_height - 25),
            (255, 255, 255),
            0.50,
            1
        )

        cv2.imshow(
            "Hand and Face Tracking",
            frame
        )

        # ====================================================
        # 종료 요청
        # ====================================================

        key = cv2.waitKey(1) & 0xFF

        # 4손가락 종료는 현재 비활성화한다.
        # 인식 자체는 남겨 두지만 shutdown 요청과 연결하지 않는다.
        if four_finger_quit_requested:
            pass

        # cv2.waitKey는 한 프레임에 key code 하나만 반환하고, controller도
        # MANUAL_TRANSITION 동안 새 L/Z를 거부하므로 L/Z가 겹쳐 실행되지 않는다.
        # Q는 항상 최우선이다.
        if key == ord("q"):
            state_before_q = motion_controller.snapshot()
            if state_before_q.phase == "ERROR":
                print(
                    "Q | controller is in ERROR; "
                    "safe return cannot be guaranteed, program exits without new motion"
                )
                break

            reset_finger_yaw_override_and_gesture()
            motion_controller.request_shutdown()

        elif key == ord("h"):
            if motion_controller.request_start():
                # H는 오직 19 -> 20 -> 20 -> 21 완료 뒤 WAIT_FOR_H에서만
                # 받아들여진다. 여기서 motion 1이 worker에 예약된다.
                reset_finger_yaw_override_and_gesture()
                reset_posture_transition_detection(reset_last_visible=True)
                reset_face_control_after_motion()
                post_motion_reacquire_required = False
                post_motion_face_since = None
                print("KEY H | startup accepted | motion 1 requested")
            else:
                current_phase = motion_controller.snapshot().phase
                print(
                    "KEY H | ignored | "
                    f"wait until phase=WAIT_FOR_H (current={current_phase})"
                )

        elif key == ord("l"):
            if motion_controller.request_lying_toggle():
                # L이 실제로 받아들여졌을 때만 검지 직접 yaw 목표를 삭제한다.
                # 3손가락 얼굴 원점은 그대로 유지한다.
                reset_finger_yaw_override_and_gesture()
                reset_posture_transition_detection(reset_last_visible=True)
                print("KEY L | sitting <-> lying transition accepted")
            else:
                print("KEY L | ignored in current state / motion lock")

        elif key == ord("z"):
            if motion_controller.request_zoom_toggle():
                print("KEY Z | lying center zoom toggle accepted")
            else:
                print("KEY Z | ignored (only lying center/zoom, no motion lock)")

        # motion 8까지 끝난 뒤에만 프로그램을 종료한다.
        final_motion_state = motion_controller.snapshot()
        if (
            final_motion_state.shutdown_done
            and not final_motion_state.busy
            and not final_motion_state.settling
        ):
            print("SHUTDOWN SEQUENCE COMPLETE | motion 8 completed")
            break


motion_controller.stop()
cap.release()
cv2.destroyAllWindows()