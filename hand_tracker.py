import cv2
import math
import os
import time
from dataclasses import dataclass

import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


# =====================================================
# Config
# =====================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.getenv(
    "HAND_MODEL_PATH",
    os.path.join(BASE_DIR, "hand_landmarker.task"),
)

# Iriun Webcam: 현재 확인한 장치 번호가 4.
# 실행 시 CAMERA_SOURCE=다른번호 로 덮어쓸 수 있다.
CAMERA_SOURCE = os.getenv("CAMERA_SOURCE", "4")
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "1280"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "720"))
DISPLAY_WIDTH = int(os.getenv("DISPLAY_WIDTH", "960"))
DISPLAY_HEIGHT = int(os.getenv("DISPLAY_HEIGHT", "540"))

# 화면을 거울처럼 좌우 반전할지 여부
MIRROR_VIEW = os.getenv("MIRROR_VIEW", "1") == "1"

# 거리 보정
REAL_PALM_WIDTH = 0.09
NEAR_CM = 30.0
FAR_CM = 100.0

# 좌표 발행 주기
PUB_INTERVAL = 1.0 / 30.0

# 손 위치 -> 로봇 EE 목표 위치 오프셋
EE_Y_OFFSET = 0.25
EE_Z_OFFSET = 0.05
TABLE_HEIGHT = 1.0

# 카메라와 가까울수록 높은 Z, 멀수록 낮은 Z
HAND_Z_NEAR = 0.45
HAND_Z_FAR = 0.05

# 제스처
GESTURE_HOLD_SEC = 1.5
MODE_FOLLOW = "FOLLOW"
MODE_PLACE = "PLACE"

# 손바닥 앞/뒤 판정
# 반대로 잡히면 실행할 때 PALM_Z_SIGN=-1 사용
PALM_Z_SIGN = float(os.getenv("PALM_Z_SIGN", "1.0"))

# 정면/후면으로 확실히 인정하는 각도 기준.
# 1.0에 가까울수록 카메라 축과 거의 평행해야 한다.
PALM_ENTER_THRESHOLD = 0.55
PALM_EXIT_THRESHOLD = 0.30
PALM_FILTER_ALPHA = 0.25
PALM_STABLE_SEC = 0.20

# 카메라 중심에서 손 중심까지 이어주는 선
ORIGIN_LINE_COLOR = (205, 216, 230)

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
]

# OpenCV는 BGR 순서.
# 왼손은 cyan, 오른손은 violet 계열로 구분한다.
HAND_STYLE = {
    "Left": {
        "accent": (255, 226, 72),
        "bone": (245, 206, 72),
        "joint": (255, 244, 156),
    },
    "Right": {
        "accent": (255, 92, 222),
        "bone": (232, 84, 196),
        "joint": (255, 158, 236),
    },
}

# Glass / neon UI
UI_GLASS = (20, 23, 31)
UI_GLASS_ALT = (27, 31, 41)
UI_TEXT = (241, 244, 248)
UI_MUTED = (156, 166, 181)
UI_SUBTLE = (69, 77, 91)
UI_GREEN = (126, 239, 148)
UI_AMBER = (86, 201, 255)
UI_RED = (108, 115, 255)
UI_WHITE = (238, 242, 247)

ROS_DOMAIN_DISPLAY = os.getenv("ROS_DOMAIN_ID", "unset")


# =====================================================
# Camera
# =====================================================
def parse_camera_source(value: str):
    value = value.strip()
    return int(value) if value.isdigit() else value


def open_camera(source):
    camera = cv2.VideoCapture(source)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return camera


# =====================================================
# MediaPipe
# =====================================================
if not os.path.isfile(MODEL_PATH):
    raise FileNotFoundError(f"MediaPipe model not found: {MODEL_PATH}")

options = vision.HandLandmarkerOptions(
    base_options=python.BaseOptions(model_asset_path=MODEL_PATH),
    num_hands=2,
    min_hand_detection_confidence=0.55,
    min_hand_presence_confidence=0.55,
    min_tracking_confidence=0.55,
)
detector = vision.HandLandmarker.create_from_options(options)

camera_source = parse_camera_source(CAMERA_SOURCE)
cap = open_camera(camera_source)

if not cap.isOpened():
    raise RuntimeError(
        f"Camera open failed: CAMERA_SOURCE={CAMERA_SOURCE}. "
        "Run `v4l2-ctl --list-devices` and check the Iriun device number."
    )


# =====================================================
# ROS2
# =====================================================
rclpy.init()
ros_node = Node("dual_hand_publisher")

stream_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 상태/방향 토픽은 나중에 구독한 노드도 마지막 값을 받을 수 있게 유지한다.
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass
class HandPublishers:
    raw: object
    xyz: object
    mode: object
    palm: object


publishers = {
    "Left": HandPublishers(
        raw=ros_node.create_publisher(Point, "/left_hand_raw", stream_qos),
        xyz=ros_node.create_publisher(Point, "/left_hand_xyz", stream_qos),
        mode=ros_node.create_publisher(String, "/left_hand_mode", state_qos),
        palm=ros_node.create_publisher(String, "/left_palm_direction", state_qos),
    ),
    "Right": HandPublishers(
        raw=ros_node.create_publisher(Point, "/right_hand_raw", stream_qos),
        xyz=ros_node.create_publisher(Point, "/right_hand_xyz", stream_qos),
        mode=ros_node.create_publisher(String, "/right_hand_mode", state_qos),
        palm=ros_node.create_publisher(String, "/right_palm_direction", state_qos),
    ),
}

# 기존 단일 오른손 코드와의 호환용
legacy_raw_pub = ros_node.create_publisher(Point, "/hand_raw", stream_qos)
legacy_xyz_pub = ros_node.create_publisher(Point, "/hand_xyz", stream_qos)
legacy_mode_pub = ros_node.create_publisher(String, "/hand_mode", state_qos)


# =====================================================
# Coordinate processing
# =====================================================
class CoordProcessor:
    def __init__(self, alpha=0.25):
        self.alpha = alpha
        self.filtered_ee = None
        self.z_floor = TABLE_HEIGHT - 0.05

    def reset(self):
        self.filtered_ee = None

    def update(self, camera_x, camera_y, camera_distance):
        """
        반환값:
          hand_pos: 카메라 결과를 프로젝트 좌표로 바꾼 손 위치
          ee_pos:   오프셋 + 스무딩이 적용된 로봇 EE 목표 위치
        """
        near_m = NEAR_CM / 100.0
        far_m = FAR_CM / 100.0

        distance = float(np.clip(camera_distance, near_m, far_m))
        ratio = (distance - near_m) / (far_m - near_m)

        # 가까울수록 높고, 멀수록 낮다.
        relative_z = HAND_Z_NEAR + ratio * (HAND_Z_FAR - HAND_Z_NEAR)

        hand_x = float(camera_x)
        hand_y = float(camera_y) - 0.30
        hand_z = TABLE_HEIGHT + relative_z

        hand_pos = np.array([hand_x, hand_y, hand_z], dtype=float)

        unfiltered_ee = np.array(
            [
                hand_x,
                hand_y + EE_Y_OFFSET,
                max(hand_z + EE_Z_OFFSET, self.z_floor),
            ],
            dtype=float,
        )

        if self.filtered_ee is None:
            self.filtered_ee = unfiltered_ee.copy()
        else:
            self.filtered_ee = (
                self.alpha * unfiltered_ee
                + (1.0 - self.alpha) * self.filtered_ee
            )

        return hand_pos, self.filtered_ee.copy()


processors = {
    "Left": CoordProcessor(alpha=0.25),
    "Right": CoordProcessor(alpha=0.25),
}


# =====================================================
# Hand pose detection
# =====================================================
def classify_hand_pose(hand) -> str:
    """
    손 회전에 덜 민감하도록 손바닥 중심으로부터 손가락 끝과 PIP까지의
    3차원 거리를 비교한다.

    반환:
      FIST  : 네 손가락 중 3개 이상 확실히 접힘
      OPEN  : 네 손가락 중 3개 이상 확실히 펴짐
      OTHER : 중간 자세, 반쯤 편 손 등
    """
    palm_center = np.mean(
        np.array(
            [[hand[i].x, hand[i].y, hand[i].z] for i in [0, 5, 9, 13, 17]],
            dtype=float,
        ),
        axis=0,
    )

    tip_ids = [8, 12, 16, 20]
    pip_ids = [6, 10, 14, 18]

    curled = 0
    extended = 0

    for tip_id, pip_id in zip(tip_ids, pip_ids):
        tip = np.array(
            [hand[tip_id].x, hand[tip_id].y, hand[tip_id].z],
            dtype=float,
        )
        pip = np.array(
            [hand[pip_id].x, hand[pip_id].y, hand[pip_id].z],
            dtype=float,
        )

        tip_distance = np.linalg.norm(tip - palm_center)
        pip_distance = np.linalg.norm(pip - palm_center)

        if tip_distance < pip_distance * 0.98:
            curled += 1
        elif tip_distance > pip_distance * 1.10:
            extended += 1

    if curled >= 3:
        return "FIST"
    if extended >= 3:
        return "OPEN"
    return "OTHER"


# =====================================================
# Palm-facing detection
# =====================================================
def _point3(landmarks, index):
    point = landmarks[index]
    return np.array([point.x, point.y, point.z], dtype=float)


def compute_palm_camera_score(landmarks, label: str) -> float:
    """
    카메라 축에 대한 손바닥 법선 점수:
      +1에 가까움: 손바닥이 카메라를 향함
      -1에 가까움: 손등이 카메라를 향함
       0에 가까움: 손이 옆으로 기울어짐

    가능하면 MediaPipe world landmarks를 사용한다.
    """
    wrist = _point3(landmarks, 0)
    index_mcp = _point3(landmarks, 5)
    middle_mcp = _point3(landmarks, 9)
    ring_mcp = _point3(landmarks, 13)
    pinky_mcp = _point3(landmarks, 17)

    # 손바닥 평면을 여러 삼각형으로 계산해 z 노이즈를 완화한다.
    pairs = [
        (index_mcp - wrist, middle_mcp - wrist),
        (middle_mcp - wrist, ring_mcp - wrist),
        (ring_mcp - wrist, pinky_mcp - wrist),
        (index_mcp - wrist, pinky_mcp - wrist),
    ]

    normals = []

    for vector_a, vector_b in pairs:
        normal = np.cross(vector_a, vector_b)
        norm = np.linalg.norm(normal)

        if norm > 1e-8:
            normals.append(normal / norm)

    if not normals:
        return 0.0

    normal = np.mean(normals, axis=0)
    normal_norm = np.linalg.norm(normal)

    if normal_norm < 1e-8:
        return 0.0

    normal /= normal_norm

    # 좌우 손의 관절 순서에 따른 법선 방향을 같은 기준으로 맞춘다.
    if label == "Left":
        normal *= -1.0

    # MediaPipe 카메라 좌표에서 음의 z 방향을 카메라 쪽으로 사용.
    # 실제 환경에서 반대면 PALM_Z_SIGN=-1로 실행한다.
    camera_score = -float(normal[2]) * PALM_Z_SIGN
    return float(np.clip(camera_score, -1.0, 1.0))


class PalmFacingFilter:
    """
    순간적인 landmark 떨림 때문에 TOWARD/AWAY가 바뀌지 않도록
    EMA + 진입/이탈 임계값 + 짧은 안정화 시간을 사용한다.
    """
    def __init__(self):
        self.filtered_score = None
        self.state = "SIDEWAYS"
        self.candidate = None
        self.candidate_started = None

    def reset(self):
        self.filtered_score = None
        self.state = "SIDEWAYS"
        self.candidate = None
        self.candidate_started = None

    def update(self, raw_score: float):
        now = time.monotonic()

        if self.filtered_score is None:
            self.filtered_score = raw_score
        else:
            self.filtered_score = (
                PALM_FILTER_ALPHA * raw_score
                + (1.0 - PALM_FILTER_ALPHA) * self.filtered_score
            )

        score = self.filtered_score

        # 현재 상태 유지 조건. 점수가 약해지면 SIDEWAYS로 바로 빠져
        # 제스처 유지 타이머가 잘못 누적되지 않게 한다.
        if self.state == "TOWARD_CAMERA" and score >= PALM_EXIT_THRESHOLD:
            self.candidate = None
            self.candidate_started = None
            return self.state, score

        if self.state == "AWAY_CAMERA" and score <= -PALM_EXIT_THRESHOLD:
            self.candidate = None
            self.candidate_started = None
            return self.state, score

        if self.state != "SIDEWAYS":
            self.state = "SIDEWAYS"
            self.candidate = None
            self.candidate_started = None

        if score >= PALM_ENTER_THRESHOLD:
            desired = "TOWARD_CAMERA"
        elif score <= -PALM_ENTER_THRESHOLD:
            desired = "AWAY_CAMERA"
        else:
            self.candidate = None
            self.candidate_started = None
            return self.state, score

        if self.candidate != desired:
            self.candidate = desired
            self.candidate_started = now
            return self.state, score

        if now - self.candidate_started >= PALM_STABLE_SEC:
            self.state = desired
            self.candidate = None
            self.candidate_started = None

        return self.state, score


palm_filters = {
    "Left": PalmFacingFilter(),
    "Right": PalmFacingFilter(),
}


# =====================================================
# Gesture mode
# =====================================================
class GestureModeTracker:
    """
    FOLLOW:
      TOWARD_CAMERA + FIST 상태를 1.5초 연속 유지

    PLACE:
      AWAY_CAMERA + OPEN 상태를 1.5초 연속 유지

    손바닥이 옆을 향하거나 손 자세가 조건과 달라지거나,
    손 검출이 끊기면 유지 타이머를 즉시 초기화한다.
    """
    def __init__(self):
        self.mode = MODE_PLACE
        self.candidate_mode = None
        self.candidate_started = None

    def reset(self):
        self.mode = MODE_PLACE
        self.candidate_mode = None
        self.candidate_started = None

    def cancel_candidate(self):
        self.candidate_mode = None
        self.candidate_started = None

    def update(self, hand_pose: str, palm_direction: str):
        now = time.monotonic()

        if hand_pose == "FIST" and palm_direction == "TOWARD_CAMERA":
            desired_mode = MODE_FOLLOW
        elif hand_pose == "OPEN" and palm_direction == "AWAY_CAMERA":
            desired_mode = MODE_PLACE
        else:
            self.cancel_candidate()
            return False, 0.0, None

        if desired_mode == self.mode:
            self.cancel_candidate()
            return False, 0.0, desired_mode

        if self.candidate_mode != desired_mode:
            self.candidate_mode = desired_mode
            self.candidate_started = now
            return False, 0.0, desired_mode

        elapsed = now - self.candidate_started

        if elapsed >= GESTURE_HOLD_SEC:
            self.mode = desired_mode
            self.cancel_candidate()
            return True, GESTURE_HOLD_SEC, desired_mode

        return False, elapsed, desired_mode


gesture_trackers = {
    "Left": GestureModeTracker(),
    "Right": GestureModeTracker(),
}


# =====================================================
# Drawing helpers — minimal glass / neon UI
# =====================================================
def draw_rounded_rect(image, pt1, pt2, color, radius=14, thickness=-1):
    x1, y1 = pt1
    x2, y2 = pt2
    radius = max(1, min(radius, (x2 - x1) // 2, (y2 - y1) // 2))

    if thickness < 0:
        cv2.rectangle(image, (x1 + radius, y1), (x2 - radius, y2), color, -1)
        cv2.rectangle(image, (x1, y1 + radius), (x2, y2 - radius), color, -1)
        cv2.circle(image, (x1 + radius, y1 + radius), radius, color, -1)
        cv2.circle(image, (x2 - radius, y1 + radius), radius, color, -1)
        cv2.circle(image, (x1 + radius, y2 - radius), radius, color, -1)
        cv2.circle(image, (x2 - radius, y2 - radius), radius, color, -1)
        return

    cv2.line(image, (x1 + radius, y1), (x2 - radius, y1), color, thickness, cv2.LINE_AA)
    cv2.line(image, (x1 + radius, y2), (x2 - radius, y2), color, thickness, cv2.LINE_AA)
    cv2.line(image, (x1, y1 + radius), (x1, y2 - radius), color, thickness, cv2.LINE_AA)
    cv2.line(image, (x2, y1 + radius), (x2, y2 - radius), color, thickness, cv2.LINE_AA)
    cv2.ellipse(image, (x1 + radius, y1 + radius), (radius, radius), 180, 0, 90, color, thickness, cv2.LINE_AA)
    cv2.ellipse(image, (x2 - radius, y1 + radius), (radius, radius), 270, 0, 90, color, thickness, cv2.LINE_AA)
    cv2.ellipse(image, (x2 - radius, y2 - radius), (radius, radius), 0, 0, 90, color, thickness, cv2.LINE_AA)
    cv2.ellipse(image, (x1 + radius, y2 - radius), (radius, radius), 90, 0, 90, color, thickness, cv2.LINE_AA)


def draw_text(frame, text, position, color=UI_TEXT, scale=0.44, thickness=1):
    cv2.putText(
        frame,
        text,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def blend_color(base, accent, ratio):
    return tuple(
        int(base[i] * (1.0 - ratio) + accent[i] * ratio)
        for i in range(3)
    )


def draw_glass_card(frame, pt1, pt2, opacity=0.80, radius=18):
    overlay = frame.copy()
    draw_rounded_rect(overlay, pt1, pt2, UI_GLASS, radius, -1)
    cv2.addWeighted(overlay, opacity, frame, 1.0 - opacity, 0, frame)


def draw_chip(frame, text, x, y, accent, active=True):
    scale = 0.36
    (text_width, _), _ = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        1,
    )

    width = text_width + 22
    height = 23
    fill = blend_color(UI_GLASS_ALT, accent, 0.22 if active else 0.06)
    text_color = accent if active else UI_MUTED

    overlay = frame.copy()
    draw_rounded_rect(
        overlay,
        (x, y),
        (x + width, y + height),
        fill,
        11,
        -1,
    )
    cv2.addWeighted(overlay, 0.88, frame, 0.12, 0, frame)

    draw_text(
        frame,
        text,
        (x + 11, y + 16),
        text_color,
        scale,
        1,
    )
    return width


def draw_neon_line(frame, start, end, accent, crisp_thickness=1, glow_thickness=7):
    glow = np.zeros_like(frame)
    cv2.line(glow, start, end, accent, glow_thickness, cv2.LINE_AA)
    glow = cv2.GaussianBlur(glow, (0, 0), 4.0)
    cv2.addWeighted(frame, 1.0, glow, 0.32, 0, frame)
    cv2.line(frame, start, end, accent, crisp_thickness, cv2.LINE_AA)


def draw_neon_circle(frame, center, radius, accent, filled=True):
    glow = np.zeros_like(frame)
    cv2.circle(glow, center, radius + 5, accent, -1, cv2.LINE_AA)
    glow = cv2.GaussianBlur(glow, (0, 0), 4.0)
    cv2.addWeighted(frame, 1.0, glow, 0.30, 0, frame)

    if filled:
        cv2.circle(frame, center, radius, accent, -1, cv2.LINE_AA)
    else:
        cv2.circle(frame, center, radius, accent, 1, cv2.LINE_AA)


def draw_skeleton(frame, hand, width, height, style):
    points = [(int(lm.x * width), int(lm.y * height)) for lm in hand]

    # 모든 뼈를 한 번에 glow 처리해서 과한 검은 외곽선을 없앤다.
    glow = np.zeros_like(frame)

    for start, end in HAND_CONNECTIONS:
        cv2.line(
            glow,
            points[start],
            points[end],
            style["accent"],
            6,
            cv2.LINE_AA,
        )

    for x, y in points:
        cv2.circle(glow, (x, y), 6, style["accent"], -1, cv2.LINE_AA)

    glow = cv2.GaussianBlur(glow, (0, 0), 4.5)
    cv2.addWeighted(frame, 1.0, glow, 0.24, 0, frame)

    for start, end in HAND_CONNECTIONS:
        cv2.line(
            frame,
            points[start],
            points[end],
            style["bone"],
            1,
            cv2.LINE_AA,
        )

    for index, (x, y) in enumerate(points):
        radius = 4 if index in (0, 9) else 2
        cv2.circle(frame, (x, y), radius, style["joint"], -1, cv2.LINE_AA)


def draw_palm_width_line(frame, hand, width, height, style):
    x1, y1 = int(hand[5].x * width), int(hand[5].y * height)
    x2, y2 = int(hand[17].x * width), int(hand[17].y * height)

    draw_neon_line(
        frame,
        (x1, y1),
        (x2, y2),
        style["accent"],
        crisp_thickness=1,
        glow_thickness=5,
    )

    return math.hypot(x2 - x1, y2 - y1)


def draw_origin_to_hand_line(
    frame,
    origin,
    hand_center,
    label,
    camera_x_m,
    camera_y_m,
):
    accent = HAND_STYLE[label]["accent"]

    draw_neon_line(
        frame,
        origin,
        hand_center,
        accent,
        crisp_thickness=1,
        glow_thickness=5,
    )
    draw_neon_circle(frame, hand_center, 3, accent, filled=True)

    midpoint = (
        (origin[0] + hand_center[0]) // 2,
        (origin[1] + hand_center[1]) // 2,
    )

    if camera_x_m is None or camera_y_m is None:
        tag = "CALIBRATING"
    else:
        tag = f"X {camera_x_m:+.3f} m   Y {camera_y_m:+.3f} m"

    (text_width, text_height), _ = cv2.getTextSize(
        tag,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.39,
        1,
    )

    tag_x = midpoint[0] - text_width // 2
    tag_y = max(194, min(frame.shape[0] - 48, midpoint[1] - 12))

    overlay = frame.copy()
    draw_rounded_rect(
        overlay,
        (tag_x - 11, tag_y - text_height - 8),
        (tag_x + text_width + 11, tag_y + 8),
        UI_GLASS,
        10,
        -1,
    )
    cv2.addWeighted(overlay, 0.82, frame, 0.18, 0, frame)

    draw_text(
        frame,
        tag,
        (tag_x, tag_y),
        accent,
        0.39,
        1,
    )


def draw_center_origin(frame, center):
    x, y = center

    glow = np.zeros_like(frame)
    cv2.circle(glow, (x, y), 11, UI_WHITE, 2, cv2.LINE_AA)
    glow = cv2.GaussianBlur(glow, (0, 0), 4.0)
    cv2.addWeighted(frame, 1.0, glow, 0.24, 0, frame)

    cv2.circle(frame, (x, y), 7, UI_WHITE, 1, cv2.LINE_AA)
    cv2.circle(frame, (x, y), 2, UI_WHITE, -1, cv2.LINE_AA)


def draw_hand_panel(
    frame,
    label,
    detected,
    mode,
    candidate_mode,
    elapsed,
    hand_pos,
    ee_pos,
    publishing,
):
    _, width = frame.shape[:2]
    style = HAND_STYLE[label]

    card_width = width // 2 - 18
    card_height = 166
    card_x = 8 if label == "Left" else width // 2 + 10
    card_y = 8

    draw_glass_card(
        frame,
        (card_x, card_y),
        (card_x + card_width, card_y + card_height),
        opacity=0.82,
        radius=18,
    )

    # 짧고 얇은 네온 포인트 라인
    draw_neon_line(
        frame,
        (card_x + 20, card_y + 10),
        (card_x + 82, card_y + 10),
        style["accent"],
        crisp_thickness=1,
        glow_thickness=4,
    )

    draw_text(
        frame,
        f"{label.upper()} HAND",
        (card_x + 20, card_y + 31),
        UI_TEXT,
        0.53,
        1,
    )

    mode_accent = style["accent"]
    mode_x = card_x + card_width - 157
    mode_width = draw_chip(
        frame,
        mode,
        mode_x,
        card_y + 15,
        mode_accent,
        active=True,
    )

    draw_chip(
        frame,
        "LIVE" if publishing else "IDLE",
        mode_x + mode_width + 8,
        card_y + 15,
        UI_GREEN if publishing else UI_MUTED,
        active=publishing,
    )

    pose = detected["pose"]
    direction = detected["palm_direction"]
    score = detected["palm_score"]

    pose_color = (
        UI_GREEN
        if pose == "OPEN"
        else style["accent"]
        if pose == "FIST"
        else UI_MUTED
    )

    pose_width = draw_chip(
        frame,
        pose,
        card_x + 20,
        card_y + 44,
        pose_color,
        active=pose != "OTHER",
    )

    draw_text(
        frame,
        f"{direction}   {score:+.2f}",
        (card_x + 30 + pose_width, card_y + 60),
        UI_MUTED,
        0.39,
        1,
    )

    # 얇은 divider
    cv2.line(
        frame,
        (card_x + 20, card_y + 76),
        (card_x + card_width - 20, card_y + 76),
        UI_SUBTLE,
        1,
        cv2.LINE_AA,
    )

    if hand_pos is None:
        raw_values = "--      --      --"
        ee_values = "--      --      --"
    else:
        raw_values = (
            f"{hand_pos[0]:+.3f}      "
            f"{hand_pos[1]:+.3f}      "
            f"{hand_pos[2]:.3f}"
        )
        ee_values = (
            f"{ee_pos[0]:+.3f}      "
            f"{ee_pos[1]:+.3f}      "
            f"{ee_pos[2]:.3f}"
        )

    draw_text(
        frame,
        "HAND / RAW",
        (card_x + 20, card_y + 94),
        UI_MUTED,
        0.34,
        1,
    )
    draw_text(
        frame,
        raw_values,
        (card_x + 120, card_y + 94),
        (128, 226, 255),
        0.41,
        1,
    )

    draw_text(
        frame,
        "EE TARGET",
        (card_x + 20, card_y + 116),
        UI_MUTED,
        0.34,
        1,
    )
    draw_text(
        frame,
        ee_values,
        (card_x + 120, card_y + 116),
        UI_GREEN,
        0.41,
        1,
    )

    camera_distance = detected["camera_position"][2]
    if camera_distance is None:
        meta = "DIST --   |   POSITION REQUIRES CALIBRATION"
    else:
        meta = (
            f"DIST {camera_distance:.3f} m   |   "
            f"IMAGE CENTER RELATIVE XY"
        )

    draw_text(
        frame,
        meta,
        (card_x + 20, card_y + 137),
        UI_MUTED,
        0.34,
        1,
    )

    if candidate_mode is None or candidate_mode == mode:
        instruction = "FOLLOW  FRONT+FIST    PLACE  BACK+OPEN"
        progress = 0.0
    else:
        required = (
            "FRONT + FIST"
            if candidate_mode == MODE_FOLLOW
            else "BACK + OPEN"
        )
        instruction = (
            f"{candidate_mode}   {required}   "
            f"{elapsed:.1f}/{GESTURE_HOLD_SEC:.1f}s"
        )
        progress = min(elapsed / GESTURE_HOLD_SEC, 1.0)

    draw_text(
        frame,
        instruction,
        (card_x + 20, card_y + 157),
        UI_TEXT,
        0.34,
        1,
    )

    # 아주 얇은 neon progress line
    progress_x1 = card_x + 20
    progress_x2 = card_x + card_width - 20
    progress_y = card_y + 162

    cv2.line(
        frame,
        (progress_x1, progress_y),
        (progress_x2, progress_y),
        UI_SUBTLE,
        1,
        cv2.LINE_AA,
    )

    if progress > 0.0:
        fill_x = progress_x1 + int((progress_x2 - progress_x1) * progress)
        draw_neon_line(
            frame,
            (progress_x1, progress_y),
            (fill_x, progress_y),
            style["accent"],
            crisp_thickness=1,
            glow_thickness=4,
        )


def draw_missing_hand_panel(frame, label, mode):
    _, width = frame.shape[:2]
    style = HAND_STYLE[label]

    card_width = width // 2 - 18
    card_height = 166
    card_x = 8 if label == "Left" else width // 2 + 10
    card_y = 8

    draw_glass_card(
        frame,
        (card_x, card_y),
        (card_x + card_width, card_y + card_height),
        opacity=0.78,
        radius=18,
    )

    draw_neon_line(
        frame,
        (card_x + 20, card_y + 10),
        (card_x + 82, card_y + 10),
        style["accent"],
        crisp_thickness=1,
        glow_thickness=4,
    )

    draw_text(
        frame,
        f"{label.upper()} HAND",
        (card_x + 20, card_y + 31),
        UI_TEXT,
        0.53,
        1,
    )

    draw_chip(
        frame,
        mode,
        card_x + card_width - 88,
        card_y + 15,
        style["accent"],
        active=True,
    )

    draw_text(
        frame,
        "NO HAND DETECTED",
        (card_x + 20, card_y + 92),
        style["accent"],
        0.62,
        1,
    )
    draw_text(
        frame,
        "Gesture timer reset",
        (card_x + 20, card_y + 119),
        UI_MUTED,
        0.40,
        1,
    )


def draw_calibration_banner(frame, text):
    height, width = frame.shape[:2]
    card_width = min(590, width - 70)
    card_height = 70
    x1 = (width - card_width) // 2
    y1 = height // 2 - card_height // 2
    x2 = x1 + card_width
    y2 = y1 + card_height

    draw_glass_card(
        frame,
        (x1, y1),
        (x2, y2),
        opacity=0.88,
        radius=20,
    )

    draw_neon_line(
        frame,
        (x1 + 22, y1 + 12),
        (x1 + 92, y1 + 12),
        UI_AMBER,
        crisp_thickness=1,
        glow_thickness=4,
    )

    draw_text(
        frame,
        "CALIBRATION",
        (x1 + 22, y1 + 34),
        UI_AMBER,
        0.42,
        1,
    )
    draw_text(
        frame,
        text,
        (x1 + 22, y1 + 57),
        UI_TEXT,
        0.55,
        1,
    )


def draw_footer(frame, fps, recording, calibration_step):
    height, width = frame.shape[:2]
    y = height - 14

    # 하단 전체 박스를 없애고 최소 정보만 표시
    state_text = "RUNNING" if recording else f"CAL {calibration_step}/2"
    state_color = UI_GREEN if recording else UI_AMBER

    draw_neon_circle(frame, (16, y - 1), 2, state_color, filled=True)
    draw_text(
        frame,
        f"{state_text}   |   ROS {ROS_DOMAIN_DISPLAY}",
        (27, y + 3),
        UI_MUTED,
        0.36,
        1,
    )
    draw_text(
        frame,
        f"CAM {CAMERA_SOURCE}   {fps:4.1f} FPS",
        (width - 150, y + 3),
        UI_MUTED,
        0.36,
        1,
    )


# =====================================================
# Math / publishing helpers
# =====================================================
def compute_camera_position(palm_px, dx_px, dy_px, focal_length):
    """
    카메라 중심 기준 손 위치:
      x/y: 손바닥 폭으로 환산한 화면 중심 대비 미터 거리
      z:   핀홀 모델로 계산한 카메라-손 거리
    """
    if focal_length is None or palm_px <= 0:
        return None, None, None

    distance_m = focal_length * REAL_PALM_WIDTH / palm_px
    meter_per_pixel = REAL_PALM_WIDTH / palm_px

    return (
        dx_px * meter_per_pixel,
        -dy_px * meter_per_pixel,
        distance_m,
    )


def compute_focal(near_px, far_px):
    f_near = near_px * (NEAR_CM / 100.0) / REAL_PALM_WIDTH
    f_far = far_px * (FAR_CM / 100.0) / REAL_PALM_WIDTH
    return (f_near + f_far) / 2.0


def publish_mode(label, mode):
    publishers[label].mode.publish(String(data=mode))

    if label == "Right":
        legacy_mode_pub.publish(String(data=mode))


def publish_palm_direction(label, direction):
    publishers[label].palm.publish(String(data=direction))


def publish_coordinates(label, hand_pos, ee_pos):
    """
    ROS Point에는 float64 전체 정밀도로 발행한다.
    화면 오버레이에서만 소수점 셋째 자리로 보기 좋게 표시한다.
    """
    raw_message = Point(
        x=float(hand_pos[0]),
        y=float(hand_pos[1]),
        z=float(hand_pos[2]),
    )
    xyz_message = Point(
        x=float(ee_pos[0]),
        y=float(ee_pos[1]),
        z=float(ee_pos[2]),
    )

    publishers[label].raw.publish(raw_message)
    publishers[label].xyz.publish(xyz_message)

    if label == "Right":
        legacy_raw_pub.publish(raw_message)
        legacy_xyz_pub.publish(xyz_message)


# =====================================================
# Runtime state
# =====================================================
near_px = None
far_px = None
focal_length = None
calibration_step = 0
recording = False

last_publish_time = {
    "Left": 0.0,
    "Right": 0.0,
}
last_palm_direction = {
    "Left": None,
    "Right": None,
}
last_mode_heartbeat = {
    "Left": 0.0,
    "Right": 0.0,
}

fps_value = 0.0
fps_frames = 0
fps_window_started = time.monotonic()


print("=" * 78)
print("Dual Hand Tracker - neon glass overlay")
print(f"Camera source: {CAMERA_SOURCE}")
print("SPACE: 30cm calibration -> 100cm calibration -> start")
print("R: reset | Q/ESC: quit")
print("FOLLOW: palm toward camera + fist for 1.5 sec")
print("PLACE : palm away from camera + open hand for 1.5 sec")
print("Near hand -> high Z | Far hand -> low Z")
print("=" * 78)


try:
    while True:
        frame_ok, frame = cap.read()

        if not frame_ok:
            print("[ERROR] Camera frame read failed")
            break

        frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))

        fps_frames += 1
        fps_now = time.monotonic()
        fps_elapsed = fps_now - fps_window_started
        if fps_elapsed >= 0.5:
            fps_value = fps_frames / fps_elapsed
            fps_frames = 0
            fps_window_started = fps_now

        if MIRROR_VIEW:
            frame = cv2.flip(frame, 1)

        height, width = frame.shape[:2]
        origin = (width // 2, height // 2)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb,
        )
        result = detector.detect(mp_image)

        draw_center_origin(frame, origin)

        detected_hands = {}
        calibration_candidates = []

        if result.hand_landmarks:
            for index, image_hand in enumerate(result.hand_landmarks):
                raw_label = result.handedness[index][0].category_name

                if MIRROR_VIEW:
                    label = "Right" if raw_label == "Left" else "Left"
                else:
                    label = raw_label

                style = HAND_STYLE[label]

                # 손바닥 앞/뒤 판정에는 world landmark가 더 안정적이다.
                if (
                    hasattr(result, "hand_world_landmarks")
                    and result.hand_world_landmarks
                    and index < len(result.hand_world_landmarks)
                ):
                    orientation_hand = result.hand_world_landmarks[index]
                else:
                    orientation_hand = image_hand

                draw_skeleton(frame, image_hand, width, height, style)
                palm_px = draw_palm_width_line(
                    frame,
                    image_hand,
                    width,
                    height,
                    style,
                )
                calibration_candidates.append(palm_px)

                center_landmark = image_hand[9]
                pixel_x = int(center_landmark.x * width)
                pixel_y = int(center_landmark.y * height)

                dx_px = pixel_x - origin[0]
                dy_px = pixel_y - origin[1]

                camera_position = compute_camera_position(
                    palm_px,
                    dx_px,
                    dy_px,
                    focal_length,
                )

                draw_origin_to_hand_line(
                    frame,
                    origin,
                    (pixel_x, pixel_y),
                    label,
                    camera_position[0],
                    camera_position[1],
                )

                hand_pose = classify_hand_pose(image_hand)

                raw_palm_score = compute_palm_camera_score(
                    orientation_hand,
                    label,
                )
                palm_direction, filtered_palm_score = palm_filters[label].update(
                    raw_palm_score
                )

                detected_hands[label] = {
                    "pose": hand_pose,
                    "palm_direction": palm_direction,
                    "palm_score": filtered_palm_score,
                    "camera_position": camera_position,
                    "pixel": (pixel_x, pixel_y),
                    "palm_px": palm_px,
                }

                draw_text(
                    frame,
                    f"{label.upper()}  {hand_pose}  "
                    f"{palm_direction} {filtered_palm_score:+.2f}",
                    (pixel_x + 12, max(182, pixel_y - 12)),
                    style["accent"],
                    0.50,
                    2,
                )

        calibration_palm_px = (
            max(calibration_candidates)
            if calibration_candidates
            else 0.0
        )

        now = time.monotonic()

        for label in ("Left", "Right"):
            info = detected_hands.get(label)

            if info is None:
                gesture_trackers[label].cancel_candidate()
                draw_missing_hand_panel(
                    frame,
                    label,
                    gesture_trackers[label].mode,
                )
                continue

            hand_pos = None
            ee_pos = None

            camera_x, camera_y, camera_distance = info["camera_position"]

            # 캘리브레이션 이후에는 모드와 무관하게 좌표를 계산하여
            # RAW/EE 오버레이를 항상 보여준다.
            if camera_x is not None:
                hand_pos, ee_pos = processors[label].update(
                    camera_x,
                    camera_y,
                    camera_distance,
                )

            mode = gesture_trackers[label].mode
            candidate_mode = None
            elapsed = 0.0

            if recording:
                changed, elapsed, candidate_mode = gesture_trackers[label].update(
                    info["pose"],
                    info["palm_direction"],
                )
                mode = gesture_trackers[label].mode

                if changed:
                    publish_mode(label, mode)
                    print(
                        f"[{label}] MODE -> {mode} "
                        f"(palm={info['palm_direction']}, pose={info['pose']})"
                    )

                # 1초마다 현재 모드를 다시 보내 후발 구독자/일시적 패킷 문제를 방지한다.
                if now - last_mode_heartbeat[label] >= 1.0:
                    publish_mode(label, mode)
                    last_mode_heartbeat[label] = now

                if info["palm_direction"] != last_palm_direction[label]:
                    publish_palm_direction(label, info["palm_direction"])
                    last_palm_direction[label] = info["palm_direction"]

                # 로봇으로 보내는 좌표는 FOLLOW일 때만 발행한다.
                if (
                    mode == MODE_FOLLOW
                    and hand_pos is not None
                    and now - last_publish_time[label] >= PUB_INTERVAL
                ):
                    publish_coordinates(label, hand_pos, ee_pos)
                    last_publish_time[label] = now

            publishing = (
                recording
                and mode == MODE_FOLLOW
                and hand_pos is not None
            )

            draw_hand_panel(
                frame=frame,
                label=label,
                detected=info,
                mode=mode,
                candidate_mode=candidate_mode,
                elapsed=elapsed,
                hand_pos=hand_pos,
                ee_pos=ee_pos,
                publishing=publishing,
            )

        if calibration_step == 0:
            draw_calibration_banner(
                frame,
                "ANY HAND at 30cm, press SPACE",
            )
        elif calibration_step == 1:
            draw_calibration_banner(
                frame,
                "ANY HAND at 100cm, press SPACE",
            )
        elif calibration_step == 2 and not recording:
            draw_calibration_banner(
                frame,
                "press SPACE to START",
            )

        draw_footer(
            frame,
            fps=fps_value,
            recording=recording,
            calibration_step=calibration_step,
        )

        cv2.imshow("Dual Hand Tracker", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 32:
            if calibration_step == 0 and calibration_palm_px > 0:
                near_px = calibration_palm_px
                calibration_step = 1
                print(f"[CALIB] 30cm = {near_px:.1f}px")

            elif calibration_step == 1 and calibration_palm_px > 0:
                far_px = calibration_palm_px

                if near_px <= far_px:
                    print(
                        "[CALIB ERROR] 30cm palm width must be larger "
                        "than the 100cm palm width. Press R and retry."
                    )
                    continue

                focal_length = compute_focal(near_px, far_px)
                calibration_step = 2
                print(f"[CALIB] 100cm = {far_px:.1f}px")
                print(f"[CALIB] focal_length = {focal_length:.1f}")

            elif calibration_step == 2 and not recording:
                recording = True

                for hand_label in ("Left", "Right"):
                    gesture_trackers[hand_label].reset()
                    processors[hand_label].reset()
                    palm_filters[hand_label].reset()
                    publish_mode(hand_label, MODE_PLACE)
                    publish_palm_direction(hand_label, "SIDEWAYS")

                print("[START] Both hands begin in PLACE")

        elif key in (ord("r"), ord("R")):
            near_px = None
            far_px = None
            focal_length = None
            calibration_step = 0
            recording = False

            for hand_label in ("Left", "Right"):
                gesture_trackers[hand_label].reset()
                processors[hand_label].reset()
                palm_filters[hand_label].reset()
                last_palm_direction[hand_label] = None
                last_publish_time[hand_label] = 0.0
                last_mode_heartbeat[hand_label] = 0.0

            fps_value = 0.0
            fps_frames = 0
            fps_window_started = time.monotonic()

            print("[RESET]")

        elif key in (27, ord("q"), ord("Q")):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    detector.close()
    ros_node.destroy_node()
    rclpy.shutdown()



# import cv2
# import math
# import os
# import time
# from dataclasses import dataclass

# import mediapipe as mp
# import numpy as np
# from mediapipe.tasks import python
# from mediapipe.tasks.python import vision

# import rclpy
# from geometry_msgs.msg import Point
# from rclpy.node import Node
# from rclpy.qos import (
#     DurabilityPolicy,
#     HistoryPolicy,
#     QoSProfile,
#     ReliabilityPolicy,
# )
# from std_msgs.msg import String


# # =====================================================
# # Config
# # =====================================================
# BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# MODEL_PATH = os.getenv(
#     "HAND_MODEL_PATH",
#     os.path.join(BASE_DIR, "hand_landmarker.task"),
# )

# # Iriun Webcam: 현재 확인한 장치 번호가 4.
# # 실행 시 CAMERA_SOURCE=다른번호 로 덮어쓸 수 있다.
# CAMERA_SOURCE = os.getenv("CAMERA_SOURCE", "4")
# CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "1280"))
# CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "720"))
# DISPLAY_WIDTH = int(os.getenv("DISPLAY_WIDTH", "960"))
# DISPLAY_HEIGHT = int(os.getenv("DISPLAY_HEIGHT", "540"))

# # 화면을 거울처럼 좌우 반전할지 여부
# MIRROR_VIEW = os.getenv("MIRROR_VIEW", "1") == "1"

# # 거리 보정
# REAL_PALM_WIDTH = 0.09
# NEAR_CM = 30.0
# FAR_CM = 100.0

# # 좌표 발행 주기
# PUB_INTERVAL = 1.0 / 30.0

# # 손 위치 -> 로봇 EE 목표 위치 오프셋
# EE_Y_OFFSET = 0.25
# EE_Z_OFFSET = 0.05
# TABLE_HEIGHT = 1.0

# # 카메라와 가까울수록 높은 Z, 멀수록 낮은 Z
# HAND_Z_NEAR = 0.45
# HAND_Z_FAR = 0.05

# # 제스처
# GESTURE_HOLD_SEC = 1.5
# MODE_FOLLOW = "FOLLOW"
# MODE_PLACE = "PLACE"

# # 손바닥 앞/뒤 판정
# # 반대로 잡히면 실행할 때 PALM_Z_SIGN=-1 사용
# PALM_Z_SIGN = float(os.getenv("PALM_Z_SIGN", "1.0"))

# # 정면/후면으로 확실히 인정하는 각도 기준.
# # 1.0에 가까울수록 카메라 축과 거의 평행해야 한다.
# PALM_ENTER_THRESHOLD = 0.55
# PALM_EXIT_THRESHOLD = 0.30
# PALM_FILTER_ALPHA = 0.25
# PALM_STABLE_SEC = 0.20

# # 카메라 중심에서 손 중심까지 이어주는 선
# ORIGIN_LINE_COLOR = (175, 184, 196)

# HAND_CONNECTIONS = [
#     (0, 1), (1, 2), (2, 3), (3, 4),
#     (0, 5), (5, 6), (6, 7), (7, 8),
#     (5, 9), (9, 10), (10, 11), (11, 12),
#     (9, 13), (13, 14), (14, 15), (15, 16),
#     (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
# ]

# # OpenCV는 BGR 순서
# HAND_STYLE = {
#     "Right": {
#         "accent": (168, 236, 92),     # mint
#         "bone": (140, 214, 94),
#         "joint": (190, 255, 122),
#         "label": (205, 255, 155),
#     },
#     "Left": {
#         "accent": (255, 168, 96),     # electric blue
#         "bone": (236, 146, 76),
#         "joint": (255, 194, 126),
#         "label": (255, 210, 160),
#     },
# }

# PALM_LINE_COLOR = (235, 190, 92)

# # Modern overlay palette
# UI_CARD = (25, 28, 34)
# UI_CARD_2 = (32, 36, 44)
# UI_BORDER = (65, 71, 82)
# UI_TEXT = (240, 243, 247)
# UI_MUTED = (164, 172, 184)
# UI_GREEN = (122, 226, 126)
# UI_AMBER = (74, 187, 255)
# UI_RED = (102, 102, 245)
# UI_BLUE = (255, 164, 82)
# UI_DARK = (13, 15, 19)

# ROS_DOMAIN_DISPLAY = os.getenv("ROS_DOMAIN_ID", "unset")


# # =====================================================
# # Camera
# # =====================================================
# def parse_camera_source(value: str):
#     value = value.strip()
#     return int(value) if value.isdigit() else value


# def open_camera(source):
#     camera = cv2.VideoCapture(source)
#     camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
#     camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
#     camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
#     return camera


# # =====================================================
# # MediaPipe
# # =====================================================
# if not os.path.isfile(MODEL_PATH):
#     raise FileNotFoundError(f"MediaPipe model not found: {MODEL_PATH}")

# options = vision.HandLandmarkerOptions(
#     base_options=python.BaseOptions(model_asset_path=MODEL_PATH),
#     num_hands=2,
#     min_hand_detection_confidence=0.55,
#     min_hand_presence_confidence=0.55,
#     min_tracking_confidence=0.55,
# )
# detector = vision.HandLandmarker.create_from_options(options)

# camera_source = parse_camera_source(CAMERA_SOURCE)
# cap = open_camera(camera_source)

# if not cap.isOpened():
#     raise RuntimeError(
#         f"Camera open failed: CAMERA_SOURCE={CAMERA_SOURCE}. "
#         "Run `v4l2-ctl --list-devices` and check the Iriun device number."
#     )


# # =====================================================
# # ROS2
# # =====================================================
# rclpy.init()
# ros_node = Node("dual_hand_publisher")

# stream_qos = QoSProfile(
#     reliability=ReliabilityPolicy.BEST_EFFORT,
#     history=HistoryPolicy.KEEP_LAST,
#     depth=1,
# )

# # 상태/방향 토픽은 나중에 구독한 노드도 마지막 값을 받을 수 있게 유지한다.
# state_qos = QoSProfile(
#     reliability=ReliabilityPolicy.RELIABLE,
#     durability=DurabilityPolicy.TRANSIENT_LOCAL,
#     history=HistoryPolicy.KEEP_LAST,
#     depth=1,
# )


# @dataclass
# class HandPublishers:
#     raw: object
#     xyz: object
#     mode: object
#     palm: object


# publishers = {
#     "Left": HandPublishers(
#         raw=ros_node.create_publisher(Point, "/left_hand_raw", stream_qos),
#         xyz=ros_node.create_publisher(Point, "/left_hand_xyz", stream_qos),
#         mode=ros_node.create_publisher(String, "/left_hand_mode", state_qos),
#         palm=ros_node.create_publisher(String, "/left_palm_direction", state_qos),
#     ),
#     "Right": HandPublishers(
#         raw=ros_node.create_publisher(Point, "/right_hand_raw", stream_qos),
#         xyz=ros_node.create_publisher(Point, "/right_hand_xyz", stream_qos),
#         mode=ros_node.create_publisher(String, "/right_hand_mode", state_qos),
#         palm=ros_node.create_publisher(String, "/right_palm_direction", state_qos),
#     ),
# }

# # 기존 단일 오른손 코드와의 호환용
# legacy_raw_pub = ros_node.create_publisher(Point, "/hand_raw", stream_qos)
# legacy_xyz_pub = ros_node.create_publisher(Point, "/hand_xyz", stream_qos)
# legacy_mode_pub = ros_node.create_publisher(String, "/hand_mode", state_qos)


# # =====================================================
# # Coordinate processing
# # =====================================================
# class CoordProcessor:
#     def __init__(self, alpha=0.25):
#         self.alpha = alpha
#         self.filtered_ee = None
#         self.z_floor = TABLE_HEIGHT - 0.05

#     def reset(self):
#         self.filtered_ee = None

#     def update(self, camera_x, camera_y, camera_distance):
#         """
#         반환값:
#           hand_pos: 카메라 결과를 프로젝트 좌표로 바꾼 손 위치
#           ee_pos:   오프셋 + 스무딩이 적용된 로봇 EE 목표 위치
#         """
#         near_m = NEAR_CM / 100.0
#         far_m = FAR_CM / 100.0

#         distance = float(np.clip(camera_distance, near_m, far_m))
#         ratio = (distance - near_m) / (far_m - near_m)

#         # 가까울수록 높고, 멀수록 낮다.
#         relative_z = HAND_Z_NEAR + ratio * (HAND_Z_FAR - HAND_Z_NEAR)

#         hand_x = float(camera_x)
#         hand_y = float(camera_y) - 0.30
#         hand_z = TABLE_HEIGHT + relative_z

#         hand_pos = np.array([hand_x, hand_y, hand_z], dtype=float)

#         unfiltered_ee = np.array(
#             [
#                 hand_x,
#                 hand_y + EE_Y_OFFSET,
#                 max(hand_z + EE_Z_OFFSET, self.z_floor),
#             ],
#             dtype=float,
#         )

#         if self.filtered_ee is None:
#             self.filtered_ee = unfiltered_ee.copy()
#         else:
#             self.filtered_ee = (
#                 self.alpha * unfiltered_ee
#                 + (1.0 - self.alpha) * self.filtered_ee
#             )

#         return hand_pos, self.filtered_ee.copy()


# processors = {
#     "Left": CoordProcessor(alpha=0.25),
#     "Right": CoordProcessor(alpha=0.25),
# }


# # =====================================================
# # Hand pose detection
# # =====================================================
# def classify_hand_pose(hand) -> str:
#     """
#     손 회전에 덜 민감하도록 손바닥 중심으로부터 손가락 끝과 PIP까지의
#     3차원 거리를 비교한다.

#     반환:
#       FIST  : 네 손가락 중 3개 이상 확실히 접힘
#       OPEN  : 네 손가락 중 3개 이상 확실히 펴짐
#       OTHER : 중간 자세, 반쯤 편 손 등
#     """
#     palm_center = np.mean(
#         np.array(
#             [[hand[i].x, hand[i].y, hand[i].z] for i in [0, 5, 9, 13, 17]],
#             dtype=float,
#         ),
#         axis=0,
#     )

#     tip_ids = [8, 12, 16, 20]
#     pip_ids = [6, 10, 14, 18]

#     curled = 0
#     extended = 0

#     for tip_id, pip_id in zip(tip_ids, pip_ids):
#         tip = np.array(
#             [hand[tip_id].x, hand[tip_id].y, hand[tip_id].z],
#             dtype=float,
#         )
#         pip = np.array(
#             [hand[pip_id].x, hand[pip_id].y, hand[pip_id].z],
#             dtype=float,
#         )

#         tip_distance = np.linalg.norm(tip - palm_center)
#         pip_distance = np.linalg.norm(pip - palm_center)

#         if tip_distance < pip_distance * 0.98:
#             curled += 1
#         elif tip_distance > pip_distance * 1.10:
#             extended += 1

#     if curled >= 3:
#         return "FIST"
#     if extended >= 3:
#         return "OPEN"
#     return "OTHER"


# # =====================================================
# # Palm-facing detection
# # =====================================================
# def _point3(landmarks, index):
#     point = landmarks[index]
#     return np.array([point.x, point.y, point.z], dtype=float)


# def compute_palm_camera_score(landmarks, label: str) -> float:
#     """
#     카메라 축에 대한 손바닥 법선 점수:
#       +1에 가까움: 손바닥이 카메라를 향함
#       -1에 가까움: 손등이 카메라를 향함
#        0에 가까움: 손이 옆으로 기울어짐

#     가능하면 MediaPipe world landmarks를 사용한다.
#     """
#     wrist = _point3(landmarks, 0)
#     index_mcp = _point3(landmarks, 5)
#     middle_mcp = _point3(landmarks, 9)
#     ring_mcp = _point3(landmarks, 13)
#     pinky_mcp = _point3(landmarks, 17)

#     # 손바닥 평면을 여러 삼각형으로 계산해 z 노이즈를 완화한다.
#     pairs = [
#         (index_mcp - wrist, middle_mcp - wrist),
#         (middle_mcp - wrist, ring_mcp - wrist),
#         (ring_mcp - wrist, pinky_mcp - wrist),
#         (index_mcp - wrist, pinky_mcp - wrist),
#     ]

#     normals = []

#     for vector_a, vector_b in pairs:
#         normal = np.cross(vector_a, vector_b)
#         norm = np.linalg.norm(normal)

#         if norm > 1e-8:
#             normals.append(normal / norm)

#     if not normals:
#         return 0.0

#     normal = np.mean(normals, axis=0)
#     normal_norm = np.linalg.norm(normal)

#     if normal_norm < 1e-8:
#         return 0.0

#     normal /= normal_norm

#     # 좌우 손의 관절 순서에 따른 법선 방향을 같은 기준으로 맞춘다.
#     if label == "Left":
#         normal *= -1.0

#     # MediaPipe 카메라 좌표에서 음의 z 방향을 카메라 쪽으로 사용.
#     # 실제 환경에서 반대면 PALM_Z_SIGN=-1로 실행한다.
#     camera_score = -float(normal[2]) * PALM_Z_SIGN
#     return float(np.clip(camera_score, -1.0, 1.0))


# class PalmFacingFilter:
#     """
#     순간적인 landmark 떨림 때문에 TOWARD/AWAY가 바뀌지 않도록
#     EMA + 진입/이탈 임계값 + 짧은 안정화 시간을 사용한다.
#     """
#     def __init__(self):
#         self.filtered_score = None
#         self.state = "SIDEWAYS"
#         self.candidate = None
#         self.candidate_started = None

#     def reset(self):
#         self.filtered_score = None
#         self.state = "SIDEWAYS"
#         self.candidate = None
#         self.candidate_started = None

#     def update(self, raw_score: float):
#         now = time.monotonic()

#         if self.filtered_score is None:
#             self.filtered_score = raw_score
#         else:
#             self.filtered_score = (
#                 PALM_FILTER_ALPHA * raw_score
#                 + (1.0 - PALM_FILTER_ALPHA) * self.filtered_score
#             )

#         score = self.filtered_score

#         # 현재 상태 유지 조건. 점수가 약해지면 SIDEWAYS로 바로 빠져
#         # 제스처 유지 타이머가 잘못 누적되지 않게 한다.
#         if self.state == "TOWARD_CAMERA" and score >= PALM_EXIT_THRESHOLD:
#             self.candidate = None
#             self.candidate_started = None
#             return self.state, score

#         if self.state == "AWAY_CAMERA" and score <= -PALM_EXIT_THRESHOLD:
#             self.candidate = None
#             self.candidate_started = None
#             return self.state, score

#         if self.state != "SIDEWAYS":
#             self.state = "SIDEWAYS"
#             self.candidate = None
#             self.candidate_started = None

#         if score >= PALM_ENTER_THRESHOLD:
#             desired = "TOWARD_CAMERA"
#         elif score <= -PALM_ENTER_THRESHOLD:
#             desired = "AWAY_CAMERA"
#         else:
#             self.candidate = None
#             self.candidate_started = None
#             return self.state, score

#         if self.candidate != desired:
#             self.candidate = desired
#             self.candidate_started = now
#             return self.state, score

#         if now - self.candidate_started >= PALM_STABLE_SEC:
#             self.state = desired
#             self.candidate = None
#             self.candidate_started = None

#         return self.state, score


# palm_filters = {
#     "Left": PalmFacingFilter(),
#     "Right": PalmFacingFilter(),
# }


# # =====================================================
# # Gesture mode
# # =====================================================
# class GestureModeTracker:
#     """
#     FOLLOW:
#       TOWARD_CAMERA + FIST 상태를 1.5초 연속 유지

#     PLACE:
#       AWAY_CAMERA + OPEN 상태를 1.5초 연속 유지

#     손바닥이 옆을 향하거나 손 자세가 조건과 달라지거나,
#     손 검출이 끊기면 유지 타이머를 즉시 초기화한다.
#     """
#     def __init__(self):
#         self.mode = MODE_PLACE
#         self.candidate_mode = None
#         self.candidate_started = None

#     def reset(self):
#         self.mode = MODE_PLACE
#         self.candidate_mode = None
#         self.candidate_started = None

#     def cancel_candidate(self):
#         self.candidate_mode = None
#         self.candidate_started = None

#     def update(self, hand_pose: str, palm_direction: str):
#         now = time.monotonic()

#         if hand_pose == "FIST" and palm_direction == "TOWARD_CAMERA":
#             desired_mode = MODE_FOLLOW
#         elif hand_pose == "OPEN" and palm_direction == "AWAY_CAMERA":
#             desired_mode = MODE_PLACE
#         else:
#             self.cancel_candidate()
#             return False, 0.0, None

#         if desired_mode == self.mode:
#             self.cancel_candidate()
#             return False, 0.0, desired_mode

#         if self.candidate_mode != desired_mode:
#             self.candidate_mode = desired_mode
#             self.candidate_started = now
#             return False, 0.0, desired_mode

#         elapsed = now - self.candidate_started

#         if elapsed >= GESTURE_HOLD_SEC:
#             self.mode = desired_mode
#             self.cancel_candidate()
#             return True, GESTURE_HOLD_SEC, desired_mode

#         return False, elapsed, desired_mode


# gesture_trackers = {
#     "Left": GestureModeTracker(),
#     "Right": GestureModeTracker(),
# }


# # =====================================================
# # Drawing helpers — modern UI
# # =====================================================
# def draw_rounded_rect(image, pt1, pt2, color, radius=16, thickness=-1):
#     x1, y1 = pt1
#     x2, y2 = pt2
#     radius = max(1, min(radius, (x2 - x1) // 2, (y2 - y1) // 2))

#     if thickness < 0:
#         cv2.rectangle(image, (x1 + radius, y1), (x2 - radius, y2), color, -1)
#         cv2.rectangle(image, (x1, y1 + radius), (x2, y2 - radius), color, -1)
#         cv2.circle(image, (x1 + radius, y1 + radius), radius, color, -1)
#         cv2.circle(image, (x2 - radius, y1 + radius), radius, color, -1)
#         cv2.circle(image, (x1 + radius, y2 - radius), radius, color, -1)
#         cv2.circle(image, (x2 - radius, y2 - radius), radius, color, -1)
#         return

#     cv2.line(image, (x1 + radius, y1), (x2 - radius, y1), color, thickness, cv2.LINE_AA)
#     cv2.line(image, (x1 + radius, y2), (x2 - radius, y2), color, thickness, cv2.LINE_AA)
#     cv2.line(image, (x1, y1 + radius), (x1, y2 - radius), color, thickness, cv2.LINE_AA)
#     cv2.line(image, (x2, y1 + radius), (x2, y2 - radius), color, thickness, cv2.LINE_AA)
#     cv2.ellipse(image, (x1 + radius, y1 + radius), (radius, radius), 180, 0, 90, color, thickness, cv2.LINE_AA)
#     cv2.ellipse(image, (x2 - radius, y1 + radius), (radius, radius), 270, 0, 90, color, thickness, cv2.LINE_AA)
#     cv2.ellipse(image, (x2 - radius, y2 - radius), (radius, radius), 0, 0, 90, color, thickness, cv2.LINE_AA)
#     cv2.ellipse(image, (x1 + radius, y2 - radius), (radius, radius), 90, 0, 90, color, thickness, cv2.LINE_AA)


# def draw_text(frame, text, position, color=UI_TEXT, scale=0.46, thickness=1):
#     cv2.putText(
#         frame,
#         text,
#         position,
#         cv2.FONT_HERSHEY_DUPLEX,
#         scale,
#         color,
#         thickness,
#         cv2.LINE_AA,
#     )


# def draw_pill(frame, text, x, y, bg_color, text_color=UI_TEXT, scale=0.40):
#     (text_w, text_h), _ = cv2.getTextSize(
#         text,
#         cv2.FONT_HERSHEY_DUPLEX,
#         scale,
#         1,
#     )
#     width = text_w + 20
#     height = 24
#     draw_rounded_rect(frame, (x, y), (x + width, y + height), bg_color, 12, -1)
#     draw_text(
#         frame,
#         text,
#         (x + 10, y + 16 + max(0, (text_h - 10) // 2)),
#         text_color,
#         scale,
#         1,
#     )
#     return width


# def draw_skeleton(frame, hand, width, height, style):
#     points = [(int(lm.x * width), int(lm.y * height)) for lm in hand]

#     for start, end in HAND_CONNECTIONS:
#         cv2.line(frame, points[start], points[end], UI_DARK, 5, cv2.LINE_AA)
#         cv2.line(
#             frame,
#             points[start],
#             points[end],
#             style["bone"],
#             2,
#             cv2.LINE_AA,
#         )

#     for index, (x, y) in enumerate(points):
#         radius = 6 if index in (0, 9) else 4
#         cv2.circle(frame, (x, y), radius + 3, UI_DARK, -1, cv2.LINE_AA)
#         cv2.circle(frame, (x, y), radius, style["joint"], -1, cv2.LINE_AA)


# def draw_palm_width_line(frame, hand, width, height):
#     x1, y1 = int(hand[5].x * width), int(hand[5].y * height)
#     x2, y2 = int(hand[17].x * width), int(hand[17].y * height)

#     cv2.line(frame, (x1, y1), (x2, y2), UI_DARK, 7, cv2.LINE_AA)
#     cv2.line(frame, (x1, y1), (x2, y2), PALM_LINE_COLOR, 3, cv2.LINE_AA)

#     return math.hypot(x2 - x1, y2 - y1)


# def draw_origin_to_hand_line(frame, origin, hand_center, label, dx_px, dy_px):
#     accent = HAND_STYLE[label]["accent"]

#     cv2.line(frame, origin, hand_center, UI_DARK, 5, cv2.LINE_AA)
#     cv2.line(frame, origin, hand_center, ORIGIN_LINE_COLOR, 2, cv2.LINE_AA)
#     cv2.circle(frame, hand_center, 7, UI_DARK, -1, cv2.LINE_AA)
#     cv2.circle(frame, hand_center, 4, accent, -1, cv2.LINE_AA)

#     midpoint = (
#         (origin[0] + hand_center[0]) // 2,
#         (origin[1] + hand_center[1]) // 2,
#     )

#     tag = f"{dx_px:+d}, {dy_px:+d}px"
#     (tw, _), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_DUPLEX, 0.38, 1)
#     tag_x = midpoint[0] - tw // 2
#     tag_y = midpoint[1] - 22

#     draw_rounded_rect(
#         frame,
#         (tag_x - 8, tag_y - 15),
#         (tag_x + tw + 8, tag_y + 6),
#         UI_CARD,
#         8,
#         -1,
#     )
#     draw_text(frame, tag, (tag_x, tag_y), accent, 0.38, 1)


# def draw_center_origin(frame, center):
#     x, y = center
#     cv2.circle(frame, (x, y), 11, UI_DARK, -1, cv2.LINE_AA)
#     cv2.circle(frame, (x, y), 8, (235, 239, 244), 1, cv2.LINE_AA)
#     cv2.circle(frame, (x, y), 2, (235, 239, 244), -1, cv2.LINE_AA)


# def draw_coordinate_box(frame, x, y, width, title, values, accent):
#     draw_rounded_rect(
#         frame,
#         (x, y),
#         (x + width, y + 42),
#         UI_CARD_2,
#         10,
#         -1,
#     )
#     draw_rounded_rect(
#         frame,
#         (x, y),
#         (x + width, y + 42),
#         UI_BORDER,
#         10,
#         1,
#     )
#     draw_text(frame, title, (x + 10, y + 16), UI_MUTED, 0.34, 1)
#     draw_text(frame, values, (x + 10, y + 34), accent, 0.42, 1)


# def draw_hand_panel(
#     frame,
#     label,
#     detected,
#     mode,
#     candidate_mode,
#     elapsed,
#     hand_pos,
#     ee_pos,
#     publishing,
# ):
#     _, width = frame.shape[:2]
#     style = HAND_STYLE[label]

#     card_width = width // 2 - 18
#     card_height = 178
#     card_x = 8 if label == "Left" else width // 2 + 10
#     card_y = 8

#     # Translucent card
#     overlay = frame.copy()
#     draw_rounded_rect(
#         overlay,
#         (card_x, card_y),
#         (card_x + card_width, card_y + card_height),
#         UI_CARD,
#         18,
#         -1,
#     )
#     cv2.addWeighted(overlay, 0.90, frame, 0.10, 0, frame)
#     draw_rounded_rect(
#         frame,
#         (card_x, card_y),
#         (card_x + card_width, card_y + card_height),
#         UI_BORDER,
#         18,
#         1,
#     )

#     # Accent rail
#     draw_rounded_rect(
#         frame,
#         (card_x + 7, card_y + 12),
#         (card_x + 11, card_y + card_height - 12),
#         style["accent"],
#         2,
#         -1,
#     )

#     draw_text(
#         frame,
#         f"{label.upper()} HAND",
#         (card_x + 22, card_y + 25),
#         UI_TEXT,
#         0.55,
#         2,
#     )

#     mode_color = UI_BLUE if mode == MODE_FOLLOW else UI_AMBER
#     pub_color = UI_GREEN if publishing else (58, 63, 72)

#     mode_w = draw_pill(
#         frame,
#         mode,
#         card_x + card_width - 168,
#         card_y + 10,
#         mode_color,
#         UI_DARK,
#         0.38,
#     )
#     draw_pill(
#         frame,
#         "LIVE" if publishing else "IDLE",
#         card_x + card_width - 82,
#         card_y + 10,
#         pub_color,
#         UI_DARK if publishing else UI_MUTED,
#         0.36,
#     )

#     pose = detected["pose"]
#     direction = detected["palm_direction"]
#     score = detected["palm_score"]

#     pose_color = UI_GREEN if pose == "OPEN" else UI_BLUE if pose == "FIST" else UI_MUTED
#     draw_pill(frame, pose, card_x + 22, card_y + 38, pose_color, UI_DARK, 0.35)

#     palm_text = f"{direction}  {score:+.2f}"
#     draw_text(
#         frame,
#         palm_text,
#         (card_x + 98, card_y + 55),
#         UI_MUTED,
#         0.40,
#         1,
#     )

#     box_width = (card_width - 54) // 2
#     if hand_pos is None:
#         raw_values = "calibration required"
#         ee_values = "calibration required"
#     else:
#         raw_values = f"{hand_pos[0]:+.3f}  {hand_pos[1]:+.3f}  {hand_pos[2]:.3f}"
#         ee_values = f"{ee_pos[0]:+.3f}  {ee_pos[1]:+.3f}  {ee_pos[2]:.3f}"

#     draw_coordinate_box(
#         frame,
#         card_x + 22,
#         card_y + 70,
#         box_width,
#         "HAND / RAW   X   Y   Z",
#         raw_values,
#         (126, 222, 255),
#     )
#     draw_coordinate_box(
#         frame,
#         card_x + 32 + box_width,
#         card_y + 70,
#         box_width,
#         "EE TARGET   X   Y   Z",
#         ee_values,
#         UI_GREEN,
#     )

#     camera_distance = detected["camera_position"][2]
#     if camera_distance is None:
#         meta = "DIST --   |   PIXEL --"
#     else:
#         meta = (
#             f"DIST {camera_distance:.3f} m   |   "
#             f"PIXEL {detected['pixel'][0]}, {detected['pixel'][1]}"
#         )
#     draw_text(frame, meta, (card_x + 22, card_y + 128), UI_MUTED, 0.37, 1)

#     if candidate_mode is None or candidate_mode == mode:
#         condition = "FOLLOW  front palm + fist   |   PLACE  back palm + open"
#         progress = 0.0
#     else:
#         required = (
#             "FRONT + FIST"
#             if candidate_mode == MODE_FOLLOW
#             else "BACK + OPEN"
#         )
#         condition = (
#             f"{candidate_mode}  {required}   "
#             f"{elapsed:.1f}/{GESTURE_HOLD_SEC:.1f}s"
#         )
#         progress = min(elapsed / GESTURE_HOLD_SEC, 1.0)

#     draw_text(frame, condition, (card_x + 22, card_y + 151), UI_TEXT, 0.36, 1)

#     bar_x1 = card_x + 22
#     bar_x2 = card_x + card_width - 18
#     bar_y1 = card_y + 161
#     bar_y2 = card_y + 168

#     draw_rounded_rect(
#         frame,
#         (bar_x1, bar_y1),
#         (bar_x2, bar_y2),
#         (49, 54, 63),
#         4,
#         -1,
#     )

#     if progress > 0.0:
#         fill_x = bar_x1 + max(8, int((bar_x2 - bar_x1) * progress))
#         draw_rounded_rect(
#             frame,
#             (bar_x1, bar_y1),
#             (fill_x, bar_y2),
#             style["accent"],
#             4,
#             -1,
#         )


# def draw_missing_hand_panel(frame, label, mode):
#     _, width = frame.shape[:2]
#     style = HAND_STYLE[label]
#     card_width = width // 2 - 18
#     card_height = 178
#     card_x = 8 if label == "Left" else width // 2 + 10
#     card_y = 8

#     overlay = frame.copy()
#     draw_rounded_rect(
#         overlay,
#         (card_x, card_y),
#         (card_x + card_width, card_y + card_height),
#         UI_CARD,
#         18,
#         -1,
#     )
#     cv2.addWeighted(overlay, 0.86, frame, 0.14, 0, frame)
#     draw_rounded_rect(
#         frame,
#         (card_x, card_y),
#         (card_x + card_width, card_y + card_height),
#         UI_BORDER,
#         18,
#         1,
#     )

#     draw_text(
#         frame,
#         f"{label.upper()} HAND",
#         (card_x + 22, card_y + 28),
#         UI_TEXT,
#         0.55,
#         2,
#     )
#     draw_pill(
#         frame,
#         mode,
#         card_x + card_width - 92,
#         card_y + 12,
#         UI_BLUE if mode == MODE_FOLLOW else UI_AMBER,
#         UI_DARK,
#         0.38,
#     )

#     draw_text(
#         frame,
#         "NO HAND DETECTED",
#         (card_x + 22, card_y + 85),
#         style["accent"],
#         0.70,
#         2,
#     )
#     draw_text(
#         frame,
#         "Gesture timer paused and reset",
#         (card_x + 22, card_y + 112),
#         UI_MUTED,
#         0.42,
#         1,
#     )


# def draw_calibration_banner(frame, text):
#     height, width = frame.shape[:2]
#     card_width = min(610, width - 60)
#     card_height = 76
#     x1 = (width - card_width) // 2
#     y1 = height // 2 - card_height // 2
#     x2 = x1 + card_width
#     y2 = y1 + card_height

#     overlay = frame.copy()
#     draw_rounded_rect(overlay, (x1, y1), (x2, y2), UI_CARD, 20, -1)
#     cv2.addWeighted(overlay, 0.93, frame, 0.07, 0, frame)
#     draw_rounded_rect(frame, (x1, y1), (x2, y2), UI_BLUE, 20, 1)

#     draw_text(frame, "CALIBRATION", (x1 + 24, y1 + 28), UI_BLUE, 0.48, 2)
#     draw_text(frame, text, (x1 + 24, y1 + 55), UI_TEXT, 0.58, 1)


# def draw_footer(frame, fps, recording, calibration_step):
#     height, width = frame.shape[:2]
#     bar_h = 30
#     y1 = height - bar_h

#     overlay = frame.copy()
#     cv2.rectangle(overlay, (0, y1), (width, height), UI_DARK, -1)
#     cv2.addWeighted(overlay, 0.88, frame, 0.12, 0, frame)

#     state = "RUNNING" if recording else f"CALIBRATION {calibration_step}/2"
#     state_color = UI_GREEN if recording else UI_AMBER

#     draw_text(frame, "DUAL HAND TELEOP", (14, height - 10), UI_TEXT, 0.42, 2)
#     draw_text(
#         frame,
#         f"ROS DOMAIN {ROS_DOMAIN_DISPLAY}",
#         (width // 2 - 105, height - 10),
#         UI_MUTED,
#         0.38,
#         1,
#     )
#     draw_text(
#         frame,
#         f"CAM {CAMERA_SOURCE}   {fps:4.1f} FPS",
#         (width - 180, height - 10),
#         UI_MUTED,
#         0.38,
#         1,
#     )
#     cv2.circle(frame, (width // 2 - 128, height - 14), 4, state_color, -1, cv2.LINE_AA)


# # =====================================================
# # Math / publishing helpers
# # =====================================================
# def compute_camera_position(palm_px, dx_px, dy_px, focal_length):
#     """
#     카메라 중심 기준 손 위치:
#       x/y: 손바닥 폭으로 환산한 화면 중심 대비 미터 거리
#       z:   핀홀 모델로 계산한 카메라-손 거리
#     """
#     if focal_length is None or palm_px <= 0:
#         return None, None, None

#     distance_m = focal_length * REAL_PALM_WIDTH / palm_px
#     meter_per_pixel = REAL_PALM_WIDTH / palm_px

#     return (
#         dx_px * meter_per_pixel,
#         -dy_px * meter_per_pixel,
#         distance_m,
#     )


# def compute_focal(near_px, far_px):
#     f_near = near_px * (NEAR_CM / 100.0) / REAL_PALM_WIDTH
#     f_far = far_px * (FAR_CM / 100.0) / REAL_PALM_WIDTH
#     return (f_near + f_far) / 2.0


# def publish_mode(label, mode):
#     publishers[label].mode.publish(String(data=mode))

#     if label == "Right":
#         legacy_mode_pub.publish(String(data=mode))


# def publish_palm_direction(label, direction):
#     publishers[label].palm.publish(String(data=direction))


# def publish_coordinates(label, hand_pos, ee_pos):
#     """
#     ROS Point에는 float64 전체 정밀도로 발행한다.
#     화면 오버레이에서만 소수점 셋째 자리로 보기 좋게 표시한다.
#     """
#     raw_message = Point(
#         x=float(hand_pos[0]),
#         y=float(hand_pos[1]),
#         z=float(hand_pos[2]),
#     )
#     xyz_message = Point(
#         x=float(ee_pos[0]),
#         y=float(ee_pos[1]),
#         z=float(ee_pos[2]),
#     )

#     publishers[label].raw.publish(raw_message)
#     publishers[label].xyz.publish(xyz_message)

#     if label == "Right":
#         legacy_raw_pub.publish(raw_message)
#         legacy_xyz_pub.publish(xyz_message)


# # =====================================================
# # Runtime state
# # =====================================================
# near_px = None
# far_px = None
# focal_length = None
# calibration_step = 0
# recording = False

# last_publish_time = {
#     "Left": 0.0,
#     "Right": 0.0,
# }
# last_palm_direction = {
#     "Left": None,
#     "Right": None,
# }
# last_mode_heartbeat = {
#     "Left": 0.0,
#     "Right": 0.0,
# }

# fps_value = 0.0
# fps_frames = 0
# fps_window_started = time.monotonic()


# print("=" * 78)
# print("Dual Hand Tracker - modern overlay / refined gesture")
# print(f"Camera source: {CAMERA_SOURCE}")
# print("SPACE: 30cm calibration -> 100cm calibration -> start")
# print("R: reset | Q/ESC: quit")
# print("FOLLOW: palm toward camera + fist for 1.5 sec")
# print("PLACE : palm away from camera + open hand for 1.5 sec")
# print("Near hand -> high Z | Far hand -> low Z")
# print("=" * 78)


# try:
#     while True:
#         frame_ok, frame = cap.read()

#         if not frame_ok:
#             print("[ERROR] Camera frame read failed")
#             break

#         frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))

#         fps_frames += 1
#         fps_now = time.monotonic()
#         fps_elapsed = fps_now - fps_window_started
#         if fps_elapsed >= 0.5:
#             fps_value = fps_frames / fps_elapsed
#             fps_frames = 0
#             fps_window_started = fps_now

#         if MIRROR_VIEW:
#             frame = cv2.flip(frame, 1)

#         height, width = frame.shape[:2]
#         origin = (width // 2, height // 2)

#         rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         mp_image = mp.Image(
#             image_format=mp.ImageFormat.SRGB,
#             data=rgb,
#         )
#         result = detector.detect(mp_image)

#         draw_center_origin(frame, origin)

#         detected_hands = {}
#         calibration_candidates = []

#         if result.hand_landmarks:
#             for index, image_hand in enumerate(result.hand_landmarks):
#                 raw_label = result.handedness[index][0].category_name

#                 if MIRROR_VIEW:
#                     label = "Right" if raw_label == "Left" else "Left"
#                 else:
#                     label = raw_label

#                 style = HAND_STYLE[label]

#                 # 손바닥 앞/뒤 판정에는 world landmark가 더 안정적이다.
#                 if (
#                     hasattr(result, "hand_world_landmarks")
#                     and result.hand_world_landmarks
#                     and index < len(result.hand_world_landmarks)
#                 ):
#                     orientation_hand = result.hand_world_landmarks[index]
#                 else:
#                     orientation_hand = image_hand

#                 draw_skeleton(frame, image_hand, width, height, style)
#                 palm_px = draw_palm_width_line(
#                     frame,
#                     image_hand,
#                     width,
#                     height,
#                 )
#                 calibration_candidates.append(palm_px)

#                 center_landmark = image_hand[9]
#                 pixel_x = int(center_landmark.x * width)
#                 pixel_y = int(center_landmark.y * height)

#                 dx_px = pixel_x - origin[0]
#                 dy_px = pixel_y - origin[1]

#                 draw_origin_to_hand_line(
#                     frame,
#                     origin,
#                     (pixel_x, pixel_y),
#                     label,
#                     dx_px,
#                     dy_px,
#                 )

#                 hand_pose = classify_hand_pose(image_hand)

#                 raw_palm_score = compute_palm_camera_score(
#                     orientation_hand,
#                     label,
#                 )
#                 palm_direction, filtered_palm_score = palm_filters[label].update(
#                     raw_palm_score
#                 )

#                 camera_position = compute_camera_position(
#                     palm_px,
#                     dx_px,
#                     dy_px,
#                     focal_length,
#                 )

#                 detected_hands[label] = {
#                     "pose": hand_pose,
#                     "palm_direction": palm_direction,
#                     "palm_score": filtered_palm_score,
#                     "camera_position": camera_position,
#                     "pixel": (pixel_x, pixel_y),
#                     "palm_px": palm_px,
#                 }

#                 draw_text(
#                     frame,
#                     f"{label.upper()}  {hand_pose}  "
#                     f"{palm_direction} {filtered_palm_score:+.2f}",
#                     (pixel_x + 12, max(182, pixel_y - 12)),
#                     style["label"],
#                     0.50,
#                     2,
#                 )

#         calibration_palm_px = (
#             max(calibration_candidates)
#             if calibration_candidates
#             else 0.0
#         )

#         now = time.monotonic()

#         for label in ("Left", "Right"):
#             info = detected_hands.get(label)

#             if info is None:
#                 gesture_trackers[label].cancel_candidate()
#                 draw_missing_hand_panel(
#                     frame,
#                     label,
#                     gesture_trackers[label].mode,
#                 )
#                 continue

#             hand_pos = None
#             ee_pos = None

#             camera_x, camera_y, camera_distance = info["camera_position"]

#             # 캘리브레이션 이후에는 모드와 무관하게 좌표를 계산하여
#             # RAW/EE 오버레이를 항상 보여준다.
#             if camera_x is not None:
#                 hand_pos, ee_pos = processors[label].update(
#                     camera_x,
#                     camera_y,
#                     camera_distance,
#                 )

#             mode = gesture_trackers[label].mode
#             candidate_mode = None
#             elapsed = 0.0

#             if recording:
#                 changed, elapsed, candidate_mode = gesture_trackers[label].update(
#                     info["pose"],
#                     info["palm_direction"],
#                 )
#                 mode = gesture_trackers[label].mode

#                 if changed:
#                     publish_mode(label, mode)
#                     print(
#                         f"[{label}] MODE -> {mode} "
#                         f"(palm={info['palm_direction']}, pose={info['pose']})"
#                     )

#                 # 1초마다 현재 모드를 다시 보내 후발 구독자/일시적 패킷 문제를 방지한다.
#                 if now - last_mode_heartbeat[label] >= 1.0:
#                     publish_mode(label, mode)
#                     last_mode_heartbeat[label] = now

#                 if info["palm_direction"] != last_palm_direction[label]:
#                     publish_palm_direction(label, info["palm_direction"])
#                     last_palm_direction[label] = info["palm_direction"]

#                 # 로봇으로 보내는 좌표는 FOLLOW일 때만 발행한다.
#                 if (
#                     mode == MODE_FOLLOW
#                     and hand_pos is not None
#                     and now - last_publish_time[label] >= PUB_INTERVAL
#                 ):
#                     publish_coordinates(label, hand_pos, ee_pos)
#                     last_publish_time[label] = now

#             publishing = (
#                 recording
#                 and mode == MODE_FOLLOW
#                 and hand_pos is not None
#             )

#             draw_hand_panel(
#                 frame=frame,
#                 label=label,
#                 detected=info,
#                 mode=mode,
#                 candidate_mode=candidate_mode,
#                 elapsed=elapsed,
#                 hand_pos=hand_pos,
#                 ee_pos=ee_pos,
#                 publishing=publishing,
#             )

#         if calibration_step == 0:
#             draw_calibration_banner(
#                 frame,
#                 "ANY HAND at 30cm, press SPACE",
#             )
#         elif calibration_step == 1:
#             draw_calibration_banner(
#                 frame,
#                 "ANY HAND at 100cm, press SPACE",
#             )
#         elif calibration_step == 2 and not recording:
#             draw_calibration_banner(
#                 frame,
#                 "press SPACE to START",
#             )

#         draw_footer(
#             frame,
#             fps=fps_value,
#             recording=recording,
#             calibration_step=calibration_step,
#         )

#         cv2.imshow("Dual Hand Tracker", frame)
#         key = cv2.waitKey(1) & 0xFF

#         if key == 32:
#             if calibration_step == 0 and calibration_palm_px > 0:
#                 near_px = calibration_palm_px
#                 calibration_step = 1
#                 print(f"[CALIB] 30cm = {near_px:.1f}px")

#             elif calibration_step == 1 and calibration_palm_px > 0:
#                 far_px = calibration_palm_px

#                 if near_px <= far_px:
#                     print(
#                         "[CALIB ERROR] 30cm palm width must be larger "
#                         "than the 100cm palm width. Press R and retry."
#                     )
#                     continue

#                 focal_length = compute_focal(near_px, far_px)
#                 calibration_step = 2
#                 print(f"[CALIB] 100cm = {far_px:.1f}px")
#                 print(f"[CALIB] focal_length = {focal_length:.1f}")

#             elif calibration_step == 2 and not recording:
#                 recording = True

#                 for hand_label in ("Left", "Right"):
#                     gesture_trackers[hand_label].reset()
#                     processors[hand_label].reset()
#                     palm_filters[hand_label].reset()
#                     publish_mode(hand_label, MODE_PLACE)
#                     publish_palm_direction(hand_label, "SIDEWAYS")

#                 print("[START] Both hands begin in PLACE")

#         elif key in (ord("r"), ord("R")):
#             near_px = None
#             far_px = None
#             focal_length = None
#             calibration_step = 0
#             recording = False

#             for hand_label in ("Left", "Right"):
#                 gesture_trackers[hand_label].reset()
#                 processors[hand_label].reset()
#                 palm_filters[hand_label].reset()
#                 last_palm_direction[hand_label] = None
#                 last_publish_time[hand_label] = 0.0
#                 last_mode_heartbeat[hand_label] = 0.0

#             fps_value = 0.0
#             fps_frames = 0
#             fps_window_started = time.monotonic()

#             print("[RESET]")

#         elif key in (27, ord("q"), ord("Q")):
#             break

# finally:
#     cap.release()
#     cv2.destroyAllWindows()
#     detector.close()
#     ros_node.destroy_node()
#     rclpy.shutdown()