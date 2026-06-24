# m0609_config.py

from pathlib import Path


_THIS_DIR = Path(__file__).resolve().parent


# ============================================================
# 전체 Scene USD / 로봇 Prim 설정
# ============================================================
ROBOT_USD_PATH = str(
    _THIS_DIR
    / "Collected_full_scene"
    / "full_scene.usda"
)

ROBOT_PRIM_PATH = "/World/m0609"
ROBOT_SCENE_NAME = "m0609_robot"
EE_LINK_NAME = "link_6"


# ============================================================
# Surface Gripper 설정
# ============================================================
_SURFACE_GRIPPER_BASE_PATH = (
    f"{ROBOT_PRIM_PATH}"
    "/onrobot_rg2ft/gripper_body/dual_suction_tool"
)

SURFACE_GRIPPER_PATHS = [
    (
        f"{_SURFACE_GRIPPER_BASE_PATH}"
        "/suction_contact_left/SurfaceGripper_left"
    ),
    (
        f"{_SURFACE_GRIPPER_BASE_PATH}"
        "/suction_contact_right/SurfaceGripper_right"
    ),
]

SURFACE_GRIPPER_WRITE_STATUS_TO_USD = True


# ============================================================
# 로봇 Drive 설정
# ============================================================
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8


# ============================================================
# RMPFlow 설정
# ============================================================
RMPFLOW_DIR = str(
    _THIS_DIR / "rmpflow"
)

M0609_URDF_PATH = str(
    _THIS_DIR
    / "doosan-robot2"
    / "urdf"
    / "m0609_isaac_sim.urdf"
)

M0609_DESCRIPTION_PATH = str(
    _THIS_DIR
    / "rmpflow"
    / "m0609_description.yaml"
)

M0609_RMPFLOW_CONFIG_PATH = str(
    _THIS_DIR
    / "rmpflow"
    / "m0609_rmpflow_common.yaml"
)


# ============================================================
# cuRobo 손 추종 설정
# ============================================================
CUROBO_ROBOT_CONFIG_PATH = str(
    _THIS_DIR / "m0609_v1.yml"
)

ROBOT_BASE_POSITION = (
    0.5,
    0.2,
    1.0,
)

ROBOT_BASE_YAW_DEG = 90.0

# 팀원 코드의 TOOL_ORIENTATION과 동일
TRACKING_TOOL_ORIENTATION = (
    0.0,
    0.0,
    1.0,
    0.0,
)

TRACKING_Z_MIN = 1.10
TRACKING_Z_MAX = 1.55
TRACKING_MAX_JOINT_STEP = 0.02
TRACKING_USE_MPC = True


# ============================================================
# PICK / PLACE 설정
# ============================================================
TABLE_HEIGHT = 1.0

# 외부 명령으로 현재 허용하는 트레이 번호
TARGET_TRAY_INDEX = 7

# 팀원 코드의 tray 7 위치
TRAY_7_POSITION = (
    0.72,
    0.85,
    1.05,
)

TRAY_TOP_Z = 0.0186

# PickPlaceController에 전달할 트레이 윗면 위치
PICK_POSITION = (
    TRAY_7_POSITION[0],
    TRAY_7_POSITION[1],
    TRAY_7_POSITION[2] + TRAY_TOP_Z,
)

# 트래킹 구역과 트레이 구역 사이의 임시 대기 위치.
# 로봇 베이스 바로 위를 피하기 위해 x를 왼쪽으로 배치했다.
STAGING_POSITION = (
    0.10,
    0.50,
    1.35,
)

STAGING_ORIENTATION = TRACKING_TOOL_ORIENTATION

# PickPlaceController의 기존 이벤트 속도 설정
PICK_EVENTS_DT = (
    0.008,
    0.005,
    0.02,
    0.15,
    0.0025,
    0.01,
    0.0025,
    1.0,
    0.008,
    0.08,
)

PICK_DEFAULT_EE_OFFSET = (
    0.0,
    0.0,
    0.20,
)

# 팀원 코드와 동일하게 접근 이벤트에서 보정
PICK_APPROACH_Z_CORRECTION = 0.042

# PLACE 시 link_6 기준 좌표
PLACE_LINK6_ABOVE_TRAY = 0.136
PLACE_HIGH_OFFSET = 0.10
PLACE_APPROACH_GAP = 0.05

PLACE_MOVE_TOLERANCE = 0.04
PLACE_JOINT_TOLERANCE = 0.02
PLACE_JOINT_STEP = 0.015
PLACE_SETTLE_FRAMES = 10


# ============================================================
# 일반 제어 설정
# ============================================================
POSITION_TOLERANCE = 0.01
INITIAL_SETTLING_FRAMES = 30


# ============================================================
# 설정 파일 검증
# ============================================================
_REQUIRED_FILES = [
    ("ROBOT_USD_PATH", ROBOT_USD_PATH),
    ("M0609_URDF_PATH", M0609_URDF_PATH),
    ("M0609_DESCRIPTION_PATH", M0609_DESCRIPTION_PATH),
    (
        "M0609_RMPFLOW_CONFIG_PATH",
        M0609_RMPFLOW_CONFIG_PATH,
    ),
    (
        "CUROBO_ROBOT_CONFIG_PATH",
        CUROBO_ROBOT_CONFIG_PATH,
    ),
]

for setting_name, setting_path in _REQUIRED_FILES:
    if not Path(setting_path).is_file():
        raise FileNotFoundError(
            f"{setting_name} 파일을 찾을 수 없습니다: "
            f"{setting_path}"
        )