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

SUPPORTED_TRAY_COMMANDS = (
    4,
    5,
    6,
    7,
)

# 팀원 코드의 4, 5, 6, 7 위치를 그대로 사용한다.
# 여기서 z는 큐브 중심이 아니라 기존 트레이 생성 기준 높이다.
TRAY_SPAWN_POSITIONS = {
    4: (
        0.24,
        0.55,
        1.05,
    ),
    5: (
        0.24,
        0.85,
        1.05,
    ),
    6: (
        0.72,
        0.55,
        1.05,
    ),
    7: (
        0.72,
        0.85,
        1.05,
    ),
}

# 회전 검증을 위한 임시 yaw.
# 실제 동적 생성 코드가 들어오면 그 코드가 설정한 quaternion을 사용한다.
TEMP_TRAY_YAW_DEGREES = {
    4: 0.0,
    5: 15.0,
    6: -20.0,
    7: 35.0,
}

# 흡착 테스트용 임시 큐브 크기.
# 실제 트레이보다 XY 접촉 면적을 넉넉하게 키운다.
# 높이는 기존 트레이와 동일하게 유지해서 PICK/PLACE 높이는 바꾸지 않는다.
TEMP_TRAY_SIZE = (
    0.300,
    0.220,
    0.01861830,
)

TEMP_TRAY_MASS = 0.15

# 임시 큐브 생성 기능을 쉽게 제거하기 위한 스위치.
ENABLE_TEMP_DYNAMIC_TRAYS = True

# 트래킹 구역과 트레이 구역 사이의 임시 대기 위치
STAGING_POSITION = (
    0.10,
    0.50,
    1.35,
)

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

PICK_APPROACH_Z_CORRECTION = 0.042

PLACE_LINK6_ABOVE_TRAY = 0.136
PLACE_HIGH_OFFSET = 0.10
PLACE_APPROACH_GAP = 0.05
PLACE_MOVE_TOLERANCE = 0.04


# ============================================================
# 일반 제어 설정
# ============================================================
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