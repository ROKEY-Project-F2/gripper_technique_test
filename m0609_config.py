# m0609_config.py
from pathlib import Path


_THIS_DIR = Path(__file__).resolve().parent


# ============================================================
# 전체 Scene USD
# ============================================================

ROBOT_USD_PATH = str(
    _THIS_DIR
    / "Collected_full_scene"
    / "full_scene.usda"
)


# ============================================================
# Robot A / Robot B Prim 및 Scene 이름
# ============================================================

ROBOT_A_PRIM_PATH = "/World/m0609_01"
ROBOT_A_SCENE_NAME = "m0609_robot_a"

ROBOT_B_PRIM_PATH = "/World/m0609"
ROBOT_B_SCENE_NAME = "m0609_robot_b"

EE_LINK_NAME = "link_6"


# ============================================================
# Surface Gripper 설정
# ============================================================

def _surface_gripper_paths(
    robot_prim_path: str,
):
    base_path = (
        f"{robot_prim_path}"
        "/onrobot_rg2ft/gripper_body/"
        "dual_suction_tool"
    )

    return [
        (
            f"{base_path}"
            "/suction_contact_left/"
            "SurfaceGripper_left"
        ),
        (
            f"{base_path}"
            "/suction_contact_right/"
            "SurfaceGripper_right"
        ),
    ]


ROBOT_A_SURFACE_GRIPPER_PATHS = (
    _surface_gripper_paths(
        ROBOT_A_PRIM_PATH
    )
)

ROBOT_B_SURFACE_GRIPPER_PATHS = (
    _surface_gripper_paths(
        ROBOT_B_PRIM_PATH
    )
)

SURFACE_GRIPPER_WRITE_STATUS_TO_USD = True


# ============================================================
# 로봇 Drive 설정
# ============================================================

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8


# ============================================================
# RMPflow 설정
# ============================================================

RMPFLOW_DIR = str(
    _THIS_DIR
    / "rmpflow"
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
#
# 실제 Robot A/B base pose는 main.py에서 각 Prim의
# world pose를 읽어 자동으로 계산한다.
# ============================================================

CUROBO_ROBOT_CONFIG_PATH = str(
    _THIS_DIR
    / "m0609_v1.yml"
)

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
# 실제 트레이 / 수술 도구 동적 생성 설정
# ============================================================

TABLE_HEIGHT = 1.0

TRAY_USD_PATH = str(
    _THIS_DIR
    / "Collected_model_redtray_scaled_for_180mm_pads"
    / "model_redtray_scaled_for_180mm_pads.usda"
)

TOOL_DIR = (
    _THIS_DIR
    / "SurgicalInstruments_A"
    / "Model"
)

TOOL_USDS = (
    str(
        TOOL_DIR
        / "sm_bipolardissectingscissors_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_caliper_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_clamps_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_forceps_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_handsaws_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_knife_a01_01.usd"
    ),
)

TOOL_NAMES = (
    "바이폴라",
    "캘리퍼",
    "클램프",
    "겸자",
    "톱",
    "메스",
)

TOOL_DROP_HEIGHT = 0.05
TOOL_MASS = 0.001

TOOL_SCALES = (
    (0.0060, 0.0060, 0.0060),
    (0.0060, 0.0060, 0.0060),
    (0.0075, 0.0075, 0.0075),
    (0.0075, 0.0075, 0.0075),
    (0.0035, 0.0035, 0.0035),
    (0.0040, 0.0040, 0.0040),
)

TRAY_TOP_Z = 0.0186
TRAY_Z = 1.05

TRAY_ORIENTATION = (
    0.0,
    0.0,
    0.0,
    1.0,
)

# 새 2열×3행 배치.
#
# 화면 기준:
#
#   4  5
#   2  3
# A 0  1 B
#
# 업로드된 full_scene.usda 기준:
# Robot A = (-0.55, 0.50, 1.00)
# Robot B = ( 0.55, 0.50, 1.00)
#
# Robot A/B의 y=0.50이 0·1행과 2·3행 사이에 오도록
# 첫 행은 y=0.35, 둘째 행은 y=0.65로 배치한다.
# 행 간격은 0.25 m, 열 중심 간격은 0.25 m이다.
#
TRAY_SPAWN_POSITIONS = {
    0: (-0.125, 0.40, TRAY_Z),
    1: ( 0.125, 0.40, TRAY_Z),
    2: (-0.125, 0.65, TRAY_Z),
    3: ( 0.125, 0.65, TRAY_Z),
    4: (-0.125, 0.90, TRAY_Z),
    5: ( 0.125, 0.90, TRAY_Z),
}

# 두 로봇 모두 모든 트레이에 접근할 수 있다.
ROBOT_A_SUPPORTED_TRAY_COMMANDS = (
    0,
    1,
    2,
    3,
    4,
    5,
)

ROBOT_B_SUPPORTED_TRAY_COMMANDS = (
    0,
    1,
    2,
    3,
    4,
    5,
)



# ============================================================
# IDLE 관절 자세
#
# full_scene(3).usda에 저장된 현재 관절 targetPosition과 동일하다.
# 단위는 degree이며 main.py에서 radian으로 변환해서 사용한다.
#
# Robot A = /World/m0609_01
# Robot B = /World/m0609
# ============================================================

ROBOT_A_IDLE_JOINT_POSITIONS_DEG = (
    88.0,
    0.1,
    -90.0,
    0.0,
    -96.0,
    0.0,
)

ROBOT_B_IDLE_JOINT_POSITIONS_DEG = (
    -96.0,
    3.7,
    88.1,
    0.0,
    100.200005,
    -300.6,
)

# ============================================================
# 로봇 위쪽 중간 경유지 설정
#
# 현재 배치:
#
#     4 5
#   a 2 3 b
#   A 0 1 B
#
# a, b는 각 로봇 베이스를 기준으로 Y축 +0.30m 지점이다.
# X 좌표는 각 로봇 베이스와 동일하게 유지한다.
#
# 중간 경유지 도착 후 각 로봇은 바깥쪽 방향으로 180도 회전한다.
# Robot A: +180도
# Robot B: -180도
# ============================================================

TRANSIT_HEIGHT = 1.35

ROBOT_TRANSIT_Y_OFFSET = 0.30

ROBOT_A_TRACKING_JOINT1_DELTA_DEG = 180.0
ROBOT_B_TRACKING_JOINT1_DELTA_DEG = -180.0

JOINT1_TURN_TOLERANCE_DEG = 1.0
JOINT1_TURN_MAX_STEP_DEG = 1.00

SAFE_JOINT_RETURN_MAX_STEP_DEG = 0.35
SAFE_JOINT_RETURN_TOLERANCE_DEG = 1.0

RETURN_HOME_MAX_STEP_DEG = 0.20
RETURN_HOME_WRIST_MAX_STEP_DEG = 1.00
RETURN_HOME_TOLERANCE_DEG = 1.0


# ============================================================
# PICK / PLACE 설정
# ============================================================

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

PICK_APPROACH_Z_CORRECTION = 0.032

TRANSPORT_Z_OFFSET = 0.10

PLACE_LINK6_ABOVE_TRAY = 0.136
PLACE_HIGH_OFFSET = 0.15
PLACE_APPROACH_GAP = 0.015
PLACE_MOVE_TOLERANCE = 0.04

PLACE_RELEASE_MIN_WAIT_FRAMES = 30
PLACE_RELEASE_STABLE_FRAMES = 10
PLACE_RELEASE_RETRY_INTERVAL = 15
PLACE_RELEASE_TIMEOUT_FRAMES = 120


# ============================================================
# 일반 제어 설정
# ============================================================

INITIAL_SETTLING_FRAMES = 30


# ============================================================
# 설정 파일 검증
# ============================================================

_REQUIRED_FILES = [
    (
        "ROBOT_USD_PATH",
        ROBOT_USD_PATH,
    ),
    (
        "M0609_URDF_PATH",
        M0609_URDF_PATH,
    ),
    (
        "M0609_DESCRIPTION_PATH",
        M0609_DESCRIPTION_PATH,
    ),
    (
        "M0609_RMPFLOW_CONFIG_PATH",
        M0609_RMPFLOW_CONFIG_PATH,
    ),
    (
        "CUROBO_ROBOT_CONFIG_PATH",
        CUROBO_ROBOT_CONFIG_PATH,
    ),
    (
        "TRAY_USD_PATH",
        TRAY_USD_PATH,
    ),
]

for setting_name, setting_path in _REQUIRED_FILES:
    if not Path(setting_path).is_file():
        raise FileNotFoundError(
            f"{setting_name} 파일을 찾을 수 없습니다: "
            f"{setting_path}"
        )

for tool_index, tool_path in enumerate(
    TOOL_USDS
):
    if not Path(tool_path).is_file():
        raise FileNotFoundError(
            f"TOOL_USDS[{tool_index}] "
            "파일을 찾을 수 없습니다: "
            f"{tool_path}"
        )