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
#
# 실제 Robot A/B base pose는 main.py에서 각 Prim의
# world pose를 읽어 자동으로 계산한다.
# ============================================================
CUROBO_ROBOT_CONFIG_PATH = str(
    _THIS_DIR / "m0609_v1.yml"
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
    str(
        TOOL_DIR
        / "sm_ligatureneedle_a01_01.usd"
    ),
    str(
        TOOL_DIR
        / "sm_mallet_a01_01.usd"
    ),
)

TOOL_NAMES = (
    "바이폴라",
    "캘리퍼",
    "클램프",
    "겸자",
    "톱",
    "메스",
    "갈고리",
    "망치",
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
    (0.0075, 0.0075, 0.0075),
    (0.0045, 0.0045, 0.0045),
)

TRAY_TOP_Z = 0.0186
TRAY_Z = 1.05

TRAY_ORIENTATION = (
    0.7071,
    0.0,
    0.0,
    0.7071,
)

TRAY_SPAWN_POSITIONS = {
    # 전체 배열 폭을 줄여 트레이를 중앙 쪽으로 배치한다.
    # x 간격은 0.40 m로 일정하게 유지한다.
    #
    #   1    3    5    7
    #   0    2    4    6
    #
    0: (-0.60, 0.55, TRAY_Z),
    1: (-0.60, 0.85, TRAY_Z),
    2: (-0.20, 0.55, TRAY_Z),
    3: (-0.20, 0.85, TRAY_Z),
    4: (0.20, 0.55, TRAY_Z),
    5: (0.20, 0.85, TRAY_Z),
    6: (0.60, 0.55, TRAY_Z),
    7: (0.60, 0.85, TRAY_Z),
}

ROBOT_A_SUPPORTED_TRAY_COMMANDS = (
    0,
    1,
    2,
    3,
)

ROBOT_B_SUPPORTED_TRAY_COMMANDS = (
    4,
    5,
    6,
    7,
)


# ============================================================
# 경유지 설정
#
# 경유지 2의 x, y는 main.py에서 Robot A와 Robot B 베이스의
# 정확한 중점으로 계산한다.
#
#   TRANSIT_2.xy = (Robot A base.xy + Robot B base.xy) / 2
#
# 경유지 1과 3은 각 로봇 베이스를 중심으로 경유지 2를
# 반대편에 같은 거리만큼 반사해 계산한다.
#
#   TRANSIT_1.xy = 2 * Robot A base.xy - TRANSIT_2.xy
#   TRANSIT_3.xy = 2 * Robot B base.xy - TRANSIT_2.xy
#
# 세 경유지의 높이는 동일하게 유지한다.
# ============================================================
TRANSIT_HEIGHT = 1.35

# 경유지 도착 후 TRACKING 시작 전 첫 번째 관절 회전량.
# Robot A/B 설치 방향이 반대이면 부호를 서로 바꾼다.
ROBOT_A_TRACKING_JOINT1_DELTA_DEG = 90.0
ROBOT_B_TRACKING_JOINT1_DELTA_DEG = -90.0

# joint1 회전 완료 판정 허용 오차.
JOINT1_TURN_TOLERANCE_DEG = 1.0

# joint1 회전 시 한 simulation step에서 허용할 최대 회전량.
# main loop가 약 100 Hz라면 0.25 deg/step은 약 25 deg/s이다.
JOINT1_TURN_MAX_STEP_DEG = 0.50

# PLACE 시 회전 완료 당시의 전체 관절 자세로 복귀할 때
# 한 simulation step에서 각 관절이 움직일 수 있는 최대 각도.
SAFE_JOINT_RETURN_MAX_STEP_DEG = 0.35

# 안전 관절 자세 도착 판정 허용 오차.
SAFE_JOINT_RETURN_TOLERANCE_DEG = 1.0

# PLACE 완료 후 최초 IDLE 관절 자세로 복귀할 때,
# 한 simulation step에서 각 관절이 움직일 수 있는 최대 각도.
# 값이 작을수록 천천히 복귀한다.
RETURN_HOME_MAX_STEP_DEG = 0.20

# RETURN_HOME 중 엔드이펙터 방향을 복원하는 손목 관절
# J4~J6의 최대 이동량. J1~J3보다 빠르게 복귀시킨다.
RETURN_HOME_WRIST_MAX_STEP_DEG = 1.00

# 최초 관절 자세 도착 판정 허용 오차.
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