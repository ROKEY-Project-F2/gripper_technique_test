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
    str(TOOL_DIR / "sm_bipolardissectingscissors_a01_01.usd"),
    str(TOOL_DIR / "sm_caliper_a01_01.usd"),
    str(TOOL_DIR / "sm_clamps_a01_01.usd"),
    str(TOOL_DIR / "sm_forceps_a01_01.usd"),
    str(TOOL_DIR / "sm_handsaws_a01_01.usd"),
    str(TOOL_DIR / "sm_knife_a01_01.usd"),
    str(TOOL_DIR / "sm_ligatureneedle_a01_01.usd"),
    str(TOOL_DIR / "sm_mallet_a01_01.usd"),
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

# 수술 도구 USD는 원본 에셋 단위가 커서 meter stage에 맞게 축소한다.
# 도구별로 독립 조절할 수 있도록 분리했다.
# 현재는 8종 모두 1/100 스케일을 사용한다.
TOOL_SCALES = (
    (0.0060, 0.0060, 0.0060),  # 0 바이폴라: 추가 축소
    (0.0060, 0.0060, 0.0060),  # 1 캘리퍼: 추가 축소
    (0.0075, 0.0075, 0.0075),  # 2 클램프
    (0.0075, 0.0075, 0.0075),  # 3 겸자
    (0.0035, 0.0035, 0.0035),  # 4 톱
    (0.0040, 0.0040, 0.0040),  # 5 메스
    (0.0075, 0.0075, 0.0075),  # 6 갈고리
    (0.0045, 0.0045, 0.0045),  # 7 망치: 확대
)

TRAY_TOP_Z = 0.0186
TRAY_Z = 1.05

# 공간 배치를 위해 팀원 코드의 90도 초기 회전을 그대로 사용한다.
# 모든 트레이에 동일하게 적용하며 랜덤 회전은 사용하지 않는다.
TRAY_ORIENTATION = (
    0.7071,
    0.0,
    0.0,
    0.7071,
)

# 팀원 코드의 트레이 8개 배치 순서:
# x 하나마다 y=0.55, 0.85 순서
TRAY_SPAWN_POSITIONS = {
    0: (-0.72, 0.55, TRAY_Z),
    1: (-0.72, 0.85, TRAY_Z),
    2: (-0.24, 0.55, TRAY_Z),
    3: (-0.24, 0.85, TRAY_Z),
    4: (0.24, 0.55, TRAY_Z),
    5: (0.24, 0.85, TRAY_Z),
    6: (0.72, 0.55, TRAY_Z),
    7: (0.72, 0.85, TRAY_Z),
}

# 현재 프로젝트는 우측의 단일 로봇을 사용하므로
# 실제 작업 명령은 기존과 동일하게 4~7만 허용한다.
SUPPORTED_TRAY_COMMANDS = (
    4,
    5,
    6,
    7,
)

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

PICK_APPROACH_Z_CORRECTION = 0.032

# PICK 완료 후 경유 위치로 이동하거나 PLACE를 위해 복귀할 때
# 기본 경유 위치보다 추가로 올릴 높이.
TRANSPORT_Z_OFFSET = 0.10

PLACE_LINK6_ABOVE_TRAY = 0.136

# PLACE 직전 트레이 상부 접근 높이.
PLACE_HIGH_OFFSET = 0.15

# 실제 그리퍼를 열 때 트레이 표면 위 여유 높이.
# 기존 5 cm에서 1.5 cm로 낮춰 도구를 떨어뜨리는 느낌을 줄인다.
PLACE_APPROACH_GAP = 0.015

PLACE_MOVE_TOLERANCE = 0.04

# Surface Gripper 해제 안정화 설정
#
# physics step 기준 값이다. 시뮬레이션이 60 Hz라면:
# 30 frame ≈ 0.5초, 120 frame ≈ 2초.
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
    ("TRAY_USD_PATH", TRAY_USD_PATH),
]

for setting_name, setting_path in _REQUIRED_FILES:
    if not Path(setting_path).is_file():
        raise FileNotFoundError(
            f"{setting_name} 파일을 찾을 수 없습니다: "
            f"{setting_path}"
        )

for tool_index, tool_path in enumerate(TOOL_USDS):
    if not Path(tool_path).is_file():
        raise FileNotFoundError(
            f"TOOL_USDS[{tool_index}] 파일을 찾을 수 없습니다: "
            f"{tool_path}"
        )