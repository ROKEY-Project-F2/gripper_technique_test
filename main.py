# main.py

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": False,
    }
)


from isaacsim.core.utils.extensions import enable_extension


enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.robot.surface_gripper")
enable_extension("omni.graph.scriptnode")

for _ in range(10):
    simulation_app.update()


import sys
import time

import numpy as np
import omni.usd

from isaacsim.core.api import World


from m0609_config import (
    CUROBO_ROBOT_CONFIG_PATH,
    EE_LINK_NAME,
    INITIAL_SETTLING_FRAMES,
    M0609_DESCRIPTION_PATH,
    M0609_RMPFLOW_CONFIG_PATH,
    M0609_URDF_PATH,
    PICK_APPROACH_Z_CORRECTION,
    PICK_DEFAULT_EE_OFFSET,
    PICK_EVENTS_DT,
    PICK_POSITION,
    PLACE_APPROACH_GAP,
    PLACE_HIGH_OFFSET,
    PLACE_JOINT_STEP,
    PLACE_JOINT_TOLERANCE,
    PLACE_LINK6_ABOVE_TRAY,
    PLACE_MOVE_TOLERANCE,
    PLACE_SETTLE_FRAMES,
    RMPFLOW_DIR,
    ROBOT_BASE_POSITION,
    ROBOT_BASE_YAW_DEG,
    ROBOT_PRIM_PATH,
    ROBOT_SCENE_NAME,
    ROBOT_USD_PATH,
    STAGING_POSITION,
    TABLE_HEIGHT,
    TARGET_TRAY_INDEX,
    TRACKING_MAX_JOINT_STEP,
    TRACKING_TOOL_ORIENTATION,
    TRACKING_USE_MPC,
    TRACKING_Z_MAX,
    TRACKING_Z_MIN,
    TRAY_7_POSITION,
)


if RMPFLOW_DIR not in sys.path:
    sys.path.insert(
        0,
        RMPFLOW_DIR,
    )


from hand_marker_visualizer import HandMarkerVisualizer
from m0609_pick_place_controller_surface import PickPlaceController
from m0609_move_controller import M0609MoveController
from m0609_ros_bridge import setup_m0609_ros_bridge
from m0609_state_machine import M0609StateMachine
from m0609_task import M0609BasicTask, initialize_robot
from m0609_tracking_controller import M0609TrackingController


def _open_full_scene() -> None:
    print(
        f"[main] 전체 Scene 열기: "
        f"{ROBOT_USD_PATH}",
        flush=True,
    )

    result = (
        omni.usd.get_context().open_stage(
            ROBOT_USD_PATH
        )
    )

    if result is False:
        raise RuntimeError(
            f"Stage를 열지 못했습니다: "
            f"{ROBOT_USD_PATH}"
        )

    for _ in range(80):
        simulation_app.update()

    stage = (
        omni.usd.get_context().get_stage()
    )

    if not stage.GetPrimAtPath(
        ROBOT_PRIM_PATH
    ).IsValid():
        raise RuntimeError(
            "로봇 Prim을 찾지 못했습니다: "
            f"{ROBOT_PRIM_PATH}"
        )


def main() -> None:
    _open_full_scene()

    world = World(
        stage_units_in_meters=1.0,
    )

    task = M0609BasicTask()
    world.add_task(task)
    world.reset()

    robot = world.scene.get_object(
        ROBOT_SCENE_NAME
    )

    if robot is None:
        raise RuntimeError(
            "Scene 로봇 객체 없음: "
            f"{ROBOT_SCENE_NAME}"
        )

    initialize_robot(
        robot,
        world,
    )

    # --------------------------------------------------------
    # 손 추종 컨트롤러
    # --------------------------------------------------------
    tracking_controller = (
        M0609TrackingController(
            robot=robot,
            robot_config_path=(
                CUROBO_ROBOT_CONFIG_PATH
            ),
            base_position=(
                ROBOT_BASE_POSITION
            ),
            base_yaw_deg=(
                ROBOT_BASE_YAW_DEG
            ),
            tool_orientation=(
                TRACKING_TOOL_ORIENTATION
            ),
            z_min=TRACKING_Z_MIN,
            z_max=TRACKING_Z_MAX,
            max_joint_step=(
                TRACKING_MAX_JOINT_STEP
            ),
            use_mpc=TRACKING_USE_MPC,
        )
    )

    # --------------------------------------------------------
    # PICK 컨트롤러
    # --------------------------------------------------------
    pick_place_controller = (
        PickPlaceController(
            name="pick_place_controller",
            gripper=robot.gripper,
            robot_articulation=robot,
            end_effector_initial_height=(
                TABLE_HEIGHT + 0.20
            ),
            events_dt=list(PICK_EVENTS_DT),
            urdf_path=M0609_URDF_PATH,
            robot_description_path=(
                M0609_DESCRIPTION_PATH
            ),
            rmpflow_config_path=(
                M0609_RMPFLOW_CONFIG_PATH
            ),
            end_effector_frame_name=(
                EE_LINK_NAME
            ),
        )
    )

    # --------------------------------------------------------
    # 임시구역 / PLACE 이동 컨트롤러
    # --------------------------------------------------------
    move_controller = M0609MoveController(
        name="workflow_move_controller",
        robot_articulation=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=(
            M0609_DESCRIPTION_PATH
        ),
        rmpflow_config_path=(
            M0609_RMPFLOW_CONFIG_PATH
        ),
        end_effector_frame_name=(
            EE_LINK_NAME
        ),
        tcp_offset_local=np.zeros(
            3,
            dtype=np.float64,
        ),
        position_tolerance=(
            PLACE_MOVE_TOLERANCE
        ),
    )

    # --------------------------------------------------------
    # 상태 머신
    # --------------------------------------------------------
    state_machine = M0609StateMachine(
        robot=robot,
        tracking_controller=(
            tracking_controller
        ),
        pick_place_controller=(
            pick_place_controller
        ),
        move_controller=move_controller,
        pick_position=PICK_POSITION,
        tray_position=TRAY_7_POSITION,
        staging_position=STAGING_POSITION,
        tool_orientation=(
            TRACKING_TOOL_ORIENTATION
        ),
        pick_default_ee_offset=(
            PICK_DEFAULT_EE_OFFSET
        ),
        pick_approach_z_correction=(
            PICK_APPROACH_Z_CORRECTION
        ),
        place_link6_above_tray=(
            PLACE_LINK6_ABOVE_TRAY
        ),
        place_high_offset=(
            PLACE_HIGH_OFFSET
        ),
        place_approach_gap=(
            PLACE_APPROACH_GAP
        ),
        place_joint_tolerance=(
            PLACE_JOINT_TOLERANCE
        ),
        place_joint_step=(
            PLACE_JOINT_STEP
        ),
        place_settle_frames=(
            PLACE_SETTLE_FRAMES
        ),
        supported_tray_index=(
            TARGET_TRAY_INDEX
        ),
    )

    setup_m0609_ros_bridge(
        state_machine=state_machine,
        simulation_app=simulation_app,
    )

    hand_marker = HandMarkerVisualizer(
        world=world,
        initial_position=[
            0.0,
            0.25,
            1.0,
        ],
        radius=0.03,
    )

    for _ in range(
        INITIAL_SETTLING_FRAMES
    ):
        world.step(
            render=True,
        )

    print("\n[M0609 준비 완료]")
    print("- 초기 상태: IDLE")
    print(
        "- 임시구역:",
        STAGING_POSITION,
    )
    print(
        "- 시작 명령:"
        " ros2 topic pub --once"
        " /m0609/pick_command"
        " std_msgs/msg/Int32"
        " \"{data: 7}\""
    )
    print(
        "- 트래킹 중지:"
        " /hand_mode = HOME"
    )
    print()

    was_playing = False

    while simulation_app.is_running():
        world.step(
            render=True,
        )

        time.sleep(0.01)

        is_playing = world.is_playing()

        if (
            is_playing
            and not was_playing
        ):
            world.reset()

            initialize_robot(
                robot,
                world,
            )

            tracking_controller.reset()
            state_machine.reset()
            hand_marker.reset()

            print(
                "[main] Play 재초기화 완료",
                flush=True,
            )

        if is_playing:
            hand_marker.update()
            state_machine.step()

        was_playing = is_playing


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()