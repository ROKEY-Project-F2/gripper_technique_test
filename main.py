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
    ENABLE_TEMP_DYNAMIC_TRAYS,
    INITIAL_SETTLING_FRAMES,
    M0609_DESCRIPTION_PATH,
    M0609_RMPFLOW_CONFIG_PATH,
    M0609_URDF_PATH,
    PICK_APPROACH_Z_CORRECTION,
    PICK_DEFAULT_EE_OFFSET,
    PICK_EVENTS_DT,
    PLACE_APPROACH_GAP,
    PLACE_HIGH_OFFSET,
    PLACE_LINK6_ABOVE_TRAY,
    PLACE_MOVE_TOLERANCE,
    RMPFLOW_DIR,
    ROBOT_BASE_POSITION,
    ROBOT_BASE_YAW_DEG,
    ROBOT_PRIM_PATH,
    ROBOT_SCENE_NAME,
    ROBOT_USD_PATH,
    STAGING_POSITION,
    SUPPORTED_TRAY_COMMANDS,
    TABLE_HEIGHT,
    TEMP_TRAY_MASS,
    TEMP_TRAY_SIZE,
    TEMP_TRAY_YAW_DEGREES,
    TRACKING_MAX_JOINT_STEP,
    TRACKING_TOOL_ORIENTATION,
    TRACKING_USE_MPC,
    TRACKING_Z_MAX,
    TRACKING_Z_MIN,
    TRAY_SPAWN_POSITIONS,
)


if RMPFLOW_DIR not in sys.path:
    sys.path.insert(
        0,
        RMPFLOW_DIR,
    )


from hand_marker_visualizer import HandMarkerVisualizer
from m0609_move_controller import M0609MoveController
from m0609_pick_place_controller_surface import PickPlaceController
from m0609_ros_bridge import setup_m0609_ros_bridge
from m0609_state_machine import M0609StateMachine
from m0609_task import M0609BasicTask, initialize_robot
from m0609_tracking_controller import M0609TrackingController
from temp_dynamic_trays import create_temp_dynamic_trays


def _open_full_scene() -> None:
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

    # ========================================================
    # 임시 동적 큐브 생성
    # 실제 트레이 생성 코드가 준비되면 이 블록만 교체한다.
    # ========================================================
    if not ENABLE_TEMP_DYNAMIC_TRAYS:
        raise RuntimeError(
            "현재 테스트에서는 "
            "ENABLE_TEMP_DYNAMIC_TRAYS=True가 필요합니다."
        )

    tray_registry = create_temp_dynamic_trays(
        world=world,
        spawn_positions=TRAY_SPAWN_POSITIONS,
        yaw_degrees=TEMP_TRAY_YAW_DEGREES,
        tray_size=TEMP_TRAY_SIZE,
        mass=TEMP_TRAY_MASS,
    )

    # Scene에 새 rigid body를 추가했으므로 물리 뷰를 다시 초기화한다.
    world.reset()
    initialize_robot(
        robot,
        world,
    )
    tray_registry.reset_to_spawn()

    tracking_controller = M0609TrackingController(
        robot=robot,
        robot_config_path=(
            CUROBO_ROBOT_CONFIG_PATH
        ),
        base_position=ROBOT_BASE_POSITION,
        base_yaw_deg=ROBOT_BASE_YAW_DEG,
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

    pick_place_controller = PickPlaceController(
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
        end_effector_frame_name=EE_LINK_NAME,
    )

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
        end_effector_frame_name=EE_LINK_NAME,
        tcp_offset_local=np.zeros(
            3,
            dtype=np.float64,
        ),
        position_tolerance=(
            PLACE_MOVE_TOLERANCE
        ),
    )

    state_machine = M0609StateMachine(
        robot=robot,
        tray_registry=tray_registry,
        tracking_controller=(
            tracking_controller
        ),
        pick_place_controller=(
            pick_place_controller
        ),
        move_controller=move_controller,
        staging_position=STAGING_POSITION,
        staging_orientation=(
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
        supported_tray_commands=(
            SUPPORTED_TRAY_COMMANDS
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

    print("\n[M0609 동적 큐브 테스트 준비 완료]")
    print("- 지원 명령: 4, 5, 6, 7")
    print(
        "- 예:"
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

            tray_registry.reset_to_spawn()
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