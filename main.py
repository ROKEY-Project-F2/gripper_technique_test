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
from isaacsim.core.utils.rotations import (
    quat_to_euler_angles,
)

from m0609_config import (
    CUROBO_ROBOT_CONFIG_PATH,
    EE_LINK_NAME,
    INITIAL_SETTLING_FRAMES,
    JOINT1_TURN_MAX_STEP_DEG,
    JOINT1_TURN_TOLERANCE_DEG,
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
    PLACE_RELEASE_MIN_WAIT_FRAMES,
    PLACE_RELEASE_RETRY_INTERVAL,
    PLACE_RELEASE_STABLE_FRAMES,
    PLACE_RELEASE_TIMEOUT_FRAMES,
    RMPFLOW_DIR,
    ROBOT_A_PRIM_PATH,
    ROBOT_A_TRACKING_JOINT1_DELTA_DEG,
    ROBOT_A_SCENE_NAME,
    ROBOT_A_SUPPORTED_TRAY_COMMANDS,
    ROBOT_B_PRIM_PATH,
    ROBOT_B_TRACKING_JOINT1_DELTA_DEG,
    ROBOT_B_SCENE_NAME,
    ROBOT_B_SUPPORTED_TRAY_COMMANDS,
    RETURN_HOME_MAX_STEP_DEG,
    RETURN_HOME_WRIST_MAX_STEP_DEG,
    RETURN_HOME_TOLERANCE_DEG,
    SAFE_JOINT_RETURN_MAX_STEP_DEG,
    SAFE_JOINT_RETURN_TOLERANCE_DEG,
    ROBOT_USD_PATH,
    TRANSIT_HEIGHT,
    TABLE_HEIGHT,
    TOOL_DROP_HEIGHT,
    TOOL_MASS,
    TOOL_SCALES,
    TOOL_USDS,
    TRANSPORT_Z_OFFSET,
    TRAY_ORIENTATION,
    TRAY_SPAWN_POSITIONS,
    TRAY_TOP_Z,
    TRAY_USD_PATH,
    TRACKING_MAX_JOINT_STEP,
    TRACKING_TOOL_ORIENTATION,
    TRACKING_USE_MPC,
    TRACKING_Z_MAX,
    TRACKING_Z_MIN,
)

if RMPFLOW_DIR not in sys.path:
    sys.path.insert(
        0,
        RMPFLOW_DIR,
    )


from hand_input import CachedHandInput
from hand_marker_visualizer import (
    HandMarkerVisualizer,
)
from m0609_dynamic_scene import (
    create_surgical_trays_and_tools,
)
from m0609_move_controller import (
    M0609MoveController,
)
from m0609_pick_place_controller_surface import (
    PickPlaceController,
)
from m0609_ros_bridge import (
    get_latest_left_hand_mode,
    get_latest_left_hand_raw,
    get_latest_left_hand_target,
    get_latest_right_hand_mode,
    get_latest_right_hand_raw,
    get_latest_right_hand_target,
    reset_left_hand_mode_cache,
    reset_right_hand_mode_cache,
    setup_m0609_ros_bridge,
)
from m0609_state_machine import (
    M0609StateMachine,
)
from m0609_task import (
    M0609BasicTask,
    initialize_robot,
)
from m0609_tracking_controller import (
    M0609TrackingController,
)
from robot_manager import RobotManager
from robot_runtime import RobotProfile


def _open_full_scene() -> None:
    result = (
        omni.usd
        .get_context()
        .open_stage(ROBOT_USD_PATH)
    )

    if result is False:
        raise RuntimeError(
            "Stage를 열지 못했습니다: "
            f"{ROBOT_USD_PATH}"
        )

    for _ in range(80):
        simulation_app.update()

    stage = (
        omni.usd
        .get_context()
        .get_stage()
    )

    missing_robot_paths = [
        path
        for path in (
            ROBOT_A_PRIM_PATH,
            ROBOT_B_PRIM_PATH,
        )
        if not stage.GetPrimAtPath(path).IsValid()
    ]

    if missing_robot_paths:
        raise RuntimeError(
            "로봇 Prim을 찾지 못했습니다:\n- "
            + "\n- ".join(missing_robot_paths)
        )


def _get_robot_base_pose(
    robot,
):
    position, orientation = (
        robot.get_world_pose()
    )

    position = np.asarray(
        position,
        dtype=np.float64,
    )
    orientation = np.asarray(
        orientation,
        dtype=np.float64,
    )

    euler = quat_to_euler_angles(
        orientation
    )

    yaw_deg = float(
        np.degrees(euler[2])
    )

    return position, yaw_deg


def _mirror_transit_position(
    *,
    robot_base_position,
    center_transit_position,
) -> np.ndarray:
    """
    로봇 베이스를 중심으로 중앙 경유지를 반대편에 반사한다.

    x, y:
        mirrored = 2 * robot_base - center

    z:
        세 경유지 모두 중앙 경유지와 같은 높이를 사용한다.
    """
    base = np.asarray(
        robot_base_position,
        dtype=np.float64,
    )
    center = np.asarray(
        center_transit_position,
        dtype=np.float64,
    )

    mirrored = center.copy()
    mirrored[:2] = (
        2.0 * base[:2]
        - center[:2]
    )
    mirrored[2] = center[2]

    return mirrored


def _create_tracking_controller(
    *,
    robot,
    robot_id: str,
    base_position,
    base_yaw_deg: float,
):
    print(
        f"[main] Robot {robot_id} base pose: "
        f"position={base_position.round(4)}, "
        f"yaw={base_yaw_deg:.2f} deg",
        flush=True,
    )

    return M0609TrackingController(
        robot=robot,
        robot_config_path=(
            CUROBO_ROBOT_CONFIG_PATH
        ),
        base_position=base_position,
        base_yaw_deg=base_yaw_deg,
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


def _create_pick_place_controller(
    *,
    robot,
    robot_id: str,
):
    return PickPlaceController(
        name=(
            f"pick_place_controller_"
            f"{robot_id.lower()}"
        ),
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


def _create_move_controller(
    *,
    robot,
    robot_id: str,
):
    return M0609MoveController(
        name=(
            f"workflow_move_controller_"
            f"{robot_id.lower()}"
        ),
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


def _create_state_machine(
    *,
    robot_id: str,
    robot,
    tray_registry,
    hand_input,
    initial_joint_positions,
    tracking_controller,
    pick_place_controller,
    move_controller,
):
    return M0609StateMachine(
        robot_id=robot_id,
        robot=robot,
        tray_registry=tray_registry,
        hand_input=hand_input,
        idle_joint_positions=(
            initial_joint_positions
        ),
        tracking_controller=(
            tracking_controller
        ),
        pick_place_controller=(
            pick_place_controller
        ),
        move_controller=move_controller,
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
        transport_z_offset=(
            TRANSPORT_Z_OFFSET
        ),
        place_release_min_wait_frames=(
            PLACE_RELEASE_MIN_WAIT_FRAMES
        ),
        place_release_stable_frames=(
            PLACE_RELEASE_STABLE_FRAMES
        ),
        place_release_retry_interval=(
            PLACE_RELEASE_RETRY_INTERVAL
        ),
        place_release_timeout_frames=(
            PLACE_RELEASE_TIMEOUT_FRAMES
        ),
        joint1_turn_tolerance_rad=np.deg2rad(
            JOINT1_TURN_TOLERANCE_DEG
        ),
        joint1_turn_max_step_rad=np.deg2rad(
            JOINT1_TURN_MAX_STEP_DEG
        ),
        safe_joint_return_max_step_rad=np.deg2rad(
            SAFE_JOINT_RETURN_MAX_STEP_DEG
        ),
        safe_joint_return_tolerance_rad=np.deg2rad(
            SAFE_JOINT_RETURN_TOLERANCE_DEG
        ),
        return_home_max_step_rad=np.deg2rad(
            RETURN_HOME_MAX_STEP_DEG
        ),
        return_home_wrist_max_step_rad=np.deg2rad(
            RETURN_HOME_WRIST_MAX_STEP_DEG
        ),
        idle_joint_tolerance=np.deg2rad(
            RETURN_HOME_TOLERANCE_DEG
        ),
    )


def main() -> None:
    _open_full_scene()

    world = World(
        stage_units_in_meters=1.0,
    )

    task = M0609BasicTask()
    world.add_task(task)

    tray_registry = (
        create_surgical_trays_and_tools(
            world=world,
            tray_usd_path=TRAY_USD_PATH,
            tool_usds=TOOL_USDS,
            tray_positions=(
                TRAY_SPAWN_POSITIONS
            ),
            tray_orientation=(
                TRAY_ORIENTATION
            ),
            tray_top_z=TRAY_TOP_Z,
            tool_drop_height=(
                TOOL_DROP_HEIGHT
            ),
            tool_mass=TOOL_MASS,
            tool_scales=TOOL_SCALES,
            simulation_app=(
                simulation_app
            ),
        )
    )

    world.reset()

    robot_a = world.scene.get_object(
        ROBOT_A_SCENE_NAME
    )
    robot_b = world.scene.get_object(
        ROBOT_B_SCENE_NAME
    )

    if robot_a is None:
        raise RuntimeError(
            "Scene Robot A 객체 없음: "
            f"{ROBOT_A_SCENE_NAME}"
        )

    if robot_b is None:
        raise RuntimeError(
            "Scene Robot B 객체 없음: "
            f"{ROBOT_B_SCENE_NAME}"
        )

    initialize_robot(
        robot_a,
        world,
    )
    initialize_robot(
        robot_b,
        world,
    )

    initial_joint_positions_a = (
        np.asarray(
            robot_a.get_joint_positions(),
            dtype=np.float64,
        ).copy()
    )
    initial_joint_positions_b = (
        np.asarray(
            robot_b.get_joint_positions(),
            dtype=np.float64,
        ).copy()
    )

    robot_a_base_position, robot_a_base_yaw_deg = (
        _get_robot_base_pose(robot_a)
    )
    robot_b_base_position, robot_b_base_yaw_deg = (
        _get_robot_base_pose(robot_b)
    )

    # 중앙 경유지 2는 두 로봇 베이스의 정확한 중점에 둔다.
    # 높이는 기존 경유지 높이인 TRANSIT_HEIGHT를 유지한다.
    transit_2_position = np.array(
        [
            (
                robot_a_base_position[0]
                + robot_b_base_position[0]
            )
            / 2.0,
            (
                robot_a_base_position[1]
                + robot_b_base_position[1]
            )
            / 2.0,
            TRANSIT_HEIGHT,
        ],
        dtype=np.float64,
    )

    # 경유지 1과 3은 각 로봇 베이스를 중심으로
    # 경유지 2를 반대편에 같은 거리만큼 반사한다.
    transit_1_position = _mirror_transit_position(
        robot_base_position=robot_a_base_position,
        center_transit_position=transit_2_position,
    )
    transit_3_position = _mirror_transit_position(
        robot_base_position=robot_b_base_position,
        center_transit_position=transit_2_position,
    )



    print(
        "[main] transit positions:\n"
        f"  TRANSIT_1={transit_1_position.round(4)}\n"
        f"  TRANSIT_2={transit_2_position.round(4)}\n"
        f"  TRANSIT_3={transit_3_position.round(4)}",
        flush=True,
    )

    tracking_controller_a = (
        _create_tracking_controller(
            robot=robot_a,
            robot_id="A",
            base_position=robot_a_base_position,
            base_yaw_deg=robot_a_base_yaw_deg,
        )
    )
    tracking_controller_b = (
        _create_tracking_controller(
            robot=robot_b,
            robot_id="B",
            base_position=robot_b_base_position,
            base_yaw_deg=robot_b_base_yaw_deg,
        )
    )

    pick_place_controller_a = (
        _create_pick_place_controller(
            robot=robot_a,
            robot_id="A",
        )
    )
    pick_place_controller_b = (
        _create_pick_place_controller(
            robot=robot_b,
            robot_id="B",
        )
    )

    move_controller_a = (
        _create_move_controller(
            robot=robot_a,
            robot_id="A",
        )
    )
    move_controller_b = (
        _create_move_controller(
            robot=robot_b,
            robot_id="B",
        )
    )

    hand_input_a = CachedHandInput(
        input_id="LEFT_HAND",
        target_getter=(
            get_latest_left_hand_target
        ),
        mode_getter=(
            get_latest_left_hand_mode
        ),
        mode_resetter=(
            reset_left_hand_mode_cache
        ),
    )

    hand_input_b = CachedHandInput(
        input_id="RIGHT_HAND",
        target_getter=(
            get_latest_right_hand_target
        ),
        mode_getter=(
            get_latest_right_hand_mode
        ),
        mode_resetter=(
            reset_right_hand_mode_cache
        ),
    )

    state_machine_a = (
        _create_state_machine(
            robot_id="A",
            robot=robot_a,
            tray_registry=tray_registry,
            hand_input=hand_input_a,
            initial_joint_positions=(
                initial_joint_positions_a
            ),
            tracking_controller=(
                tracking_controller_a
            ),
            pick_place_controller=(
                pick_place_controller_a
            ),
            move_controller=(
                move_controller_a
            ),
        )
    )

    state_machine_b = (
        _create_state_machine(
            robot_id="B",
            robot=robot_b,
            tray_registry=tray_registry,
            hand_input=hand_input_b,
            initial_joint_positions=(
                initial_joint_positions_b
            ),
            tracking_controller=(
                tracking_controller_b
            ),
            pick_place_controller=(
                pick_place_controller_b
            ),
            move_controller=(
                move_controller_b
            ),
        )
    )

    robot_manager = RobotManager(
        routes={
            "TRANSIT_1": (
                transit_1_position,
                TRACKING_TOOL_ORIENTATION,
                np.deg2rad(
                    ROBOT_A_TRACKING_JOINT1_DELTA_DEG
                ),
            ),
            "TRANSIT_2": (
                transit_2_position,
                TRACKING_TOOL_ORIENTATION,
                0.0,
            ),
            "TRANSIT_3": (
                transit_3_position,
                TRACKING_TOOL_ORIENTATION,
                np.deg2rad(
                    ROBOT_B_TRACKING_JOINT1_DELTA_DEG
                ),
            ),
        },
        locked_routes=(
            "TRANSIT_2",
        ),
        shared_trays=(),
    )

    robot_manager.register_robot(
        profile=RobotProfile.create(
            robot_id="A",
            reachable_trays=(
                ROBOT_A_SUPPORTED_TRAY_COMMANDS
            ),
            preferred_route="TRANSIT_2",
            fallback_route="TRANSIT_1",
        ),
        state_machine=state_machine_a,
    )

    robot_manager.register_robot(
        profile=RobotProfile.create(
            robot_id="B",
            reachable_trays=(
                ROBOT_B_SUPPORTED_TRAY_COMMANDS
            ),
            preferred_route="TRANSIT_2",
            fallback_route="TRANSIT_3",
        ),
        state_machine=state_machine_b,
    )

    setup_m0609_ros_bridge(
        manager=robot_manager,
        simulation_app=simulation_app,
    )

    right_hand_marker = (
        HandMarkerVisualizer(
            world=world,
            coordinate_getter=(
                get_latest_right_hand_raw
            ),
            prim_path=(
                "/World/RightHandMarker"
            ),
            object_name=(
                "right_hand_marker"
            ),
            color=[
                0.1,
                0.3,
                1.0,
            ],
            initial_position=[
                0.0,
                0.25,
                1.0,
            ],
            radius=0.03,
            label="RIGHT HAND",
        )
    )

    left_hand_marker = (
        HandMarkerVisualizer(
            world=world,
            coordinate_getter=(
                get_latest_left_hand_raw
            ),
            prim_path=(
                "/World/LeftHandMarker"
            ),
            object_name=(
                "left_hand_marker"
            ),
            color=[
                1.0,
                0.85,
                0.1,
            ],
            initial_position=[
                0.0,
                -0.25,
                1.0,
            ],
            radius=0.03,
            label="LEFT HAND",
        )
    )

    for _ in range(
        INITIAL_SETTLING_FRAMES
    ):
        world.step(
            render=True,
        )

    print(
        "\n[M0609 Robot A/B 준비 완료]"
    )
    print(
        "- Robot A: trays 0,1,2,3 "
        "/ left hand"
    )
    print(
        "- Robot B: trays 4,5,6,7 "
        "/ right hand"
    )
    print(
        "- TRANSIT_2: A/B 공용 우선 경로 + LOCK"
    )
    print(
        "- Robot A fallback: "
        "TRANSIT_1 -> joint1 +90deg -> TRACKING"
    )
    print(
        "- Robot B fallback: "
        "TRANSIT_3 -> joint1 -90deg -> TRACKING"
    )
    print(
        "- PLACE 명령: 회전 완료 관절 자세 복귀 -> joint1 원복 -> 같은 경유지 -> 트레이"
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
                robot_a,
                world,
            )
            initialize_robot(
                robot_b,
                world,
            )

            tray_registry.reset_to_spawn()

            tracking_controller_a.reset()
            tracking_controller_b.reset()

            robot_manager.reset()

            right_hand_marker.reset()
            left_hand_marker.reset()

            print(
                "[main] Robot A/B "
                "Play 재초기화 완료",
                flush=True,
            )

        if is_playing:
            right_hand_marker.update()
            left_hand_marker.update()
            robot_manager.step()

        was_playing = is_playing


if __name__ == "__main__":
    import traceback

    try:
        main()
    except Exception:
        print(
            "\n[FATAL] Isaac Sim 실행 중 예외가 발생했습니다.",
            flush=True,
        )
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()