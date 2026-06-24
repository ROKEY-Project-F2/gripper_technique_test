# m0609_task.py

from __future__ import annotations

import omni.usd
from pxr import Usd, UsdPhysics

from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.manipulators import SingleManipulator

from dual_surface_gripper_adapter import DualSurfaceGripperAdapter

from m0609_config import (
    DRIVE_DAMPING,
    DRIVE_MAX_FORCE,
    DRIVE_STIFFNESS,
    EE_LINK_NAME,
    ROBOT_PRIM_PATH,
    ROBOT_SCENE_NAME,
    SURFACE_GRIPPER_PATHS,
    SURFACE_GRIPPER_WRITE_STATUS_TO_USD,
)


def find_prim_path_by_name(
    root_path: str,
    name: str,
):
    """root_path 아래에서 이름이 일치하는 Prim 경로를 찾는다."""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)

    if not root_prim.IsValid():
        return None

    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())

    return None


def initialize_robot(robot, world) -> None:
    """SingleManipulator와 Surface Gripper를 초기화한다."""
    print("[task] robot.initialize 시작")
    robot.initialize()
    print("[task] robot.initialize 완료")

    print("[task] gripper.initialize 시작")
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    print("[task] gripper.initialize 완료")

    robot.gripper.open()


class M0609BasicTask(BaseTask):
    """
    이미 열린 full_scene.usda 안의 M0609을 검색하고
    Isaac Sim Scene 객체로 등록한다.

    이 Task에서는 full_scene.usda를 다시 Reference하지 않는다.
    """

    def __init__(
        self,
        name: str = "m0609_basic_task",
    ) -> None:
        super().__init__(
            name=name,
            offset=None,
        )

        self._robot = None
        self._ee_path = None

    def set_up_scene(self, scene) -> None:
        super().set_up_scene(scene)

        self._validate_robot_prim()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)

        print(
            "\n[완료] 기존 full_scene의 M0609 등록 완료\n",
            flush=True,
        )

    def _validate_robot_prim(self) -> None:
        stage = omni.usd.get_context().get_stage()
        robot_root = stage.GetPrimAtPath(
            ROBOT_PRIM_PATH
        )

        if not robot_root.IsValid():
            top_level_paths = []

            world_prim = stage.GetPrimAtPath("/World")
            if world_prim.IsValid():
                top_level_paths = [
                    str(child.GetPath())
                    for child in world_prim.GetChildren()
                ]

            raise RuntimeError(
                "full_scene.usda에서 로봇 Prim을 찾지 못했습니다.\n"
                f"기대 경로: {ROBOT_PRIM_PATH}\n"
                f"/World 하위 Prim: {top_level_paths}"
            )

    def _discover_links(self) -> None:
        self._ee_path = find_prim_path_by_name(
            ROBOT_PRIM_PATH,
            EE_LINK_NAME,
        )

        if self._ee_path is None:
            raise RuntimeError(
                f"{ROBOT_PRIM_PATH} 아래에서 "
                f"'{EE_LINK_NAME}'을 찾지 못했습니다."
            )

        stage = omni.usd.get_context().get_stage()

        missing_grippers = [
            path
            for path in SURFACE_GRIPPER_PATHS
            if not stage.GetPrimAtPath(path).IsValid()
        ]

        if missing_grippers:
            raise RuntimeError(
                "SurfaceGripper Prim을 찾지 못했습니다:\n- "
                + "\n- ".join(missing_grippers)
            )

        print(
            f"[task] End Effector 발견: {self._ee_path}",
            flush=True,
        )

    def _setup_physics(self) -> None:
        stage = omni.usd.get_context().get_stage()
        robot_root = stage.GetPrimAtPath(
            ROBOT_PRIM_PATH
        )

        drive_count = 0

        for prim in Usd.PrimRange(robot_root):
            for drive_type in (
                "angular",
                "linear",
            ):
                drive = UsdPhysics.DriveAPI.Get(
                    prim,
                    drive_type,
                )

                if not drive:
                    continue

                drive.GetStiffnessAttr().Set(
                    DRIVE_STIFFNESS
                )
                drive.GetDampingAttr().Set(
                    DRIVE_DAMPING
                )
                drive.GetMaxForceAttr().Set(
                    DRIVE_MAX_FORCE
                )

                drive_count += 1

        print(
            f"[task] Drive 설정 완료: {drive_count}개",
            flush=True,
        )

    def _register_robot(self, scene) -> None:
        gripper = DualSurfaceGripperAdapter(
            end_effector_prim_path=self._ee_path,
            surface_gripper_prim_paths=SURFACE_GRIPPER_PATHS,
            write_status_to_usd=(
                SURFACE_GRIPPER_WRITE_STATUS_TO_USD
            ),
        )

        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name=ROBOT_SCENE_NAME,
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )

        print(
            "[task] 로봇 Scene 등록 완료:"
            f" name={ROBOT_SCENE_NAME},"
            f" prim={ROBOT_PRIM_PATH}",
            flush=True,
        )

    def post_reset(self) -> None:
        if self._robot is not None:
            self._robot.gripper.post_reset()