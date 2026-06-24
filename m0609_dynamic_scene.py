# m0609_dynamic_scene.py

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Sequence

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage


def quaternion_to_yaw(
    quaternion: Sequence[float],
) -> float:
    """Isaac Sim quaternion [w, x, y, z]에서 yaw를 추출한다."""
    w, x, y, z = np.asarray(
        quaternion,
        dtype=np.float64,
    )

    return float(
        np.arctan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
    )


def downward_tool_orientation_for_tray(
    tray_orientation: Sequence[float],
) -> np.ndarray:
    """트레이의 현재 yaw에 맞춘 하향 그리퍼 quaternion을 만든다."""
    tray_yaw = quaternion_to_yaw(
        tray_orientation
    )
    ee_yaw = tray_yaw + np.pi / 2.0
    half = ee_yaw * 0.5

    return np.array(
        [
            0.0,
            np.cos(half),
            np.sin(half),
            0.0,
        ],
        dtype=np.float64,
    )


@dataclass
class TrayPoseSnapshot:
    tray_id: int
    current_reference_position: np.ndarray
    current_orientation: np.ndarray
    pick_position: np.ndarray
    pick_orientation: np.ndarray
    spawn_reference_position: np.ndarray
    spawn_orientation: np.ndarray


@dataclass
class TrayRecord:
    tray_id: int
    rigid_object: object
    spawn_reference_position: np.ndarray
    spawn_orientation: np.ndarray
    top_offset_z: float


class DynamicTrayRegistry:
    """실제 트레이의 현재 pose와 생성 당시 pose를 ID별로 관리한다."""

    def __init__(self) -> None:
        self._records: Dict[int, TrayRecord] = {}

    def register_tray(
        self,
        *,
        tray_id: int,
        rigid_object,
        spawn_reference_position: Sequence[float],
        spawn_orientation: Sequence[float],
        top_offset_z: float,
    ) -> None:
        tray_id = int(tray_id)

        self._records[tray_id] = TrayRecord(
            tray_id=tray_id,
            rigid_object=rigid_object,
            spawn_reference_position=np.asarray(
                spawn_reference_position,
                dtype=np.float64,
            ).copy(),
            spawn_orientation=np.asarray(
                spawn_orientation,
                dtype=np.float64,
            ).copy(),
            top_offset_z=float(top_offset_z),
        )

    def has_tray(
        self,
        tray_id: int,
    ) -> bool:
        return int(tray_id) in self._records

    def get_snapshot(
        self,
        tray_id: int,
    ) -> TrayPoseSnapshot:
        tray_id = int(tray_id)

        if tray_id not in self._records:
            raise KeyError(
                f"등록되지 않은 트레이입니다: {tray_id}"
            )

        record = self._records[tray_id]

        current_position, current_orientation = (
            record.rigid_object.get_world_pose()
        )

        current_position = np.asarray(
            current_position,
            dtype=np.float64,
        )
        current_orientation = np.asarray(
            current_orientation,
            dtype=np.float64,
        )

        pick_position = (
            current_position
            + np.array(
                [0.0, 0.0, record.top_offset_z],
                dtype=np.float64,
            )
        )

        return TrayPoseSnapshot(
            tray_id=tray_id,
            current_reference_position=current_position.copy(),
            current_orientation=current_orientation.copy(),
            pick_position=pick_position,
            pick_orientation=(
                downward_tool_orientation_for_tray(
                    current_orientation
                )
            ),
            spawn_reference_position=(
                record.spawn_reference_position.copy()
            ),
            spawn_orientation=(
                record.spawn_orientation.copy()
            ),
        )

    def reset_to_spawn(self) -> None:
        """트레이만 최초 생성 위치로 복원한다."""
        for record in self._records.values():
            record.rigid_object.set_world_pose(
                position=record.spawn_reference_position,
                orientation=record.spawn_orientation,
            )
            _clear_velocity(
                record.rigid_object
            )


def _clear_velocity(
    rigid_object,
) -> None:
    for method_name in (
        "set_linear_velocity",
        "set_angular_velocity",
    ):
        method = getattr(
            rigid_object,
            method_name,
            None,
        )
        if callable(method):
            method(
                np.zeros(
                    3,
                    dtype=np.float64,
                )
            )


def _find_source_rigid_prim_path(
    tool_usd_path: str,
) -> Sdf.Path:
    """
    도구 USD 전체 defaultPrim이 아니라 실제 RigidBody Prim만 찾는다.

    이 방식으로 도구 USD 안에 잘못 저장된 트레이 payload 같은
    불필요한 sibling Prim은 현재 Stage에 불러오지 않는다.
    """
    source_stage = Usd.Stage.Open(
        tool_usd_path,
        load=Usd.Stage.LoadNone,
    )

    if source_stage is None:
        raise RuntimeError(
            f"도구 USD를 열 수 없습니다: {tool_usd_path}"
        )

    default_prim = source_stage.GetDefaultPrim()

    if default_prim.IsValid():
        search_root = default_prim
    else:
        search_root = source_stage.GetPseudoRoot()

    rigid_candidates = []

    for prim in Usd.PrimRange(
        search_root
    ):
        if prim.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            rigid_candidates.append(
                prim.GetPath()
            )

    if not rigid_candidates:
        expected_name = Path(
            tool_usd_path
        ).stem

        for prim in Usd.PrimRange(
            search_root
        ):
            if prim.GetName() == expected_name:
                return prim.GetPath()

        raise RuntimeError(
            "도구 USD에서 RigidBody Prim을 "
            f"찾을 수 없습니다: {tool_usd_path}"
        )

    # 가장 상위에 있는 RigidBody를 선택한다.
    rigid_candidates.sort(
        key=lambda path: len(
            path.pathString.split("/")
        )
    )

    return rigid_candidates[0]


def _configure_existing_tool_physics(
    tool_prim,
    *,
    mass: float,
) -> None:
    """
    도구 USD에 이미 존재하는 RigidBody를 그대로 사용한다.

    부모 루트에 RigidBodyAPI를 새로 적용하지 않으므로
    중첩 RigidBody hierarchy 오류가 발생하지 않는다.
    """
    if not tool_prim.HasAPI(
        UsdPhysics.RigidBodyAPI
    ):
        UsdPhysics.RigidBodyAPI.Apply(
            tool_prim
        )

    if not tool_prim.HasAPI(
        UsdPhysics.MassAPI
    ):
        UsdPhysics.MassAPI.Apply(
            tool_prim
        )

    UsdPhysics.MassAPI(
        tool_prim
    ).GetMassAttr().Set(
        float(mass)
    )

    for child in Usd.PrimRange(
        tool_prim
    ):
        if child.GetTypeName() != "Mesh":
            continue

        if not child.HasAPI(
            UsdPhysics.CollisionAPI
        ):
            UsdPhysics.CollisionAPI.Apply(
                child
            )

        collision = (
            UsdPhysics.MeshCollisionAPI.Apply(
                child
            )
        )
        collision.GetApproximationAttr().Set(
            "convexHull"
        )


def _create_tool_reference(
    *,
    stage,
    tool_id: int,
    tool_usd_path: str,
    tool_position: Sequence[float],
    tool_scale: Sequence[float],
    tool_mass: float,
) -> None:
    """
    source USD의 실제 도구 RigidBody Prim만 /World/tool_N에 참조한다.

    tool USD 전체 /RootNode를 참조하지 않기 때문에,
    잘못 포함된 model_redtray payload도 로드되지 않는다.
    """
    source_prim_path = (
        _find_source_rigid_prim_path(
            tool_usd_path
        )
    )

    target_path = (
        f"/World/tool_{int(tool_id)}"
    )

    with Usd.EditContext(
        stage,
        stage.GetRootLayer(),
    ):
        target_prim = stage.DefinePrim(
            target_path,
            "Xform",
        )

        target_prim.GetReferences().AddReference(
            Sdf.Reference(
                assetPath=tool_usd_path,
                primPath=source_prim_path,
            )
        )

    tool_prim = stage.GetPrimAtPath(
        target_path
    )

    if not tool_prim.IsValid():
        raise RuntimeError(
            f"도구 Prim 생성 실패: {target_path}"
        )

    with Usd.EditContext(
        stage,
        stage.GetRootLayer(),
    ):
        xformable = UsdGeom.Xformable(
            tool_prim
        )
        xformable.ClearXformOpOrder()
        xformable.SetResetXformStack(
            True
        )
        xformable.AddTranslateOp().Set(
            Gf.Vec3d(
                float(tool_position[0]),
                float(tool_position[1]),
                float(tool_position[2]),
            )
        )

        scale = np.asarray(
            tool_scale,
            dtype=np.float64,
        )

        if scale.shape != (3,):
            raise ValueError(
                f"tool_scale은 길이 3이어야 합니다: {scale}"
            )

        xformable.AddScaleOp().Set(
            Gf.Vec3f(
                float(scale[0]),
                float(scale[1]),
                float(scale[2]),
            )
        )

        _configure_existing_tool_physics(
            tool_prim,
            mass=tool_mass,
        )


def create_surgical_trays_and_tools(
    *,
    world: World,
    tray_usd_path: str,
    tool_usds: Sequence[str],
    tray_positions: Mapping[int, Sequence[float]],
    tray_orientation: Sequence[float],
    tray_top_z: float,
    tool_drop_height: float,
    tool_mass: float,
    tool_scales: Sequence[Sequence[float]],
    simulation_app,
) -> DynamicTrayRegistry:
    """
    실제 트레이 8개와 도구 8개를 물리 초기화 전에 생성한다.

    트레이 초기 회전은 모두 동일하며 랜덤 회전은 없다.
    """
    if len(tool_usds) < len(
        tray_positions
    ):
        raise ValueError(
            "트레이 수보다 TOOL_USDS 수가 적습니다."
        )

    if len(tool_scales) < len(
        tray_positions
    ):
        raise ValueError(
            "트레이 수보다 TOOL_SCALES 수가 적습니다."
        )

    stage = (
        omni.usd
        .get_context()
        .get_stage()
    )
    registry = DynamicTrayRegistry()

    tray_orientation = np.asarray(
        tray_orientation,
        dtype=np.float64,
    )

    for tray_id, position in sorted(
        tray_positions.items()
    ):
        tray_id = int(tray_id)
        position = np.asarray(
            position,
            dtype=np.float64,
        )

        tray_ref = (
            f"/World/tray_{tray_id}"
        )

        add_reference_to_stage(
            usd_path=tray_usd_path,
            prim_path=tray_ref,
        )

        for _ in range(5):
            simulation_app.update()

        tray_rigid_path = (
            f"{tray_ref}/E_redtray_28"
        )

        if not stage.GetPrimAtPath(
            tray_rigid_path
        ).IsValid():
            raise RuntimeError(
                "트레이 rigid Prim을 찾지 못했습니다: "
                f"{tray_rigid_path}"
            )

        tray = world.scene.add(
            SingleRigidPrim(
                prim_path=tray_rigid_path,
                name=f"tray_{tray_id}",
                position=position,
                orientation=tray_orientation,
            )
        )

        registry.register_tray(
            tray_id=tray_id,
            rigid_object=tray,
            spawn_reference_position=position,
            spawn_orientation=tray_orientation,
            top_offset_z=tray_top_z,
        )

        tool_position = (
            position
            + np.array(
                [
                    0.0,
                    0.0,
                    float(tool_drop_height),
                ],
                dtype=np.float64,
            )
        )

        _create_tool_reference(
            stage=stage,
            tool_id=tray_id,
            tool_usd_path=tool_usds[tray_id],
            tool_position=tool_position,
            tool_scale=tool_scales[tray_id],
            tool_mass=tool_mass,
        )

        print(
            f"[DynamicScene] tray={tray_id}, "
            f"position={position.tolist()}, "
            f"tool={Path(tool_usds[tray_id]).name}, "
            f"scale={tuple(tool_scales[tray_id])}",
            flush=True,
        )

    print(
        "[DynamicScene] 실제 트레이 8개와 "
        "도구 8개 생성 완료",
        flush=True,
    )

    return registry