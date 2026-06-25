# robot_runtime.py

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class RobotTask:
    """매니저가 상태 머신에 전달하는 실행 단위."""

    tray_id: int
    route_id: str

    # 트레이를 집은 뒤 도착할 Cartesian 경유지.
    transit_position: np.ndarray
    transit_orientation: np.ndarray

    # 경유지에 도착한 뒤 첫 번째 관절만 회전할 상대 각도(rad).
    # 0이면 별도 관절 회전 없이 바로 TRACKING에 들어간다.
    joint1_delta_rad: float = 0.0

    uses_shared_zone: bool = False

    @classmethod
    def create(
        cls,
        *,
        tray_id: int,
        route_id: str,
        transit_position: Sequence[float],
        transit_orientation: Sequence[float],
        joint1_delta_rad: float = 0.0,
        uses_shared_zone: bool = False,
    ) -> "RobotTask":
        position = np.asarray(
            transit_position,
            dtype=np.float64,
        ).copy()

        orientation = np.asarray(
            transit_orientation,
            dtype=np.float64,
        ).copy()

        if position.shape != (3,):
            raise ValueError(
                "transit_position must have shape (3,)"
            )

        if orientation.shape != (4,):
            raise ValueError(
                "transit_orientation must have shape (4,)"
            )

        if not np.all(np.isfinite(position)):
            raise ValueError(
                "transit_position must be finite"
            )

        if not np.all(np.isfinite(orientation)):
            raise ValueError(
                "transit_orientation must be finite"
            )

        delta = float(joint1_delta_rad)

        if not np.isfinite(delta):
            raise ValueError(
                "joint1_delta_rad must be finite"
            )

        return cls(
            tray_id=int(tray_id),
            route_id=str(route_id).strip().upper(),
            transit_position=position,
            transit_orientation=orientation,
            joint1_delta_rad=delta,
            uses_shared_zone=bool(uses_shared_zone),
        )


@dataclass(frozen=True)
class RobotProfile:
    """매니저가 로봇을 선택할 때 사용하는 정적 설정."""

    robot_id: str
    reachable_trays: FrozenSet[int]
    preferred_route: str
    fallback_route: str

    @classmethod
    def create(
        cls,
        *,
        robot_id: str,
        reachable_trays,
        preferred_route: str,
        fallback_route: str,
    ) -> "RobotProfile":
        return cls(
            robot_id=str(robot_id).strip().upper(),
            reachable_trays=frozenset(
                int(value)
                for value in reachable_trays
            ),
            preferred_route=str(
                preferred_route
            ).strip().upper(),
            fallback_route=str(
                fallback_route
            ).strip().upper(),
        )