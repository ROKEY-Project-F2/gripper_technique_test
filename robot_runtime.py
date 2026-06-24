# robot_runtime.py

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Sequence

import numpy as np


@dataclass(frozen=True)
class RobotTask:
    """매니저가 상태 머신에 전달하는 실행 단위."""

    tray_id: int
    route_id: str
    transit_position: np.ndarray
    transit_orientation: np.ndarray
    uses_shared_zone: bool = False

    @classmethod
    def create(
        cls,
        *,
        tray_id: int,
        route_id: str,
        transit_position: Sequence[float],
        transit_orientation: Sequence[float],
        uses_shared_zone: bool = False,
    ) -> "RobotTask":
        return cls(
            tray_id=int(tray_id),
            route_id=str(route_id).strip().upper(),
            transit_position=np.asarray(
                transit_position,
                dtype=np.float64,
            ).copy(),
            transit_orientation=np.asarray(
                transit_orientation,
                dtype=np.float64,
            ).copy(),
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
                int(value) for value in reachable_trays
            ),
            preferred_route=str(
                preferred_route
            ).strip().upper(),
            fallback_route=str(
                fallback_route
            ).strip().upper(),
        )