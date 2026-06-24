# hand_marker_visualizer.py

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualSphere

from m0609_ros_bridge import get_latest_hand_raw


class HandMarkerVisualizer:
    """ROS /hand_raw 좌표를 Isaac Sim의 VisualSphere로 표시한다."""

    def __init__(
        self,
        world: World,
        *,
        prim_path: str = "/World/HandMarker",
        object_name: str = "hand_marker",
        initial_position: Optional[Sequence[float]] = None,
        radius: float = 0.03,
    ) -> None:
        self._last_sequence = -1

        if initial_position is None:
            initial_position = (
                0.0,
                0.25,
                1.0,
            )

        existing = world.scene.get_object(
            object_name
        )

        if existing is not None:
            self._marker = existing
        else:
            self._marker = world.scene.add(
                VisualSphere(
                    prim_path=prim_path,
                    name=object_name,
                    position=np.asarray(
                        initial_position,
                        dtype=np.float64,
                    ),
                    radius=float(radius),
                    color=np.array(
                        [0.1, 0.3, 1.0],
                        dtype=np.float64,
                    ),
                )
            )

    def update(self) -> None:
        hand_raw, sequence = get_latest_hand_raw()

        if hand_raw is None:
            return

        if sequence == self._last_sequence:
            return

        position = np.asarray(
            hand_raw,
            dtype=np.float64,
        )

        if position.shape != (3,):
            print(
                f"[HandMarker] 잘못된 좌표 형태: {position}",
                flush=True,
            )
            return

        if not np.all(
            np.isfinite(position)
        ):
            print(
                f"[HandMarker] 유효하지 않은 좌표: {position}",
                flush=True,
            )
            return

        self._marker.set_world_pose(
            position=position,
        )

        self._last_sequence = sequence

    def reset(self) -> None:
        self._last_sequence = -1