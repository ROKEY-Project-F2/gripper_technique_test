# robot_manager.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from robot_runtime import RobotProfile, RobotTask
from tool_state_manager import ToolStateManager


@dataclass
class ManagedRobot:
    profile: RobotProfile
    state_machine: object
    active_task: Optional[RobotTask] = None
    last_state: str = "IDLE"
    last_place_release_sequence: int = 0


class RobotManager:
    """
    여러 로봇의 작업 배정과 트레이 PICK/PLACE 구역 락을 관리한다.

    삭제된 기능:
    - TRANSIT_2 공용 경로
    - 경로 우선순위
    - fallback 경로
    - 경유지 락
    - route owner
    - 경유지 대기 큐

    유지되는 기능:
    - 두 로봇 모두 Tray 0~5 접근
    - 담당 트레이 우선 로봇 선택
      · Tray 0/2/4: Robot A 우선
      · Tray 1/3/5: Robot B 우선
    - 우선 로봇이 바쁘면 다른 IDLE 로봇이 대신 수행
    - 신규 트레이 PICK/PLACE 구역 락
    - 신규 락은 큐 없이 단순 차단
    """

    def __init__(
        self,
        *,
        tray_positions: Mapping[
            int,
            Sequence[float],
        ],
        shared_trays: Sequence[int] = (),
        tool_state_manager: Optional[
            ToolStateManager
        ] = None,
    ) -> None:
        self._robots: Dict[str, ManagedRobot] = {}
        self._tool_state_manager = (
            tool_state_manager
        )

        self._tray_positions = {
            int(tray_id): np.asarray(
                position,
                dtype=np.float64,
            ).copy()
            for tray_id, position
            in tray_positions.items()
        }

        for tray_id, position in self._tray_positions.items():
            if position.shape != (3,):
                raise ValueError(
                    f"Tray {tray_id} position "
                    "must have shape (3,)"
                )

            if not np.all(np.isfinite(position)):
                raise ValueError(
                    f"Tray {tray_id} position "
                    "must be finite"
                )

        self._shared_trays = frozenset(
            int(value)
            for value in shared_trays
        )

        unknown_shared_trays = (
            self._shared_trays
            - self._tray_positions.keys()
        )

        if unknown_shared_trays:
            raise ValueError(
                "공용 구역 대상 트레이가 "
                "tray_positions에 없습니다: "
                f"{sorted(unknown_shared_trays)}"
            )

        # 신규 PICK/PLACE 구역 락.
        # 큐를 두지 않고 현재 소유자만 저장한다.
        self._tray_zone_owner: Optional[str] = None

    def register_robot(
        self,
        *,
        profile: RobotProfile,
        state_machine,
    ) -> None:
        robot_id = profile.robot_id

        if robot_id in self._robots:
            raise ValueError(
                f"Robot {robot_id} is already registered"
            )

        unknown_trays = (
            profile.reachable_trays
            - self._tray_positions.keys()
        )

        if unknown_trays:
            raise KeyError(
                f"Robot {robot_id} reachable trays "
                "are not registered: "
                f"{sorted(unknown_trays)}"
            )

        self._robots[robot_id] = ManagedRobot(
            profile=profile,
            state_machine=state_machine,
            last_state=state_machine.state_name,
            last_place_release_sequence=int(
                getattr(
                    state_machine,
                    "place_release_sequence",
                    0,
                )
            ),
        )

        print(
            f"[Manager] registered robot={robot_id}, "
            f"reachable={sorted(profile.reachable_trays)}, "
            f"route={profile.route_id}, "
            f"transit={profile.transit_position.round(4)}, "
            f"selection_position="
            f"{profile.selection_position.round(4)}",
            flush=True,
        )

    def _can_accept_command(
        self,
        managed: ManagedRobot,
    ) -> Tuple[bool, str]:
        current_state = (
            managed.state_machine.state_name
        )

        if current_state != "IDLE":
            return (
                False,
                f"Robot {managed.profile.robot_id} "
                f"is busy: state={current_state}",
            )

        if managed.active_task is not None:
            return (
                False,
                f"Robot {managed.profile.robot_id} "
                "is busy: active_task exists",
            )

        return True, "accepted"

    def _distance_to_tray(
        self,
        *,
        managed: ManagedRobot,
        tray_id: int,
    ) -> float:
        tray_position = self._tray_positions[tray_id]
        robot_position = (
            managed.profile.selection_position
        )

        return float(
            np.linalg.norm(
                tray_position - robot_position
            )
        )

    @staticmethod
    def _preferred_robot_priority(
        *,
        robot_id: str,
        tray_id: int,
    ) -> int:
        """
        두 로봇이 모두 명령을 받을 수 있을 때 적용되는
        트레이별 담당 우선순위.

        Tray 0, 2, 4 -> Robot A 우선
        Tray 1, 3, 5 -> Robot B 우선

        우선 로봇이 바쁘면 후보 목록에서 제외되므로,
        다른 로봇이 자동으로 대신 수행한다.
        """
        normalized = str(robot_id).strip().upper()
        preferred_robot = (
            "A"
            if int(tray_id) % 2 == 0
            else "B"
        )

        if normalized == preferred_robot:
            return 0

        if normalized in {"A", "B"}:
            return 1

        return 2

    def _find_candidate_robots(
        self,
        tray_id: int,
    ) -> list[ManagedRobot]:
        candidates: list[ManagedRobot] = []

        for managed in self._robots.values():
            if (
                tray_id
                not in managed.profile.reachable_trays
            ):
                continue

            can_accept, _ = self._can_accept_command(
                managed
            )

            if not can_accept:
                continue

            candidates.append(managed)

        candidates.sort(
            key=lambda managed: (
                self._preferred_robot_priority(
                    robot_id=(
                        managed.profile.robot_id
                    ),
                    tray_id=tray_id,
                ),
                self._distance_to_tray(
                    managed=managed,
                    tray_id=tray_id,
                ),
                managed.profile.robot_id,
            )
        )

        return candidates

    # ========================================================
    # 신규 트레이 PICK/PLACE 구역 락
    # ========================================================

    def _try_acquire_tray_zone(
        self,
        *,
        robot_id: str,
    ) -> bool:
        robot_id = str(robot_id).strip().upper()
        owner = self._tray_zone_owner

        if owner is None:
            self._tray_zone_owner = robot_id

            print(
                "[Manager TRAY-ZONE] acquired: "
                f"robot={robot_id}",
                flush=True,
            )
            return True

        if owner == robot_id:
            return True

        return False

    def _release_tray_zone(
        self,
        *,
        robot_id: str,
    ) -> None:
        robot_id = str(robot_id).strip().upper()
        owner = self._tray_zone_owner

        if owner is None:
            return

        if owner != robot_id:
            print(
                "[Manager TRAY-ZONE] "
                "release ignored: "
                f"requester={robot_id}, "
                f"owner={owner}",
                flush=True,
            )
            return

        self._tray_zone_owner = None

        print(
            "[Manager TRAY-ZONE] released: "
            f"robot={robot_id}",
            flush=True,
        )

    def _task_uses_tray_zone(
        self,
        managed: ManagedRobot,
    ) -> bool:
        task = managed.active_task

        return bool(
            task is not None
            and task.uses_shared_zone
        )

    def _must_hold_tray_zone_before_step(
        self,
        managed: ManagedRobot,
    ) -> bool:
        if not self._task_uses_tray_zone(managed):
            return False

        state_name = (
            managed.state_machine.state_name
        )

        if (
            state_name == "IDLE"
            and managed.active_task is not None
        ):
            return True

        return state_name in {
            "PICK",
            "PLACE",
        }

    def _release_tray_zone_after_transition(
        self,
        *,
        managed: ManagedRobot,
        previous_state: str,
        current_state: str,
    ) -> None:
        robot_id = managed.profile.robot_id

        if (
            previous_state == "PICK"
            and current_state == "TRACKING"
        ):
            self._release_tray_zone(
                robot_id=robot_id
            )
            return

        if (
            previous_state == "PLACE"
            and current_state == "RETURN_HOME"
        ):
            self._release_tray_zone(
                robot_id=robot_id
            )

    # ========================================================
    # 작업 배정
    # ========================================================

    def _create_task(
        self,
        *,
        managed: ManagedRobot,
        tray_id: int,
    ) -> RobotTask:
        profile = managed.profile

        return RobotTask.create(
            tray_id=tray_id,
            route_id=profile.route_id,
            transit_position=(
                profile.transit_position
            ),
            transit_orientation=(
                profile.transit_orientation
            ),
            joint1_delta_rad=(
                profile.tracking_joint1_delta_rad
            ),
            uses_shared_zone=(
                tray_id in self._shared_trays
            ),
        )

    def request_pick_command(
        self,
        tray_index: int,
    ) -> Tuple[bool, str]:
        tray_index = int(tray_index)

        if tray_index not in self._tray_positions:
            message = (
                f"Unknown tray: {tray_index}"
            )

            print(
                f"[Manager BLOCK] {message}",
                flush=True,
            )
            return False, message

        if (
            self._tool_state_manager is not None
            and self._tool_state_manager.get_tool_for_tray(
                tray_index
            ) is None
        ):
            message = (
                f"Tray {tray_index} is empty"
            )

            print(
                f"[Manager BLOCK] {message}",
                flush=True,
            )
            return False, message

        candidates = self._find_candidate_robots(
            tray_index
        )

        if not candidates:
            message = (
                "No IDLE robot can reach tray "
                f"{tray_index}"
            )

            print(
                f"[Manager BLOCK] {message}",
                flush=True,
            )
            return False, message

        # 큐를 사용하지 않는다.
        # 구역 점유 중이면 신규 PICK 요청을 즉시 차단한다.
        if (
            tray_index in self._shared_trays
            and self._tray_zone_owner is not None
        ):
            message = (
                "Tray PICK/PLACE zone is occupied: "
                f"owner={self._tray_zone_owner}"
            )

            print(
                "[Manager BLOCK] "
                f"tray={tray_index}, "
                f"{message}",
                flush=True,
            )
            return False, message

        for managed in candidates:
            robot_id = managed.profile.robot_id

            task = self._create_task(
                managed=managed,
                tray_id=tray_index,
            )

            tray_zone_acquired = False

            if task.uses_shared_zone:
                tray_zone_acquired = (
                    self._try_acquire_tray_zone(
                        robot_id=robot_id
                    )
                )

                if not tray_zone_acquired:
                    return (
                        False,
                        "Tray PICK/PLACE zone "
                        "is occupied: "
                        f"owner="
                        f"{self._tray_zone_owner}",
                    )

            accepted, message = (
                managed.state_machine.assign_task(
                    task
                )
            )

            if not accepted:
                if tray_zone_acquired:
                    self._release_tray_zone(
                        robot_id=robot_id
                    )
                continue

            managed.active_task = task

            distance = self._distance_to_tray(
                managed=managed,
                tray_id=tray_index,
            )

            return (
                True,
                f"Robot {robot_id} assigned "
                f"tray {tray_index}, "
                f"route={task.route_id}, "
                f"distance={distance:.4f} m, "
                f"joint1_delta="
                f"{task.joint1_delta_rad:.4f} rad",
            )

        message = (
            f"No robot accepted tray "
            f"{tray_index}"
        )

        print(
            f"[Manager BLOCK] {message}",
            flush=True,
        )
        return False, message

    def request_tool_command(
        self,
        tool_id: str,
    ) -> Tuple[bool, str]:
        if self._tool_state_manager is None:
            return (
                False,
                "ToolStateManager is not configured",
            )

        tray_id = (
            self._tool_state_manager
            .get_tray_for_tool(tool_id)
        )

        if tray_id is None:
            return (
                False,
                f"Tool is not available on a tray: {tool_id}",
            )

        return self.request_pick_command(
            tray_id
        )

    def update_external_tool_detection(
        self,
        *,
        tool_id: str,
        position: Sequence[float],
        tray_id: Optional[int] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        if self._tool_state_manager is None:
            raise RuntimeError(
                "ToolStateManager is not configured"
            )

        self._tool_state_manager.update_external_detection(
            tool_id=tool_id,
            position=position,
            tray_id=tray_id,
            timestamp=timestamp,
        )

    def get_tool_state_snapshot(self) -> dict:
        if self._tool_state_manager is None:
            return {}

        return (
            self._tool_state_manager
            .get_debug_snapshot()
        )

    def request_move(
        self,
        x: float,
        y: float,
        z: float,
    ) -> Tuple[bool, str]:
        return (
            False,
            "Direct move is disabled "
            "in workflow mode",
        )

    def step(self) -> None:
        if self._tool_state_manager is not None:
            self._tool_state_manager.synchronize_external_state()

        for managed in self._robots.values():
            robot_id = managed.profile.robot_id
            previous_state = (
                managed.state_machine.state_name
            )

            if self._must_hold_tray_zone_before_step(
                managed
            ):
                acquired = (
                    self._try_acquire_tray_zone(
                        robot_id=robot_id
                    )
                )

                if not acquired:
                    continue

            managed.state_machine.step()

            current_release_sequence = int(
                getattr(
                    managed.state_machine,
                    "place_release_sequence",
                    0,
                )
            )

            if (
                current_release_sequence
                != managed.last_place_release_sequence
            ):
                managed.last_place_release_sequence = (
                    current_release_sequence
                )

                if self._tool_state_manager is not None:
                    if managed.active_task is None:
                        raise RuntimeError(
                            "PLACE release confirmed without active task: "
                            f"robot={robot_id}"
                        )

                    # 실제 해제가 확인된 이 로봇의 도구만
                    # HELD_BY_ROBOT -> 해당 트레이로 복귀시킨다.
                    self._tool_state_manager.on_place_enter(
                        robot_id=robot_id,
                        tray_id=managed.active_task.tray_id,
                    )

                    self._tool_state_manager.print_tool_locations(
                        title=(
                            "physical PLACE release confirmed "
                            f"by Robot {robot_id}"
                        )
                    )

            current_state = (
                managed.state_machine.state_name
            )

            if current_state != managed.last_state:
                print(
                    f"[Manager {robot_id}] "
                    f"state: {managed.last_state} "
                    f"-> {current_state}",
                    flush=True,
                )

                managed.last_state = current_state

            if (
                self._tool_state_manager is not None
                and managed.active_task is not None
            ):
                if (
                    previous_state == "PICK"
                    and current_state == "TRACKING"
                ):
                    self._tool_state_manager.on_tracking_enter(
                        robot_id=robot_id,
                        tray_id=managed.active_task.tray_id,
                    )

                elif (
                    previous_state == "TRACKING"
                    and current_state == "PLACE"
                ):
                    # 실제 그리퍼 해제가 확인될 때까지
                    # 해당 로봇의 도구는 HELD_BY_ROBOT으로 유지한다.
                    pass

            self._release_tray_zone_after_transition(
                managed=managed,
                previous_state=previous_state,
                current_state=current_state,
            )

            if (
                managed.active_task is not None
                and (
                    managed.state_machine.current_task
                    is None
                )
            ):
                completed_task = managed.active_task

                self._release_tray_zone(
                    robot_id=robot_id
                )

                print(
                    f"[Manager {robot_id}] "
                    "task complete: "
                    f"tray={completed_task.tray_id}, "
                    f"route={completed_task.route_id}",
                    flush=True,
                )

                managed.active_task = None

    def reset(
        self,
        *,
        randomize_tools: bool = False,
    ) -> Optional[Dict[int, str]]:
        self._tray_zone_owner = None

        for managed in self._robots.values():
            managed.active_task = None
            managed.state_machine.reset()
            managed.last_state = (
                managed.state_machine.state_name
            )
            managed.last_place_release_sequence = int(
                getattr(
                    managed.state_machine,
                    "place_release_sequence",
                    0,
                )
            )

        tool_layout = None

        if (
            randomize_tools
            and self._tool_state_manager is not None
        ):
            tool_layout = (
                self._tool_state_manager
                .reset_random_layout()
            )

        print(
            "[Manager] reset: "
            "tray-zone lock cleared"
            + (
                ", tools randomized"
                if tool_layout is not None
                else ""
            ),
            flush=True,
        )

        return tool_layout

    def get_tray_zone_owner(
        self,
    ) -> Optional[str]:
        return self._tray_zone_owner