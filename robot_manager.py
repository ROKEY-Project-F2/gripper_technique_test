# robot_manager.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Sequence, Tuple

from robot_runtime import RobotProfile, RobotTask


@dataclass
class ManagedRobot:
    profile: RobotProfile
    state_machine: object
    active_task: Optional[RobotTask] = None
    last_state: str = "IDLE"


class RobotManager:
    """
    한 대와 여러 대를 같은 인터페이스로 관리하는 작업 매니저.

    현재 main.py에서는 로봇 A 하나만 등록한다.
    이후 동일한 상태 머신 인스턴스를 하나 더 생성해
    register_robot()만 호출하면 로봇 목록에 추가할 수 있다.
    """

    def __init__(
        self,
        *,
        routes: Mapping[str, Tuple[Sequence[float], Sequence[float]]],
        shared_trays: Sequence[int] = (),
    ) -> None:
        self._robots: Dict[str, ManagedRobot] = {}
        self._routes = {
            str(route_id).strip().upper(): (position, orientation)
            for route_id, (position, orientation) in routes.items()
        }
        self._shared_trays = frozenset(
            int(value) for value in shared_trays
        )

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

        self._robots[robot_id] = ManagedRobot(
            profile=profile,
            state_machine=state_machine,
            last_state=state_machine.state_name,
        )

        print(
            f"[Manager] registered robot={robot_id}, "
            f"reachable={sorted(profile.reachable_trays)}",
            flush=True,
        )

    def _select_robot(self, tray_id: int) -> Optional[ManagedRobot]:
        candidates = [
            managed
            for managed in self._robots.values()
            if tray_id in managed.profile.reachable_trays
            and managed.state_machine.state_name == "IDLE"
            and managed.active_task is None
        ]
        if not candidates:
            return None
        return sorted(
            candidates,
            key=lambda item: item.profile.robot_id,
        )[0]

    def _create_task(
        self,
        *,
        managed: ManagedRobot,
        tray_id: int,
    ) -> RobotTask:
        route_id = managed.profile.preferred_route
        if route_id not in self._routes:
            raise KeyError(
                f"Route {route_id} is not registered"
            )
        position, orientation = self._routes[route_id]
        return RobotTask.create(
            tray_id=tray_id,
            route_id=route_id,
            transit_position=position,
            transit_orientation=orientation,
            uses_shared_zone=(tray_id in self._shared_trays),
        )

    def request_pick_command(
        self,
        tray_index: int,
    ) -> Tuple[bool, str]:
        tray_index = int(tray_index)
        managed = self._select_robot(tray_index)
        if managed is None:
            return (
                False,
                f"No idle robot can reach tray {tray_index}",
            )

        task = self._create_task(
            managed=managed,
            tray_id=tray_index,
        )
        accepted, message = managed.state_machine.assign_task(task)
        if not accepted:
            return False, message

        managed.active_task = task
        return (
            True,
            f"Robot {managed.profile.robot_id} assigned "
            f"tray {tray_index} via route {task.route_id}",
        )

    def request_move(
        self,
        x: float,
        y: float,
        z: float,
    ) -> Tuple[bool, str]:
        return False, "Direct move is disabled in workflow mode"

    def step(self) -> None:
        for managed in self._robots.values():
            managed.state_machine.step()
            current_state = managed.state_machine.state_name

            if current_state != managed.last_state:
                print(
                    f"[Manager {managed.profile.robot_id}] "
                    f"state: {managed.last_state} -> {current_state}",
                    flush=True,
                )
                managed.last_state = current_state

            if (
                current_state == "IDLE"
                and managed.active_task is not None
                and managed.state_machine.current_task is None
            ):
                print(
                    f"[Manager {managed.profile.robot_id}] "
                    f"task complete: tray="
                    f"{managed.active_task.tray_id}",
                    flush=True,
                )
                managed.active_task = None

    def reset(self) -> None:
        for managed in self._robots.values():
            managed.active_task = None
            managed.state_machine.reset()
            managed.last_state = managed.state_machine.state_name