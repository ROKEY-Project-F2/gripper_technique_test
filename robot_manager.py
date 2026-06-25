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
    여러 로봇의 작업 배정과 공용 경유지 락을 관리한다.

    경로 선택 규칙:
      1. 각 로봇은 preferred_route를 먼저 시도한다.
      2. preferred_route가 잠겨 있으면 fallback_route를 사용한다.
      3. 작업에 선택된 경로는 RobotTask에 저장되어 왕복 내내 유지된다.

    현재 설정:
      Robot A: TRANSIT_2 우선, TRANSIT_1 대체
      Robot B: TRANSIT_2 우선, TRANSIT_3 대체

    TRANSIT_2 락은 작업 배정 시 획득하고,
    트레이 반환이 끝나 상태 머신의 current_task가 제거될 때 해제한다.
    RETURN_HOME과 IDLE 복귀는 락 점유 범위에 포함하지 않는다.
    """

    def __init__(
        self,
        *,
        routes: Mapping[
            str,
            Tuple[
                Sequence[float],
                Sequence[float],
                float,
            ],
        ],
        locked_routes: Sequence[str] = (),
        shared_trays: Sequence[int] = (),
    ) -> None:
        self._robots: Dict[str, ManagedRobot] = {}

        self._routes = {
            str(route_id).strip().upper(): (
                tuple(float(value) for value in position),
                tuple(float(value) for value in orientation),
                float(joint1_delta_rad),
            )
            for route_id, (
                position,
                orientation,
                joint1_delta_rad,
            ) in routes.items()
        }

        self._locked_routes = frozenset(
            str(route_id).strip().upper()
            for route_id in locked_routes
        )

        unknown_locked_routes = (
            self._locked_routes
            - self._routes.keys()
        )

        if unknown_locked_routes:
            raise ValueError(
                "잠금 대상 경로가 routes에 없습니다: "
                f"{sorted(unknown_locked_routes)}"
            )

        # route_id -> robot_id
        self._route_owners: Dict[str, str] = {}

        self._shared_trays = frozenset(
            int(value)
            for value in shared_trays
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

        for route_id in (
            profile.preferred_route,
            profile.fallback_route,
        ):
            if route_id not in self._routes:
                raise KeyError(
                    f"Robot {robot_id} route "
                    f"{route_id} is not registered"
                )

        self._robots[robot_id] = ManagedRobot(
            profile=profile,
            state_machine=state_machine,
            last_state=state_machine.state_name,
        )

        print(
            f"[Manager] registered robot={robot_id}, "
            f"reachable={sorted(profile.reachable_trays)}, "
            f"preferred={profile.preferred_route}, "
            f"fallback={profile.fallback_route}",
            flush=True,
        )

    def _select_robot(
        self,
        tray_id: int,
    ) -> Optional[ManagedRobot]:
        candidates = [
            managed
            for managed in self._robots.values()
            if (
                tray_id
                in managed.profile.reachable_trays
                and (
                    managed.state_machine.state_name
                    == "IDLE"
                )
                and managed.active_task is None
            )
        ]

        if not candidates:
            return None

        return sorted(
            candidates,
            key=lambda item: item.profile.robot_id,
        )[0]

    def _try_acquire_route(
        self,
        *,
        route_id: str,
        robot_id: str,
    ) -> bool:
        """
        잠금이 필요 없는 경로는 항상 성공한다.

        잠금 경로는 소유자가 없을 때만 현재 로봇에게 배정한다.
        같은 로봇의 중복 획득은 성공으로 처리한다.
        """
        route_id = str(route_id).strip().upper()
        robot_id = str(robot_id).strip().upper()

        if route_id not in self._locked_routes:
            return True

        owner = self._route_owners.get(route_id)

        if owner is None:
            self._route_owners[route_id] = robot_id

            print(
                f"[Manager LOCK] acquired: "
                f"route={route_id}, robot={robot_id}",
                flush=True,
            )
            return True

        if owner == robot_id:
            return True

        print(
            f"[Manager LOCK] denied: "
            f"route={route_id}, "
            f"requester={robot_id}, "
            f"owner={owner}",
            flush=True,
        )
        return False

    def _release_route(
        self,
        *,
        route_id: str,
        robot_id: str,
    ) -> None:
        route_id = str(route_id).strip().upper()
        robot_id = str(robot_id).strip().upper()

        if route_id not in self._locked_routes:
            return

        owner = self._route_owners.get(route_id)

        if owner is None:
            return

        if owner != robot_id:
            print(
                f"[Manager LOCK] release ignored: "
                f"route={route_id}, "
                f"requester={robot_id}, "
                f"owner={owner}",
                flush=True,
            )
            return

        del self._route_owners[route_id]

        print(
            f"[Manager LOCK] released: "
            f"route={route_id}, robot={robot_id}",
            flush=True,
        )

    def _select_route(
        self,
        *,
        managed: ManagedRobot,
    ) -> Optional[str]:
        robot_id = managed.profile.robot_id
        preferred_route = (
            managed.profile.preferred_route
        )

        if self._try_acquire_route(
            route_id=preferred_route,
            robot_id=robot_id,
        ):
            return preferred_route

        fallback_route = (
            managed.profile.fallback_route
        )

        if self._try_acquire_route(
            route_id=fallback_route,
            robot_id=robot_id,
        ):
            print(
                f"[Manager] route fallback: "
                f"robot={robot_id}, "
                f"{preferred_route} -> {fallback_route}",
                flush=True,
            )
            return fallback_route

        return None

    def _create_task(
        self,
        *,
        managed: ManagedRobot,
        tray_id: int,
        route_id: str,
    ) -> RobotTask:
        (
            position,
            orientation,
            joint1_delta_rad,
        ) = self._routes[route_id]

        return RobotTask.create(
            tray_id=tray_id,
            route_id=route_id,
            transit_position=position,
            transit_orientation=orientation,
            joint1_delta_rad=joint1_delta_rad,
            uses_shared_zone=(
                tray_id in self._shared_trays
            ),
        )

    def request_pick_command(
        self,
        tray_index: int,
    ) -> Tuple[bool, str]:
        tray_index = int(tray_index)

        managed = self._select_robot(
            tray_index
        )

        if managed is None:
            return (
                False,
                "No idle robot can reach "
                f"tray {tray_index}",
            )

        route_id = self._select_route(
            managed=managed
        )

        if route_id is None:
            return (
                False,
                f"No available route for "
                f"Robot {managed.profile.robot_id}",
            )

        task = self._create_task(
            managed=managed,
            tray_id=tray_index,
            route_id=route_id,
        )

        accepted, message = (
            managed.state_machine.assign_task(task)
        )

        if not accepted:
            self._release_route(
                route_id=route_id,
                robot_id=(
                    managed.profile.robot_id
                ),
            )
            return False, message

        managed.active_task = task

        return (
            True,
            f"Robot {managed.profile.robot_id} "
            f"assigned tray {tray_index} "
            f"via route {task.route_id}, "
            f"joint1_delta="
            f"{task.joint1_delta_rad:.4f} rad",
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
        for managed in self._robots.values():
            managed.state_machine.step()

            current_state = (
                managed.state_machine.state_name
            )

            if current_state != managed.last_state:
                print(
                    f"[Manager "
                    f"{managed.profile.robot_id}] "
                    f"state: {managed.last_state} "
                    f"-> {current_state}",
                    flush=True,
                )
                managed.last_state = current_state

            # 상태 머신은 트레이 반환과 LIFT가 끝난 뒤
            # current_task를 제거하고 RETURN_HOME으로 전환한다.
            # 이 시점에 공용 경유지 락을 해제한다.
            if (
                managed.active_task is not None
                and (
                    managed.state_machine.current_task
                    is None
                )
            ):
                completed_task = managed.active_task

                self._release_route(
                    route_id=completed_task.route_id,
                    robot_id=(
                        managed.profile.robot_id
                    ),
                )

                print(
                    f"[Manager "
                    f"{managed.profile.robot_id}] "
                    f"task complete: "
                    f"tray={completed_task.tray_id}, "
                    f"route={completed_task.route_id}",
                    flush=True,
                )

                managed.active_task = None

    def reset(self) -> None:
        self._route_owners.clear()

        for managed in self._robots.values():
            managed.active_task = None
            managed.state_machine.reset()
            managed.last_state = (
                managed.state_machine.state_name
            )

        print(
            "[Manager] reset: all route locks cleared",
            flush=True,
        )

    def get_route_owner(
        self,
        route_id: str,
    ) -> Optional[str]:
        """디버깅용 경로 락 소유자 조회."""
        return self._route_owners.get(
            str(route_id).strip().upper()
        )