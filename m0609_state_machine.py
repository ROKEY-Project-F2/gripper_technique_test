# m0609_state_machine.py

from __future__ import annotations

from enum import Enum, auto
from typing import Optional, Sequence, Tuple

import numpy as np

from hand_input import HandInput
from robot_runtime import RobotTask
from m0609_dynamic_scene import (
    downward_tool_orientation_for_tray,
)


class M0609State(Enum):
    IDLE = auto()
    PICK = auto()
    TRACKING = auto()
    PLACE = auto()


class PickPhase(Enum):
    PICKING = auto()
    MOVE_STAGING = auto()


class PlacePhase(Enum):
    MOVE_STAGING = auto()
    MOVE_HIGH = auto()
    MOVE_DOWN = auto()
    RELEASE = auto()
    RELEASE_WAIT = auto()
    LIFT = auto()
    RETURN_STAGING = auto()


class M0609StateMachine:
    def __init__(
        self,
        *,
        robot_id: str,
        robot,
        tray_registry,
        hand_input: HandInput,
        idle_position: Sequence[float],
        idle_orientation: Sequence[float],
        tracking_controller,
        pick_place_controller,
        move_controller,
        pick_default_ee_offset: Sequence[float],
        pick_approach_z_correction: float,
        place_link6_above_tray: float,
        place_high_offset: float,
        place_approach_gap: float,
        transport_z_offset: float,
        place_release_min_wait_frames: int,
        place_release_stable_frames: int,
        place_release_retry_interval: int,
        place_release_timeout_frames: int,
    ) -> None:
        self.robot_id = str(robot_id).strip().upper()
        self.robot = robot
        self.tray_registry = tray_registry
        self.hand_input = hand_input
        self.idle_position = np.asarray(
            idle_position,
            dtype=np.float64,
        )
        self.idle_orientation = np.asarray(
            idle_orientation,
            dtype=np.float64,
        )
        self.tracking_controller = tracking_controller
        self.pick_place_controller = pick_place_controller
        self.move_controller = move_controller

        self.pick_default_ee_offset = np.asarray(
            pick_default_ee_offset,
            dtype=np.float64,
        )

        self.pick_approach_z_correction = float(
            pick_approach_z_correction
        )
        self.place_link6_above_tray = float(
            place_link6_above_tray
        )
        self.place_high_offset = float(
            place_high_offset
        )
        self.place_approach_gap = float(
            place_approach_gap
        )
        self.transport_z_offset = float(
            transport_z_offset
        )

        self.place_release_min_wait_frames = int(
            place_release_min_wait_frames
        )
        self.place_release_stable_frames = int(
            place_release_stable_frames
        )
        self.place_release_retry_interval = int(
            place_release_retry_interval
        )
        self.place_release_timeout_frames = int(
            place_release_timeout_frames
        )

        if self.place_release_min_wait_frames < 0:
            raise ValueError(
                "place_release_min_wait_frames must be >= 0"
            )
        if self.place_release_stable_frames <= 0:
            raise ValueError(
                "place_release_stable_frames must be > 0"
            )
        if self.place_release_retry_interval <= 0:
            raise ValueError(
                "place_release_retry_interval must be > 0"
            )
        if self.place_release_timeout_frames <= 0:
            raise ValueError(
                "place_release_timeout_frames must be > 0"
            )

        self._state = M0609State.IDLE
        self._state_entered = True

        self._pick_phase = PickPhase.PICKING
        self._place_phase = PlacePhase.MOVE_STAGING

        self._current_task: Optional[RobotTask] = None
        self._pending_pick_index: Optional[int] = None
        self._active_tray_id: Optional[int] = None

        # PICK 순간 실제 위치/방향
        self._pick_position: Optional[np.ndarray] = None
        self._pick_orientation: Optional[np.ndarray] = None

        # 생성 당시 원복 기준 위치/방향
        self._place_reference_position: Optional[
            np.ndarray
        ] = None
        self._place_reference_orientation: Optional[
            np.ndarray
        ] = None
        self._place_tool_orientation: Optional[
            np.ndarray
        ] = None

        self._idle_at_staging = False
        self._last_hand_mode_sequence = -1
        self._tracking_error_reported = False

        self._release_wait_frames = 0
        self._release_stable_frames = 0
        self._release_timeout_reported = False

        print(
            f"[StateMachine {self.robot_id}] 초기 상태: IDLE",
            flush=True,
        )

    @property
    def state_name(self) -> str:
        return self._state.name

    @property
    def current_task(self) -> Optional[RobotTask]:
        return self._current_task

    def _change_state(
        self,
        new_state: M0609State,
    ) -> None:
        if new_state == self._state:
            return

        old_state = self._state
        self._state = new_state
        self._state_entered = True

        print(
            f"[StateMachine {self.robot_id}] {old_state.name} -> "
            f"{new_state.name}",
            flush=True,
        )

    def assign_task(
        self,
        task: RobotTask,
    ) -> Tuple[bool, str]:
        if self._state != M0609State.IDLE:
            return False, f"Task rejected in {self._state.name}"

        if self._current_task is not None:
            return False, "A task is already assigned"

        if not self.tray_registry.has_tray(task.tray_id):
            return False, f"Tray {task.tray_id} is not registered"

        self._current_task = task
        self._pending_pick_index = task.tray_id
        return True, f"Tray {task.tray_id} accepted"

    def request_move(
        self,
        x: float,
        y: float,
        z: float,
    ) -> Tuple[bool, str]:
        return (
            False,
            "Direct move is disabled in workflow mode",
        )

    def _require_task(self) -> RobotTask:
        if self._current_task is None:
            raise RuntimeError("No task is assigned")
        return self._current_task

    def _set_move_target(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> None:
        self.move_controller.reset()
        self.move_controller.set_target(
            position=np.asarray(
                position,
                dtype=np.float64,
            ),
            orientation=np.asarray(
                orientation,
                dtype=np.float64,
            ),
        )

    def _get_transport_position(self) -> np.ndarray:
        """
        작업 경유 위치에 이동용 Z 오프셋을 적용한다.

        PICK 직후 이동과 PLACE 복귀 이동에서 동일하게 사용한다.
        상태 머신 내부에 특정 좌표를 하드코딩하지 않고,
        매니저가 전달한 RobotTask 위치를 기준으로 계산한다.
        """
        position = (
            self._require_task()
            .transit_position
            .copy()
        )
        position[2] += self.transport_z_offset
        return position

    def _calculate_place_targets(self):
        if self._place_reference_position is None:
            raise RuntimeError(
                "Place reference position is missing"
            )

        base = np.array(
            [
                self._place_reference_position[0],
                self._place_reference_position[1],
                (
                    self._place_reference_position[2]
                    + self.place_link6_above_tray
                ),
            ],
            dtype=np.float64,
        )

        high = base + np.array(
            [0.0, 0.0, self.place_high_offset],
            dtype=np.float64,
        )
        down = base + np.array(
            [0.0, 0.0, self.place_approach_gap],
            dtype=np.float64,
        )
        lift = base + np.array(
            [
                0.0,
                0.0,
                self.place_approach_gap + 0.05,
            ],
            dtype=np.float64,
        )

        return high, down, lift

    def _on_enter_idle(self) -> None:
        self.robot.gripper.open()
        self._idle_at_staging = False

        self._set_move_target(
            self.idle_position,
            self.idle_orientation,
        )

        print(
            "[IDLE] 대기 위치 이동/대기",
            flush=True,
        )

    def _on_enter_pick(self) -> None:
        if self._pending_pick_index is None:
            raise RuntimeError(
                "PICK entered without command"
            )

        snapshot = self.tray_registry.get_snapshot(
            self._pending_pick_index
        )

        self._active_tray_id = (
            self._pending_pick_index
        )

        # 피킹 순간 실제 pose
        self._pick_position = (
            snapshot.pick_position.copy()
        )
        self._pick_orientation = (
            snapshot.pick_orientation.copy()
        )

        # 생성 당시 원복 pose
        self._place_reference_position = (
            snapshot.spawn_reference_position.copy()
        )
        self._place_reference_orientation = (
            snapshot.spawn_orientation.copy()
        )
        self._place_tool_orientation = (
            downward_tool_orientation_for_tray(
                snapshot.spawn_orientation
            )
        )

        self._pick_phase = PickPhase.PICKING
        self.pick_place_controller.reset()

        print(
            f"[PICK] tray={self._active_tray_id}, "
            f"position={self._pick_position.round(4)}, "
            f"orientation={self._pick_orientation.round(4)}",
            flush=True,
        )

    def _on_enter_tracking(self) -> None:
        self.tracking_controller.reset()
        self._tracking_error_reported = False

        # 이전 사이클의 HOME 값이 남아 다음 사이클에 영향을
        # 주지 않도록 TRACKING 진입 시 캐시를 명시적으로 초기화한다.
        self._last_hand_mode_sequence = (
            self.hand_input.reset_mode("TRACKING")
        )

        print(
            "[TRACKING] 손 추종 시작 "
            "(hand mode reset)",
            flush=True,
        )

    def _on_enter_place(self) -> None:
        self._place_phase = PlacePhase.MOVE_STAGING
        self._release_wait_frames = 0
        self._release_stable_frames = 0
        self._release_timeout_reported = False

        self._set_move_target(
            self._get_transport_position(),
            self._require_task().transit_orientation,
        )

        print(
            "[PLACE] 임시구역으로 복귀",
            flush=True,
        )

    def _step_idle(self) -> None:
        if not self._idle_at_staging:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._idle_at_staging = True
                print(
                    "[IDLE] 명령 대기",
                    flush=True,
                )
            return

        if self._pending_pick_index is not None:
            self._change_state(
                M0609State.PICK
            )

    def _step_pick(self) -> None:
        if (
            self._pick_position is None
            or self._pick_orientation is None
        ):
            raise RuntimeError(
                "Dynamic pick pose is missing"
            )

        if self._pick_phase == PickPhase.PICKING:
            event = (
                self.pick_place_controller
                .get_current_event()
            )

            ee_offset = (
                self.pick_default_ee_offset.copy()
            )

            if event in (1, 2, 3):
                ee_offset[2] -= (
                    self.pick_approach_z_correction
                )

            actions = self.pick_place_controller.forward(
                picking_position=self._pick_position,
                placing_position=self._pick_position,
                current_joint_positions=(
                    self.robot.get_joint_positions()
                ),
                end_effector_offset=ee_offset,
                end_effector_orientation=(
                    self._pick_orientation
                ),
            )

            self.robot.apply_action(actions)

            if event >= 4:
                self._set_move_target(
                    self._get_transport_position(),
                    self._require_task().transit_orientation,
                )
                self._pick_phase = (
                    PickPhase.MOVE_STAGING
                )

                print(
                    "[PICK] 흡착 완료, 임시구역 이동",
                    flush=True,
                )

        elif (
            self._pick_phase
            == PickPhase.MOVE_STAGING
        ):
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._change_state(
                    M0609State.TRACKING
                )

    def _step_tracking(self) -> None:
        hand_mode, sequence = (
            self.hand_input.get_mode()
        )

        if sequence != self._last_hand_mode_sequence:
            self._last_hand_mode_sequence = sequence

            if str(hand_mode).strip().upper() == "HOME":
                self._change_state(
                    M0609State.PLACE
                )
                return

        hand_target, _ = (
            self.hand_input.get_target()
        )

        if hand_target is None:
            return

        try:
            self.tracking_controller.update(
                hand_target
            )
            self._tracking_error_reported = False

        except Exception as error:
            if not self._tracking_error_reported:
                print(
                    f"[TRACKING] 오류: {error}",
                    flush=True,
                )
                self._tracking_error_reported = True

    def _step_place(self) -> None:
        if self._place_tool_orientation is None:
            raise RuntimeError(
                "Place orientation is missing"
            )

        high, down, lift = (
            self._calculate_place_targets()
        )

        if self._place_phase == PlacePhase.MOVE_STAGING:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(
                    high,
                    self._place_tool_orientation,
                )
                self._place_phase = (
                    PlacePhase.MOVE_HIGH
                )

        elif self._place_phase == PlacePhase.MOVE_HIGH:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(
                    down,
                    self._place_tool_orientation,
                )
                self._place_phase = (
                    PlacePhase.MOVE_DOWN
                )

        elif self._place_phase == PlacePhase.MOVE_DOWN:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._place_phase = (
                    PlacePhase.RELEASE
                )

        elif self._place_phase == PlacePhase.RELEASE:
            # open() 반환값은 명령 호출 성공 여부다.
            # 실제 attachment 해제는 RELEASE_WAIT에서 별도로 확인한다.
            self.robot.gripper.open()

            self._release_wait_frames = 0
            self._release_stable_frames = 0
            self._release_timeout_reported = False

            self._place_phase = (
                PlacePhase.RELEASE_WAIT
            )

            print(
                "[PLACE] 듀얼 그리퍼 해제 대기 시작",
                flush=True,
            )

        elif (
            self._place_phase
            == PlacePhase.RELEASE_WAIT
        ):
            self._release_wait_frames += 1

            fully_released = bool(
                self.robot.gripper
                .is_fully_released()
            )

            if fully_released:
                self._release_stable_frames += 1
            else:
                self._release_stable_frames = 0

            # attachment가 남아 있으면 일정 간격으로 양쪽 open 재호출.
            if (
                not fully_released
                and self._release_wait_frames
                % self.place_release_retry_interval
                == 0
            ):
                print(
                    "[PLACE] attachment 잔류, "
                    "open 재시도: "
                    f"frame={self._release_wait_frames}, "
                    f"states="
                    f"{self.robot.gripper.get_release_states()}",
                    flush=True,
                )
                self.robot.gripper.open()

            waited_long_enough = (
                self._release_wait_frames
                >= self.place_release_min_wait_frames
            )

            released_stably = (
                self._release_stable_frames
                >= self.place_release_stable_frames
            )

            if (
                waited_long_enough
                and released_stably
            ):
                print(
                    "[PLACE] 듀얼 그리퍼 완전 해제 확인: "
                    f"wait={self._release_wait_frames}, "
                    f"stable={self._release_stable_frames}",
                    flush=True,
                )

                self._set_move_target(
                    lift,
                    self._place_tool_orientation,
                )
                self._place_phase = (
                    PlacePhase.LIFT
                )

            elif (
                self._release_wait_frames
                >= self.place_release_timeout_frames
                and not fully_released
            ):
                # 해제가 확인되지 않은 상태에서 상승하면 도구를 끌 수 있다.
                # 따라서 상승하지 않고 그 자리에서 계속 open을 재시도한다.
                if not self._release_timeout_reported:
                    print(
                        "[PLACE] 그리퍼 해제 타임아웃. "
                        "상승하지 않고 해제를 계속 재시도합니다: "
                        f"{self.robot.gripper.get_release_states()}",
                        flush=True,
                    )
                    self._release_timeout_reported = True

        elif self._place_phase == PlacePhase.LIFT:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(
                    self._get_transport_position(),
                    self._require_task().transit_orientation,
                )
                self._place_phase = (
                    PlacePhase.RETURN_STAGING
                )

        elif (
            self._place_phase
            == PlacePhase.RETURN_STAGING
        ):
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._clear_active_tray()
                self._change_state(
                    M0609State.IDLE
                )

    def _clear_active_tray(self) -> None:
        self._current_task = None
        self._pending_pick_index = None
        self._active_tray_id = None
        self._pick_position = None
        self._pick_orientation = None
        self._place_reference_position = None
        self._place_reference_orientation = None
        self._place_tool_orientation = None
        self._release_wait_frames = 0
        self._release_stable_frames = 0
        self._release_timeout_reported = False

    def step(self) -> None:
        if self._state_entered:
            self._state_entered = False

            if self._state == M0609State.IDLE:
                self._on_enter_idle()
            elif self._state == M0609State.PICK:
                self._on_enter_pick()
            elif self._state == M0609State.TRACKING:
                self._on_enter_tracking()
            elif self._state == M0609State.PLACE:
                self._on_enter_place()

        if self._state == M0609State.IDLE:
            self._step_idle()
        elif self._state == M0609State.PICK:
            self._step_pick()
        elif self._state == M0609State.TRACKING:
            self._step_tracking()
        elif self._state == M0609State.PLACE:
            self._step_place()

    def reset(self) -> None:
        self._state = M0609State.IDLE
        self._state_entered = True
        self._pick_phase = PickPhase.PICKING
        self._place_phase = PlacePhase.MOVE_STAGING
        self._idle_at_staging = False
        self._last_hand_mode_sequence = -1
        self._tracking_error_reported = False

        self._clear_active_tray()

        self.pick_place_controller.reset()
        self.move_controller.reset()

        self._last_hand_mode_sequence = (
            self.hand_input.reset_mode("TRACKING")
        )

        print(
            f"[StateMachine {self.robot_id}] reset -> IDLE",
            flush=True,
        )