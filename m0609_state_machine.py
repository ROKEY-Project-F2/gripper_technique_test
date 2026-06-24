# m0609_state_machine.py

from __future__ import annotations

from enum import Enum, auto
from typing import Optional, Sequence, Tuple

import numpy as np

from m0609_ros_bridge import (
    get_latest_hand_mode,
    get_latest_hand_target,
    reset_hand_mode_cache,
)
from temp_dynamic_trays import (
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
    LIFT = auto()
    RETURN_STAGING = auto()


class M0609StateMachine:
    def __init__(
        self,
        *,
        robot,
        tray_registry,
        tracking_controller,
        pick_place_controller,
        move_controller,
        staging_position: Sequence[float],
        staging_orientation: Sequence[float],
        pick_default_ee_offset: Sequence[float],
        pick_approach_z_correction: float,
        place_link6_above_tray: float,
        place_high_offset: float,
        place_approach_gap: float,
        supported_tray_commands: Sequence[int],
    ) -> None:
        self.robot = robot
        self.tray_registry = tray_registry
        self.tracking_controller = tracking_controller
        self.pick_place_controller = pick_place_controller
        self.move_controller = move_controller

        self.staging_position = np.asarray(
            staging_position,
            dtype=np.float64,
        )
        self.staging_orientation = np.asarray(
            staging_orientation,
            dtype=np.float64,
        )
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
        self.supported_tray_commands = tuple(
            int(value)
            for value in supported_tray_commands
        )

        self._state = M0609State.IDLE
        self._state_entered = True

        self._pick_phase = PickPhase.PICKING
        self._place_phase = PlacePhase.MOVE_STAGING

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

        print(
            "[StateMachine] 초기 상태: IDLE",
            flush=True,
        )

    @property
    def state_name(self) -> str:
        return self._state.name

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
            f"[StateMachine] {old_state.name} -> "
            f"{new_state.name}",
            flush=True,
        )

    def request_pick_command(
        self,
        tray_index: int,
    ) -> Tuple[bool, str]:
        tray_index = int(tray_index)

        if tray_index not in self.supported_tray_commands:
            return (
                False,
                "Supported trays: "
                f"{self.supported_tray_commands}",
            )

        if not self.tray_registry.has_tray(
            tray_index
        ):
            return (
                False,
                f"Tray {tray_index} is not registered",
            )

        if self._state != M0609State.IDLE:
            return (
                False,
                f"Pick rejected in {self._state.name}",
            )

        self._pending_pick_index = tray_index

        return (
            True,
            f"Tray {tray_index} accepted",
        )

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
            self.staging_position,
            self.staging_orientation,
        )

        print(
            "[IDLE] 임시구역 이동/대기",
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
            reset_hand_mode_cache("TRACKING")
        )

        print(
            "[TRACKING] 손 추종 시작 "
            "(hand mode reset)",
            flush=True,
        )

    def _on_enter_place(self) -> None:
        self._place_phase = PlacePhase.MOVE_STAGING

        self._set_move_target(
            self.staging_position,
            self.staging_orientation,
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
                    self.staging_position,
                    self.staging_orientation,
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
            get_latest_hand_mode()
        )

        if sequence != self._last_hand_mode_sequence:
            self._last_hand_mode_sequence = sequence

            if str(hand_mode).strip().upper() == "HOME":
                self._change_state(
                    M0609State.PLACE
                )
                return

        hand_target, _ = (
            get_latest_hand_target()
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
            self.robot.gripper.open()

            self._set_move_target(
                lift,
                self._place_tool_orientation,
            )
            self._place_phase = PlacePhase.LIFT

        elif self._place_phase == PlacePhase.LIFT:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(
                    self.staging_position,
                    self.staging_orientation,
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
        self._pending_pick_index = None
        self._active_tray_id = None
        self._pick_position = None
        self._pick_orientation = None
        self._place_reference_position = None
        self._place_reference_orientation = None
        self._place_tool_orientation = None

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
            reset_hand_mode_cache("TRACKING")
        )

        print(
            "[StateMachine] reset -> IDLE",
            flush=True,
        )