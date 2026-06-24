# m0609_state_machine.py

from __future__ import annotations

from enum import Enum, auto
from typing import Optional, Sequence, Tuple

import numpy as np

from isaacsim.core.utils.types import ArticulationAction

from m0609_ros_bridge import (
    get_latest_hand_mode,
    get_latest_hand_target,
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
    JOINT_PLACE = auto()
    RELEASE = auto()
    LIFT = auto()
    RETURN_STAGING = auto()


class M0609StateMachine:
    """
    상위 상태:
        IDLE -> PICK -> TRACKING -> PLACE -> IDLE

    동작:
        IDLE:
            임시구역에서 대기
            /m0609/pick_command 값 7을 기다림

        PICK:
            7번 트레이 피킹
            임시구역 이동
            TRACKING 전환

        TRACKING:
            /hand_xyz 추종
            /hand_mode == HOME 수신 시 PLACE 전환

        PLACE:
            임시구역 복귀
            피킹 위치로 이동
            저장된 피킹 관절로 정밀 안착
            그리퍼 해제
            임시구역 복귀
            IDLE 전환
    """

    def __init__(
        self,
        *,
        robot,
        tracking_controller,
        pick_place_controller,
        move_controller,
        pick_position: Sequence[float],
        tray_position: Sequence[float],
        staging_position: Sequence[float],
        tool_orientation: Sequence[float],
        pick_default_ee_offset: Sequence[float],
        pick_approach_z_correction: float,
        place_link6_above_tray: float,
        place_high_offset: float,
        place_approach_gap: float,
        place_joint_tolerance: float,
        place_joint_step: float,
        place_settle_frames: int,
        supported_tray_index: int = 7,
    ) -> None:
        self.robot = robot
        self.tracking_controller = tracking_controller
        self.pick_place_controller = pick_place_controller
        self.move_controller = move_controller

        self.pick_position = np.asarray(
            pick_position,
            dtype=np.float64,
        )
        self.tray_position = np.asarray(
            tray_position,
            dtype=np.float64,
        )
        self.staging_position = np.asarray(
            staging_position,
            dtype=np.float64,
        )
        self.tool_orientation = np.asarray(
            tool_orientation,
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
        self.place_joint_tolerance = float(
            place_joint_tolerance
        )
        self.place_joint_step = float(
            place_joint_step
        )
        self.place_settle_frames = int(
            place_settle_frames
        )
        self.supported_tray_index = int(
            supported_tray_index
        )

        self._state = M0609State.IDLE
        self._state_entered = True

        self._pick_phase = PickPhase.PICKING
        self._place_phase = PlacePhase.MOVE_STAGING

        self._pending_pick_index: Optional[int] = None
        self._saved_pick_joints: Optional[np.ndarray] = None

        self._idle_at_staging = False
        self._place_settle_count = 0

        self._last_hand_mode_sequence = -1
        self._tracking_error_reported = False

        print(
            "[StateMachine] 초기 상태: IDLE",
            flush=True,
        )

    @property
    def state(self) -> M0609State:
        return self._state

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

        if tray_index != self.supported_tray_index:
            return (
                False,
                f"Only tray {self.supported_tray_index} "
                "is supported",
            )

        if self._state != M0609State.IDLE:
            return (
                False,
                f"Pick command rejected in "
                f"{self._state.name}",
            )

        if self._pending_pick_index is not None:
            return (
                False,
                "A pick command is already pending",
            )

        self._pending_pick_index = tray_index

        return (
            True,
            f"Tray {tray_index} pick command accepted",
        )

    def request_move(
        self,
        x: float,
        y: float,
        z: float,
    ) -> Tuple[bool, str]:
        # 기존 좌표 테스트 토픽과의 호환용.
        return (
            False,
            "Direct move command is disabled in workflow mode",
        )

    def _set_move_target(
        self,
        position: np.ndarray,
    ) -> None:
        self.move_controller.reset()
        self.move_controller.set_target(
            position=np.asarray(
                position,
                dtype=np.float64,
            ),
            orientation=self.tool_orientation,
        )

    def _limit_joint_step(
        self,
        current: np.ndarray,
        target: np.ndarray,
    ) -> np.ndarray:
        current = np.asarray(
            current,
            dtype=np.float64,
        )
        target = np.asarray(
            target,
            dtype=np.float64,
        )

        delta = target - current
        delta = np.clip(
            delta,
            -self.place_joint_step,
            self.place_joint_step,
        )

        return current + delta

    def _place_targets(self):
        base = np.array(
            [
                self.tray_position[0],
                self.tray_position[1],
                (
                    self.tray_position[2]
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

    # ========================================================
    # 상태 진입
    # ========================================================
    def _on_enter_idle(self) -> None:
        self.robot.gripper.open()

        self._idle_at_staging = False
        self._set_move_target(
            self.staging_position
        )

        print(
            "[IDLE] 임시구역 이동/대기",
            flush=True,
        )

    def _on_enter_pick(self) -> None:
        self._pick_phase = PickPhase.PICKING
        self._saved_pick_joints = None

        self.pick_place_controller.reset()

        print(
            f"[PICK] tray "
            f"{self.supported_tray_index} 피킹 시작",
            flush=True,
        )

    def _on_enter_tracking(self) -> None:
        self.tracking_controller.reset()
        self._tracking_error_reported = False

        # TRACKING 진입 전에 들어온 HOME 메시지는 무시하고,
        # 진입 이후 새 HOME 메시지만 처리한다.
        _, mode_sequence = get_latest_hand_mode()
        self._last_hand_mode_sequence = (
            mode_sequence
        )

        print(
            "[TRACKING] 손 추종 시작",
            flush=True,
        )

    def _on_enter_place(self) -> None:
        self._place_phase = PlacePhase.MOVE_STAGING
        self._place_settle_count = 0

        self._set_move_target(
            self.staging_position
        )

        print(
            "[PLACE] 먼저 임시구역으로 복귀",
            flush=True,
        )

    # ========================================================
    # 상태 처리
    # ========================================================
    def _step_idle(self) -> None:
        if not self._idle_at_staging:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._idle_at_staging = True
                print(
                    "[IDLE] 임시구역 도착, 명령 대기",
                    flush=True,
                )

            return

        if self._pending_pick_index is None:
            return

        self._pending_pick_index = None
        self._change_state(
            M0609State.PICK
        )

    def _step_pick(self) -> None:
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

            actions = (
                self.pick_place_controller.forward(
                    picking_position=(
                        self.pick_position
                    ),
                    placing_position=(
                        self.pick_position
                    ),
                    current_joint_positions=(
                        self.robot
                        .get_joint_positions()
                    ),
                    end_effector_offset=ee_offset,
                    end_effector_orientation=(
                        self.tool_orientation
                    ),
                )
            )

            self.robot.apply_action(actions)

            if event >= 4:
                if self._saved_pick_joints is None:
                    self._saved_pick_joints = (
                        self.robot
                        .get_joint_positions()
                        .copy()
                    )

                self._set_move_target(
                    self.staging_position
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
                print(
                    "[PICK] 임시구역 도착",
                    flush=True,
                )
                self._change_state(
                    M0609State.TRACKING
                )

    def _step_tracking(self) -> None:
        hand_mode, mode_sequence = (
            get_latest_hand_mode()
        )

        if (
            mode_sequence
            != self._last_hand_mode_sequence
        ):
            self._last_hand_mode_sequence = (
                mode_sequence
            )

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
                    "[TRACKING] 오류: "
                    f"{error}",
                    flush=True,
                )
                self._tracking_error_reported = True

    def _step_place(self) -> None:
        high, down, lift = (
            self._place_targets()
        )

        if (
            self._place_phase
            == PlacePhase.MOVE_STAGING
        ):
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(high)
                self._place_phase = (
                    PlacePhase.MOVE_HIGH
                )
                print(
                    "[PLACE] 임시구역 도착, "
                    "트레이 상단 이동",
                    flush=True,
                )

        elif (
            self._place_phase
            == PlacePhase.MOVE_HIGH
        ):
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(down)
                self._place_phase = (
                    PlacePhase.MOVE_DOWN
                )
                print(
                    "[PLACE] 트레이 상단 도착, "
                    "접근 시작",
                    flush=True,
                )

        elif (
            self._place_phase
            == PlacePhase.MOVE_DOWN
        ):
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self.move_controller.reset()
                self._place_phase = (
                    PlacePhase.JOINT_PLACE
                )
                print(
                    "[PLACE] 저장된 피킹 관절로 "
                    "정밀 안착",
                    flush=True,
                )

        elif (
            self._place_phase
            == PlacePhase.JOINT_PLACE
        ):
            if self._saved_pick_joints is None:
                raise RuntimeError(
                    "Saved pick joints are missing"
                )

            current = np.asarray(
                self.robot.get_joint_positions(),
                dtype=np.float64,
            )

            stepped = self._limit_joint_step(
                current,
                self._saved_pick_joints,
            )

            self.robot.apply_action(
                ArticulationAction(
                    joint_positions=stepped
                )
            )

            error = float(
                np.linalg.norm(
                    current
                    - self._saved_pick_joints
                )
            )

            if (
                error
                < self.place_joint_tolerance
            ):
                self._place_settle_count += 1

                if (
                    self._place_settle_count
                    >= self.place_settle_frames
                ):
                    self._place_phase = (
                        PlacePhase.RELEASE
                    )
                    self._place_settle_count = 0
            else:
                self._place_settle_count = 0

        elif (
            self._place_phase
            == PlacePhase.RELEASE
        ):
            self.robot.gripper.open()

            self._set_move_target(lift)
            self._place_phase = (
                PlacePhase.LIFT
            )

            print(
                "[PLACE] 그리퍼 해제, 상승",
                flush=True,
            )

        elif (
            self._place_phase
            == PlacePhase.LIFT
        ):
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(
                    self.staging_position
                )
                self._place_phase = (
                    PlacePhase.RETURN_STAGING
                )

                print(
                    "[PLACE] 임시구역 복귀",
                    flush=True,
                )

        elif (
            self._place_phase
            == PlacePhase.RETURN_STAGING
        ):
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._saved_pick_joints = None

                print(
                    "[PLACE] 완료",
                    flush=True,
                )

                self._change_state(
                    M0609State.IDLE
                )

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

        self._pending_pick_index = None
        self._saved_pick_joints = None

        self._idle_at_staging = False
        self._place_settle_count = 0

        self._last_hand_mode_sequence = -1
        self._tracking_error_reported = False

        self.pick_place_controller.reset()
        self.move_controller.reset()

        print(
            "[StateMachine] reset -> IDLE",
            flush=True,
        )