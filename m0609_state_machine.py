# m0609_state_machine.py

from __future__ import annotations

from enum import Enum, auto
from typing import Optional, Sequence, Tuple

import numpy as np
from isaacsim.core.utils.types import ArticulationAction

from hand_input import HandInput
from m0609_dynamic_scene import downward_tool_orientation_for_tray
from robot_runtime import RobotTask


class M0609State(Enum):
    IDLE = auto()
    PICK = auto()
    TRACKING = auto()
    PLACE = auto()
    RETURN_HOME = auto()


class PickPhase(Enum):
    PICKING = auto()
    MOVE_TRANSIT = auto()
    ROTATE_JOINT1_OUT = auto()


class PlacePhase(Enum):
    MOVE_SAFE_RETURN = auto()
    ROTATE_JOINT1_BACK = auto()
    MOVE_TRANSIT = auto()
    MOVE_HIGH = auto()
    MOVE_DOWN = auto()
    RELEASE = auto()
    RELEASE_WAIT = auto()
    LIFT = auto()


class M0609StateMachine:
    def __init__(
        self,
        *,
        robot_id: str,
        robot,
        tray_registry,
        hand_input: HandInput,
        idle_joint_positions: Sequence[float],
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
        joint1_turn_tolerance_rad: float,
        joint1_turn_max_step_rad: float,
        safe_joint_return_max_step_rad: float = np.deg2rad(0.35),
        safe_joint_return_tolerance_rad: float = np.deg2rad(1.0),
        return_home_max_step_rad: float = np.deg2rad(0.20),
        return_home_wrist_max_step_rad: float = np.deg2rad(0.60),
        idle_joint_tolerance: float = np.deg2rad(1.0),
    ) -> None:
        self.robot_id = str(robot_id).strip().upper()
        self.robot = robot
        self.tray_registry = tray_registry
        self.hand_input = hand_input

        # initialize_robot() 직후 저장한 최초 관절 자세.
        # 작업 종료 후 RETURN_HOME 상태에서 이 자세로 복귀한다.
        self.idle_joint_positions = np.asarray(
            idle_joint_positions,
            dtype=np.float64,
        ).copy()

        if (
            self.idle_joint_positions.ndim != 1
            or self.idle_joint_positions.size == 0
            or not np.all(np.isfinite(self.idle_joint_positions))
        ):
            raise ValueError(
                "idle_joint_positions must be a finite 1-D array"
            )

        self.return_home_max_step_rad = float(
            return_home_max_step_rad
        )
        self.return_home_wrist_max_step_rad = float(
            return_home_wrist_max_step_rad
        )
        self.idle_joint_tolerance = float(idle_joint_tolerance)

        if self.return_home_max_step_rad <= 0.0:
            raise ValueError(
                "return_home_max_step_rad must be > 0"
            )

        if self.return_home_wrist_max_step_rad <= 0.0:
            raise ValueError(
                "return_home_wrist_max_step_rad must be > 0"
            )

        if self.idle_joint_tolerance <= 0.0:
            raise ValueError(
                "idle_joint_tolerance must be > 0"
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
        self.joint1_turn_tolerance_rad = float(
            joint1_turn_tolerance_rad
        )
        self.joint1_turn_max_step_rad = float(
            joint1_turn_max_step_rad
        )
        self.safe_joint_return_max_step_rad = float(
            safe_joint_return_max_step_rad
        )
        self.safe_joint_return_tolerance_rad = float(
            safe_joint_return_tolerance_rad
        )

        if self.joint1_turn_tolerance_rad <= 0.0:
            raise ValueError(
                "joint1_turn_tolerance_rad must be > 0"
            )

        if self.joint1_turn_max_step_rad <= 0.0:
            raise ValueError(
                "joint1_turn_max_step_rad must be > 0"
            )

        if self.safe_joint_return_max_step_rad <= 0.0:
            raise ValueError(
                "safe_joint_return_max_step_rad must be > 0"
            )

        if self.safe_joint_return_tolerance_rad <= 0.0:
            raise ValueError(
                "safe_joint_return_tolerance_rad must be > 0"
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

        # 최초 실행 시 initialize_robot()으로 이미 최초 관절 자세에 있으므로
        # 실제 대기 상태인 IDLE에서 시작한다.
        self._state = M0609State.IDLE
        self._state_entered = True

        self._pick_phase = PickPhase.PICKING
        self._place_phase = PlacePhase.MOVE_SAFE_RETURN

        # 경유지 도착 당시의 joint1 각도와 회전 목표.
        self._joint1_before_tracking: Optional[float] = None
        self._joint1_target: Optional[float] = None

        # joint1 90도 회전이 끝난 순간의 전체 관절 자세.
        # PLACE 명령 시 Cartesian IK가 아니라 이 관절 자세로 복귀한다.
        self._safe_tracking_joint_positions: Optional[np.ndarray] = None

        self._current_task: Optional[RobotTask] = None
        self._pending_pick_index: Optional[int] = None
        self._active_tray_id: Optional[int] = None

        # PICK 순간 실제 트레이 pose
        self._pick_position: Optional[np.ndarray] = None
        self._pick_orientation: Optional[np.ndarray] = None

        # 생성 당시 원복 기준 pose
        self._place_reference_position: Optional[np.ndarray] = None
        self._place_reference_orientation: Optional[np.ndarray] = None
        self._place_tool_orientation: Optional[np.ndarray] = None

        # 흡착 완료 순간의 실제 엔드이펙터 자세.
        # 정방향/반환 경유지 이동에서 동일하게 유지한다.
        self._transport_orientation: Optional[np.ndarray] = None

        self._last_hand_mode_sequence = -1
        self._tracking_error_reported = False

        self._release_wait_frames = 0
        self._release_stable_frames = 0
        self._release_timeout_reported = False

        print(
            f"[StateMachine {self.robot_id}] 초기 상태: IDLE "
            "(center/side PLACE separated)",
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
            f"[StateMachine {self.robot_id}] "
            f"{old_state.name} -> {new_state.name}",
            flush=True,
        )

    def assign_task(
        self,
        task: RobotTask,
    ) -> Tuple[bool, str]:
        # 최초 관절 자세 복귀가 완료되어 실제 IDLE 상태일 때만
        # 새로운 작업을 받는다.
        if self._state != M0609State.IDLE:
            return (
                False,
                f"Task rejected in {self._state.name}",
            )

        if self._current_task is not None:
            return (
                False,
                "A task is already assigned",
            )

        if not self.tray_registry.has_tray(task.tray_id):
            return (
                False,
                f"Tray {task.tray_id} is not registered",
            )

        self._current_task = task
        self._pending_pick_index = task.tray_id

        return (
            True,
            f"Tray {task.tray_id} accepted",
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

    def _require_task(self) -> RobotTask:
        if self._current_task is None:
            raise RuntimeError(
                "No task is assigned"
            )

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
        task = self._require_task()

        position = task.transit_position.copy()
        position[2] += self.transport_z_offset
        return position

    def _capture_transport_orientation(self) -> None:
        end_effector = getattr(
            self.robot,
            "end_effector",
            None,
        )

        if end_effector is None:
            raise RuntimeError(
                "robot.end_effector is not available"
            )

        _, orientation = end_effector.get_world_pose()

        orientation = np.asarray(
            orientation,
            dtype=np.float64,
        )

        if orientation.shape != (4,):
            raise RuntimeError(
                "invalid transport orientation shape: "
                f"{orientation.shape}"
            )

        if not np.all(np.isfinite(orientation)):
            raise RuntimeError(
                "transport orientation contains invalid values"
            )

        self._transport_orientation = orientation.copy()

        print(
            f"[TRANSPORT {self.robot_id}] "
            "흡착 완료 자세 저장: "
            f"{self._transport_orientation.round(5)}",
            flush=True,
        )

    def _set_transit_target(self) -> None:
        if self._transport_orientation is None:
            raise RuntimeError(
                "transport orientation is not captured"
            )

        self._set_move_target(
            self._get_transport_position(),
            self._transport_orientation,
        )

        print(
            f"[TRANSIT {self.robot_id}] "
            f"route={self._require_task().route_id}, "
            f"position={self._get_transport_position().round(4)}, "
            f"orientation={self._transport_orientation.round(5)}",
            flush=True,
        )

    def _start_joint1_rotation(
        self,
        target_rad: float,
    ) -> None:
        self._joint1_target = float(target_rad)

        print(
            f"[JOINT1 {self.robot_id}] "
            f"target={np.degrees(self._joint1_target):.2f} deg",
            flush=True,
        )

    def _step_joint1_rotation(self) -> bool:
        if self._joint1_target is None:
            raise RuntimeError(
                "joint1 target is not set"
            )

        current = np.asarray(
            self.robot.get_joint_positions(),
            dtype=np.float64,
        )

        if current.size == 0:
            raise RuntimeError(
                "robot has no joint positions"
            )

        error = float(
            self._joint1_target - current[0]
        )

        if abs(error) <= self.joint1_turn_tolerance_rad:
            print(
                f"[JOINT1 {self.robot_id}] 완료: "
                f"{np.degrees(current[0]):.2f} deg",
                flush=True,
            )
            return True

        # 최종 목표를 한 번에 전달하지 않고, 현재 각도에서
        # 최대 joint1_turn_max_step_rad만큼만 다음 목표를 보낸다.
        step = float(
            np.clip(
                error,
                -self.joint1_turn_max_step_rad,
                self.joint1_turn_max_step_rad,
            )
        )
        next_target = float(current[0] + step)

        self.robot.apply_action(
            ArticulationAction(
                joint_positions=np.array(
                    [next_target],
                    dtype=np.float64,
                ),
                joint_indices=np.array(
                    [0],
                    dtype=np.int64,
                ),
            )
        )

        return False



    def _capture_safe_tracking_pose(self) -> None:
        joints = np.asarray(
            self.robot.get_joint_positions(),
            dtype=np.float64,
        ).copy()

        if joints.ndim != 1 or joints.size == 0:
            raise RuntimeError(
                "invalid safe tracking joint positions"
            )

        self._safe_tracking_joint_positions = joints

        print(
            f"[SAFE {self.robot_id}] "
            "joint1 회전 완료 관절 자세 저장: "
            f"{np.degrees(joints).round(2)} deg",
            flush=True,
        )

    def _step_safe_joint_return(self) -> bool:
        if self._safe_tracking_joint_positions is None:
            raise RuntimeError(
                "safe tracking joint positions are not captured"
            )

        current = np.asarray(
            self.robot.get_joint_positions(),
            dtype=np.float64,
        )

        target = self._safe_tracking_joint_positions

        if current.shape != target.shape:
            raise RuntimeError(
                "safe joint shape mismatch: "
                f"current={current.shape}, target={target.shape}"
            )

        error = target - current
        max_error = float(np.max(np.abs(error)))

        if max_error <= self.safe_joint_return_tolerance_rad:
            print(
                f"[SAFE {self.robot_id}] "
                "회전 완료 관절 자세 복귀 완료",
                flush=True,
            )
            return True

        step = np.clip(
            error,
            -self.safe_joint_return_max_step_rad,
            self.safe_joint_return_max_step_rad,
        )

        next_target = current + step

        self.robot.apply_action(
            ArticulationAction(
                joint_positions=next_target,
            )
        )

        return False


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
        # IDLE은 이동 상태가 아니다.
        # 최초 관절 자세 도착 후 명령을 기다리는 상태다.
        self.robot.gripper.open()

        print(
            f"[IDLE {self.robot_id}] 명령 대기",
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

        self._active_tray_id = self._pending_pick_index

        self._pick_position = snapshot.pick_position.copy()
        self._pick_orientation = snapshot.pick_orientation.copy()

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
        self._transport_orientation = None
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

        self._last_hand_mode_sequence = (
            self.hand_input.reset_mode("TRACKING")
        )

        print(
            "[TRACKING] 손 추종 시작 "
            "(PLACE 명령 대기)",
            flush=True,
        )

    def _is_side_route(self) -> bool:
        """
        joint1 회전량이 있는 TRANSIT_1/3만 좌우 우회 경로로 본다.
        TRANSIT_2는 중앙 경로다.
        """
        task = self._require_task()
        return abs(task.joint1_delta_rad) > 1e-9

    def _enter_center_place(self) -> None:
        """
        중앙 경로 전용 PLACE 진입.

        기존 동작을 유지한다.
        TRACKING 위치에서 중앙 경유지로 이동한 뒤 트레이로 복귀한다.
        joint1 회전 및 안전 관절 자세 복귀는 사용하지 않는다.
        """
        self._place_phase = PlacePhase.MOVE_TRANSIT
        self._set_transit_target()

        print(
            f"[PLACE {self.robot_id}] "
            "중앙 경로 복귀: TRANSIT_2 -> 트레이",
            flush=True,
        )

    def _enter_side_place(self) -> None:
        """
        좌우 우회 경로 전용 PLACE 진입.

        회전 완료 당시 관절 자세로 복귀한 뒤,
        joint1을 원복하고 TRANSIT_1/3을 거쳐 트레이로 돌아간다.
        """
        if self._joint1_before_tracking is None:
            raise RuntimeError(
                "side route joint1 return angle is missing"
            )

        if self._safe_tracking_joint_positions is None:
            raise RuntimeError(
                "side route safe joint pose is missing"
            )

        self._place_phase = PlacePhase.MOVE_SAFE_RETURN

        print(
            f"[PLACE {self.robot_id}] "
            "좌우 경로 복귀: 안전 관절 자세 -> "
            "joint1 원복 -> 경유지 -> 트레이",
            flush=True,
        )

    def _on_enter_place(self) -> None:
        self._release_wait_frames = 0
        self._release_stable_frames = 0
        self._release_timeout_reported = False

        if self._is_side_route():
            self._enter_side_place()
        else:
            self._enter_center_place()

    def _on_enter_return_home(self) -> None:
        # PLACE의 수직 상승이 끝난 뒤 경유지로 다시 가지 않고,
        # 최초 실행 관절 자세로 바로 복귀한다.
        print(
            f"[RETURN_HOME {self.robot_id}] "
            "최초 관절 자세로 복귀",
            flush=True,
        )

    def _step_idle(self) -> None:
        if self._pending_pick_index is not None:
            self._change_state(M0609State.PICK)

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

            ee_offset = self.pick_default_ee_offset.copy()

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
                self._capture_transport_orientation()
                self._set_transit_target()
                self._pick_phase = PickPhase.MOVE_TRANSIT

                print(
                    f"[PICK {self.robot_id}] "
                    "흡착 완료, 저장 자세 유지하며 경유지 이동",
                    flush=True,
                )

        elif self._pick_phase == PickPhase.MOVE_TRANSIT:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                current = np.asarray(
                    self.robot.get_joint_positions(),
                    dtype=np.float64,
                )
                self._joint1_before_tracking = float(
                    current[0]
                )

                delta = (
                    self._require_task().joint1_delta_rad
                )

                if abs(delta) <= 1e-9:
                    # 중앙 경로는 기존 동작을 유지한다.
                    # joint1 회전이나 안전 관절 자세 저장을 사용하지 않는다.
                    self._change_state(
                        M0609State.TRACKING
                    )
                    return

                self._start_joint1_rotation(
                    self._joint1_before_tracking
                    + delta
                )
                self._pick_phase = (
                    PickPhase.ROTATE_JOINT1_OUT
                )

        elif self._pick_phase == PickPhase.ROTATE_JOINT1_OUT:
            if self._step_joint1_rotation():
                # 90도 회전이 끝난 실제 TCP pose 자체를
                # 안전 위치로 저장하고 바로 TRACKING을 시작한다.
                self._capture_safe_tracking_pose()
                self._change_state(
                    M0609State.TRACKING
                )

    def _step_tracking(self) -> None:
        hand_mode, sequence = self.hand_input.get_mode()

        if sequence != self._last_hand_mode_sequence:
            self._last_hand_mode_sequence = sequence

            if str(hand_mode).strip().upper() == "PLACE":
                self._change_state(
                    M0609State.PLACE
                )
                return

        hand_target, _ = self.hand_input.get_target()

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

        high, down, lift = self._calculate_place_targets()

        if self._place_phase == PlacePhase.MOVE_SAFE_RETURN:
            # 좌우 우회 경로 전용
            if self._step_safe_joint_return():
                self._start_joint1_rotation(
                    self._joint1_before_tracking
                )
                self._place_phase = (
                    PlacePhase.ROTATE_JOINT1_BACK
                )

        elif (
            self._place_phase
            == PlacePhase.ROTATE_JOINT1_BACK
        ):
            # 좌우 우회 경로 전용
            if self._step_joint1_rotation():
                self._set_transit_target()
                self._place_phase = (
                    PlacePhase.MOVE_TRANSIT
                )

        elif self._place_phase == PlacePhase.MOVE_TRANSIT:
            # 중앙 경로는 PLACE 진입 직후 여기서 시작한다.
            # 좌우 경로는 관절 원복 후 여기로 들어온다.
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(
                    high,
                    self._place_tool_orientation,
                )
                self._place_phase = PlacePhase.MOVE_HIGH

        elif self._place_phase == PlacePhase.MOVE_HIGH:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._set_move_target(
                    down,
                    self._place_tool_orientation,
                )
                self._place_phase = PlacePhase.MOVE_DOWN

        elif self._place_phase == PlacePhase.MOVE_DOWN:
            self.robot.apply_action(
                self.move_controller.forward()
            )

            if self.move_controller.is_done():
                self._place_phase = PlacePhase.RELEASE

        elif self._place_phase == PlacePhase.RELEASE:
            self.robot.gripper.open()

            self._release_wait_frames = 0
            self._release_stable_frames = 0
            self._release_timeout_reported = False
            self._place_phase = PlacePhase.RELEASE_WAIT

            print(
                "[PLACE] 듀얼 그리퍼 해제 대기 시작",
                flush=True,
            )

        elif self._place_phase == PlacePhase.RELEASE_WAIT:
            self._release_wait_frames += 1

            fully_released = bool(
                self.robot.gripper.is_fully_released()
            )

            if fully_released:
                self._release_stable_frames += 1
            else:
                self._release_stable_frames = 0

            if (
                not fully_released
                and (
                    self._release_wait_frames
                    % self.place_release_retry_interval
                    == 0
                )
            ):
                print(
                    "[PLACE] attachment 잔류, "
                    "open 재시도: "
                    f"frame={self._release_wait_frames}, "
                    "states="
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

            if waited_long_enough and released_stably:
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
                self._place_phase = PlacePhase.LIFT

            elif (
                self._release_wait_frames
                >= self.place_release_timeout_frames
                and not fully_released
            ):
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
                # 트레이 위 수직 상승이 끝나면 작업 데이터는 정리한다.
                # 경유지로 다시 이동하지 않고 RETURN_HOME으로 전환한다.
                self._clear_active_tray()
                self._change_state(
                    M0609State.RETURN_HOME
                )

    def _step_return_home(self) -> None:
        current_joint_positions = np.asarray(
            self.robot.get_joint_positions(),
            dtype=np.float64,
        )

        if (
            current_joint_positions.shape
            != self.idle_joint_positions.shape
        ):
            raise RuntimeError(
                "현재 관절값과 최초 관절값의 크기가 다릅니다: "
                f"current={current_joint_positions.shape}, "
                f"home={self.idle_joint_positions.shape}"
            )

        joint_error = float(
            np.max(
                np.abs(
                    current_joint_positions
                    - self.idle_joint_positions
                )
            )
        )

        if joint_error <= self.idle_joint_tolerance:
            print(
                f"[RETURN_HOME {self.robot_id}] "
                "최초 관절 자세 도착",
                flush=True,
            )
            self._change_state(
                M0609State.IDLE
            )
            return

        error = (
            self.idle_joint_positions
            - current_joint_positions
        )

        # J1~J3는 기존 복귀 속도를 유지한다.
        # 엔드이펙터 방향을 만드는 J4~J6만 더 빠르게 복귀한다.
        max_step = np.full_like(
            error,
            self.return_home_max_step_rad,
            dtype=np.float64,
        )

        if max_step.size >= 6:
            max_step[3:6] = (
                self.return_home_wrist_max_step_rad
            )

        step = np.clip(
            error,
            -max_step,
            max_step,
        )

        next_joint_positions = (
            current_joint_positions + step
        )

        self.robot.apply_action(
            ArticulationAction(
                joint_positions=next_joint_positions,
            )
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
        self._transport_orientation = None

        self._joint1_before_tracking = None
        self._joint1_target = None
        self._safe_tracking_joint_positions = None

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
            elif self._state == M0609State.RETURN_HOME:
                self._on_enter_return_home()

        if self._state == M0609State.IDLE:
            self._step_idle()
        elif self._state == M0609State.PICK:
            self._step_pick()
        elif self._state == M0609State.TRACKING:
            self._step_tracking()
        elif self._state == M0609State.PLACE:
            self._step_place()
        elif self._state == M0609State.RETURN_HOME:
            self._step_return_home()

    def reset(self) -> None:
        # Play 재시작 시 initialize_robot()이 최초 자세를 다시 적용하므로
        # 상태 머신은 실제 대기 상태인 IDLE에서 시작한다.
        self._state = M0609State.IDLE
        self._state_entered = True

        self._pick_phase = PickPhase.PICKING
        self._place_phase = PlacePhase.MOVE_TRANSIT

        # 좌우 우회 경로 전용 상태도 Play 재시작 시 초기화한다.
        self._joint1_before_tracking = None
        self._joint1_target = None
        self._safe_tracking_joint_positions = None
        self._transport_orientation = None

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