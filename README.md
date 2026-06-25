# M0609 Dual-Robot Surgical Tray Workflow

Isaac Sim에서 두 대의 Doosan M0609과 듀얼 흡착 그리퍼를 사용해 수술 도구 트레이를 집고, 사용자의 손 위치를 추종한 뒤 원래 트레이 위치로 반환하는 프로젝트입니다.

현재 구현은 다음 기능을 포함합니다.

- Robot A / Robot B 동시 운용
- 트레이 번호 기반 작업 로봇 배정
- 공용 중앙 경유지 잠금
- 중앙 경로와 좌우 우회 경로 분리
- 듀얼 Surface Gripper 기반 트레이 흡착 및 해제
- 왼손/오른손 위치 기반 TRACKING
- `PLACE` 명령 기반 트레이 반환
- 운반 중 엔드이펙터 자세 유지
- 작업 완료 후 초기 관절 자세로 저속 복귀
- 작업 중 추가 PICK 명령을 `RobotManager`에서 차단

---

## 1. 시스템 구성

### Robot A

| 항목 | 설정 |
|---|---|
| Prim path | `/World/m0609_01` |
| 담당 트레이 | `0, 1, 2, 3` |
| TRACKING 입력 | 왼손 |
| 우선 경로 | `TRANSIT_2` |
| 대체 경로 | `TRANSIT_1` |
| 대체 경로 joint1 회전 | `+90°` |

### Robot B

| 항목 | 설정 |
|---|---|
| Prim path | `/World/m0609` |
| 담당 트레이 | `4, 5, 6, 7` |
| TRACKING 입력 | 오른손 |
| 우선 경로 | `TRANSIT_2` |
| 대체 경로 | `TRANSIT_3` |
| 대체 경로 joint1 회전 | `-90°` |

트레이 번호는 코드 내부에서 `0`부터 시작합니다.

```text
상단: 1    3    5    7
하단: 0    2    4    6
```

현재 트레이 좌표는 다음과 같습니다.

```python
TRAY_SPAWN_POSITIONS = {
    0: (-0.60, 0.55, 1.05),
    1: (-0.60, 0.85, 1.05),
    2: (-0.20, 0.55, 1.05),
    3: (-0.20, 0.85, 1.05),
    4: ( 0.20, 0.55, 1.05),
    5: ( 0.20, 0.85, 1.05),
    6: ( 0.60, 0.55, 1.05),
    7: ( 0.60, 0.85, 1.05),
}
```

---

## 2. 상태 구조

각 로봇은 다음 상태를 사용합니다.

```text
IDLE
  ↓ PICK 명령 수락
PICK
  ↓ 트레이 흡착 및 경유지 이동
TRACKING
  ↓ PLACE 명령
PLACE
  ↓ 트레이 반환 및 해제
RETURN_HOME
  ↓ 초기 관절 자세 복귀
IDLE
```

### `IDLE`

- 새로운 PICK 명령을 기다립니다.
- 현재 단계에서는 `IDLE` 상태의 로봇만 새 작업을 받을 수 있습니다.
- 대상 로봇이 `PICK`, `TRACKING`, `PLACE`, `RETURN_HOME` 중 하나라면 `RobotManager`에서 명령을 즉시 차단합니다.

### `PICK`

- 지정된 트레이 위로 이동합니다.
- 듀얼 흡착 그리퍼로 트레이를 잡습니다.
- 흡착이 완료된 순간의 실제 엔드이펙터 orientation을 저장합니다.
- 이후 경유지 이동에서는 저장한 orientation을 유지합니다.

### `TRACKING`

- Robot A는 왼손 좌표를 사용합니다.
- Robot B는 오른손 좌표를 사용합니다.
- `PLACE` 명령이 들어올 때까지 손 위치를 추종합니다.

### `PLACE`

- TRACKING 중 사용한 트레이를 원래 위치로 반환합니다.
- 중앙 경로와 좌우 우회 경로의 복귀 방법이 다릅니다.
- 트레이 상공에 도착하기 전까지 PICK 직후 저장한 운반 orientation을 유지합니다.
- 트레이 접근 단계에서만 PLACE용 orientation으로 전환합니다.

### `RETURN_HOME`

- 트레이를 해제하고 수직 상승한 뒤 초기 관절 자세로 복귀합니다.
- J1~J3과 J4~J6의 복귀 속도를 별도로 제한합니다.
- 초기 관절 자세에 도착하면 다시 `IDLE`로 전환됩니다.

---

## 3. 경유지 구조

세 개의 경유지를 사용합니다.

### `TRANSIT_2`

Robot A와 Robot B 사이의 중앙 경유지입니다.

```text
TRANSIT_2.xy =
(Robot A base.xy + Robot B base.xy) / 2
```

- 두 로봇이 모두 우선적으로 사용합니다.
- 공용 구역이므로 `RobotManager`가 락을 관리합니다.
- 한 로봇이 사용 중이면 다른 로봇은 대체 경로를 선택합니다.

### `TRANSIT_1`

Robot A의 대체 경로입니다.

```text
TRANSIT_1.xy =
2 × Robot A base.xy - TRANSIT_2.xy
```

### `TRANSIT_3`

Robot B의 대체 경로입니다.

```text
TRANSIT_3.xy =
2 × Robot B base.xy - TRANSIT_2.xy
```

모든 경유지의 기본 높이는 다음과 같습니다.

```python
TRANSIT_HEIGHT = 1.35
```

---

## 4. 중앙 경로와 좌우 경로

### 중앙 경로

`TRANSIT_2`를 사용하는 경우 joint1 회전을 사용하지 않습니다.

정방향:

```text
PICK
→ TRANSIT_2
→ TRACKING
```

반환:

```text
PLACE
→ TRANSIT_2
→ 트레이 상공
→ 트레이 위치
→ RELEASE
→ RETURN_HOME
```

### Robot A 좌측 우회 경로

정방향:

```text
PICK
→ TRANSIT_1
→ joint1 +90°
→ 회전 완료 관절 자세 저장
→ TRACKING
```

반환:

```text
PLACE
→ 회전 완료 당시 전체 관절 자세로 복귀
→ joint1 원복
→ TRANSIT_1
→ 트레이 상공
→ 트레이 위치
→ RELEASE
→ RETURN_HOME
```

### Robot B 우측 우회 경로

정방향:

```text
PICK
→ TRANSIT_3
→ joint1 -90°
→ 회전 완료 관절 자세 저장
→ TRACKING
```

반환:

```text
PLACE
→ 회전 완료 당시 전체 관절 자세로 복귀
→ joint1 원복
→ TRANSIT_3
→ 트레이 상공
→ 트레이 위치
→ RELEASE
→ RETURN_HOME
```

좌우 경로의 joint1 회전은 첫 번째 관절만 직접 제어합니다.

```python
ArticulationAction(
    joint_positions=np.array([next_target]),
    joint_indices=np.array([0]),
)
```

---

## 5. 엔드이펙터 자세 유지

PICK과 경유지 이동에서 orientation 목표가 갑자기 바뀌지 않도록, 흡착 완료 순간의 실제 엔드이펙터 자세를 저장합니다.

```text
트레이 접근
→ 흡착 완료
→ 실제 엔드이펙터 orientation 저장
→ 저장한 orientation으로 경유지 이동
→ TRACKING
```

반환할 때도 같은 orientation을 사용합니다.

```text
TRACKING
→ PLACE
→ 저장한 orientation으로 경유지 복귀
→ 트레이 상공에서 PLACE orientation으로 전환
→ 트레이 내려놓기
```

따라서 정방향과 반환 과정에서 같은 운반 자세를 사용합니다.

좌우 우회 경로에서 joint1만 ±90° 회전할 때는 엔드이펙터의 월드 기준 yaw가 함께 변할 수 있습니다. 이 단계의 목적은 트레이의 수평 상태를 유지하면서 로봇 베이스 방향을 전환하는 것입니다.

---

## 6. 작업 명령 차단

현재 `RobotManager`는 트레이 번호로 담당 로봇을 먼저 결정합니다.

```text
PICK 명령 수신
→ tray_id 담당 로봇 결정
→ 대상 로봇 상태 확인
→ IDLE이면 작업 배정
→ IDLE이 아니면 즉시 차단
```

대상 로봇이 다음 상태라면 명령을 상태 머신에 전달하지 않습니다.

```text
PICK
TRACKING
PLACE
RETURN_HOME
```

차단 시 다음 작업은 수행되지 않습니다.

- 경로 선택
- 공용 경로 락 획득
- `RobotTask` 생성
- 상태 머신 `assign_task()` 호출

예시 로그:

```text
[Manager BLOCK] command rejected:
tray=4, Robot B is busy: state=TRACKING
```

현재는 중복 명령 래치나 명령 큐를 사용하지 않습니다. 추가 명령 처리 정책은 추후 별도 기능으로 구현할 예정입니다.

---

## 7. 속도 관련 설정

### 좌우 경로 joint1 회전

```python
JOINT1_TURN_MAX_STEP_DEG = 0.50
JOINT1_TURN_TOLERANCE_DEG = 1.0
```

- simulation step당 최대 `0.50°` 이동합니다.
- 목표 각도와 `1.0°` 이내가 되면 완료로 판정합니다.

### 좌우 경로 안전 관절 자세 복귀

```python
SAFE_JOINT_RETURN_MAX_STEP_DEG = 0.35
SAFE_JOINT_RETURN_TOLERANCE_DEG = 1.0
```

- TRACKING 위치에서 회전 완료 당시의 전체 관절 자세로 복귀할 때 사용합니다.

### 초기 관절 자세 복귀

```python
RETURN_HOME_MAX_STEP_DEG = 0.20
RETURN_HOME_WRIST_MAX_STEP_DEG = 0.60
RETURN_HOME_TOLERANCE_DEG = 1.0
```

- J1~J3은 `0.20°/step`으로 복귀합니다.
- 엔드이펙터 방향을 만드는 J4~J6은 `0.60°/step`으로 복귀합니다.
- 위치 복귀 속도는 유지하면서 손목 방향만 더 빠르게 복원합니다.

---

## 8. ROS 2 토픽

### PICK 명령

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/m0609/pick_command` | `std_msgs/msg/Int32` | 트레이 번호 기반 PICK 명령 |

트레이 번호에 따라 Robot A 또는 Robot B가 자동 선택됩니다.

### 왼손 입력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/left_hand_raw` | `geometry_msgs/msg/Point` | 왼손 마커 위치 |
| `/left_hand_xyz` | `geometry_msgs/msg/Point` | Robot A TRACKING 목표 |
| `/left_hand_mode` | `std_msgs/msg/String` | Robot A 모드 명령 |
| `/left_palm_direction` | `geometry_msgs/msg/Vector3` | 왼손 손바닥 방향 |

### 오른손 입력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/right_hand_raw` | `geometry_msgs/msg/Point` | 오른손 마커 위치 |
| `/right_hand_xyz` | `geometry_msgs/msg/Point` | Robot B TRACKING 목표 |
| `/right_hand_mode` | `std_msgs/msg/String` | Robot B 모드 명령 |
| `/right_palm_direction` | `geometry_msgs/msg/Vector3` | 오른손 손바닥 방향 |

마커 색상:

```text
왼손: 노란색
오른손: 파란색
```

---

## 9. 실행 환경

- Ubuntu
- Isaac Sim 5.1
- Isaac Sim 내장 Python 3.11
- ROS 2 Humble
- Fast DDS
- Doosan M0609
- RMPflow
- cuRobo
- Isaac Sim ROS 2 Bridge

기본 ROS 설정:

```bash
export ROS_DOMAIN_ID=140
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Isaac Sim ROS 2 Bridge 라이브러리 경로:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib
```

Isaac Sim 설치 경로가 다르면 실제 경로에 맞게 변경해야 합니다.

다른 ROS 2 터미널에서는 다음 설정을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=140
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 10. 실행 방법

프로젝트 폴더로 이동합니다.

```bash
cd ~/cobot3_ws/isaacpjt/gripper_technique_test
```

Isaac Sim 내장 Python으로 실행합니다.

```bash
$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh main.py
```

Isaac Sim 창이 열린 뒤 Play 버튼을 누릅니다.

---

## 11. 테스트 명령

### Robot A: 트레이 0 PICK

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/Int32 \
  "{data: 0}"
```

### Robot B: 트레이 4 PICK

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/Int32 \
  "{data: 4}"
```

### Robot A 왼손 TRACKING 좌표

```bash
ros2 topic pub -r 20 \
  /left_hand_xyz \
  geometry_msgs/msg/Point \
  "{x: -0.45, y: 0.10, z: 1.30}"
```

### Robot B 오른손 TRACKING 좌표

```bash
ros2 topic pub -r 20 \
  /right_hand_xyz \
  geometry_msgs/msg/Point \
  "{x: 0.45, y: 0.10, z: 1.30}"
```

### Robot A PLACE

```bash
ros2 topic pub --once \
  /left_hand_mode \
  std_msgs/msg/String \
  "{data: PLACE}"
```

### Robot B PLACE

```bash
ros2 topic pub --once \
  /right_hand_mode \
  std_msgs/msg/String \
  "{data: PLACE}"
```

### 왼손 마커 테스트

```bash
ros2 topic pub -r 20 \
  /left_hand_raw \
  geometry_msgs/msg/Point \
  "{x: -0.45, y: 0.10, z: 1.30}"
```

### 오른손 마커 테스트

```bash
ros2 topic pub -r 20 \
  /right_hand_raw \
  geometry_msgs/msg/Point \
  "{x: 0.45, y: 0.10, z: 1.30}"
```

---

## 12. 주요 파일

```text
gripper_technique_test/
├── main.py
├── m0609_config.py
├── m0609_task.py
├── m0609_state_machine.py
├── m0609_move_controller.py
├── m0609_tracking_controller.py
├── m0609_pick_place_controller_surface.py
├── m0609_dynamic_scene.py
├── m0609_ros_bridge.py
├── hand_input.py
├── hand_marker_visualizer.py
├── robot_manager.py
├── robot_runtime.py
├── dual_surface_gripper_adapter.py
├── surface_gripper_adapter.py
└── README.md
```

### `main.py`

- 전체 USD Scene 로드
- Robot A / Robot B 초기화
- 로봇 베이스 pose 읽기
- 경유지 좌표 계산
- 트레이 및 수술 도구 생성
- PICK / MOVE / TRACKING 컨트롤러 생성
- 상태 머신 생성
- `RobotManager` 생성 및 로봇 등록
- ROS 2 Bridge 연결
- 손 마커 갱신
- simulation step 실행

### `m0609_config.py`

- 로봇 Prim 경로
- 트레이 좌표
- 로봇별 담당 트레이
- 경유지 높이
- joint1 회전량 및 속도
- TRACKING 범위
- RETURN_HOME 속도
- PICK / PLACE 관련 파라미터

### `m0609_state_machine.py`

다음 상태와 세부 동작을 관리합니다.

```text
IDLE
PICK
TRACKING
PLACE
RETURN_HOME
```

중앙 경로와 좌우 경로의 PLACE 복귀 과정을 별도 함수로 분리합니다.

### `robot_manager.py`

- 트레이 번호 기반 대상 로봇 선택
- 대상 로봇 `IDLE` 상태 확인
- 공용 `TRANSIT_2` 락 관리
- 우선 경로 및 대체 경로 선택
- `RobotTask` 생성 및 상태 머신 전달
- 작업 완료 후 경로 락 해제

### `robot_runtime.py`

`RobotProfile`과 `RobotTask` 데이터 구조를 정의합니다.

### `m0609_ros_bridge.py`

ROS 2 토픽을 구독하고 최신 손 좌표, 손 모드 및 PICK 명령을 내부 객체에 전달합니다.

### `hand_marker_visualizer.py`

왼손과 오른손 raw 좌표를 Isaac Sim Scene에 마커로 표시합니다.

---

## 13. 주요 로그

작업 배정:

```text
[Manager] Robot B assigned tray 4 via route TRANSIT_2
```

공용 경로 락:

```text
[Manager LOCK] acquired: route=TRANSIT_2, robot=B
[Manager LOCK] released: route=TRANSIT_2, robot=B
```

작업 중 명령 차단:

```text
[Manager BLOCK] command rejected:
tray=4, Robot B is busy: state=TRACKING
```

운반 orientation 저장:

```text
[TRANSPORT B] 흡착 완료 자세 저장: [...]
```

경유지 이동:

```text
[TRANSIT B] route=TRANSIT_3, position=[...], orientation=[...]
```

좌우 경로 joint1 회전:

```text
[JOINT1 B] target=...
[SAFE B] joint1 회전 완료 관절 자세 저장: [...]
```

초기 자세 복귀:

```text
[RETURN_HOME B] initial joint pose reached
```

---

## 14. 현재 제한 사항

- TRACKING 기능은 손 좌표 입력을 기반으로 동작하며 실제 카메라 입력 품질에 영향을 받습니다.
- 좌우 우회 경로에서는 joint1만 회전하므로 엔드이펙터의 월드 기준 yaw가 함께 변경됩니다.
- 공용 락은 현재 `TRANSIT_2`에만 적용됩니다.
- 작업 중 입력되는 추가 PICK 명령은 큐에 저장하지 않고 즉시 거부합니다.
- 중복 명령 래치, 작업 취소, 우선순위, 타임아웃 복구는 추후 기능으로 추가할 예정입니다.
- RMPflow 및 cuRobo 파라미터는 현재 Scene과 로봇 배치에 맞게 조정되어 있습니다.