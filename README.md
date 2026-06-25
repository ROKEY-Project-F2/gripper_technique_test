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

---

# 시나리오 개편 및 향후 구현 계획

# Surgical Robot Agent 시나리오 개편 작업 계획

## 1. 문서 목적

이 문서는 다음 대화에서 Isaac Sim 내부 코드를 단계적으로 수정하기 위한 기준을 정리한다.

핵심 목표:

- 트레이 수를 8개에서 6개로 변경
- 두 로봇이 모든 트레이에 접근 가능하도록 변경
- 모든 트레이 PICK/PLACE 구역을 하나의 공용 구역으로 관리
- 손의 OPEN/CLOSED 상태를 이용한 도구 집기·놓기 처리
- 매니저 중심의 도구·트레이·손 상태 추적
- 도구 ID 기반 요청
- 자동 손 선택과 로봇 스케줄링
- 정상 교체 시나리오
- 특정 도구 작업 취소
- 최근 활성 작업 취소
- 잘못된 요청 무시
- 취소 복구 시나리오

---

# 2. 최종 책임 구조

## 2.1 외부 요청

```text
/tool/request
std_msgs/msg/Int32
data = tool_id
```

```text
/tool/cancel
std_msgs/msg/Int32
data = 취소할 tool_id
```

```text
/tool/cancel_last
std_msgs/msg/Empty
```

외부는 다음을 지정하지 않는다.

```text
tray_id
Robot A / Robot B
왼손 / 오른손
```

---

## 2.2 매니저 책임

매니저는 도구 기준 작업을 관리하는 단일 기준이다.

관리 대상:

```text
도구 위치
트레이 위치
트레이에 포함된 도구
왼손 보유 도구
오른손 보유 도구
각 로봇이 운반 중인 트레이
활성 작업
도구별 활성 작업 인덱스
최근 활성 작업 스택
트레이 예약
공용 구역 락
취소 요청 상태
```

판단 대상:

```text
요청 수락 또는 거부
잘못된 요청 무시
요청 도구 위치 조회
대상 손 선택
담당 로봇 선택
정상 전달
도구 교체
특정 도구 작업 취소
최근 활성 작업 취소
취소 복구 순서
```

---

## 2.3 상태 머신 책임

상태 머신은 로봇의 물리 동작만 수행한다.

큰 상태는 최대한 유지한다.

```text
IDLE
PICK
TRACKING
PLACE
RETURN_HOME
```

상태 머신이 수행하는 것:

```text
트레이 접근
흡착 및 해제
안전 상승
경유지 이동
손 위치 추종
트레이 반납
기본 자세 복귀
손 OPEN/CLOSED 변화 감지
물리 동작 완료 이벤트 반환
```

상태 머신이 하지 않는 것:

```text
취소 대상 선택
취소 가능 여부 판단
최근 작업 조회
도구 위치의 최종 관리
손 또는 로봇 선택
교체·취소 작업 순서 결정
```

취소는 매니저가 판단하고 상태 머신에는 기존 물리 명령만 전달한다.

---

# 3. 초기 상태

트레이와 도구는 각각 6개다.

```text
Tool 0 → Tray 0
Tool 1 → Tray 1
Tool 2 → Tray 2
Tool 3 → Tray 3
Tool 4 → Tray 4
Tool 5 → Tray 5
```

초기에는 외부 트레이 영상인식을 사용하지 않는다.

매니저가 최초 배치와 확정된 작업 결과를 기준으로 위치를 추적한다.

---

# 4. 권장 구현 순서

## 1단계: 트레이 6개 및 공용 구역

수정 목표:

```text
트레이 8개 → 6개
트레이별 고정 로봇 담당 제거
두 로봇 모두 Tray 0~5 접근 가능
모든 PICK/PLACE 공간을 하나의 공용 구역으로 처리
```

공용 구역 사용 범위:

```text
락 획득
→ 공용 구역 진입
→ PICK 또는 PLACE
→ 안전 이탈 위치 도착
→ 락 해제
```

검증:

```text
Robot A가 Tray 0~5 접근 가능
Robot B가 Tray 0~5 접근 가능
동시에 한 로봇만 공용 구역 진입
대기 로봇이 안전 위치 유지
안전 이탈 전에 락이 해제되지 않음
```

---

## 2단계: 손 집기·놓기 사건 감지

집기:

```text
OPEN → CLOSED
AND 손이 비어 있음
AND 로봇이 도구가 있는 트레이를 해당 손에 제공 중

→ TOOL_TAKEN
```

놓기:

```text
CLOSED → OPEN
AND 손이 도구를 들고 있음
AND 로봇이 빈 트레이를 해당 손에 제공 중

→ TOOL_PLACED
```

상태 머신은 전체 위치표를 수정하지 않고 사건만 매니저에 전달한다.

예시 내부 이벤트:

```text
TOOL_TAKEN
- robot_id
- tray_id
- hand_side
- tool_id
```

```text
TOOL_PLACED
- robot_id
- tray_id
- hand_side
- tool_id
```

검증:

```text
반복 프레임으로 이벤트가 중복 발생하지 않음
트레이를 제공하지 않을 때 손 동작은 무시
왼손과 오른손이 독립적으로 처리됨
```

---

## 3단계: 매니저 상태 추적

매니저 최소 데이터:

```text
Tray 0~5의 포함 도구
Tray 0~5의 현재 위치
왼손 보유 도구
오른손 보유 도구
Robot A/B 보유 트레이
각 Tool의 현재 위치
```

상태는 명령 전송 시점이 아니라 확정 사건 또는 완료 시점에 갱신한다.

```text
PICK 성공 → 트레이 위치를 로봇으로 변경
PLACE 성공 → 트레이 위치를 슬롯으로 변경
TOOL_TAKEN → 도구를 손으로 이동
TOOL_PLACED → 도구를 트레이로 이동
```

검증:

```text
도구 하나는 한 위치에만 존재
한 트레이에는 도구 최대 하나
한 손에는 도구 최대 하나
Tool 위치와 Tray 내용이 일치
```

---

## 4단계: 도구 요청과 자동 스케줄링

```text
/tool/request 수신
→ tool_id 유효성 검사
→ 도구 현재 위치 확인
→ 대상 손 선택
→ 담당 로봇 선택
→ 트레이 예약
→ 공용 구역 락
→ PICK
→ 안전 이탈
→ TRACKING
```

손 선택:

```text
1. 요청 도구를 이미 사용 중이면 중복 요청
2. 한쪽 손만 비어 있으면 빈손 선택
3. 양손이 비어 있으면 가까운 손 선택
4. 양손이 사용 중이면 교체 비용이 작은 손 선택
```

로봇 선택:

```text
1. IDLE 로봇
2. 선택된 손과 가까운 로봇
3. 요청 트레이까지 이동 비용이 작은 로봇
4. 동률이면 고정 우선순위
```

---

## 5단계: 정상 도구 교체

```text
1. 기존 빈 트레이 원복
2. 안전 이탈
3. 새 도구 트레이 PICK
4. 사용자에게 전달
5. 새 도구 집기
6. 기존 도구 놓기
7. 현재 트레이 원복
8. 기존 빈 트레이 재획득
9. TRACKING 복귀
```

교체 후 도구가 다른 트레이로 이동할 수 있으므로 이후 요청은 동적 위치표를 사용한다.

---

## 6단계: 작업 레지스트리와 최근 활성 작업 스택

취소 기능을 넣기 전에 매니저에 작업 관리 구조를 추가한다.

권장 구조:

```python
active_operations: dict[str, Operation]
operation_by_tool: dict[int, str]
recent_operation_stack: list[str]
```

의미:

```text
active_operations
→ 현재 진행 중인 전체 작업

operation_by_tool
→ 특정 tool_id의 활성 작업 조회

recent_operation_stack
→ 최근에 수락된 활성 작업 조회
```

스택에는 Operation 전체가 아니라 `operation_id`만 저장한다.

작업 등록:

```text
요청 검증 성공
→ 작업 생성
→ active_operations 등록
→ operation_by_tool 등록
→ recent_operation_stack push
```

작업 제거:

```text
정상 완료
취소 복구 완료
실행 실패
요청 폐기
세션 종료

→ 모든 인덱스에서 제거
```

취소 요청을 받은 즉시 제거하지 않는다.

```text
취소 요청
→ cancel_requested = True
→ 작업은 계속 활성
→ 취소 복구 완료 후 제거
```

최근 작업 조회 시 오래된 ID가 남아 있으면 스택 끝에서 자동 제거한다.

### 메모리 및 부하

이 구조는 이벤트가 발생했을 때만 갱신한다.

```text
요청 수락 시 추가
취소 요청 시 조회
완료·실패 시 제거
```

매 시뮬레이션 프레임마다 전체 작업을 순회하지 않는다.

동시 작업 수가 적으므로 메모리와 CPU 부담은 무시할 수준이다.

방어적 제한:

```text
MAX_ACTIVE_OPERATIONS = 6 또는 8
```

한도 초과 시 오래된 활성 작업을 자동 삭제하지 않고 새 요청을 거부한다.

`deque(maxlen=...)`를 활성 작업 스택에 사용해 조용히 밀어내지 않는다.

---

## 7단계: 특정 작업 취소

토픽:

```text
/tool/cancel
std_msgs/msg/Int32
data = tool_id
```

처리:

```text
operation_by_tool에서 tool_id 조회
→ 활성 작업 확인
→ 취소 가능 여부 검사
→ 공통 취소 처리로 전달
```

말이 안 되는 요청:

```text
존재하지 않는 tool_id
→ 무시

해당 도구의 활성 작업 없음
→ 무시

이미 취소 중
→ 무시
```

---

## 8단계: 최근 활성 작업 취소

토픽:

```text
/tool/cancel_last
std_msgs/msg/Empty
```

처리:

```text
recent_operation_stack의 마지막 활성 작업 조회
→ 취소 가능 여부 검사
→ 공통 취소 처리로 전달
```

예:

```text
Tool 1 작업 수락
stack = [op1]

Tool 2 작업 수락
stack = [op1, op2]

Tool 2 완료
stack = [op1]

cancel_last
→ op1 취소
```

최근 작업 스택은 완료 이력을 저장하는 Undo 스택이 아니다.

```text
현재 활성 작업 중
가장 최근 작업을 찾기 위한 LIFO 인덱스
```

작업 완료 시 제거하므로 크기가 계속 증가하지 않는다.

---

## 9단계: 취소 가능 여부와 잘못된 요청 무시

취소 가능 여부 판단은 상태 머신이 아니라 매니저가 담당한다.

취소 기준은 로봇 상태 이름이 아니라 매니저의 활성 Operation 존재 여부와 작업 단계다.

권장 작업 단계:

```text
QUEUED
FETCHING
DELIVERING
EXCHANGING
RETURNING
CANCELLING
FINISHED
FAILED
```

취소 가능:

```text
QUEUED
FETCHING
DELIVERING
EXCHANGING
RETURNING
```

취소 불가:

```text
CANCELLING
FINISHED
FAILED
```

평상시 IDLE 또는 TRACKING은 활성 취소 대상 작업이 없는 상태로 처리한다.

```text
IDLE + 활성 Operation 없음
→ 취소 무시

평상시 TRACKING + 활성 Operation 없음
→ 취소 무시
```

말이 안 되는 요청 예:

```text
잘못된 tool_id 요청
이미 사용 중인 도구 재요청
UNKNOWN 위치 도구 요청
IDLE에서 cancel_last
TRACKING에서 cancel_last
활성 작업 없는 tool_id 취소
이미 취소 중인 작업 재취소
```

처리 원칙:

```text
로봇 상태 변경 없음
물리 명령 전송 없음
필요하면 로그 또는 결과 토픽만 발행
```

---

## 10단계: 취소 복구

취소는 즉시 물리 동작을 끊지 않는다.

```text
cancel_requested = True
```

로 기록하고 안전 체크포인트에서 복구 계획을 실행한다.

안전 체크포인트:

```text
흡착 후 안전 상승 완료
공용 구역 이탈 완료
사용자 전달 위치 도착
PLACE 후 안전 이탈 완료
```

경우별 처리:

```text
PICK 전
→ 예약 제거

운반 중
→ 트레이 원복

교체 전
→ 새 트레이 원복
→ 기존 빈 트레이 재획득
→ 이전 TRACKING 복귀

교체 후
→ 역교환
→ 새 트레이 원복
→ 기존 빈 트레이 재획득
→ 이전 TRACKING 복귀
```

---

# 5. 전체 구현 순서 요약

```text
1. 트레이 6개 및 공용 구역
2. 손 집기·놓기 사건 감지
3. 매니저 도구·트레이·손 상태 추적
4. 도구 요청 및 자동 스케줄링
5. 정상 도구 교체
6. 작업 레지스트리와 최근 활성 작업 스택
7. 특정 도구 작업 취소
8. 최근 활성 작업 취소
9. 잘못된 요청 무시 및 취소 가능성 검사
10. 취소 복구
11. 향후 트레이 영상인식 연결
```

---

# 6. 반드시 유지할 상태 불변 조건

```text
도구 하나는 한 위치에만 존재
트레이 하나는 한 위치에만 존재
한 손은 도구 최대 하나
한 로봇은 트레이 최대 하나
한 트레이에는 도구 최대 하나
하나의 tool_id에는 활성 작업 최대 하나
트레이 예약 소유자는 최대 하나
공용 구역 락 소유자는 최대 한 로봇
Tool 위치와 Tray 내용은 서로 일치
손이 보유한 Tool은 Tray에 동시에 존재하지 않음
```

---

# 7. 수정 예상 파일

## `m0609_config.py`

```text
트레이 6개
트레이 좌표
초기 Tool ↔ Tray 배치
공용 구역 안전 위치
트레이별 고정 로봇 소유 규칙 제거
```

## `robot_manager.py`

```text
도구·트레이·손 상태
공용 구역 락
활성 작업 레지스트리
도구별 작업 인덱스
최근 활성 작업 스택
잘못된 요청 검증
손·로봇 선택
정상 교체 계획
특정 작업 취소
최근 작업 취소
취소 복구 계획
```

## `m0609_state_machine.py`

```text
기존 물리 동작 유지
손 상태 변화 사건
PICK/PLACE 완료 이벤트
매니저 명령 기반 다음 동작 수행
```

## `robot_runtime.py`

```text
매니저 명령 전달
상태 머신 결과 반환
```

## `m0609_ros_bridge.py`

```text
/tool/request 구독
/tool/cancel 구독
/tool/cancel_last 구독
기존 손 토픽 유지
```

## `main.py`

```text
매니저 초기화
Tool/Tray 초기 상태 생성
ROS 콜백 연결
```

---

# 8. 변경하지 않을 항목

```text
로봇 USD 경로
관절 구성
그리퍼 prim 경로
흡착 그리퍼 구현
검증된 PICK/PLACE 높이
기존 안전 경유점
기본 자세
정상 동작 중인 손 좌표 변환
ROS_DOMAIN_ID 및 브리지 설정
```

---

# 9. 다음 대화 시작 요청문

```text
이 README 기준으로 1단계부터 진행하자.

현재 프로젝트 파일을 확인한 다음,
트레이를 6개로 줄이고 두 로봇이 모든 트레이에 접근할 수 있게 수정해줘.
모든 PICK/PLACE 구역에는 매니저가 관리하는 단일 공용 구역 락을 적용해줘.
기존 PICK, TRACKING, PLACE, RETURN_HOME 물리 동작은 최대한 유지해줘.
이번에는 1단계만 구현하고 테스트 방법까지 정리해줘.
```

---

# 10. 단계별 완료 보고 형식

```text
수정 파일
각 파일의 변경 내용
유지한 기존 기능
추가한 데이터와 상태
테스트 방법
정상 확인 항목
아직 구현하지 않은 다음 단계
알려진 제한사항
```