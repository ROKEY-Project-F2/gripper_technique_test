# M0609 Dual Suction Gripper Workflow

Isaac Sim에서 Doosan M0609과 듀얼 흡착 그리퍼를 이용해 수술 도구 트레이를 집고, 손 위치를 추적한 뒤 원래 위치에 반환하는 프로젝트입니다.

현재 배치된 로봇은 **Robot B**입니다.

## 현재 구현 상태

Robot B는 다음 과정을 수행합니다.

```text
IDLE
  ↓ PICK 명령
PICK
  ↓ 트레이 흡착
선택 경유지
  ↓
TRACKING
  ↓ /right_hand_mode = HOME
선택 경유지
  ↓
PLACE
  ↓ 트레이 해제 및 수직 상승
RETURN_HOME
  ↓ 최초 관절 자세 복귀
IDLE
```

현재 Robot B는 트레이 명령 `4`, `5`, `6`, `7`을 담당합니다.

> 프로젝트 내부 트레이 ID가 0부터 시작하므로 화면상의 트레이 5~8에 대응합니다.

## 주요 기능

- Isaac Sim 5.1 기반 Doosan M0609 시뮬레이션
- 듀얼 Surface Gripper를 하나의 그리퍼처럼 제어
- 동적 트레이 및 수술 도구 생성
- ROS 2 PICK 명령 수신
- 오른손 위치 기반 Robot B TRACKING
- 오른손 HOME 명령 기반 트레이 반환
- 오른손 파란색 마커 표시
- 왼손 노란색 마커 표시
- 작업 종료 후 최초 관절 자세 복귀
- `RobotManager` 기반 작업 배정 구조
- 이후 Robot A 추가를 고려한 객체 및 변수명 분리

## 개발 환경

- Ubuntu
- Isaac Sim 5.1
- Isaac Sim 내장 Python 3.11
- ROS 2 Humble
- Fast DDS
- Doosan M0609
- RMPflow
- Isaac Sim ROS 2 Bridge

기본 ROS 설정:

```text
ROS_DOMAIN_ID=140
ROS_DISTRO=humble
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 핵심 파일

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

전체 객체를 생성하고 연결하는 실행 진입점입니다.

- Isaac Sim 및 World 초기화
- Robot B 초기화
- Robot B 최초 관절 자세 저장
- 트레이와 도구 생성
- Robot B 컨트롤러 생성
- 오른손 입력을 Robot B에 연결
- 좌우 손 마커 생성
- `RobotManager`와 상태 머신 실행

### `m0609_state_machine.py`

Robot B 작업 순서를 관리합니다.

```text
IDLE → PICK → TRACKING → PLACE → RETURN_HOME → IDLE
```

`IDLE`은 이동 상태가 아니라 최초 관절 자세에서 명령을 기다리는 상태입니다.

### `m0609_ros_bridge.py`

ROS 2 토픽과 내부 캐시 및 `RobotManager`를 연결합니다.

### `hand_marker_visualizer.py`

손 raw 좌표를 Isaac Sim의 구체로 표시합니다.

```text
오른손: 파란색
왼손:   노란색
```

## ROS 2 토픽

### 작업 명령

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/m0609/pick_command` | `std_msgs/msg/Int32` | Robot B 트레이 PICK 명령 |
| `/m0609/move_command` | `std_msgs/msg/String` | 직접 좌표 명령용 기존 인터페이스 |
| `/m0609/move_result` | `std_msgs/msg/String` | 직접 좌표 명령 수락 결과 |

현재 workflow 모드에서는 직접 좌표 이동보다 `/m0609/pick_command`를 사용합니다.

### 왼손

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/left_hand_raw` | `geometry_msgs/msg/Point` | 왼손 노란색 마커 위치 |
| `/left_hand_xyz` | `geometry_msgs/msg/Point` | 왼손 변환 좌표 |
| `/left_hand_mode` | `std_msgs/msg/String` | 왼손 상태 |
| `/left_palm_direction` | `geometry_msgs/msg/Vector3` | 왼손 손바닥 방향 |

### 오른손

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/right_hand_raw` | `geometry_msgs/msg/Point` | 오른손 파란색 마커 위치 |
| `/right_hand_xyz` | `geometry_msgs/msg/Point` | Robot B TRACKING 목표 |
| `/right_hand_mode` | `std_msgs/msg/String` | Robot B TRACKING/HOME 제어 |
| `/right_palm_direction` | `geometry_msgs/msg/Vector3` | 오른손 손바닥 방향 |

Robot B가 실제로 사용하는 입력은 다음 두 개입니다.

```text
/right_hand_xyz
/right_hand_mode
```

## 실행 방법

### 1. Isaac Sim 환경 변수 설정

프로젝트를 실행할 터미널에서 다음 환경 변수를 설정합니다.

```bash
export ROS_DOMAIN_ID=140
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Isaac Sim ROS 2 Bridge 라이브러리 경로를 추가합니다.

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib
```

Isaac Sim 설치 위치가 다르면 실제 설치 경로에 맞게 수정합니다.

### 2. 프로젝트 실행

프로젝트 폴더로 이동합니다.

```bash
cd ~/cobot3_ws/isaacpjt/gripper_technique_test
```

Isaac Sim 내장 Python으로 `main.py`를 실행합니다.

```bash
$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh main.py
```

Isaac Sim 창이 열리면 Play 버튼을 누릅니다.


## ROS 토픽 보내기


### ROS 2 CLI 명령

Python 송신 도구 없이 직접 보낼 때 사용할 수 있습니다.

#### Robot B PICK

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/Int32 \
  "{data: 7}"
```

#### 오른손 TRACKING 위치

```bash
ros2 topic pub -r 20 \
  /right_hand_xyz \
  geometry_msgs/msg/Point \
  "{x: 0.50, y: 0.10, z: 0.75}"
```

#### 오른손 raw 좌표

```bash
ros2 topic pub -r 20 \
  /right_hand_raw \
  geometry_msgs/msg/Point \
  "{x: 0.50, y: 0.10, z: 0.75}"
```

#### 오른손 HOME

```bash
ros2 topic pub --once \
  /right_hand_mode \
  std_msgs/msg/String \
  "{data: HOME}"
```

#### 오른손 손바닥 방향

```bash
ros2 topic pub -r 20 \
  /right_palm_direction \
  geometry_msgs/msg/Vector3 \
  "{x: 0.0, y: 0.0, z: -1.0}"
```

#### 왼손 xyz 좌표

```bash
ros2 topic pub -r 20 \
  /left_hand_xyz \
  geometry_msgs/msg/Point \
  "{x: 0.45, y: -0.20, z: 0.80}"
```

#### 왼손 raw 좌표

```bash
ros2 topic pub -r 20 \
  /left_hand_raw \
  geometry_msgs/msg/Point \
  "{x: 0.45, y: -0.20, z: 0.80}"
```

#### 왼손 mode

```bash
ros2 topic pub --once \
  /left_hand_mode \
  std_msgs/msg/String \
  "{data: TRACKING}"
```

#### 왼손 손바닥 방향

```bash
ros2 topic pub -r 20 \
  /left_palm_direction \
  geometry_msgs/msg/Vector3 \
  "{x: 0.0, y: 0.0, z: -1.0}"
```

## 기본 테스트 순서

터미널 1:

```bash
export ROS_DOMAIN_ID=140
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib

cd ~/cobot3_ws/isaacpjt/gripper_technique_test

$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh main.py
```

Isaac Sim에서 Play를 누릅니다.

터미널 2에서 ROS 2 환경을 설정합니다.

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=140
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd ~/cobot3_ws/isaacpjt/gripper_technique_test
```

그다음 오른손 raw/xyz 좌표를 발행합니다.

```bash
ros2 topic pub -r 20   /right_hand_raw   geometry_msgs/msg/Point   "{x: 0.50, y: 0.10, z: 0.75}"
```

```bash
ros2 topic pub -r 20   /right_hand_xyz   geometry_msgs/msg/Point   "{x: 0.50, y: 0.10, z: 0.75}"
```

필요하면 왼손 마커 좌표도 발행합니다.

```bash
ros2 topic pub -r 20   /left_hand_raw   geometry_msgs/msg/Point   "{x: 0.45, y: -0.20, z: 0.80}"
```

터미널 3을 새로 열었다면 동일하게 ROS 2 환경을 설정합니다.

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=140
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd ~/cobot3_ws/isaacpjt/gripper_technique_test
```

그다음 PICK 명령을 보냅니다.

```bash
ros2 topic pub --once   /m0609/pick_command   std_msgs/msg/Int32   "{data: 7}"
```

Robot B가 트레이를 집고 오른손을 추적하면 반환 명령을 보냅니다.

```bash
ros2 topic pub --once   /right_hand_mode   std_msgs/msg/String   "{data: HOME}"
```

## 현재 Robot B 동작

```text
1. IDLE에서 PICK 명령 대기
2. 트레이 5~8 중 지정된 트레이 접근
3. 듀얼 그리퍼로 흡착
4. 작업에 지정된 경유지 이동
5. /right_hand_xyz 추적
6. /right_hand_mode가 HOME이면 반환 시작
7. 같은 경유지를 통해 원래 트레이 위치로 이동
8. 도구 해제
9. 트레이 위로 수직 상승
10. 최초 관절 자세로 복귀
11. IDLE 전환
```

## 향후 작업

1. 경유지 1, 2, 3 분리
2. 경유지 2 공용 락 추가
3. 공동 작업구역 분리
4. Robot A 추가
5. Robot A와 Robot B 동시 작업 검증