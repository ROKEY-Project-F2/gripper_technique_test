# M0609 수술 도구 전달 로봇 제어 프로젝트

## 1. 문서 목적

이 문서는 현재 구현된 **단일 로봇 실행 상태**와, 이후 **2대 로봇 구조로 확장하기 위해 적용한 코드 구조**를 정리한다.

현재 Isaac Sim에서는 로봇 A 한 대만 실행한다.  
다만 상태 머신과 매니저는 로봇별 객체, 컨트롤러, 손 입력, 작업 범위, 경유 위치를 외부에서 주입받도록 변경해 두었다.

따라서 두 번째 로봇을 추가할 때 기존 상태 머신의 내부 동작을 복사하거나 수정하지 않고, 로봇 B에 필요한 객체를 생성한 뒤 매니저에 등록하는 방식으로 확장할 수 있다.

---

## 2. 현재 구현 상태

### 구현 완료

- M0609 로봇 한 대 실행
- 실제 트레이 8개와 수술 도구 8개 동적 생성
- ROS 2 도구 선택 명령 수신
- 동적 트레이 위치를 기준으로 PICK
- 지정 경유 위치로 이동
- 손 좌표 추적
- `/hand_mode`의 `HOME` 명령 수신
- 원래 트레이 생성 위치로 PLACE
- 작업 완료 후 IDLE 복귀
- 외부 명령을 상태 머신이 직접 받지 않고 `RobotManager`를 통해 배정
- 상태 머신 내부의 손 입력, 컨트롤러, 로봇 객체, 대기 위치, 보정값을 생성 시 주입
- `RobotTask`를 통한 트레이와 경유 위치 전달
- `RobotProfile`을 통한 로봇별 작업 가능 트레이와 선호 경로 등록
- 여러 로봇을 등록할 수 있는 `RobotManager.register_robot()` 구조 적용

### 현재 미구현

- 실제 두 번째 로봇 B 생성 및 실행
- 왼손·오른손 ROS 토픽 분리
- 공동 작업구역 락
- 공용 경로 B 락
- B 경로 점유 시 A 또는 C 경로 자동 선택
- PLACE 재진입 시 공동 작업구역 권한 확인
- 로봇 간 충돌 회피 및 정교한 동시 작업 제어
- 명령 대기열
- 오류 복구 및 비상 정지 처리

---

## 3. 현재 실행 구조

```text
외부 ROS 2 명령
        │
        ▼
m0609_ros_bridge.py
        │
        ▼
RobotManager
        │
        ├─ 로봇 선택
        ├─ 접근 가능 트레이 확인
        ├─ RobotTask 생성
        └─ 상태 머신에 작업 배정
        │
        ▼
M0609StateMachine
        │
        ├─ PICK
        ├─ 경유 위치 이동
        ├─ TRACKING
        ├─ HOME 확인
        ├─ PLACE
        └─ IDLE 복귀
```

손 데이터는 현재 다음 구조로 처리한다.

```text
/hand_raw
/hand_xyz
/hand_mode
    │
    ▼
m0609_ros_bridge.py의 손 데이터 캐시
    │
    ▼
CachedHandInput
    │
    ▼
M0609StateMachine
```

상태 머신은 ROS 토픽 이름이나 ROS Bridge 전역 함수를 직접 알지 않는다.  
생성 시 주입받은 `HandInput` 인터페이스만 사용한다.

---

## 4. 현재 ROS 2 토픽

| 용도 | 토픽 | 메시지 |
|---|---|---|
| 도구 선택 명령 | `/m0609/pick_command` | `std_msgs/msg/Int32` |
| 직접 이동 명령 | `/m0609/move_command` | 기존 메시지 형식 |
| 이동 결과 | `/m0609/move_result` | 기존 메시지 형식 |
| 손 원본 좌표 | `/hand_raw` | 현재 손 개발 노드 형식 |
| 변환된 손 목표 좌표 | `/hand_xyz` | 현재 손 개발 노드 형식 |
| 손 모드 | `/hand_mode` | `TRACKING`, `HOME` |

현재 손 개발이 한 손 기준이므로 토픽 이름은 변경하지 않았다.

향후 두 손 입력이 준비되면 다음처럼 별도 `HandInput`을 만들 수 있다.

```text
Robot A → 왼손 HandInput
Robot B → 오른손 HandInput
```

상태 머신 코드는 수정하지 않고 생성부에서 입력 객체만 다르게 전달한다.

---

## 5. 현재 지원 도구 명령

현재 `main.py`와 설정 기준 지원 명령은 다음과 같다.

```text
4, 5, 6, 7
```

명령 예시:

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/Int32 \
  "{data: 7}"
```

TRACKING 종료 및 PLACE 시작:

```bash
ros2 topic pub --once \
  /hand_mode \
  std_msgs/msg/String \
  "{data: HOME}"
```

현재 실제 손 개발 노드가 `/hand_mode`를 발행한다면 별도 수동 명령은 필요하지 않다.

---

## 6. 상태 머신 동작 과정

현재 상태는 다음 네 가지다.

```text
IDLE
PICK
TRACKING
PLACE
```

### IDLE

- 로봇을 생성 시 전달받은 대기 위치로 이동한다.
- 매니저가 `RobotTask`를 배정할 때까지 대기한다.

### PICK

- `tray_registry`에서 선택된 트레이의 현재 위치와 방향을 조회한다.
- 트레이가 물리적으로 이동했더라도 현재 pose를 기준으로 PICK한다.
- 흡착 완료 후 `RobotTask.transit_position`으로 이동한다.

### TRACKING

- 생성 시 주입받은 `HandInput`에서 손 목표 좌표를 읽는다.
- 추적 컨트롤러에 좌표를 전달한다.
- 손 모드가 `HOME`으로 변경되면 PLACE로 전환한다.
- TRACKING 진입 시 이전 작업의 `HOME` 값이 남지 않도록 손 모드를 `TRACKING`으로 초기화한다.

### PLACE

- 작업 시작 당시 저장한 트레이의 생성 기준 위치로 이동한다.
- 도구를 내려놓는다.
- 경유 위치로 다시 이동한다.
- 작업 데이터를 초기화하고 IDLE로 복귀한다.

---

## 7. 주요 파일 역할

### `main.py`

- Isaac Sim 실행
- 전체 USD Stage 열기
- 트레이와 도구 생성
- 로봇 초기화
- 로봇별 컨트롤러 생성
- `HandInput` 생성
- 상태 머신 생성
- 매니저 생성 및 로봇 등록
- 메인 시뮬레이션 반복 실행

### `m0609_state_machine.py`

- 로봇 한 대의 PICK, TRACKING, PLACE 실행
- ROS 토픽을 직접 구독하지 않음
- 다른 로봇의 상태나 자원 점유 여부를 판단하지 않음
- 생성 시 주입받은 객체와 `RobotTask`만 사용

### `robot_manager.py`

- 등록된 로봇 목록 관리
- 명령한 트레이에 접근 가능한 IDLE 로봇 선택
- `RobotTask` 생성
- 상태 머신에 작업 배정
- 로봇별 상태 변경 및 작업 완료 감시

### `robot_runtime.py`

`RobotTask`와 `RobotProfile` 데이터 구조를 정의한다.

#### `RobotTask`

```python
RobotTask(
    tray_id,
    route_id,
    transit_position,
    transit_orientation,
    uses_shared_zone,
)
```

#### `RobotProfile`

```python
RobotProfile(
    robot_id,
    reachable_trays,
    preferred_route,
    fallback_route,
)
```

### `hand_input.py`

상태 머신이 사용하는 손 입력 인터페이스를 정의한다.

```python
get_target()
get_mode()
reset_mode()
```

현재는 `CachedHandInput`이 기존 ROS Bridge 손 캐시 함수를 감싼다.

### `m0609_ros_bridge.py`

- ROS 2 노드 및 구독자 생성
- 외부 PICK 명령을 `RobotManager`로 전달
- 현재 손 데이터를 캐시에 저장
- 상태 머신을 직접 호출하지 않음

---

## 8. 하드코딩 제거 상태

다음 항목은 상태 머신 외부에서 생성 시 전달된다.

- 로봇 ID
- 로봇 articulation 객체
- 그리퍼가 포함된 로봇 객체
- 트레이 registry
- 손 입력 객체
- IDLE 위치 및 방향
- Tracking 컨트롤러
- Pick/Place 컨트롤러
- 일반 이동 컨트롤러
- PICK EE offset
- PICK 접근 높이 보정
- PLACE 높이 및 접근 보정

경유 위치와 방향은 상태 머신 생성자가 아니라 작업마다 매니저가 만든 `RobotTask`로 전달한다.

따라서 상태 머신 내부에는 특정 로봇 이름, 특정 손 토픽, 특정 경로 B 좌표를 직접 참조하는 코드가 없어야 한다.

---

## 9. 현재 매니저 설정

현재는 로봇 A 한 대만 등록한다.

```python
robot_manager = RobotManager(
    routes={
        "B": (
            STAGING_POSITION,
            TRACKING_TOOL_ORIENTATION,
        ),
    },
    shared_trays=(),
)
```

```python
robot_manager.register_robot(
    profile=RobotProfile.create(
        robot_id="A",
        reachable_trays=SUPPORTED_TRAY_COMMANDS,
        preferred_route="B",
        fallback_route="A",
    ),
    state_machine=state_machine,
)
```

현재 조건:

- 등록된 로봇: A
- 등록된 경로: B
- 공동 작업 트레이: 없음
- 경로 락: 없음
- 작업 선택: 접근 가능하며 IDLE인 첫 번째 로봇
- 여러 후보가 있으면 `robot_id` 순서로 선택

`fallback_route` 필드는 준비되어 있지만 아직 실제 경로 선택에는 사용하지 않는다.

---

## 10. 두 번째 로봇 추가 방법

두 번째 로봇을 추가할 때 상태 머신 클래스를 수정하지 않는다.

### 1. 로봇 B 객체 생성

```python
robot_b = ...
```

### 2. 로봇 B용 컨트롤러 생성

```python
tracking_controller_b = ...
pick_place_controller_b = ...
move_controller_b = ...
```

각 컨트롤러 인스턴스는 로봇 A와 공유하지 않고 별도로 생성해야 한다.

### 3. 로봇 B용 손 입력 생성

손 토픽이 분리된 이후에는 B에 오른손 입력을 연결한다.

```python
right_hand_input = CachedHandInput(
    input_id="RIGHT_HAND",
    target_getter=get_latest_right_hand_target,
    mode_getter=get_latest_right_hand_mode,
    mode_resetter=reset_right_hand_mode_cache,
)
```

### 4. 동일한 상태 머신 생성

```python
state_machine_b = M0609StateMachine(
    robot_id="B",
    robot=robot_b,
    tray_registry=tray_registry,
    hand_input=right_hand_input,
    idle_position=...,
    idle_orientation=...,
    tracking_controller=tracking_controller_b,
    pick_place_controller=pick_place_controller_b,
    move_controller=move_controller_b,
    pick_default_ee_offset=...,
    pick_approach_z_correction=...,
    place_link6_above_tray=...,
    place_high_offset=...,
    place_approach_gap=...,
)
```

### 5. 경로 A, B, C 등록

```python
robot_manager = RobotManager(
    routes={
        "A": (...),
        "B": (...),
        "C": (...),
    },
    shared_trays={3, 4, 5, 6},
)
```

### 6. 로봇 B 등록

```python
robot_manager.register_robot(
    profile=RobotProfile.create(
        robot_id="B",
        reachable_trays={3, 4, 5, 6, 7, 8},
        preferred_route="B",
        fallback_route="C",
    ),
    state_machine=state_machine_b,
)
```

---

## 11. 목표 2대 로봇 구성

계획된 작업 범위는 다음과 같다.

```text
Robot A
- 작업 가능 트레이: 1, 2, 3, 4, 5, 6
- 선호 경로: B
- 우회 경로: A
- 추적 손: 왼손

Robot B
- 작업 가능 트레이: 3, 4, 5, 6, 7, 8
- 선호 경로: B
- 우회 경로: C
- 추적 손: 오른손
```

공동 작업구역:

```text
3, 4, 5, 6
```

계획된 규칙:

- 한 로봇이 공동 작업구역을 사용 중이면 다른 로봇의 공동 작업구역 진입을 제한한다.
- 경로 B는 한 로봇만 점유할 수 있다.
- 경로 B가 사용 중이면 A는 경로 A, B는 경로 C를 사용한다.
- 매니저가 로봇, 트레이, 경로, 공동 작업구역 사용 권한을 결정한다.
- 상태 머신은 배정받은 작업을 실행하고 상태만 보고한다.

---

## 12. 다음 개발 순서

권장 작업 순서:

1. 현재 한 대 구조에서 매니저 경유 PICK–TRACKING–PLACE 검증
2. 상태 머신과 매니저 로그 확인
3. 로봇 B USD 및 articulation 추가
4. B용 컨트롤러와 상태 머신 생성
5. A/B/C 경로 좌표 등록
6. 왼손·오른손 `HandInput` 분리
7. 경로 B 점유 관리 추가
8. 공동 작업구역 점유 관리 추가
9. PLACE 재진입 권한 처리
10. 동시 작업 시나리오 검증

---

## 13. 현재 검증 범위

현재 패치는 Python 문법 구조를 기준으로 정리되었다.

확인이 필요한 실제 실행 항목:

- Isaac Sim에서 모든 import 정상 여부
- 로봇 초기화 후 IDLE 위치 이동
- ROS PICK 명령이 매니저를 거쳐 상태 머신에 전달되는지
- PICK 후 경유 위치 이동
- 손 좌표 TRACKING
- HOME 명령 후 PLACE
- 작업 완료 시 매니저의 `active_task` 해제
- Play 재시작 시 로봇과 트레이 초기화
- 기존 손 개발 노드와 토픽 호환

---

## 14. 현재 핵심 결론

현재 코드는 로봇 A 한 대만 실제로 동작한다.

하지만 상태 머신은 다음과 같이 재사용 가능한 실행기로 변경되었다.

```text
동일한 M0609StateMachine
+ 로봇별 객체
+ 로봇별 컨트롤러
+ 로봇별 HandInput
+ 매니저가 만든 RobotTask
```

따라서 두 번째 로봇 추가 시 상태 머신의 내부 로직을 복사하거나 수정하는 대신, 로봇 B용 객체를 생성하고 매니저에 등록하는 방식으로 확장한다.

현재 단계의 목적은 2대 동시 제어 자체를 완성하는 것이 아니라, 이후 공동 작업구역과 경로 점유 기능을 매니저에 추가해도 상태 머신을 다시 크게 변경하지 않도록 기반 구조를 완성하는 것이다.

---

## 15. 실행 방법

### 15.1 프로젝트 파일 배치

패치 ZIP에 포함된 파일을 현재 프로젝트 루트에 복사한다.

```text
프로젝트 루트/
├── main.py
├── m0609_state_machine.py
├── m0609_ros_bridge.py
├── hand_input.py
├── robot_runtime.py
├── robot_manager.py
├── m0609_config.py
├── m0609_dynamic_scene.py
├── m0609_tracking_controller.py
├── m0609_move_controller.py
├── ...
└── README.md
```

기존 파일을 교체하기 전에 현재 작업 내용을 커밋하거나 백업하는 것을 권장한다.

```bash
git status
git add .
git commit -m "backup before reusable manager patch"
```

패치 적용 후 새 파일이 프로젝트 루트에 있는지 확인한다.

```bash
ls
```

최소한 다음 파일이 보여야 한다.

```text
main.py
m0609_state_machine.py
m0609_ros_bridge.py
hand_input.py
robot_runtime.py
robot_manager.py
```

---

### 15.2 Isaac Sim 실행 터미널 환경 설정

Isaac Sim을 실행할 터미널에서 다음 환경 변수를 설정한다.

```bash
export ROS_DOMAIN_ID=140
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Isaac Sim ROS 2 Bridge 라이브러리 경로도 추가한다.

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib
```

Isaac Sim 설치 위치가 다르면 위 경로를 실제 설치 경로에 맞게 수정해야 한다.

현재 값 확인:

```bash
echo $ROS_DOMAIN_ID
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION
echo $LD_LIBRARY_PATH
```

예상 값:

```text
140
humble
rmw_fastrtps_cpp
```

환경 변수를 매번 입력하지 않으려면 `~/.bashrc`에 추가할 수 있다.

```bash
echo 'export ROS_DOMAIN_ID=140' >> ~/.bashrc
echo 'export ROS_DISTRO=humble' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib' >> ~/.bashrc
source ~/.bashrc
```

---

### 15.3 프로젝트 실행

프로젝트 디렉터리로 이동한다.

현재 작업 경로 예시:

```bash
cd ~/cobot3_ws/isaacpjt/dualrobot_0623
```

실제 프로젝트 위치가 다르면 해당 경로로 변경한다.

Isaac Sim 내장 Python으로 `main.py`를 실행한다.

```bash
~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh main.py
```

Isaac Sim 설치 경로가 다르면 `python.sh` 경로를 실제 설치 위치에 맞게 수정한다.

실행 중 다음 항목을 확인한다.

```text
- 전체 수술실 Stage가 정상적으로 열리는지
- 트레이와 도구가 생성되는지
- M0609 로봇이 초기화되는지
- RobotManager에 Robot A가 등록되는지
- ROS 2 Bridge가 준비 완료 로그를 출력하는지
- 로봇이 IDLE 대기 위치로 이동하는지
```

현재 구조에서는 메인 반복문이 다음 순서로 실행된다.

```text
simulation_app.update()
→ RobotManager.step()
→ Robot A StateMachine.step()
→ PICK / TRACKING / PLACE 제어
```

---

### 15.4 외부 ROS 2 터미널 설정

새 터미널을 열고 ROS 2 환경을 설정한다.

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=140
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Isaac Sim 실행 터미널과 외부 ROS 2 터미널의 `ROS_DOMAIN_ID`와 `RMW_IMPLEMENTATION`이 같아야 한다.

확인:

```bash
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
```

---

### 15.5 ROS 2 연결 확인

현재 사용 중인 토픽 목록을 확인한다.

```bash
ros2 topic list
```

현재 구조에서는 최소한 다음 토픽을 확인한다.

```text
/m0609/pick_command
/hand_raw
/hand_xyz
/hand_mode
```

기존 직접 이동 기능이 활성화된 경우 다음 토픽도 보일 수 있다.

```text
/m0609/move_command
/m0609/move_result
```

PICK 명령 토픽의 연결 정보를 확인한다.

```bash
ros2 topic info /m0609/pick_command -v
```

메시지 타입 확인:

```bash
ros2 topic type /m0609/pick_command
```

예상 출력:

```text
std_msgs/msg/Int32
```

손 모드 토픽 타입 확인:

```bash
ros2 topic type /hand_mode
```

예상 출력:

```text
std_msgs/msg/String
```

---

### 15.6 PICK 명령 보내기

현재 지원 명령은 다음과 같다.

```text
4, 5, 6, 7
```

예를 들어 7번 트레이 작업을 요청한다.

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/Int32 \
  "{data: 7}"
```

6번 트레이 작업:

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/Int32 \
  "{data: 6}"
```

매니저는 다음 조건을 확인한 뒤 작업을 배정한다.

```text
- 명령한 트레이가 Robot A의 작업 가능 범위인지
- Robot A가 IDLE 상태인지
- 진행 중인 작업이 없는지
- 설정된 경유 경로가 존재하는지
```

정상적으로 수락되면 동작 순서는 다음과 같다.

```text
PICK 명령 수신
→ RobotManager가 Robot A 선택
→ RobotTask 생성
→ 상태 머신에 작업 배정
→ PICK
→ 경로 B 위치로 이동
→ TRACKING
```

로봇이 작업 중일 때 새 명령을 보내면 현재 단계에서는 대기열에 저장하지 않고 거부한다.

---

### 15.7 손 추적 및 HOME 테스트

현재 손 개발 노드가 실행 중이라면 다음 토픽이 계속 발행되어야 한다.

```text
/hand_raw
/hand_xyz
/hand_mode
```

토픽 데이터 확인:

```bash
ros2 topic echo /hand_xyz
```

```bash
ros2 topic echo /hand_mode
```

로봇이 `TRACKING` 상태에 들어가면 `/hand_xyz`의 목표 좌표를 따라간다.

손 개발 노드가 아직 연결되지 않았거나 PLACE 전환만 수동으로 시험하려면 다음 명령을 사용한다.

```bash
ros2 topic pub --once \
  /hand_mode \
  std_msgs/msg/String \
  "{data: HOME}"
```

`HOME`을 수신하면 현재 작업 중인 Robot A 상태 머신만 다음 과정으로 전환한다.

```text
TRACKING
→ PLACE
→ 선택했던 트레이 위치로 복귀
→ 도구 내려놓기
→ 경유 위치 이동
→ IDLE
```

다음 작업에서 이전 `HOME` 값이 즉시 재사용되지 않도록 TRACKING 진입 시 손 모드 캐시를 `TRACKING`으로 초기화한다.

---

### 15.8 현재 상태 확인 방법

Isaac Sim 실행 로그에서 다음 항목을 확인한다.

```text
[RobotManager] Robot A 등록
[RobotManager] 작업 배정
[Robot A] IDLE -> PICK
[Robot A] PICK -> TRACKING
[Robot A] TRACKING -> PLACE
[Robot A] PLACE -> IDLE
[RobotManager] 작업 완료
```

정확한 문구는 코드의 로그 형식에 따라 일부 다를 수 있다.

ROS 토픽 발행 여부 확인:

```bash
ros2 topic hz /hand_xyz
```

특정 토픽을 누가 발행하고 구독하는지 확인:

```bash
ros2 topic info /hand_xyz -v
ros2 topic info /hand_mode -v
ros2 topic info /m0609/pick_command -v
```

전체 노드 확인:

```bash
ros2 node list
```

---

### 15.9 반복 테스트 순서

현재 한 대 구조 검증은 다음 순서로 진행한다.

#### 첫 번째 작업

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/Int32 \
  "{data: 7}"
```

로봇이 TRACKING 상태에 들어간 뒤:

```bash
ros2 topic pub --once \
  /hand_mode \
  std_msgs/msg/String \
  "{data: HOME}"
```

PLACE 완료 후 IDLE 복귀를 확인한다.

#### 두 번째 작업

```bash
ros2 topic pub --once \
  /m0609/pick_command \
  std_msgs/msg/Int32 \
  "{data: 6}"
```

다시 TRACKING 상태에 들어간 뒤:

```bash
ros2 topic pub --once \
  /hand_mode \
  std_msgs/msg/String \
  "{data: HOME}"
```

두 번째 작업도 정상적으로 PLACE까지 완료하는지 확인한다.

이 검증에서 확인할 핵심 항목:

```text
- 첫 번째 작업 완료 후 매니저의 active task가 제거되는지
- 상태 머신이 IDLE로 돌아오는지
- 두 번째 PICK 명령이 정상 수락되는지
- 이전 HOME 값 때문에 TRACKING을 건너뛰지 않는지
- 트레이와 도구의 현재 pose가 정상적으로 조회되는지
```

---

### 15.10 종료 방법

Isaac Sim Python 실행 터미널에서:

```text
Ctrl + C
```

를 입력한다.

외부 ROS 2 반복 발행을 실행 중인 경우 해당 터미널에서도:

```text
Ctrl + C
```

를 입력한다.

실행이 비정상 종료되었을 때 남아 있는 ROS 2 노드를 확인한다.

```bash
ros2 node list
```

ROS 2 CLI 데몬 정보가 갱신되지 않는 경우:

```bash
ros2 daemon stop
ros2 daemon start
```

---

### 15.11 실행 문제 확인

#### ROS 토픽이 보이지 않는 경우

두 터미널의 Domain ID를 비교한다.

```bash
echo $ROS_DOMAIN_ID
```

두 터미널 모두 `140`이어야 한다.

RMW 구현도 확인한다.

```bash
echo $RMW_IMPLEMENTATION
```

두 터미널 모두 다음 값이어야 한다.

```text
rmw_fastrtps_cpp
```

#### Isaac Sim에서 ROS 2 Bridge 로드 오류가 발생하는 경우

라이브러리 경로를 확인한다.

```bash
echo $LD_LIBRARY_PATH
```

Isaac Sim 설치 경로 안의 다음 디렉터리가 포함되어야 한다.

```text
exts/isaacsim.ros2.bridge/humble/lib
```

#### `ModuleNotFoundError`가 발생하는 경우

일반 시스템 Python이 아니라 Isaac Sim의 `python.sh`로 실행했는지 확인한다.

```bash
~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh main.py
```

다음 방식으로 실행하면 Isaac Sim 전용 모듈을 찾지 못할 수 있다.

```bash
python3 main.py
```

#### PICK 명령이 거부되는 경우

현재 지원 명령인지 확인한다.

```text
4, 5, 6, 7
```

로봇이 이미 다음 상태라면 새 요청이 거부될 수 있다.

```text
PICK
TRACKING
PLACE
```

기존 작업이 완료되어 `IDLE` 상태로 돌아온 뒤 다시 명령한다.

#### HOME 명령이 동작하지 않는 경우

토픽 이름과 메시지 타입을 확인한다.

```bash
ros2 topic type /hand_mode
ros2 topic echo /hand_mode
```

수동 발행 형식:

```bash
ros2 topic pub --once \
  /hand_mode \
  std_msgs/msg/String \
  "{data: HOME}"
```

대소문자와 앞뒤 공백은 코드에서 정규화하지만, 테스트에서는 `HOME`을 사용하는 것을 권장한다.