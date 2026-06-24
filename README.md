# Dual Hand Tracking

두 대의 로봇 제어를 위한 MediaPipe 기반 양손 추적 노드입니다.

## 구성 파일

```text
README.md
hand_tracker.py
hand_landmarker.task
```

## 주요 기능

- Iriun Webcam을 이용한 iPhone 카메라 입력
- MediaPipe Hand Landmarker 기반 양손 인식
- 왼손 / 오른손 좌표 분리 publish
- 손바닥 방향 인식
- 제스처 기반 모드 전환
- ROS2 Humble 토픽 발행
- 오버레이 화면에서 HAND/RAW 좌표, EE 목표 좌표, 손 상태 확인

## 제스처 규칙

```text
손바닥이 카메라 방향 + 주먹 1.5초 유지
→ FOLLOW

손등이 카메라 방향 + 손 펼침 1.5초 유지
→ PLACE
```

## ROS 토픽

### Left Hand

```text
/left_hand_raw
/left_hand_xyz
/left_hand_mode
/left_palm_direction
```

### Right Hand

```text
/right_hand_raw
/right_hand_xyz
/right_hand_mode
/right_palm_direction
```

### Legacy Right Hand Compatibility

```text
/hand_raw
/hand_xyz
/hand_mode
```

## 좌표 의미

```text
*_hand_raw
→ 보정된 손 위치 좌표

*_hand_xyz
→ 로봇 End Effector가 따라갈 최종 목표 좌표
```

로봇 제어에서는 보통 아래 토픽을 사용합니다.

```text
Robot A → /left_hand_xyz, /left_hand_mode
Robot B → /right_hand_xyz, /right_hand_mode
```

## 실행 방법

Iriun 카메라 번호가 4번인 경우:

```bash
cd ~/cobot3_ws/isaacpjt/gripper_technique_test

source /opt/ros/humble/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=142

CAMERA_SOURCE=4 MIRROR_VIEW=0 python hand_tracker.py
```

손바닥 방향이 반대로 잡히면 다음처럼 실행합니다.

```bash
CAMERA_SOURCE=4 MIRROR_VIEW=0 PALM_Z_SIGN=-1 python hand_tracker.py
```

좌우 손이 반대로 표시되면 `MIRROR_VIEW` 값을 바꿔 실행합니다.

```bash
CAMERA_SOURCE=4 MIRROR_VIEW=1 python hand_tracker.py
```

## 캘리브레이션

실행 후 화면 안내에 따라 진행합니다.

```text
1. 손을 카메라에서 약 30cm 위치에 두고 Space
2. 손을 카메라에서 약 100cm 위치에 두고 Space
3. 다시 Space를 눌러 publish 시작
```

## 조작키

```text
Space : 캘리브레이션 / 시작
R     : 재설정
Q/ESC : 종료
```

## 토픽 확인

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=142

ros2 topic list | grep -E "left|right|hand|palm"
```

좌표 확인:

```bash
ros2 topic echo /left_hand_xyz
ros2 topic echo /right_hand_xyz
```

모드 확인:

```bash
ros2 topic echo /left_hand_mode
ros2 topic echo /right_hand_mode
```

발행 속도 확인:

```bash
ros2 topic hz /left_hand_xyz
ros2 topic hz /right_hand_xyz
```

## 의존성

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- OpenCV
- MediaPipe
- NumPy
- Iriun Webcam

## Git 업로드 예시

`handtracking` orphan 브랜치에 세 파일만 올리는 경우:

```bash
cd ~/cobot3_ws/isaacpjt/gripper_technique_test

rm -rf Collected_full_scene rmpflow __pycache__

cp ../dualrobot_0623/hand_tracker.py .
cp ../dualrobot_0623/hand_landmarker.task .
cp ~/Downloads/README.md .

git add README.md hand_tracker.py hand_landmarker.task
git commit -m "add hand tracking files"
git push -u origin handtracking
```

최종 파일 확인:

```bash
git ls-tree --name-only HEAD
```

정상 결과:

```text
README.md
hand_landmarker.task
hand_tracker.py
```
