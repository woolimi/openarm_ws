# 시뮬레이션 실습

로봇 없이 mock hardware 위에서 진행하는 실습이다. 환경 구축(1~4단계)은 실기 실습에서도
그대로 쓰는 공통 단계다.

## 1단계 — workspace 전개

배포받은 zip 을 홈 디렉터리에 푼다.

```bash
unzip openarm_ws.zip -d ~
```

풀린 `~/openarm_ws` 가 colcon workspace 루트다. 이후 모든 명령은 이 디렉터리에서 실행한다.

```bash
cd ~/openarm_ws
```

**확인** — `ls` 결과에 `openarm.repos`, `README.md`, `src` 가 보인다. `src` 안에는 이 리포의
패키지들과 팔로워 제어 스택 `openarm_ros2` 가 들어 있다.

> git 으로 받는 경우에도 이후 단계는 같다. `git clone <repo> ~/openarm_ws` 로 받은 뒤 2단계부터 진행한다.

## 2단계 — 업스트림 소스 확보

업스트림 리포를 내려받는 도구를 설치한다.

```bash
sudo apt install python3-vcstool
```

`openarm.repos` 에 적힌 커밋으로 2개 리포를 `src/` 에 받는다.

```bash
vcs import src < openarm.repos
```

**확인** — `ls src` 에 여섯 디렉터리가 보인다.

```
openarm_can  openarm_description  openarm_follower  openarm_leader  openarm_moveit  openarm_ros2
```

## 3단계 — 의존성 설치

ROS 2 환경을 연다. 새 터미널을 열 때마다 이 명령이 필요하다.

```bash
source /opt/ros/jazzy/setup.bash
```

패키지가 요구하는 의존성을 채운다.

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys openarm_can
```

**확인** — 마지막 줄에 `#All required rosdeps installed successfully` 가 나온다.

## 4단계 — 전체 빌드

시뮬레이션과 실기에 쓰는 패키지를 한 번에 빌드한다. `openarm_hardware`(CAN-FD 실기 전용
ros2_control 플러그인)와 metapackage `openarm` 도 로봇 없이 빌드되므로 여기서 함께 올려 둔다.

```bash
colcon build --symlink-install
```

빌드 결과를 환경에 얹는다. 새 터미널을 열 때마다 `/opt/ros/jazzy/setup.bash` 다음에 이 명령이 필요하다.

```bash
source install/setup.bash
```

**확인** — 빌드 요약에 `Summary: 9 packages finished` 가 나오고 실패 패키지가 없다.

리더암 서보와 통신하는 Feetech SDK 는 rosdep 이 다루지 않으므로 pip 으로 따로 넣는다.
Ubuntu 24.04 는 시스템 파이썬에 대한 pip 설치를 막으므로 `--break-system-packages` 를 붙인다
(사용자 홈 `~/.local` 에만 설치되고 시스템 패키지는 건드리지 않는다).

```bash
sudo apt install python3-pip
python3 -m pip install --user --break-system-packages feetech-servo-sdk
```

**확인** — `python3 -c "import rclpy, scservo_sdk; print('ok')"` 가 `ok` 를 찍는다.

## 5단계 — 시뮬레이션 bringup

mock hardware 팔로워와 RViz 를 띄운다. follower·MoveIt launch 는 시작할 때 이전 실행이 남긴
세션 프로세스(ros2_control·RViz 등)를 먼저 정리하므로, 창을 덜 닫고 다시 띄워도 겹치지 않는다.

```bash
ros2 launch openarm_follower launch.py
```

**확인** — RViz 창에 양팔 OpenArm 이 보인다. 새 터미널에서 아래를 실행하면 컨트롤러 다섯 개가
`active` 다.

```bash
ros2 control list_controllers
```

```
joint_state_broadcaster            joint_state_broadcaster/JointStateBroadcaster          active
left_forward_position_controller   forward_command_controller/ForwardCommandController    active
right_forward_position_controller  forward_command_controller/ForwardCommandController    active
left_gripper_controller            joint_trajectory_controller/JointTrajectoryController  active
right_gripper_controller           joint_trajectory_controller/JointTrajectoryController  active
```

## 6단계 — 슬라이더 teleop

5단계 팔로워를 켜둔 채, 새 터미널에서 teleop 을 띄우면 슬라이더 창과 relay 노드가 더 붙는다.
슬라이더가 리더암 역할을 한다.

```bash
ros2 launch openarm_leader teleop.launch.py source:=sliders
```

**확인** — 슬라이더 창을 움직이면 RViz 의 로봇이 따라온다. 신호가 지나는 경로는 새 터미널에서
직접 볼 수 있다.

```bash
ros2 topic echo /left_forward_position_controller/commands
```

슬라이더를 끝까지 밀어도 로봇이 먼저 멈추는 관절이 있다. `config/leader.yaml` 의
`joint_limits_deg` 가 URDF 한계보다 좁게 잡혀 있어 relay 노드가 그 범위로 clamp 한다.

노드를 띄운 직후에는 팔로워 팔이 슬라이더 자세로 곧장 뛰지 않고 최소 2초에 걸쳐 옮겨간다.
시작 시점의 팔로워 자세에서 리더 자세로 보간하는 구간이고, 거리가 멀면 관절당 0.5 rad/s 를
넘지 않게 시간이 늘어난다. 그리퍼는 보간 없이 바로 따른다.

## 7단계 — MoveIt 역기구학

5·6단계를 끄고 MoveIt demo 를 띄운다.

```bash
ros2 launch openarm_moveit demo.launch.py
```

**확인** — RViz 왼쪽에 MotionPlanning 패널이 있다. `Planning Group` 을 `left_arm` 으로 두고
끝단의 interactive marker 를 끌어 목표 자세를 잡은 뒤 `Plan & Execute` 를 누르면 팔이 그 자세로
움직인다. 손끝 위치를 주면 관절각을 푸는 역기구학이 그 사이에서 돈다. `left_gripper` 그룹은
`open`·`closed` 자세로 `Plan & Execute` 하면 손가락이 여닫힌다.

## 8단계 — MoveIt Python 예제

7단계의 demo 를 그대로 둔 채 새 터미널에서 예제를 하나씩 돌린다. 같은 launch 이고, RViz 설정
`config/demo.rviz` 에 예제가 그리는 마커(`/openarm_markers`) display 가 이미 켜져 있다.

한 번에 하나씩만 돌린다 — 예제 둘이 동시에 MoveGroup 에 목표를 보내면 궤적이 충돌한다. 예제 01 은 관절 7개를 하나씩 움직이고, 02~04 는 단계마다
`/next_step` 신호를 기다린다.

```bash
ros2 run openarm_moveit ex01_joint_goal
ros2 run openarm_moveit ex02_pose_goal
ros2 run openarm_moveit ex03_cartesian_path
ros2 run openarm_moveit ex04_pick_and_place
```

세 번째 터미널에서 신호를 보낼 때마다 한 단계 진행한다.

```bash
ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
```

**확인** — 예제 02 는 목표 셋(`Front`·`Left`·`High`)이 초록으로 바뀌고, 예제 03 은 정사각형과
하강 경로의 달성률이 `100.0%` 로 찍히며, 예제 04 는 그리퍼가 `0.044 m` 로 열렸다 `0.000 m` 로
닫힌다.

## 9단계 — MoveIt Servo 키보드 teleop

demo 를 끄고 Servo 노드를 포함한 launch 로 다시 띄운 뒤, 새 터미널에서 키보드 노드를 연다.

```bash
ros2 launch openarm_moveit servo.launch.py
```

```bash
ros2 run openarm_moveit ex05_keyboard_servo
```

**확인** — 팔이 ready 자세로 간 뒤 `Servo READY` 가 뜬다. `w/s`·`a/d`·`q/e` 를 누르는 동안 손끝이
x·y·z 로, `W/S`·`A/D`·`Q/E` 는 roll·pitch·yaw 로 움직인다. 관절 한계나 특이자세에서 상태가
`정지` 로 바뀌면 `r` 로 ready 에 돌아온다. `ESC` 로 끝낸다.

## 다음 실습

실물 OpenArm 과 Feetech 리더암 세팅은 [실기 운용](real.md)으로 이어진다.
