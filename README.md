# openarm_ws

Feetech 리더암으로 OpenArm v1.0 을 teleoperation 하고 MoveIt 역기구학을 다루는 ROS 2 workspace.

팔로워 제어·시뮬레이션·MoveIt 은 Enactic 업스트림 스택을 그대로 쓴다. 이 리포가 직접 가진 것은
리더 입력을 팔로워 컨트롤러 명령으로 옮기는 `openarm_leader` 패키지 하나다.

5~7단계는 mock hardware 위에서 돌아가 로봇 없이 실습할 수 있다. 실물 장비는 8단계에서만 쓴다.

## 실행 환경

- Ubuntu 24.04, ROS 2 Jazzy (`/opt/ros/jazzy`)
- 8단계 추가 준비물: OpenArm v1.0 본체, CAN-FD 어댑터, Feetech STS3215 리더암

## 디렉터리 구성

| 경로 | 내용 |
| --- | --- |
| `openarm.repos` | 업스트림 3개 리포의 커밋 고정 |
| `src/openarm_leader/` | 리더 relay 노드, 실습 launch, 캘리브레이션 CLI |
| `src/openarm_ros2/`, `src/openarm_description/`, `src/openarm_can/` | 2단계에서 내려받는 업스트림 소스 |

---

## 1단계 — workspace 전개

배포받은 zip 을 홈 디렉터리에 푼다.

```bash
unzip openarm_ws.zip -d ~
```

풀린 `~/openarm_ws` 가 colcon workspace 루트다. 이후 모든 명령은 이 디렉터리에서 실행한다.

```bash
cd ~/openarm_ws
```

**확인** — `ls` 결과에 `openarm.repos`, `README.md`, `src` 가 보인다. `src` 안에는 아직
`openarm_leader` 하나뿐이다.

> git 으로 받는 경우에도 이후 단계는 같다. `git clone <repo> ~/openarm_ws` 로 받은 뒤 2단계부터 진행한다.

## 2단계 — 업스트림 소스 확보

업스트림 리포를 내려받는 도구를 설치한다.

```bash
sudo apt install python3-vcstool
```

`openarm.repos` 에 적힌 커밋으로 3개 리포를 `src/` 에 받는다.

```bash
vcs import src < openarm.repos
```

**확인** — `ls src` 에 네 디렉터리가 보인다.

```
openarm_can  openarm_description  openarm_leader  openarm_ros2
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

## 4단계 — 시뮬레이션용 빌드

`openarm_hardware` 는 CAN-FD 실기 전용 ros2_control 플러그인이고 `openarm` 은 그것을 묶는
metapackage 다. 시뮬레이션 실습에는 둘 다 필요 없으므로 빼고 빌드한다.

```bash
colcon build --symlink-install --packages-ignore openarm_hardware openarm
```

빌드 결과를 환경에 얹는다. 새 터미널을 열 때마다 `/opt/ros/jazzy/setup.bash` 다음에 이 명령이 필요하다.

```bash
source install/setup.bash
```

**확인** — 빌드 요약에 `Summary: 5 packages finished` 가 나오고 실패 패키지가 없다.

## 5단계 — 시뮬레이션 bringup

mock hardware 팔로워와 RViz 를 띄운다.

```bash
ros2 launch openarm_leader teleop.launch.py source:=none
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

`source` 인자를 `sliders` 로 바꾸면 5단계 구성에 슬라이더 창과 relay 노드가 더 붙는다.
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

노드를 띄운 직후에는 팔로워가 슬라이더 자세로 곧장 뛰지 않고 2초에 걸쳐 옮겨간다. 시작 시점의
팔로워 자세에서 리더 자세로 보간하는 구간이다.

## 7단계 — MoveIt 역기구학

5·6단계를 끄고 MoveIt demo 를 띄운다. `arm_type` 기본값이 v2.0 이므로 반드시 지정한다.

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py arm_type:=v1.0
```

**확인** — RViz 왼쪽에 MotionPlanning 패널이 있다. `Planning Group` 을 `left_arm` 으로 두고
끝단의 interactive marker 를 끌어 목표 자세를 잡은 뒤 `Plan & Execute` 를 누르면 팔이 그 자세로
움직인다. 손끝 위치를 주면 관절각을 푸는 역기구학이 그 사이에서 돈다.

## 8단계 — 실기 운용

실물 OpenArm 과 Feetech 리더암으로 같은 실습을 반복한다.

### 8-1. CAN-FD 인터페이스

오른팔은 `can0`, 왼팔은 `can1` 이다. 쓰는 팔의 인터페이스를 올린다.

```bash
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
```

```bash
sudo ip link set up can1
```

**확인** — `ip -details link show can1` 출력에 `state UP` 과 `fd on` 이 보인다.

### 8-2. 리더암 시리얼 포트 권한

```bash
sudo usermod -aG dialout $USER
```

**확인** — 다시 로그인한 뒤 `groups` 에 `dialout` 이 있고, 리더암을 꽂으면 `ls /dev/ttyUSB*` 에
포트가 나온다.

### 8-3. Feetech SDK 설치

시스템 파이썬을 그대로 보는 venv 를 만든다.

```bash
python3 -m venv --system-site-packages ~/venv/openarm
```

```bash
source ~/venv/openarm/bin/activate
```

```bash
pip install feetech-servo-sdk
```

`ros2 run` 은 설치된 실행 스크립트의 shebang 인터프리터로 노드를 띄운다. venv 의 파이썬이 그
자리에 박히도록 venv 를 켠 채로, venv 의 파이썬으로 colcon 을 불러 전체 패키지를 다시 빌드한다.
이번에는 `openarm_hardware` 도 함께 빌드한다.

```bash
python3 -m colcon build --symlink-install
```

```bash
source install/setup.bash
```

**확인** — `Summary: 7 packages finished` 가 나오고, 아래 명령이 두 모듈을 모두 찾는다.

```bash
python3 -c "import rclpy, scservo_sdk; print('ok')"
```

### 8-4. 리더암 캘리브레이션

리더 서보의 영점과 그리퍼 범위를 잡아 `config/leader.yaml` 에 기록한다.

```bash
ros2 run openarm_leader calibrate --arm left --port /dev/ttyUSB0
```

안내에 따라 리더암을 영점 자세로 두고 Enter, 그리퍼를 끝까지 열고 Enter, 끝까지 닫고 Enter 를
누른다.

**확인** — `offset_ticks`, `gripper open`, `gripper closed` 값이 화면에 찍히고 마지막 줄에
기록한 파일 경로가 나온다. `config/leader.yaml` 의 `arms.left.leader` 항목이 그 값으로 바뀐다.

관절 방향이 반대로 도는 서보는 같은 파일의 `signs` 를 `1` 과 `-1` 사이에서 뒤집어 맞춘다.

### 8-5. 실기 teleop

```bash
ros2 launch openarm_leader teleop.launch.py source:=feetech use_fake_hardware:=false arms:=left
```

**확인** — 리더암을 움직이면 실물 팔로워가 따라온다. 처음 2초는 팔로워 현재 자세에서 리더 자세로
옮겨가는 구간이다.

---

## openarm_leader 노드 인터페이스

`leader_node` 는 리더 자세를 읽어 팔로워 컨트롤러 토픽으로 내보낸다.

### 구독

| 토픽 | 타입 | 용도 |
| --- | --- | --- |
| `/leader/joint_states` | `sensor_msgs/JointState` | `source:=sliders` 의 리더 입력 |
| `/joint_states` | `sensor_msgs/JointState` | 시작 보간의 출발 자세 |

### 발행

| 토픽 | 타입 | 용도 |
| --- | --- | --- |
| `/<arm>_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 팔 관절 7개 목표각 |
| `/<arm>_gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 그리퍼 목표 |

`<arm>` 은 `left` 또는 `right` 다. `source:=feetech` 일 때는 구독 없이 시리얼 버스에서 직접 읽는다.

### 파라미터

기본값은 모두 `config/leader.yaml` 에서 온다.

| 이름 | 기본값 | 내용 |
| --- | --- | --- |
| `config_file` | 설치된 `leader.yaml` | 설정 파일 경로 |
| `source` | `sliders` | `none`, `sliders`, `feetech` |
| `arms` | `left,right` | 대상 팔, 쉼표 구분 |
| `rate_hz` | `30.0` | 명령 발행 주기 |
| `ramp_sec` | `2.0` | 시작 보간 시간 |
| `leader_joint_states_topic` | `/leader/joint_states` | 리더 입력 토픽 |
| `follower_joint_states_topic` | `/joint_states` | 팔로워 상태 토픽 |
| `left_port`, `right_port` | `/dev/ttyUSB0`, `/dev/ttyUSB1` | 리더암 시리얼 포트 |

### teleop.launch.py 인자

| 인자 | 기본값 | 내용 |
| --- | --- | --- |
| `source` | `sliders` | `none` 은 팔로워만, `sliders` 는 슬라이더 리더, `feetech` 는 실물 리더암 |
| `arms` | `left,right` | 대상 팔 |
| `arm_type` | `v1.0` | OpenArm 모델 |
| `use_fake_hardware` | `true` | `false` 면 CAN-FD 실기 |
| `config_file` | 설치된 `leader.yaml` | 설정 파일 경로 |
| `left_can_interface` | `can1` | 왼팔 CAN 인터페이스 |
| `right_can_interface` | `can0` | 오른팔 CAN 인터페이스 |

## config/leader.yaml

teleop 설정의 단일 진실 공급원이다. 환경변수는 쓰지 않는다.

| 항목 | 내용 |
| --- | --- |
| `source`, `active_arms`, `rate_hz`, `ramp_sec` | 노드 파라미터 기본값 |
| `gripper_travel` | 팔로워 finger 관절 이동 범위 [m] |
| `feetech` | 리더 버스 baudrate, protocol_end, tick 해상도, present position 주소 |
| `arms.<arm>.joint_limits_deg` | 팔로워 관절 clamp 범위 [deg] |
| `arms.<arm>.leader.ids` | 팔로워 관절 순서로 적은 리더 서보 id. 마지막이 그리퍼 |
| `arms.<arm>.leader.signs` | 관절별 회전 방향 |
| `arms.<arm>.leader.offset_ticks` | 관절 영점 tick |
| `arms.<arm>.leader.gripper_ticks` | 그리퍼 열림·닫힘 tick |

`ids` 의 6·7번 자리가 뒤집혀 있다. 리더 6번 서보가 팔로워 joint7 에, 7번 서보가 joint6 에 걸려 있다.

## 문제 해결

| 증상 | 원인과 조치 |
| --- | --- |
| RViz 에 로봇이 안 보인다 | `source install/setup.bash` 를 빼먹었다. 그 터미널에서 실행하고 다시 띄운다 |
| 슬라이더를 움직여도 팔로워가 그대로다 | 팔로워 `/joint_states` 를 못 받은 상태다. `ros2 control list_controllers` 로 `joint_state_broadcaster` 가 `active` 인지 본다 |
| 관절 하나가 슬라이더보다 일찍 멈춘다 | `joint_limits_deg` clamp 다. 필요하면 그 관절의 범위를 넓힌다 |
| MoveIt 에서 팔 모양이 다르다 | `arm_type:=v1.0` 을 빠뜨렸다. 기본값이 v2.0 이다 |
| `ModuleNotFoundError: scservo_sdk` | venv 를 켠 채로 `python3 -m colcon build` 를 다시 돌린다 (8-3) |
| `could not open port /dev/ttyUSB0` | 포트 번호와 `dialout` 그룹을 확인한다 (8-2) |

## 라이선스

Apache-2.0. 업스트림 `openarm_ros2`, `openarm_description`, `openarm_can` 은 각 리포의 라이선스를 따른다.
