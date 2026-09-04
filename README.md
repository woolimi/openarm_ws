# openarm_ws

OpenArm v1.0 실습용 ROS 2 workspace.

- mock hardware 팔로워 bringup — RViz 시뮬레이션 (`openarm_follower`)
- 중력보상 설정과 캘리브레이션 CLI (`openarm_follower`)
- 슬라이더·Feetech 리더암 입력의 teleoperation relay (`openarm_leader`)
- 리더암 셋업 CLI — udev·서보 id·캘리브레이션 (`openarm_leader`)
- MoveIt 실습 — demo 조립, Python 예제 다섯, Servo teleop (`openarm_moveit`)
- CAN-FD 실기 구동 — Enactic 제어 스택 내장 (`openarm_ros2`)

## 실행 환경

- Ubuntu 24.04, ROS 2 Jazzy (`/opt/ros/jazzy`)
- 실기 추가 준비물: OpenArm v1.0 본체, CAN-FD 어댑터, Feetech STS3215 리더암

## 초기 세팅

배포받은 zip 을 홈 디렉터리에 푼다. 풀린 `~/openarm_ws` 가 workspace 루트다.

```bash
unzip openarm_ws.zip -d ~
```

업스트림 소스를 받아 시뮬레이션·실기 패키지를 한 번에 빌드한다. 단계별 설명과 확인 항목은
[docs/simulation.md](docs/simulation.md) 1~4단계에 있다.

```bash
cd ~/openarm_ws
sudo apt install python3-vcstool
vcs import src < openarm.repos
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys openarm_can
colcon build --symlink-install
python3 -m pip install --user --break-system-packages feetech-servo-sdk
source install/setup.bash
```

## 디렉터리 구성

| 경로 | 내용 |
| --- | --- |
| `docs/` | 단계별 실습 문서 |
| `openarm.repos` | 업스트림 2개 리포의 커밋 고정 |
| `src/openarm_follower/` | 팔로워 bringup launch, 중력보상 설정과 캘리브레이션 CLI |
| `src/openarm_leader/` | 리더 relay 노드, teleop launch, 셋업 CLI |
| `src/openarm_moveit/` | MoveIt demo·Servo launch, SRDF·관절 한계·컨트롤러 설정, Python 예제 5개 |
| `src/openarm_ros2/` | 내장한 Enactic 팔로워 제어·MoveIt 스택 |
| `src/openarm_description/`, `src/openarm_can/` | 실습 2단계에서 내려받는 업스트림 소스 |

## 실습 문서

| 문서 | 내용 |
| --- | --- |
| [docs/simulation.md](docs/simulation.md) | 환경 구축, mock hardware bringup, 슬라이더 teleop, MoveIt, Python 예제 |
| [docs/real.md](docs/real.md) | CAN-FD 세팅, 팔로워·리더 모터 체크와 캘리브레이션, 중력보상 실측, 실기 teleop |

시뮬레이션 실습부터 진행한다. 환경 구축(시뮬레이션 1~4단계)은 두 실습의 공통 단계다.

## CLI 도구

실기 셋업 도구다. 사용법은 [docs/real.md](docs/real.md).

| 명령 | 용도 |
| --- | --- |
| `ros2 run openarm_leader udev` | 보드 고정 장치 이름(udev rule) 등록 |
| `ros2 run openarm_leader register` | 서보 id 스캔·배정 |
| `ros2 run openarm_leader check` | 서보 응답 확인, 관절 매핑 실시간 표시 |
| `ros2 run openarm_leader calibrate` | 리더암 영점·그리퍼 범위 캘리브레이션 |
| `ros2 run openarm_follower calibrate` | 중력보상 페이로드·토크 오프셋 실측 |

팔로워(OpenArm 본체) 쪽 모터 스캔은 업스트림 `openarm-can-cli` 가 맡는다.

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
| `ramp_sec` | `2.0` | 시작 보간 최소 시간 (팔 관절만, 관절 속도 0.5 rad/s 상한) |
| `gripper_smoothing_alpha` | `0.25` | 그리퍼 low-pass 계수, 1 이면 필터 없음 |
| `leader_joint_states_topic` | `/leader/joint_states` | 리더 입력 토픽 |
| `follower_joint_states_topic` | `/joint_states` | 팔로워 상태 토픽 |
| `left_port`, `right_port` | `/dev/ttyUSB0`, `/dev/ttyUSB1` | 리더암 시리얼 포트 |

### teleop.launch.py 인자

| 인자 | 기본값 | 내용 |
| --- | --- | --- |
| `source` | `sliders` | `sliders` 는 슬라이더 리더, `feetech` 는 실물 리더암 |
| `arms` | `left,right` | 대상 팔 |
| `config_file` | 설치된 `leader.yaml` | 설정 파일 경로 |

### openarm_follower launch.py 인자

| 인자 | 기본값 | 내용 |
| --- | --- | --- |
| `use_fake_hardware` | `true` | `true` 는 mock hardware, `false` 는 CAN-FD 실기 |
| `left_can_interface` | `can1` | 왼팔 CAN 인터페이스 |
| `right_can_interface` | `can0` | 오른팔 CAN 인터페이스 |
| `config_file` | 설치된 `follower.yaml` | 중력보상 설정 파일 경로 |

## openarm_moveit 예제

`demo.launch.py` 가 mock hardware · move_group · 컨트롤러 · RViz 를 한 번에 띄우고, `servo.launch.py` 는
그 위에 `moveit_servo` 노드를 더한다. 예제는 그 위에서 돈다. 모든 예제는 `arm:=left|right` 파라미터로
팔을 고른다(기본 `left`). 예제는 한 번에 하나만 돌린다 — 둘이 동시에 목표를 보내면 궤적이 충돌한다.

| 명령 | 내용 |
| --- | --- |
| `ros2 launch openarm_moveit display.launch.py` | URDF 형상만 RViz 로 — 컨트롤러 없이 슬라이더로 관절을 움직인다 |
| `ros2 launch openarm_moveit demo.launch.py` | MoveIt demo + 예제용 RViz |
| `ros2 run openarm_moveit ex01_joint_goal` | 관절 하나씩 움직이기 — MoveGroup 액션, JointConstraint |
| `ros2 run openarm_moveit ex02_pose_goal` | 손끝 자세 지령 — PositionConstraint·OrientationConstraint, IK |
| `ros2 run openarm_moveit ex03_cartesian_path` | 손끝 직선 보간 — `compute_cartesian_path`, fraction, ExecuteTrajectory |
| `ros2 run openarm_moveit ex04_pick_and_place` | pick and place — seed IK, GripperCommand 액션 |
| `ros2 launch openarm_moveit servo.launch.py` | MoveIt demo + Servo 노드 |
| `ros2 run openarm_moveit ex05_keyboard_servo` | 키보드 teleop — TwistStamped 를 Servo 로 |

ex02~ex04 는 단계마다 `/next_step` 신호를 기다린다. 다른 터미널에서 한 단계씩 넘긴다.

```bash
ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
```

설정은 `openarm_moveit/robot.py`(프레임·planning group·이름 붙은 자세·손끝 방향) 와
`config/`(SRDF·관절 한계·컨트롤러·RViz·Servo) 에 있다.

`display.launch.py` 는 업스트림 `openarm_description` 의 `display_openarm.launch.py` 를 감싼 것이다.
업스트림 기본값은 `arm_type` 이 v2.0 이라 그대로 부르면 다른 로봇이 뜬다. 이 launch 는 v1.0 을 박아
두었으므로 인자 없이 부르면 된다. 한 팔만 보려면 `bimanual:=false` 를 준다.

## config/leader.yaml

teleop 설정의 단일 진실 공급원이다. 환경변수는 쓰지 않는다.

| 항목 | 내용 |
| --- | --- |
| `source`, `active_arms`, `rate_hz`, `ramp_sec`, `gripper_smoothing_alpha` | 노드 파라미터 기본값 |
| `gripper_travel` | 팔로워 finger 관절 이동 범위 [m] |
| `feetech` | 리더 버스 baudrate, protocol_end, tick 해상도, present position 주소 |
| `arms.<arm>.joint_limits_deg` | 팔로워 관절 clamp 범위 [deg] |
| `arms.<arm>.leader.ids` | 팔로워 관절 순서로 적은 리더 서보 id. 마지막이 그리퍼 |
| `arms.<arm>.leader.signs` | 관절별 회전 방향 |
| `arms.<arm>.leader.offset_ticks` | 관절 영점 tick |
| `arms.<arm>.leader.gripper_ticks` | 그리퍼 열림·닫힘 tick |

## 중력보상

팔로워 하드웨어는 MIT 임피던스로 돈다 — 관절 토크는 `kp·(q* − q) + kd·(q̇* − q̇) + τ_ff` 다.
`τ_ff` 를 0 으로 두면 위치 게인만으로 팔 무게를 버텨야 해서, 지령 사이에서 팔이 아래로 쳐진다.
`gravity_comp` 를 켜면 URDF 로 만든 KDL 모델이 그 자세의 중력토크 G(q) 를 계산해 `τ_ff` 에 얹는다.
모터가 무게를 들고, PD 항은 추종 오차만 고친다.

모델이 모르는 것은 두 가지다. 손끝에 달린 미모델 질량(그리퍼 손가락·배선·물린 물체)과 모터마다
다른 토크 영점 오차다. 둘 다 로봇 한 대의 실측이라 `config/follower.yaml` 에 산다.

## config/follower.yaml

팔로워 설정의 단일 진실 공급원이다. bringup 이 이 값을 URDF 의 하드웨어 블록에 실어
플러그인에 넘긴다. 환경변수는 쓰지 않는다.

| 항목 | 내용 |
| --- | --- |
| `gravity_comp` | 중력 피드포워드 on/off |
| `root_link` | 중력을 표현하는 프레임. world 정렬 링크여야 한다 |
| `saturation_cap` | 피드포워드 항의 관절별 절대 상한 [Nm] |
| `arms.<arm>.tip_link` | 중력 모델 사슬의 끝. `payload_com` 의 기준 프레임이기도 하다 |
| `arms.<arm>.payload_mass` | 손끝 미모델 질량 [kg] — 캘리브레이션 산출값 |
| `arms.<arm>.payload_com` | 그 질량의 무게중심 [m] — 캘리브레이션 산출값 |
| `arms.<arm>.tau_bias` | 관절별 상수 토크 오프셋 [Nm] — 캘리브레이션 산출값 |
| `calibration` | 자세 이동 속도, 안정화·표본 시간, 양방향 접근 |
| `plausibility` | 저장 전 타당성 상자. 자릿수 오타를 여기서 막는다 |

## 라이선스

Apache-2.0. 내장한 `openarm_ros2` 와 업스트림 `openarm_description`·`openarm_can` 은 Enactic 의 Apache-2.0 을 따른다.
