# openarm_ws

Feetech 리더암으로 OpenArm v1.0 을 teleoperation 하고 MoveIt 역기구학을 다루는 ROS 2 workspace.

이 리포의 패키지는 리더 입력을 팔로워 컨트롤러 명령으로 옮기는 `openarm_leader` 하나뿐이고,
팔로워 제어·시뮬레이션·MoveIt 은 Enactic 업스트림 스택을 그대로 쓴다.

## 실행 환경

- Ubuntu 24.04, ROS 2 Jazzy (`/opt/ros/jazzy`)
- 실기 추가 준비물: OpenArm v1.0 본체, CAN-FD 어댑터, Feetech STS3215 리더암

## 초기 세팅

배포받은 zip 을 홈 디렉터리에 푼다. 풀린 `~/openarm_ws` 가 workspace 루트다.

```bash
unzip openarm_ws.zip -d ~
```

업스트림 소스를 받아 시뮬레이션용으로 빌드한다. 단계별 설명과 확인 항목은
[docs/simulation.md](docs/simulation.md) 1~4단계에 있다.

```bash
cd ~/openarm_ws
sudo apt install python3-vcstool
vcs import src < openarm.repos
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys openarm_can
colcon build --symlink-install --packages-ignore openarm_hardware openarm
source install/setup.bash
```

## 디렉터리 구성

| 경로 | 내용 |
| --- | --- |
| `docs/` | 단계별 실습 문서 |
| `openarm.repos` | 업스트림 3개 리포의 커밋 고정 |
| `src/openarm_leader/` | 리더 relay 노드, 실습 launch, 셋업 CLI |
| `src/openarm_ros2/`, `src/openarm_description/`, `src/openarm_can/` | 실습 2단계에서 내려받는 업스트림 소스 |

## 실습 문서

| 문서 | 내용 |
| --- | --- |
| [docs/simulation.md](docs/simulation.md) | 환경 구축, mock hardware bringup, 슬라이더 teleop, MoveIt |
| [docs/hardware.md](docs/hardware.md) | CAN-FD 세팅, 팔로워·리더 모터 체크와 캘리브레이션, 실기 teleop |

시뮬레이션 실습부터 진행한다. 환경 구축(시뮬레이션 1~4단계)은 두 실습의 공통 단계다.

## CLI 도구

`openarm_leader` 가 설치하는 리더암 셋업 도구다. 사용법은 [docs/hardware.md](docs/hardware.md).

| 명령 | 용도 |
| --- | --- |
| `ros2 run openarm_leader register` | 서보 id 스캔·배정 |
| `ros2 run openarm_leader check` | 서보 응답 확인, 관절 매핑 실시간 표시 |
| `ros2 run openarm_leader calibrate` | 영점·그리퍼 범위 캘리브레이션 |

팔로워(OpenArm 본체) 쪽 모터 스캔·영점 설정은 업스트림 `openarm-can-cli` 가 맡는다.

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

## 라이선스

Apache-2.0. 업스트림 `openarm_ros2`, `openarm_description`, `openarm_can` 은 각 리포의 라이선스를 따른다.
