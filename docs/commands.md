# 명령어 모음

teleop 과 MoveIt 예제 ex01~ex05 를 돌리는 데 필요한 명령만 모았다. 단계별 설명과 확인 항목은
[시뮬레이션 실습](simulation.md)·[실기 운용](real.md)에 있다.

빌드는 끝난 상태를 전제한다. mock hardware 만 쓸 때는 CAN 항목을 건너뛰고, 실기는 launch 에
`use_fake_hardware:=false` 를 붙인다.

## 터미널 준비

새 터미널마다 매번.

```bash
cd ~/openarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## CAN-FD 기동 (실기만)

`can0` 오른팔, `can1` 왼팔. MoveIt 은 URDF 가 양팔이라 둘 다 올라와 있어야 한다.

```bash
~/openarm_ws/scripts/canup.sh
```

## 팔로워 영점 (실기만)

팔을 영점 자세로 받쳐 든 채 실행한다 — 실행 중 팔에 힘이 없다.

```bash
openarm-can-cli -i can0 set_zero
openarm-can-cli -i can1 set_zero
```

## 리더 보드 고정 이름 (실기, 새 PC 에서 1회)

```bash
ros2 run openarm_leader udev
```

## teleop

터미널 1 — 팔로워 bringup.

```bash
ros2 launch openarm_follower launch.py                          # mock hardware
ros2 launch openarm_follower launch.py use_fake_hardware:=false # 실기
```

터미널 2 — 리더 relay.

```bash
ros2 launch openarm_leader teleop.launch.py source:=sliders # 슬라이더 리더
ros2 launch openarm_leader teleop.launch.py source:=feetech # 실물 리더암
```

## MoveIt 예제 ex01~ex04

teleop 을 끄고 시작한다. 터미널 1 — demo.

```bash
ros2 launch openarm_moveit demo.launch.py                          # mock hardware
ros2 launch openarm_moveit demo.launch.py use_fake_hardware:=false # 실기
```

터미널 2 — 예제 하나씩. 동시에 둘을 돌리지 않는다.

```bash
ros2 run openarm_moveit ex01_joint_goal
ros2 run openarm_moveit ex02_pose_goal
ros2 run openarm_moveit ex03_cartesian_path
ros2 run openarm_moveit ex04_pick_and_place
```

터미널 3 — ex02~ex04 의 단계 진행 신호.

```bash
ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
```

오른팔로 돌리려면 팔을 지정한다(기본 `left`).

```bash
ros2 run openarm_moveit ex01_joint_goal --ros-args -p arm:=right
```

## MoveIt Servo 예제 ex05

demo 를 끄고 Servo 노드까지 포함한 launch 로 다시 띄운다. 터미널 1.

```bash
ros2 launch openarm_moveit servo.launch.py                          # mock hardware
ros2 launch openarm_moveit servo.launch.py use_fake_hardware:=false # 실기
```

터미널 2 — 키보드 노드. 입력은 새로 뜨는 창에서 받는다.

```bash
ros2 run openarm_moveit ex05_keyboard_servo
```

## 게인 모드 전환 (실기만)

팔이 뜬 상태에서 MIT 게인을 바꾼다. 팔마다 노드가 하나씩 뜨고(`openarm_hw_left`·
`openarm_hw_right`), 서비스는 그 아래에 있다. mock hardware 는 플러그인을 쓰지 않으므로
서비스도 없다.

```bash
ros2 service call /openarm_hw_left/gains/default      std_srvs/srv/Trigger   # 실물 게인
ros2 service call /openarm_hw_left/gains/impedance    std_srvs/srv/Trigger   # kp 1/4 — 밀면 물러난다
ros2 service call /openarm_hw_left/gains/zero_gravity std_srvs/srv/Trigger   # kp 0 — 손으로 끈다
```

| 모드 | kp (J1~J7) | kd (J1~J7) |
| --- | --- | --- |
| default | 70 70 70 60 10 10 10 | 2.75 2.5 2.0 2.0 0.7 0.6 0.5 |
| impedance | 17.5 17.5 17.5 15 2.5 2.5 2.5 | 1.65 1.5 1.2 1.2 0.42 0.36 0.3 |
| zero_gravity | 0 0 0 0 0 0 0 | 1.0 1.0 0.8 0.8 0.2 0.2 0.2 |

`impedance` 와 `zero_gravity` 는 중력보상이 켜져 있어야 한다. 꺼져 있으면 서비스가
`success: false` 로 거절한다 — kp 를 내린 팔을 앞먹임이 받쳐 주지 못하면 그대로 주저앉는다.

무중력에서 팔을 끌어다 놓고 `default` 로 돌아오면, 지령은 손으로 옮기기 전 자리에 남아
있다. 그래서 복귀할 때 지령을 현재 자세에서 다시 출발시켜 0.4 rad/s 로 걸어가게 한다 —
팔이 옛 목표로 튕기지 않는다. 걸어가는 동안에는 컨트롤러 지령을 그만큼 늦게 따른다.

## 상태 확인

```bash
ros2 control list_controllers                                  # 컨트롤러 5개 active
ros2 topic echo /left_forward_position_controller/commands     # relay 가 보내는 목표각
openarm-can-cli -i can1 monitor                                # 실기 관절 위치
```
