# 실기 운용

실물 OpenArm 팔로워와 Feetech 리더암을 세팅하고 teleoperation 까지 가는 실습이다.
[시뮬레이션 실습](simulation.md)의 1~4단계(환경 구축)를 마친 상태에서 시작한다.

준비물은 OpenArm v1.0 본체, CAN-FD 어댑터, Feetech STS3215 리더암이다.

실기 패키지와 Feetech SDK 는 시뮬레이션 실습 3~4단계에서 이미 들어가 있다. 시작 전에
`python3 -c "import rclpy, scservo_sdk; print('ok')"` 와 `ros2 pkg list | grep openarm_hardware`
가 각각 출력되는지만 본다.

## 1단계 — CAN-FD 인터페이스

오른팔은 `can0`, 왼팔은 `can1` 이다. 쓰는 팔의 인터페이스를 올린다.

```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
```

```bash
sudo ip link set up can0
sudo ip link set up can1
```

**확인** — `ip -details link show can1` 출력에 `state UP` 과 `fd on` 이 보인다.

## 2단계 — 팔로워 모터 확인

업스트림 `openarm_can` 이 설치하는 `openarm-can-cli` 로 버스의 모터를 스캔한다.
한 팔에는 모터 8개(관절 7 + 그리퍼)가 id 1~8 로 달려 있다.

```bash
openarm-can-cli -i can1 discover
```

**확인** — id 1~8 이 모두 응답으로 나온다. 빠진 id 가 있으면 그 모터의 배선·전원을 확인한다.

## 3단계 — 팔로워 영점

모터가 내는 관절각은 `set_zero` 를 실행한 그 순간의 물리적 자세를 0 으로 삼는다. 그래서 팔을
영점 자세로 세워 두고 굽는 것이 이 단계다. 뒤의 모든 단계가 이 영점을 전제한다 — bringup 이
활성화 때 찾아가는 자세도, 리더 캘리브레이션이 맞추는 기준 자세도, 중력보상이 도는 자세 아홉
개도 전부 관절각이다.

영점 자세는 팔이 어깨에서 아래로 곧게 뻗고 그리퍼가 바닥을 향한 자세다. 어깨 장착점이 바닥에서
0.698 m 이고 그리퍼 끝(`hand_tcp`)이 0.076 m 이니, 손끝은 어깨보다 62 cm 아래에 온다. RViz 로
미리 보려면 시뮬레이션에서 `ros2 launch openarm_moveit display.launch.py` 를 띄우고 슬라이더를
모두 0 에 둔다.

`set_zero` 는 모터를 Disable 한 뒤 굽고 다시 Disable 한다. **그동안 팔에 힘이 없으므로
받쳐 든 채로 실행한다.** 팔을 영점 자세로 잡고:

```bash
openarm-can-cli -i can1 set_zero
```

관절 하나만 다시 잡으려면 `--id 4` 처럼 지정한다.

**확인** — 팔을 영점 자세 그대로 둔 채 텔레메트리를 읽으면 관절 위치가 모두 0 근처다.

```bash
openarm-can-cli -i can1 monitor
```

## 4단계 — 서보 id 배정

Feetech 서보는 출하 상태에서 모두 같은 id 를 쓴다. 팔로워 관절 순서대로 어깨에서 그리퍼까지
1~8 을 하나씩 구워 넣어야 한 버스에서 서로 구분된다. 시리얼 포트를 열려면 `dialout` 그룹이
필요하다.

```bash
sudo usermod -aG dialout $USER
```

다시 로그인한 뒤 버스를 스캔한다. 리더 보드는 `/dev/ttyACM*` 로 잡힌다.

```bash
ros2 run openarm_leader register --port /dev/ttyACM1
```

**확인** — `groups` 에 `dialout` 이 있고, 스캔 결과에 id 1~8 이 모두 나온다.

새로 조립한 팔은 서보를 **하나만** 버스에 연결한 채 id 를 굽고, 다음 서보로 옮겨 8번까지
반복한다. 여러 개가 잡힌 상태에서는 `--from` 으로 바꿀 서보의 현재 id 를 지정한다.

```bash
ros2 run openarm_leader register --port /dev/ttyACM1 --id 3
```

## 5단계 — 리더 보드 고정 이름

`/dev/ttyACM0` 같은 번호는 꽂는 순서에 따라 바뀐다. 보드의 USB 어댑터를 udev 규칙으로
`/dev/openarm_leader_left` 같은 고정 이름에 매어 두면 좌우가 뒤바뀌지 않는다.

```bash
ros2 run openarm_leader udev
```

안내에 따라 보드를 모두 뽑고 Enter, `left` 보드를 꽂고 Enter, `right` 보드를 꽂고 Enter 를
누른다. 새로 나타난 포트를 자동 인식해 어댑터 식별값으로 규칙을 만들고, sudo 로
`/etc/udev/rules.d/99-openarm-leader.rules` 에 등록한 뒤 `config/leader.yaml` 의 `port` 를
고정 이름으로 바꾼다.

**확인** — 보드를 뽑았다 다시 꽂으면 `ls -l /dev/openarm_leader_left` 가 실제 포트를 가리킨다.

한 팔만 다시 등록할 때는 `--arm left` 처럼 지정한다. 어댑터에 serial 번호가 없으면 물리
USB 포트 위치로 고정되므로, 그때는 보드를 늘 같은 포트에 꽂는다.

## 6단계 — 리더 모터 체크

조립을 마친 리더암의 서보 응답과 관절 매핑을 양팔 한 번에 확인한다.

```bash
ros2 run openarm_leader check
```

포트는 `config/leader.yaml` 의 값(5단계에서 고정 이름으로 바뀜)을 쓴다. 한 팔만 보려면
`--arm left`, 포트를 직접 주려면 `--arm left --port /dev/ttyACM1` 처럼 지정한다.

팔마다 id 8개의 응답을 확인한 뒤 관절별 위치를 실시간으로 보여준다. 관절을 하나씩 손으로
움직여 화면의 해당 관절 값만 변하는지, 방향이 `+` 로 갈 관절이 `+` 로 가는지 본다. 종료는 Ctrl+C.

**확인** — 양팔 모두 id 8개가 응답하고, 움직인 관절과 화면에서 변하는 관절이 일치한다.

관절이 어긋나면 서보 id 배정을 `register` 로 확인하고, 방향이 반대면 `config/leader.yaml` 의 `signs` 를 본다.

## 7단계 — 리더 캘리브레이션

리더 서보의 영점과 그리퍼 범위를 잡아 `config/leader.yaml` 에 기록한다. 양팔을 차례로
이어서 진행한다.

```bash
ros2 run openarm_leader calibrate
```

안내에 따라 그 팔의 리더암을 영점 자세로 두고 Enter, 그리퍼를 끝까지 열고 Enter, 끝까지 닫고
Enter 를 누르면 다음 팔로 넘어간다. 영점 자세는 3단계에서 팔로워에 구워 넣은 그 자세다 —
팔로워를 그 자세로 두고 리더암을 그 모양에 맞춘다. 한 팔만 다시 잡으려면 `--arm left` 처럼
지정한다.

**확인** — 팔마다 `offset_ticks`, `gripper open`, `gripper closed` 값이 찍히고 기록한 파일
경로가 나온다. `config/leader.yaml` 의 `arms.<arm>.leader` 항목이 그 값으로 바뀐다.

캘리브레이션이 파일을 다시 쓰면서 `leader.yaml` 의 주석은 사라진다. 값은 모두 유지된다.

관절 방향이 반대로 도는 서보는 같은 파일의 `signs` 를 `1` 과 `-1` 사이에서 뒤집어 맞춘다.

## 8단계 — 중력보상 캘리브레이션

팔로워 모터는 위치 게인만으로는 팔 무게를 다 버티지 못한다. 그래서 하드웨어가 URDF 로 만든
모델로 그 자세의 중력토크를 계산해 토크 지령에 얹는다(`config/follower.yaml` 의 `gravity_comp`).
모델이 모르는 것은 손끝에 달린 미모델 질량과 모터별 토크 영점 오차뿐이고, 그 둘을 재는 것이
이 단계다.

팔로워를 실기로 띄운다.

```bash
ros2 launch openarm_follower launch.py use_fake_hardware:=false
```

새 터미널에서 캘리브레이션을 시작한다. 팔이 스스로 아홉 자세를 돌며 정지 상태의 관절 토크를
잰다. 자세마다 홈을 경유하고 위·아래 두 방향에서 접근하므로 한 팔에 6~8분 걸린다.
**팔이 지나갈 공간을 비우고 비상 정지에 손이 닿는 자리에 선다.**

```bash
ros2 run openarm_follower calibrate_gravity
```

**확인** — 팔마다 `payload_mass`, `payload_com`, `tau_bias` 와 잔차 RMS 가 찍힌다. 잔차가
줄지 않거나(예: 0.6 → 0.5 Nm) 질량이 음수면 저장하지 않는다. 저장에 동의하면
`config/follower.yaml` 의 그 팔 항목만 바뀌고, 다음 bringup 부터 반영된다.

한 팔만 다시 잡으려면 `--arm left` 처럼 지정하고, 재기만 하고 저장하지 않으려면 `--dry-run`
을 붙인다.

보상이 듣는지는 팔을 손으로 들어 옮겨 보면 안다. 보상 전에는 지령 자세보다 아래에 멈추고,
보상 후에는 지령 자세 근처에 머문다.

## 9단계 — 실기 teleop

팔로워를 실기로 띄운다. 활성화 때 양팔이 영점 자세까지 천천히 이동한다 — 관절당 최대
0.4 rad/s 라 어디에 놓여 있어도 빠르게 돌지 않는다. 이동 경로의 공간은 비워 둔다.

```bash
ros2 launch openarm_follower launch.py use_fake_hardware:=false
```

새 터미널에서 리더 relay 를 띄운다. 시작 보간 동안 팔로워가 리더 자세까지 이동한다 —
관절당 최대 0.4 rad/s 라 멀수록 오래 걸린다. 이동 경로의 공간을 비워 둔다.

```bash
ros2 launch openarm_leader teleop.launch.py source:=feetech
```

**확인** — 시작 보간이 끝나면 리더암을 움직이는 대로 실물 팔로워가 따라온다.

## 10단계 — 실기 MoveIt 예제

시뮬레이션 실습 [8·9단계](simulation.md)의 예제 다섯을 실물에서 그대로 돌린다. 코드도 명령도
같고, 바뀌는 것은 launch 인자 하나다. 실기로 띄우면 중력보상 값이 하드웨어 블록에 실려
팔로워 bringup 과 같은 보상 아래에서 돈다.

URDF 가 양팔이라 두 팔의 하드웨어를 모두 연다. **`can0` 과 `can1` 이 둘 다 올라와 있어야
한다** — 1단계를 두 인터페이스에 대해 해 둔다.

teleop 을 끄고 demo 를 실기로 띄운다. 활성화 때 양팔이 영점 자세까지 관절당 최대 0.4 rad/s 로
이동한다. 이동 경로의 공간은 비워 둔다.

```bash
ros2 launch openarm_moveit demo.launch.py use_fake_hardware:=false
```

새 터미널에서 예제를 하나씩 돌린다. 한 번에 하나씩만 — 예제 둘이 동시에 목표를 보내면 궤적이
충돌한다. **예제는 계획한 궤적을 실제로 실행한다.** 계획 속도는 관절 한계의 25% 지만 팔이
작업 영역을 크게 쓰므로(03 은 정사각형 경로, 04 는 집어 옮기기) 앞 공간을 비우고 비상 정지에
손이 닿는 자리에 선다.

```bash
ros2 run openarm_moveit ex01_joint_goal
ros2 run openarm_moveit ex02_pose_goal
ros2 run openarm_moveit ex03_cartesian_path
ros2 run openarm_moveit ex04_pick_and_place
```

02~04 는 단계마다 `/next_step` 신호를 기다린다. 세 번째 터미널에서 한 단계씩 넘긴다 — 실물에서는
이 신호가 곧 "다음 동작을 지금 해도 되는가" 라, 매번 팔 주변을 보고 누른다.

```bash
ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
```

키보드 teleop 은 Servo 노드를 포함한 launch 로 다시 띄운다.

```bash
ros2 launch openarm_moveit servo.launch.py use_fake_hardware:=false
```

```bash
ros2 run openarm_moveit ex05_keyboard_servo
```

**확인** — 시뮬레이션과 같은 출력이 찍히고, RViz 의 팔이 아니라 실물이 그대로 움직인다. 계획은
성공하는데 실물이 지령 자세보다 아래에 멈추면 중력보상이 안 실린 것이다 — 8단계를 다시 본다.

`error_code -26 (START_STATE_INVALID)` 로 전부 실패하면 지금 팔의 어느 관절이 한계 밖이라는
뜻이다. 어느 관절인지는 `openarm-can-cli -i can1 monitor` 로 위치를 읽어 URDF 한계와 대보면
안다. 팔꿈치(joint4)와 그리퍼처럼 아래 한계가 0 인 관절은 영점에 서 있는 것만으로 엔코더 값이
1e-4 rad 쯤 음수가 되는데, 그만큼은 `config/moveit_joint_limits.yaml` 이 이미 넓혀 두었다.
그보다 크게 벗어나 있으면 영점이 어긋난 것이므로 3단계를 다시 한다.
