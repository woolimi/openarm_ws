# 실기 운용

실물 OpenArm 팔로워와 Feetech 리더암을 세팅하고 teleoperation 까지 가는 실습이다.
[시뮬레이션 실습](simulation.md)의 1~4단계(환경 구축)를 마친 상태에서 시작한다.

준비물은 OpenArm v1.0 본체, CAN-FD 어댑터, Feetech STS3215 리더암이다.

## 1단계 — CAN-FD 인터페이스

오른팔은 `can0`, 왼팔은 `can1` 이다. 쓰는 팔의 인터페이스를 올린다.

```bash
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
```

```bash
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

모터 상태는 실시간으로도 볼 수 있다. 손으로 관절을 움직이면 해당 id 의 위치값이 변한다.

```bash
openarm-can-cli -i can1 monitor
```

## 3단계 — 팔로워 영점 설정

모터를 교체했거나 영점이 틀어진 경우에만 하는 단계다. 출하 시 영점이 잡혀 있으면 건너뛴다.

`set_zero` 는 모터의 현재 위치를 기계적 영점으로 EEPROM 에 기록한다. 토크를 끄고 팔을 영점
자세로 맞춘 뒤 실행한다.

```bash
openarm-can-cli -i can1 disable
```

```bash
openarm-can-cli -i can1 set_zero
```

**확인** — `openarm-can-cli -i can1 monitor` 에서 영점 자세의 모든 관절 위치가 0 근처다.

## 4단계 — 리더암 시리얼 포트 권한

```bash
sudo usermod -aG dialout $USER
```

**확인** — 다시 로그인한 뒤 `groups` 에 `dialout` 이 있고, 리더암을 꽂으면 `ls /dev/ttyUSB*` 에
포트가 나온다.

## 5단계 — 리더 보드 고정 이름

`/dev/ttyUSB0` 같은 번호는 꽂는 순서에 따라 바뀐다. 보드의 USB 어댑터를 udev 규칙으로
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

## 6단계 — Feetech SDK 설치

pip 를 설치하고, 리더 서보와 통신하는 파이썬 SDK 를 사용자 영역(`~/.local`)에 넣는다.
Ubuntu 24.04 는 시스템 파이썬에 대한 pip 설치를 막으므로 `--break-system-packages` 를 붙인다
(사용자 홈에만 설치되고 시스템 패키지는 건드리지 않는다).

```bash
sudo apt install python3-pip
```

```bash
python3 -m pip install --user --break-system-packages feetech-servo-sdk
```

시뮬레이션 빌드에서 뺐던 실기 전용 패키지(`openarm_hardware` 와 metapackage `openarm`)를
마저 빌드한다.

```bash
colcon build --symlink-install
```

```bash
source install/setup.bash
```

**확인** — `Summary: 8 packages finished` 가 나오고, 아래 명령이 두 모듈을 모두 찾는다.

```bash
python3 -c "import rclpy, scservo_sdk; print('ok')"
```

## 7단계 — 리더 서보 id 등록

리더암을 새로 조립했거나 서보를 교체한 경우에만 하는 단계다. 서보는 공장 출하 시 모두 id 1 이라,
조립 전에 서보를 **하나씩** 버스에 연결해 id 를 배정한다. 배정표는 `config/leader.yaml` 의
`ids` 를 따른다 — joint1 부터 차례로 1~5, joint6 은 **7**, joint7 은 **6**, 그리퍼는 8 이다.

서보 하나만 연결한 상태에서 실행한다.

```bash
ros2 run openarm_leader register --port /dev/ttyUSB0 --id 3
```

인자 없이 실행하면 버스를 스캔만 한다. 버스에 서보가 여러 개면 `--from` 으로 바꿀 서보의 현재
id 를 지정한다.

**확인** — `id 1 → 3 변경 완료.` 가 나오고, 다시 스캔하면 새 id 로 응답한다.

## 8단계 — 리더 모터 체크

조립을 마친 리더암의 서보 응답과 관절 매핑을 양팔 한 번에 확인한다.

```bash
ros2 run openarm_leader check
```

포트는 `config/leader.yaml` 의 값(5단계에서 고정 이름으로 바뀜)을 쓴다. 한 팔만 보려면
`--arm left`, 포트를 직접 주려면 `--arm left --port /dev/ttyUSB0` 처럼 지정한다.

팔마다 id 8개의 응답을 확인한 뒤 관절별 위치를 실시간으로 보여준다. 관절을 하나씩 손으로
움직여 화면의 해당 관절 값만 변하는지, 방향이 `+` 로 갈 관절이 `+` 로 가는지 본다. 종료는 Ctrl+C.

**확인** — 양팔 모두 id 8개가 응답하고, 움직인 관절과 화면에서 변하는 관절이 일치한다.

관절이 어긋나면 서보 id 배정(7단계)을, 방향이 반대면 `config/leader.yaml` 의 `signs` 를 본다.

## 9단계 — 리더 캘리브레이션

리더 서보의 영점과 그리퍼 범위를 잡아 `config/leader.yaml` 에 기록한다. 양팔을 차례로
이어서 진행한다.

```bash
ros2 run openarm_leader calibrate
```

안내에 따라 그 팔의 리더암을 영점 자세로 두고 Enter, 그리퍼를 끝까지 열고 Enter, 끝까지 닫고
Enter 를 누르면 다음 팔로 넘어간다. 영점 자세는 팔로워의 영점과 같은 자세다 — 팔로워를 3단계
영점 자세로 두고 리더암을 그 모양에 맞춘다. 한 팔만 다시 잡으려면 `--arm left` 처럼 지정한다.

**확인** — 팔마다 `offset_ticks`, `gripper open`, `gripper closed` 값이 찍히고 기록한 파일
경로가 나온다. `config/leader.yaml` 의 `arms.<arm>.leader` 항목이 그 값으로 바뀐다.

캘리브레이션이 파일을 다시 쓰면서 `leader.yaml` 의 주석은 사라진다. 값은 모두 유지된다.

관절 방향이 반대로 도는 서보는 같은 파일의 `signs` 를 `1` 과 `-1` 사이에서 뒤집어 맞춘다.

## 10단계 — 실기 teleop

```bash
ros2 launch openarm_leader teleop.launch.py source:=feetech use_fake_hardware:=false arms:=left
```

**확인** — 리더암을 움직이면 실물 팔로워가 따라온다. 처음 2초는 팔로워 현재 자세에서 리더 자세로
옮겨가는 구간이다.

## 문제 해결

| 증상 | 원인과 조치 |
| --- | --- |
| `openarm-can-cli discover` 에 모터가 안 나온다 | CAN 인터페이스가 안 올라갔거나 배선 문제다. 1단계 확인 명령부터 다시 본다 |
| `scservo_sdk 를 찾지 못했다` | Feetech SDK 를 설치한다 (6단계) |
| `could not open port /dev/ttyUSB0` | 포트 번호와 `dialout` 그룹을 확인한다 (4단계) |
| `/dev/openarm_leader_*` 가 안 생긴다 | udev 규칙은 다시 꽂을 때 적용된다. 보드를 뽑았다 다시 꽂는다 (5단계) |
| `check` 에서 일부 서보가 응답 없음 | 배선 순서·id 배정을 확인한다 (7단계) |
| 리더를 움직여도 팔로워가 그대로다 | 팔로워 `/joint_states` 를 못 받은 상태다. `ros2 control list_controllers` 로 `joint_state_broadcaster` 가 `active` 인지 본다 |
| 관절 하나가 리더보다 일찍 멈춘다 | `joint_limits_deg` clamp 다. 필요하면 그 관절의 범위를 넓힌다 |
