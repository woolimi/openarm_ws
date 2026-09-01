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

## 5단계 — Feetech SDK 설치

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

## 6단계 — 리더 서보 id 등록

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

## 7단계 — 리더 모터 체크

조립을 마친 리더암의 서보 응답과 관절 매핑을 확인한다.

```bash
ros2 run openarm_leader check --arm left --port /dev/ttyUSB0
```

id 8개의 응답을 확인한 뒤 관절별 위치를 실시간으로 보여준다. 관절을 하나씩 손으로 움직여
화면의 해당 관절 값만 변하는지, 방향이 `+` 로 갈 관절이 `+` 로 가는지 본다. 종료는 Ctrl+C.

**확인** — id 8개가 모두 응답하고, 움직인 관절과 화면에서 변하는 관절이 일치한다.

관절이 어긋나면 서보 id 배정(6단계)을, 방향이 반대면 `config/leader.yaml` 의 `signs` 를 본다.

## 8단계 — 리더 캘리브레이션

리더 서보의 영점과 그리퍼 범위를 잡아 `config/leader.yaml` 에 기록한다.

```bash
ros2 run openarm_leader calibrate --arm left --port /dev/ttyUSB0
```

안내에 따라 리더암을 영점 자세로 두고 Enter, 그리퍼를 끝까지 열고 Enter, 끝까지 닫고 Enter 를
누른다. 영점 자세는 팔로워의 영점과 같은 자세다 — 팔로워를 3단계 영점 자세로 두고 리더암을
그 모양에 맞춘다.

**확인** — `offset_ticks`, `gripper open`, `gripper closed` 값이 화면에 찍히고 마지막 줄에
기록한 파일 경로가 나온다. `config/leader.yaml` 의 `arms.left.leader` 항목이 그 값으로 바뀐다.

캘리브레이션이 파일을 다시 쓰면서 `leader.yaml` 의 주석은 사라진다. 값은 모두 유지된다.

관절 방향이 반대로 도는 서보는 같은 파일의 `signs` 를 `1` 과 `-1` 사이에서 뒤집어 맞춘다.

## 9단계 — 실기 teleop

```bash
ros2 launch openarm_leader teleop.launch.py source:=feetech use_fake_hardware:=false arms:=left
```

**확인** — 리더암을 움직이면 실물 팔로워가 따라온다. 처음 2초는 팔로워 현재 자세에서 리더 자세로
옮겨가는 구간이다.

## 문제 해결

| 증상 | 원인과 조치 |
| --- | --- |
| `openarm-can-cli discover` 에 모터가 안 나온다 | CAN 인터페이스가 안 올라갔거나 배선 문제다. 1단계 확인 명령부터 다시 본다 |
| `ModuleNotFoundError: scservo_sdk` | venv 를 켠 채로 `python3 -m colcon build` 를 다시 돌린다 (5단계) |
| `could not open port /dev/ttyUSB0` | 포트 번호와 `dialout` 그룹을 확인한다 (4단계) |
| `check` 에서 일부 서보가 응답 없음 | 배선 순서·id 배정을 확인한다 (6단계) |
| 리더를 움직여도 팔로워가 그대로다 | 팔로워 `/joint_states` 를 못 받은 상태다. `ros2 control list_controllers` 로 `joint_state_broadcaster` 가 `active` 인지 본다 |
| 관절 하나가 리더보다 일찍 멈춘다 | `joint_limits_deg` clamp 다. 필요하면 그 관절의 범위를 넓힌다 |
