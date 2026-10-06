"""CAN 으로 직접 여는 팔로워 한 팔 — 관절 한계 측정·구동 테스트 공용.

모터 배치는 openarm_hardware 의 DEFAULT_MOTOR_TYPES·CAN ID 와 같다. 관절 7개가
송신 ID 0x01~0x07, 그리퍼가 0x08 이고 응답은 각각 0x10 을 더한 ID 로 온다.
bringup(ros2_control)이 같은 버스를 쓰고 있으면 둘이 충돌하므로 끄고 쓴다.

openarm_can Python binding 은 colcon 이 빌드하지 않는다. OpenArm PPA 의
python3-openarm-can 이 시스템 python 에 설치한다.
"""

import time

LABELS = ('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', '그리퍼')
SIDE_PREFIX = {'right': 'R-', 'left': 'L-'}

INSTALL_HINT = ('openarm_can Python binding 이 없습니다. 설치: '
                'sudo add-apt-repository ppa:openarm/main && '
                'sudo apt install python3-openarm-can')


def import_bindings():
    """openarm_can 모듈. 없으면 설치 안내와 함께 SystemExit."""
    try:
        import openarm_can
    except ImportError:
        raise SystemExit(f'  {INSTALL_HINT}') from None
    return openarm_can


def labels(side):
    """side 팔의 모터 이름, CAN ID 순서. 예: R-J1 … R-그리퍼."""
    return [SIDE_PREFIX[side] + label for label in LABELS]


def open_arm(oa, interface):
    """한 팔의 모터 여덟 개를 상태 콜백으로 연다. enable 은 보내지 않는다.

    인터페이스를 열 수 없으면(없거나 내려가 있으면) 안내와 함께 SystemExit.
    """
    motor = oa.MotorType
    try:
        openarm = oa.OpenArm(interface, True)
    except oa.CANSocketException as e:
        raise SystemExit(f'  {interface} 를 열 수 없습니다 ({e}). '
                         'scripts/canup.sh 로 인터페이스를 올렸는지 확인하세요.') from None
    openarm.init_arm_motors(
        [motor.DM8009, motor.DM8009, motor.DM4340, motor.DM4340,
         motor.DM4310, motor.DM4310, motor.DM4310],
        [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
        [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17])
    openarm.init_gripper_motor(motor.DM4310, 0x08, 0x18)
    openarm.set_callback_mode_all(oa.CallbackMode.STATE)
    return openarm


def positions(openarm):
    """마지막으로 받은 모터 위치 여덟 개(rad), CAN ID 순서."""
    motors = (list(openarm.get_arm().get_motors())
              + list(openarm.get_gripper().get_motors()))
    return [motor.get_position() for motor in motors]


def read(openarm):
    """모터를 켜지 않고 위치를 읽는다. 꺼진 모터도 상태 질의(refresh)에는 답한다."""
    openarm.refresh_all()
    time.sleep(0.02)
    openarm.recv_all()
    return positions(openarm)
