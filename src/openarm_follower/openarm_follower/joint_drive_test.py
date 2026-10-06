"""자동 관절 구동 테스트 CLI — URDF 관절 한계 안쪽까지 한 관절씩 천천히.

    ros2 run openarm_follower joint_drive_test --arm right --interface can0
    ros2 run openarm_follower joint_drive_test --arm left --interface can1

그 팔의 URDF 관절 한계(arm_limits, 왼팔은 거울상까지 전개된 값)를 읽어 관절마다 목표
두 곳을 정한다. 모터를 켠 뒤 한 관절씩 하한 쪽 목표, 상한 쪽 목표, 원점(0°) 순으로
최고 15°/s 로 움직이고(↑·↓ 로 5°/s 씩, 5~30°/s), 나머지 관절은 제자리를 붙잡는다.
한 관절이 원점으로 돌아온 뒤에 다음 관절로 넘어간다.

목표는 하드스톱을 누르지 않게 한계 안쪽에 둔다.
  - J1·J2: 영점 기준 한계각의 80%. 하한 -80°, 상한 +200° 이면 -64°, +160° 까지 간다.
    80% 지점이 한계에서 3° 보다 가까우면(오른팔 J2 의 -10° 쪽) 3° 안쪽까지만 간다.
  - J3~J7·그리퍼: 한계에서 3° 안쪽.
원점이 두 목표 사이에 없으면(하한 스톱이 0° 인 J4, 닫힘이 0° 인 그리퍼) 원점 대신
가까운 쪽 목표로 돌아와 스톱을 누르지 않는다.

모터를 켜기 직전에 자세를 다시 읽는다. 계획 표를 보는 동안 힘 빠진 팔이 처졌어도
켜는 순간 처음 읽은 자세로 끌려가지 않는다. 응답하지 않는 모터가 있거나 한계를 벗어난
관절이 있으면 모터를 켜지 않고 끝낸다.

멈추는 조건:
  - 어느 관절이든 명령과 실제 위치가 10° 넘게 벌어지면 무언가에 걸린 것으로 보고,
    움직이던 관절을 그 자리에 붙잡은 뒤 원점으로 돌아갈지 묻는다. 걸린 적이 있으면
    종료 코드 1 로 끝난다.
  - 한 제어 주기에 위치가 45° 넘게 뛰면 DM 모터의 ±12.5 rad wrap 이라 곧바로 모터를 끈다.
  - Ctrl-C 는 그 자리에서 붙잡고 원점으로 돌아갈지 묻는다.

키는 Enter 없이 한 글자로 받는다. 관절을 시작하기 전에는 Enter 시작 · s 건너뛰기 ·
q 끝내기이고, 움직이는 중에도 s 는 그 관절을 멈춰 원점으로 되돌린 뒤 다음 관절로,
q 는 그 자리에 붙잡고 원점으로 돌아갈지 묻는다. 방향키 ↑·↓ 는 언제든 최고 속도를
올리고 내린다. 움직이는 중에 바꿔도 가속도 제한을 따라 부드럽게 바뀐다.
끝날 때는 팔을 받치라고 안내한 뒤 모터를 끈다.
"""

import argparse
import contextlib
import math
import os
import select
import sys
import termios
import time
import tty

from openarm_follower import arm_limits, can_arm
from openarm_follower.terminal import deg, pad, status

# J1·J2 는 팔 전체를 드는 어깨 관절이라 한계각의 80% 까지만 간다. 한계가 영점에 가까운
# 쪽(오른팔 J2 의 -10°)은 80% 지점도 한계에 가까워 STOP_MARGIN_RAD 를 같이 지킨다.
SHOULDER_JOINTS = (0, 1)
SHOULDER_SCALE = 0.8
STOP_MARGIN_RAD = math.radians(3)
SPEED_RAD_S = math.radians(15)          # 최고 속도 시작값
SPEED_STEP_RAD_S = math.radians(5)      # ↑·↓ 한 번에 바뀌는 양
MIN_SPEED_RAD_S = math.radians(5)
MAX_SPEED_RAD_S = math.radians(30)
# 가감속 한계. 15°/s 까지 0.5 s 에 오르고, 목표 앞에서는 이 감속으로 멈춘다.
ACCEL_RAD_S2 = math.radians(30)
PAUSE_AT_TARGET_S = 1.0
CONTROL_PERIOD_S = 0.005
TRACKING_LIMIT_RAD = math.radians(10)
WRAP_STEP_RAD = math.radians(45)
# 출발 자세가 URDF 한계를 이만큼 넘게 벗어나면 영점이 어긋났다고 보고 시작하지 않는다.
POSE_TOLERANCE_RAD = math.radians(5)

# 중력 토크 앞먹임 없이 위치 오차만으로 자세를 버티는 MIT 게인. J1·J2(DM8009)는
# kp 300 Nm/rad 이라 10 Nm 를 버틸 때 오차가 약 1.9° 다.
ARM_KP = (300.0, 300.0, 150.0, 150.0, 40.0, 40.0, 30.0)
ARM_KD = (2.5, 2.5, 2.5, 2.5, 0.8, 0.8, 0.8)
GRIPPER_KP = 10.0
GRIPPER_KD = 0.9


class Snag(Exception):
    """관절이 명령을 따라오지 못했다. 무언가에 걸렸다."""


class PositionWrap(Exception):
    """위치 보고가 한 주기에 물리적으로 불가능하게 뛰었다."""


class Skip(Exception):
    """움직이는 중에 s 를 눌렀다. 이 관절을 멈추고 원점으로 되돌린 뒤 다음 관절로 간다."""


class Quit(Exception):
    """움직이는 중에 q 를 눌렀다. 그 자리에 붙잡고 끝낸다."""


def targets(index, lower, upper):
    """관절 index 의 (하한 쪽 목표, 상한 쪽 목표). 목표를 정할 수 없으면 ValueError."""
    if index in SHOULDER_JOINTS:
        if not lower < 0.0 < upper:
            raise ValueError(f'영점(0°)이 한계 [{deg(lower)}, {deg(upper)}] 안에 없다')
        return (max(lower * SHOULDER_SCALE, lower + STOP_MARGIN_RAD),
                min(upper * SHOULDER_SCALE, upper - STOP_MARGIN_RAD))
    low, high = lower + STOP_MARGIN_RAD, upper - STOP_MARGIN_RAD
    if not low < high:
        raise ValueError(
            f'한계 폭 {math.degrees(upper - lower):.1f}° 가 양쪽 여유 '
            f'{math.degrees(2 * STOP_MARGIN_RAD):.0f}° 보다 좁다')
    return low, high


def origin_goal(goal_low, goal_high):
    """한 관절을 마친 뒤 돌아갈 위치. 원점(0°)을 두 목표 사이로 자른 값이다."""
    return min(max(0.0, goal_low), goal_high)


def pose_outside(q, limits):
    """한계를 POSE_TOLERANCE_RAD 넘게 벗어난 관절의 인덱스."""
    return [i for i, (qi, (lower, upper)) in enumerate(zip(q, limits))
            if qi < lower - POSE_TOLERANCE_RAD or qi > upper + POSE_TOLERANCE_RAD]


def ramp(q, goal, v, speed, dt):
    """q 를 goal 쪽으로 한 주기(dt) 옮긴 (새 위치, 새 속력).

    속력은 ACCEL_RAD_S2 로만 오르내리며 speed 를 따라가고, 남은 거리에서 그
    감속으로 멈출 수 있는 속력을 넘지 않는다. speed 가 도중에 바뀌어도 이어진다.
    """
    distance = abs(goal - q)
    if distance <= 1e-9:
        return goal, 0.0
    change = ACCEL_RAD_S2 * dt
    v = min(v + change, max(v - change, speed))
    v = min(v, math.sqrt(2.0 * ACCEL_RAD_S2 * distance))
    travel = v * dt
    if travel >= distance:
        return goal, 0.0
    return q + math.copysign(travel, goal - q), v


def plan_rows(labels, limits, plan):
    rows = ['  ' + pad('관절', 11) + pad('하한', 10) + pad('상한', 10)
            + pad('하한 쪽 목표', 14) + pad('상한 쪽 목표', 14) + '복귀']
    for label, (lower, upper), (goal_low, goal_high) in zip(labels, limits, plan):
        rows.append('  ' + pad(label, 11) + pad(deg(lower), 10) + pad(deg(upper), 10)
                    + pad(deg(goal_low), 14) + pad(deg(goal_high), 14)
                    + deg(origin_goal(goal_low, goal_high)))
    return rows


class Keys:
    """Enter 없이 한 키씩 읽는 입력. Enter 는 '', 방향키 ↑·↓ 는 'up'·'down'.

    터미널을 cbreak 로 두어 줄 단위 버퍼링을 끄고, Ctrl-C(SIGINT)는 그대로 둔다.
    tty 가 아니면(테스트) 모드를 바꾸지 않는다. 방향키는 ESC [ A/B 세 바이트라
    Python 버퍼를 거치지 않고 파일 디스크립터에서 곧바로 읽어 한 키로 묶는다.
    """

    ARROWS = {'\x1b[A': 'up', '\x1b[B': 'down', '\x1bOA': 'up', '\x1bOB': 'down'}

    def __init__(self, stream=None):
        self._stream = sys.stdin if stream is None else stream
        self._pending = []

    @contextlib.contextmanager
    def cbreak(self):
        if not self._stream.isatty():
            yield self
            return
        fd = self._stream.fileno()
        saved = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            yield self
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)

    def poll(self, timeout=0.0):
        """눌린 키 하나. 글자는 소문자, Enter 는 '', ↑·↓ 는 'up'·'down', 없으면 None."""
        if not self._pending:
            ready, _, _ = select.select([self._stream], [], [], timeout)
            if not ready:
                return None
            self._pending.extend(self.split(os.read(self._stream.fileno(), 64)))
        return self._pending.pop(0) if self._pending else None

    @classmethod
    def split(cls, data):
        """한 번에 읽은 바이트를 키 목록으로."""
        text = data.decode(errors='ignore')
        keys, i = [], 0
        while i < len(text):
            arrow = next((seq for seq in cls.ARROWS if text.startswith(seq, i)), None)
            if arrow:
                keys.append(cls.ARROWS[arrow])
                i += len(arrow)
                continue
            char = text[i]
            if char == '\x1b':          # 다른 이스케이프 시퀀스는 통째로 버린다
                i += 3
                continue
            keys.append('' if char in ('\n', '\r') else char.lower())
            i += 1
        return keys


class Arm:
    """한 팔의 모터 여덟 개. 명령 위치 q_cmd 를 들고 매 주기 전부에 MIT 명령을 보낸다.

    snags 는 관절이 명령을 따라오지 못한(Snag) 횟수다.
    """

    def __init__(self, openarm, oa, keys=None):
        self._openarm = openarm
        self._oa = oa
        self._keys = keys
        self.speed = SPEED_RAD_S
        self.q_cmd = None
        self._q_prev = None
        self.snags = 0

    def positions(self):
        return can_arm.positions(self._openarm)

    def read(self):
        """모터를 켜지 않고 위치를 읽는다."""
        return can_arm.read(self._openarm)

    def silent(self):
        """아직 응답하지 않은 모터의 인덱스."""
        return can_arm.silent(self._openarm)

    def enable(self, q):
        """q 를 붙잡을 위치로 두고 모터를 켠다."""
        self.q_cmd = list(q)
        self._q_prev = list(q)
        self._openarm.enable_all()
        self.step()

    def disable(self):
        self._openarm.disable_all()
        self._openarm.recv_all()

    def step(self):
        """q_cmd 를 한 번 보내고 새 위치를 돌려준다. wrap 이면 PositionWrap."""
        oa = self._oa
        self._openarm.get_arm().mit_control_all(
            [oa.MITParam(kp, kd, q, 0.0, 0.0)
             for kp, kd, q in zip(ARM_KP, ARM_KD, self.q_cmd[:7])])
        self._openarm.get_gripper().mit_control_all(
            [oa.MITParam(GRIPPER_KP, GRIPPER_KD, self.q_cmd[7], 0.0, 0.0)])
        self._openarm.recv_all()
        q = self.positions()
        jump = max(abs(a - b) for a, b in zip(q, self._q_prev))
        if jump > WRAP_STEP_RAD:
            raise PositionWrap(
                f'한 주기에 위치가 {math.degrees(jump):.0f}° 뛰었다 (DM ±12.5 rad wrap)')
        self._q_prev = q
        return q

    def hold(self, seconds, index=None):
        """seconds 동안 자세를 붙잡는다. index 관절을 움직이는 중이면 s·q 를 받는다."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            q = self.step()
            if index is not None:
                self._check_keys(index, q)
            time.sleep(CONTROL_PERIOD_S)

    def change_speed(self, key):
        """↑·↓ 로 최고 속도를 한 칸 올리고 내린다. 속도 키였으면 True."""
        if key not in ('up', 'down'):
            return False
        step = SPEED_STEP_RAD_S if key == 'up' else -SPEED_STEP_RAD_S
        self.speed = min(MAX_SPEED_RAD_S, max(MIN_SPEED_RAD_S, self.speed + step))
        return True

    def _check_keys(self, index, q, stoppable=True):
        if self._keys is None:
            return
        key = self._keys.poll()
        if self.change_speed(key):
            return
        if stoppable and key in ('s', 'q'):
            self.q_cmd[index] = q[index]
            raise Skip() if key == 's' else Quit()

    def move(self, index, goal, labels, keys=True):
        """관절 index 를 goal 까지. 걸리면 그 관절을 실제 위치에 붙잡고 Snag.

        최고 속도는 self.speed 를 매 주기 다시 읽어 ↑·↓ 가 곧바로 먹는다. keys 가
        참이면 움직이는 중 s 는 Skip, q 는 Quit 이고, 둘 다 그 관절을 실제 위치에
        붙잡은 뒤 올린다.
        """
        velocity = 0.0
        last = time.monotonic()
        while True:
            now = time.monotonic()
            self.q_cmd[index], velocity = ramp(
                self.q_cmd[index], goal, velocity, self.speed, now - last)
            last = now
            q = self.step()
            worst = max(range(len(q)), key=lambda i: abs(q[i] - self.q_cmd[i]))
            error = q[worst] - self.q_cmd[worst]
            if abs(error) > TRACKING_LIMIT_RAD:
                self.q_cmd[index] = q[index]
                self.snags += 1
                raise Snag(f'{labels[worst]} 의 실제 위치가 명령과 {math.degrees(error):+.1f}° 어긋났다')
            self._check_keys(index, q, stoppable=keys)
            status(f'  {labels[index]}  목표 {math.degrees(goal):+6.1f}°'
                   f'  명령 {math.degrees(self.q_cmd[index]):+6.1f}°'
                   f'  실제 {math.degrees(q[index]):+6.1f}°'
                   f'  속도 {math.degrees(self.speed):.0f}°/s (↑↓)')
            if self.q_cmd[index] == goal:
                break
            time.sleep(CONTROL_PERIOD_S)
        sys.stdout.write('\n')

    def ask(self, prompt, choices):
        """자세를 붙잡은 채 choices 중 한 키를 기다린다. Enter 는 ''."""
        sys.stdout.write(prompt)
        sys.stdout.flush()
        while True:
            self.step()
            key = self._keys.poll(CONTROL_PERIOD_S)
            if self.change_speed(key):
                status(prompt.lstrip('\n') + f'[속도 {math.degrees(self.speed):.0f}°/s] ')
                continue
            if key in choices:
                sys.stdout.write('\n')
                return key


def start_pose(arm, labels, limits):
    """모터를 켜지 않고 읽은 지금 자세.

    응답하지 않은 모터가 있거나 한계를 POSE_TOLERANCE_RAD 넘게 벗어난 관절이 있으면
    이유를 찍고 None.
    """
    q = arm.read()
    silent = arm.silent()
    if silent:
        print(f'  응답하지 않는 모터: {", ".join(labels[i] for i in silent)}. '
              '모터 점검으로 배선과 전원을 확인하세요.')
        return None
    outside = pose_outside(q, limits)
    if outside:
        for i in outside:
            print(f'  {labels[i]}: 현재 {deg(q[i])} 가 URDF 한계 '
                  f'[{deg(limits[i][0])}, {deg(limits[i][1])}] 밖입니다.')
        print('  영점이 어긋났거나 URDF 한계가 실물과 다릅니다. 영점 세팅과 수동 관절 한계 측정으로 확인하세요.')
        return None
    return q


def _return_to_origin(arm, index, origin, labels):
    """붙잡은 관절을 원점으로 되돌릴지 묻는다. 되돌리고 계속하면 True."""
    answer = arm.ask(
        f'  r {labels[index]} 를 원점({deg(origin)})으로 복귀 · q 끝내기: ', ('r', 'q'))
    if answer != 'r':
        return False
    return _go_origin(arm, index, origin, labels)


def _go_origin(arm, index, origin, labels):
    """index 관절을 원점으로. 복귀가 막히면 False."""
    try:
        arm.move(index, origin, labels, keys=False)
    except Snag as e:
        print(f'\n  ⚠ {e}. 복귀도 막혀 여기서 끝냅니다.')
        return False
    return True


def run(arm, plan, labels):
    """관절마다 하한 쪽 목표 → 상한 쪽 목표 → 원점. 끝나면 팔을 받치게 한다."""
    moving = None
    try:
        for index, (goal_low, goal_high) in enumerate(plan):
            origin = origin_goal(goal_low, goal_high)
            answer = arm.ask(
                f'\n  {labels[index]}: {deg(goal_low)} → {deg(goal_high)} → {deg(origin)}'
                ' · Enter 시작 · s 건너뛰기 · q 끝내기 (움직이는 중에도 s·q): ', ('', 's', 'q'))
            if answer == 'q':
                break
            if answer == 's':
                continue
            moving = (index, origin)
            try:
                for goal in (goal_low, goal_high, origin):
                    arm.move(index, goal, labels)
                    arm.hold(PAUSE_AT_TARGET_S, index)
                print(f'  {labels[index]} 완료, 원점으로 돌아왔습니다.')
            except Skip:
                print(f'\n  {labels[index]} 건너뜀. 원점으로 돌아갑니다.')
                if not _go_origin(arm, *moving, labels):
                    break
            except Quit:
                print(f'\n  멈췄습니다. {labels[index]} 를 그 자리에 붙잡았습니다.')
                _return_to_origin(arm, *moving, labels)
                break
            except Snag as e:
                print(f'\n  ⚠ {e}. {labels[index]} 를 그 자리에 붙잡았습니다.')
                if not _return_to_origin(arm, *moving, labels):
                    break
            moving = None
    except KeyboardInterrupt:
        print('\n  중단했습니다. 그 자리에서 붙잡습니다.')
        if moving is not None:
            arm.q_cmd[moving[0]] = arm.positions()[moving[0]]
            try:
                _return_to_origin(arm, *moving, labels)
            except KeyboardInterrupt:
                pass
    try:
        arm.ask('\n  모터를 끄면 팔이 중력으로 처집니다. 팔을 받친 뒤 Enter: ', ('',))
    except KeyboardInterrupt:
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='자동 관절 구동 테스트 (URDF 관절 한계 안쪽까지 한 관절씩 천천히)')
    parser.add_argument('--arm', required=True, choices=arm_limits.SIDES)
    parser.add_argument('-i', '--interface', required=True, help='그 팔의 CAN 인터페이스')
    args = parser.parse_args(argv)
    if not sys.stdin.isatty():
        parser.error('대화형 터미널에서 실행해야 합니다 (관절마다 Enter 로 시작).')

    try:
        described = arm_limits.side_limits(args.arm)
    except ValueError as e:
        print(f'  {e}')
        return 1
    limits = [described[joint] for joint in arm_limits.JOINTS]
    labels = can_arm.labels(args.arm)
    plan = []
    for index, (lower, upper) in enumerate(limits):
        try:
            plan.append(targets(index, lower, upper))
        except ValueError as e:
            print(f'  {labels[index]}: URDF 한계에서 {e}.')
            return 1

    oa = can_arm.import_bindings()
    keys = Keys()
    arm = Arm(can_arm.open_arm(oa, args.interface), oa, keys)
    if start_pose(arm, labels, limits) is None:
        return 1

    print()
    print(f'  ── 구동 계획 ({args.interface} · {args.arm}) ──')
    for row in plan_rows(labels, limits, plan):
        print(row)
    print()
    print(f'  ⚠ 모터를 켜고 관절을 하나씩 위 목표까지 최고 {math.degrees(SPEED_RAD_S):.0f}°/s 로 움직입니다.')
    print('    팔 주변을 비우세요. 관절마다 Enter 로 시작하고, 움직이는 중에는 s 건너뛰기 · q 멈추기 ·')
    print(f'    ↑·↓ 속도 {math.degrees(SPEED_STEP_RAD_S):.0f}°/s 씩'
          f'({math.degrees(MIN_SPEED_RAD_S):.0f}~{math.degrees(MAX_SPEED_RAD_S):.0f}°/s) · Ctrl-C 멈추기.')
    try:
        input('  모터를 켜려면 Enter (취소: Ctrl-C): ')
    except KeyboardInterrupt:
        print('\n  취소했습니다. 모터는 켜지 않았습니다.')
        return 0
    # 표를 보는 동안 팔이 움직였을 수 있다. 처음 읽은 자세로 켜면 그 자세로 끌려간다.
    q_start = start_pose(arm, labels, limits)
    if q_start is None:
        print('  모터는 켜지 않았습니다.')
        return 1

    rc = 0
    try:
        with keys.cbreak():
            arm.enable(q_start)
            run(arm, plan, labels)
    except PositionWrap as e:
        print(f'\n  ⚠ {e}. 모터를 끕니다.')
        rc = 1
    finally:
        arm.disable()
        print('  모터를 껐습니다.')
    if arm.snags:
        print(f'  관절이 명령을 따라오지 못한 적이 {arm.snags}번 있어 실패로 끝냅니다.')
        rc = 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
