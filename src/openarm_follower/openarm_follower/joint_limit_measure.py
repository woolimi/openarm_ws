"""수동 관절 한계 측정 CLI — 모터를 끈 채 손으로 하드스톱에서 하드스톱까지.

    ros2 run openarm_follower joint_limit_measure --arm right --interface can0
    ros2 run openarm_follower joint_limit_measure --arm left --interface can1

모터에 enable 을 보내지 않고 상태 질의(refresh)만 보낸다. 꺼진 모터도 이 질의에는
위치를 돌려주므로, 팔을 손으로 자유롭게 움직이면서 관절마다 지나간 최솟값(하한)과
최댓값(상한)을 잴 수 있다. 양 끝 스톱에 한 번씩 닿게 움직이고 Enter 를 누르면 다음
관절로 넘어간다. 움직이지 않은 관절(응답하지 않는 모터 포함)은 폭 0 으로 표에 남는다.

위치는 모터 영점 기준이라 영점(openarm-can-cli set_zero)을 먼저 잡은 뒤에 재고,
영점을 다시 잡으면 다시 잰다. 하드웨어가 닫힌 그리퍼를 모터 0 rad 으로, 열린 그리퍼를
음(-)의 모터 각으로 보므로 그리퍼는 닫은 채로 영점을 잡는다. 그리퍼가 영점에서 다른
쪽으로 열리면 경고한다.

측정만 하고 아무 파일에도 쓰지 않는다. 끝에 잰 하한·상한을 URDF 의 같은 팔 한계와
나란히 표로 보여 준다.
"""

import argparse
import math
import select
import sys

from openarm_follower import arm_limits, can_arm
from openarm_follower.terminal import deg, pad, status

# DM 모터의 위치 보고가 ±12.5 rad 경계에서 반대편으로 넘어가면(wrap) 한 샘플에
# 약 25 rad 를 뛴다. 손으로는 한 샘플(수십 ms) 사이에 180° 를 돌릴 수 없으므로
# 그보다 큰 점프는 wrap 으로 보고, 그때까지 잰 하한·상한을 버리고 다시 잰다.
WRAP_JUMP_RAD = math.radians(180)
# 이보다 좁으면 스톱까지 움직이지 않았거나 모터가 응답하지 않은 것으로 보고 표에 표시한다.
MIN_WIDTH_RAD = math.radians(5)
# 잰 하한·상한이 URDF 한계와 이만큼 넘게 다르면 결과 표에 표시한다.
URDF_TOLERANCE_RAD = math.radians(5)
# 영점(0°)이 측정 범위 밖으로 이만큼 넘게 벗어나면 영점 세팅을 의심해 표시한다.
ZERO_TOLERANCE_RAD = math.radians(3)
# 화면 갱신 사이 키 입력 대기 시간.
SAMPLE_PERIOD_S = 0.05
# 그리퍼 한쪽 끝이 영점에서 이만큼 안이면 그 끝을 닫힘으로 본다. 손으로 닫고 영점을
# 잡으므로 정확히 0 은 아니다. 행정이 약 60° 라 열림 끝과 헷갈리지 않는다.
GRIPPER_CLOSED_TOLERANCE_RAD = math.radians(10)


class RangeTracker:
    """한 관절이 지나간 위치의 최솟값·최댓값."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.low = None
        self.high = None
        self.last = None
        self.jumped = False

    def feed(self, q):
        if not math.isfinite(q):
            return
        if self.last is not None and abs(q - self.last) > WRAP_JUMP_RAD:
            self.low = self.high = q
            self.jumped = True
        else:
            self.low = q if self.low is None else min(self.low, q)
            self.high = q if self.high is None else max(self.high, q)
        self.last = q

    @property
    def width(self):
        return 0.0 if self.low is None else self.high - self.low


def live_line(label, q, tracker, urdf_width):
    """측정 중인 관절의 한 줄 상태."""
    parts = [pad(label, 9) + f'현재 {math.degrees(q):+6.1f}°']
    if tracker.low is not None:
        parts.append(f'하한 {math.degrees(tracker.low):+6.1f}°')
        parts.append(f'상한 {math.degrees(tracker.high):+6.1f}°')
    width = f'폭 {math.degrees(tracker.width):5.1f}°'
    if urdf_width is not None:
        width += f'/URDF {math.degrees(urdf_width):.0f}°'
    parts.append(width)
    if tracker.jumped:
        parts.append('⚠ 위치 점프, 다시 재는 중')
    return '  ' + '  '.join(parts)


def result_rows(side, limits, urdf):
    """측정 결과 표. 잰 하한·상한 옆에 같은 팔의 URDF 한계를 둔다.

    관절은 URDF 와 5° 넘게 다른 끝, 영점이 범위 밖인 것, 거의 움직이지 않은 것을
    표시한다. 그리퍼의 여는 방향은 표 아래에서 따로 판정한다.
    """
    rows = ['  ' + pad('관절', 11) + pad('하한', 10) + pad('상한', 10)
            + pad('URDF 하한', 11) + 'URDF 상한']
    for label, joint in zip(can_arm.labels(side), arm_limits.JOINTS):
        low, high = limits[joint]
        urdf_low, urdf_high = urdf[joint]
        cells = (pad(label, 11) + pad(deg(low), 10) + pad(deg(high), 10)
                 + pad(deg(urdf_low), 11) + deg(urdf_high))
        notes = []
        if high - low < MIN_WIDTH_RAD:
            notes.append('거의 움직이지 않음')
        elif joint != 'gripper':
            for name, measured, described in (('하한', low, urdf_low), ('상한', high, urdf_high)):
                diff = measured - described
                if abs(diff) > URDF_TOLERANCE_RAD:
                    notes.append(f'{name} URDF 와 {math.degrees(diff):+.0f}°')
        if low > ZERO_TOLERANCE_RAD or high < -ZERO_TOLERANCE_RAD:
            notes.append('영점이 범위 밖')
        if notes:
            cells += '  ⚠ ' + ' · '.join(notes)
        rows.append('  ' + cells)
    return rows


def gripper_open_sign(low, high):
    """닫힘(영점) 끝에서 열림 끝으로 가는 부호 +1/-1. 어느 끝도 영점 근처가 아니면 None."""
    if abs(high) <= GRIPPER_CLOSED_TOLERANCE_RAD < abs(low):
        return -1
    if abs(low) <= GRIPPER_CLOSED_TOLERANCE_RAD < abs(high):
        return +1
    return None


def gripper_opened(low, high):
    """그리퍼를 닫힘(영점) 근처 밖으로 연 적이 있는가."""
    return max(abs(low), abs(high)) > GRIPPER_CLOSED_TOLERANCE_RAD


def gripper_direction_problem(low, high, described):
    """그리퍼 여는 방향의 문제. described 는 URDF 의 그리퍼 모터 한계(하한, 상한).

    하드웨어가 열림으로 보는 쪽은 described 에서 영점과 먼 끝이다. 문제가 없거나
    그리퍼를 열지 않아 판정할 방향이 없으면 None.
    """
    if not gripper_opened(low, high):
        return None
    sign = gripper_open_sign(low, high)
    if sign is None:
        return (f'그리퍼의 어느 끝도 영점에서 {math.degrees(GRIPPER_CLOSED_TOLERANCE_RAD):.0f}° '
                '안에 있지 않습니다. 그리퍼를 연 채로 영점을 잡았습니다. 그리퍼를 끝까지 '
                '닫고 영점 세팅을 다시 한 뒤 재세요.')
    want = -1 if abs(described[0]) > abs(described[1]) else +1
    if sign != want:
        return (f'그리퍼가 영점에서 {"+" if sign > 0 else "-"} 쪽으로만 움직였습니다. 하드웨어는 '
                f'닫힘을 0, 열림을 {"+" if want > 0 else "-"} 쪽으로 봅니다. 그리퍼를 연 채로 '
                '영점을 잡았거나 그리퍼 모터를 반대로 조립했습니다. 그리퍼를 끝까지 닫고 '
                '영점 세팅을 다시 한 뒤 재세요.')
    return None


def measure_joint(openarm, index, label, urdf_width):
    """index 번 모터의 (하한, 상한) rad. q 로 그만두면 None."""
    tracker = RangeTracker()
    print()
    print(f'  ── {label} ({index + 1}/{len(can_arm.LABELS)}) ──')
    print('  한쪽 스톱까지, 이어서 반대쪽 스톱까지 손으로 천천히 움직이세요.')
    print('  Enter 다음 관절 · r Enter 다시 재기 · q Enter 종료')
    while True:
        q = can_arm.read(openarm)[index]
        tracker.feed(q)
        status(live_line(label, q, tracker, urdf_width))
        ready, _, _ = select.select([sys.stdin], [], [], SAMPLE_PERIOD_S)
        if not ready:
            continue
        command = sys.stdin.readline().strip().lower()
        if command == 'q':
            return None
        if command == 'r':
            tracker.reset()
            print('  처음부터 다시 잽니다.')
            continue
        if command:
            print('  Enter · r · q 중 하나를 입력하세요.')
            continue
        if tracker.low is None:
            print('  위치를 아직 하나도 받지 못했습니다. 모터가 응답하는지 확인하세요.')
            continue
        return tracker.low, tracker.high


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='수동 관절 한계 측정 (모터를 켜지 않음 · 손으로 하드스톱에서 하드스톱까지 · '
                    '파일에 쓰지 않음)')
    parser.add_argument('--arm', required=True, choices=arm_limits.SIDES)
    parser.add_argument('-i', '--interface', required=True, help='그 팔의 CAN 인터페이스')
    args = parser.parse_args(argv)
    if not sys.stdin.isatty():
        parser.error('대화형 터미널에서 실행해야 합니다 (관절마다 Enter 로 넘어감).')

    urdf = arm_limits.side_limits(args.arm)
    oa = can_arm.import_bindings()
    openarm = can_arm.open_arm(oa, args.interface)

    labels = can_arm.labels(args.arm)
    print(f'  {args.interface} · {args.arm}: 모터를 켜지 않고 위치만 읽습니다.')
    print('  한계는 모터 영점 기준입니다. 영점 세팅을 마친 뒤에 재고, 영점을 다시 잡으면 다시 재세요.')
    limits = {}
    try:
        for index, (label, joint) in enumerate(zip(labels, arm_limits.JOINTS)):
            urdf_width = None if joint == 'gripper' else urdf[joint][1] - urdf[joint][0]
            measured = measure_joint(openarm, index, label, urdf_width)
            if measured is None:
                print('  종료합니다.')
                return 0
            limits[joint] = measured
    except KeyboardInterrupt:
        print('\n  종료합니다.')
        return 0

    print()
    print(f'  ── 측정 결과 ({args.arm}) ──')
    for row in result_rows(args.arm, limits, urdf):
        print(row)
    problem = gripper_direction_problem(*limits['gripper'], urdf['gripper'])
    if problem:
        print(f'  ⚠ {problem}')
        return 1
    if not gripper_opened(*limits['gripper']):
        print('  그리퍼를 열지 않아 여는 방향은 판정하지 않았습니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
