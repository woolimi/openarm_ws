"""관절 한계 측정·관절 구동 테스트 CLI 의 로직 테스트.

binding(openarm_can)은 main() 안에서만 import 하므로 여기서는 필요 없다. 구동
테스트의 제어 루프는 명령을 그대로 따라가거나 한 관절이 걸리는 가짜 팔로 검증한다.
"""

import math
import types

from openarm_follower import arm_limits, can_arm
from openarm_follower import joint_drive_test as drive
from openarm_follower import joint_limit_measure as measure
from openarm_follower.terminal import cols, fit

import pytest


@pytest.fixture(scope='module')
def urdf_xml():
    return arm_limits.urdf_xml()


# ── URDF 관절 한계 ─────────────────────────────────────────────────────────


def test_limits_mirror_the_shoulder_between_the_arms(urdf_xml):
    right = arm_limits.side_limits('right', urdf_xml)
    left = arm_limits.side_limits('left', urdf_xml)
    assert set(right) == set(arm_limits.JOINTS)
    deg = [round(math.degrees(v)) for v in right['joint1'] + left['joint1']]
    assert deg == [-80, 200, -200, 80]
    assert right['joint4'] == left['joint4'] == pytest.approx((0.0, math.radians(140)))


def test_the_gripper_closes_at_zero_and_opens_below_it(urdf_xml):
    for side in arm_limits.SIDES:
        low, high = arm_limits.side_limits(side, urdf_xml)['gripper']
        assert (low, high) == pytest.approx((-1.0472, 0.0))


def test_a_joint_without_a_limit_is_refused():
    with pytest.raises(ValueError, match='openarm_right_joint1'):
        arm_limits.side_limits('right', '<robot name="r"/>')


def test_an_unknown_arm_is_refused():
    with pytest.raises(ValueError, match='middle'):
        arm_limits.side_limits('middle', '<robot name="r"/>')


def test_every_described_joint_has_a_drive_plan(urdf_xml):
    """URDF 한계 그대로 두 팔 모두 구동 계획이 선다. 그리퍼는 닫힘 스톱 3° 앞으로 돌아온다."""
    for side in arm_limits.SIDES:
        limits = arm_limits.side_limits(side, urdf_xml)
        plan = [drive.targets(i, *limits[joint]) for i, joint in enumerate(arm_limits.JOINTS)]
        assert math.degrees(drive.origin_goal(*plan[7])) == pytest.approx(-3.0)


def test_the_drive_test_opens_the_gripper_the_way_the_measure_wants(urdf_xml):
    """구동 테스트가 그리퍼를 여는 쪽과 측정이 열림으로 판정하는 쪽이 같다."""
    described = arm_limits.side_limits('right', urdf_xml)['gripper']
    goal_low, goal_high = drive.targets(7, *described)
    assert measure.gripper_direction_problem(goal_low, goal_high, described) is None


# ── 수동 관절 한계 측정 ─────────────────────────────────────────────────────


def test_tracker_follows_the_extremes():
    tracker = measure.RangeTracker()
    for q in (0.0, -0.5, 0.2, 1.0, 2.5, 0.4):
        tracker.feed(q)
    assert (tracker.low, tracker.high) == (-0.5, 2.5)
    assert tracker.width == pytest.approx(3.0)


def test_tracker_restarts_on_a_wrap_jump():
    tracker = measure.RangeTracker()
    for q in (0.0, 0.5, 0.5 - 25.0):
        tracker.feed(q)
    assert tracker.jumped and tracker.width == 0.0
    tracker.feed(-24.0)
    assert tracker.width == pytest.approx(0.5)


def test_tracker_ignores_unreadable_positions():
    tracker = measure.RangeTracker()
    tracker.feed(float('nan'))
    assert tracker.low is None and tracker.width == 0.0


@pytest.mark.parametrize('columns', [40, 60, 80])
def test_status_lines_never_wrap_the_terminal(columns):
    """상태 줄이 터미널 폭 이상이면 접혀서 갱신마다 화면이 밀려 올라간다."""
    tracker = measure.RangeTracker()
    for q in (-1.2, 3.4):
        tracker.feed(q)
    tracker.jumped = True
    line = measure.live_line('R-그리퍼', 3.4, tracker, math.radians(280))
    assert cols(fit(line, columns)) < columns
    assert fit('abc', 80) == 'abc'


def test_live_line_fits_an_80_column_terminal_untruncated():
    tracker = measure.RangeTracker()
    for d in range(-179, 200, 10):          # 손으로 쓸 듯 조금씩, wrap 판정에 안 걸린다
        tracker.feed(math.radians(d + 0.9))
    assert not tracker.jumped
    line = measure.live_line('R-그리퍼', math.radians(-179.9), tracker, math.radians(280))
    assert cols(line) < 80, line


def test_result_rows_put_the_urdf_beside_each_joint():
    rad = math.radians
    limits = {joint: (rad(-50), rad(50)) for joint in arm_limits.JOINTS}
    limits['joint2'] = (rad(-60), rad(52))          # 하한만 URDF 와 10° 차이
    limits['joint3'] = (rad(10), rad(110))          # 영점이 범위 밖
    limits['joint5'] = (rad(1), rad(2))             # 움직이지 않았다
    limits['gripper'] = (rad(-58), rad(1))          # 그리퍼는 URDF 와 비교하지 않는다
    urdf = {joint: (rad(-50), rad(50)) for joint in arm_limits.JOINTS}
    urdf['joint1'] = (rad(-40), rad(60))
    urdf['joint3'] = (rad(10), rad(110))
    urdf['gripper'] = (rad(-60), 0.0)

    rows = measure.result_rows('right', limits, urdf)
    assert rows[1].split()[:5] == ['R-J1', '-50.0°', '+50.0°', '-40.0°', '+60.0°']
    assert '하한 URDF 와 -10°' in rows[1] and '상한 URDF 와 -10°' in rows[1]
    assert '하한 URDF 와 -10°' in rows[2] and '상한' not in rows[2].split('⚠')[1]
    assert '영점이 범위 밖' in rows[3]
    assert '⚠' not in rows[4]
    assert '거의 움직이지 않음' in rows[5] and 'URDF 와' not in rows[5]
    assert rows[-1].split()[0] == 'R-그리퍼' and '⚠' not in rows[-1]


def test_the_rows_name_the_left_arm():
    zero = {joint: (0.0, 0.0) for joint in arm_limits.JOINTS}
    rows = measure.result_rows('left', zero, zero)
    assert [row.split()[0] for row in rows[1:]] == can_arm.labels('left')


# ── 수동 관절 한계 측정: 그리퍼 여는 방향 ───────────────────────────────────
# URDF 의 그리퍼 모터 한계는 (-60°, 0°) 다. 닫힘에서 영점을 잡으면 손으로 연 그리퍼는
# 음(-)의 쪽으로 간다.

_GRIPPER = (-1.0472, 0.0)


@pytest.mark.parametrize('low, high, want', [
    (-1.02, 0.01, -1),     # 닫힘 영점에서 음의 쪽으로 열림
    (-0.03, 1.05, +1),     # 닫힘 영점에서 양의 쪽으로 열림
])
def test_gripper_open_sign_is_the_far_end_past_a_closed_zero(low, high, want):
    assert measure.gripper_open_sign(low, high) == want


@pytest.mark.parametrize('low, high', [(-0.50, 0.55), (0.3, 1.2)])
def test_gripper_open_sign_is_unknown_when_the_zero_is_not_the_closed_end(low, high):
    assert measure.gripper_open_sign(low, high) is None


def test_a_gripper_opening_below_zero_passes():
    assert measure.gripper_direction_problem(-1.03, 0.02, _GRIPPER) is None


def test_a_gripper_moving_above_zero_is_refused():
    """연 채로 영점을 잡았거나 모터가 반대로 달려 있으면 영점에서 + 쪽으로만 움직인다."""
    problem = measure.gripper_direction_problem(-0.02, 1.03, _GRIPPER)
    assert problem is not None and '+ 쪽' in problem


def test_a_gripper_zeroed_halfway_is_refused():
    problem = measure.gripper_direction_problem(-0.50, 0.55, _GRIPPER)
    assert problem is not None and '연 채로' in problem


def test_a_gripper_never_opened_has_no_direction_to_judge():
    assert not measure.gripper_opened(-0.02, 0.03)
    assert measure.gripper_direction_problem(-0.02, 0.03, _GRIPPER) is None


# ── 관절 구동 테스트: 목표·계획 ─────────────────────────────────────────────


def test_shoulder_targets_are_80_percent_of_the_limits():
    low, high = drive.targets(0, math.radians(-80), math.radians(200))
    assert (math.degrees(low), math.degrees(high)) == pytest.approx((-64.0, 160.0))
    low, high = drive.targets(1, math.radians(-10), math.radians(190))
    assert (math.degrees(low), math.degrees(high)) == pytest.approx((-8.0, 152.0))


def test_other_targets_stop_3_degrees_short():
    for index in range(2, 8):
        low, high = drive.targets(index, math.radians(-90), math.radians(90))
        assert (math.degrees(low), math.degrees(high)) == pytest.approx((-87.0, 87.0))


def test_each_joint_returns_to_the_origin_between_its_targets():
    assert drive.origin_goal(math.radians(-64), math.radians(160)) == 0.0
    # J4 는 하한 스톱이 0° 라 하한 쪽 목표가 +3°. 원점 대신 그 목표로 돌아온다.
    low, high = drive.targets(3, 0.0, math.radians(140))
    assert drive.origin_goal(low, high) == pytest.approx(math.radians(3))


def test_shoulder_needs_zero_inside_its_limits():
    with pytest.raises(ValueError, match='영점'):
        drive.targets(0, math.radians(5), math.radians(200))


def test_a_range_narrower_than_both_margins_refuses():
    with pytest.raises(ValueError, match='좁다'):
        drive.targets(4, math.radians(-2), math.radians(3))


def test_pose_outside_names_joints_past_the_limits():
    limits = [(-1.0, 1.0)] * 8
    q = [0.0] * 8
    q[2] = 1.0 + math.radians(4)       # 허용 안
    q[5] = -1.0 - math.radians(6)      # 허용 밖
    assert drive.pose_outside(q, limits) == [5]


def test_the_drive_speed_starts_at_15_and_steps_by_5_within_5_to_30():
    arm = drive.Arm(None, None)
    assert math.degrees(arm.speed) == pytest.approx(15.0)
    for _ in range(10):
        arm.change_speed('up')
    assert math.degrees(arm.speed) == pytest.approx(30.0)
    for _ in range(10):
        arm.change_speed('down')
    assert math.degrees(arm.speed) == pytest.approx(5.0)
    assert not arm.change_speed('s')


def test_keys_split_arrows_letters_and_enter():
    assert drive.Keys.split(b'\x1b[As\x1b[B\nQ\x1bOA\x1b[C') == ['up', 's', 'down', '', 'q', 'up']


def _ramp_run(goal, speed, dt=1e-3):
    q, v, t, peak = 0.0, 0.0, 0.0, 0.0
    while q != goal:
        q, v = drive.ramp(q, goal, v, speed, dt)
        t += dt
        peak = max(peak, v)
        assert t < 100.0
    return t, peak


def test_ramp_reaches_the_goal_at_the_set_peak_speed():
    t, peak = _ramp_run(1.0, 0.5)
    assert peak == pytest.approx(0.5, rel=1e-3)
    accel = drive.ACCEL_RAD_S2
    assert t == pytest.approx(1.0 / 0.5 + 0.5 / accel, rel=1e-2)   # 사다리꼴 시간


def test_ramp_follows_a_speed_change_without_a_jump():
    """도중에 속도를 바꿔도 한 주기 속력 변화가 가속도 한계를 넘지 않는다."""
    dt, accel = 1e-3, drive.ACCEL_RAD_S2
    q, v, t = 0.0, 0.0, 0.0
    while q != 2.0:
        speed = math.radians(30) if t < 1.5 else math.radians(5)
        q_next, v_next = drive.ramp(q, 2.0, v, speed, dt)
        if 2.0 - q_next > 0.01:        # 멈추기 직전 몇 주기는 sqrt 감속의 이산화 꼬리
            assert abs(v_next - v) <= accel * dt + 1e-12
        q, v, t = q_next, v_next, t + dt
        assert t < 100.0


# ── 관절 구동 테스트: 제어 루프 ─────────────────────────────────────────────


class _FakeMotor:
    def __init__(self, q):
        self.q = q

    def get_position(self):
        return self.q


class _FakeGroup:
    def __init__(self, motors, stuck):
        self._motors = motors
        self._stuck = stuck

    def get_motors(self):
        return self._motors

    def mit_control_all(self, params):
        for i, (motor, param) in enumerate(zip(self._motors, params)):
            if i not in self._stuck:
                motor.q = param[2]


class _FakeOpenArm:
    """명령 위치를 그대로 따라가는 팔. stuck 관절은 제자리에 머문다."""

    def __init__(self, q, stuck=()):
        motors = [_FakeMotor(v) for v in q]
        self._arm = _FakeGroup(motors[:7], set(stuck))
        self._gripper = _FakeGroup(motors[7:], {i - 7 for i in stuck})
        self.enabled = False

    def get_arm(self):
        return self._arm

    def get_gripper(self):
        return self._gripper

    def recv_all(self):
        pass

    def refresh_all(self):
        pass

    def enable_all(self):
        self.enabled = True

    def disable_all(self):
        self.enabled = False


_OA = types.SimpleNamespace(MITParam=lambda kp, kd, q, dq, tau: (kp, kd, q, dq, tau))
_LABELS = can_arm.labels('right')


class _ScriptedKeys:
    """poll 마다 정해 둔 키를 하나씩 돌려준다."""

    def __init__(self, keys):
        self._keys = list(keys)

    def poll(self, timeout=0.0):
        return self._keys.pop(0) if self._keys else None


@pytest.fixture
def fast(monkeypatch):
    monkeypatch.setattr(drive, 'SPEED_RAD_S', math.radians(400))
    monkeypatch.setattr(drive, 'ACCEL_RAD_S2', math.radians(40000))
    monkeypatch.setattr(drive, 'CONTROL_PERIOD_S', 0.0)
    return drive


def test_move_follows_and_holds_the_other_joints(fast):
    openarm = _FakeOpenArm([0.1] * 8)
    arm = fast.Arm(openarm, _OA)
    arm.enable(arm.read())
    arm.move(2, math.radians(40), _LABELS)
    assert openarm.enabled
    assert arm.q_cmd[2] == pytest.approx(math.radians(40))
    assert arm.positions()[2] == pytest.approx(math.radians(40))
    assert [q for i, q in enumerate(arm.positions()) if i != 2] == [0.1] * 7


def test_a_snagged_joint_stops_where_it_is(fast):
    openarm = _FakeOpenArm([0.0] * 8, stuck=(3,))
    arm = fast.Arm(openarm, _OA)
    arm.enable(arm.read())
    with pytest.raises(fast.Snag, match='R-J4'):
        arm.move(3, math.radians(60), _LABELS)
    assert arm.q_cmd[3] == 0.0


def test_a_position_wrap_is_refused(fast):
    openarm = _FakeOpenArm([0.0] * 8)
    arm = fast.Arm(openarm, _OA)
    arm.enable(arm.read())
    openarm.get_arm().get_motors()[0].q = 25.0
    openarm._arm._stuck = {0}
    with pytest.raises(fast.PositionWrap):
        arm.step()


@pytest.mark.parametrize('key, error', [('s', 'Skip'), ('q', 'Quit')])
def test_s_and_q_stop_the_moving_joint_where_it_is(fast, key, error):
    openarm = _FakeOpenArm([0.0] * 8)
    arm = fast.Arm(openarm, _OA, _ScriptedKeys([None, None, key]))
    arm.enable(arm.read())
    with pytest.raises(getattr(fast, error)):
        arm.move(2, math.radians(40), _LABELS)
    assert arm.q_cmd[2] == arm.positions()[2]
    assert 0.0 < arm.q_cmd[2] < math.radians(40)


def test_arrow_keys_change_the_speed_while_moving(fast):
    openarm = _FakeOpenArm([0.0] * 8)
    arm = fast.Arm(openarm, _OA, _ScriptedKeys(['down', 'down']))
    arm.speed = math.radians(15)
    arm.enable(arm.read())
    arm.move(2, math.radians(1), _LABELS)
    assert math.degrees(arm.speed) == pytest.approx(5.0)
    assert arm.q_cmd[2] == pytest.approx(math.radians(1))


def test_the_return_to_origin_ignores_s_and_q(fast):
    openarm = _FakeOpenArm([0.0] * 8)
    arm = fast.Arm(openarm, _OA, _ScriptedKeys(['q', 's']))
    arm.enable(arm.read())
    arm.move(2, math.radians(40), _LABELS, keys=False)
    assert arm.q_cmd[2] == pytest.approx(math.radians(40))
