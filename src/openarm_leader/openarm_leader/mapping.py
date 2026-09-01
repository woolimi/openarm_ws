"""리더 신호를 팔로워 관절 명령으로 옮기는 순수 변환 함수."""

import math

TWO_PI = 2.0 * math.pi


def clamp(value, lower, upper):
    """value 를 [lower, upper] 안으로 자른다."""
    if lower > upper:
        lower, upper = upper, lower
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def ticks_to_rad(ticks, offset_ticks, sign, ticks_per_rev):
    """서보 tick 을 관절각 [rad] 으로 바꾼다. offset 이 관절 영점이다."""
    return sign * (ticks - offset_ticks) * (TWO_PI / ticks_per_rev)


def rad_to_ticks(rad, offset_ticks, sign, ticks_per_rev):
    """ticks_to_rad 의 역변환."""
    return offset_ticks + sign * rad * (ticks_per_rev / TWO_PI)


def gripper_ticks_to_rad(ticks, open_ticks, closed_ticks, open_value, closed_value):
    """리더 그리퍼 tick 을 팔로워 finger 관절값으로 선형 대응시킨다."""
    span = open_ticks - closed_ticks
    if span == 0:
        return closed_value
    ratio = clamp((ticks - closed_ticks) / span, 0.0, 1.0)
    return closed_value + ratio * (open_value - closed_value)


def ramp_duration(start, target, base_sec, max_speed):
    """보간 시간을 정한다. 최속 관절이 max_speed [rad/s] 를 넘지 않게 늘린다."""
    if max_speed <= 0.0:
        return base_sec
    max_dist = max((abs(t - s) for s, t in zip(start, target)), default=0.0)
    return max(base_sec, max_dist / max_speed)


def ramp_alpha(elapsed_sec, ramp_sec):
    """보간 계수. 0 이면 팔로워 시작 자세, 1 이면 리더 자세."""
    if ramp_sec <= 0.0:
        return 1.0
    return clamp(elapsed_sec / ramp_sec, 0.0, 1.0)


def blend(start, target, alpha):
    """start 와 target 사이를 alpha 로 선형 보간한다."""
    return start + (target - start) * alpha


def limits_from_degrees(limits_deg, joint_keys):
    """joint_limits_deg 매핑을 joint_keys 순서의 (하한, 상한) [rad] 목록으로 편다."""
    limits = []
    for key in joint_keys:
        lower, upper = limits_deg[key]
        limits.append((math.radians(float(lower)), math.radians(float(upper))))
    return limits
