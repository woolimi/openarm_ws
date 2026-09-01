"""mapping 모듈 단위 테스트."""

import math

from openarm_leader import mapping

import pytest


def test_ticks_to_rad_at_offset_is_zero():
    assert mapping.ticks_to_rad(2048, 2048, 1.0, 4096) == 0.0


def test_ticks_to_rad_quarter_turn():
    assert mapping.ticks_to_rad(3072, 2048, 1.0, 4096) == pytest.approx(math.pi / 2)


def test_ticks_to_rad_sign_flips_direction():
    forward = mapping.ticks_to_rad(3072, 2048, 1.0, 4096)
    backward = mapping.ticks_to_rad(3072, 2048, -1.0, 4096)
    assert backward == pytest.approx(-forward)


def test_rad_to_ticks_round_trips():
    ticks = mapping.rad_to_ticks(math.pi / 2, 2048, -1.0, 4096)
    assert mapping.ticks_to_rad(ticks, 2048, -1.0, 4096) == pytest.approx(math.pi / 2)


def test_clamp_holds_bounds():
    assert mapping.clamp(5.0, -1.0, 1.0) == 1.0
    assert mapping.clamp(-5.0, -1.0, 1.0) == -1.0
    assert mapping.clamp(0.25, -1.0, 1.0) == 0.25


def test_clamp_accepts_reversed_bounds():
    assert mapping.clamp(5.0, 1.0, -1.0) == 1.0


def test_gripper_ticks_map_to_travel_ends():
    assert mapping.gripper_ticks_to_rad(2048, 3072, 2048, 0.044, 0.0) == 0.0
    assert mapping.gripper_ticks_to_rad(3072, 3072, 2048, 0.044, 0.0) == pytest.approx(0.044)


def test_gripper_ticks_clamp_outside_range():
    assert mapping.gripper_ticks_to_rad(4000, 3072, 2048, 0.044, 0.0) == pytest.approx(0.044)
    assert mapping.gripper_ticks_to_rad(1000, 3072, 2048, 0.044, 0.0) == 0.0


def test_gripper_ticks_zero_span_is_closed():
    assert mapping.gripper_ticks_to_rad(2500, 2048, 2048, 0.044, 0.0) == 0.0


def test_ramp_alpha_saturates():
    assert mapping.ramp_alpha(0.0, 2.0) == 0.0
    assert mapping.ramp_alpha(1.0, 2.0) == pytest.approx(0.5)
    assert mapping.ramp_alpha(9.0, 2.0) == 1.0
    assert mapping.ramp_alpha(0.0, 0.0) == 1.0


def test_blend_endpoints():
    assert mapping.blend(1.0, 3.0, 0.0) == 1.0
    assert mapping.blend(1.0, 3.0, 1.0) == 3.0
    assert mapping.blend(1.0, 3.0, 0.5) == 2.0


def test_limits_from_degrees_converts_and_orders():
    limits = mapping.limits_from_degrees(
        {'joint1': [-90.0, 90.0], 'joint2': [0.0, 135.0]}, ('joint1', 'joint2'))
    assert limits[0] == pytest.approx((-math.pi / 2, math.pi / 2))
    assert limits[1] == pytest.approx((0.0, math.radians(135.0)))
