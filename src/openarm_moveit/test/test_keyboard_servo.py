"""ex05 의 키 → 방향 변환 테스트.

창에서 오는 것은 지금 눌려 있는 키의 집합이라, 방향을 만드는 일은 ROS 도 창도 없이
그 집합만으로 끝난다. 크기를 1 로 맞추는 부분이 이 파일이 지키려는 성질이다 — 맞추지
않으면 대각선이 축 하나보다 √2 배 빨라진다.
"""

import collections
import math

from openarm_moveit.ex05_keyboard_servo import direction_from_keys

import pygame

import pytest


def held(*keys):
    pressed = collections.defaultdict(bool)
    for key in keys:
        pressed[key] = True
    return pressed


def test_nothing_pressed_is_no_motion():
    assert direction_from_keys(held()) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('key, expected', [
    (pygame.K_w, (1.0, 0.0, 0.0)),
    (pygame.K_s, (-1.0, 0.0, 0.0)),
    (pygame.K_a, (0.0, 1.0, 0.0)),
    (pygame.K_d, (0.0, -1.0, 0.0)),
    (pygame.K_q, (0.0, 0.0, 1.0)),
    (pygame.K_e, (0.0, 0.0, -1.0)),
])
def test_single_key_drives_one_axis(key, expected):
    assert direction_from_keys(held(key)) == expected


def test_opposite_keys_cancel():
    assert direction_from_keys(held(pygame.K_w, pygame.K_s)) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('keys', [
    (pygame.K_w, pygame.K_a),
    (pygame.K_w, pygame.K_q),
    (pygame.K_w, pygame.K_a, pygame.K_q),
])
def test_combinations_keep_unit_speed(keys):
    direction = direction_from_keys(held(*keys))
    assert math.dist((0.0, 0.0, 0.0), direction) == pytest.approx(1.0)


def test_diagonal_splits_evenly():
    x, y, z = direction_from_keys(held(pygame.K_w, pygame.K_a))
    assert (x, y, z) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5), 0.0))
