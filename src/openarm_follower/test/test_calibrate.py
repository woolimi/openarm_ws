"""자세 순회와 저장 전 판정 테스트 — 로봇 없이 가짜 핸들로 돌린다.

핸들이 하는 일은 넷뿐이다: 지금 자세를 알려주고, 자세로 옮기고, 표본을 내고,
관절 이름을 안다. 그래서 실기 없이도 순회 순서와 판정을 그대로 확인할 수 있다.
"""

import os

from ament_index_python.packages import get_package_share_directory

import numpy as np

from openarm_follower.calib_poses import HOME_POSE, poses_for_arm
from openarm_follower.calibrate import measure_arm, validate
from openarm_follower.gravity_model import GravityModel, solve

import pytest

import xacro


FAST_CALIBRATION = {
    'rate_hz': 1000.0,
    'move_speed': 100.0,
    'settle_sec': 0.0,
    'sample_sec': 0.0,
    'static_speed': 0.02,
    'bidirectional': True,
    'approach_offset': 0.06,
}

PLAUSIBILITY = {
    'mass_max': 2.0, 'com_xy_max': 0.15, 'com_z_lo': -0.20, 'com_z_hi': 0.35,
}

SATURATION_CAP = [20.0, 20.0, 13.5, 13.5, 3.5, 3.5, 3.5]


class StubArm:
    """지령을 그대로 따르는 팔. 토크는 모델 중력에 참값 페이로드를 얹어 낸다."""

    def __init__(self, model, mass=0.3, com=(0.01, -0.02, 0.08)):
        self._model = model
        self._mass = mass
        self._com = np.asarray(com)
        self._position = np.array(HOME_POSE, dtype=float)
        self.visited = []

    def positions(self, _arm):
        return self._position.copy()

    def move_to(self, _arm, target):
        self._position = np.asarray(target, dtype=float)
        self.visited.append(self._position.copy())

    def collect_sample(self, _arm, _seconds):
        torque = (self._model.nominal_gravity(self._position)
                  + self._model.payload_torque(self._position, self._mass, self._com))
        return self._position.copy(), torque, 0.0


@pytest.fixture(scope='module')
def model():
    path = os.path.join(
        get_package_share_directory('openarm_description'),
        'assets', 'robot', 'openarm_v1.0', 'urdf', 'openarm_v10.urdf.xacro')
    urdf_xml = xacro.process_file(path, mappings={
        'arm_type': 'v1.0', 'bimanual': 'true',
        'use_fake_hardware': 'false', 'ros2_control': 'true'}).toxml()
    return GravityModel(urdf_xml, 'left', 'openarm_left_hand')


def test_sweep_visits_home_between_poses(model):
    robot = StubArm(model)
    measure_arm(robot, model, 'left', FAST_CALIBRATION, pose_limit=2)
    home = model.clamp(HOME_POSE)
    poses = poses_for_arm('left')[:2]
    # 자세마다 홈 한 번, 그리고 마지막에 홈 복귀.
    home_visits = sum(1 for p in robot.visited if np.allclose(p, home))
    assert home_visits == len(poses) + 1


def test_sweep_recovers_the_stub_payload(model):
    robot = StubArm(model, mass=0.27, com=(0.005, -0.015, 0.09))
    samples = measure_arm(robot, model, 'left', FAST_CALIBRATION)
    # 자세 아홉 개를 양방향으로 접근하므로 표본은 열여덟 개다.
    assert len(samples) == 2 * len(poses_for_arm('left'))
    result = solve(model, samples)
    assert result['mass'] == pytest.approx(0.27, abs=1e-3)
    assert result['com'] == pytest.approx([0.005, -0.015, 0.09], abs=1e-3)
    assert result['bias'] == pytest.approx([0.0] * 7, abs=1e-3)


def test_one_way_sweep_halves_the_samples(model):
    robot = StubArm(model)
    samples = measure_arm(robot, model, 'left', FAST_CALIBRATION,
                          bidirectional=False)
    assert len(samples) == len(poses_for_arm('left'))


def _cfg():
    return {'plausibility': dict(PLAUSIBILITY), 'saturation_cap': list(SATURATION_CAP)}


def test_validate_accepts_a_measured_payload(model):
    robot = StubArm(model)
    samples = measure_arm(robot, model, 'left', FAST_CALIBRATION, pose_limit=4)
    result = solve(model, samples)
    assert validate(result, model, samples, _cfg())[0]


def test_validate_rejects_a_mass_typo(model):
    robot = StubArm(model)
    samples = measure_arm(robot, model, 'left', FAST_CALIBRATION, pose_limit=4)
    result = solve(model, samples)
    result['mass'] = 5.0
    ok, why = validate(result, model, samples, _cfg())
    assert not ok and 'kg' in why


def test_validate_rejects_a_feed_forward_over_the_cap(model):
    robot = StubArm(model)
    samples = measure_arm(robot, model, 'left', FAST_CALIBRATION, pose_limit=4)
    result = solve(model, samples)
    result['mass'], result['com'] = 1.9, [0.0, 0.0, 0.3]
    ok, why = validate(result, model, samples, _cfg())
    assert not ok and '포화 캡' in why
