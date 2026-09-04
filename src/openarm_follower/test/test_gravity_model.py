"""중력 모델과 최소자승 풀이 단위 테스트.

플러그인이 쓰는 KDL 사슬은 여기서 못 부르므로, 대신 회귀 행렬이 정말 그 점질량의
중력토크인지를 pinocchio 로 확인한다 — 모델에 질량을 직접 붙여 얻은 중력토크와
A(q)·θ 가 같아야 한다. 부호가 뒤집혀도 최소자승은 답을 내므로, 이 대조가 없으면
캘리브레이션이 조용히 반대 부호의 보상을 저장한다.
"""

import os

from ament_index_python.packages import get_package_share_directory

import numpy as np

from openarm_follower.calib_poses import CALIB_POSES, poses_for_arm
from openarm_follower.gravity_model import GravityModel, solve

import pinocchio

import pytest

import xacro


TRUE_MASS = 0.31
TRUE_COM = np.array([0.012, -0.021, 0.085])
TRUE_BIAS = np.array([0.11, 0.06, -0.05, 0.19, -0.01, 0.05, 0.01])


@pytest.fixture(scope='module')
def urdf_xml():
    path = os.path.join(
        get_package_share_directory('openarm_description'),
        'assets', 'robot', 'openarm_v1.0', 'urdf', 'openarm_v10.urdf.xacro')
    return xacro.process_file(path, mappings={
        'arm_type': 'v1.0', 'bimanual': 'true',
        'use_fake_hardware': 'false', 'ros2_control': 'true'}).toxml()


def _model_with_payload(urdf_xml, arm):
    """같은 물체 집합에 점질량을 실제로 붙인 참조 모델."""
    model = pinocchio.buildModelFromXML(urdf_xml)
    frame = model.getFrameId(f'openarm_{arm}_hand')
    tip_joint = model.frames[frame].parentJoint
    on_chain = {int(j) for j in model.supports[tip_joint]}
    for joint_id in range(1, model.njoints):
        if joint_id not in on_chain:
            model.inertias[joint_id] = pinocchio.Inertia.Zero()
    model.appendBodyToJoint(
        tip_joint, pinocchio.Inertia(TRUE_MASS, TRUE_COM, np.zeros((3, 3))),
        model.frames[frame].placement)
    return model, model.createData()


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_regression_matches_a_real_point_mass(urdf_xml, arm):
    model = GravityModel(urdf_xml, arm, f'openarm_{arm}_hand')
    reference, data = _model_with_payload(urdf_xml, arm)
    for pose in poses_for_arm(arm):
        actual = pinocchio.computeGeneralizedGravity(
            reference, data, model.configuration(pose))[model.idx_v]
        predicted = (model.nominal_gravity(pose)
                     + model.payload_torque(pose, TRUE_MASS, TRUE_COM))
        assert actual == pytest.approx(predicted, abs=1e-9)


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_solve_recovers_payload_and_bias(urdf_xml, arm):
    model = GravityModel(urdf_xml, arm, f'openarm_{arm}_hand')
    noise = np.random.default_rng(0)
    samples = [
        (np.array(pose),
         model.nominal_gravity(pose)
         + model.payload_torque(pose, TRUE_MASS, TRUE_COM)
         + TRUE_BIAS + noise.normal(0.0, 0.01, 7))
        for pose in poses_for_arm(arm)]

    result = solve(model, samples)
    assert result['mass'] == pytest.approx(TRUE_MASS, abs=0.01)
    assert result['com'] == pytest.approx(TRUE_COM, abs=0.005)
    assert result['bias'] == pytest.approx(TRUE_BIAS, abs=0.02)
    assert result['rms_after'] < result['rms_before']


def test_solve_needs_three_poses(urdf_xml):
    model = GravityModel(urdf_xml, 'left', 'openarm_left_hand')
    with pytest.raises(ValueError):
        solve(model, [(np.zeros(7), np.zeros(7))])


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_poses_stay_inside_the_joint_limits(urdf_xml, arm):
    model = GravityModel(urdf_xml, arm, f'openarm_{arm}_hand')
    for pose in poses_for_arm(arm):
        assert np.allclose(model.clamp(pose), pose), (
            f'{arm} 자세가 관절 한계 밖이다: {pose}')


def test_right_poses_mirror_the_first_three_joints():
    for left, right in zip(CALIB_POSES, poses_for_arm('right')):
        assert right[:3] == [-v for v in left[:3]]
        assert right[3:] == left[3:]
