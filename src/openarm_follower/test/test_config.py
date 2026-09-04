"""follower.yaml 읽기·쓰기 테스트.

저장이 파일 전체를 다시 직렬화하지 않는다는 것이 이 파일에서 지키려는 성질이다.
이 yaml 의 주석이 각 값이 무엇인지에 대한 유일한 설명이라 재직렬화는 그걸 지운다.
"""

import shutil

from openarm_follower import config as config_io

import pytest


@pytest.fixture()
def config_path(tmp_path):
    path = tmp_path / 'follower.yaml'
    shutil.copy(config_io.source_path(), path)
    return str(path)


def test_load_returns_every_required_key(config_path):
    data = config_io.load(config_path)
    for key in config_io.REQUIRED_KEYS:
        assert key in data
    for arm in ('left', 'right'):
        for key in config_io.ARM_REQUIRED_KEYS:
            assert key in data['arms'][arm]


def test_update_arm_writes_only_that_arm(config_path):
    before = config_io.load(config_path)
    assert config_io.update_arm(
        'right', 0.31, [0.01, -0.02, 0.08], [0.1] * 7, config_path)

    after = config_io.load(config_path)
    assert after['arms']['right']['payload_mass'] == pytest.approx(0.31)
    assert after['arms']['right']['payload_com'] == pytest.approx([0.01, -0.02, 0.08])
    assert after['arms']['right']['tau_bias'] == pytest.approx([0.1] * 7)
    assert after['arms']['left'] == before['arms']['left']


def test_update_arm_keeps_the_comments(config_path):
    with open(config_path, encoding='utf-8') as handle:
        before = handle.read()
    config_io.update_arm('left', 0.2, [0.0, 0.0, 0.05], [0.0] * 7, config_path)
    with open(config_path, encoding='utf-8') as handle:
        after = handle.read()
    assert after.count('#') == before.count('#')


def test_update_arm_refuses_an_unexpected_layout(tmp_path):
    path = tmp_path / 'follower.yaml'
    path.write_text('arms:\n  left:\n    tip_link: x\n', encoding='utf-8')
    assert config_io.update_arm(
        'left', 0.1, [0.0] * 3, [0.0] * 7, str(path)) is None


def test_load_rejects_a_missing_arm_key(tmp_path):
    path = tmp_path / 'follower.yaml'
    path.write_text(
        'gravity_comp: true\nroot_link: r\nsaturation_cap: [1]\n'
        'calibration: {}\nplausibility: {}\n'
        'arms:\n  left:\n    tip_link: x\n', encoding='utf-8')
    with pytest.raises(ValueError, match='arms.left'):
        config_io.load(str(path))
