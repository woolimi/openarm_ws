"""leader.yaml 로드·저장. teleop 설정을 읽고 쓰는 유일한 통로."""

import os

import yaml

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'openarm_leader'
CONFIG_FILE_NAME = 'leader.yaml'

#: calibrate 가 파일을 다시 쓸 때 앞에 붙이는 머리말.
CONFIG_HEADER = """\
# 리더암 teleoperation 설정. teleop 동작에 필요한 모든 값의 단일 진실 공급원이다.
# arms.<arm>.leader 아래 값은 `ros2 run openarm_leader calibrate` 가 갱신한다.
"""

REQUIRED_KEYS = (
    'source',
    'active_arms',
    'rate_hz',
    'ramp_sec',
    'gripper_smoothing_alpha',
    'leader_joint_states_topic',
    'follower_joint_states_topic',
    'gripper_travel',
    'feetech',
    'arms',
)


def default_path():
    """설치된 leader.yaml 경로."""
    return os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', CONFIG_FILE_NAME)


def load(path=None):
    """설정을 읽어 dict 로 돌려준다."""
    path = path or default_path()
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: 최상위가 매핑이 아니다.')
    missing = [key for key in REQUIRED_KEYS if key not in data]
    if missing:
        raise ValueError(f'{path}: 항목 누락 {missing}')
    return data


def save(data, path=None):
    """설정을 파일에 쓴다. 머리말은 다시 붙고 나머지 주석은 남지 않는다."""
    path = path or default_path()
    body = yaml.safe_dump(
        data, allow_unicode=True, default_flow_style=False, sort_keys=False)
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(CONFIG_HEADER)
        handle.write('\n')
        handle.write(body)
    return path
