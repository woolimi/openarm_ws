"""follower.yaml 로드·저장. 팔로워 설정을 읽고 쓰는 유일한 통로."""

import os
import re

import yaml

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'openarm_follower'
CONFIG_FILE_NAME = 'follower.yaml'

ARM_JOINT_COUNT = 7

REQUIRED_KEYS = (
    'gravity_comp',
    'root_link',
    'saturation_cap',
    'arms',
    'calibration',
    'plausibility',
)

ARM_REQUIRED_KEYS = ('tip_link', 'payload_mass', 'payload_com', 'tau_bias')


def default_path():
    """설치된 follower.yaml 경로."""
    return os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', CONFIG_FILE_NAME)


def source_path(path=None):
    """실제로 글자가 사는 파일 경로.

    `--symlink-install` 이면 설치본이 소스 파일을 가리키는 심링크라, 그것을 풀어야
    저장한 값이 다음 빌드에 지워지지 않는다.
    """
    return os.path.realpath(path or default_path())


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
    for arm, arm_cfg in data['arms'].items():
        arm_missing = [key for key in ARM_REQUIRED_KEYS if key not in arm_cfg]
        if arm_missing:
            raise ValueError(f'{path}: arms.{arm} 항목 누락 {arm_missing}')
    return data


def _format_list(values, digits=4):
    return '[' + ', '.join(f'{float(v):.{digits}f}' for v in values) + ']'


def _arm_block(text, arm):
    """arms 아래 한 팔의 블록과 그 위치. 없으면 None.

    블록에 앵커해야 하는 이유는 두 팔의 줄 모양이 서로 같아서다. 파일 전체에 건
    치환은 왼팔을 고치라는 요청으로 오른팔 줄을 바꿔 놓는다. 팔 이름 줄에서
    시작해 더 깊이 들여쓴 줄만 먹으므로 옆 블록으로 샐 수 없다.
    """
    match = re.search(rf'^  {re.escape(arm)}:\n(?:(?:    [^\n]*)?\n)*',
                      text, re.M)
    return (match.group(0), match.start(), match.end()) if match else None


def update_arm(arm, payload_mass, payload_com, tau_bias, path=None):
    """한 팔의 실측값 세 줄만 바꿔 쓴다. 성공하면 쓴 경로, 실패하면 None.

    파일 전체를 다시 직렬화하지 않는 이유는 이 yaml 의 주석이 곧 각 값이 무엇인지에
    대한 설명이라서다. 세 치환이 각각 정확히 한 번씩 걸리지 않으면 쓰지 않는다 —
    서식이 바뀌었다면 파일이 아니라 이 함수가 따라가야 한다.
    """
    path = source_path(path)
    with open(path, 'r', encoding='utf-8') as handle:
        text = handle.read()

    found = _arm_block(text, arm)
    if not found:
        return None
    block, start, end = found

    block, n_mass = re.subn(
        r'^(    payload_mass: )[^\n]*',
        lambda m: m.group(1) + f'{float(payload_mass):.4f}', block, flags=re.M)
    block, n_com = re.subn(
        r'^(    payload_com: )\[[^\]\n]*\]',
        lambda m: m.group(1) + _format_list(payload_com), block, flags=re.M)
    block, n_bias = re.subn(
        r'^(    tau_bias: )\[[^\]\n]*\]',
        lambda m: m.group(1) + _format_list(tau_bias), block, flags=re.M)
    if (n_mass, n_com, n_bias) != (1, 1, 1):
        return None

    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(text[:start] + block + text[end:])
    return path
