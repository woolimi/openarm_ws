"""이전 실행이 남긴 로봇 세션 프로세스 정리."""

import os
import signal
import time

#: 로봇 세션을 이루는 실행 파일 이름. launch 가 새로 띄우기 전에 이전 것을 거둔다.
SESSION_PROCESS_NAMES = (
    'ros2_control_node',
    'robot_state_publisher',
    'move_group',
    'rviz2',
    'leader_node',
    'servo_node',
    'joint_state_publisher_gui',
)
GRACE_SEC = 2.0


def _command_name(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as handle:
            cmdline = handle.read().split(b'\0')
    except OSError:
        return None
    if not cmdline or not cmdline[0]:
        return None
    base = os.path.basename(cmdline[0].decode(errors='replace'))
    # 파이썬 노드는 인터프리터가 첫 인자라 스크립트 이름을 본다.
    if base.startswith('python') and len(cmdline) > 1 and cmdline[1]:
        return os.path.basename(cmdline[1].decode(errors='replace'))
    return base


def find_stale(names=SESSION_PROCESS_NAMES):
    """이름이 겹치는 다른 프로세스의 pid 목록."""
    own_pid = os.getpid()
    stale = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == own_pid:
            continue
        if _command_name(pid) in names:
            stale.append(pid)
    return stale


def kill_stale(names=SESSION_PROCESS_NAMES, grace_sec=GRACE_SEC):
    """이전 세션 프로세스에 SIGINT 를 보내고, 안 죽으면 SIGKILL 한다."""
    stale = find_stale(names)
    for pid in stale:
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            pass
    if not stale:
        return []
    deadline = time.monotonic() + grace_sec
    remaining = set(stale)
    while remaining and time.monotonic() < deadline:
        remaining = {pid for pid in remaining if os.path.exists(f'/proc/{pid}')}
        time.sleep(0.1)
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    return stale
