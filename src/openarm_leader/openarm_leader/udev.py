"""리더 보드를 하나씩 꽂아 udev 고정 이름으로 등록하는 대화형 CLI."""

import argparse
import glob
import os
import subprocess
import sys
import time

from openarm_leader import config as config_io

RULES_PATH = '/etc/udev/rules.d/99-openarm-leader.rules'
RULES_HEADER = '# openarm_leader 리더 보드 고정 이름 규칙\n'
SYMLINK_PREFIX = 'openarm_leader_'
TTY_PATTERNS = ('/dev/ttyUSB*', '/dev/ttyACM*')
ENUMERATION_WAIT_SEC = 1.0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='udev',
        description='리더 보드에 /dev/openarm_leader_<arm> 고정 이름을 배정합니다. '
                    '안내에 따라 보드를 하나씩 꽂으면 새 포트를 자동 인식합니다.')
    parser.add_argument(
        '--arm',
        help='이 팔 하나만 다시 등록합니다. 생략하면 모든 팔을 차례로 등록합니다.')
    parser.add_argument(
        '--port',
        help='자동 인식 대신 이 포트를 씁니다. --arm 과 함께 씁니다.')
    parser.add_argument(
        '--config',
        help='갱신할 leader.yaml 경로. 생략하면 설치본을 씁니다.')
    return parser.parse_args(argv)


def prompt(message):
    input(f'{message} — 준비되면 Enter: ')


def list_ttys():
    ports = []
    for pattern in TTY_PATTERNS:
        ports.extend(glob.glob(pattern))
    return set(ports)


def usb_attributes(port):
    """tty 뒤에 붙은 USB 어댑터의 식별 속성. 못 찾으면 None."""
    name = os.path.basename(port)
    device = os.path.realpath(f'/sys/class/tty/{name}/device')
    node = device
    while node not in ('/', ''):
        vendor_path = os.path.join(node, 'idVendor')
        if os.path.exists(vendor_path):
            def read(attr):
                path = os.path.join(node, attr)
                if not os.path.exists(path):
                    return None
                with open(path, 'r', encoding='ascii') as handle:
                    return handle.read().strip()
            return {
                'idVendor': read('idVendor'),
                'idProduct': read('idProduct'),
                'serial': read('serial'),
                'kernels': os.path.basename(node),
            }
        node = os.path.dirname(node)
    return None


def build_rule(arm, attrs):
    """한 팔의 udev 규칙 한 줄을 만든다."""
    parts = [
        'SUBSYSTEM=="tty"',
        f'ATTRS{{idVendor}}=="{attrs["idVendor"]}"',
        f'ATTRS{{idProduct}}=="{attrs["idProduct"]}"',
    ]
    if attrs.get('serial'):
        parts.append(f'ATTRS{{serial}}=="{attrs["serial"]}"')
    else:
        # serial 이 없는 어댑터는 물리 USB 포트 위치로 고정한다.
        parts.append(f'KERNELS=="{attrs["kernels"]}"')
    parts.append(f'SYMLINK+="{SYMLINK_PREFIX}{arm}"')
    parts.append('MODE="0660"')
    parts.append('GROUP="dialout"')
    return ', '.join(parts)


def resolve_conflicts(collected):
    """serial 이 서로 같은 어댑터들을 물리 포트 고정으로 바꾼다."""
    serials = [attrs['serial'] for attrs in collected.values() if attrs['serial']]
    duplicated = {serial for serial in serials if serials.count(serial) > 1}
    for arm, attrs in collected.items():
        if attrs['serial'] in duplicated:
            collected[arm] = dict(attrs, serial=None)
    return collected


def merge_rules(existing_text, arm, rule):
    """기존 규칙 파일에서 같은 팔의 줄만 바꾼 새 파일 내용을 만든다."""
    marker = f'SYMLINK+="{SYMLINK_PREFIX}{arm}"'
    lines = [
        line for line in (existing_text or '').splitlines()
        if line.strip() and not line.startswith('#') and marker not in line
    ]
    lines.append(rule)
    return RULES_HEADER + '\n'.join(lines) + '\n'


def read_rules_file():
    if not os.path.exists(RULES_PATH):
        return ''
    with open(RULES_PATH, 'r', encoding='utf-8') as handle:
        return handle.read()


def install(content):
    subprocess.run(
        ['sudo', 'tee', RULES_PATH], input=content, text=True,
        check=True, stdout=subprocess.DEVNULL)
    subprocess.run(['sudo', 'udevadm', 'control', '--reload-rules'], check=True)
    subprocess.run(
        ['sudo', 'udevadm', 'trigger', '--subsystem-match=tty'], check=True)


def attributes_or_exit(port):
    attrs = usb_attributes(port)
    if attrs is None or not attrs['idVendor']:
        sys.exit(f'{port} 의 USB 속성을 찾지 못했습니다. USB 어댑터 포트가 맞는지 확인해 주세요.')
    return attrs


def describe(arm, port, attrs):
    serial = attrs['serial'] or '(없음 — 물리 포트로 고정)'
    print(f'  {arm}: {port}, vendor {attrs["idVendor"]}, '
          f'product {attrs["idProduct"]}, serial {serial}')


def detect_new_port(baseline):
    """baseline 이후 새로 나타난 tty 포트 하나를 찾는다."""
    time.sleep(ENUMERATION_WAIT_SEC)
    new_ports = sorted(list_ttys() - baseline)
    if not new_ports:
        sys.exit('새 포트가 보이지 않습니다. 보드를 꽂았는지 확인하고 다시 실행해 주세요.')
    if len(new_ports) > 1:
        sys.exit(f'새 포트가 여러 개입니다 {new_ports}. 보드를 하나씩 꽂아 주세요.')
    return new_ports[0]


def collect_ports(arms, manual_port):
    """팔별로 보드를 꽂게 해 (포트, 어댑터 속성) 을 모은다."""
    if manual_port is not None:
        if not os.path.exists(manual_port):
            sys.exit(f'{manual_port} 가 없습니다. 보드가 꽂혀 있는지 확인해 주세요.')
        arm = arms[0]
        return {arm: (manual_port, attributes_or_exit(manual_port))}

    prompt('리더 보드를 모두 뽑아 주세요')
    baseline = list_ttys()
    collected = {}
    for arm in arms:
        prompt(f'{arm} 보드를 꽂아 주세요')
        port = detect_new_port(baseline)
        collected[arm] = (port, attributes_or_exit(port))
        describe(arm, port, collected[arm][1])
        baseline = list_ttys()
    return collected


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.port and not args.arm:
        sys.exit('--port 는 --arm 과 함께 쓰세요.')

    path = args.config or config_io.default_path()
    cfg = config_io.load(path)

    arms = [args.arm] if args.arm else list(cfg['arms'])
    unknown = [arm for arm in arms if arm not in cfg['arms']]
    if unknown:
        sys.exit(f'설정에 없는 팔: {unknown} (가능: {sorted(cfg["arms"])})')

    collected = collect_ports(arms, args.port)
    attributes = resolve_conflicts(
        {arm: attrs for arm, (_port, attrs) in collected.items()})

    fallbacks = {arm: attrs['kernels'] for arm, attrs in attributes.items()
                 if not attrs['serial']}
    if len(fallbacks) > 1 and len(set(fallbacks.values())) < len(fallbacks):
        sys.exit('어댑터에 serial 이 없고 같은 USB 포트를 썼습니다. '
                 '좌우를 구분할 수 없으니 서로 다른 포트에 꽂아 다시 실행해 주세요.')

    content = read_rules_file()
    rules = {arm: build_rule(arm, attrs) for arm, attrs in attributes.items()}
    for arm, rule in rules.items():
        content = merge_rules(content, arm, rule)

    print(f'\n{RULES_PATH} 에 들어갈 규칙:')
    for rule in rules.values():
        print(f'  {rule}')
    prompt('sudo 로 등록합니다')

    try:
        install(content)
    except subprocess.CalledProcessError as error:
        sys.exit(f'규칙 설치에 실패했습니다: {error}')

    for arm in rules:
        cfg['arms'][arm]['leader']['port'] = f'/dev/{SYMLINK_PREFIX}{arm}'
    config_io.save(cfg, path)

    print('\n등록 완료. leader.yaml 의 port 를 고정 이름으로 바꿨습니다.')
    print('보드를 뽑았다 다시 꽂은 뒤 고정 이름을 확인해 주세요:')
    for arm in rules:
        print(f'  ls -l /dev/{SYMLINK_PREFIX}{arm}')


if __name__ == '__main__':
    main()
