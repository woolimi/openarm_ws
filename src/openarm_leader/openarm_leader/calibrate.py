"""리더암 오프셋·그리퍼 범위를 잡아 leader.yaml 에 기록하는 대화형 CLI."""

import argparse
import sys

from openarm_leader import config as config_io
from openarm_leader.feetech import FeetechBus

ARM_JOINT_COUNT = 7


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='calibrate',
        description='Feetech 리더암 캘리브레이션')
    parser.add_argument(
        '--arm', required=True,
        help='캘리브레이션할 팔 이름 (leader.yaml 의 arms 키)')
    parser.add_argument(
        '--port',
        help='리더암 시리얼 포트. 생략하면 leader.yaml 값을 쓴다.')
    parser.add_argument(
        '--config',
        help='읽고 쓸 leader.yaml 경로. 생략하면 설치본을 쓴다.')
    return parser.parse_args(argv)


def prompt(message):
    """안내를 띄우고 Enter 를 기다린다."""
    input(f'{message} — 준비되면 Enter: ')


def read_or_exit(bus, ids, what):
    ticks = bus.read_positions(ids)
    if ticks is None:
        sys.exit(f'{what} 읽기에 실패했다. 배선·id·baudrate 를 확인하라.')
    return ticks


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    path = args.config or config_io.default_path()
    cfg = config_io.load(path)

    if args.arm not in cfg['arms']:
        sys.exit(f'설정에 없는 팔: {args.arm} (가능: {sorted(cfg["arms"])})')

    leader_cfg = cfg['arms'][args.arm]['leader']
    ids = [int(value) for value in leader_cfg['ids']]
    port = args.port or leader_cfg['port']
    feetech_cfg = cfg['feetech']

    print(f'포트 {port}, 서보 id {ids}')
    bus = FeetechBus(
        port,
        int(feetech_cfg['baudrate']),
        int(feetech_cfg['protocol_end']),
        int(feetech_cfg['present_position_address']),
    )
    try:
        prompt('리더암을 영점 자세로 두어라')
        offsets = read_or_exit(bus, ids, '영점 자세')
        print(f'offset_ticks = {offsets}')

        gripper_id = ids[ARM_JOINT_COUNT]
        prompt('그리퍼를 끝까지 열어라')
        open_ticks = read_or_exit(bus, [gripper_id], '그리퍼 열림')[0]
        print(f'gripper open = {open_ticks}')

        prompt('그리퍼를 끝까지 닫아라')
        closed_ticks = read_or_exit(bus, [gripper_id], '그리퍼 닫힘')[0]
        print(f'gripper closed = {closed_ticks}')
    finally:
        bus.close()

    leader_cfg['port'] = port
    leader_cfg['offset_ticks'] = [int(value) for value in offsets]
    leader_cfg['gripper_ticks'] = {
        'open': int(open_ticks),
        'closed': int(closed_ticks),
    }
    config_io.save(cfg, path)
    print(f'{path} 에 기록했다.')


if __name__ == '__main__':
    main()
