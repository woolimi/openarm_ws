"""리더암 오프셋·그리퍼 범위를 잡아 leader.yaml 에 기록하는 대화형 CLI."""

import argparse
import sys

from openarm_leader import config as config_io
from openarm_leader.feetech import FeetechBus

ARM_JOINT_COUNT = 7


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='calibrate',
        description='Feetech 리더암 캘리브레이션. 모든 팔을 차례로 진행한다.')
    parser.add_argument(
        '--arm',
        help='이 팔 하나만 캘리브레이션한다. 생략하면 모든 팔을 이어서 한다.')
    parser.add_argument(
        '--port',
        help='leader.yaml 대신 쓸 시리얼 포트. --arm 과 함께 쓴다.')
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


def open_bus(arm, cfg, port_override):
    """한 팔의 버스를 연다. 실패하면 안내를 찍고 None."""
    leader_cfg = cfg['arms'][arm]['leader']
    port = port_override or leader_cfg['port']
    feetech_cfg = cfg['feetech']
    print(f'{arm}: 포트 {port}, 서보 id {[int(v) for v in leader_cfg["ids"]]}')
    try:
        return FeetechBus(
            port,
            int(feetech_cfg['baudrate']),
            int(feetech_cfg['protocol_end']),
            int(feetech_cfg['present_position_address']),
        )
    except RuntimeError as error:
        print(f'  {error}')
        return None


def calibrate_arm(arm, bus, cfg, path, port_override):
    """한 팔의 영점·그리퍼 범위를 읽어 그 자리에서 저장한다."""
    leader_cfg = cfg['arms'][arm]['leader']
    ids = [int(value) for value in leader_cfg['ids']]

    prompt(f'{arm} 리더암을 영점 자세로 두어라')
    offsets = read_or_exit(bus, ids, f'{arm} 영점 자세')
    print(f'  {arm} offset_ticks = {offsets}')

    gripper_id = ids[ARM_JOINT_COUNT]
    prompt(f'{arm} 그리퍼를 끝까지 열어라')
    open_ticks = read_or_exit(bus, [gripper_id], f'{arm} 그리퍼 열림')[0]
    print(f'  {arm} gripper open = {open_ticks}')

    prompt(f'{arm} 그리퍼를 끝까지 닫아라')
    closed_ticks = read_or_exit(bus, [gripper_id], f'{arm} 그리퍼 닫힘')[0]
    print(f'  {arm} gripper closed = {closed_ticks}')

    if port_override:
        leader_cfg['port'] = port_override
    leader_cfg['offset_ticks'] = [int(value) for value in offsets]
    leader_cfg['gripper_ticks'] = {
        'open': int(open_ticks),
        'closed': int(closed_ticks),
    }
    config_io.save(cfg, path)
    print(f'  {arm} 캘리브레이션을 {path} 에 기록했다.')


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.port and not args.arm:
        sys.exit('--port 는 --arm 과 함께 쓴다.')

    path = args.config or config_io.default_path()
    cfg = config_io.load(path)

    arms = [args.arm] if args.arm else list(cfg['arms'])
    unknown = [arm for arm in arms if arm not in cfg['arms']]
    if unknown:
        sys.exit(f'설정에 없는 팔: {unknown} (가능: {sorted(cfg["arms"])})')

    skipped = []
    for arm in arms:
        bus = open_bus(arm, cfg, args.port)
        if bus is None:
            skipped.append(arm)
            continue
        try:
            calibrate_arm(arm, bus, cfg, path, args.port)
        finally:
            bus.close()

    if skipped:
        sys.exit(f'{skipped} 는 포트를 못 열어 건너뛰었다.')


if __name__ == '__main__':
    main()
