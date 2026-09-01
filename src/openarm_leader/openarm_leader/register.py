"""리더 서보에 id 를 배정하는 CLI. 서보를 하나씩 연결해 실행한다."""

import argparse
import sys

from openarm_leader import config as config_io
from openarm_leader.feetech import FeetechBus

SCAN_MAX_ID_DEFAULT = 20
SCAN_MAX_ID_FULL = 253


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='register',
        description='Feetech 리더 서보 id 등록. 인자 없이 실행하면 버스를 스캔만 한다.')
    parser.add_argument(
        '--port', default='/dev/ttyUSB0',
        help='리더암 시리얼 포트 (기본 /dev/ttyUSB0)')
    parser.add_argument(
        '--id', type=int, dest='new_id',
        help='배정할 id (1~253). 생략하면 스캔 결과만 보여준다.')
    parser.add_argument(
        '--from', type=int, dest='current_id',
        help='바꿀 서보의 현재 id. 버스에 서보가 하나뿐이면 생략한다.')
    parser.add_argument(
        '--full-scan', action='store_true',
        help=f'id 1~{SCAN_MAX_ID_FULL} 전체를 스캔한다 (기본 1~{SCAN_MAX_ID_DEFAULT})')
    parser.add_argument(
        '--config',
        help='feetech 버스 설정을 읽을 leader.yaml 경로. 생략하면 설치본을 쓴다.')
    return parser.parse_args(argv)


def open_bus(args):
    cfg = config_io.load(args.config)
    feetech_cfg = cfg['feetech']
    return FeetechBus(
        args.port,
        int(feetech_cfg['baudrate']),
        int(feetech_cfg['protocol_end']),
        int(feetech_cfg['present_position_address']),
    )


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.new_id is not None and not 1 <= args.new_id <= SCAN_MAX_ID_FULL:
        sys.exit(f'id 는 1~{SCAN_MAX_ID_FULL} 범위여야 한다: {args.new_id}')

    try:
        bus = open_bus(args)
    except RuntimeError as error:
        sys.exit(str(error))
    try:
        max_id = SCAN_MAX_ID_FULL if args.full_scan else SCAN_MAX_ID_DEFAULT
        print(f'{args.port} 에서 id 1~{max_id} 스캔 중...')
        found = bus.scan(range(1, max_id + 1))

        if not found:
            sys.exit('응답하는 서보가 없다. 전원·배선·baudrate 를 확인하라.')
        for servo_id, model in sorted(found.items()):
            print(f'  id {servo_id:3d}  응답 (모델 {model})')

        if args.new_id is None:
            return

        if args.current_id is not None:
            if args.current_id not in found:
                sys.exit(f'id {args.current_id} 서보가 버스에 없다.')
            current_id = args.current_id
        elif len(found) == 1:
            current_id = next(iter(found))
        else:
            sys.exit('서보가 여러 개 잡혔다. --from 으로 바꿀 서보의 현재 id 를 지정하라.')

        if current_id == args.new_id:
            print(f'서보가 이미 id {args.new_id} 다.')
            return
        if args.new_id in found:
            sys.exit(f'id {args.new_id} 는 버스의 다른 서보가 이미 쓰고 있다.')

        if not bus.set_servo_id(current_id, args.new_id):
            sys.exit(f'id {current_id} → {args.new_id} 변경에 실패했다.')
        print(f'id {current_id} → {args.new_id} 변경 완료.')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
