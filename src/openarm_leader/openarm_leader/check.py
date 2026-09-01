"""리더암 서보 응답과 관절 매핑을 눈으로 확인하는 CLI."""

import argparse
import math
import sys
import time

from openarm_leader import config as config_io
from openarm_leader import mapping
from openarm_leader.feetech import FeetechBus

ARM_JOINT_COUNT = 7
READ_PERIOD_SEC = 0.1


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='check',
        description='리더암 모터 체크. 서보 응답을 확인한 뒤 관절값을 실시간으로 보여준다.')
    parser.add_argument(
        '--arm', required=True,
        help='확인할 팔 이름 (leader.yaml 의 arms 키)')
    parser.add_argument(
        '--port',
        help='리더암 시리얼 포트. 생략하면 leader.yaml 값을 쓴다.')
    parser.add_argument(
        '--config',
        help='읽을 leader.yaml 경로. 생략하면 설치본을 쓴다.')
    parser.add_argument(
        '--once', action='store_true',
        help='응답 확인만 하고 실시간 표시는 건너뛴다.')
    return parser.parse_args(argv)


def joint_label(index):
    if index < ARM_JOINT_COUNT:
        return f'joint{index + 1}'
    return 'gripper'


def render_rows(ticks, leader_cfg, feetech_cfg, gripper_travel):
    """현재 tick 을 관절별 표시 문자열 목록으로 바꾼다."""
    ticks_per_rev = int(feetech_cfg['ticks_per_rev'])
    rows = []
    for index in range(ARM_JOINT_COUNT):
        rad = mapping.ticks_to_rad(
            ticks[index],
            float(leader_cfg['offset_ticks'][index]),
            float(leader_cfg['signs'][index]),
            ticks_per_rev,
        )
        rows.append(
            f'{joint_label(index):8s} id {leader_cfg["ids"][index]:2d}  '
            f'{ticks[index]:4d} tick  {math.degrees(rad):+7.1f} deg')
    open_ticks = float(leader_cfg['gripper_ticks']['open'])
    closed_ticks = float(leader_cfg['gripper_ticks']['closed'])
    value = mapping.gripper_ticks_to_rad(
        ticks[ARM_JOINT_COUNT], open_ticks, closed_ticks,
        float(gripper_travel['open']), float(gripper_travel['closed']))
    span = float(gripper_travel['open']) - float(gripper_travel['closed'])
    percent = 100.0 * (value - float(gripper_travel['closed'])) / span if span else 0.0
    rows.append(
        f'{joint_label(ARM_JOINT_COUNT):8s} id {leader_cfg["ids"][ARM_JOINT_COUNT]:2d}  '
        f'{ticks[ARM_JOINT_COUNT]:4d} tick  {percent:5.1f} % 열림')
    return rows


def live_view(bus, ids, leader_cfg, feetech_cfg, gripper_travel):
    print('\n관절을 하나씩 움직여 자리·방향이 맞는지 확인하라. 종료는 Ctrl+C.')
    printed = 0
    try:
        while True:
            ticks = bus.read_positions(ids)
            if ticks is None:
                rows = ['읽기 실패 — 배선을 확인하라.']
            else:
                rows = render_rows(ticks, leader_cfg, feetech_cfg, gripper_travel)
            if printed:
                sys.stdout.write(f'\x1b[{printed}A')
            for row in rows:
                sys.stdout.write(f'\x1b[2K{row}\n')
            sys.stdout.flush()
            printed = len(rows)
            time.sleep(READ_PERIOD_SEC)
    except KeyboardInterrupt:
        print()


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    cfg = config_io.load(args.config)

    if args.arm not in cfg['arms']:
        sys.exit(f'설정에 없는 팔: {args.arm} (가능: {sorted(cfg["arms"])})')

    leader_cfg = cfg['arms'][args.arm]['leader']
    ids = [int(value) for value in leader_cfg['ids']]
    port = args.port or leader_cfg['port']
    feetech_cfg = cfg['feetech']

    print(f'포트 {port}, 서보 id {ids}')
    try:
        bus = FeetechBus(
            port,
            int(feetech_cfg['baudrate']),
            int(feetech_cfg['protocol_end']),
            int(feetech_cfg['present_position_address']),
        )
    except RuntimeError as error:
        sys.exit(str(error))
    try:
        missing = []
        for index, servo_id in enumerate(ids):
            model = bus.ping(servo_id)
            if model is None:
                missing.append(servo_id)
                print(f'  {joint_label(index):8s} id {servo_id:2d}  응답 없음')
            else:
                print(f'  {joint_label(index):8s} id {servo_id:2d}  응답 (모델 {model})')
        if missing:
            sys.exit(f'서보 {missing} 가 응답하지 않는다. '
                     '배선을 확인하고, id 배정은 register 로 한다.')
        if not args.once:
            live_view(bus, ids, leader_cfg, feetech_cfg, cfg['gripper_travel'])
    finally:
        bus.close()


if __name__ == '__main__':
    main()
