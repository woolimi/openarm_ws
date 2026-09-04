"""중력보상 캘리브레이션 CLI — 자세를 돌며 재고, 풀어서 follower.yaml 에 적는다.

정지한 팔의 관절 토크에서 모델 중력을 빼면 모델이 모르는 것만 남는다. 그 잔차를
자세 아홉 개에서 모아 손끝 점질량(질량·무게중심)과 관절별 상수 오프셋을 함께 푼다.

    ros2 run openarm_follower calibrate_gravity              # 양팔
    ros2 run openarm_follower calibrate_gravity --arm left   # 한 팔만
    ros2 run openarm_follower calibrate_gravity --dry-run    # 재기만 하고 저장 안 함

실기에서만 뜻이 있다. mock hardware 는 토크를 0 으로 내므로 잴 것이 없다.
"""

import argparse
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from openarm_follower import config as config_io
from openarm_follower.calib_poses import HOME_POSE, poses_for_arm
from openarm_follower.gravity_model import (
    GravityModel, arm_joint_names, solve)


class FollowerArm(Node):
    """팔로워 한 대를 읽고 움직이는 핸들. 토픽 두 개와 팔별 명령 발행자를 갖는다."""

    def __init__(self, arms, calibration_cfg):
        super().__init__('gravity_calibrate')
        self._cfg = calibration_cfg
        self._joint_state = None
        self._urdf = None

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        # robot_state_publisher 가 latched 로 한 번만 내보내므로 transient local 이어야
        # 늦게 붙은 이 노드도 받는다.
        self.create_subscription(
            String, '/robot_description', self._on_urdf,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))

        self._command_publishers = {
            arm: self.create_publisher(
                Float64MultiArray,
                f'/{arm}_forward_position_controller/commands', 10)
            for arm in arms
        }

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _on_urdf(self, msg):
        self._urdf = msg.data

    # --- 준비 대기 ------------------------------------------------
    def wait_for_inputs(self, timeout_sec=20.0):
        """joint_states 와 robot_description 이 모두 올 때까지 기다린다."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._joint_state is not None and self._urdf is not None:
                return True
            time.sleep(0.1)
        return False

    @property
    def urdf(self):
        return self._urdf

    def sample_joint_state(self, joint_names):
        msg = self._joint_state
        index = {name: i for i, name in enumerate(msg.name)}
        missing = [n for n in joint_names if n not in index]
        if missing:
            raise RuntimeError(f'/joint_states 에 없는 관절: {missing}')
        position = [msg.position[index[n]] for n in joint_names]
        velocity = ([msg.velocity[index[n]] for n in joint_names]
                    if len(msg.velocity) == len(msg.name) else [0.0] * len(joint_names))
        effort = ([msg.effort[index[n]] for n in joint_names]
                  if len(msg.effort) == len(msg.name) else [0.0] * len(joint_names))
        return position, velocity, effort

    def positions(self, arm):
        position, _, _ = self.sample_joint_state(arm_joint_names(arm))
        return np.array(position)

    def move_to(self, arm, target):
        """지금 자세에서 target 까지 보간해 보낸다. 관절 속도 상한을 지킨다."""
        rate_hz = float(self._cfg['rate_hz'])
        speed = float(self._cfg['move_speed'])
        start = self.positions(arm)
        target = np.asarray(target, dtype=float)

        distance = float(np.max(np.abs(target - start))) if len(start) else 0.0
        duration = max(0.5, distance / speed) if speed > 0.0 else 0.5
        steps = max(1, int(duration * rate_hz))

        for step in range(1, steps + 1):
            blend = start + (target - start) * (step / steps)
            self._command_publishers[arm].publish(
                Float64MultiArray(data=[float(v) for v in blend]))
            time.sleep(1.0 / rate_hz)
        # 마지막 지령을 정확히 목표로 못박는다.
        self._command_publishers[arm].publish(
            Float64MultiArray(data=[float(v) for v in target]))

    def collect_sample(self, arm, seconds):
        """seconds 동안 관절각·토크를 평균 내고, 그동안의 최대 관절 속도를 함께 낸다."""
        joint_names = arm_joint_names(arm)
        positions, efforts, speed_max = [], [], 0.0
        count = max(1, int(seconds / 0.1))
        for _ in range(count):
            position, velocity, effort = self.sample_joint_state(joint_names)
            positions.append(position)
            efforts.append(effort)
            speed_max = max(speed_max, max(abs(v) for v in velocity))
            time.sleep(0.1)
        return (np.mean(positions, axis=0), np.mean(efforts, axis=0), speed_max)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='calibrate_gravity',
        description='중력보상 캘리브레이션. 자세를 돌며 재고 follower.yaml 에 적는다.')
    parser.add_argument(
        '--arm', choices=['left', 'right'],
        help='이 팔 하나만 캘리브레이션한다. 생략하면 양팔을 이어서 한다.')
    parser.add_argument(
        '--config', help='읽고 쓸 follower.yaml 경로. 생략하면 설치본을 쓴다.')
    parser.add_argument(
        '--poses', type=int,
        help='앞에서부터 이 개수의 자세만 쓴다. 짧게 확인할 때만.')
    parser.add_argument(
        '--no-bidirectional', action='store_true',
        help='자세마다 한 방향에서만 접근한다. 시간은 절반, 마찰 오차는 남는다.')
    parser.add_argument(
        '--dry-run', action='store_true', help='결과를 보여주기만 하고 저장하지 않는다.')
    parser.add_argument(
        '--yes', action='store_true', help='시작 확인과 저장 확인을 묻지 않는다.')
    return parser.parse_args(argv)


def measure_arm(robot, model, arm, cfg, pose_limit=None, bidirectional=True):
    """한 팔의 자세를 순회하며 표본을 모은다. 끝나면 홈으로 돌아간다."""
    poses = poses_for_arm(arm)
    if pose_limit:
        poses = poses[:pose_limit]
    home = model.clamp(HOME_POSE)
    offset = float(cfg['approach_offset'])
    approaches = ((+offset, '↑'), (-offset, '↓')) if bidirectional else ((0.0, ''),)

    samples = []
    for index, pose in enumerate(poses, start=1):
        target = model.clamp(pose)
        # 자세 사이는 늘 홈을 경유한다. 직행은 joint1 이 대각으로 쓸며 몸통을 지난다.
        print(f'  자세 {index}/{len(poses)} — 홈 경유…')
        robot.move_to(arm, home)
        for approach, tag in approaches:
            if approach:
                robot.move_to(arm, model.clamp(np.asarray(pose) + approach))
            robot.move_to(arm, target)
            time.sleep(float(cfg['settle_sec']))
            position, effort, speed_max = robot.collect_sample(
                arm, float(cfg['sample_sec']))
            if speed_max > float(cfg['static_speed']):
                # 아직 흔들리는 중이면 한 번 더 기다렸다 잰다.
                time.sleep(1.5)
                position, effort, speed_max = robot.collect_sample(
                    arm, float(cfg['sample_sec']))
            samples.append((position, effort))
            print(f'    {tag}접근 표본 — |q̇|max {speed_max:.3f} rad/s, '
                  f'|τ|max {np.max(np.abs(effort)):.2f} Nm')
    print('  홈 복귀…')
    robot.move_to(arm, home)
    return samples


def validate(result, model, samples, cfg):
    """저장 전 판정. (통과, 이유) 를 돌려주고 값을 몰래 깎지 않는다."""
    plausibility = cfg['plausibility']
    mass, com = result['mass'], result['com']

    if not (0.0 <= mass <= float(plausibility['mass_max'])):
        return False, (f'질량 {mass:.3f} kg 이 [0, {plausibility["mass_max"]}] kg '
                       f'밖이다')
    for axis, value in zip('xy', com[:2]):
        if abs(value) > float(plausibility['com_xy_max']):
            return False, (f'무게중심 c{axis} {value:.3f} m 이 '
                           f'±{plausibility["com_xy_max"]} m 밖이다')
    if not (float(plausibility['com_z_lo']) <= com[2]
            <= float(plausibility['com_z_hi'])):
        return False, (f'무게중심 cz {com[2]:.3f} m 이 '
                       f'[{plausibility["com_z_lo"]}, {plausibility["com_z_hi"]}] m '
                       f'밖이다')

    # 표본으로 쓴 자세에서 피드포워드가 포화 캡을 넘으면 캡이 잘라 버려 보상이
    # 예상대로 듣지 않는다. 자세가 아니라 값이 잘못됐다는 신호다.
    cap = np.asarray(cfg['saturation_cap'], dtype=float)
    for position, _ in samples:
        torque = np.abs(model.nominal_gravity(position)
                        + model.payload_torque(position, mass, com))
        over = np.where(torque > cap)[0]
        if len(over):
            joint = int(over[np.argmax(torque[over] - cap[over])])
            return False, (f'joint{joint + 1} 피드포워드 {torque[joint]:.2f} Nm 이 '
                           f'포화 캡 {cap[joint]:.2f} Nm 을 넘는다')
    return True, 'ok'


def report(arm, result):
    print(f'\n  {arm} 결과 — 표본 {result["n"]}개, 조건수 {result["condition"]:.1f}')
    print(f'    payload_mass  {result["mass"]:.4f} kg')
    print('    payload_com   ['
          + ', '.join(f'{v:.4f}' for v in result['com']) + '] m')
    print('    tau_bias      ['
          + ', '.join(f'{v:.3f}' for v in result['bias']) + '] Nm')
    print(f'    잔차 RMS      {result["rms_before"]:.3f} → {result["rms_after"]:.3f} Nm')


def spin_until_shutdown(node):
    """백그라운드에서 콜백을 돌린다. 본류는 자세를 옮기며 그냥 기다린다."""
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def confirm(question):
    return input(f'{question} [y/N]: ').strip().lower() in ('y', 'yes')


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    cfg = config_io.load(args.config)
    arms = [args.arm] if args.arm else sorted(cfg['arms'])

    print('중력보상 캘리브레이션 — 팔이 스스로 자세 아홉 개를 돕니다.')
    print(f'  대상: {", ".join(arms)}')
    print('  팔 주변을 비우고, 비상 정지에 손이 닿는 자리에 서 주세요.')
    if not args.yes and not confirm('시작할까요?'):
        return 0

    rclpy.init()
    robot = FollowerArm(arms, cfg['calibration'])
    spin = threading.Thread(
        target=spin_until_shutdown, args=(robot,), daemon=True)
    spin.start()

    try:
        if not robot.wait_for_inputs():
            print('/joint_states 또는 /robot_description 이 오지 않습니다. '
                  'bringup 이 떠 있는지 확인해 주세요.')
            return 1

        results = {}
        for arm in arms:
            arm_cfg = cfg['arms'][arm]
            model = GravityModel(robot.urdf, arm, arm_cfg['tip_link'])

            _, _, effort = robot.sample_joint_state(arm_joint_names(arm))
            if max(abs(v) for v in effort) < 1e-6:
                print(f'{arm}: 관절 토크가 모두 0 입니다. mock hardware 로는 '
                      '캘리브레이션할 것이 없습니다.')
                return 1

            print(f'\n{arm} 캘리브레이션 시작')
            samples = measure_arm(
                robot, model, arm, cfg['calibration'],
                pose_limit=args.poses,
                bidirectional=not args.no_bidirectional
                and bool(cfg['calibration']['bidirectional']))
            result = solve(model, samples)
            report(arm, result)

            ok, why = validate(result, model, samples, cfg)
            if not ok:
                print(f'    저장하지 않습니다 — {why}')
                continue
            results[arm] = result

        if args.dry_run:
            print('\n--dry-run 이라 저장하지 않습니다.')
            return 0

        for arm, result in results.items():
            if not args.yes and not confirm(f'{arm} 결과를 follower.yaml 에 저장할까요?'):
                continue
            path = config_io.update_arm(
                arm, result['mass'], result['com'], result['bias'], args.config)
            if path is None:
                print(f'{arm}: follower.yaml 의 서식이 예상과 달라 저장하지 못했습니다.')
            else:
                print(f'{arm}: {path} 에 저장했습니다. bringup 을 다시 띄우면 반영됩니다.')
        return 0
    finally:
        # 먼저 shutdown 해야 spin 이 풀린다. 그 뒤에 노드를 거둔다.
        rclpy.shutdown()
        spin.join(timeout=2.0)
        robot.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
