"""리더 입력을 팔로워 ros2_control 컨트롤러 명령으로 옮기는 relay 노드."""

import rclpy

from builtin_interfaces.msg import Duration

from rclpy.executors import ExternalShutdownException

from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64MultiArray

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from openarm_leader import config as config_io
from openarm_leader import mapping
from openarm_leader.feetech import FeetechBus

ARM_JOINT_COUNT = 7
#: 시작 보간에서 관절이 넘지 않을 속도 [rad/s]. 리더가 멀리 있으면 보간 시간이 늘어난다.
RAMP_MAX_SPEED = 0.5
JOINT_KEYS = tuple(f'joint{index}' for index in range(1, ARM_JOINT_COUNT + 1))


def arm_joint_names(arm):
    """팔로워 팔 관절 이름. 업스트림 컨트롤러 설정과 같은 순서다."""
    return [f'openarm_{arm}_{key}' for key in JOINT_KEYS]


def gripper_joint_name(arm):
    """팔로워 그리퍼 관절 이름."""
    return f'openarm_{arm}_finger_joint1'


class ArmChannel:
    """한 팔의 관절 이름·한계·퍼블리셔·리더 캘리브레이션 묶음."""

    def __init__(self, node, arm, cfg, port):
        arm_cfg = cfg['arms'][arm]
        leader_cfg = arm_cfg['leader']
        travel = cfg['gripper_travel']

        self.arm = arm
        self.port = port
        self.joint_names = arm_joint_names(arm)
        self.gripper_joint = gripper_joint_name(arm)

        self.limits = mapping.limits_from_degrees(
            arm_cfg['joint_limits_deg'], JOINT_KEYS)
        self.gripper_closed = float(travel['closed'])
        self.gripper_open = float(travel['open'])
        self.gripper_limits = (
            min(self.gripper_closed, self.gripper_open),
            max(self.gripper_closed, self.gripper_open),
        )

        self.ids = [int(value) for value in leader_cfg['ids']]
        self.signs = [float(value) for value in leader_cfg['signs']]
        self.offset_ticks = [float(value) for value in leader_cfg['offset_ticks']]
        self.gripper_open_ticks = float(leader_cfg['gripper_ticks']['open'])
        self.gripper_closed_ticks = float(leader_cfg['gripper_ticks']['closed'])

        self.arm_publisher = node.create_publisher(
            Float64MultiArray, f'/{arm}_forward_position_controller/commands', 10)
        self.gripper_publisher = node.create_publisher(
            JointTrajectory, f'/{arm}_gripper_controller/joint_trajectory', 10)

        self.start_arm = None
        self.ramp_start_ns = None
        self.ramp_sec = None
        self.gripper_filtered = None


class SliderSource:
    """joint_state_publisher_gui 가 내는 리더 joint_states 를 읽는다."""

    def __init__(self, node, topic):
        self._latest = None
        self._subscription = node.create_subscription(
            JointState, topic, self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self._latest = msg

    def read(self, channel):
        return sample_joint_state(self._latest, channel)

    def close(self):
        pass


class FeetechSource:
    """Feetech 리더암 서보에서 관절각을 읽는다."""

    def __init__(self, cfg, channels):
        feetech_cfg = cfg['feetech']
        self._ticks_per_rev = int(feetech_cfg['ticks_per_rev'])
        self._buses = {
            channel.arm: FeetechBus(
                channel.port,
                int(feetech_cfg['baudrate']),
                int(feetech_cfg['protocol_end']),
                int(feetech_cfg['present_position_address']),
            )
            for channel in channels
        }

    def read(self, channel):
        # ids 순서대로 tick 을 읽어 offset_ticks·signs 로 rad 로 바꾼다.
        # 세 목록 모두 calibrate 가 leader.yaml 에 채운 값이다.
        ticks = self._buses[channel.arm].read_positions(channel.ids)
        if ticks is None:
            return None
        positions = [
            mapping.ticks_to_rad(
                ticks[index],
                channel.offset_ticks[index],
                channel.signs[index],
                self._ticks_per_rev,
            )
            for index in range(ARM_JOINT_COUNT)
        ]
        gripper = mapping.gripper_ticks_to_rad(
            ticks[ARM_JOINT_COUNT],
            channel.gripper_open_ticks,
            channel.gripper_closed_ticks,
            channel.gripper_open,
            channel.gripper_closed,
        )
        return positions, gripper

    def close(self):
        for bus in self._buses.values():
            bus.close()


def sample_joint_state(msg, channel):
    """JointState 에서 한 팔의 (관절 7개, 그리퍼) 값을 뽑는다. 없으면 None."""
    if msg is None:
        return None
    # 관절 이름으로 찾는다.
    # 슬라이더 창도 팔로워도 URDF 관절 이름을 그대로 쓴다.
    index_of = {name: index for index, name in enumerate(msg.name)}
    try:
        positions = [float(msg.position[index_of[name]])
                     for name in channel.joint_names]
        gripper = float(msg.position[index_of[channel.gripper_joint]])
    except (KeyError, IndexError):
        return None
    return positions, gripper


class LeaderNode(Node):
    """리더 자세를 읽어 팔로워 컨트롤러 토픽으로 흘려보낸다."""

    def __init__(self):
        super().__init__('leader_node')

        # leader.yaml 값이 파라미터 기본값. launch 인자가 그 위를 덮는다.
        self.declare_parameter('config_file', config_io.default_path())
        config_file = self.get_parameter('config_file').value
        cfg = config_io.load(config_file)
        self.get_logger().info(f'설정 파일: {config_file}')

        self.declare_parameter('source', str(cfg['source']))
        self.declare_parameter('arms', ','.join(cfg['active_arms']))
        self.declare_parameter('rate_hz', float(cfg['rate_hz']))
        self.declare_parameter('ramp_sec', float(cfg['ramp_sec']))
        self.declare_parameter(
            'gripper_smoothing_alpha', float(cfg['gripper_smoothing_alpha']))
        self.declare_parameter(
            'leader_joint_states_topic', str(cfg['leader_joint_states_topic']))
        self.declare_parameter(
            'follower_joint_states_topic', str(cfg['follower_joint_states_topic']))
        for arm, arm_cfg in cfg['arms'].items():
            self.declare_parameter(f'{arm}_port', str(arm_cfg['leader']['port']))

        source_name = self.get_parameter('source').value
        self._ramp_sec = float(self.get_parameter('ramp_sec').value)
        self._gripper_alpha = mapping.clamp(
            float(self.get_parameter('gripper_smoothing_alpha').value), 0.0, 1.0)
        rate_hz = float(self.get_parameter('rate_hz').value)
        arms = [name.strip() for name in
                self.get_parameter('arms').value.split(',') if name.strip()]

        unknown = [arm for arm in arms if arm not in cfg['arms']]
        if unknown:
            raise ValueError(f'설정에 없는 팔: {unknown}')

        self._channels = [
            ArmChannel(self, arm, cfg, self.get_parameter(f'{arm}_port').value)
            for arm in arms
        ]

        self._follower_state = None
        self.create_subscription(
            JointState,
            self.get_parameter('follower_joint_states_topic').value,
            self._on_follower_state,
            10,
        )

        # 입력 소스 선택.
        # 둘 다 read(channel) 로 (관절 7개, 그리퍼) 를 돌려준다.
        if source_name == 'none':
            self._source = None
            self.get_logger().info('source=none — 리더 입력을 읽지 않는다.')
            return
        if source_name == 'sliders':
            self._source = SliderSource(
                self, self.get_parameter('leader_joint_states_topic').value)
        elif source_name == 'feetech':
            self._source = FeetechSource(cfg, self._channels)
        else:
            raise ValueError(f'알 수 없는 source: {source_name}')

        # 그리퍼 JTC 는 interpolation_method=none 이라 목표점 시각이 지나야 값을 넘긴다.
        # 다음 명령이 덮어쓰기 전에 지나도록 명령 주기의 절반으로 잡는다.
        self._gripper_horizon_ns = int(0.5e9 / rate_hz)
        self._waiting_logged = False
        # rate_hz 마다 _on_timer: 팔마다 read → _relay.
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(
            f'source={source_name}, arms={arms}, rate={rate_hz} Hz, '
            f'ramp={self._ramp_sec} s')

    def _on_follower_state(self, msg):
        self._follower_state = msg

    def _on_timer(self):
        # 팔로워 실제 자세가 시작 보간의 출발점.
        # 받기 전에는 명령을 내지 않는다.
        if self._follower_state is None:
            if not self._waiting_logged:
                self.get_logger().warn(
                    '팔로워 joint_states 를 아직 못 받았다. 명령을 내보내지 않는다.')
                self._waiting_logged = True
            return

        now_ns = self.get_clock().now().nanoseconds
        for channel in self._channels:
            sample = self._source.read(channel)
            if sample is None:
                continue
            self._relay(channel, sample, now_ns)

    def _relay(self, channel, sample, now_ns):
        positions, gripper = sample
        # 1) leader.yaml 의 joint_limits_deg 로 clamp
        positions = [
            mapping.clamp(value, lower, upper)
            for value, (lower, upper) in zip(positions, channel.limits)
        ]
        gripper = mapping.clamp(gripper, *channel.gripper_limits)

        # 2) 첫 주기: 팔로워 현재 자세가 출발점.
        #    보간 시간은 0.5 rad/s 상한으로 늘어난다.
        if channel.start_arm is None:
            start = sample_joint_state(self._follower_state, channel)
            if start is None:
                return
            channel.start_arm = start[0]
            channel.ramp_start_ns = now_ns
            channel.ramp_sec = mapping.ramp_duration(
                channel.start_arm, positions, self._ramp_sec, RAMP_MAX_SPEED)

        # 3) 출발 자세 → 리더 자세 선형 보간.
        #    alpha 가 1 이 되면 리더 값 그대로.
        alpha = mapping.ramp_alpha(
            (now_ns - channel.ramp_start_ns) / 1e9, channel.ramp_sec)
        arm_command = [
            mapping.blend(start, target, alpha)
            for start, target in zip(channel.start_arm, positions)
        ]
        # 그리퍼는 시작 보간 없이 리더를 따르되, low-pass 로 손떨림을 걸러낸다.
        if channel.gripper_filtered is None:
            channel.gripper_filtered = gripper
        else:
            channel.gripper_filtered = mapping.blend(
                channel.gripper_filtered, gripper, self._gripper_alpha)
        gripper_command = channel.gripper_filtered

        # 4) 팔은 Float64MultiArray 로,
        #    그리퍼는 JointTrajectory 한 점으로 publish.
        channel.arm_publisher.publish(Float64MultiArray(data=arm_command))

        point = JointTrajectoryPoint()
        point.positions = [gripper_command]
        point.time_from_start = Duration(
            sec=self._gripper_horizon_ns // 1000000000,
            nanosec=self._gripper_horizon_ns % 1000000000,
        )
        trajectory = JointTrajectory()
        trajectory.joint_names = [channel.gripper_joint]
        trajectory.points = [point]
        channel.gripper_publisher.publish(trajectory)

    def destroy_node(self):
        if getattr(self, '_source', None) is not None:
            self._source.close()
        return super().destroy_node()


def main(args=None):
    # launch 의 leader_node 실행 파일이 여기로 들어온다 (setup.py entry_points).
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
