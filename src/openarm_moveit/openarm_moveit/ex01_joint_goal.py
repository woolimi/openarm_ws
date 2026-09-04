"""예제 01 — 관절 하나씩 움직이기 (joint goal)

home 자세에서 관절 7개를 하나씩 움직였다 되돌려, 각 관절이 팔의 어디를 돌리는지 본다.
움직이는 동안 그 관절의 회전축을 RViz 에 화살표로 띄운다.

실행:
  터미널 1: ros2 launch openarm_moveit demo.launch.py
  터미널 2: ros2 run openarm_moveit ex01_joint_goal
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from openarm_moveit.robot import JOINT_AXES, MARKER_TOPIC, Arm
from openarm_moveit.utils import MoveGroupHelper, spin_sleep
from openarm_moveit.viz import BLUE, MarkerBoard

# 관절별 (역할, home 에서 얼마나 움직일지). 부호는 관절 한계 안쪽 방향이다.
JOINT_STEPS = [
    ('어깨 앞뒤 흔들기', math.radians(-45)),
    ('어깨 옆으로 들기', math.radians(-30)),
    ('위팔 비틀기', math.radians(45)),
    ('팔꿈치 굽히기', math.radians(60)),
    ('아래팔 비틀기', math.radians(45)),
    ('손목 좌우 꺾기', math.radians(30)),
    ('손목 앞뒤 꺾기', math.radians(45)),
]


class JointGoalDemo(Node):

    def __init__(self):
        super().__init__('ex01_joint_goal')
        self.declare_parameter('arm', 'left')
        self.arm = Arm(self.get_parameter('arm').value)
        self.get_logger().info(f'=== 예제 01: 관절 하나씩 움직이기 ({self.arm.side}) ===')
        self._axis_pub = MarkerBoard(self)._pub      # 같은 토픽 — 이전 마커를 지우고 시작

    def show_axis(self, index: int) -> None:
        """index 번째 관절의 회전축을 그 관절의 child link 프레임에 화살표로 그린다."""
        axis = JOINT_AXES[self.arm.side][index]
        half = 0.15
        m = Marker()
        m.header.frame_id = f'openarm_{self.arm.side}_link{index + 1}'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'joint_axis', 0
        m.type, m.action = Marker.ARROW, Marker.ADD
        m.points = [Point(x=-axis[0] * half, y=-axis[1] * half, z=-axis[2] * half),
                    Point(x=axis[0] * half, y=axis[1] * half, z=axis[2] * half)]
        m.scale = Vector3(x=0.012, y=0.03, z=0.03)
        m.color = BLUE
        self._axis_pub.publish(MarkerArray(markers=[m]))

    def hide_axis(self) -> None:
        m = Marker()
        m.header.frame_id = self.arm.tcp_link
        m.ns, m.id, m.action = 'joint_axis', 0, Marker.DELETE
        self._axis_pub.publish(MarkerArray(markers=[m]))

    def run(self):
        helper = MoveGroupHelper(self, self.arm)

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        self.get_logger().info('--- home 자세로 초기화 ---')
        helper.go_to_named('home')
        spin_sleep(self, 1.0)

        home = self.arm.named('home')
        for index, (role, delta) in enumerate(JOINT_STEPS):
            name = self.arm.joints[index]
            self.get_logger().info(f'--- {index + 1}단계: {name} ({role}) '
                                   f'{math.degrees(delta):+.0f}° ---')
            self.show_axis(index)

            target = dict(home)                     # home 을 복사해 관절 하나만 바꾼다
            target[name] = home[name] + delta
            if not helper.go_to_joint_goal(target):
                self.get_logger().error(f'{name} 이동 실패!')
                continue
            spin_sleep(self, 1.5)

            self.get_logger().info('  → home 복귀')
            helper.go_to_named('home')
            spin_sleep(self, 1.0)
            self.hide_axis()

        self.get_logger().info('--- 보너스: 관절 7개를 한 번에 — ready 자세 ---')
        helper.go_to_named('ready')
        spin_sleep(self, 2.0)
        helper.go_to_named('home')
        self.get_logger().info('=== 예제 01 완료 ===')


def main(args=None):
    rclpy.init(args=args)
    node = JointGoalDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
