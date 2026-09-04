"""예제 03 — Cartesian 경로 (손끝 직선 보간)

waypoint 사이를 직선으로 잇는 궤적을 /compute_cartesian_path 서비스로 계획하고,
달성률(fraction)이 충분할 때만 ExecuteTrajectory 로 실행한다.
손끝 아래 자세로 정사각형을 그린 뒤 수직으로 내려간다.

실행:
  터미널 1: ros2 launch openarm_moveit demo.launch.py
  터미널 2: ros2 run openarm_moveit ex03_cartesian_path
  터미널 3: ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
"""

import copy

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from openarm_moveit.robot import TOOL_DOWN, Arm
from openarm_moveit.utils import MoveGroupHelper, StepTrigger, make_pose, spin_sleep
from openarm_moveit.viz import CYAN, GREEN, ORANGE, RED, MarkerBoard

SQUARE_START = {'left': (0.25, 0.10, 0.30), 'right': (0.25, -0.10, 0.30)}   # 정사각형 시작점 [m]
SIDE = 0.10          # 한 변 [m]
DESCENT = 0.08       # 수직 하강 [m]
MIN_FRACTION = 0.8   # 이 달성률 아래면 실행하지 않는다


def shifted(pose: Pose, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Pose:
    """pose 를 복사해 위치만 옮긴다. 방향은 그대로."""
    p = copy.deepcopy(pose)
    p.position.x += dx
    p.position.y += dy
    p.position.z += dz
    return p


def xyz(pose: Pose):
    return (pose.position.x, pose.position.y, pose.position.z)


class CartesianPathDemo(Node):

    def __init__(self):
        super().__init__('ex03_cartesian_path')
        self.declare_parameter('arm', 'left')
        self.arm = Arm(self.get_parameter('arm').value)
        self.get_logger().info(f'=== 예제 03: Cartesian 경로 ({self.arm.side}) ===')
        self.board = MarkerBoard(self)
        self.step = StepTrigger(self)

    def plan_and_run(self, helper, waypoints, ns: str, label: str) -> bool:
        """계획 → fraction 게이트 → 실행. 결과를 경로 색으로 보여 준다."""
        trajectory, fraction = helper.compute_cartesian_path(waypoints, max_step=0.01)
        self.get_logger().info(f'  [{label}] 달성률 {fraction * 100:.1f}%')
        if trajectory is None or fraction < MIN_FRACTION:
            self.get_logger().warn(f'  [{label}] 달성률 미달 — 실행하지 않습니다')
            self.board.recolor(f'{ns}_line', RED)
            return False
        self.board.put(self.board.line('tcp_path', 0, helper.trajectory_tcp_path(trajectory), ORANGE))
        ok = helper.execute_trajectory(trajectory)
        self.board.recolor(f'{ns}_line', GREEN if ok else RED)
        return ok

    def run(self):
        helper = MoveGroupHelper(self, self.arm)
        helper.planning_time = 10.0

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        self.step.wait('ready 자세로 이동')
        helper.go_to_named('ready')
        spin_sleep(self, 1.0)

        # 시작점 — 손끝 아래. 직선은 현재 자세에서만 시작하므로 pose goal 로 정확히 그 점에 먼저 선다.
        start = make_pose(*SQUARE_START[self.arm.side], *TOOL_DOWN)
        self.step.wait('정사각형 시작점으로 이동')
        if not helper.go_to_pose_goal(start):
            self.get_logger().error('시작점 이동 실패')
            return
        spin_sleep(self, 1.0)

        # 정사각형 — 수평면(z 고정)에서 바깥쪽 → 앞 → 안쪽 → 뒤. 오른팔은 y 부호가 반대다.
        side_y = SIDE if self.arm.side == 'left' else -SIDE
        square = [shifted(start, dy=side_y), shifted(start, dx=SIDE, dy=side_y),
                  shifted(start, dx=SIDE), start]
        self.board.path('square', [xyz(start)] + [xyz(p) for p in square], CYAN, 'Square')
        self.step.wait('정사각형 경로 계획·실행')
        self.plan_and_run(helper, square, 'square', 'Square')
        spin_sleep(self, 1.0)

        # 수직 하강 — z 만 줄어드는 waypoint 셋
        descent = [shifted(start, dz=-DESCENT * k / 3) for k in (1, 2, 3)]
        self.board.path('descent', [xyz(start)] + [xyz(p) for p in descent], CYAN, 'Descent')
        self.step.wait('수직 하강 계획·실행')
        self.plan_and_run(helper, descent, 'descent', 'Descent')
        spin_sleep(self, 1.0)

        self.step.wait('home 복귀')
        helper.go_to_named('home')
        self.get_logger().info('=== 예제 03 완료 ===')


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPathDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
