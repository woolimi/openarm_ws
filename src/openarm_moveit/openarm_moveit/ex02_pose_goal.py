"""예제 02 — 손끝 자세 지령 (pose goal)

손끝(tcp)의 위치 x·y·z 와 방향 roll·pitch·yaw 를 주면 MoveIt 이 IK 로 관절각을 풀고
경로를 계획해 움직인다. 목표는 RViz 마커로 먼저 보이고, 계획된 손끝 경로를 선으로
그린 뒤 실행한다. 단계는 /next_step 신호로 넘어간다.

실행:
  터미널 1: ros2 launch openarm_moveit demo.launch.py
  터미널 2: ros2 run openarm_moveit ex02_pose_goal
  터미널 3: ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
"""

import math

import rclpy
from rclpy.node import Node

from openarm_moveit.robot import TOOL_DOWN, TOOL_FORWARD, Arm
from openarm_moveit.utils import MoveGroupHelper, StepTrigger, make_pose, spin_sleep
from openarm_moveit.viz import BLUE, GREEN, ORANGE, RED, YELLOW, MarkerBoard

# 목표 자세 — 위치 [m] 와 방향 (roll, pitch, yaw) [rad]
TARGETS = {
    'left': [
        dict(label='Front', desc='몸 앞, 손끝 아래', xyz=(0.30, 0.15, 0.30), rpy=TOOL_DOWN),
        dict(label='Left', desc='왼쪽 바깥, 손끝 아래', xyz=(0.20, 0.35, 0.40), rpy=TOOL_DOWN),
        dict(label='High', desc='앞으로 들어 손끝이 앞을 향함', xyz=(0.35, 0.15, 0.60), rpy=TOOL_FORWARD),
    ],
}
TARGETS['right'] = [dict(t, xyz=(t['xyz'][0], -t['xyz'][1], t['xyz'][2]),
                         rpy=(t['rpy'][0], t['rpy'][1], -t['rpy'][2])) for t in TARGETS['left']]


class PoseGoalDemo(Node):

    def __init__(self):
        super().__init__('ex02_pose_goal')
        self.declare_parameter('arm', 'left')
        self.arm = Arm(self.get_parameter('arm').value)
        self.get_logger().info(f'=== 예제 02: 손끝 자세 지령 ({self.arm.side}) ===')
        self.board = MarkerBoard(self)
        self.step = StepTrigger(self)

    def plan_show_execute(self, helper, plan_result) -> bool:
        """계획 결과의 손끝 경로를 주황 선으로 그린 뒤 실행한다."""
        ok, trajectory = plan_result
        if not ok or trajectory is None:
            return False
        path = helper.trajectory_tcp_path(trajectory)
        self.board.put(self.board.line('tcp_path', 0, path, ORANGE))
        return helper.execute_trajectory(trajectory)

    def run(self):
        helper = MoveGroupHelper(self, self.arm)
        helper.max_velocity_scaling = 0.3
        helper.planning_time = 10.0

        if not helper.wait_for_servers(timeout_sec=30.0):
            return
        if not helper.wait_for_joint_state(timeout_sec=10.0):
            return

        targets = TARGETS[self.arm.side]
        poses = [make_pose(*t['xyz'], *t['rpy']) for t in targets]

        # 목표 전부를 노란색(대기)으로 먼저 그린다
        for index, (target, pose) in enumerate(zip(targets, poses), 1):
            self.board.target(index, pose, target['label'], YELLOW)

        self.step.wait('ready 자세로 이동')
        self.plan_show_execute(helper, helper.plan_to_joint_goal(self.arm.named('ready')))
        spin_sleep(self, 1.0)

        for index, (target, pose) in enumerate(zip(targets, poses), 1):
            self.step.wait(f'{index}단계 {target["label"]} 이동')
            self.get_logger().info(f'--- {index}단계: {target["desc"]} ---')
            x, y, z = target['xyz']
            roll, pitch, yaw = (math.degrees(v) for v in target['rpy'])
            self.get_logger().info(f'  위치 ({x:.2f}, {y:.2f}, {z:.2f}) m  '
                                   f'방향 roll {roll:.0f}° pitch {pitch:.0f}° yaw {yaw:.0f}°')
            self.board.target(index, pose, target['label'], BLUE)     # 진행 중

            ok = self.plan_show_execute(helper, helper.plan_to_pose_goal(pose))
            if ok:
                self.get_logger().info(f'  → {target["label"]} 도달')
                self.board.target(index, pose, target['label'], GREEN)
            else:
                self.get_logger().warn(f'  → {target["label"]} 실패 (IK 해 없음 또는 충돌)')
                self.board.target(index, pose, target['label'], RED)

        self.step.wait('home 복귀')
        self.plan_show_execute(helper, helper.plan_to_joint_goal(self.arm.named('home')))
        self.get_logger().info('=== 예제 02 완료 ===')


def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
