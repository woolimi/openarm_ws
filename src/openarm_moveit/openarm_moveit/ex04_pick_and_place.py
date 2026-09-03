"""예제 04 — pick and place

가상 물체를 Pick 자리에서 집어 Place 자리로 옮긴다. 팔은 매 단계 현재 자세를 seed 로
IK 를 풀어 관절 목표로 움직이고(불필요한 큰 회전 방지), 그리퍼는 GripperCommand 액션으로
여닫는다. 단계는 /next_step 신호로 넘어간다.

실행:
  터미널 1: ros2 launch openarm_moveit demo.launch.py
  터미널 2: ros2 run openarm_moveit ex04_pick_and_place
  터미널 3: ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA

from openarm_moveit.robot import TOOL_DOWN, Arm
from openarm_moveit.utils import GripperHelper, MoveGroupHelper, StepTrigger, make_pose, spin_sleep
from openarm_moveit.viz import ORANGE, MarkerBoard

PICK = {'left': (0.30, 0.10, 0.30), 'right': (0.30, -0.10, 0.30)}     # 물체 위치 [m]
PLACE = {'left': (0.22, 0.32, 0.30), 'right': (0.22, -0.32, 0.30)}    # 내려놓을 위치 [m]
LIFT = 0.10                                                            # 들어올리는 높이 [m]
TABLE = (0.10, 0.10, 0.10)                                             # 받침 마커 크기 [m]
PICK_COLOR = ColorRGBA(r=0.3, g=0.7, b=0.3, a=0.6)
PLACE_COLOR = ColorRGBA(r=0.3, g=0.3, b=0.8, a=0.6)


class PickAndPlaceDemo(Node):

    def __init__(self):
        super().__init__('ex04_pick_and_place')
        self.declare_parameter('arm', 'left')
        self.arm = Arm(self.get_parameter('arm').value)
        self.get_logger().info(f'=== 예제 04: pick and place ({self.arm.side}) ===')
        self.board = MarkerBoard(self)
        self.step = StepTrigger(self)

    def go(self, helper, pose, label: str) -> bool:
        """현재 자세 seed 로 IK → 관절 목표 계획 → 손끝 경로 표시 → 실행."""
        joints = helper.solve_ik(pose)
        if joints is None:
            self.get_logger().warn(f'{label}: IK 실패 — pose goal 로 대신 계획')
            ok, trajectory = helper.plan_to_pose_goal(pose)
        else:
            ok, trajectory = helper.plan_to_joint_goal(joints)
        if not ok or trajectory is None:
            self.get_logger().error(f'{label}: 계획 실패')
            return False
        self.board.put(self.board.line('tcp_path', 0, helper.trajectory_tcp_path(trajectory), ORANGE))
        return helper.execute_trajectory(trajectory)

    def run(self):
        arm = MoveGroupHelper(self, self.arm)
        gripper = GripperHelper(self, self.arm)
        arm.max_velocity_scaling = 0.3
        arm.planning_time = 10.0

        if not arm.wait_for_servers(timeout_sec=30.0) or not gripper.wait_for_server(timeout_sec=30.0):
            return
        if not arm.wait_for_joint_state(timeout_sec=10.0):
            return

        pick, place = PICK[self.arm.side], PLACE[self.arm.side]
        turn = math.pi / 4 if self.arm.side == 'left' else -math.pi / 4      # Place 에서 물체를 45° 돌려 놓는다
        below = lambda p: (p[0], p[1], p[2] - TABLE[2] / 2 - 0.02)        # 받침은 손끝 아래
        self.board.put(self.board.cube('table', 0, below(pick), TABLE, PICK_COLOR),
                       self.board.cube('table', 1, below(place), TABLE, PLACE_COLOR),
                       self.board.text('label', 0, (pick[0], pick[1], pick[2] + 0.08), 'Pick'),
                       self.board.text('label', 1, (place[0], place[1], place[2] + 0.08), 'Place'))

        # 손끝은 항상 아래(TOOL_DOWN). Place 쪽은 yaw 만 turn 만큼 돌린다.
        roll, pitch, yaw = TOOL_DOWN
        pick_pose = make_pose(*pick, roll, pitch, yaw)
        lift_pose = make_pose(pick[0], pick[1], pick[2] + LIFT, roll, pitch, yaw)
        above_pose = make_pose(place[0], place[1], place[2] + LIFT, roll, pitch, yaw + turn)
        place_pose = make_pose(*place, roll, pitch, yaw + turn)

        self.step.wait('1단계 ready 자세 + 그리퍼 열기')
        arm.go_to_named('ready')
        gripper.open()

        self.step.wait('2단계 Pick 위치로')
        if not self.go(arm, pick_pose, 'Pick'):
            return

        self.step.wait('3단계 그리퍼 닫기 (물체 잡기)')
        gripper.close()
        spin_sleep(self, 0.5)

        self.step.wait('4단계 들어올리기')
        self.go(arm, lift_pose, 'Lift')

        self.step.wait('5단계 Place 위로 옮기기')
        self.go(arm, above_pose, 'Transport')

        self.step.wait('6단계 내려놓기')
        self.go(arm, place_pose, 'Place')

        self.step.wait('7단계 그리퍼 열기 (물체 놓기)')
        gripper.open()
        spin_sleep(self, 0.5)

        self.step.wait('8단계 후퇴 + home 복귀')
        self.go(arm, above_pose, 'Retreat')
        arm.go_to_named('home')
        self.get_logger().info('=== 예제 04 완료 ===')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
