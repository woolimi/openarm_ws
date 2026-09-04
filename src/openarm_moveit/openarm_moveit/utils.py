"""MoveIt 을 rclpy 로 직접 부르는 헬퍼.

move_group 이 여는 인터페이스를 그대로 쓴다.
  - MoveGroup 액션(/move_action)            — 계획 + 실행, 또는 plan_only
  - ExecuteTrajectory 액션(/execute_trajectory) — 이미 계획한 궤적만 실행
  - /compute_cartesian_path 서비스          — 손끝 직선 보간
  - /compute_ik · /compute_fk 서비스        — 역기구학 · 정기구학
  - GripperCommand 액션(/<side>_gripper_controller/gripper_cmd) — 그리퍼
"""

import copy
import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import tf_transformations
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty

from openarm_moveit.robot import (
    ACCELERATION_SCALING, BASE_FRAME, GRIPPER_CLOSED, GRIPPER_MAX_EFFORT,
    GRIPPER_OPEN, RVIZ_GOAL_SYNC_TOPIC, STEP_TOPIC, VELOCITY_SCALING, Arm,
)


# ------------------------------------------------------------------
#  Pose 만들기
# ------------------------------------------------------------------

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """오일러 각(rad) → Quaternion 메시지."""
    q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def make_pose(x: float, y: float, z: float,
              roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Pose:
    """위치(m) + 오일러 각(rad) → Pose 메시지."""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = euler_to_quaternion(roll, pitch, yaw)
    return pose


def spin_sleep(node: Node, seconds: float) -> None:
    """콜백을 처리하면서 기다린다 (joint_states 구독이 멈추지 않게)."""
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


# ------------------------------------------------------------------
#  단계 진행 신호
# ------------------------------------------------------------------

class StepTrigger:
    """/next_step 토픽에 Empty 가 오면 다음 단계로 넘어간다."""

    def __init__(self, node: Node):
        self.node = node
        self._received = False
        node.create_subscription(Empty, STEP_TOPIC, self._on_message, 10)

    def _on_message(self, _msg):
        self._received = True

    def wait(self, prompt: str) -> None:
        """신호가 올 때까지 spin 하며 기다린다."""
        self.node.get_logger().info(f'>>> [대기] {prompt} — 신호를 기다리는 중...')
        self._received = False
        while rclpy.ok() and not self._received:
            rclpy.spin_once(self.node, timeout_sec=0.1)


# ------------------------------------------------------------------
#  MoveGroupHelper — 팔 하나
# ------------------------------------------------------------------

def error_name(code: int) -> str:
    """MoveIt error_code 를 이름과 함께 읽을 수 있게 만든다.

    숫자만 찍으면 매번 헤더를 뒤져야 한다. 자주 보게 되는 것들:
    -1 PLANNING_FAILED, -10 START_STATE_IN_COLLISION, -12 GOAL_IN_COLLISION,
    -26 START_STATE_INVALID(현재 자세가 관절 한계 밖 — 실기에서 흔하다),
    -31 NO_IK_SOLUTION.
    """
    for name, value in vars(MoveItErrorCodes).items():
        if name.isupper() and value == code:
            return f'{code} ({name})'
    return str(code)


class MoveGroupHelper:
    """MoveGroup 액션과 관련 서비스를 감싼다. 팔 하나(Arm)를 다룬다."""

    def __init__(self, node: Node, arm: Arm):
        self.node = node
        self.arm = arm
        self.logger = node.get_logger()

        # 현재 관절 상태 (joint_states 구독)
        self._joint_state: Optional[JointState] = None
        node.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        # 액션 클라이언트 둘
        self._move_client = ActionClient(node, MoveGroup, '/move_action')
        self._execute_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')

        # 서비스 클라이언트 셋
        self._cartesian_client = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._ik_client = node.create_client(GetPositionIK, '/compute_ik')
        self._fk_client = node.create_client(GetPositionFK, '/compute_fk')

        # 실행이 끝날 때마다 RViz 의 goal state 를 로봇 자세로 끌어온다
        self._goal_sync_pub = node.create_publisher(Empty, RVIZ_GOAL_SYNC_TOPIC, 1)

        # 계획 파라미터 — 예제마다 바꿔 쓴다
        self.planning_time = 5.0
        self.num_planning_attempts = 5
        self.max_velocity_scaling = VELOCITY_SCALING
        self.max_acceleration_scaling = ACCELERATION_SCALING

    def _on_joint_state(self, msg: JointState):
        self._joint_state = msg

    def sync_rviz_goal_state(self) -> None:
        """RViz 의 goal state(파란 팔·interactive marker·Joints 탭 슬라이더)를
        로봇의 현재 자세로 맞춘다. 실행 뒤 목표 표시가 옛 자리에 남지 않게 한다."""
        self._goal_sync_pub.publish(Empty())

    # --- 준비 대기 ------------------------------------------------
    def wait_for_servers(self, timeout_sec: float = 10.0) -> bool:
        """move_group 의 액션 서버 둘이 뜰 때까지 기다린다."""
        self.logger.info('MoveGroup 액션 서버 연결 대기 중...')
        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            self.logger.error('MoveGroup 액션 서버에 연결할 수 없습니다!')
            return False
        if not self._execute_client.wait_for_server(timeout_sec=timeout_sec):
            self.logger.error('ExecuteTrajectory 액션 서버에 연결할 수 없습니다!')
            return False
        self.logger.info('액션 서버 연결 완료')
        return True

    def wait_for_joint_state(self, timeout_sec: float = 10.0) -> bool:
        """첫 joint_states 가 올 때까지 기다린다."""
        start = time.time()
        while self._joint_state is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.logger.error('joint_states 를 받지 못했습니다!')
                return False
        return True

    # --- 현재 상태 ------------------------------------------------
    def current_joint_values(self) -> dict:
        """이 팔 관절 7개의 현재 값 {이름: rad}."""
        if self._joint_state is None:
            return {}
        return {name: self._joint_state.position[i]
                for i, name in enumerate(self._joint_state.name)
                if name in self.arm.joints}

    def current_robot_state(self) -> RobotState:
        """현재 joint_states 를 RobotState 로 감싼다 (start_state · IK seed 용)."""
        state = RobotState()
        if self._joint_state is not None:
            state.joint_state = self._joint_state
        return state

    # --- MotionPlanRequest → MoveGroup.Goal --------------------------
    def _build_request(self) -> MotionPlanRequest:
        """group·시도 횟수·시간·속도 스케일을 채운 빈 요청."""
        req = MotionPlanRequest()
        req.group_name = self.arm.group
        req.num_planning_attempts = self.num_planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.max_velocity_scaling
        req.max_acceleration_scaling_factor = self.max_acceleration_scaling
        req.start_state.is_diff = True         # 빈 diff = 현재 자세에서 출발
        return req

    @staticmethod
    def _joint_constraints(joint_values: dict, tolerance: float = 0.01) -> Constraints:
        """{관절: 값} → JointConstraint 묶음."""
        constraints = Constraints()
        for name, value in joint_values.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def _pose_constraints(self, pose: Pose, link: str,
                          position_tol: float = 0.01, orientation_tol: float = 0.01) -> Constraints:
        """손끝 Pose → PositionConstraint(반지름 1 cm 구) + OrientationConstraint."""
        constraints = Constraints()

        # 위치 제약 — 목표점을 중심으로 한 작은 구 안에 link 원점이 들어와야 한다
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tol]
        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(Pose(position=copy.deepcopy(pose.position),
                                           orientation=Quaternion(w=1.0)))
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = link
        pc.target_point_offset = Vector3()
        pc.constraint_region = region
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        # 방향 제약 — 세 축 각각 orientation_tol rad 안
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = link
        oc.orientation = copy.deepcopy(pose.orientation)
        oc.absolute_x_axis_tolerance = orientation_tol
        oc.absolute_y_axis_tolerance = orientation_tol
        oc.absolute_z_axis_tolerance = orientation_tol
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        return constraints

    def _send_goal(self, request: MotionPlanRequest,
                   plan_only: bool) -> Tuple[bool, Optional[RobotTrajectory]]:
        """MoveGroup.Goal 을 보내고 결과를 기다린다. (성공 여부, 계획된 궤적)."""
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only     # True 면 로봇은 움직이지 않는다
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.logger.error('MoveGroup 목표가 거부되었습니다!')
            return False, None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.logger.error(f'MoveGroup 실패 — error_code {error_name(code)}')
            return False, None
        if not plan_only:
            self.sync_rviz_goal_state()
        return True, result.planned_trajectory

    # --- joint goal ----------------------------------------------------
    def go_to_joint_goal(self, joint_values: dict) -> bool:
        """관절 목표로 계획 + 실행."""
        req = self._build_request()
        req.goal_constraints.append(self._joint_constraints(joint_values))
        ok, _ = self._send_goal(req, plan_only=False)
        return ok

    def plan_to_joint_goal(self, joint_values: dict) -> Tuple[bool, Optional[RobotTrajectory]]:
        """관절 목표로 계획만."""
        req = self._build_request()
        req.goal_constraints.append(self._joint_constraints(joint_values))
        return self._send_goal(req, plan_only=True)

    def go_to_named(self, name: str) -> bool:
        """이름 붙은 자세(home · hands_up · ready)로 이동."""
        self.logger.info(f"'{name}' 자세로 이동")
        return self.go_to_joint_goal(self.arm.named(name))

    # --- pose goal -----------------------------------------------------
    def go_to_pose_goal(self, pose: Pose) -> bool:
        """손끝(tcp) Pose 목표로 계획 + 실행. IK 는 move_group 안에서 돈다."""
        req = self._build_request()
        req.goal_constraints.append(self._pose_constraints(pose, self.arm.tcp_link))
        ok, _ = self._send_goal(req, plan_only=False)
        return ok

    def plan_to_pose_goal(self, pose: Pose) -> Tuple[bool, Optional[RobotTrajectory]]:
        """손끝 Pose 목표로 계획만."""
        req = self._build_request()
        req.goal_constraints.append(self._pose_constraints(pose, self.arm.tcp_link))
        return self._send_goal(req, plan_only=True)

    # --- IK · FK 서비스 -------------------------------------------------
    def solve_ik(self, pose: Pose, seed: Optional[RobotState] = None) -> Optional[dict]:
        """/compute_ik 로 손끝 Pose 의 관절값을 푼다. seed 가 없으면 현재 자세에서 시작."""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('compute_ik 서비스를 찾을 수 없습니다!')
            return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.arm.group
        req.ik_request.ik_link_name = self.arm.tcp_link
        req.ik_request.robot_state = seed or self.current_robot_state()
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = BASE_FRAME
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.avoid_collisions = True

        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        resp = future.result()
        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        return {name: resp.solution.joint_state.position[i]
                for i, name in enumerate(resp.solution.joint_state.name)
                if name in self.arm.joints}

    def trajectory_tcp_path(self, trajectory: RobotTrajectory) -> List[Tuple[float, float, float]]:
        """궤적의 점마다 /compute_fk 를 불러 손끝 위치 목록을 만든다 (경로 미리보기용)."""
        if not self._fk_client.wait_for_service(timeout_sec=5.0):
            self.logger.warn('compute_fk 서비스 없음 — 경로 표시 생략')
            return []
        joint_traj = trajectory.joint_trajectory
        points = []
        for point in joint_traj.points:
            req = GetPositionFK.Request()
            req.header.frame_id = BASE_FRAME
            req.fk_link_names = [self.arm.tcp_link]
            req.robot_state.joint_state.name = list(joint_traj.joint_names)
            req.robot_state.joint_state.position = list(point.positions)
            future = self._fk_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            resp = future.result()
            if resp is not None and resp.error_code.val == MoveItErrorCodes.SUCCESS and resp.pose_stamped:
                p = resp.pose_stamped[0].pose.position
                points.append((p.x, p.y, p.z))
        return points

    # --- Cartesian path ------------------------------------------------
    def compute_cartesian_path(self, waypoints: List[Pose], max_step: float = 0.01,
                               avoid_collisions: bool = True) -> Tuple[Optional[RobotTrajectory], float]:
        """/compute_cartesian_path — waypoints 를 직선으로 잇는 궤적과 fraction(달성률)."""
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.logger.error('compute_cartesian_path 서비스를 찾을 수 없습니다!')
            return None, 0.0
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.group_name = self.arm.group
        req.link_name = self.arm.tcp_link
        req.waypoints = list(waypoints)
        req.max_step = max_step                      # 직선을 자르는 간격 [m]
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = self.max_velocity_scaling
        req.max_acceleration_scaling_factor = self.max_acceleration_scaling
        req.start_state = self.current_robot_state()

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        resp = future.result()
        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.error('Cartesian 경로 계획 실패')
            return None, 0.0
        self.logger.info(f'Cartesian 경로 달성률 {resp.fraction * 100:.1f}%')
        return resp.solution, resp.fraction

    # --- 실행 ------------------------------------------------------------
    def execute_trajectory(self, trajectory: RobotTrajectory) -> bool:
        """ExecuteTrajectory 액션으로 계획된 궤적을 실행한다."""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.logger.error('궤적 실행 목표가 거부되었습니다!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        code = result_future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.logger.error(f'궤적 실행 실패 — error_code {error_name(code)}')
            return False
        self.sync_rviz_goal_state()
        return True


# ------------------------------------------------------------------
#  GripperHelper
# ------------------------------------------------------------------

class GripperHelper:
    """GripperCommand 액션으로 finger_joint1 위치를 지령한다."""

    def __init__(self, node: Node, arm: Arm):
        self.node = node
        self.logger = node.get_logger()
        self._client = ActionClient(node, GripperCommand, arm.gripper_action)

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.logger.error('그리퍼 액션 서버에 연결할 수 없습니다!')
            return False
        return True

    def move(self, position: float, max_effort: float = GRIPPER_MAX_EFFORT) -> bool:
        """finger_joint1 을 position[m] 으로. 0 이 닫힘, 0.044 가 열림."""
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.logger.error('그리퍼 명령이 거부되었습니다!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        self.logger.info(f'그리퍼 위치 {result.position:.3f} m'
                         f'{" (물체에 걸림)" if result.stalled else ""}')
        return result.reached_goal or result.stalled

    def open(self) -> bool:
        self.logger.info('그리퍼 열기')
        return self.move(GRIPPER_OPEN)

    def close(self) -> bool:
        self.logger.info('그리퍼 닫기')
        return self.move(GRIPPER_CLOSED)
