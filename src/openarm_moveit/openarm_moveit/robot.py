"""OpenArm v1.0 예제 공통 설정 — 프레임·planning group·이름 붙은 자세·토픽.

값을 바꿀 곳은 이 파일 하나다. 다른 모듈은 여기서 가져다 쓴다.
"""

import math

# 모든 목표 pose 와 마커의 기준 프레임. 몸통 밑면이 원점이고 x 가 앞, y 가 왼쪽, z 가 위다.
BASE_FRAME = 'openarm_body_link0'

# 예제가 그리는 RViz 마커 토픽. demo.launch.py 의 RViz 설정에 MarkerArray display 로 들어 있다.
MARKER_TOPIC = '/openarm_markers'

# 단계 진행 신호. `ros2 topic pub --once /next_step std_msgs/msg/Empty '{}'` 로 한 단계씩 넘긴다.
STEP_TOPIC = '/next_step'

# 계획 궤적의 속도·가속도 상한 — 관절 한계(moveit_joint_limits.yaml)에 대한 비율.
# 예제 전부가 이 값을 쓴다. 짧은 이동은 속도 상한에 닿지 못하고 끝나므로
# 체감 속도를 바꾸려면 ACCELERATION_SCALING 을 건드려야 한다.
VELOCITY_SCALING = 0.25
ACCELERATION_SCALING = 0.4

# RViz MotionPlanning 패널의 goal state 를 로봇 현재 자세로 맞추는 신호.
# 패널의 External Comm. 이 켜져 있어야 RViz 가 듣는다 (demo.rviz 에서 켜 둔다).
RVIZ_GOAL_SYNC_TOPIC = '/rviz/moveit/update_goal_state'

# 손끝 방향 (roll, pitch, yaw) [rad].
# TOOL_DOWN — 손끝이 아래를 보되 30° 앞으로 기운 자세. 손목 앞뒤 관절(joint7)이 ±90° 뿐이라
#             수직 아래(roll=π)는 좁은 띠에서만 풀리고, 이 기울기가 도달 범위를 크게 넓힌다.
# TOOL_FORWARD — 손끝이 앞을 보며 위로 든 자세 (hands_up 과 같은 계열).
TOOL_DOWN = (math.pi, -0.5, 0.0)
TOOL_FORWARD = (0.0, -1.0, math.pi)

# 그리퍼 finger_joint1 의 위치 [m]. 0 이 닫힘, 0.044 가 활짝 열림 (URDF 한계).
GRIPPER_OPEN = 0.044
GRIPPER_CLOSED = 0.0
GRIPPER_MAX_EFFORT = 10.0

# SRDF 의 group_state 와 같은 값 (home · hands_up). ready 는 예제용으로 더한 자세다 —
# 팔꿈치를 깊이 굽혀 손끝이 몸 앞 가까이(x≈0.30 m) 오고 관절 한계와 특이자세에서 멀어,
# pose goal 과 Servo 의 출발점으로 쓴다.
NAMED_JOINTS = {
    'left': {
        'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'hands_up': [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
        'ready': [0.2, -0.3, 0.0, 2.0, 0.0, 0.0, 1.0],
    },
    'right': {
        'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'hands_up': [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
        'ready': [-0.2, 0.3, 0.0, 2.0, 0.0, 0.0, -1.0],
    },
}


# 관절 회전축 — URDF 의 <axis>, 관절(child link) 프레임 기준. 예제 01 이 화살표로 그린다.
JOINT_AXES = {
    'left': [(0, 0, 1), (-1, 0, 0), (0, 0, 1), (0, 1, 0), (0, 0, 1), (1, 0, 0), (0, -1, 0)],
    'right': [(0, 0, 1), (-1, 0, 0), (0, 0, 1), (0, 1, 0), (0, 0, 1), (1, 0, 0), (0, 1, 0)],
}


class Arm:
    """한쪽 팔의 이름표 모음. Arm('left') 또는 Arm('right')."""

    def __init__(self, side: str = 'left'):
        if side not in NAMED_JOINTS:
            raise ValueError(f"side 는 'left' 또는 'right' — {side!r}")
        self.side = side
        self.group = f'{side}_arm'                          # SRDF planning group
        self.joints = [f'openarm_{side}_joint{i}' for i in range(1, 8)]
        self.tcp_link = f'openarm_{side}_hand_tcp'          # 손가락 사이 tool center point
        self.finger_joint = f'openarm_{side}_finger_joint1'
        self.gripper_action = f'/{side}_gripper_controller/gripper_cmd'
        self.servo_command_topic = f'/{side}_joint_trajectory_controller/joint_trajectory'

    def named(self, name: str) -> dict:
        """이름 붙은 자세 → {관절 이름: 값}."""
        return dict(zip(self.joints, NAMED_JOINTS[self.side][name]))
