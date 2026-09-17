"""중력보상 시뮬레이션의 설정 한 곳.

값은 전부 여기서만 정하고, 나머지 모듈은 이 상수만 참조한다.
"""

from pathlib import Path

#: xacro 로 URDF 를 만들 때 쓰는 인자. ros2_control 은 실기 하드웨어 블록이라 시뮬에선 뺀다.
XACRO_RELATIVE = 'assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro'
XACRO_MAPPINGS = {
    'arm_type': 'v1.0',
    'bimanual': 'true',
    'ros2_control': 'false',
}

#: 마운트(몸통)는 bimanual 일 때만 생기므로 양팔로 뽑은 뒤 한쪽 팔을 지운다.
#: 남길 팔과 지울 팔.
ARM_SIDE = 'left'
PRUNED_SIDE = 'right'

#: 남은 팔의 관절 이름.
ARM_JOINTS = tuple(f'openarm_{ARM_SIDE}_joint{i}' for i in range(1, 8))
FINGER_JOINTS = (f'openarm_{ARM_SIDE}_finger_joint1',
                 f'openarm_{ARM_SIDE}_finger_joint2')

#: ament index 가 없는 환경에서 쓰는 openarm_description 경로.
DESCRIPTION_FALLBACK = Path.home() / 'openarm_ws' / 'src' / 'openarm_description'

#: 생성한 URDF 를 두는 곳.
GENERATED_DIR = Path(__file__).resolve().parent / 'generated'

#: 비교할 세 팔. 이름 · 중력토크에 곱하는 비율 · 화면 라벨 · 색.
#:
#: 비율은 강성이 아니라 앞먹임 토크의 배수다. 1.0 이 URDF 가 말하는 중력토크 전부를
#: 그대로 싣는 것이고, 0.6 이면 그 60%만 싣는다. 화면 글꼴이 ASCII 뿐이라 라벨은 영문.
ARMS = (
    ('under', 0.6, 'gravity comp 60%  (under)', (0.90, 0.35, 0.25, 1.0)),
    ('exact', 1.0, 'gravity comp 100%  (exact)', (0.30, 0.75, 0.40, 1.0)),
    ('over', 1.4, 'gravity comp 140%  (over)', (0.30, 0.50, 0.95, 1.0)),
)

#: 마운트 사이 간격(m). 몸통이 넓어 팔 하나짜리 기둥보다 멀리 떼어 놓는다.
ARM_SPACING = 1.3

#: 마운트를 z 축으로 돌리는 각도(deg). 팔이 뻗는 쪽이 정면으로 오게 한다.
MOUNT_YAW_DEG = -90.0

#: 시작 자세 — 팔을 옆으로 뻗어 중력토크가 크게 걸리는 7관절 값(rad).
START_QPOS = (0.0, -1.45, 0.0, 0.9, 0.0, 0.25, 0.0)

#: 관절 감쇠(Nms/rad). 실물의 마찰을 대신한다. 0 으로 두면 정확보상 팔이
#: 한 번 밀린 뒤 영원히 표류한다 — 물리적으로는 맞지만 보기에 불편하다.
JOINT_DAMPING = 0.4

#: 그리퍼 손가락은 비교 대상이 아니라 잠가 둔다.
LOCK_FINGERS = True

#: 팔끼리 부딪혀 폭발하는 것을 막는다. 바닥과의 접촉도 같이 꺼진다.
ENABLE_CONTACTS = False

#: 시작할 때 보상을 켜 둘지. C 키로 언제든 뒤집는다.
COMP_ENABLED_AT_START = True

#: 재생 속도. 1.0 이 실시간이고, 작을수록 처지고 떠오르는 과정이 천천히 보인다.
REALTIME_FACTOR = 0.5

#: R 로 되돌린 뒤 멈춰 있는 시간(s). 세 팔이 같은 자세에서 출발하는 것을 보여 준다.
RESET_HOLD = 1.0

#: 보상 토크 화살표의 길이(m/Nm).
TORQUE_ARROW_SCALE = 0.06

#: 처음 잡히는 시점 — 방위각(deg) · 고도(deg) · 거리(m) · 바라보는 점(m).
CAMERA = {
    'azimuth': 120.0,
    'elevation': -8.0,
    'distance': 4.2,
    'lookat': (0.25, 0.0, 1.0),
}

#: 오프스크린 렌더 해상도(px). 강의용 캡처·영상이 이 크기로 나온다.
OFFSCREEN_SIZE = (1920, 1080)

#: 적분기 시간 간격(s).
TIMESTEP = 0.002

#: 외력 펄스 — 손끝에 거는 힘(N)과 지속 시간(s).
PUSH_FORCE = 20.0
PUSH_DURATION = 0.4

#: F 를 누를 때마다 방향을 새로 뽑을지. 끄면 늘 PUSH_DIRECTION 으로 민다.
#: 방향은 한 번 뽑아 세 팔에 똑같이 걸린다 — 같은 외란이라야 셋을 견줄 수 있다.
PUSH_RANDOM_DIRECTION = True

#: 고정 방향으로 밀 때 쓰는 방향(월드 좌표).
PUSH_DIRECTION = (0.0, 0.0, -1.0)

#: 뽑은 방향이 수평에 가까우면 팔이 옆으로만 돌아 차이가 잘 안 보인다. 위아래 성분의
#: 최소 크기를 정해 둔다(0 이면 제한 없음, 1 이면 수직).
PUSH_MIN_VERTICAL = 0.35

#: 외력 화살표의 길이(m)와 굵기(m).
PUSH_ARROW_LENGTH = 0.6
PUSH_ARROW_WIDTH = 0.03

#: 미지 페이로드(kg). 키 P 로 켜면 플랜트의 손 링크에만 얹히고 보상 모델은 모른다 —
#: 실물에서 보상이 모자라는 이유를 그대로 재현한다.
PAYLOAD_MASS = 0.8
PAYLOAD_BODY = f'openarm_{ARM_SIDE}_link7'

#: 손끝 힘을 거는 링크.
TIP_BODY = f'openarm_{ARM_SIDE}_link7'
