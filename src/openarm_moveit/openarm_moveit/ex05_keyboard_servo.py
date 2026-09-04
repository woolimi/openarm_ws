"""예제 05 — MoveIt Servo 키보드 teleop

누르고 있는 키를 매 주기 읽어 손끝 속도 명령(TwistStamped)을 20 Hz 로 보내고, Servo 가
매 주기 Jacobian 으로 관절 속도를 풀어 컨트롤러에 넘긴다. 계획 없이 실시간으로 움직인다.

입력은 창(pygame)으로 받는다. 터미널은 눌림·뗌이 아니라 문자만 주기 때문이다 — 문자만
보고 있으면 키를 뗀 순간을 알 수 없어 "얼마간 문자가 없으면 뗀 것" 으로 추측해야 하고,
그 추측은 키보드 자동 반복이 시작되기 전(보통 0.5초)과 어긋나 팔이 한 번 멈칫한다. 창은
지금 눌려 있는 키 집합을 그대로 주므로 멈칫이 없고, 키를 떼면 다음 주기에 멈추며,
여러 키를 같이 눌러 대각선으로도 움직일 수 있다. 창이 포커스를 잃으면 명령도 멈춘다.

키:
  w/s  앞/뒤 (x)     a/d  왼/오른 (y)     q/e  위/아래 (z)
  Shift 를 같이 누르면 회전 — w/s roll ±, a/d pitch ±, q/e yaw ±
  r    ready 자세 복귀     space  즉시 정지     ESC  종료

실행:
  터미널 1: ros2 launch openarm_moveit servo.launch.py
  터미널 2: ros2 run openarm_moveit ex05_keyboard_servo
"""

import math

import pygame

import rclpy
import tf2_ros
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool

from openarm_moveit.robot import BASE_FRAME, Arm
from openarm_moveit.utils import MoveGroupHelper
from openarm_moveit.viz import CYAN, GREEN, MarkerBoard

LOOP_HZ = 20            # 키를 읽고 twist 를 보내는 주기
PATH_STEP = 0.005       # 손끝이 이만큼[m] 움직일 때마다 경로 점 추가
PATH_MAX_POINTS = 2000

WINDOW_SIZE = (560, 360)
FONT_SIZE = 18
BG = (24, 26, 32)
FG = (226, 230, 238)
DIM = (128, 136, 150)
ACCENT = (120, 200, 255)
WARN = (255, 170, 90)

#: 키 → (축, 부호). Shift 없이 누르면 이동, 같이 누르면 회전.
KEY_MAP = {
    pygame.K_w: ('x', 1.0), pygame.K_s: ('x', -1.0),
    pygame.K_a: ('y', 1.0), pygame.K_d: ('y', -1.0),
    pygame.K_q: ('z', 1.0), pygame.K_e: ('z', -1.0),
}
STATUS_TEXT = {
    -1: '연결 대기', 0: '정상', 1: '감속 (특이점 접근)', 2: '정지 (특이점)',
    3: '감속 (특이점 이탈)', 4: '감속 (충돌 근접)', 5: '정지 (충돌)', 6: '관절 한계',
}


def direction_from_keys(pressed) -> tuple:
    """눌려 있는 방향키를 합쳐 (x, y, z) 하나로. 아무것도 안 눌렸으면 (0, 0, 0).

    여러 키를 같이 누르면 성분이 더해진다. 그대로 두면 대각선이 축 하나보다 √2 배
    빨라지므로 크기를 1 로 맞춘다 — Servo 는 [-1, 1] 무차원 입력을 축마다 따로
    배율하기 때문이다.
    """
    axes = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    for key, (axis, sign) in KEY_MAP.items():
        if pressed[key]:
            axes[axis] += sign
    vector = (axes['x'], axes['y'], axes['z'])
    norm = math.sqrt(sum(v ** 2 for v in vector))
    return tuple(v / norm for v in vector) if norm > 1.0 else vector


class KeyboardServoNode(Node):

    def __init__(self):
        super().__init__('ex05_keyboard_servo')
        self.declare_parameter('arm', 'left')
        self.arm = Arm(self.get_parameter('arm').value)

        # Servo 입력 — 손끝 속도. frame_id 는 속도를 표현하는 기준 프레임.
        self._twist = TwistStamped()
        self._twist.header.frame_id = BASE_FRAME
        self._twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Servo 서비스 둘(명령 종류 전환 · 일시 정지) · 상태 구독
        self._switch_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self._pause_client = self.create_client(SetBool, '/servo_node/pause_servo')
        self._status = -1
        self.create_subscription(ServoStatus, '/servo_node/status', self._on_status, 10)

        self._running = True
        self._servo_ready = False
        # space 로 세운 뒤에는 방향키를 모두 뗄 때까지 다시 움직이지 않는다.
        self._halted = False

        # 손끝 경로 기록용 TF + 마커
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.board = MarkerBoard(self)
        self._path = []

    def _on_status(self, msg):
        self._status = msg.code

    # --- Servo 준비 ------------------------------------------------------------
    def switch_to_twist(self) -> bool:
        """Servo 를 TWIST 명령 모드로 바꾼다 (기본은 JOINT_JOG)."""
        if not self._switch_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('switch_command_type 서비스가 없습니다')
            return False
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        future = self._switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success

    def pause_servo(self, paused: bool) -> None:
        """Servo 를 멈추거나 다시 켠다. 멈추지 않으면 MoveGroup 의 궤적을 Servo 가 덮어쓴다."""
        if self._pause_client.wait_for_service(timeout_sec=5.0):
            future = self._pause_client.call_async(SetBool.Request(data=paused))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def go_ready(self) -> None:
        """Servo 를 멈춘 뒤 MoveGroup 으로 ready 자세로 옮긴다.
        팔을 쭉 편 home 은 특이자세라 Servo 의 출발점으로 쓰지 않는다."""
        self._reset_twist()
        self.pause_servo(True)
        helper = MoveGroupHelper(self, self.arm)
        if helper.wait_for_servers(timeout_sec=30.0) and helper.wait_for_joint_state(timeout_sec=10.0):
            helper.go_to_named('ready')
        self.pause_servo(False)

    # --- twist 발행 ------------------------------------------------------------
    def _reset_twist(self) -> None:
        for vec in (self._twist.twist.linear, self._twist.twist.angular):
            vec.x = vec.y = vec.z = 0.0

    def publish_twist(self) -> None:
        self._twist.header.stamp = self.get_clock().now().to_msg()
        self._twist_pub.publish(self._twist)

    def apply_keys(self, pressed, shift: bool) -> None:
        """지금 눌려 있는 키 집합 전체 → twist 하나. Shift 면 회전, 아니면 이동."""
        self._reset_twist()
        x, y, z = direction_from_keys(pressed)
        if (x, y, z) == (0.0, 0.0, 0.0):
            self._halted = False        # 다 뗐으니 space 정지도 풀린다
            return
        if self._halted:
            return
        vec = self._twist.twist.angular if shift else self._twist.twist.linear
        vec.x, vec.y, vec.z = x, y, z

    # --- 손끝 경로 · 명령 방향 마커 ------------------------------------------------
    def tcp_position(self):
        try:
            t = self._tf_buffer.lookup_transform(BASE_FRAME, self.arm.tcp_link, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)

    def update_markers(self) -> None:
        """20 Hz — 손끝 위치를 경로에 쌓고, 누르고 있는 이동 방향을 화살표로 그린다."""
        tcp = self.tcp_position()
        if tcp is None:
            return
        if not self._path or math.dist(tcp, self._path[-1]) > PATH_STEP:
            self._path.append(tcp)
            self._path = self._path[-PATH_MAX_POINTS:]
        markers = []
        if len(self._path) >= 2:
            markers.append(self.board.line('servo_path', 0, self._path, GREEN, width=0.004))
        lin = self._twist.twist.linear
        if abs(lin.x) + abs(lin.y) + abs(lin.z) > 0.0:
            end = (tcp[0] + lin.x * 0.15, tcp[1] + lin.y * 0.15, tcp[2] + lin.z * 0.15)
            markers.append(self.board.segment('servo_cmd', 0, tcp, end, CYAN))
        else:
            self.board.clear('servo_cmd')
        if markers:
            self.board.put(*markers)

    # --- 창 --------------------------------------------------------------------
    def draw(self, screen, font, focused: bool = True, busy: str = '') -> None:
        screen.fill(BG)

        def put(row, text, color=FG, indent=0):
            screen.blit(font.render(text, True, color), (16 + indent, 12 + row * 24))

        put(0, f'MoveIt Servo 키보드 teleop — {self.arm.side}', ACCENT)
        put(1, '이동', DIM)
        put(2, 'w/s  앞/뒤 (x)    a/d  왼/오른 (y)    q/e  위/아래 (z)', indent=16)
        put(3, '회전 — Shift 를 같이', DIM)
        put(4, 'W/S  roll ±       A/D  pitch ±        Q/E  yaw ±', indent=16)
        put(5, '기타', DIM)
        put(6, 'r  ready 자세 복귀    space  정지    ESC  종료', indent=16)

        lin, ang = self._twist.twist.linear, self._twist.twist.angular
        put(8, '현재 명령', DIM)
        put(9, f'linear  x={lin.x:+.2f} y={lin.y:+.2f} z={lin.z:+.2f}', indent=16)
        put(10, f'angular x={ang.x:+.2f} y={ang.y:+.2f} z={ang.z:+.2f}', indent=16)

        state = 'READY' if self._servo_ready else 'NOT READY'
        put(12, f'Servo {state} | 상태: {STATUS_TEXT.get(self._status, self._status)}')
        if not focused:
            put(13, '창이 포커스를 잃었습니다 — 클릭하면 다시 받습니다', WARN)
        elif self._halted:
            put(13, '정지 — 방향키를 모두 떼면 풀립니다', WARN)
        if busy:
            put(13, busy, WARN)
        pygame.display.flip()

    def run_window(self, screen, font, clock) -> None:
        """메인 루프 — 눌린 키 읽기 → twist 발행 → ROS 콜백 처리 → 창 갱신."""
        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self._running = False
                    elif event.key == pygame.K_SPACE:
                        self._halted = True
                    elif event.key == pygame.K_r:
                        self._reset_twist()
                        self.publish_twist()
                        self.draw(screen, font, busy='*** ready 자세 복귀 중 ***')
                        self.go_ready()

            focused = pygame.key.get_focused()
            if focused:
                shift = bool(pygame.key.get_mods() & pygame.KMOD_SHIFT)
                self.apply_keys(pygame.key.get_pressed(), shift)
            else:
                # 창 밖에서 친 키가 팔을 움직이면 안 된다.
                self._reset_twist()

            self.publish_twist()
            rclpy.spin_once(self, timeout_sec=0.0)   # status 구독 · 마커 타이머 처리
            self.draw(screen, font, focused)
            clock.tick(LOOP_HZ)

        self._reset_twist()
        self.publish_twist()


def open_window():
    """창과 글꼴을 연다. 화면이 없으면 (None, None, None)."""
    try:
        pygame.display.init()
        pygame.font.init()
        screen = pygame.display.set_mode(WINDOW_SIZE)
    except pygame.error as error:
        print(f'창을 열 수 없습니다 ({error}). 화면이 있는 데스크톱에서 실행하세요.')
        return None, None, None
    pygame.display.set_caption('OpenArm Servo teleop')
    # 자동 반복은 쓰지 않는다 — 지금 눌려 있는 키 집합만 본다.
    pygame.key.set_repeat(0)
    # 한글이 나오는 글꼴을 시스템에서 찾는다. 없으면 기본 글꼴이라 한글이 네모로 보인다.
    font_path = pygame.font.match_font('nanumgothic,notosanscjkkr,nanumbarungothic')
    font = (pygame.font.Font(font_path, FONT_SIZE) if font_path
            else pygame.font.SysFont(None, FONT_SIZE + 4))
    return screen, font, pygame.time.Clock()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServoNode()

    screen, font, clock = open_window()
    if screen is None:
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    node.get_logger().info('ready 자세로 이동한 뒤 Servo 를 켭니다...')
    node.go_ready()
    node._servo_ready = node.switch_to_twist()
    if not node._servo_ready:
        node.get_logger().warn('TWIST 모드 전환 실패 — 키를 눌러도 움직이지 않을 수 있습니다')
    node.create_timer(1.0 / LOOP_HZ, node.update_markers)

    try:
        node.run_window(screen, font, clock)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        pygame.quit()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
