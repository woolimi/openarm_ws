"""예제 05 — MoveIt Servo 키보드 teleop

키를 누르는 동안 손끝 속도 명령(TwistStamped)을 30 Hz 로 보내고, Servo 가 매 주기
Jacobian 으로 관절 속도를 풀어 컨트롤러에 넘긴다. 계획 없이 실시간으로 움직인다.

키:
  w/s  앞/뒤 (x)     a/d  왼/오른 (y)     q/e  위/아래 (z)
  W/S  roll ±        A/D  pitch ±         Q/E  yaw ±
  r    ready 자세 복귀     space  정지     ESC  종료

실행:
  터미널 1: ros2 launch openarm_moveit servo.launch.py
  터미널 2: ros2 run openarm_moveit ex05_keyboard_servo
"""

import curses
import math
import time

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

LOOP_SEC = 0.05         # 키 입력을 살피고 twist 를 보내는 주기 (20 Hz)
HOLD_SEC = 0.3          # 마지막 키 입력 뒤 이 시간이 지나면 키를 뗀 것으로 본다
PATH_STEP = 0.005       # 손끝이 이만큼[m] 움직일 때마다 경로 점 추가
PATH_MAX_POINTS = 2000

# 키 → (linear | angular, 축, 부호). 소문자는 이동, 대문자는 회전.
KEY_MAP = {
    ord('w'): ('linear', 'x', 1.0), ord('s'): ('linear', 'x', -1.0),
    ord('a'): ('linear', 'y', 1.0), ord('d'): ('linear', 'y', -1.0),
    ord('q'): ('linear', 'z', 1.0), ord('e'): ('linear', 'z', -1.0),
    ord('W'): ('angular', 'x', 1.0), ord('S'): ('angular', 'x', -1.0),
    ord('A'): ('angular', 'y', 1.0), ord('D'): ('angular', 'y', -1.0),
    ord('Q'): ('angular', 'z', 1.0), ord('E'): ('angular', 'z', -1.0),
}
STATUS_TEXT = {
    -1: '연결 대기', 0: '정상', 1: '감속 (특이점 접근)', 2: '정지 (특이점)',
    3: '감속 (특이점 이탈)', 4: '감속 (충돌 근접)', 5: '정지 (충돌)', 6: '관절 한계',
}


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
        self._last_key_time = 0.0
        self._servo_ready = False

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

    def apply_key(self, key: int) -> None:
        """키 하나 → twist 한 성분. 키보드 자동 반복이 같은 키를 계속 보내므로 매번 다시 채운다."""
        self._reset_twist()
        if key in KEY_MAP:
            kind, axis, sign = KEY_MAP[key]
            vec = self._twist.twist.linear if kind == 'linear' else self._twist.twist.angular
            setattr(vec, axis, sign)
            self._last_key_time = time.time()

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

    # --- curses 화면 -------------------------------------------------------------
    def run_curses(self, screen) -> None:
        """메인 루프 — 키 읽기 → twist 발행 → ROS 콜백 처리 → 화면 갱신, LOOP_SEC 마다."""
        curses.curs_set(0)
        screen.nodelay(True)
        screen.timeout(int(LOOP_SEC * 1000))

        def put(row, col, text, attr=curses.A_NORMAL):
            try:
                screen.addstr(row, col, text, attr)
            except curses.error:
                pass

        while self._running:
            key = screen.getch()                # LOOP_SEC 안에 키가 없으면 -1
            if key == 27:                       # ESC
                self._running = False
                self._reset_twist()
                break
            if key in (ord('r'), ord('R')):
                put(16, 2, '*** ready 자세 복귀 중 ***', curses.A_BOLD)
                screen.refresh()
                self.go_ready()
            elif key != -1:
                self.apply_key(key)
            elif time.time() - self._last_key_time > HOLD_SEC:
                self._reset_twist()             # 키를 뗀 지 HOLD_SEC 이 지나면 정지

            self.publish_twist()
            rclpy.spin_once(self, timeout_sec=0.0)   # status 구독 · 마커 타이머 처리

            screen.erase()
            put(0, 2, f'=== MoveIt Servo 키보드 teleop ({self.arm.side}) ===', curses.A_BOLD)
            put(2, 2, '이동', curses.A_UNDERLINE)
            put(3, 4, 'w/s  앞/뒤 (x)    a/d  왼/오른 (y)    q/e  위/아래 (z)')
            put(5, 2, '회전', curses.A_UNDERLINE)
            put(6, 4, 'W/S  roll ±       A/D  pitch ±        Q/E  yaw ±')
            put(8, 2, '기타', curses.A_UNDERLINE)
            put(9, 4, 'r  ready 자세 복귀    space  정지    ESC  종료')
            lin, ang = self._twist.twist.linear, self._twist.twist.angular
            put(11, 2, '현재 명령', curses.A_UNDERLINE)
            put(12, 4, f'linear  x={lin.x:+.1f} y={lin.y:+.1f} z={lin.z:+.1f}')
            put(13, 4, f'angular x={ang.x:+.1f} y={ang.y:+.1f} z={ang.z:+.1f}')
            state = 'READY' if self._servo_ready else 'NOT READY'
            put(15, 2, f'Servo {state} | 상태: {STATUS_TEXT.get(self._status, self._status)}')
            screen.refresh()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServoNode()

    node.get_logger().info('ready 자세로 이동한 뒤 Servo 를 켭니다...')
    node.go_ready()
    node._servo_ready = node.switch_to_twist()
    if not node._servo_ready:
        node.get_logger().warn('TWIST 모드 전환 실패 — 키를 눌러도 움직이지 않을 수 있습니다')
    node.create_timer(1.0 / 20.0, node.update_markers)

    try:
        curses.wrapper(node.run_curses)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
