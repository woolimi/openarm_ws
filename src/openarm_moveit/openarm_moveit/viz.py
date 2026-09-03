"""RViz 마커 — 목표점·경로·물체를 MarkerArray 하나로 그린다.

MarkerBoard 는 (ns, id) 로 마커를 기억해 두고, 같은 자리에 다시 그리면 바꿔 끼운다.
publisher 는 TRANSIENT_LOCAL 이라 RViz display 를 나중에 켜도 마지막 그림이 보인다.
"""

from typing import Iterable, Optional, Sequence, Tuple

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from openarm_moveit.robot import BASE_FRAME, MARKER_TOPIC

XYZ = Tuple[float, float, float]

# 색 이름표 — 예제 전체가 같은 뜻으로 쓴다
YELLOW = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)   # 대기 · 시작점
BLUE = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.9)      # 진행 중
GREEN = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9)     # 성공
RED = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9)       # 실패
ORANGE = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)    # 계획된 손끝 경로
CYAN = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.9)      # 미리보기 경로
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def _base(ns: str, marker_id: int, marker_type: int, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


class MarkerBoard:
    """MarkerArray 를 들고 있다가 통째로 발행한다."""

    def __init__(self, node: Node, topic: str = MARKER_TOPIC):
        self.node = node
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub = node.create_publisher(MarkerArray, topic, qos)
        self._markers = {}      # (ns, id) → Marker
        self.clear_all()        # 이전 예제가 남긴 마커를 지우고 시작한다

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    # --- 마커 만들기 ----------------------------------------------------
    def sphere(self, ns: str, marker_id: int, xyz: XYZ, color: ColorRGBA, size: float = 0.04) -> Marker:
        m = _base(ns, marker_id, Marker.SPHERE, self._stamp())
        m.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        m.scale = Vector3(x=size, y=size, z=size)
        m.color = color
        return m

    def arrow(self, ns: str, marker_id: int, pose: Pose, color: ColorRGBA, length: float = 0.10) -> Marker:
        """pose 의 z 축(접근 방향)을 따라 뻗는 화살표. pose 를 그대로 마커 pose 로 쓰고 x 축을 z 로 돌린다."""
        m = _base(ns, marker_id, Marker.ARROW, self._stamp())
        m.pose = pose
        m.scale = Vector3(x=length, y=0.015, z=0.015)
        m.color = color
        return m

    def segment(self, ns: str, marker_id: int, start: XYZ, end: XYZ, color: ColorRGBA,
                shaft: float = 0.008) -> Marker:
        """start 에서 end 로 가는 화살표."""
        m = _base(ns, marker_id, Marker.ARROW, self._stamp())
        m.points = [Point(x=start[0], y=start[1], z=start[2]), Point(x=end[0], y=end[1], z=end[2])]
        m.scale = Vector3(x=shaft, y=shaft * 2.5, z=shaft * 3.0)
        m.color = color
        return m

    def text(self, ns: str, marker_id: int, xyz: XYZ, label: str, color: ColorRGBA = WHITE,
             height: float = 0.04) -> Marker:
        m = _base(ns, marker_id, Marker.TEXT_VIEW_FACING, self._stamp())
        m.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        m.scale.z = height
        m.color = color
        m.text = label
        return m

    def line(self, ns: str, marker_id: int, points: Iterable[XYZ], color: ColorRGBA,
             width: float = 0.006) -> Marker:
        """점들을 잇는 선 (LINE_STRIP)."""
        m = _base(ns, marker_id, Marker.LINE_STRIP, self._stamp())
        m.scale.x = width
        m.color = color
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        return m

    def dots(self, ns: str, marker_id: int, points: Iterable[XYZ], color: ColorRGBA,
             size: float = 0.02) -> Marker:
        """점마다 작은 구 (SPHERE_LIST)."""
        m = _base(ns, marker_id, Marker.SPHERE_LIST, self._stamp())
        m.scale = Vector3(x=size, y=size, z=size)
        m.color = color
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        return m

    def cube(self, ns: str, marker_id: int, xyz: XYZ, size: XYZ, color: ColorRGBA) -> Marker:
        m = _base(ns, marker_id, Marker.CUBE, self._stamp())
        m.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        m.scale = Vector3(x=size[0], y=size[1], z=size[2])
        m.color = color
        return m

    # --- 올리기 · 지우기 ----------------------------------------------------
    def put(self, *markers: Marker) -> None:
        """같은 (ns, id) 가 있으면 바꿔 끼우고 발행한다."""
        for m in markers:
            self._markers[(m.ns, m.id)] = m
        self.publish()

    def recolor(self, ns: str, color: ColorRGBA) -> None:
        """ns 가 같은 마커 전부의 색을 바꾼다 (성공 초록 · 실패 빨강)."""
        for (namespace, _), m in self._markers.items():
            if namespace == ns:
                m.color = color
                m.header.stamp = self._stamp()
        self.publish()

    def clear(self, ns_prefix: Optional[str] = None) -> None:
        """ns 가 prefix 로 시작하는 마커를 RViz 에서도 지운다. prefix 가 없으면 전부."""
        gone = MarkerArray()
        for (namespace, marker_id) in list(self._markers):
            if ns_prefix is None or namespace.startswith(ns_prefix):
                m = Marker()
                m.header.frame_id = BASE_FRAME
                m.ns = namespace
                m.id = marker_id
                m.action = Marker.DELETE
                gone.markers.append(m)
                del self._markers[(namespace, marker_id)]
        if gone.markers:
            self._pub.publish(gone)

    def clear_all(self) -> None:
        """RViz 에 남아 있는 이 토픽의 마커를 전부 지운다 (DELETEALL)."""
        m = Marker()
        m.header.frame_id = BASE_FRAME
        m.action = Marker.DELETEALL
        self._pub.publish(MarkerArray(markers=[m]))
        self._markers = {}

    def publish(self) -> None:
        array = MarkerArray()
        array.markers = list(self._markers.values())
        self._pub.publish(array)

    # --- 자주 쓰는 묶음 ------------------------------------------------------
    def target(self, marker_id: int, pose: Pose, label: str, color: ColorRGBA) -> None:
        """목표 pose 하나 = 화살표(접근 방향) + 구(위치) + 글자."""
        p = pose.position
        self.put(self.arrow('target_arrow', marker_id, pose, color),
                 self.sphere('target_sphere', marker_id, (p.x, p.y, p.z), color),
                 self.text('target_text', marker_id, (p.x, p.y, p.z + 0.08), label))

    def path(self, ns: str, points: Sequence[XYZ], color: ColorRGBA, label: Optional[str] = None) -> None:
        """경로 = 선 + 점 + (시작점 큰 구 + 라벨)."""
        markers = [self.line(f'{ns}_line', 0, points, color), self.dots(f'{ns}_dots', 0, points, color)]
        if label and points:
            markers.append(self.sphere(f'{ns}_start', 0, points[0], YELLOW))
            markers.append(self.text(f'{ns}_label', 0,
                                     (points[0][0], points[0][1], points[0][2] + 0.06), label))
        self.put(*markers)
