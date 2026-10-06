"""팔 한 쪽의 관절 한계 — OpenArm v1.0 URDF 에 적힌 값을 모터 축으로.

수동 관절 한계 측정(joint_limit_measure)은 잰 값을 이 한계와 나란히 보여 주고, 관절
구동 테스트(joint_drive_test)는 이 한계 안쪽으로만 관절을 움직인다.

한계는 bringup 과 같은 openarm_description 의 v1.0 양팔 xacro 를 전개해 읽는다. 왼팔
한계는 xacro 가 오른팔의 거울상으로 만든다(J1 오른팔 -80°~200°, 왼팔 -200°~80°).

하드웨어(openarm_hardware)는 관절 모터 위치를 URDF 관절각으로 부호 변환 없이 쓰므로
joint1~7 은 그대로 모터 축 값이다. 그리퍼는 손가락 관절이 직동(0~0.044 m)이라
하드웨어와 같은 비례 변환으로 모터 각(0~-1.0472 rad)으로 옮긴다.
"""

import os
import xml.etree.ElementTree as ET

SIDES = ('right', 'left')
# 모터 순서(CAN ID 1~8)와 같다. 관절 7개 다음이 그리퍼다.
JOINTS = tuple(f'joint{i}' for i in range(1, 8)) + ('gripper',)

# 손가락 관절 값 → 그리퍼 모터 각(rad) 배율. openarm_hardware 의
# joint_to_motor_radians 와 같다: 0 m(닫힘) → 0 rad, 0.044 m(열림) → -1.0472 rad.
GRIPPER_MOTOR_PER_JOINT = -1.0472 / 0.044

# 한계만 읽으므로 하드웨어 블록(ros2_control)은 전개하지 않는다.
XACRO_MAPPINGS = {'arm_type': 'v1.0', 'bimanual': 'true', 'ros2_control': 'false'}


def urdf_xml():
    """설치된 openarm_description 의 v1.0 양팔 URDF."""
    import xacro
    from ament_index_python.packages import get_package_share_directory

    path = os.path.join(
        get_package_share_directory('openarm_description'),
        'assets', 'robot', 'openarm_v1.0', 'urdf', 'openarm_v10.urdf.xacro')
    return xacro.process_file(path, mappings=XACRO_MAPPINGS).toxml()


def side_limits(side, xml=None):
    """side 팔의 관절별 (하한, 상한) rad, 모터 축 기준.

    xml 을 주지 않으면 설치된 URDF 를 전개해 읽는다. 관절이나 그 <limit> 이 없으면
    ValueError — 추측한 한계로 모터를 움직이지 않는다.
    """
    if side not in SIDES:
        raise ValueError(f'알 수 없는 팔 {side!r} ({" · ".join(SIDES)} 중 하나)')
    root = ET.fromstring(xml if xml is not None else urdf_xml())
    described = {}
    for joint in root.iter('joint'):
        limit = joint.find('limit')
        if limit is not None and limit.get('lower') is not None:
            described[joint.get('name')] = (float(limit.get('lower')),
                                             float(limit.get('upper')))

    def lookup(name):
        if name not in described:
            raise ValueError(f'URDF 에 {name} 의 <limit> 이 없습니다')
        return described[name]

    limits = {f'joint{i}': lookup(f'openarm_{side}_joint{i}') for i in range(1, 8)}
    lower, upper = lookup(f'openarm_{side}_finger_joint1')
    # 닫힘 0 m 의 곱이 -0.0 이라 + 0.0 으로 부호 없는 0 을 만든다(표에 -0.0° 로 찍힌다).
    limits['gripper'] = tuple(sorted((lower * GRIPPER_MOTOR_PER_JOINT + 0.0,
                                      upper * GRIPPER_MOTOR_PER_JOINT + 0.0)))
    return limits
