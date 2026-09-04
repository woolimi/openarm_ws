"""OpenArm v1.0 양팔 URDF 를 RViz 에 띄우고 슬라이더로 관절을 움직인다.

업스트림 openarm_description 의 display_openarm.launch.py 를 그대로 부르되 이 과정에서 쓰는
값을 기본으로 박아 둔다 — 업스트림 기본값은 arm_type 이 v2.0 이라 그대로 두면 다른 로봇이 뜬다.

    ros2 launch openarm_moveit display.launch.py
    ros2 launch openarm_moveit display.launch.py bimanual:=false   # 한 팔만
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from openarm_leader import stale_processes

#: 형상 보기가 띄우는 노드. 이전 실행이 남아 있으면 RViz 가 둘이 된다.
DISPLAY_PROCESS_NAMES = stale_processes.SESSION_PROCESS_NAMES + ('joint_state_publisher',)


def cleanup_previous_session(_context):
    stale = stale_processes.kill_stale(DISPLAY_PROCESS_NAMES)
    if stale:
        print(f'이전 실행이 남긴 세션 프로세스를 정리했습니다: {stale}')
    return []


def generate_launch_description():
    upstream = os.path.join(
        get_package_share_directory('openarm_description'), 'launch',
        'display_openarm.launch.py')
    return LaunchDescription([
        # arm_type 은 v1.0 고정 — 이 과정은 v1.0 로봇만 쓴다.
        DeclareLaunchArgument(
            'bimanual', default_value='true', choices=['true', 'false'],
            description='true 는 양팔, false 는 왼팔 하나. 한 팔은 world 링크가 없어 '
                        'RViz Fixed Frame 을 openarm_link0 으로 바꿔야 보인다.'),
        OpaqueFunction(function=cleanup_previous_session),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream),
            launch_arguments={
                'arm_type': 'v1.0',
                'bimanual': LaunchConfiguration('bimanual'),
            }.items(),
        ),
    ])
