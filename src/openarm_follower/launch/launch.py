"""OpenArm v1.0 팔로워를 mock 또는 CAN-FD 실기로 띄우는 bringup launch.

노드를 직접 띄우지 않는다. vendored openarm_bringup 의 bimanual launch 를
include 하면서 이 스택에 맞는 인자를 고정하는 것이 이 파일의 전부다.

그 인자 하나가 config/follower.yaml 이다. 중력보상·페이로드·토크 오프셋은 이 로봇
한 대의 실측이라 URDF 가 알 수 없고, bringup 이 하드웨어 블록에 실어 준다.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from openarm_leader import stale_processes


def cleanup_previous_session(_context):
    """이전 실행이 남긴 로봇 세션 프로세스를 거둔다.

    ros2_control_node·robot_state_publisher·rviz2 가 살아 있으면 새 세션과
    같은 이름의 토픽·컨트롤러를 두고 충돌하므로 include 전에 정리한다.
    """
    stale = stale_processes.kill_stale()
    if stale:
        print(f'이전 실행이 남긴 세션 프로세스를 정리했습니다: {stale}')
    return []


def generate_launch_description():
    # 명령줄에서 주는 인자는 이 넷뿐이다. 나머지는 아래 include 에서 고정한다.
    # use_fake_hardware 는 upstream openarm_bringup 의 인자 이름을 그대로 쓴다.
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')
    config_file = LaunchConfiguration('config_file')

    declared_arguments = [
        # true|false 밖의 값은 launch 가 거부한다.
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            choices=['true', 'false'],
            description='true 는 mock_components, false 는 CAN-FD 실기.',
        ),
        # CAN 인터페이스는 false(실기)일 때만 hardware 플러그인에 전달된다.
        DeclareLaunchArgument(
            'left_can_interface',
            default_value='can1',
            description='왼팔 CAN 인터페이스. 실기에서만 쓴다.',
        ),
        DeclareLaunchArgument(
            'right_can_interface',
            default_value='can0',
            description='오른팔 CAN 인터페이스. 실기에서만 쓴다.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('openarm_follower'), 'config', 'follower.yaml',
            ]),
            description='중력보상 설정 파일. 하드웨어 블록의 값이 여기서 온다.',
        ),
    ]

    # vendored openarm_bringup 의 bimanual launch 를 그대로 쓴다.
    # use_fake_hardware 와 CAN 인터페이스는 그대로 넘기고,
    # 컨트롤러는 teleop 이 기대하는 forward_position_controller 로 고정한다.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('openarm_bringup'),
                'launch',
                'openarm.bimanual.launch.py',
            ])
        ),
        launch_arguments={
            # 로봇은 OpenArm v1.0 고정. 명령줄에서 바꿀 수 없다.
            'arm_type': 'v1.0',
            # false 면 URDF 가 mock 대신 OpenArmHW 플러그인을 고른다.
            'use_fake_hardware': use_fake_hardware,
            # /<arm>_forward_position_controller/commands 토픽을 여는 컨트롤러.
            'robot_controller': 'forward_position_controller',
            'left_can_interface': left_can_interface,
            'right_can_interface': right_can_interface,
            # 중력보상·페이로드·토크 오프셋을 하드웨어 블록에 싣는다.
            'hardware_config_file': config_file,
        }.items(),
    )

    # 실행 순서: 인자 선언 → 이전 세션 정리 → bringup include.
    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=cleanup_previous_session), bringup])
