"""OpenArm v1.0 팔로워를 mock 또는 CAN-FD 실기로 띄우는 bringup launch."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)

from launch_ros.substitutions import FindPackageShare

from openarm_leader import stale_processes


def cleanup_previous_session(_context):
    stale = stale_processes.kill_stale()
    if stale:
        print(f'이전 실행이 남긴 세션 프로세스를 정리했습니다: {stale}')
    return []


def generate_launch_description():
    hardware = LaunchConfiguration('hardware')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')

    declared_arguments = [
        DeclareLaunchArgument(
            'hardware',
            default_value='sim',
            choices=['sim', 'real'],
            description='sim 은 mock_components, real 은 CAN-FD 실기.',
        ),
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
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('openarm_bringup'),
                'launch',
                'openarm.bimanual.launch.py',
            ])
        ),
        launch_arguments={
            'arm_type': 'v1.0',
            'use_fake_hardware': PythonExpression(
                ["'false' if '", hardware, "' == 'real' else 'true'"]),
            'robot_controller': 'forward_position_controller',
            'left_can_interface': left_can_interface,
            'right_can_interface': right_can_interface,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=cleanup_previous_session), bringup])
