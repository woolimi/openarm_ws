"""팔로워 bringup 과 리더 relay 를 한 번에 띄우는 실습용 launch.

source 인자 하나로 실습 단계를 넘긴다.
  none     — 팔로워와 RViz 만 띄운다
  sliders  — joint_state_publisher_gui 슬라이더가 리더 역할을 한다
  feetech  — 실제 Feetech 리더암을 읽는다
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LEADER_JOINT_STATES_TOPIC = '/leader/joint_states'


def generate_launch_description():
    source = LaunchConfiguration('source')
    arms = LaunchConfiguration('arms')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    config_file = LaunchConfiguration('config_file')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')

    declared_arguments = [
        DeclareLaunchArgument(
            'source',
            default_value='sliders',
            choices=['none', 'sliders', 'feetech'],
            description='리더 입력 소스.',
        ),
        DeclareLaunchArgument(
            'arms',
            default_value='left,right',
            description='teleoperation 대상 팔. 쉼표로 구분한다.',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            choices=['true', 'false'],
            description='true 면 mock_components, false 면 CAN-FD 실기.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openarm_leader'), 'config', 'leader.yaml']),
            description='리더 설정 파일 경로.',
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

    follower_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('openarm_bringup'),
                'launch',
                'openarm.bimanual.launch.py',
            ])
        ),
        launch_arguments={
            'arm_type': 'v1.0',
            'use_fake_hardware': use_fake_hardware,
            'robot_controller': 'forward_position_controller',
            'left_can_interface': left_can_interface,
            'right_can_interface': right_can_interface,
        }.items(),
    )

    slider_leader = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='leader_joint_state_publisher_gui',
        output='screen',
        remappings=[('joint_states', LEADER_JOINT_STATES_TOPIC)],
        condition=IfCondition(PythonExpression(["'", source, "' == 'sliders'"])),
    )

    leader_node = Node(
        package='openarm_leader',
        executable='leader_node',
        name='leader_node',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'source': source,
            'arms': arms,
        }],
        condition=UnlessCondition(PythonExpression(["'", source, "' == 'none'"])),
    )

    return LaunchDescription(
        declared_arguments + [follower_bringup, slider_leader, leader_node])
