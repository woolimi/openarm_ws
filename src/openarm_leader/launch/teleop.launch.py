"""리더 입력을 팔로워 컨트롤러 명령으로 relay 하는 teleop launch.

팔로워는 openarm_follower 로 따로 띄운다. source 인자로 리더 입력을 고른다.
  sliders  — joint_state_publisher_gui 슬라이더가 리더 역할을 한다
  feetech  — 실제 Feetech 리더암을 읽는다
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from openarm_leader import stale_processes

LEADER_JOINT_STATES_TOPIC = '/leader/joint_states'
LEADER_PROCESS_NAMES = ('leader_node', 'joint_state_publisher_gui')


def cleanup_previous_leader(_context):
    stale = stale_processes.kill_stale(LEADER_PROCESS_NAMES)
    if stale:
        print(f'이전 실행이 남긴 리더 프로세스를 정리했습니다: {stale}')
    return []


def generate_launch_description():
    source = LaunchConfiguration('source')
    arms = LaunchConfiguration('arms')
    config_file = LaunchConfiguration('config_file')

    declared_arguments = [
        DeclareLaunchArgument(
            'source',
            default_value='sliders',
            choices=['sliders', 'feetech'],
            description='리더 입력 소스.',
        ),
        DeclareLaunchArgument(
            'arms',
            default_value='left,right',
            description='teleoperation 대상 팔. 쉼표로 구분한다.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openarm_leader'), 'config', 'leader.yaml']),
            description='리더 설정 파일 경로.',
        ),
    ]

    # 슬라이더 GUI. source 가 sliders 일 때만 뜬다.
    # joint_states 를 /leader/joint_states 로 바꿔
    # 팔로워의 /joint_states 와 섞이지 않게 한다.
    slider_leader = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='leader_joint_state_publisher_gui',
        output='screen',
        remappings=[('joint_states', LEADER_JOINT_STATES_TOPIC)],
        condition=IfCondition(
            PythonExpression(["'", source, "' == 'sliders'"])),
    )

    # 리더 relay 노드. 항상 뜨고, launch 인자 셋을 파라미터로 받는다.
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
    )

    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=cleanup_previous_leader),
           slider_leader, leader_node])
