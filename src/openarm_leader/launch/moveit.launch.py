"""OpenArm v1.0 MoveIt demo 를 mock hardware 위에 띄우는 실습용 launch.

업스트림 openarm_bimanual_moveit_config 의 v1.0 구성을 그대로 쓰되, joint_limits 만
acceleration 한계가 채워진 이 패키지의 moveit_joint_limits.yaml 로 바꿔 조립한다.
"""

import os

import xacro

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

CONFIG_DIR = 'openarm_v1.0'
XACRO_MAPPINGS = {
    'arm_type': 'v1.0',
    'bimanual': 'true',
    'use_fake_hardware': 'true',
    'ros2_control': 'true',
}


def moveit_nodes(_context):
    description_path = get_package_share_directory('openarm_description')
    moveit_path = get_package_share_directory('openarm_bimanual_moveit_config')
    bringup_path = get_package_share_directory('openarm_bringup')
    leader_path = get_package_share_directory('openarm_leader')

    xacro_path = os.path.join(
        description_path, 'assets', 'robot', CONFIG_DIR,
        'urdf', 'openarm_v10.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_path, mappings=XACRO_MAPPINGS).toprettyxml(indent='  ')
    controllers_file = os.path.join(
        bringup_path, 'config', 'controllers',
        'openarm_bimanual_moveit_controllers.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[{'robot_description': robot_description}, controllers_file],
    )

    moveit_config = (
        MoveItConfigsBuilder(
            'openarm', package_name='openarm_bimanual_moveit_config')
        .robot_description(file_path=xacro_path, mappings=XACRO_MAPPINGS)
        .robot_description_semantic(
            file_path=f'config/{CONFIG_DIR}/openarm_bimanual.srdf')
        .robot_description_kinematics(
            file_path=f'config/{CONFIG_DIR}/kinematics.yaml')
        .joint_limits(
            file_path=os.path.join(
                leader_path, 'config', 'moveit_joint_limits.yaml'))
        .trajectory_execution(
            file_path=f'config/{CONFIG_DIR}/moveit_controllers.yaml')
        .planning_pipelines(
            pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()

    pilz_path = os.path.join(
        moveit_path, 'config', CONFIG_DIR, 'pilz_cartesian_limits.yaml')
    if os.path.exists(pilz_path):
        with open(pilz_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        if 'cartesian_limits' in data:
            moveit_params.setdefault(
                'robot_description_planning', {}).update(data)

    rviz_config = os.path.join(
        moveit_path, 'config', CONFIG_DIR, 'moveit.rviz')
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_params],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[moveit_params],
    )
    return [robot_state_publisher, control_node, move_group, rviz]


def generate_launch_description():
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_joint_trajectory_controller',
                   'right_joint_trajectory_controller',
                   '-c', '/controller_manager'],
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller', 'right_gripper_controller',
                   '-c', '/controller_manager'],
    )
    return LaunchDescription([
        OpaqueFunction(function=moveit_nodes),
        TimerAction(period=2.0, actions=[jsb_spawner]),
        TimerAction(period=1.0, actions=[arm_spawner]),
        TimerAction(period=1.0, actions=[gripper_spawner]),
    ])
