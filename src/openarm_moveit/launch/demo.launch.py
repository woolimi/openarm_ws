"""OpenArm v1.0 MoveIt demo 를 mock hardware 위에 띄운다.

업스트림 openarm_bimanual_moveit_config 의 v1.0 구성을 그대로 쓰되, 이 패키지의 파일 다섯으로 바꿔 조립한다.
  - config/openarm_bimanual.srdf — 충돌 제외 쌍에 Never(절대 닿지 않는 쌍)를 더한 SRDF
  - config/moveit_joint_limits.yaml — acceleration 한계를 채운 관절 한계
  - config/controllers.yaml — 그리퍼를 GripperActionController 로 두어 MoveIt 의 GripperCommand 와 맞춘다
  - config/ompl_planning.yaml — 그룹별 OMPL planner 목록(RViz Planning Library 드롭다운)
  - config/demo.rviz — MotionPlanning 패널에 예제 마커 display 를 더한 RViz 설정

    ros2 launch openarm_moveit demo.launch.py [rviz_config:=<.rviz 경로>]
"""

import os

import xacro

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

from openarm_leader import stale_processes

CONFIG_DIR = 'openarm_v1.0'
XACRO_MAPPINGS = {
    'arm_type': 'v1.0',
    'bimanual': 'true',
    'use_fake_hardware': 'true',
    'ros2_control': 'true',
}


def cleanup_previous_session(_context):
    stale = stale_processes.kill_stale()
    if stale:
        print(f'이전 실행이 남긴 세션 프로세스를 정리했습니다: {stale}')
    return []


def moveit_nodes(context):
    description_path = get_package_share_directory('openarm_description')
    moveit_path = get_package_share_directory('openarm_bimanual_moveit_config')
    demo_path = get_package_share_directory('openarm_moveit')

    xacro_path = os.path.join(
        description_path, 'assets', 'robot', CONFIG_DIR,
        'urdf', 'openarm_v10.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_path, mappings=XACRO_MAPPINGS).toprettyxml(indent='  ')
    controllers_file = os.path.join(demo_path, 'config', 'controllers.yaml')

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
            file_path=os.path.join(demo_path, 'config', 'openarm_bimanual.srdf'))
        .robot_description_kinematics(
            file_path=f'config/{CONFIG_DIR}/kinematics.yaml')
        .joint_limits(
            file_path=os.path.join(
                demo_path, 'config', 'moveit_joint_limits.yaml'))
        .trajectory_execution(
            file_path=f'config/{CONFIG_DIR}/moveit_controllers.yaml')
        .planning_pipelines(
            pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()

    # planner 정의 24종은 moveit_configs_utils 기본값으로 들어온다.
    # 그룹별로 어떤 planner 를 쓸지는 이 패키지 파일이 정한다.
    ompl_path = os.path.join(demo_path, 'config', 'ompl_planning.yaml')
    with open(ompl_path, 'r', encoding='utf-8') as handle:
        moveit_params['ompl'].update(yaml.safe_load(handle))

    pilz_path = os.path.join(
        moveit_path, 'config', CONFIG_DIR, 'pilz_cartesian_limits.yaml')
    if os.path.exists(pilz_path):
        with open(pilz_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        if 'cartesian_limits' in data:
            moveit_params.setdefault(
                'robot_description_planning', {}).update(data)

    rviz_config = LaunchConfiguration('rviz_config').perform(context)
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
    default_rviz = os.path.join(
        get_package_share_directory('openarm_moveit'), 'config', 'demo.rviz')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='RViz 설정 파일')
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
        rviz_config_arg,
        OpaqueFunction(function=cleanup_previous_session),
        OpaqueFunction(function=moveit_nodes),
        TimerAction(period=2.0, actions=[jsb_spawner]),
        TimerAction(period=1.0, actions=[arm_spawner]),
        TimerAction(period=1.0, actions=[gripper_spawner]),
    ])
