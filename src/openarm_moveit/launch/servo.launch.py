"""MoveIt demo + Servo 노드.

demo.launch.py 위에 moveit_servo 의 servo_node 를 얹는다. Servo 는 move_group 과 같은
URDF·SRDF·kinematics 를 받아 Jacobian 과 충돌 검사를 같은 모델로 하고, 관절 한계만
config/servo_joint_limits.yaml 의 느린 속도 상한을 쓴다.

    ros2 launch openarm_moveit servo.launch.py arm:=left
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

CONFIG_DIR = 'openarm_v1.0'
XACRO_MAPPINGS = {
    'arm_type': 'v1.0',
    'bimanual': 'true',
    'use_fake_hardware': 'true',
    'ros2_control': 'true',
}


def servo_node(context):
    arm = LaunchConfiguration('arm').perform(context)
    description_path = get_package_share_directory('openarm_description')
    pkg_path = get_package_share_directory('openarm_moveit')
    xacro_path = os.path.join(
        description_path, 'assets', 'robot', CONFIG_DIR, 'urdf', 'openarm_v10.urdf.xacro')

    # move_group 과 같은 URDF·SRDF·IK 솔버. 관절 한계만 Servo 전용(느린 속도 상한)으로 바꾼다.
    moveit_config = (
        MoveItConfigsBuilder('openarm', package_name='openarm_bimanual_moveit_config')
        .robot_description(file_path=xacro_path, mappings=XACRO_MAPPINGS)
        .robot_description_semantic(file_path=os.path.join(pkg_path, 'config', 'openarm_bimanual.srdf'))
        .robot_description_kinematics(file_path=f'config/{CONFIG_DIR}/kinematics.yaml')
        .joint_limits(file_path=os.path.join(pkg_path, 'config', 'servo_joint_limits.yaml'))
        .trajectory_execution(file_path=f'config/{CONFIG_DIR}/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    # Servo 파라미터 — yaml 을 읽고 팔에 맞는 group·출력 토픽으로 덮어쓴다
    servo_params = (
        ParameterBuilder('openarm_moveit').yaml('config/servo.yaml').to_dict())
    servo_params['move_group_name'] = f'{arm}_arm'
    servo_params['command_out_topic'] = f'/{arm}_joint_trajectory_controller/joint_trajectory'

    node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            {'moveit_servo': servo_params},
            {'update_period': servo_params['publish_period']},    # 가속도 제한 필터 주기
            {'planning_group_name': f'{arm}_arm'},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    # move_group 과 컨트롤러가 뜬 뒤에 시작한다
    return [TimerAction(period=5.0, actions=[node])]


def generate_launch_description():
    demo_launch = os.path.join(
        get_package_share_directory('openarm_moveit'), 'launch', 'demo.launch.py')
    demo_rviz = os.path.join(
        get_package_share_directory('openarm_moveit'), 'config', 'demo.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='left', choices=['left', 'right'],
                              description='Servo 로 움직일 팔'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true',
                              choices=['true', 'false'],
                              description='true 는 mock_components, false 는 CAN-FD 실기.'),
        DeclareLaunchArgument('rviz_config', default_value=demo_rviz,
                              description='RViz 설정 파일'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch),
            launch_arguments={
                'rviz_config': LaunchConfiguration('rviz_config'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            }.items(),
        ),
        OpaqueFunction(function=servo_node),
    ])
