"""OpenArm v1.0 MoveIt demo 를 띄우는 실습용 launch."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('openarm_bimanual_moveit_config'),
                'launch',
                'demo.launch.py',
            ])
        ),
        launch_arguments={'arm_type': 'v1.0'}.items(),
    )
    return LaunchDescription([demo])
