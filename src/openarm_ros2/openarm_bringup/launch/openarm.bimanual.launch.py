# Copyright 2025 Enactic, Inc.
# Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# All accepted arm_type values
VALID_ARM_TYPES = {
    "v1.0", "v10", "v1_0", "openarm_v1.0", "openarm_v10", "openarm_v1_0",
    "v2.0", "v20", "v2_0", "openarm_v2.0", "openarm_v20", "openarm_v2_0",
}


def resolve_arm_config(arm_type_str: str) -> tuple[str, str]:
    """
    Resolve folder name and xacro file name from arm_type.
    Accepts: v1.0, v10, v1_0, openarm_v1.0, openarm_v10, openarm_v1_0 (and v2.0 variants)
    Raises ValueError if arm_type is not recognized.
    """
    if arm_type_str not in VALID_ARM_TYPES:
        raise ValueError(
            f"Invalid arm_type: '{arm_type_str}'. "
            f"Please specify openarm_v1.0 or openarm_v2.0."
        )
    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        return "openarm_v1.0", "openarm_v10.urdf.xacro"
    return "openarm_v2.0", "openarm_v20.urdf.xacro"


#: Hardware params the description cannot know: they are measured per robot.
#: Whole-robot values sit at the top level of the yaml, per-arm values under
#: arms.<arm>. The names are the ones OpenArmHW reads.
ROBOT_HARDWARE_PARAMS = ("gravity_comp", "root_link", "saturation_cap")
ARM_HARDWARE_PARAMS = ("tip_link", "payload_mass", "payload_com", "tau_bias")


def format_hardware_param(value):
    """Render a yaml value the way OpenArmHW parses it: a list becomes the
    space-separated numbers its istringstream reads, everything else its
    lower-case text (the plugin compares booleans as "true"/"1"/"on")."""
    if isinstance(value, (list, tuple)):
        return " ".join(f"{float(v):.6g}" for v in value)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def inject_hardware_params(document, config_path):
    """Add the measured params to every OpenArmHW <hardware> block in place.

    The ros2_control xacro lives in openarm_description, which this workspace
    pulls from upstream and does not edit, so the values are added to the
    generated document instead. Blocks are matched by their plugin, so a mock
    hardware run is left untouched, and the arm is picked by the block's own
    arm_prefix, so the left arm can never be handed the right arm's payload.
    """
    with open(config_path, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    arms = config.get("arms") or {}

    for hardware in document.getElementsByTagName("hardware"):
        plugins = hardware.getElementsByTagName("plugin")
        if not plugins or not plugins[0].firstChild:
            continue
        if plugins[0].firstChild.data.strip() != "openarm_hardware/OpenArmHW":
            continue

        params = {}
        for node in hardware.getElementsByTagName("param"):
            if node.firstChild:
                params[node.getAttribute("name")] = node.firstChild.data.strip()
        arm = params.get("arm_prefix", "").rstrip("_")
        values = {key: config[key]
                  for key in ROBOT_HARDWARE_PARAMS if key in config}
        values.update({key: (arms.get(arm) or {})[key]
                       for key in ARM_HARDWARE_PARAMS
                       if key in (arms.get(arm) or {})})

        for name, value in values.items():
            element = document.createElement("param")
            element.setAttribute("name", name)
            element.appendChild(
                document.createTextNode(format_hardware_param(value)))
            hardware.appendChild(element)


def namespace_from_context(context, arm_prefix):
    arm_prefix_str = context.perform_substitution(arm_prefix)
    if arm_prefix_str:
        return arm_prefix_str.strip('/')
    return None


def generate_robot_description(context: LaunchContext, description_package, description_file,
                               arm_type, use_fake_hardware, right_can_interface, left_can_interface,
                               hardware_config_file):
    """Generate robot description using xacro processing."""
    description_package_str = context.perform_substitution(description_package)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)
    hardware_config_file_str = context.perform_substitution(hardware_config_file)

    folder_name, file_name = resolve_arm_config(arm_type_str)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "assets", "robot", folder_name, "urdf", file_name
    )

    document = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "right_can_interface": right_can_interface_str,
            "left_can_interface": left_can_interface_str,
        }
    )

    if hardware_config_file_str:
        inject_hardware_params(document, hardware_config_file_str)

    return document.toprettyxml(indent="  ")


def robot_nodes_spawner(context: LaunchContext, description_package, description_file,
                        arm_type, use_fake_hardware, controllers_file,
                        right_can_interface, left_can_interface, arm_prefix,
                        hardware_config_file):
    """Spawn both robot state publisher and control nodes with shared robot description."""
    namespace = namespace_from_context(context, arm_prefix)

    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type,
        use_fake_hardware, right_can_interface, left_can_interface,
        hardware_config_file,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    if namespace:
        controllers_file_str = controllers_file_str.replace(
            "openarm_bimanual_controllers.yaml",
            "openarm_bimanual_controllers_namespaced.yaml"
        )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    """Spawn controller based on robot_controller argument."""
    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        robot_controller_left = "left_forward_position_controller"
        robot_controller_right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        robot_controller_left = "left_joint_trajectory_controller"
        robot_controller_right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=[robot_controller_left, robot_controller_right,
                       "-c", controller_manager_ref],
        )
    ]


def generate_launch_description():
    """Generate launch description for OpenArm bimanual configuration."""

    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v20.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="openarm_v2.0",
            description="Arm type. Accepts: v1.0, v10, openarm_v1.0, v2.0, v20, openarm_v2.0, etc.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for the arm for topic namespacing.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="CAN interface to use for the right arm.",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can1",
            description="CAN interface to use for the left arm.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_bimanual_controllers.yaml",
            description="Controllers file to use.",
        ),
        DeclareLaunchArgument(
            "hardware_config_file",
            default_value="",
            description="YAML with the measured hardware params (gravity "
                        "compensation, payload, torque bias). Empty leaves the "
                        "description untouched.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")
    hardware_config_file = LaunchConfiguration("hardware_config_file")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "controllers", controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file, arm_type,
              use_fake_hardware, controllers_file,
              right_can_interface, left_can_interface, arm_prefix,
              hardware_config_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "bimanual.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                f"/{namespace_from_context(context, arm_prefix)}/controller_manager"
                if namespace_from_context(context, arm_prefix)
                else "/controller_manager"
            ],
        )]
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner,
        args=[robot_controller, arm_prefix]
    )

    gripper_controller_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=[
                "left_gripper_controller", "right_gripper_controller",
                "-c",
                f"/{namespace_from_context(context, arm_prefix)}/controller_manager"
                if namespace_from_context(context, arm_prefix)
                else "/controller_manager"
            ],
        )]
    )

    LAUNCH_DELAY_SECONDS = 1.0

    return LaunchDescription(
        declared_arguments + [
            robot_nodes_spawner_func,
            rviz_node,
            TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[
                        joint_state_broadcaster_spawner]),
            TimerAction(period=LAUNCH_DELAY_SECONDS,
                        actions=[controller_spawner_func]),
            TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[
                        gripper_controller_spawner]),
        ]
    )
