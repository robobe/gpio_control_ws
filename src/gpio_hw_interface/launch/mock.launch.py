from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
import os
from ament_index_python.packages import get_package_share_directory

PKG="gpio_hw_interface"
ROBOT="mock.urdf"
CONFIG = "mock.yaml"

def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = os.path.join(get_package_share_directory(PKG))
    xacro_file = os.path.join(pkg_path, "urdf", ROBOT)
    robot_description_config = xacro.process_file(xacro_file).toxml()
    params = {"robot_description": robot_description_config, "use_sim_time": False}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(PKG),
            "config",
            CONFIG,
        ]
    )



    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "--param-file", robot_controllers],
    )

    ld.add_action(node_robot_state_publisher)
    ld.add_action(control_node)
    ld.add_action(gpio_controller_spawner)
    return ld

