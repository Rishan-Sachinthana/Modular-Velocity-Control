from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution ,FindExecutable
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare("modular_velocity_control").find("modular_velocity_control")
    robot_description_path = os.path.join(pkg_share, "urdf", "modular_mecanum_bot.urdf.xacro")
    controller_config_path = os.path.join(pkg_share, "config", "mecanum_controllers.yaml")

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        robot_description_path
    ])

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                ])
            )
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]
        ),

        # ros2_control controller manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controller_config_path],
            output="screen"
        ),

        # Spawn robot into Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "modular_bot", "-topic", "robot_description"],
            output="screen"
        ),

        # Spawn mecanum drive controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["mecanum_controller"],
            output="screen"
        ),

        # Velocity dispatcher
        Node(
            package="modular_velocity_control",
            executable="velocity_dispatcher",
            name="velocity_dispatcher",
            output="screen"
        ),
    ])
