from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modular_velocity_control',
            executable='velocity_dispatcher',
            name='velocity_dispatcher',
            output='screen'
        ),
        Node(
            package='modular_velocity_control',
            executable='visualizer',
            name='robot_visualizer',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/rishan/ros2_ws/src/modular_velocity_control/config/rviz_config.rviz'],
            output='screen'
        )
    ])
