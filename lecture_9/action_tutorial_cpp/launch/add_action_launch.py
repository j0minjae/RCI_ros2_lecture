from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    target_number_arg = DeclareLaunchArgument(
        'target_number',
        default_value='2',
        description='Target number for the action client'
    )

    return LaunchDescription([
        target_number_arg,
        Node(
            package='action_tutorial_cpp',
            executable='add_action_server',
            output='screen'),
        Node(
            package='action_tutorial_cpp',
            executable='add_action_client',
            output='screen',
            parameters=[{'target_number': LaunchConfiguration('target_number')}]),
    ])