from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='9.0',
        description='X coordinate for the turtle to move to'
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='3.0',
        description='Y coordinate for the turtle to move to'
    )
    theta_arg = DeclareLaunchArgument(
        'theta',
        default_value='1.57',
        description='Theta orientation for the turtle to move to'
    )
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='3.0',
        description='Duration for the turtle to move'
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        theta_arg,
        duration_arg,
        Node(
            package='action_tutorial_cpp',
            executable='move_action_server',
            output='screen'),
        Node(
            package='action_tutorial_cpp',
            executable='move_action_client',
            output='screen',
            parameters=[{
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'theta': LaunchConfiguration('theta'),
                'duration': LaunchConfiguration('duration'),
            }]),
    ])