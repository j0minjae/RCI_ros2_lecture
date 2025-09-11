from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_teleop',
            default_value='1',
            description = 'Flag to indicate whether to use teleop or not',
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen',
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_key',
            output='screen',
            prefix='xterm -e',
            condition=IfCondition(LaunchConfiguration('use_teleop')),
        ),
        Node(
            package='topic_tutorial_cpp',
            executable='pub_vel_node',
            name='vel_publisher',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_teleop')),

        ),
        Node(
            package='topic_tutorial_cpp',
            executable='sub_pose_node',
            name='pose_subscriber',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_teleop')),
        ),
    ])
