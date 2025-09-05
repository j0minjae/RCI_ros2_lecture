from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            remappings=[('/turtle1/cmd_vel','/turtle1/cmd_vel_reverse')],
        ),
        Node(
            package='topic_tutorial_cpp',
            executable='pub_reverse_vel_node',
            name='reverse_twist'
        )
    ])
