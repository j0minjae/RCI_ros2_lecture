from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='launch_tutorial',
            executable='parameter_node',
            name='random_publish_velocity',
            parameters=[{'cmd_vel_rate':1.0}],
            output='screen',
        ),
    ])
