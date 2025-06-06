from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bouncing',
            executable='bouncing_node',
            name='bouncing_node',
            output='screen'
        ),
    ])

