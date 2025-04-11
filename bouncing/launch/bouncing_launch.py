from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bouncing',
            executable='bouncing_detector',
            name='bouncing_detector',
            output='screen'
        ),
        Node(
            package='bouncing',
            executable='bouncing_manager',
            name='bouncing_manager',
            output='screen'
        ),
        Node(
            package='bouncing',
            executable='bouncing_movement',
            name='bouncing_movement',
            output='screen'
        ),
    ])

