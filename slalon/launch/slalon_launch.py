from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slalon',
            executable='depth_st',
            name='depth_measurement',
            output='screen'
        ),
        Node(
            package='slalon',
            executable='movement',
            name='movement_st',
            output='screen'
        )
    ])