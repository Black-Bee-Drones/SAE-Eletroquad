from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([LifecycleNode(
        package='lifecycle',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
    ), 
    LifecycleNode(
        package='lifecycle',
        executable='vision_node',
        name='vision_node',
        output='screen',
    ),
    LifecycleNode(
        package='lifecycle',
        executable='mission_control_node',
        name='mission_control_node',
        output='screen',
    ),
    LifecycleNode(
        package='lifecycle',
        executable='actuator_node',
        name='actuator_node',
        output='screen',
    )
])
