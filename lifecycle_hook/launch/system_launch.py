from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([LifecycleNode(
        package='lifecycle_hook',
        executable='navigation_node',
        name='navigation_node',
        namespace='',
        output='screen',
    ), 
    LifecycleNode(
        package='lifecycle_hook',
        executable='vision_node',
        name='vision_node',
        namespace='',
        output='screen',
    ),
    LifecycleNode(
        package='lifecycle_hook',
        executable='mission_control_node',
        name='mission_control_node',
        namespace='',
        output='screen',
    ),
    LifecycleNode(
        package='lifecycle_hook',
        executable='actuator_node',
        name='actuator_node',
        namespace='',
        output='screen',
    )
])
