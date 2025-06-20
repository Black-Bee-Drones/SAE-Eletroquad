#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    bouncing_pkg_dir = get_package_share_directory('bouncing')
    pid_controller_pkg_dir = get_package_share_directory('pid_controller')
    
    # Launch configuration variables
    figure_type = LaunchConfiguration('figure_type', default='cross')
    p1_lat = LaunchConfiguration('p1_lat', default='-23.1979885')
    p1_lon = LaunchConfiguration('p1_lon', default='-45.909908')
    p2_lat = LaunchConfiguration('p2_lat', default='-23.1978449')
    p2_lon = LaunchConfiguration('p2_lon', default='-45.9098869')
    p3_lat = LaunchConfiguration('p3_lat', default='-23.1980094')
    p3_lon = LaunchConfiguration('p3_lon', default='-45.9098283')
    p4_lat = LaunchConfiguration('p4_lat', default='-23.1978557')
    p4_lon = LaunchConfiguration('p4_lon', default='-45.9098052')
    
    # Declare arguments
    args = [
        DeclareLaunchArgument(
            'figure_type',
            default_value='cross',
            description='Type of figure to search for (circle, square, triangle, etc.)'
        ),
        DeclareLaunchArgument(
            'p1_lat',
            default_value='-23.1979885',
            description='Latitude of top-left corner'
        ),
        DeclareLaunchArgument(
            'p1_lon',
            default_value='-45.909908',
            description='Longitude of top-left corner'
        ),
        DeclareLaunchArgument(
            'p2_lat',
            default_value='-23.1978449',
            description='Latitude of top-right corner'
        ),
        DeclareLaunchArgument(
            'p2_lon',
            default_value='-45.9098869',
            description='Longitude of top-right corner'
        ),
        DeclareLaunchArgument(
            'p3_lat',
            default_value='-23.1980094',
            description='Latitude of bottom-left corner'
        ),
        DeclareLaunchArgument(
            'p3_lon',
            default_value='-45.9098283',
            description='Longitude of bottom-left corner'
        ),
        DeclareLaunchArgument(
            'p4_lat',
            default_value='-23.1978557',
            description='Latitude of bottom-right corner'
        ),
        DeclareLaunchArgument(
            'p4_lon',
            default_value='-45.9098052',
            description='Longitude of bottom-right corner'
        ),
    ]
    
    # Nodes to launch
    nodes = [
        # YOLO Inference Node
        Node(
            package='bouncing',
            executable='yolo_inference',
            name='yolo_inference_node',
            parameters=[{
                'model_path': os.path.join(bouncing_pkg_dir, 'bouncing', 'ai', 'yolo', 'models', 'best.onnx'),
                'conf_thres': 0.5,
                'iou_thres': 0.5,
                'image_source': 'webcam',
                'c920_config': 2
            }],
            output='screen'
        ),
        
        # PID Controller X (horizontal centering)
        Node(
            package='pid_controller',
            executable='pid_controller_standalone',
            name='pid_controller_x',
            parameters=[{
                'p_gain': 0.005,          # Adjust based on testing
                'i_gain': 0.0,            # No integral term needed for this application
                'd_gain': 0.0,            # No derivative term needed for this application
                'output_min': -1.0,
                'output_max': 1.0,
                'state_topic': '/pid_controller_x/state',
                'setpoint_topic': '/pid_controller_x/setpoint',
                'control_effort_topic': '/pid_controller_x/control_effort',
                'auto_start': True,
                'reverse_action': False
            }],
            output='screen'
        ),
        
        # PID Controller Y (vertical centering)
        Node(
            package='pid_controller',
            executable='pid_controller_standalone',
            name='pid_controller_y',
            parameters=[{
                'p_gain': 0.005,          # Adjust based on testing
                'i_gain': 0.0,            # No integral term needed for this application
                'd_gain': 0.0,            # No derivative term needed for this application
                'output_min': -1.0,
                'output_max': 1.0,
                'state_topic': '/pid_controller_y/state',
                'setpoint_topic': '/pid_controller_y/setpoint',
                'control_effort_topic': '/pid_controller_y/control_effort',
                'auto_start': True,
                'reverse_action': False
            }],
            output='screen'
        ),
        
        # Bouncing Node V2
        Node(
            package='bouncing',
            executable='bouncing_nodev2',
            name='bouncing_node_v2',
            parameters=[{
                'figure_type': figure_type,
                'p1_lat': p1_lat,
                'p1_lon': p1_lon,
                'p2_lat': p2_lat,
                'p2_lon': p2_lon,
                'p3_lat': p3_lat,
                'p3_lon': p3_lon,
                'p4_lat': p4_lat,
                'p4_lon': p4_lon
            }],
            output='screen'
        )
    ]
    
    return LaunchDescription(args + nodes)
