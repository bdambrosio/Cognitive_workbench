#!/usr/bin/env python3
"""
Memory Node Launch File

Launches the memory node for information storage and consolidation.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for the memory node."""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for the memory node'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='cognitive_system',
        description='Namespace for the memory node'
    )
    
    # Get launch configurations
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
    
    # Define the memory node
    memory_node = Node(
        package='cognitive_framework',
        executable='memory_node',
        name='memory_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {'log_level': log_level}
        ],
        ros_arguments=['--log-level', log_level]
    )
    
    # Launch information
    launch_info = LogInfo(
        msg=[
            'Starting Memory Node\n',
            '  - Subscriber: /cognitive/sense_data\n',
            '  - Publisher: /cognitive/memory_data\n',
            '  - Service: /cognitive/memory_request\n',
            '  - Memory types: short_term, long_term, working\n',
            '  - Namespace: ', namespace, '\n',
            '  - Log Level: ', log_level
        ]
    )
    
    return LaunchDescription([
        # Arguments
        log_level_arg,
        namespace_arg,
        
        # Information
        launch_info,
        
        # Node
        memory_node
    ]) 