#!/usr/bin/env python3
"""
Sense Node Launch File

Launches the sense node for perception and sensory input processing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for the sense node."""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for the sense node'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='cognitive_system',
        description='Namespace for the sense node'
    )
    
    # Get launch configurations
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
    
    # Define the sense node
    sense_node = Node(
        package='cognitive_framework',
        executable='sense_node',
        name='sense_node',
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
            'Starting Sense Node\n',
            '  - Publisher: /cognitive/sense_data\n',
            '  - Subscriber: /cognitive/external_text_input (optional)\n',
            '  - Console input: Available in terminal\n',
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
        sense_node
    ]) 