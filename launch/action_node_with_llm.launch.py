#!/usr/bin/env python3
"""
Action Node with LLM Launch File

Launches the enhanced action node with LLM integration for decision making and execution.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for the action node with LLM."""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for the action node'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='cognitive_system',
        description='Namespace for the action node'
    )
    
    # Get launch configurations
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
    
    # Define the action node with LLM
    action_node_with_llm = Node(
        package='cognitive_framework',
        executable='action_node_with_llm',
        name='action_node_with_llm',
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
            'Starting Action Node with LLM\n',
            '  - Subscribers: /cognitive/sense_data, /cognitive/memory_data\n',
            '  - Publisher: /cognitive/action_data\n',
            '  - LLM Integration: Enhanced decision making\n',
            '  - Features: Strategic reasoning, goal creation\n',
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
        action_node_with_llm
    ]) 