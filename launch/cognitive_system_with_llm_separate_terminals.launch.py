#!/usr/bin/env python3
"""
Cognitive System with LLM Launch File - Separate Terminals

Launches the complete cognitive framework with LLM integration in separate terminals:
- sense_node: Perception and sensory input
- memory_node: Storage and consolidation  
- action_node_with_llm: Enhanced decision making with LLM
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Generate launch description for the cognitive system with LLM in separate terminals."""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='cognitive_system',
        description='Namespace for all cognitive nodes'
    )
    
    # Get launch configurations
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
    
    # Source the workspace setup
    workspace_setup = os.path.join(os.getcwd(), 'install', 'setup.bash')
    
    # Define nodes in separate terminals
    sense_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            f'source {workspace_setup} && ros2 run cognitive_framework sense_node --ros-args --log-level {log_level} --ros-args -r __ns:=/{namespace}'
        ],
        output='screen'
    )
    
    memory_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            f'source {workspace_setup} && ros2 run cognitive_framework memory_node --ros-args --log-level {log_level} --ros-args -r __ns:=/{namespace}'
        ],
        output='screen'
    )
    
    action_node_with_llm = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            f'source {workspace_setup} && ros2 run cognitive_framework action_node_with_llm --ros-args --log-level {log_level} --ros-args -r __ns:=/{namespace}'
        ],
        output='screen'
    )
    
    # Launch information
    launch_info = LogInfo(
        msg=[
            'Starting Cognitive Framework System with LLM in Separate Terminals\n',
            '  - Sense Node: /cognitive/sense_data publisher\n',
            '  - Memory Node: sense subscriber → /cognitive/memory_data publisher\n', 
            '  - Action Node with LLM: sense + memory subscribers → /cognitive/action_data publisher\n',
            '  - LLM Features: Strategic reasoning, goal creation, enhanced decision making\n',
            '  - Namespace: ', namespace, '\n',
            '  - Log Level: ', log_level, '\n',
            '  - Each node will open in a separate terminal window'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        log_level_arg,
        namespace_arg,
        
        # Information
        launch_info,
        
        # Nodes in separate terminals (order matters for dependencies)
        sense_node,
        memory_node, 
        action_node_with_llm
    ]) 