#!/usr/bin/env python3
"""
Cognitive System with LLM Launch File - TMUX Multiplexing

Launches the complete cognitive framework with LLM integration using tmux:
- sense_node: Perception and sensory input
- memory_node: Storage and consolidation  
- action_node_with_llm: Enhanced decision making with LLM
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Generate launch description for the cognitive system with LLM using tmux."""
    
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
    
    # Create tmux session with multiple panes
    tmux_session = ExecuteProcess(
        cmd=[
            'tmux', 'new-session', '-d', '-s', 'cognitive_system',
            '-c', os.getcwd(),
            'bash', '-c', f'source {workspace_setup} && echo "Cognitive System TMUX Session" && sleep 2'
        ],
        output='screen'
    )
    
    # Split into panes and run nodes
    split_and_run_sense = ExecuteProcess(
        cmd=[
            'tmux', 'split-window', '-h', '-t', 'cognitive_system',
            'bash', '-c', f'source {workspace_setup} && ros2 run cognitive_framework sense_node --ros-args --log-level {log_level} --ros-args -r __ns:=/{namespace}'
        ],
        output='screen'
    )
    
    split_and_run_memory = ExecuteProcess(
        cmd=[
            'tmux', 'split-window', '-v', '-t', 'cognitive_system:0.1',
            'bash', '-c', f'source {workspace_setup} && ros2 run cognitive_framework memory_node --ros-args --log-level {log_level} --ros-args -r __ns:=/{namespace}'
        ],
        output='screen'
    )
    
    split_and_run_action = ExecuteProcess(
        cmd=[
            'tmux', 'split-window', '-v', '-t', 'cognitive_system:0.0',
            'bash', '-c', f'source {workspace_setup} && ros2 run cognitive_framework action_node_with_llm --ros-args --log-level {log_level} --ros-args -r __ns:=/{namespace}'
        ],
        output='screen'
    )
    
    # Attach to tmux session
    attach_tmux = ExecuteProcess(
        cmd=['tmux', 'attach-session', '-t', 'cognitive_system'],
        output='screen'
    )
    
    # Launch information
    launch_info = LogInfo(
        msg=[
            'Starting Cognitive Framework System with LLM using TMUX\n',
            '  - Sense Node: /cognitive/sense_data publisher\n',
            '  - Memory Node: sense subscriber → /cognitive/memory_data publisher\n', 
            '  - Action Node with LLM: sense + memory subscribers → /cognitive/action_data publisher\n',
            '  - LLM Features: Strategic reasoning, goal creation, enhanced decision making\n',
            '  - Namespace: ', namespace, '\n',
            '  - Log Level: ', log_level, '\n',
            '  - Each node will run in a separate tmux pane\n',
            '  - Use Ctrl+B then arrow keys to navigate between panes\n',
            '  - Use Ctrl+B then % to split vertically, " to split horizontally'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        log_level_arg,
        namespace_arg,
        
        # Information
        launch_info,
        
        # TMUX setup and node execution
        tmux_session,
        split_and_run_sense,
        split_and_run_memory,
        split_and_run_action,
        attach_tmux
    ]) 