#!/usr/bin/env python3

"""
Launch file for Single LLM Action Example

This launch file starts the single_llm_action_example node,
which demonstrates a clean, simple LLM integration with exactly
ONE LLM call per text input.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for single LLM action example."""
    
    # Launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the node'
    )
    
    # Single LLM Action Example Node
    single_llm_action_node = Node(
        package='cognitive_framework',
        executable='single_llm_action_example',
        name='single_llm_action_example',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level'),
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        log_level_arg,
        single_llm_action_node,
    ]) 