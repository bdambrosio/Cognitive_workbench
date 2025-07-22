#!/usr/bin/env python3

"""
Launch file for Single LLM Action System

This launch file starts a complete cognitive system with:
- sense_node: Handles text input
- memory_node: Stores and retrieves memories
- single_llm_action_example: Makes exactly ONE LLM call per input

This provides a clean, simple demonstration of LLM integration.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for single LLM action system."""
    
    # Launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )
    
    # Sense Node
    sense_node = Node(
        package='cognitive_framework',
        executable='sense_node',
        name='sense_node',
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
    
    # Memory Node
    memory_node = Node(
        package='cognitive_framework',
        executable='memory_node',
        name='memory_node',
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
        sense_node,
        memory_node,
        single_llm_action_node,
    ]) 