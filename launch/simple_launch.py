#!/usr/bin/env python3
"""
Simple Launch File (Development Version)

A simplified launch file that doesn't require launch_ros package.
Useful for development environments where the full ROS2 setup isn't available.
"""

import sys
import subprocess
import argparse
import signal
import time
import os
from pathlib import Path


def launch_node(node_name, log_level='info', namespace='cognitive_system'):
    """Launch a single node using subprocess."""
    
    # Get the path to the node script
    script_path = Path(__file__).parent.parent / 'cognitive_framework' / f'{node_name}.py'
    
    if not script_path.exists():
        print(f"❌ Error: Node script not found: {script_path}")
        return None
    
    # Set up environment variables
    env = dict(os.environ)
    env['ROS_LOG_LEVEL'] = log_level
    
    # Build the command
    cmd = [
        sys.executable, str(script_path),
        '--ros-args',
        '--log-level', log_level,
        '--ros-args', '-r', f'__ns:={namespace}'
    ]
    
    print(f"🚀 Starting {node_name}...")
    print(f"   Log Level: {log_level}")
    print(f"   Namespace: {namespace}")
    
    try:
        # Run the node
        process = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        
        return process
        
    except Exception as e:
        print(f"❌ Error launching {node_name}: {e}")
        return None


def main():
    """Main entry point for simple launch."""
    parser = argparse.ArgumentParser(description='Simple cognitive framework launcher')
    parser.add_argument('--nodes', nargs='+', 
                       choices=['sense_node', 'memory_node', 'action_node_with_llm'],
                       default=['sense_node', 'memory_node', 'action_node_with_llm'],
                       help='Nodes to launch (default: all)')
    parser.add_argument('--log-level', default='info',
                       choices=['debug', 'info', 'warn', 'error', 'fatal'],
                       help='Log level (default: info)')
    parser.add_argument('--namespace', default='cognitive_system',
                       help='ROS2 namespace (default: cognitive_system)')
    
    args = parser.parse_args()
    
    # Check if ROS2 environment is sourced
    if 'ROS_DISTRO' not in os.environ:
        print("⚠️  Warning: ROS2 environment not detected.")
        print("   Make sure to source your ROS2 setup file:")
        print("   source /opt/ros/humble/setup.bash  # or your ROS2 version")
        print()
    
    print("🧠 Simple Cognitive Framework Launcher")
    print("=" * 50)
    print(f"Nodes: {', '.join(args.nodes)}")
    print(f"Log Level: {args.log_level}")
    print(f"Namespace: {args.namespace}")
    print("Press Ctrl+C to stop all nodes")
    print("-" * 50)
    
    # Launch all requested nodes
    processes = []
    for node in args.nodes:
        process = launch_node(node, args.log_level, args.namespace)
        if process:
            processes.append((node, process))
        else:
            print(f"❌ Failed to launch {node}")
            # Stop any already launched processes
            for _, p in processes:
                p.terminate()
            sys.exit(1)
    
    # Wait for all processes
    try:
        while processes:
            for node, process in processes[:]:
                if process.poll() is not None:
                    print(f"🛑 {node} stopped (exit code: {process.returncode})")
                    processes.remove((node, process))
            
            if processes:
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print(f"\n🛑 Stopping all nodes...")
        for node, process in processes:
            print(f"   Stopping {node}...")
            process.terminate()
        
        # Wait for processes to terminate
        for node, process in processes:
            try:
                process.wait(timeout=5)
                print(f"   ✅ {node} stopped")
            except subprocess.TimeoutExpired:
                process.kill()
                print(f"   ⚠️  {node} force killed")
    
    print("✅ All nodes stopped")


if __name__ == '__main__':
    main() 