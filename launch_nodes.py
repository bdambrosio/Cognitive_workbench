#!/usr/bin/env python3
"""
Simple Node Launcher

A Python script to launch individual cognitive framework nodes without requiring
the full ROS2 launch system. Useful for development and testing.
"""

import sys
import subprocess
import argparse
import signal
import time
import os
from pathlib import Path


def run_node(node_name, log_level='info', namespace='cognitive_system'):
    """Run a single node using subprocess."""
    
    # Get the path to the node script
    script_path = Path(__file__).parent / 'cognitive_framework' / f'{node_name}.py'
    
    if not script_path.exists():
        print(f"❌ Error: Node script not found: {script_path}")
        return False
    
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
    print(f"   Command: {' '.join(cmd)}")
    print(f"   Log Level: {log_level}")
    print(f"   Namespace: {namespace}")
    print("   Press Ctrl+C to stop")
    print("-" * 50)
    
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
        
        # Stream output
        if process.stdout:
            for line in process.stdout:
                print(line.rstrip())
        
        process.wait()
        return process.returncode == 0
        
    except KeyboardInterrupt:
        print(f"\n🛑 Stopping {node_name}...")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
        return True
    except Exception as e:
        print(f"❌ Error running {node_name}: {e}")
        return False


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Launch cognitive framework nodes')
    parser.add_argument('node', choices=['sense_node', 'memory_node', 'action_node_with_llm', 'single_llm_action_example'],
                       help='Node to launch')
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
    
    # Launch the node
    success = run_node(args.node, args.log_level, args.namespace)
    
    if success:
        print(f"✅ {args.node} completed successfully")
    else:
        print(f"❌ {args.node} failed")
        sys.exit(1)


if __name__ == '__main__':
    import os
    main() 