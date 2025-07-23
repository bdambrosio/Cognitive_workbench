#!/usr/bin/env python3
"""
Zenoh Cognitive Framework Launcher

This script launches all the cognitive framework nodes as separate processes.
Each node runs on its own core for optimal performance.
"""

import subprocess
import sys
import os
import time
import signal
import threading
from typing import List, Dict


class ZenohCognitiveLauncher:
    """Launcher for the Zenoh cognitive framework nodes."""
    
    def __init__(self):
        self.processes: Dict[str, subprocess.Popen] = {}
        self.node_configs = {
            'memory_node': {
                'script': 'memory_node.py',
                'description': 'Memory storage and retrieval',
                'core': 1,
                'console_access': False  # Background process with log file
            },
            'llm_service_node': {
                'script': 'llm_service_node.py',
                'description': 'LLM API service',
                'core': 2,
                'console_access': False  # Background process with log file
            },
            'sense_node': {
                'script': 'sense_node.py',
                'description': 'Sensory input processing',
                'core': 3,
                'console_access': True   # Console access for input
            },
            'single_llm_action_example': {
                'script': 'single_llm_action_example.py',
                'description': 'Single LLM action example',
                'core': 4,
                'console_access': False  # Background process with log file
            },
            'action_display_node': {
                'script': 'action_display_node.py',
                'description': 'Action display and text input',
                'core': 5,
                'console_access': False  # Background process with log file
            }
        }
        
        # Get the directory of this script
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Create logs directory
        self.logs_dir = os.path.join(self.script_dir, 'logs')
        os.makedirs(self.logs_dir, exist_ok=True)
        
        print('🚀 Zenoh Cognitive Framework Launcher')
        print('=' * 50)
        print(f'📁 Logs will be written to: {self.logs_dir}')
        print()
    
    def launch_node(self, node_name: str, config: Dict) -> bool:
        """Launch a single node."""
        try:
            script_path = os.path.join(self.script_dir, config['script'])
            
            if not os.path.exists(script_path):
                print(f'❌ Script not found: {script_path}')
                return False
            
            # Set CPU affinity if possible
            env = os.environ.copy()
            env['PYTHONPATH'] = self.script_dir + ':' + env.get('PYTHONPATH', '')
            
            # Configure process based on console access needs
            if config.get('console_access', False):
                # For sense_node: direct console access (no pipes)
                process = subprocess.Popen(
                    [sys.executable, script_path],
                    cwd=self.script_dir,
                    env=env
                )
                print(f'✅ Launched {node_name} (PID: {process.pid}) - {config["description"]} [CONSOLE ACCESS]')
            else:
                # For other nodes: background processes with log files
                # Note: Nodes now write their own logs, so we don't need to capture output
            process = subprocess.Popen(
                [sys.executable, script_path],
                cwd=self.script_dir,
                    env=env
                )
                
                print(f'✅ Launched {node_name} (PID: {process.pid}) - {config["description"]} [LOG: {node_name}.log]')
            
            self.processes[node_name] = process
            return True
            
        except Exception as e:
            print(f'❌ Failed to launch {node_name}: {e}')
            return False
    
    def launch_all_nodes(self) -> bool:
        """Launch all nodes in the correct order."""
        print('Starting nodes in order...')
        print()
        
        # Launch order: memory -> llm_service -> others
        launch_order = ['memory_node', 'llm_service_node', 'sense_node', 'single_llm_action_example', 'action_display_node']
        
        for node_name in launch_order:
            if node_name in self.node_configs:
                config = self.node_configs[node_name]
                
                print(f'🚀 Starting {node_name}...')
                success = self.launch_node(node_name, config)
                
                if not success:
                    print(f'❌ Failed to start {node_name}, stopping launch')
                    self.shutdown()
                    return False
                
                # Wait a moment for the node to initialize
                time.sleep(2)
                print()
        
        print('✅ All nodes launched successfully!')
        print()
        print('📊 Node Status:')
        for node_name, process in self.processes.items():
            status = '🟢 Running' if process.poll() is None else '🔴 Stopped'
            if self.node_configs[node_name].get('console_access', False):
                console_info = ' [CONSOLE]'
            else:
                console_info = f' [LOG: {node_name}.log]'
            print(f'   {node_name}: {status} (PID: {process.pid}){console_info}')
        
        return True
    
    def monitor_processes(self):
        """Monitor all processes and restart if needed."""
        while True:
            time.sleep(5)
            
            # Check if any processes have died
            for node_name, process in list(self.processes.items()):
                if process.poll() is not None:
                    print(f'⚠️  {node_name} has stopped (exit code: {process.returncode})')
                    
                    # Optionally restart the process (but not sense_node with console access)
                    if node_name in self.node_configs and not self.node_configs[node_name].get('console_access', False):
                        print(f'🔄 Restarting {node_name}...')
                        config = self.node_configs[node_name]
                        if self.launch_node(node_name, config):
                            print(f'✅ {node_name} restarted successfully')
                        else:
                            print(f'❌ Failed to restart {node_name}')
    
    def shutdown(self):
        """Shutdown all processes gracefully."""
        print('\n🛑 Shutting down all nodes...')
        
        # Send SIGTERM to all processes
        for node_name, process in self.processes.items():
            try:
                print(f'   Stopping {node_name}...')
                process.terminate()
            except Exception as e:
                print(f'   Error stopping {node_name}: {e}')
        
        # Wait for processes to terminate
        time.sleep(3)
        
        # Force kill any remaining processes
        for node_name, process in self.processes.items():
            try:
                if process.poll() is None:
                    print(f'   Force killing {node_name}...')
                    process.kill()
            except Exception as e:
                print(f'   Error force killing {node_name}: {e}')
        
        print('✅ All nodes stopped')
    
    def run(self):
        """Main launcher loop."""
        try:
            # Launch all nodes
            if not self.launch_all_nodes():
                return
            
            print('🎯 System is running! Press Ctrl+C to stop all nodes')
            print('📝 You can now type text input directly in this terminal for the sense_node')
            print()
            print('📋 Monitoring commands:')
            print(f'   tail -f {self.logs_dir}/memory_node.log')
            print(f'   tail -f {self.logs_dir}/llm_service_node.log')
            print(f'   tail -f {self.logs_dir}/single_llm_action_example.log')
            print(f'   tail -f {self.logs_dir}/action_display_node.log')
            print(f'   tail -f {self.logs_dir}/*.log  # Monitor all nodes')
            print('=' * 50)
            
            # Start monitoring thread
            monitor_thread = threading.Thread(target=self.monitor_processes, daemon=True)
            monitor_thread.start()
            
            # Wait for user interrupt
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print('\n🛑 Received interrupt signal')
        finally:
            self.shutdown()


def main():
    """Main entry point."""
    launcher = ZenohCognitiveLauncher()
    launcher.run()


if __name__ == '__main__':
    main() 