#!/usr/bin/env python3
"""
Zenoh Character Launcher

This script launches multiple character instances with their respective nodes.
Each character gets its own sense_node, memory_node, and single_llm_action_example.
The LLM service node is shared across all characters.
"""

import subprocess
import time
import signal
import sys
import json
import argparse
import logging
import yaml
from typing import Dict, List, Any
from dataclasses import dataclass


@dataclass
class CharacterInstance:
    """Represents a character instance with its processes."""
    name: str
    config: Dict[str, Any]
    processes: List[subprocess.Popen] = None
    
    def __post_init__(self):
        if self.processes is None:
            self.processes = []


class CharacterLauncher:
    """Manages launching and monitoring character instances."""
    
    def __init__(self):
        self.characters: List[CharacterInstance] = []
        self.shared_processes: List[subprocess.Popen] = []
        self.running = True
        
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self.logger = logging.getLogger('character_launcher')
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        self.logger.info(f'Received signal {signum}, initiating shutdown...')
        self.running = False
    
    def add_character(self, name: str, config: Dict[str, Any]):
        """Add a character to be launched."""
        character = CharacterInstance(name=name, config=config)
        self.characters.append(character)
        self.logger.info(f'Added character: {name}')
    
    def launch_shared_services(self, map_file: str = None):
        """Launch shared services (LLM service node and map node)."""
        self.logger.info('Launching shared services...')
        
        # Launch LLM service node (shared across all characters)
        try:
            llm_process = subprocess.Popen([
                sys.executable, 'llm_service_node.py'
            ])
            self.shared_processes.append(llm_process)
            self.logger.info('✅ LLM Service Node launched')
        except Exception as e:
            self.logger.error(f'❌ Failed to launch LLM Service Node: {e}')
        
        # Launch map node if map file is specified
        if map_file:
            try:
                map_process = subprocess.Popen([
                    sys.executable, 'map_node.py', '-m', map_file
                ])
                self.shared_processes.append(map_process)
                self.logger.info(f'✅ Map Node launched with map: {map_file}')
            except Exception as e:
                self.logger.error(f'❌ Failed to launch Map Node: {e}')
    
    def launch_character(self, character: CharacterInstance):
        """Launch all nodes for a specific character."""
        self.logger.info(f'Launching character: {character.name}')
        
        # Launch sense_node for this character
        try:
            sense_process = subprocess.Popen([
                sys.executable, 'sense_node.py', 
                '-c', character.name, 
                '-config', json.dumps(character.config)
            ])
            character.processes.append(sense_process)
            self.logger.info(f'✅ {character.name} sense_node launched')
        except Exception as e:
            self.logger.error(f'❌ Failed to launch {character.name} sense_node: {e}')
        
        # Launch memory_node for this character
        try:
            memory_process = subprocess.Popen([
                sys.executable, 'memory_node.py', 
                '-c', character.name, 
                '-config', json.dumps(character.config)
            ])
            character.processes.append(memory_process)
            self.logger.info(f'✅ {character.name} memory_node launched')
        except Exception as e:
            self.logger.error(f'❌ Failed to launch {character.name} memory_node: {e}')
        
        # Launch single_llm_action_example for this character
        try:
            action_process = subprocess.Popen([
                sys.executable, 'single_llm_action_example.py', 
                '-c', character.name, 
                '-config', json.dumps(character.config)
            ])
            character.processes.append(action_process)
            self.logger.info(f'✅ {character.name} single_llm_action_example launched')
        except Exception as e:
            self.logger.error(f'❌ Failed to launch {character.name} single_llm_action_example: {e}')
    
    def launch_all_characters(self, map_file: str = None):
        """Launch all character instances."""
        self.logger.info(f'Launching {len(self.characters)} characters...')
        
        # Launch shared services first
        self.launch_shared_services(map_file)
        time.sleep(2)  # Give shared services time to start
        
        # Launch each character
        for character in self.characters:
            self.launch_character(character)
            time.sleep(1)  # Small delay between characters
        
        self.logger.info('✅ All characters launched')
    
    def monitor_processes(self):
        """Monitor running processes and restart if needed."""
        self.logger.info('Monitoring processes...')
        
        while self.running:
            # Check shared processes
            for i, process in enumerate(self.shared_processes):
                if process.poll() is not None:
                    self.logger.warning(f'Shared process {i} has stopped')
            
            # Check character processes
            for character in self.characters:
                for i, process in enumerate(character.processes):
                    if process.poll() is not None:
                        self.logger.warning(f'{character.name} process {i} has stopped')
            
            time.sleep(5)  # Check every 5 seconds
    
    def shutdown(self):
        """Gracefully shutdown all processes."""
        self.logger.info('Shutting down all processes...')
        
        # Stop monitoring
        self.running = False
        
        # Terminate character processes
        for character in self.characters:
            self.logger.info(f'Terminating {character.name} processes...')
            for process in character.processes:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                except Exception as e:
                    self.logger.error(f'Error terminating process: {e}')
        
        # Terminate shared processes
        self.logger.info('Terminating shared processes...')
        for process in self.shared_processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            except Exception as e:
                self.logger.error(f'Error terminating shared process: {e}')
        
        self.logger.info('✅ All processes terminated')


def main():
    """Main entry point for the character launcher."""
    parser = argparse.ArgumentParser(description='Zenoh Character Launcher')
    parser.add_argument('--config-file', help='YAML or JSON file with character configurations')
    parser.add_argument('--characters', nargs='+', help='Character names to launch')
    parser.add_argument('--list-only', action='store_true', help='List available characters and exit')
    parser.add_argument('--map-file', help='Map file name (e.g., forest.py) to load in the shared map node')
    
    args = parser.parse_args()
    
    launcher = CharacterLauncher()
    
    # Load characters from config file if provided
    if args.config_file:
        try:
            with open(args.config_file, 'r') as f:
                # Try YAML first, fall back to JSON
                try:
                    config_data = yaml.safe_load(f)
                except yaml.YAMLError:
                    # If YAML fails, try JSON
                    f.seek(0)  # Reset file pointer
                    config_data = json.load(f)
            
            # Don't make assumptions about config structure
            # Just iterate through whatever is provided
            if isinstance(config_data, dict):
                # Assume it's a dict of character_name: config
                for name, config in config_data.items():
                    launcher.add_character(name, config)
            elif isinstance(config_data, list):
                # Assume it's a list of character objects
                for char_config in config_data:
                    if isinstance(char_config, dict):
                        # Try to extract name and config
                        name = char_config.get('name', f'character_{len(launcher.characters)}')
                        config = {k: v for k, v in char_config.items() if k != 'name'}
                        launcher.add_character(name, config)
        except Exception as e:
            print(f"Error loading config file: {e}")
            return
    
    # Add characters from command line if provided
    if args.characters:
        for name in args.characters:
            launcher.add_character(name, {})
    
    # If no characters specified, add some defaults
    if not launcher.characters:
        launcher.add_character('default', {})
        launcher.add_character('samantha', {})
    
    if args.list_only:
        print("Available characters:")
        for character in launcher.characters:
            print(f"  - {character.name}")
        return
    
    try:
        # Launch all characters
        launcher.launch_all_characters(args.map_file)
        
        # Monitor processes
        launcher.monitor_processes()
        
    except KeyboardInterrupt:
        print('\nReceived interrupt signal')
    finally:
        launcher.shutdown()


if __name__ == '__main__':
    main() 