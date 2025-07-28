#!/usr/bin/env python3
"""
Zenoh Character Launcher

This script launches multiple character instances with their respective nodes.
Each character gets its own sense_node, memory_node, and executive_node.
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
from pathlib import Path
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
        # Console handler with WARNING level (less verbose)
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.WARNING)
        
        # File handler with INFO level (full logging)
        file_handler = logging.FileHandler('logs/character_launcher.log', mode='w')
        file_handler.setLevel(logging.INFO)
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            handlers=[console_handler, file_handler],
            force=True
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
        canonical_name = name.capitalize()
        character = CharacterInstance(name=canonical_name, config=config)
        self.characters.append(character)
        self.logger.info(f'Added character: {canonical_name}')
    
    def launch_shared_services(self, map_file: str = None, launch_ui: bool = False):
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
        
        # Launch FastAPI Action Display Node (optional UI)
        if launch_ui:
            try:
                ui_process = subprocess.Popen([
                    sys.executable, 'fastapi_action_display.py', '--port', '3000'
                ])
                self.shared_processes.append(ui_process)
                self.logger.info('✅ FastAPI Action Display Node launched on port 3000')
                self.logger.info('   - Web UI available at: http://localhost:3000')
            except Exception as e:
                self.logger.error(f'❌ Failed to launch FastAPI Action Display Node: {e}')
                
        
        # Launch map node (required for situation awareness)
        try:
            map_args = [sys.executable, 'map_node.py']
            if map_file:
                # Check for existing world data
                world_name = map_file.replace('.py', '')
                world_file = Path(f"data/world/{world_name}_world.json")
                
                if world_file.exists():
                    print(f"\nFound existing world data for '{world_name}'")
                    reuse = input("Reuse existing world? (y/n): ").strip().lower()
                    if reuse != 'y':
                        print("Creating new world...")
                        # Remove existing world file
                        world_file.unlink()
                        print(f"Removed existing world data for '{world_name}'")
                        
                        # Remove existing character data from all subdirectories
                        data_dir = Path("data")
                        if data_dir.exists():
                            # Remove memory files
                            memory_dir = data_dir / "memory"
                            if memory_dir.exists():
                                for mem_file in memory_dir.glob("*_memory.json"):
                                    mem_file.unlink()
                                    print(f"Removed existing memory data: {mem_file.name}")
                            
                            # Remove situation files
                            situation_dir = data_dir / "situation"
                            if situation_dir.exists():
                                for sit_file in situation_dir.glob("*_situation.json"):
                                    sit_file.unlink()
                                    print(f"Removed existing situation data: {sit_file.name}")
                    else:
                        print(f"Reusing existing world '{world_name}'")
                
                map_args.extend(['-m', map_file, '-w', world_name])
            map_process = subprocess.Popen(map_args)
            self.shared_processes.append(map_process)
            self.logger.info(f'✅ Map Node launched' + (f' with map: {map_file}' if map_file else ''))
            
            # Wait for map node to be ready (check for initialization message)
            self.logger.info('⏳ Waiting for Map Node to initialize...')
            time.sleep(5)  # Give map node time to start up
        except Exception as e:
            self.logger.error(f'❌ Failed to launch Map Node: {e}')
    
    def launch_character(self, character: CharacterInstance):
        """Launch all nodes for a specific character."""
        self.logger.info(f'Launching character: {character.name}')
        
        # Launch memory_node for this character (1st - provides storage)
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
        
        # Small delay to ensure memory_node initializes
        time.sleep(0.5)
        
        # Launch situation_node for this character (2nd - needs map_node)
        try:
            situation_process = subprocess.Popen([
                sys.executable, 'situation_node.py', 
                '-c', character.name, 
                '-config', json.dumps(character.config)
            ])
            character.processes.append(situation_process)
            self.logger.info(f'✅ {character.name} situation_node launched')
        except Exception as e:
            self.logger.error(f'❌ Failed to launch {character.name} situation_node: {e}')
        
        # Small delay to ensure situation_node initializes
        time.sleep(0.5)
        
        # Launch sense_node for this character (3rd - provides input)
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
        
        # Small delay to ensure sense_node initializes
        time.sleep(0.5)
        
        # Launch executive_node for this character (4th - needs memory and sense)
        try:
            executive_process = subprocess.Popen([
                sys.executable, 'executive_node.py', 
                '-c', character.name, 
                '-config', json.dumps(character.config)
            ])
            character.processes.append(executive_process)
            self.logger.info(f'✅ {character.name} executive_node launched')
        except Exception as e:
            self.logger.error(f'❌ Failed to launch {character.name} executive_node: {e}')
    
    def launch_all_characters(self, map_file: str = None, launch_ui: bool = False):
        """Launch all character instances."""
        self.logger.info(f'Launching {len(self.characters)} characters...')
        
        # Launch shared services first
        self.launch_shared_services(map_file, launch_ui)
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
        """Gracefully shutdown all processes using standard Zenoh pattern."""
        self.logger.info('🛑 Initiating graceful shutdown...')
        
        # Stop monitoring
        self.running = False
        
        # Step 1: Send SIGTERM to ALL processes (they handle shutdown themselves)
        all_processes = []
        
        # Collect all character processes
        for character in self.characters:
            for process in character.processes:
                all_processes.append((f'{character.name}', process))
        
        # Collect all shared processes
        for process in self.shared_processes:
            all_processes.append(('shared', process))
        
        # Send SIGTERM to all processes simultaneously
        self.logger.info(f'📨 Sending SIGTERM to {len(all_processes)} processes...')
        for name, process in all_processes:
            try:
                process.terminate()
                self.logger.info(f'  ✉️  SIGTERM sent to {name} process {process.pid}')
            except Exception as e:
                self.logger.error(f'  ❌ Error sending SIGTERM to {name} process {process.pid}: {e}')
        
        # Step 2: Wait for graceful shutdown (standard timeout)
        shutdown_timeout = 30  # 30 seconds total
        self.logger.info(f'⏳ Waiting up to {shutdown_timeout}s for graceful shutdown...')
        
        start_time = time.time()
        while time.time() - start_time < shutdown_timeout:
            alive_processes = [p for _, p in all_processes if p.poll() is None]
            if not alive_processes:
                self.logger.info('✅ All processes shut down gracefully')
                return
            
            time.sleep(1)  # Check every second
            elapsed = int(time.time() - start_time)
            if elapsed % 5 == 0:  # Log every 5 seconds
                self.logger.info(f'  ⏰ {len(alive_processes)} processes still running after {elapsed}s...')
        
        # Step 3: Force kill any remaining processes
        remaining_processes = [(name, p) for name, p in all_processes if p.poll() is None]
        if remaining_processes:
            self.logger.warning(f'⚠️  Force killing {len(remaining_processes)} unresponsive processes...')
            for name, process in remaining_processes:
                try:
                    process.kill()
                    self.logger.warning(f'  💀 Force killed {name} process {process.pid}')
                except Exception as e:
                    self.logger.error(f'  ❌ Error force killing {name} process {process.pid}: {e}')
        
        self.logger.info('✅ Shutdown complete')


def main():
    """Main entry point for the character launcher."""
    parser = argparse.ArgumentParser(description='Zenoh Character Launcher')
    parser.add_argument('--config-file', help='YAML or JSON file with character configurations')
    parser.add_argument('--characters', nargs='+', help='Character names to launch')
    parser.add_argument('--list-only', action='store_true', help='List available characters and exit')
    parser.add_argument('--map-file', help='Map file name (e.g., forest.py) to load in the shared map node')
    parser.add_argument('--ui', action='store_true', help='Launch FastAPI web UI on port 3000')
    
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
        launcher.launch_all_characters(args.map_file, args.ui)
        
        # Monitor processes
        launcher.monitor_processes()
        
    except KeyboardInterrupt:
        print('\nReceived interrupt signal')
    finally:
        launcher.shutdown()


if __name__ == '__main__':
    main() 