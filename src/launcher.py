#!/usr/bin/env python3
"""
Zenoh Character Launcher

This script launches multiple character instances with their respective nodes.
Each character gets its own situation_node and executive_node.
"""

import subprocess
import time
import signal
import sys
import json
import argparse
import logging
import yaml
import os
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
        self.zenoh_session = None
        
        # Configure logging
        # Console handler with WARNING level (less verbose)
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.WARNING)

        # Raise console verbosity when CWB_DEBUG is set
        _debug_env = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        if _debug_env:
            console_handler.setLevel(logging.INFO)
        
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
        if _debug_env:
            self.logger.info('🔧 Debug mode enabled for Launcher (console INFO)')
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Initialize Zenoh session and subscribe to shutdown requests
        try:
            import zenoh
            config = zenoh.Config()
            self.zenoh_session = zenoh.open(config)
            # Listen for centralized shutdown requests
            self.zenoh_session.declare_subscriber(
                "cognitive/launcher/shutdown",
                self._shutdown_request_callback
            )
            # Publisher for ready signal
            self.ready_publisher = self.zenoh_session.declare_publisher("cognitive/launcher/ready")
            self.logger.info('✅ Launcher subscribed to cognitive/launcher/shutdown and ready to publish ready signal')
        except Exception as e:
            self.logger.warning(f'⚠️  Could not initialize Zenoh in launcher for shutdown subscription: {e}')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        self.logger.info(f'Received signal {signum}, initiating shutdown...')
        self.running = False

    def _shutdown_request_callback(self, sample):
        """Handle shutdown request published by UI or other controller."""
        try:
            payload = sample.payload.to_bytes().decode('utf-8')
            self.logger.warning(f'Received shutdown request via Zenoh: {payload}')
        except Exception:
            self.logger.warning('Received shutdown request via Zenoh')
        # Trigger graceful shutdown
        self.running = False
        self.shutdown()
    
    def add_character(self, name: str, config: Dict[str, Any]):
        """Add a character to be launched."""
        canonical_name = name.capitalize()
        character = CharacterInstance(name=canonical_name, config=config)
        self.characters.append(character)
        self.logger.info(f'Added character: {canonical_name}')
    
    def launch_shared_services(self, world_name: str = None, launch_ui: bool = False, launch_resource_browser: bool = False, ui_port: int = 3000, setting: str = None, scenario_name: str = None):
        """Launch shared services (map node, optional UI, optional resource browser)."""
        self.logger.info('Launching shared services...')
        world_label = world_name or scenario_name or 'infospace'
        
        # Launch FastAPI Action Display Node (optional UI)
        if launch_ui:
            try:
                ui_args = [sys.executable, 'fastapi_action_display.py', '--port', str(ui_port)]
                if scenario_name:
                    ui_args.extend(['--scenario', scenario_name])
                ui_process = subprocess.Popen(ui_args)
                self.shared_processes.append(ui_process)
                self.logger.info(f'✅ FastAPI Action Display Node launched on port {ui_port}')
                self.logger.info(f'   - Web UI available at: http://localhost:{ui_port}')
            except Exception as e:
                self.logger.error(f'❌ Failed to launch FastAPI Action Display Node: {e}')
        
        # Launch Resource Browser (optional debug tool)
        if launch_resource_browser:
            try:
                browser_args = [sys.executable, 'resource_browser.py', '--map', world_label, '--port', '3001', '--no-browser']
                browser_process = subprocess.Popen(browser_args)
                self.shared_processes.append(browser_process)
                self.logger.info(f'✅ Resource Browser launched on port 3001')
                self.logger.info(f'   - Access via button in Action Display UI')
            except Exception as e:
                self.logger.error(f'❌ Failed to launch Resource Browser: {e}')
                
        
        # Resource management handled by executive_node
        self.logger.info(f'✅ Resource management handled by executive_node (world: {world_label})')
    
    def launch_character(self, character: CharacterInstance):
        """Launch all nodes for a specific character."""
        self.logger.info(f'Launching character: {character.name}')

    
        
                
        # Launch executive_node for this character (needs memory, situation, and agenda)
        _src_dir = str(Path(__file__).resolve().parent)
        _env = os.environ.copy()
        _env.setdefault('PYTHONPATH', _src_dir)
        if _src_dir not in _env.get('PYTHONPATH', '').split(os.pathsep):
            _env['PYTHONPATH'] = os.pathsep.join([_src_dir, _env.get('PYTHONPATH', '')])
        try:
            executive_process = subprocess.Popen([
                sys.executable, 'executive_node.py',
                '-c', character.name,
                '-config', json.dumps(character.config)
            ], cwd=_src_dir, env=_env)
            character.processes.append(executive_process)
            self.logger.info(f'✅ {character.name} executive_node launched')
        except Exception as e:
            self.logger.error(f'❌ Failed to launch {character.name} executive_node: {e}')
    
    def launch_all_characters(self, world_name: str = None, launch_ui: bool = False, launch_resource_browser: bool = False, ui_port: int = 3000, setting: str = None, scenario_name: str = None):
        """Launch all character instances."""
        self.logger.info(f'Launching {len(self.characters)} characters...')
        
        world_label = world_name or scenario_name or 'infospace'
        
        # Add infospace flag and map name to all character configs
        for character in self.characters:
            character.config['is_infospace'] = True
            character.config['map_name'] = world_label
        
        # Launch shared services first
        self.launch_shared_services(world_name, launch_ui, launch_resource_browser, ui_port, setting, scenario_name)
        time.sleep(2)  # Give shared services time to start
        
        # Launch each character
        for character in self.characters:
            self.launch_character(character)
            time.sleep(1)  # Small delay between characters
        
        self.logger.info('✅ All characters launched')
        
        # Publish ready signal with character count
        try:
            ready_message = {
                'status': 'ready',
                'character_count': len(self.characters),
                'timestamp': time.time()
            }
            self.ready_publisher.put(json.dumps(ready_message))
            self.logger.info(f'🚀 Published ready signal: {len(self.characters)} characters active')
        except Exception as e:
            self.logger.error(f'❌ Failed to publish ready signal: {e}')
    
    def monitor_processes(self):
        """Monitor running processes and restart if needed."""
        self.logger.info('Monitoring processes...')
        
        while self.running:
            # Track if anything is still alive
            any_alive = False

            # Check shared processes
            for i, process in enumerate(self.shared_processes):
                if process.poll() is None:
                    any_alive = True
                else:
                    self.logger.warning(f'Shared process {i} has stopped')
            
            # Check character processes
            for character in self.characters:
                for i, process in enumerate(character.processes):
                    if process.poll() is None:
                        any_alive = True
                    else:
                        self.logger.warning(f'{character.name} process {i} has stopped')

            # If everything has stopped, end monitoring loop
            if not any_alive:
                self.logger.info('All child processes have exited; stopping monitor loop')
                break

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
        shutdown_timeout = 5  # 5 seconds total
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
                self.logger.warning(f'  ⏰ {len(alive_processes)} processes still running after {elapsed}s...')
        
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
        # Close zenoh session if present
        try:
            if self.zenoh_session is not None:
                self.zenoh_session.close()
        except Exception:
            pass


def main():
    """Main entry point for the launcher."""
    parser = argparse.ArgumentParser(description='Zenoh Cognitive Workbench Launcher')
    parser.add_argument('config_file', help='YAML configuration file with character and LLM settings')
    parser.add_argument('--characters', nargs='+', help='Character names to launch (overrides config file)')
    parser.add_argument('--list-only', action='store_true', help='List available characters and exit')
    parser.add_argument('--ui', action='store_true', help='Launch FastAPI web UI')
    parser.add_argument('--ui-port', type=int, default=3000, help='Port for FastAPI web UI (default: 3000)')
    parser.add_argument('--resource-browser', action='store_true', help='Launch Resource Browser for debugging (port 3001)')
    parser.add_argument('--scenario-id', type=str, default=None, help='Optional scenario identifier (forwarded, not used yet)')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode (disables map turn timeout)')
    
    args = parser.parse_args()
    
    launcher = CharacterLauncher()
    # Propagate debug mode to environment (read by map_node)
    if args.debug:
        os.environ['CWB_DEBUG'] = '1'
        launcher.logger.warning('Debug mode enabled: map_node will disable turn timeout')
    
    # Load configuration from file
    try:
        # Assume config files are in scenarios subdirectory
        config_path = os.path.join('../scenarios', args.config_file)
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
        
        # Extract scenario name from config filename (e.g., "laTerre.yaml" -> "laTerre")
        scenario_name = Path(args.config_file).stem
        
        # Extract LLM configuration (still needed by executive_node for SGLang)
        llm_config = config_data.get('llm_config', {})
        
        # Extract world configuration (if present) - generalized for any external world API
        world_config = config_data.get('world_config', {})
        world_name = world_config.get('world_name') if world_config else None
        
        setting = config_data.get('setting', {})
        # Optional: scenario-level max_turns (documented in YAML; can be commented out with #)
        max_turns_yaml = config_data.get('max_turns', None)
        if max_turns_yaml is not None:
            try:
                os.environ['CWB_MAX_TURNS'] = str(int(max_turns_yaml))
            except Exception:
                pass

        # Extract characters configuration
        characters_config = config_data.get('characters', [])
        ontology = config_data.get('Ontology', False)
        activities = config_data.get('Activities', False)
        if isinstance(characters_config, dict):
            # Handle dict format: character_name: config
            for name, config in characters_config.items():
                new_config = config.copy()
                new_config['ontology'] = ontology
                new_config['activities'] = activities
                new_config['characters'] = characters_config.copy()
                new_config['llm_config'] = llm_config
                new_config['world_config'] = world_config
                new_config['setting'] = setting
                launcher.add_character(name, new_config)
        elif isinstance(characters_config, list):
            # Handle list format: [{'name': 'Alice', ...}, ...]
            for char_config in characters_config:
                new_config = char_config.copy()
                if isinstance(char_config, dict):
                    name = char_config.get('name', f'character_{len(launcher.characters)}')
                    new_config = {k: v for k, v in char_config.items() if k != 'name'}
                    new_config['ontology'] = ontology
                    new_config['activities'] = activities
                    new_config['characters'] = characters_config
                    new_config['llm_config'] = llm_config
                    new_config['world_config'] = world_config
                    new_config['setting'] = setting
                    launcher.add_character(name, new_config)
        
    except Exception as e:
        print(f"Error loading config file '{config_path}': {e}")
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
        # Use world_name from world_config as authoritative, fallback to scenario_name
        final_world_name = world_name or scenario_name or 'infospace'
        world_file = Path(f"data/world/{final_world_name}_world.json")

        if world_file.exists():
            print(f"\nFound existing world data for '{final_world_name}'")
            reuse = input("Reuse existing world? (y/n): ").strip().lower()
            if reuse in ['n', 'no']:
                confirm_delete = input("Are you sure you want to delete existing world data and create new? (y/n): ").strip().lower()
                if confirm_delete == 'y':
                    print("Creating new world...")
                    try:
                        world_file.unlink()
                        print(f"Removed existing world data for '{final_world_name}'")
                    except Exception as e:
                        print(f"Failed to remove existing world data: {e}")
                    
                    data_dir = Path("data")
                    if data_dir.exists():
                        character_names = [char.name for char in launcher.characters]
                        
                        memory_dir = data_dir / "memory"
                        if memory_dir.exists():
                            for mem_file in memory_dir.glob("*_memory.json"):
                                char_name = mem_file.stem.replace('_memory', '')
                                if char_name in character_names:
                                    try:
                                        mem_file.unlink()
                                        print(f"Removed existing memory data: {mem_file.name}")
                                    except Exception:
                                        pass
                        
                        situation_dir = data_dir / "situation"
                        if situation_dir.exists():
                            for sit_file in situation_dir.glob("*_situation.json"):
                                char_name = sit_file.stem.replace('_situation', '')
                                if char_name in character_names:
                                    try:
                                        sit_file.unlink()
                                        print(f"Removed existing situation data: {sit_file.name}")
                                    except Exception:
                                        pass
                        
                        rag_stores_dir = data_dir / "rag_stores"
                        if rag_stores_dir.exists():
                            for char_name in character_names:
                                char_rag_dir = rag_stores_dir / char_name
                                if char_rag_dir.exists():
                                    try:
                                        import shutil
                                        shutil.rmtree(char_rag_dir)
                                        print(f"Removed existing RAG store: {char_name}")
                                    except Exception as e:
                                        print(f"Failed to remove RAG store for {char_name}: {e}")
                else:
                    print(f"Reusing existing world '{final_world_name}'")
            else:
                print(f"Reusing existing world '{final_world_name}'")

        launcher.launch_all_characters(world_name, args.ui, args.resource_browser, ui_port=args.ui_port, setting=setting, scenario_name=scenario_name)
        
        # Monitor processes
        launcher.monitor_processes()
        
    except KeyboardInterrupt:
        print('\nReceived interrupt signal')
    finally:
        launcher.shutdown()
        os._exit(0)


if __name__ == '__main__':
    main()
