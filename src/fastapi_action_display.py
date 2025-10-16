#!/usr/bin/env python3
"""
FastAPI Action Display Node - FIXED VERSION

This node displays actions via a web UI and provides text input interface.
Replaces the console-based action_display_node with a web interface.
"""

import zenoh
import json
import time
import threading
import argparse
import webbrowser
import asyncio
import signal
import logging
import shutil
from datetime import datetime
from typing import Dict, List, Any, Set
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse
import uvicorn
from pathlib import Path
from concurrent.futures import TimeoutError
import os

# Set up logging
logger = logging.getLogger(__name__)
console_handler = logging.StreamHandler()
file_handler = logging.FileHandler('logs/fastapi_action_display.log')
console_handler.setLevel(logging.WARNING)
file_handler.setLevel(logging.DEBUG)

# Raise console verbosity when CWB_DEBUG is set
_debug_env = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
if _debug_env:
    console_handler.setLevel(logging.INFO)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
file_handler.setFormatter(formatter)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[console_handler, file_handler],
    force=True
)


class ActionTracer:
    """Records actions to a markdown trace file for analysis."""
    
    def __init__(self, scenario_name: str = None):
        self.scenario_name = scenario_name or "unknown"
        self.trace_file = None
        self.start_time = datetime.now()
        self._init_trace_file()
    
    def _init_trace_file(self):
        """Initialize trace file with header."""
        trace_dir = Path('logs')
        trace_dir.mkdir(exist_ok=True)
        
        trace_path = trace_dir / f"action_trace_{self.scenario_name}.md"
        self.trace_file = open(trace_path, 'w')
        
        # Write header
        self.trace_file.write(f"# Action Trace: {self.scenario_name}\n")
        self.trace_file.write(f"Started: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        self.trace_file.flush()
        
        logger.info(f"Action trace initialized: {trace_path}")
    
    def log_action(self, action_data: dict, character_name: str):
        """Log an action to the trace file."""
        if not self.trace_file:
            return
        
        try:
            # Extract timestamp (HH:MM:SS format)
            timestamp_str = action_data.get('timestamp', '')
            if timestamp_str:
                try:
                    ts = datetime.fromisoformat(timestamp_str)
                    time_str = ts.strftime('%H:%M:%S')
                except:
                    time_str = datetime.now().strftime('%H:%M:%S')
            else:
                time_str = datetime.now().strftime('%H:%M:%S')
            
            # Extract action type
            action_type = action_data.get('type', 'unknown')
            
            # Skip announcements
            if action_type == 'announcement':
                return
            
            # Build action description
            parts = [f"{time_str} **{character_name}** {action_type}"]
            
            # Add action-specific details
            if action_type == 'say' or action_type == 'response':
                target = action_data.get('target', '')
                text = action_data.get('text', '')
                if target:
                    parts.append(f" to {target}:")
                else:
                    parts.append(":")
                parts.append(f' "{text}"')
            elif action_type == 'think':
                thought = action_data.get('thought', action_data.get('text', ''))
                parts.append(f': "{thought}"')
            elif action_type == 'move':
                target = action_data.get('target', action_data.get('location', ''))
                if target:
                    parts.append(f" to {target}")
            elif action_type == 'scan':
                target = action_data.get('target', '')
                result = action_data.get('result', '')
                out_var = action_data.get('out', '')
                if target:
                    parts.append(f" {target}")
                if out_var and result:
                    parts.append(f" -> {out_var}={result}")
            elif action_type in ('take', 'inspect', 'use', 'place'):
                # These use 'target', 'resolved_target', or 'requested_target'
                target = action_data.get('target', '') or action_data.get('resolved_target', '') or action_data.get('requested_target', '')
                if target:
                    parts.append(f" {target}")
            
            # Add goal if present
            goal = action_data.get('goal', '')
            if goal and isinstance(goal, str):
                parts.append(f" (goal: {goal})")
            
            # Add status/error if present
            status = action_data.get('status', '')
            error = action_data.get('error', '')
            
            if status and status.lower() in ('failure', 'failed', 'error'):
                if error:
                    parts.append(f" (failed, {error})")
                else:
                    parts.append(" (failed)")
            elif error:
                # Error present even without explicit failure status
                parts.append(f" (failed, {error})")
            
            # Build final line and skip if truly empty (just action type, no details)
            line = ''.join(parts)
            # Check if line has any content beyond timestamp + character + action_type
            minimal_line = f"{time_str} **{character_name}** {action_type}"
            if line.strip() == minimal_line.strip():
                # No details whatsoever - skip duplicate/empty
                logger.debug(f"Skipping empty action entry: {line}")
                return
            
            # Write to file
            self.trace_file.write(line + '\n')
            self.trace_file.flush()
            
        except Exception as e:
            logger.error(f"Error logging action to trace: {e}")
    
    def close(self):
        """Close the trace file."""
        if self.trace_file:
            self.trace_file.close()
            self.trace_file = None


class FastAPIActionDisplayNode:
    """
    The FastAPI Action Display node provides:
    - Web UI for displaying incoming actions
    - Text input interface via web
    - Action history and statistics
    """
    
    def __init__(self, port: int = 3000, scenario_name: str = None, obsidian_vault: str = None):
        # Initialize FastAPI app
        self.app = FastAPI(title="Zenoh Action Display")
        self.port = port
        
        # Debug mode flag
        self.debug = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        if self.debug:
            logger.info(f'🔧 Debug mode enabled for FastAPI Action Display')
        
        # Initialize action tracer
        self.tracer = ActionTracer(scenario_name=scenario_name)
        
        # Store Obsidian vault path if provided
        self.obsidian_vault = obsidian_vault
        if self.obsidian_vault:
            vault_path = Path(self.obsidian_vault)
            if vault_path.exists():
                logger.info(f'📓 Obsidian vault configured: {self.obsidian_vault}')
            else:
                logger.warning(f'⚠️  Obsidian vault path does not exist: {self.obsidian_vault}')
        
        # Store scenario name for export
        self.scenario_name = scenario_name or "unknown"
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Publisher for User actions (so User text appears in UI/trace)
        self.user_action_publisher = self.session.declare_publisher("cognitive/User/action")
        
        # Track active characters
        self.active_characters: Set[str] = set()
        self.character_publishers: Dict[str, Any] = {}
        self.last_character_name: str = None
        
        # Character goals tracking
        self.character_goals: Dict[str, str] = {}  # character_name -> current_goal_string
        self.character_goals_lock = threading.Lock()
        
        # Character decided actions tracking
        self.character_decided_actions: Dict[str, str] = {}  # character_name -> decided_action_string
        self.character_decided_actions_lock = threading.Lock()
        
        # Character current plans tracking
        self.character_current_plans: Dict[str, str] = {}  # character_name -> current_plan_string
        self.character_current_plans_lock = threading.Lock()
        
        # Character current activities tracking
        self.character_current_activities: Dict[str, Dict[str, Any]] = {}  # character_name -> current_activity_data
        self.character_current_activities_lock = threading.Lock()
        
        # Character situation data tracking
        self.character_situation_data: Dict[str, Dict[str, Any]] = {}  # character_name -> situation_data
        self.character_situation_data_lock = threading.Lock()
        
        # Turn state tracking with proper locking
        self.turn_state_lock = threading.Lock()
        self.turn_state = {
            'active_characters': [],
            'completed_characters': [],
            'turn_number': 0,
            'turn_start_time': None,
            'mode': 'step'  # 'step' or 'run'
        }
        
        # System ready state
        self.system_ready = False
        self.character_count = 0
        
        # Subscriber for all character actions
        self.action_subscriber = self.session.declare_subscriber(
            "cognitive/*/action",
            self.action_callback
        )
        
        # Subscriber for step complete events from map node
        self.step_complete_subscriber = self.session.declare_subscriber(
            "cognitive/map/step_complete",
            self.step_complete_callback
        )
        
        # Subscriber for turn start events
        self.turn_start_subscriber = self.session.declare_subscriber(
            "cognitive/map/turn",
            self.turn_start_callback
        )
        
        # Subscriber for turn control status from map node
        self.turn_control_subscriber = self.session.declare_subscriber(
            "cognitive/map/turn_status",
            self.turn_control_callback
        )
        
        # NEW: Subscriber for unified turn state updates from map node
        self.turn_state_update_subscriber = self.session.declare_subscriber(
            "cognitive/map/turn_state_update",
            self.turn_state_update_callback
        )
        
        # Subscriber for character goals
        self.goal_subscriber = self.session.declare_subscriber(
            "cognitive/*/goal",
            self.goal_callback
        )
        
        # Subscriber for character decided actions
        self.decided_action_subscriber = self.session.declare_subscriber(
            "cognitive/*/decided_action",
            self.decided_action_callback
        )
        
        # Subscriber for character current plans
        self.current_plan_subscriber = self.session.declare_subscriber(
            "cognitive/*/current_plan",
            self.current_plan_callback
        )
        
        # Subscriber for character current activities
        self.current_activity_subscriber = self.session.declare_subscriber(
            "cognitive/*/current_activity",
            self.current_activity_callback
        )
        # Subscriber for character current state
        self.current_state_subscriber = self.session.declare_subscriber(
            "cognitive/*/current_state",
            self.current_state_callback
        )
        
        # Subscriber for character situation data
        self.situation_subscriber = self.session.declare_subscriber(
            "cognitive/*/situation/update",
            self.situation_callback
        )
        
        # Subscriber for launcher ready signal
        self.ready_subscriber = self.session.declare_subscriber(
            "cognitive/launcher/ready",
            self.ready_callback
        )
        
        # Publisher for memory storage
        self.memory_publisher = self.session.declare_publisher("cognitive/memory/store")
        
        # Publishers for turn control
        self.turn_step_publisher = self.session.declare_publisher("cognitive/map/turn/step")
        self.turn_run_publisher = self.session.declare_publisher("cognitive/map/turn/run")
        self.turn_stop_publisher = self.session.declare_publisher("cognitive/map/turn/stop")
        
        # Publisher for time management
        self.time_delay_publisher = self.session.declare_publisher("cognitive/map/time_delay_setting")
        
        # Subscriber for time advancement updates
        self.time_advanced_subscriber = self.session.declare_subscriber(
            "cognitive/map/time_advanced",
            self.time_advanced_callback
        )
        
        # Publisher for save commands
        self.save_publisher = self.session.declare_publisher("cognitive/save_all")
        
        # Current simulation time tracking
        self.current_simulation_time = None
        
        # Publishers for shutdown commands
        self.shutdown_executive_publisher = self.session.declare_publisher("cognitive/shutdown/executive")
        self.shutdown_sense_publisher = self.session.declare_publisher("cognitive/shutdown/sense") 
        self.shutdown_memory_publisher = self.session.declare_publisher("cognitive/shutdown/memory")
        self.shutdown_situation_publisher = self.session.declare_publisher("cognitive/shutdown/situation")
        self.shutdown_shared_publisher = self.session.declare_publisher("cognitive/shutdown/shared")
        # Centralized launcher shutdown publisher (preferred path)
        self.launcher_shutdown_publisher = self.session.declare_publisher("cognitive/launcher/shutdown")
        
        # Shutdown coordination state
        self.shutdown_in_progress = False
        self.shutdown_acknowledgments = {}
        
        # Internal state
        self.action_history = []
        self.max_history = 100
        self.action_counter = 0
        
        # WebSocket connections with proper locking
        self.websocket_connections: List[WebSocket] = []
        self.websocket_lock = threading.Lock()
        
        # Event loop reference (will be set when FastAPI starts)
        self.event_loop = None
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Setup FastAPI routes
        self._setup_routes()
        
        print(f'🖥️  FastAPI Action Display Node initialized on port {port}')
        print('   - Subscribing to: cognitive/*/action (all characters)')
        print('   - Subscribing to: cognitive/*/goal (character goals)')
        print('   - Subscribing to: cognitive/*/decided_action (character decided actions)')
        print('   - Subscribing to: cognitive/*/current_plan (character current plans)')
        print('   - Subscribing to: cognitive/*/current_state (character current state)')
        print('   - Subscribing to: cognitive/map/step_complete (step completion)')
        print('   - Subscribing to: cognitive/map/turn (turn start)')
        print('   - Publishing to: cognitive/{character}/sense_data (dynamic)')
        print('   - Publishing to: cognitive/memory/store')
        print(f'   - Web UI available at: http://localhost:{port}')
        if self.obsidian_vault and Path(self.obsidian_vault).exists():
            print(f'   - 📓 Obsidian export enabled: {self.obsidian_vault}')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        print(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def _setup_routes(self):
        """Setup FastAPI routes."""
        
        @self.app.get("/", response_class=HTMLResponse)
        async def get_main_page():
            return self._get_html_template()
        
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            
            with self.websocket_lock:
                self.websocket_connections.append(websocket)
        
            
            # Set event loop reference when first WebSocket connects
            if self.event_loop is None:
                self._set_event_loop()
            
            # Send a test message to verify connection
            try:
                test_data = {
                    'type': 'test',
                    'message': 'WebSocket connection established successfully'
                }
                await websocket.send_text(json.dumps(test_data))
                
                # Send current simulation time if available
                if self.current_simulation_time:
                    time_data = {
                        'type': 'time_update',
                        'time_info': self.current_simulation_time.get('time_info', {}),
                        'timestamp': datetime.now().isoformat()
                    }
                    await websocket.send_text(json.dumps(time_data))
                    logger.info(f'📤 Sent current time to new WebSocket connection: {time_data["time_info"].get("datetime", "unknown")}')
                else:
                    logger.info('📤 No simulation time available for new WebSocket connection')
            except Exception:
                pass  # Ignore test message errors
            
            try:
                while True:
                    # Keep connection alive - handle both text and binary messages
                    try:
                        data = await websocket.receive()
                        # Handle ping messages to keep connection alive
                        if data.get("type") == "websocket.receive":
                            if "text" in data:
                                text = data["text"]
                                if text == "ping":
                                    # Send pong response
                                    await websocket.send_text("pong")
                                continue
                    except Exception:
                        break
            except WebSocketDisconnect:
                pass
            except Exception:
                pass
            finally:
                with self.websocket_lock:
                    if websocket in self.websocket_connections:
                        self.websocket_connections.remove(websocket)
        
        @self.app.post("/api/text_input")
        async def send_text_input(data: Dict[str, str]):
            character_name = data.get('character', '')
            message = data.get('message', '')
            
            if not message:
                return {"error": "Message is required"}
            
            # If no character specified, use last character
            if not character_name:
                if not self.last_character_name:
                    return {"error": "No character specified and no previous character"}
                character_name = self.last_character_name
            
            # Find actual character name (case-insensitive)
            actual_character_name = None
            for active_char in self.active_characters:
                if active_char.lower() == character_name.lower():
                    actual_character_name = active_char
                    break
            
            if not actual_character_name:
                return {"error": f"Character '{character_name}' not found. Available: {', '.join(sorted(self.active_characters))}"}
            
            # Store as last character used
            self.last_character_name = actual_character_name
            
            # Get or create publisher for this character
            if actual_character_name not in self.character_publishers:
                self.character_publishers[actual_character_name] = self.session.declare_publisher(
                    f"cognitive/{actual_character_name}/sense_data"
                )
            
            # Publish sense data directly
            sense_data = {
                'timestamp': datetime.now().isoformat(),
                'sequence_id': 0,
                'mode': 'text',
                'content': json.dumps({
                    'source': 'User',
                    'text': message
                })
            }
            self.character_publishers[actual_character_name].put(json.dumps(sense_data))
            
            # Publish User's action so it appears in UI/trace
            user_action = {
                'type': 'say',
                'action_id': f'user_say_{int(time.time() * 1000)}',
                'timestamp': datetime.now().isoformat(),
                'text': message,
                'source': 'User',
                'target': actual_character_name
            }
            self.user_action_publisher.put(json.dumps(user_action))
            
            # Store in memory
            self._store_text_input_in_memory(message, actual_character_name)
            
            return {"success": True, "message": f"Sent to {actual_character_name}: {message}"}
        
        @self.app.post("/api/turn/step")
        async def step_turn():
            """Advance one turn for each character."""
            try:
                with self.turn_state_lock:
                    self.turn_state['mode'] = 'step'
                
                self.turn_step_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                
                return {"success": True, "message": "Step Turn command sent"}
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/turn/run")
        async def run_turns():
            """Start automatic turn progression."""
            try:
                logger.info(f"🏃 Run command received in FastAPI")
                
                self.turn_run_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                logger.info(f"🏃 Run command published to map node")
                
                # Note: Turn state will be updated when map_node publishes the status change
                return {"success": True, "message": "Run Turns command sent"}
            except Exception as e:
                logger.error(f"Error in run_turns: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/turn/stop")
        async def stop_turns():
            """Stop automatic turn progression."""
            try:
                logger.info(f"🛑 Stop command received in FastAPI")
                
                self.turn_stop_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                logger.info(f"🛑 Stop command published to map node")
                
                # Note: Turn state will be updated when map_node publishes the status change
                return {"success": True, "message": "Stop Turns command sent"}
            except Exception as e:
                logger.error(f"Error in stop_turns: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/save")
        async def save_all():
            """Trigger save for all nodes."""
            try:
                # Broadcast save command to all nodes
                save_data = {
                    "timestamp": datetime.now().isoformat(),
                    "source": "User"
                }
                self.save_publisher.put(json.dumps(save_data))
                
                return {"success": True, "message": "Save command sent to all nodes"}
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/export_to_obsidian")
        async def export_to_obsidian():
            """Export current trace file to Obsidian vault."""
            try:
                # Check if Obsidian vault is configured
                if not self.obsidian_vault:
                    return {"success": False, "message": "Obsidian vault not configured. Use --obsidian-vault argument."}
                
                # Verify vault path exists
                vault_path = Path(self.obsidian_vault)
                if not vault_path.exists():
                    return {"success": False, "message": f"Obsidian vault path does not exist: {self.obsidian_vault}"}
                
                # Check if trace file exists
                if not self.tracer.trace_file:
                    return {"success": False, "message": "No trace file is currently open"}
                
                # Flush the trace file to ensure all data is written
                self.tracer.trace_file.flush()
                
                # Create target directory
                target_dir = vault_path / "Cognitive_Workbench_Traces"
                target_dir.mkdir(exist_ok=True)
                
                # Generate timestamped filename
                timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                target_filename = f"action_trace_{self.scenario_name}_{timestamp}.md"
                target_path = target_dir / target_filename
                
                # Get source trace file path
                source_path = Path('logs') / f"action_trace_{self.scenario_name}.md"
                
                if not source_path.exists():
                    return {"success": False, "message": f"Source trace file not found: {source_path}"}
                
                # Copy the file
                shutil.copy2(source_path, target_path)
                
                logger.info(f"📓 Exported trace to Obsidian: {target_path}")
                
                return {
                    "success": True, 
                    "message": f"Exported to Obsidian: {target_filename}",
                    "path": str(target_path)
                }
                
            except Exception as e:
                logger.error(f"Error exporting to Obsidian: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/save_and_shutdown")
        async def save_and_shutdown():
            """Save data then request centralized shutdown via launcher."""
            try:
                if self.shutdown_in_progress:
                    return {"success": False, "message": "Shutdown already in progress"}
                
                # First save all data
                save_data = {
                    "timestamp": datetime.now().isoformat(),
                    "source": "ui_shutdown"
                }
                self.save_publisher.put(json.dumps(save_data))
                
                # Wait briefly for saves to complete
                await asyncio.sleep(2.0)
                
                # Request launcher shutdown
                try:
                    self.launcher_shutdown_publisher.put(json.dumps({
                        "timestamp": datetime.now().isoformat(),
                        "source": "ui",
                        "mode": "save_and_shutdown"
                    }))
                except Exception:
                    pass
                return {"success": True, "message": "Save and shutdown requested"}
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/shutdown")
        async def shutdown_only():
            """Request centralized shutdown via launcher (no save)."""
            try:
                if self.shutdown_in_progress:
                    return {"success": False, "message": "Shutdown already in progress"}
                try:
                    self.launcher_shutdown_publisher.put(json.dumps({
                        "timestamp": datetime.now().isoformat(),
                        "source": "ui",
                        "mode": "shutdown"
                    }))
                except Exception:
                    pass
                return {"success": True, "message": "Shutdown requested"}
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/time/delay")
        async def set_time_delay(request: Request):
            """Set the delay between turns in run mode."""
            try:
                data = await request.json()
                minutes = data.get('minutes')
                
                if not isinstance(minutes, int) or not (0 <= minutes <= 30):
                    return {"success": False, "message": "Minutes must be an integer between 0 and 30"}
                
                # Send time delay setting to map_node
                delay_data = {
                    "delay_minutes": minutes,
                    "timestamp": datetime.now().isoformat()
                }
                
                if hasattr(self, 'time_delay_publisher'):
                    self.time_delay_publisher.put(json.dumps(delay_data))
                    return {"success": True, "message": f"Turn delay set to {minutes} minutes"}
                else:
                    return {"success": False, "message": "Time delay publisher not available"}
                    
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/time/initial")
        async def get_initial_time():
            """Get initial simulation time information."""
            try:
                if hasattr(self, 'session'):
                    # If we have cached time, return it
                    if self.current_simulation_time:
                        return {"success": True, "time_info": self.current_simulation_time}
                    
                    # Otherwise, try to query map_node for current time
                    try:
                        replies = self.session.get("cognitive/map/simulation_time", timeout=5.0 if not self.debug else 300.0)
                        for reply in replies:
                            if reply.ok:
                                response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                                if response.get('success'):
                                    # Cache the time for future requests
                                    self.current_simulation_time = response
                                    return {"success": True, "time_info": response}
                        
                        return {"success": False, "message": "No response from map_node"}
                    except Exception as query_error:
                        logger.warning(f"Failed to query map_node for initial time: {query_error}")
                        return {"success": False, "message": "Map node not ready yet"}
                else:
                    return {"success": False, "message": "Zenoh session not available"}
                    
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/relation/{character_name}/{target_character}")
        async def get_relation_data(character_name: str, target_character: str):
            """Get relation data (discourse_state and tom_model) for a character pair."""
            try:
                if not hasattr(self, 'session'):
                    return {"success": False, "message": "Zenoh session not available", "no_interaction": False}
                
                # Query memory_node for relation data
                query_key = f"cognitive/{character_name}/memory/entity/{target_character}?query=relation"
                
                try:
                    replies = self.session.get(query_key, timeout=5.0 if not self.debug else 300.0)
                    for reply in replies:
                        try:
                            # Handle different Zenoh reply object types
                            if hasattr(reply, 'ok') and reply.ok is not None:
                                payload_bytes = reply.ok.payload.to_bytes()
                            elif hasattr(reply, 'payload'):
                                payload_bytes = reply.payload.to_bytes()
                            else:
                                continue
                            
                            response = json.loads(payload_bytes.decode('utf-8'))
                            
                            if response.get('success'):
                                return {
                                    "success": True,
                                    "discourse_state": response.get('discourse_state', ''),
                                    "tom_model": response.get('tom_model', ''),
                                    "no_interaction": False
                                }
                            else:
                                # Check if entity not found (no interaction yet)
                                error_msg = response.get('error', '')
                                if 'not found' in error_msg.lower():
                                    return {
                                        "success": True,
                                        "discourse_state": '',
                                        "tom_model": '',
                                        "no_interaction": True
                                    }
                                return {"success": False, "message": error_msg, "no_interaction": False}
                        except Exception as parse_error:
                            logger.warning(f"Failed to parse reply for {character_name}->{target_character}: {parse_error}")
                            continue
                    
                    # No valid response means entity doesn't exist yet (no interaction)
                    return {
                        "success": True,
                        "discourse_state": '',
                        "tom_model": '',
                        "no_interaction": True
                    }
                except Exception as query_error:
                    logger.warning(f"Failed to query relation data for {character_name}->{target_character}: {query_error}")
                    return {"success": False, "message": f"Query error: {str(query_error)}", "no_interaction": False}
                    
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}", "no_interaction": False}

    
    async def _initiate_shutdown(self):
        """Coordinate system shutdown in proper sequence."""
        self.shutdown_in_progress = True
        
        shutdown_data = {
            "timestamp": datetime.now().isoformat(),
            "source": "ui",
            "characters": list(self.active_characters)
        }
        
        # Step 1: Shutdown executive nodes for each character
        logger.info("🔌 Step 1: Shutting down executive nodes...")
        self.shutdown_executive_publisher.put(json.dumps(shutdown_data))
        
        # Step 2: Shutdown sense nodes for each character 
        logger.info("🔌 Step 2: Shutting down sense nodes...")
        self.shutdown_sense_publisher.put(json.dumps(shutdown_data))
        
        # Step 3: Shutdown memory and situation nodes for each character
        logger.info("🔌 Step 3: Shutting down memory and situation nodes...")
        self.shutdown_memory_publisher.put(json.dumps(shutdown_data))
        self.shutdown_situation_publisher.put(json.dumps(shutdown_data))
        
        # Step 4: Shutdown shared nodes (map, llm_service)
        logger.info("🔌 Step 4: Shutting down shared nodes...")
        self.shutdown_shared_publisher.put(json.dumps(shutdown_data))
        
        # Step 5: Shutdown self after brief delay
        logger.info("🔌 Step 5: Shutting down display node...")
        await asyncio.sleep(1.0)
        
        # Trigger our own graceful shutdown
        try:
            self.shutdown()
        finally:
            import sys
            sys.exit(0)

    def _get_html_template(self) -> str:
        """Get the HTML template for the web UI."""
        return """
<!DOCTYPE html>
<html>
<head>
    <title>Zenoh Action Display</title>
    <style>
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
            margin: 0; 
            padding: 0; 
            background-color: #1a1a1a; 
            color: #e0e0e0; 
            height: 100vh;
            overflow: hidden;
        }
        
        /* Main layout with sidebar and content */
        .main-layout {
            display: flex;
            height: 100vh;
        }
        
        /* Character tabs sidebar */
        .character-sidebar {
            width: 500px;
            min-width: 300px;
            max-width: 800px;
            background: #252525;
            border-right: 1px solid #404040;
            display: flex;
            flex-direction: column;
            overflow-y: auto;
            position: relative;
        }
        
        /* Resizer divider */
        .sidebar-resizer {
            position: absolute;
            right: -3px;
            top: 0;
            bottom: 0;
            width: 6px;
            background: transparent;
            cursor: col-resize;
            z-index: 10;
        }
        
        .sidebar-resizer:hover {
            background: rgba(0, 212, 255, 0.3);
        }
        
        .sidebar-resizer.dragging {
            background: rgba(0, 212, 255, 0.5);
        }
        
        .sidebar-header {
            background: #2d2d2d;
            padding: 8px;
            border-bottom: 1px solid #404040;
            font-weight: bold;
            color: #00d4ff;
            text-align: center;
        }
        
        .character-tabs {
            flex: 0 0 auto;
            max-height: 200px;
            overflow-y: auto;
        }
        
        /* Character data area */
        .character-data-area {
            flex: 1;
            padding: 8px;
            border-top: 1px solid #404040;
            overflow-y: auto;
        }
        
        .character-data-header {
            color: #00d4ff;
            font-weight: bold;
            margin-bottom: 10px;
            font-size: 14px;
        }
        
        .character-data-item {
            margin-bottom: 4px;
            padding: 4px 6px;
            background: #1a1a1a;
            border-radius: 4px;
            border-left: 3px solid #404040;
        }
        
        .character-data-label {
            color: #888;
            font-size: 11px;
            font-weight: bold;
            text-transform: uppercase;
            margin-bottom: 2px;
        }
        
        .character-data-value {
            color: #e0e0e0;
            font-size: 12px;
            line-height: 1.3;
        }
        
        /* Character data tabs */
        .character-data-tabs {
            display: flex;
            border-bottom: 1px solid #404040;
            margin-bottom: 10px;
        }
        
        .character-data-tab {
            padding: 6px 10px;
            cursor: pointer;
            border-bottom: 2px solid transparent;
            color: #888;
            font-size: 12px;
            font-weight: bold;
            transition: all 0.2s;
        }
        
        .character-data-tab:hover {
            color: #00d4ff;
        }
        
        .character-data-tab.active {
            color: #00d4ff;
            border-bottom-color: #00d4ff;
        }
        
        /* Character data panels */
        .character-data-content {
            flex: 1;
            overflow-y: auto;
        }
        
        .character-data-panel {
            display: none;
        }
        
        .character-data-panel.active {
            display: block;
        }
        
        .character-tab {
            padding: 8px 10px;
            border-bottom: 1px solid #333;
            cursor: pointer;
            transition: background-color 0.2s;
            display: flex;
            align-items: center;
        }
        
        .character-tab:hover {
            background: #333;
        }
        
        .character-tab.active {
            background: #00d4ff;
            color: #1a1a1a;
            font-weight: bold;
        }
        
        .character-tab-icon {
            margin-right: 8px;
            font-size: 16px;
        }
        
        /* Main content area */
        .main-content {
            flex: 1;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }
        
        /* Character content area (above action log) */
        .character-content {
            background: #2d2d2d;
            border-bottom: 1px solid #404040;
            padding: 10px;
            min-height: 60px;
            max-height: 150px;
            overflow-y: auto;
        }
        
        .character-content h3 {
            margin: 0 0 10px 0;
            color: #00d4ff;
            font-size: 16px;
        }
        
        .character-goal {
            color: #4ecdc4;
            font-style: italic;
            line-height: 1.4;
        }
        
        .no-character-selected {
            color: #888;
            text-align: center;
            padding: 20px;
        }
        
        /* Scrollable content wrapper */
        .scrollable-content {
            flex: 1;
            overflow-y: auto;
            padding: 10px;
        }
        
        .header { 
            background: #2d2d2d; 
            padding: 10px; 
            border-radius: 8px; 
            margin-bottom: 10px; 
            border: 1px solid #404040;
        }
        .header h1 { 
            margin: 0 0 10px 0; 
            color: #00d4ff; 
        }
        .header p { 
            margin: 0; 
            color: #b0b0b0; 
        }
        .action-log { 
            background: #2d2d2d; 
            border: 1px solid #404040; 
            border-radius: 8px; 
            padding: 10px; 
            height: 400px; 
            overflow-y: auto; 
            font-family: 'Consolas', 'Monaco', 'Courier New', monospace; 
            font-size: 13px;
            color: #e0e0e0;
        }
        .input-section { 
            background: #2d2d2d; 
            padding: 10px; 
            border-radius: 8px; 
            margin-top: 10px; 
            border: 1px solid #404040;
        }
        .input-section h3 { 
            margin: 0 0 15px 0; 
            color: #00d4ff; 
        }
        .input-section input { 
            margin: 5px; 
            padding: 8px 12px; 
            border: 1px solid #404040; 
            border-radius: 4px; 
            background: #1a1a1a; 
            color: #e0e0e0; 
            font-size: 14px;
        }
        .input-section input:focus { 
            outline: none; 
            border-color: #00d4ff; 
            box-shadow: 0 0 5px rgba(0, 212, 255, 0.3); 
        }
        .input-section button { 
            margin: 5px; 
            padding: 8px 16px; 
            border: none; 
            border-radius: 4px; 
            background: #00d4ff; 
            color: #1a1a1a; 
            font-weight: bold; 
            cursor: pointer; 
            font-size: 14px;
        }
        .input-section button:hover { 
            background: #00b8e6; 
        }
        .input-section button:disabled { 
            background: #555; 
            color: #888; 
            cursor: not-allowed; 
        }
        .action-entry { 
            margin: 4px 0; 
            padding: 5px; 
            border-left: 3px solid #00d4ff; 
            background: #1a1a1a; 
            border-radius: 4px;
        }
        .character-name { 
            font-weight: bold; 
            color: #00d4ff; 
        }
        .action-type { 
            color: #ff6b6b; 
            font-weight: bold;
        }
        .timestamp { 
            color: #888; 
            font-size: 11px; 
        }
        .input-text { 
            color: #4ecdc4; 
            font-style: italic;
        }
        .response-text { 
            color: #ffe66d; 
        }
        .action-details { 
            color: #4ecdc4; 
            font-weight: bold;
        }
        .additional-fields { 
            color: #a8e6cf; 
            font-size: 11px;
            font-style: italic;
        }
        #sendResult { 
            margin-top: 15px; 
            padding: 10px; 
            border-radius: 4px; 
            font-weight: bold;
        }
        .success { 
            background: #2d5a2d; 
            color: #4ecdc4; 
        }
        .error { 
            background: #5a2d2d; 
            color: #ff6b6b; 
        }
        /* Scrollbar styling */
        .action-log::-webkit-scrollbar, .character-tabs::-webkit-scrollbar, .scrollable-content::-webkit-scrollbar {
            width: 8px;
        }
        .action-log::-webkit-scrollbar-track, .character-tabs::-webkit-scrollbar-track, .scrollable-content::-webkit-scrollbar-track {
            background: #1a1a1a;
        }
        .action-log::-webkit-scrollbar-thumb, .character-tabs::-webkit-scrollbar-thumb, .scrollable-content::-webkit-scrollbar-thumb {
            background: #404040;
            border-radius: 4px;
        }
        .action-log::-webkit-scrollbar-thumb:hover, .character-tabs::-webkit-scrollbar-thumb:hover, .scrollable-content::-webkit-scrollbar-thumb:hover {
            background: #555;
        }
        
        /* Responsive design */
        @media (max-width: 768px) {
            .main-layout {
                flex-direction: column;
            }
            .character-sidebar {
                width: 100%;
                height: 200px;
                border-right: none;
                border-bottom: 1px solid #404040;
            }
            .character-tabs {
                display: flex;
                flex-direction: row;
                overflow-x: auto;
                overflow-y: hidden;
            }
            .character-tab {
                min-width: 120px;
                border-bottom: none;
                border-right: 1px solid #333;
            }
        }
    </style>
</head>
<body>
    <div class="main-layout">
        <!-- Character Tabs Sidebar -->
        <div class="character-sidebar" id="characterSidebar">
            <div class="sidebar-header">
                👥 Characters
            </div>
            <div class="character-tabs" id="characterTabs">
                <!-- Character tabs will be added dynamically -->
            </div>
            
            <!-- Character data area -->
            <div class="character-data-area" id="characterDataArea">
                <div class="character-data-header">Character Data</div>
                
                <!-- Character data tabs -->
                <div class="character-data-tabs" id="characterDataTabs">
                    <div class="character-data-tab active" data-tab="activity">Activity</div>
                    <div class="character-data-tab" data-tab="plan">Plan</div>
                    <div class="character-data-tab" data-tab="view">View</div>
                    <div class="character-data-tab" data-tab="state">State</div>
                    <div class="character-data-tab" data-tab="relations">Relations</div>
                </div>
                
                <!-- Character data content -->
                <div class="character-data-content">
                    <!-- Activity tab content -->
                    <div class="character-data-panel active" id="activityPanel">
                        <div id="activityData" style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                            No activity data available
                        </div>
                    </div>
                    
                    <!-- Plan tab content -->
                    <div class="character-data-panel" id="planPanel">
                        <div id="characterDataItems">
                            <div style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                                Select a character to view data
                            </div>
                        </div>
                    </div>
                    
                    <!-- View tab content -->
                    <div class="character-data-panel" id="viewPanel">
                        <div id="situationData" style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                            No situation data available
                        </div>
                    </div>
                    
                    <!-- State tab content -->
                    <div class="character-data-panel" id="statePanel">
                        <div id="stateData" style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                            No state data available
                        </div>
                    </div>
                    
                    <!-- Relations tab content -->
                    <div class="character-data-panel" id="relationsPanel">
                        <div id="relationsData" style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                            No relations data available
                        </div>
                    </div>
                </div>
            </div>
            
            <!-- Resizer divider -->
            <div class="sidebar-resizer" id="sidebarResizer"></div>
        </div>
        
        <!-- Main Content Area -->
        <div class="main-content">
            <!-- Character-specific content area -->
            <div class="character-content" id="characterContent">
                <div class="no-character-selected">
                    Select a character tab to view their current goal
                </div>
            </div>
            
            <!-- Scrollable main content -->
            <div class="scrollable-content">
                <div class="header">
                    <h1>🖥️ Zenoh Action Display</h1>
                    <p>Real-time action monitoring and text input</p>
                </div>
                
                <div class="action-log" id="actionLog">
                    <div style="color: #888; font-style: italic;">Waiting for actions...</div>
                </div>
                
                <div class="input-section">
                    <h3>Turn Control</h3>
                    <div style="margin-bottom: 15px;">
                        <button id="stepButton" onclick="stepTurn()" style="background: #555; margin-right: 10px;" disabled>Step Turn</button>
                        <button id="runButton" onclick="runTurns()" style="background: #555; color: #888; margin-right: 10px;" disabled>Run</button>
                        <button onclick="stopTurns()" style="background: #ff6b6b; margin-right: 10px;">Stop</button>
                        <button onclick="saveAll()" style="background: #95e1d3; color: #1a1a1a; margin-right: 10px;">Save</button>
                        <button onclick="exportToObsidian()" style="background: #7c3aed; color: white; margin-right: 10px;">Obsidian</button>
                        <button onclick="showShutdownDialog()" style="background: #ff4757; color: white;">Shutdown</button>
                    </div>
                    <div style="margin-bottom: 15px; padding: 10px; background: #333; border-radius: 5px;">
                        <label for="timeSlider" style="display: block; margin-bottom: 5px; font-size: 14px; color: #ccc;">
                            Turn Delay: <span id="timeSliderValue">0</span> minutes
                        </label>
                        <input type="range" id="timeSlider" min="0" max="30" value="0" style="width: 100%; margin-bottom: 10px;">
                        <div style="font-size: 12px; color: #888;">
                            <span>Simulation Time: </span><span id="simulationTime">Loading...</span>
                        </div>
                    </div>
                    <div id="turnStatus" style="margin-bottom: 10px; font-size: 12px; color: #888;">
                        <span id="turnMode">Mode: Step</span> | 
                        <span id="turnNumber">Turn: 0</span> | 
                        <span id="turnProgress">Progress: 0/0</span>
                    </div>
                    <div id="systemStatus" style="margin-bottom: 10px; font-size: 12px; color: #888;">
                        <span id="systemStatusText">Starting...</span>
                    </div>
                    <div id="turnResult" style="margin-top: 10px;"></div>
                </div>
                
                <div class="input-section">
                    <h3>Send Text Input</h3>
                    <input type="text" id="characterInput" placeholder="Character name (optional)" style="width: 150px;">
                    <textarea id="messageInput" placeholder="Message or Plan (multi-line supported)" style="width: 450px; height: 80px; resize: vertical; background-color: #2b2b2b; color: #ffffff; border: 1px solid #555; padding: 8px; font-family: monospace;"></textarea>
                    <button onclick="sendText()">Send</button>
                    <div id="sendResult"></div>
                </div>
            </div>
        </div>
    </div>

    <script>
        let ws = null;
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 5;
        const reconnectDelay = 2000;
        
        // Character tabs state
        let characterTabs = new Map(); // character_name -> {element, goal, decidedAction}
        // Track which characters have announced themselves
        let announcedCharacters = new Set();
        // Remember last ready state payload for gating
        let lastReadyData = null;
        let activeCharacter = null;
        let commandInProgress = false; // Prevent rapid button clicks
        
        // Sidebar resizer state
        let isResizing = false;
        let startX = 0;
        let startWidth = 0;
        
        function connectWebSocket() {
            const port = window.location.port;
            ws = new WebSocket(`ws://localhost:${port}/ws`);
            const actionLog = document.getElementById('actionLog');
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                reconnectAttempts = 0;
                
                // Send a ping message to keep connection alive
                setInterval(function() {
                    if (ws && ws.readyState === WebSocket.OPEN) {
                        ws.send('ping');
                    }
                }, 30000); // Send ping every 30 seconds
                
                // Request initial system state
                // Note: The server will send ready_state when available
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                
                // Attempt to reconnect
                if (reconnectAttempts < maxReconnectAttempts) {
                    reconnectAttempts++;
                    console.log(`Attempting to reconnect (${reconnectAttempts}/${maxReconnectAttempts})...`);
                    setTimeout(connectWebSocket, reconnectDelay);
                } else {
                    console.error('Max reconnection attempts reached');
                    const entry = document.createElement('div');
                    entry.className = 'action-entry';
                    entry.innerHTML = `<span class="timestamp">[${new Date().toLocaleTimeString()}]</span> <span style="color: #ff6b6b;">X WebSocket connection lost - please refresh page</span>`;
                    actionLog.appendChild(entry);
                    actionLog.scrollTop = actionLog.scrollHeight;
                }
            };
            
            ws.onmessage = function(event) {
                console.log('WebSocket message received:', event.data);
                
                // Handle ping/pong messages
                if (event.data === 'pong') {
                    console.log('Received pong from server');
                    return;
                }
                
                // Parse JSON messages
                let data;
                try {
                    data = JSON.parse(event.data);
                } catch (e) {
                    console.log('Non-JSON message received:', event.data);
                    return;
                }
                if (data.type === 'action') {
                    addActionEntry(data);
                    // Check if this is an announcement to create a character tab
                    if (data.action_type === 'announcement') {
                        createCharacterTab(data.character);
                        try { announcedCharacters.add(data.character); } catch (e) {}
                        // If system is ready and all expected characters have announced, select first and enable controls
                        try {
                            if (lastReadyData && lastReadyData.system_ready) {
                                const expected = Array.isArray(lastReadyData.characters_active) ? lastReadyData.characters_active.length : (lastReadyData.character_count || 0);
                                if (expected > 0 && announcedCharacters.size >= expected) {
                                    if (!activeCharacter) {
                                        const first = lastReadyData.characters_active && lastReadyData.characters_active[0] ? lastReadyData.characters_active[0] : data.character;
                                        selectCharacterTab(first);
                                    }
                                    // Enable controls if not in run mode
                                    const stepButton = document.getElementById('stepButton');
                                    const runButton = document.querySelector('button[onclick="runTurns()"]');
                                    if (currentTurnMode !== 'run') {
                                        stepButton.disabled = false; stepButton.style.background = '#4ecdc4'; stepButton.title = 'Click to advance to next turn';
                                        runButton.disabled = false; runButton.style.background = '#ffe66d'; runButton.style.color = '#1a1a1a'; runButton.title = 'Click to run multiple turns';
                                    }
                                }
                            }
                        } catch (e) {}
                    }
                } else if (data.type === 'goal') {
                    handleGoalUpdate(data);
                } else if (data.type === 'decided_action') {
                    handleDecidedActionUpdate(data);
                } else if (data.type === 'current_plan') {
                    handleCurrentPlanUpdate(data);
                } else if (data.type === 'current_activity') {
                    handleCurrentActivityUpdate(data);
                } else if (data.type === 'current_state') {
                    handleCurrentStateUpdate(data);
                } else if (data.type === 'situation_data') {
                    handleSituationDataUpdate(data);
                } else if (data.type === 'turn_mode_update') {
                    updateTurnMode(data);
                } else if (data.type === 'turn_start') {
                    handleTurnStart(data);
                } else if (data.type === 'turn_state') {
                    updateTurnState(data);
                } else if (data.type === 'ready_state') {
                    handleReadyState(data);
                } else if (data.type === 'time_update') {
                    handleTimeUpdate(data);
                } else if (data.type === 'turn_state_update') {
                    // Handle unified turn state update (replaces step_complete)
                    handleTurnStateUpdate(data);
                } else if (data.type === 'test') {
                    console.log('Test message received:', data.message);
                    // Add a test entry to the action log
                    const entry = document.createElement('div');
                    entry.className = 'action-entry';
                    entry.innerHTML = `<span class="timestamp">[${new Date().toLocaleTimeString()}]</span> <span style="color: #4ecdc4;">* ${data.message}</span>`;
                    actionLog.appendChild(entry);
                    actionLog.scrollTop = actionLog.scrollHeight;
                }
            };
        }
        
        // Sidebar Resizer Functions
        function initSidebarResizer() {
            const resizer = document.getElementById('sidebarResizer');
            const sidebar = document.getElementById('characterSidebar');
            
            // Load saved width from localStorage
            const savedWidth = localStorage.getItem('sidebarWidth');
            if (savedWidth) {
                sidebar.style.width = savedWidth + 'px';
            }
            
            resizer.addEventListener('mousedown', startResize);
            document.addEventListener('mousemove', resize);
            document.addEventListener('mouseup', stopResize);
        }
        
        function startResize(e) {
            isResizing = true;
            startX = e.clientX;
            startWidth = parseInt(document.getElementById('characterSidebar').offsetWidth);
            document.getElementById('sidebarResizer').classList.add('dragging');
            e.preventDefault();
        }
        
        function resize(e) {
            if (!isResizing) return;
            
            const sidebar = document.getElementById('characterSidebar');
            const newWidth = startWidth + (e.clientX - startX);
            const minWidth = 300;
            const maxWidth = 800;
            
            if (newWidth >= minWidth && newWidth <= maxWidth) {
                sidebar.style.width = newWidth + 'px';
            }
        }
        
        function stopResize() {
            if (isResizing) {
                isResizing = false;
                document.getElementById('sidebarResizer').classList.remove('dragging');
                
                // Save width to localStorage
                const sidebar = document.getElementById('characterSidebar');
                localStorage.setItem('sidebarWidth', sidebar.offsetWidth);
            }
        }
        
        // Connect WebSocket on page load
        connectWebSocket();
        
        // Initialize sidebar resizer when DOM is loaded
        document.addEventListener('DOMContentLoaded', function() {
            initSidebarResizer();
            initCharacterDataTabs();
            
            // Initialize buttons as disabled until system is ready
            const stepButton = document.getElementById('stepButton');
            const runButton = document.getElementById('runButton');
            
            stepButton.disabled = true;
            stepButton.style.background = '#555';
            stepButton.title = 'Waiting for system startup...';
            
            runButton.disabled = true;
            runButton.style.background = '#555';
            runButton.style.color = '#888';
            runButton.title = 'Waiting for system startup...';
        });
        
        // Character Data Tab Functions
        function initCharacterDataTabs() {
            const tabs = document.querySelectorAll('.character-data-tab');
            tabs.forEach(tab => {
                tab.addEventListener('click', function() {
                    const tabName = this.getAttribute('data-tab');
                    switchCharacterDataTab(tabName);
                });
            });
        }
        
        function switchCharacterDataTab(tabName) {
            // Update tab active states
            const tabs = document.querySelectorAll('.character-data-tab');
            tabs.forEach(tab => {
                tab.classList.remove('active');
                if (tab.getAttribute('data-tab') === tabName) {
                    tab.classList.add('active');
                }
            });
            
            // Update panel visibility
            const panels = document.querySelectorAll('.character-data-panel');
            panels.forEach(panel => {
                panel.classList.remove('active');
            });
            
            if (tabName === 'activity') {
                document.getElementById('activityPanel').classList.add('active');
                // Refresh activity display for current character
                if (activeCharacter) {
                    updateActivityDataDisplay(activeCharacter);
                }
            } else if (tabName === 'plan') {
                document.getElementById('planPanel').classList.add('active');
                // Refresh plan display for current character
                if (activeCharacter) {
                    updateCharacterDataDisplay(activeCharacter);
                }
            } else if (tabName === 'view') {
                document.getElementById('viewPanel').classList.add('active');
                // Refresh situation display for current character
                if (activeCharacter) {
                    updateSituationDataDisplay(activeCharacter);
                }
            } else if (tabName === 'state') {
                document.getElementById('statePanel').classList.add('active');
                if (activeCharacter) {
                    updateStateDataDisplay(activeCharacter);
                }
            } else if (tabName === 'relations') {
                document.getElementById('relationsPanel').classList.add('active');
                if (activeCharacter) {
                    updateRelationsDataDisplay(activeCharacter);
                }
            }
        }
        
        // Character Tab Management Functions
        function createCharacterTab(characterName) {
            // Avoid duplicate tabs
            if (characterTabs.has(characterName)) {
                return;
            }
            
            const tabsContainer = document.getElementById('characterTabs');
            const tabElement = document.createElement('div');
            tabElement.className = 'character-tab';
            tabElement.innerHTML = `
                <span class="character-tab-icon">👤</span>
                <span>${characterName}</span>
            `;
            
            // Add click handler for tab selection
            tabElement.addEventListener('click', () => selectCharacterTab(characterName));
            
            tabsContainer.appendChild(tabElement);
            characterTabs.set(characterName, {
                element: tabElement,
                goal: null,
                decidedAction: null,
                currentPlan: null,
                currentActivity: null,
                situationData: null
            });
            
            console.log(`Created character tab for: ${characterName}`);
        }
        
        function selectCharacterTab(characterName) {
            // Update active character
            activeCharacter = characterName;
            
            // Update visual state of tabs
            characterTabs.forEach((tabData, name) => {
                if (name === characterName) {
                    tabData.element.classList.add('active');
                } else {
                    tabData.element.classList.remove('active');
                }
            });
            
            // Update character content area
            updateCharacterContent(characterName);
            
            // Update character data display based on active tab
            updateActivityDataDisplay(characterName);
            updateCharacterDataDisplay(characterName);
            updateSituationDataDisplay(characterName);
            // Also refresh state panel if it is the active tab
            try {
                const statePanel = document.getElementById('statePanel');
                if (statePanel && statePanel.classList.contains('active')) {
                    updateStateDataDisplay(characterName);
                }
            } catch (e) { /* no-op */ }
            
            console.log(`Selected character tab: ${characterName}`);
        }
        
        function updateCharacterContent(characterName) {
            const contentDiv = document.getElementById('characterContent');
            const tabData = characterTabs.get(characterName);
            
            if (!tabData) {
                contentDiv.innerHTML = '<div class="no-character-selected">Character not found</div>';
                return;
            }
            
            let content = `<h3>👤 ${characterName}</h3>`;
            
            if (tabData.goal) {
                content += `<div class="character-goal">Goal: ${tabData.goal}</div>`;
            } else {
                content += `<div style="color: #888; font-style: italic;">No current goal</div>`;
            }
            
            contentDiv.innerHTML = content;
        }
        
        function handleGoalUpdate(goalData) {
            const characterName = goalData.character;
            const goal = goalData.goal;
            
            console.log(`Goal update for ${characterName}: ${goal}`);
            
            // Update the stored goal for this character
            if (characterTabs.has(characterName)) {
                const tabData = characterTabs.get(characterName);
                tabData.goal = goal;
                
                // If this character's tab is currently active, update the display
                if (activeCharacter === characterName) {
                    updateCharacterContent(characterName);
                }
            } else {
                // Character tab doesn't exist yet, this shouldn't happen
                // but we can create it if needed
                console.warn(`Received goal for unknown character: ${characterName}`);
            }
        }
        
        function handleDecidedActionUpdate(decidedActionData) {
            const characterName = decidedActionData.character;
            const decidedAction = decidedActionData.decided_action;
            
            console.log(`Decided action update for ${characterName}: ${decidedAction}`);
            
            // Update the stored decided action for this character
            if (characterTabs.has(characterName)) {
                const tabData = characterTabs.get(characterName);
                tabData.decidedAction = decidedAction;
                
                // If this character's tab is currently active, update the display
                if (activeCharacter === characterName) {
                    updateCharacterDataDisplay(characterName);
                }
            } else {
                // Character tab doesn't exist yet, this shouldn't happen
                console.warn(`Received decided action for unknown character: ${characterName}`);
            }
        }
        
        function handleCurrentPlanUpdate(currentPlanData) {
            const characterName = currentPlanData.character;
            const currentPlan = currentPlanData.current_plan;
            
            console.log(`Current plan update for ${characterName}: ${currentPlan ? 'Plan updated' : 'Plan cleared'}`);
            
            // Update the stored current plan for this character
            if (characterTabs.has(characterName)) {
                const tabData = characterTabs.get(characterName);
                tabData.currentPlan = currentPlan;
                
                // If this character's tab is currently active, update the display
                if (activeCharacter === characterName) {
                    updateCharacterDataDisplay(characterName);
                }
            } else {
                // Character tab doesn't exist yet, this shouldn't happen
                console.warn(`Received current plan for unknown character: ${characterName}`);
            }
        }
        
        function handleCurrentActivityUpdate(currentActivityData) {
            const characterName = currentActivityData.character;
            const currentActivity = currentActivityData.current_activity;
            
            console.log(`Current activity update for ${characterName}: ${currentActivity ? 'Activity updated' : 'Activity cleared'}`);
            
            // Update the stored current activity for this character
            if (characterTabs.has(characterName)) {
                const tabData = characterTabs.get(characterName);
                tabData.currentActivity = currentActivityData;
                
                // If this character's tab is currently active and activity tab is selected, update the display
                if (activeCharacter === characterName) {
                    updateActivityDataDisplay(characterName);
                }
            } else {
                // Character tab doesn't exist yet, this shouldn't happen
                console.warn(`Received current activity for unknown character: ${characterName}`);
            }
        }

        function handleCurrentStateUpdate(currentStateData) {
            const characterName = currentStateData.character;
            console.log(`Current state update for ${characterName}`);
            if (characterTabs.has(characterName)) {
                const tabData = characterTabs.get(characterName);
                tabData.currentState = currentStateData;
                if (activeCharacter === characterName) {
                    updateStateDataDisplay(characterName);
                }
            } else {
                console.warn(`Received current state for unknown character: ${characterName}`);
                createCharacterTab(characterName);
                const tabData = characterTabs.get(characterName);
                tabData.currentState = currentStateData;
            }
        }
        
        function handleSituationDataUpdate(situationData) {
            const characterName = situationData.character;
            const situation = situationData.situation_data;
            
            console.log(`Situation data update for ${characterName}`);
            
            // Update the stored situation data for this character
            if (characterTabs.has(characterName)) {
                const tabData = characterTabs.get(characterName);
                tabData.situationData = situation;
                
                // If this character's tab is currently active and view tab is selected, update the display
                if (activeCharacter === characterName) {
                    updateSituationDataDisplay(characterName);
                }
            } else {
                // Character tab doesn't exist yet, this shouldn't happen
                console.warn(`Received situation data for unknown character: ${characterName}`);
            }
        }
        
        // Cache of latest simulation time (ISO string)
        let latestSimTimeISO = null;

        function handleTimeUpdate(timeData) {
            const timeInfo = timeData.time_info;
            if (timeInfo && timeInfo.datetime) {
                const dateTime = new Date(timeInfo.datetime);
                // Cache latest simulation time for action log timestamps
                latestSimTimeISO = timeInfo.datetime;
                const displayTime = dateTime.toLocaleString('en-US', {
                    weekday: 'short',
                    year: 'numeric',
                    month: 'short',
                    day: 'numeric',
                    hour: '2-digit',
                    minute: '2-digit'
                });
                document.getElementById('simulationTime').textContent = displayTime;
                console.log('⏰ Time updated via WebSocket:', displayTime);
            } else {
                console.warn('Received time update but no valid datetime in data');
            }
        }
        
        function updateCharacterDataDisplay(characterName) {
            const dataItemsDiv = document.getElementById('characterDataItems');
            const tabData = characterTabs.get(characterName);
            
            if (!tabData) {
                dataItemsDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">Character not found</div>';
                return;
            }
            
            let content = '';
            
            // Add current plan if available
            if (tabData.currentPlan) {
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Current Plan</div>
                        <div class="character-data-value"><pre style="white-space: pre-wrap; font-family: 'Courier New', monospace; font-size: 12px; margin: 0;">${tabData.currentPlan}</pre></div>
                    </div>
                `;
            }
            
            // Add horizontal line between plan and action if both exist
            if (tabData.currentPlan && tabData.decidedAction) {
                content += '<hr style="border: none; border-top: 1px solid #404040; margin: 8px 0;">';
            }
            
            // Add decided action if available
            if (tabData.decidedAction) {
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Next Action</div>
                        <div class="character-data-value">${tabData.decidedAction}</div>
                    </div>
                `;
            }
            
            // If no data items, show placeholder
            if (content === '') {
                content = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No character data available</div>';
            }
            
            dataItemsDiv.innerHTML = content;
        }
        
        function updateActivityDataDisplay(characterName) {
            const activityDataDiv = document.getElementById('activityData');
            const tabData = characterTabs.get(characterName);
            
            if (!tabData || !tabData.currentActivity) {
                activityDataDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No activity data available</div>';
                return;
            }
            
            const activityData = tabData.currentActivity;
            let content = '';
            
            // Add activity name if available
            if (activityData.activity_data && activityData.activity_data.name) {
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Activity Name</div>
                        <div class="character-data-value">${activityData.activity_data.name}</div>
                    </div>
                `;
            }
            
            // Add activity duration if available
            if (activityData.activity_data && activityData.activity_data.duration) {
                const duration = activityData.activity_data.duration;
                let durationText = '';
                if (Array.isArray(duration)) {
                    if (duration.length === 2) {
                        durationText = `${duration[0]}-${duration[1]} minutes`;
                    } else if (duration.length === 1) {
                        durationText = `${duration[0]} minutes`;
                    }
                } else {
                    durationText = `${duration} minutes`;
                }
                
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Duration</div>
                        <div class="character-data-value">${durationText}</div>
                    </div>
                `;
            }
            
            // Add activity category if available
            if (activityData.activity_data && activityData.activity_data.category) {
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Category</div>
                        <div class="character-data-value">${activityData.activity_data.category}</div>
                    </div>
                `;
            }
            
            // Add activity tags if available
            if (activityData.activity_data && activityData.activity_data.tags && activityData.activity_data.tags.length > 0) {
                const tagsText = activityData.activity_data.tags.join(', ');
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Tags</div>
                        <div class="character-data-value">${tagsText}</div>
                    </div>
                `;
            }
            
            // Add current step if available
            if (activityData.current_step && activityData.current_step.name) {
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Current Step</div>
                        <div class="character-data-value">${activityData.current_step.name}</div>
                    </div>
                `;
            }
            
            // Add activity steps if available
            if (activityData.activity_data && activityData.activity_data.steps) {
                const steps = activityData.activity_data.steps;
                const currentStepIndex = activityData.activity_state ? activityData.activity_state.current_step_index || 0 : 0;
                
                let stepsContent = '';
                steps.forEach((step, index) => {
                    const isCurrentStep = index === currentStepIndex;
                    const stepStyle = isCurrentStep ? 
                        'background: #2a2a2a; color: #00d4ff; padding: 2px 4px; border-radius: 3px;' : 
                        'color: #ccc;';
                    const prefix = isCurrentStep ? '▶ ' : '  ';
                    stepsContent += `${prefix}<span style="${stepStyle}">${index + 1}. ${step}</span>\n`;
                });
                
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Activity Steps</div>
                        <div class="character-data-value"><pre style="white-space: pre-wrap; font-family: 'Courier New', monospace; font-size: 12px; margin: 0; line-height: 1.4;">${stepsContent}</pre></div>
                    </div>
                `;
            }
            
            // Add activity state if available
            if (activityData.activity_state) {
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Activity State</div>
                        <div class="character-data-value"><pre style="white-space: pre-wrap; font-family: 'Courier New', monospace; font-size: 12px; margin: 0;">${JSON.stringify(activityData.activity_state, null, 2)}</pre></div>
                    </div>
                `;
            }
            
            // If no content, show raw activity data
            if (content === '' && activityData.current_activity) {
                content = `
                    <div class="character-data-item">
                        <div class="character-data-label">Raw Activity Data</div>
                        <div class="character-data-value"><pre style="white-space: pre-wrap; font-family: 'Courier New', monospace; font-size: 12px; margin: 0;">${activityData.current_activity}</pre></div>
                    </div>
                `;
            }
            
            // Final fallback
            if (content === '') {
                content = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No activity data available</div>';
            }
            
            activityDataDiv.innerHTML = content;
        }

        function updateStateDataDisplay(characterName) {
            const stateDiv = document.getElementById('stateData');
            if (!stateDiv) return;
            if (!characterTabs.has(characterName)) {
                stateDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">Character not found</div>';
                return;
            }
            const tabData = characterTabs.get(characterName);
            const state = tabData.currentState ? tabData.currentState.state || {} : {};
            if (!state || Object.keys(state).length === 0) {
                stateDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No state data available</div>';
                return;
            }
            let html = '';
            Object.keys(state).forEach(key => {
                const v = state[key] && typeof state[key].value !== 'undefined' ? state[key].value : '';
                html += `<div class="character-data-item"><div class="character-data-label">${key.charAt(0).toUpperCase() + key.slice(1)}</div><div class="character-data-value">${Math.round(Number(v) || 0)}/100</div></div>`;
            });
            stateDiv.innerHTML = html;
        }
        
        function updateSituationDataDisplay(characterName) {
            const situationDataDiv = document.getElementById('situationData');
            const tabData = characterTabs.get(characterName);
            
            if (!tabData || !tabData.situationData) {
                situationDataDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No situation data available</div>';
                return;
            }
            
            // Format the situation data as JSON
            const formattedData = JSON.stringify(tabData.situationData, null, 2);
            situationDataDiv.innerHTML = `<pre style="white-space: pre-wrap; font-family: 'Courier New', monospace; font-size: 11px; margin: 0; color: #e0e0e0; text-align: left;">${formattedData}</pre>`;
        }
        
        function updateRelationsDataDisplay(characterName) {
            const relationsDataDiv = document.getElementById('relationsData');
            
            if (!relationsDataDiv) return;
            
            // Get all other characters
            const otherCharacters = Array.from(characterTabs.keys()).filter(c => c !== characterName);
            
            if (otherCharacters.length === 0) {
                relationsDataDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No other characters available</div>';
                return;
            }
            
            // Build accordion HTML
            let html = '<div style="padding: 10px;">';
            otherCharacters.forEach(otherChar => {
                html += `
                    <div class="relation-accordion-item" style="margin-bottom: 5px; border: 1px solid #444; border-radius: 4px; overflow: hidden;">
                        <div class="relation-accordion-header" 
                             onclick="toggleRelationAccordion('${characterName}', '${otherChar}')"
                             style="padding: 10px; background: #333; cursor: pointer; user-select: none; display: flex; justify-content: space-between; align-items: center;">
                            <span style="font-weight: bold;">${otherChar}</span>
                            <span id="accordion-arrow-${otherChar}" style="transition: transform 0.3s;">▶</span>
                        </div>
                        <div id="relation-content-${otherChar}" 
                             style="display: none; padding: 15px; background: #2a2a2a; max-height: 400px; overflow-y: auto;">
                            <div style="color: #888; font-style: italic;">Click to load relation data...</div>
                        </div>
                    </div>
                `;
            });
            html += '</div>';
            relationsDataDiv.innerHTML = html;
        }
        
        async function toggleRelationAccordion(characterName, targetCharacter) {
            const contentDiv = document.getElementById(`relation-content-${targetCharacter}`);
            const arrowSpan = document.getElementById(`accordion-arrow-${targetCharacter}`);
            
            if (!contentDiv) return;
            
            // Toggle visibility
            if (contentDiv.style.display === 'none') {
                // Opening - fetch data
                contentDiv.style.display = 'block';
                arrowSpan.style.transform = 'rotate(90deg)';
                
                // Show loading message
                contentDiv.innerHTML = '<div style="color: #888; font-style: italic;">Loading...</div>';
                
                // Fetch relation data
                try {
                    const response = await fetch(`/api/relation/${characterName}/${targetCharacter}`);
                    const data = await response.json();
                    
                    if (data.success) {
                        // Check if no interaction has occurred yet
                        if (data.no_interaction) {
                            contentDiv.innerHTML = `
                                <div style="text-align: center; padding: 20px; color: #888;">
                                    <div style="font-size: 18px; margin-bottom: 10px;">👥</div>
                                    <div style="font-style: italic;">No interaction history yet</div>
                                    <div style="font-size: 11px; margin-top: 8px; color: #666;">
                                        ${characterName} and ${targetCharacter} haven't interacted
                                    </div>
                                </div>
                            `;
                        } else {
                            let html = '';
                            
                            // Display discourse_state
                            html += '<div style="margin-bottom: 15px;">';
                            html += '<div style="font-weight: bold; color: #95e1d3; margin-bottom: 5px;">Discourse State:</div>';
                            html += '<div style="padding: 10px; background: #1a1a1a; border-radius: 4px; white-space: pre-wrap; font-family: monospace; font-size: 12px;">';
                            html += data.discourse_state || '<span style="color: #888; font-style: italic;">No discourse state</span>';
                            html += '</div></div>';
                            
                            // Display tom_model
                            html += '<div>';
                            html += '<div style="font-weight: bold; color: #95e1d3; margin-bottom: 5px;">Theory of Mind:</div>';
                            html += '<div style="padding: 10px; background: #1a1a1a; border-radius: 4px; white-space: pre-wrap; font-family: monospace; font-size: 12px;">';
                            html += data.tom_model || '<span style="color: #888; font-style: italic;">No ToM model</span>';
                            html += '</div></div>';
                            
                            contentDiv.innerHTML = html;
                        }
                    } else {
                        contentDiv.innerHTML = `<div style="color: #ff6b6b;">Error: ${data.message || 'Failed to load data'}</div>`;
                    }
                } catch (error) {
                    contentDiv.innerHTML = `<div style="color: #ff6b6b;">Error: ${error.message}</div>`;
                }
            } else {
                // Closing
                contentDiv.style.display = 'none';
                arrowSpan.style.transform = 'rotate(0deg)';
            }
        }
        
        function addActionEntry(actionData) {
            console.log('Adding action entry:', actionData);
            const entry = document.createElement('div');
            entry.className = 'action-entry';
            
            // Prefer simulation time (latest from time_update); fallback to local clock
            const timestamp = latestSimTimeISO ? new Date(latestSimTimeISO).toLocaleTimeString() : new Date().toLocaleTimeString();
            let html = `<span class="timestamp">[${timestamp}]</span> `;
            html += `<span class="character-name">[${actionData.character.toUpperCase()}]</span> `;
            const typeLower = (actionData.action_type || '').toLowerCase();
            let actorLabel = '';
            if (actionData.is_text_only) {
                if (typeLower === 'say' && actionData.target) {
                    actorLabel = ` ${actionData.target}:`;
                } else if (typeLower === 'response' && actionData.source) {
                    actorLabel = ` ${actionData.source}:`;
                }
            } else {
                if (typeLower === 'take' && actionData.target) {
                    actorLabel = ` ${actionData.target}`;
                } else if (typeLower === 'scan' && actionData.target) {
                    actorLabel = ` ${actionData.target}`;
                }
            }
            html += `<span class="action-type">${actionData.action_type}</span>${actorLabel}`;
            
            // For text-only actions (Say/response), only show the text
            if (actionData.is_text_only) {
                const textContent = actionData.text || actionData.input_text || actionData.llm_response || '';
                if (textContent) {
                    html += `<br><span class="response-text">"${textContent}"</span>`;
                }
            } else {
                // Display action details if available
                if (actionData.action || actionData.target || actionData.value || actionData.requested_target || actionData.error || actionData.status) {
                    let actionDetails = [];
                    if (actionData.action) actionDetails.push(`Action: ${actionData.action}`);
                    // Prefer resolved target if present, else show requested target, else target
                    const targetLabel = actionData.resolved_target || actionData.target || actionData.requested_target;
                    if (targetLabel) actionDetails.push(`Target: ${targetLabel}`);
                    if (actionData.requested_target && (!actionData.resolved_target && !actionData.target)) {
                        actionDetails.push(`Requested: ${actionData.requested_target}`);
                    }
                    if (actionData.value) actionDetails.push(`Value: ${actionData.value}`);
                    // Add scan-specific details
                    if (typeLower === 'scan') {
                        if (actionData.out) actionDetails.push(`Variable: ${actionData.out}`);
                        if (actionData.result) actionDetails.push(`Found: ${actionData.result}`);
                        if (actionData.variable_bound && actionData.bound_value) {
                            actionDetails.push(`Bound: ${actionData.variable_bound} = ${actionData.bound_value}`);
                        }
                    }
                    // Append status/error inline without adding vertical height
                    if ((actionData.status && actionData.status.toLowerCase() === 'failed') || actionData.error) {
                        const status = actionData.status ? actionData.status.toUpperCase() : 'FAILED';
                        const errorMsg = actionData.error ? ` - ${actionData.error}` : '';
                        actionDetails.push(`${status}${errorMsg}`);
                    }
                    html += `<br><span class="action-details">${actionDetails.join(' | ')}</span>`;
                }
                
                // Display input text if available
                if (actionData.input_text) {
                    html += `<br><span class="input-text">Input: "${actionData.input_text}"</span>`;
                }
                
                // Display LLM response if available
                if (actionData.llm_response) {
                    html += `<br><span class="response-text">Response: ${actionData.llm_response}</span>`;
                }
                
                // Display any additional fields from raw_data for future flexibility
                if (actionData.raw_data) {
                    const additionalFields = [];
                    for (const [key, value] of Object.entries(actionData.raw_data)) {
                        // Skip fields we've already displayed
                        if (!['action', 'target', 'value', 'input_text', 'llm_response', 'action_type', 'action_id', 'timestamp', 'confidence'].includes(key)) {
                            if (value && value !== '') {
                                additionalFields.push(`${key}: ${value}`);
                            }
                        }
                    }
                    if (additionalFields.length > 0) {
                        html += `<br><span class="additional-fields">${additionalFields.join(' | ')}</span>`;
                    }
                }
            }
            
            entry.innerHTML = html;
            const actionLog = document.getElementById('actionLog');
            actionLog.appendChild(entry);
            actionLog.scrollTop = actionLog.scrollHeight;
        }
        
        // REMOVED: isTurnInProgress() - button states now come from backend via turn_state_update
        
        function updateTurnMode(turnModeData) {
            // DEPRECATED: State management now handled by handleTurnStateUpdate()
            // Clear command lock - backend has confirmed the state change
            commandInProgress = false;
            console.log(`updateTurnMode() called with mode=${turnModeData.mode} - button states handled by turn_state_update`);
        }
        
        function updateTurnState(turnData) {
            // DEPRECATED: Display updates now handled by handleTurnStateUpdate()
            // Kept for backward compatibility during transition
            const turnMode = document.getElementById('turnMode');
            const turnNumber = document.getElementById('turnNumber');
            const turnProgress = document.getElementById('turnProgress');
            
            // Update display only (if turn_state_update hasn't already done it)
            if (turnMode) turnMode.textContent = `Mode: ${turnData.mode.charAt(0).toUpperCase() + turnData.mode.slice(1)}`;
            if (turnNumber) turnNumber.textContent = `Turn: ${turnData.turn_number}`;
            if (turnProgress) turnProgress.textContent = `Progress: ${turnData.completed_characters.length}/${turnData.active_characters.length}`;
        }
        
        function handleTurnStateUpdate(stateData) {
            /**
             * NEW: Handle unified turn state update.
             * 
             * This receives all state in one message:
             * - Button states (computed by Python)
             * - Turn information
             * - Progress tracking
             * 
             * Just apply what the backend tells us - no complex logic needed!
             */
            console.log(`🆕 Turn state update: turn=${stateData.turn.number}, mode=${stateData.turn.mode}, ` +
                       `step_enabled=${stateData.buttons.step.enabled}, ` +
                       `progress=${stateData.turn.completed_count}/${stateData.turn.active_count}`);
            
            // Clear command lock when backend sends state update
            commandInProgress = false;
            
            // Apply button states directly from backend computation
            const stepButton = document.getElementById('stepButton');
            const runButton = document.getElementById('runButton');
            const stopButton = document.getElementById('stopButton');
            
            if (stepButton) {
                const shouldBeEnabled = stateData.buttons.step.enabled;
                stepButton.disabled = !shouldBeEnabled;
                stepButton.style.background = shouldBeEnabled ? '#4ecdc4' : '#555';
                stepButton.title = stateData.buttons.step.tooltip;
                console.log(`🔘 Step button: enabled=${shouldBeEnabled}, disabled=${stepButton.disabled}, background=${stepButton.style.background}, commandInProgress=${commandInProgress}`);
            }
            
            if (runButton) {
                runButton.disabled = !stateData.buttons.run.enabled;
                runButton.style.background = stateData.buttons.run.enabled ? '#ffe66d' : '#555';
                runButton.style.color = stateData.buttons.run.enabled ? '#1a1a1a' : '#888';
                runButton.title = stateData.buttons.run.tooltip;
            }
            
            if (stopButton) {
                stopButton.disabled = !stateData.buttons.stop.enabled;
                stopButton.title = stateData.buttons.stop.tooltip;
            }
            
            // Update turn progress tracking
            turnActiveCharacters = stateData.turn.active_count;
            turnCompletedCharacters = stateData.turn.completed_count;
            
            // Update mode tracking
            currentTurnMode = stateData.turn.mode;
            
            console.log(`✅ Applied button states: step=${stateData.buttons.step.enabled}, ` +
                       `run=${stateData.buttons.run.enabled}, stop=${stateData.buttons.stop.enabled}`);
        }
        
        function handleTurnStart(turnData) {
            // DEPRECATED: Button state management now handled by handleTurnStateUpdate()
            console.log(`Turn start: turn=${turnData.turn_number}, active=${turnData.active_characters.length}`);
        }
        
        function handleReadyState(readyData) {
            /**
             * Handle system ready signal from launcher.
             * 
             * NOTE: Button states are now managed entirely by the backend via turn_state_update.
             * This handler ONLY updates the system status text.
             */
            const systemStatusText = document.getElementById('systemStatusText');
            
            // Remember last ready payload for character tab creation
            try { lastReadyData = readyData; } catch (e) {}
            
            if (readyData.system_ready) {
                // Update status text only
                systemStatusText.textContent = `Ready - ${readyData.character_count} characters launching`;
                systemStatusText.style.color = '#4ecdc4'; // Green color for ready

                // Auto-select first character if none selected yet
                try {
                    const expected = Array.isArray(readyData.characters_active) ? readyData.characters_active.length : (readyData.character_count || 0);
                    const allAnnounced = (expected > 0) && (announcedCharacters.size >= expected);
                    if (!activeCharacter && allAnnounced && Array.isArray(readyData.characters_active) && readyData.characters_active.length > 0) {
                        const firstCharacter = readyData.characters_active[0];
                        if (!characterTabs.has(firstCharacter)) {
                            createCharacterTab(firstCharacter);
                        }
                        selectCharacterTab(firstCharacter);
                    }
                } catch (e) { /* no-op */ }
                
                console.log(`System ready signal: ${readyData.character_count} characters expected`);
            } else {
                // System not ready - update status
                systemStatusText.textContent = 'Starting...';
                systemStatusText.style.color = '#888';
            }
        }
        
        async function sendText() {
            const character = document.getElementById('characterInput').value;
            const message = document.getElementById('messageInput').value;
            const resultDiv = document.getElementById('sendResult');
            
            if (!message) {
                resultDiv.innerHTML = '<span class="error">Message is required</span>';
                return;
            }
            
            try {
                const response = await fetch('/api/text_input', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        character: character,
                        message: message
                    })
                });
                
                const result = await response.json();
                
                if (result.success) {
                    resultDiv.innerHTML = `<span class="success">${result.message}</span>`;
                    document.getElementById('messageInput').value = '';
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.error}</span>`;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
            }
        }
        
        // Allow Shift+Enter to send message, Enter for new line
        document.getElementById('messageInput').addEventListener('keypress', function(e) {
            if (e.key === 'Enter' && e.shiftKey) {
                e.preventDefault(); // Prevent default to avoid new line
                sendText();
            }
            // Regular Enter key will create new line (default behavior)
        });
        
        async function stepTurn() {
            if (commandInProgress) return; // Prevent rapid clicks
            commandInProgress = true;
            
            const resultDiv = document.getElementById('turnResult');
            const stepButton = document.getElementById('stepButton');
            
            console.log('🔍 DEBUG: stepTurn() called - disabling Step button');
            
            // Immediately disable and shade the button for responsive UI
            stepButton.disabled = true;
            stepButton.style.background = '#555';
            stepButton.title = 'Waiting for all characters to complete their turns...';
            
            console.log('🔍 DEBUG: Step button disabled, commandInProgress =', commandInProgress);
            
            try {
                const response = await fetch('/api/turn/step', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    resultDiv.innerHTML = `<span class="success">${result.message}</span>`;
                    // Button state will be updated by backend via turn_state_update message
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.message}</span>`;
                    // Clear command lock on error so user can retry
                    commandInProgress = false;
                    stepButton.disabled = false;
                    stepButton.style.background = '#4ecdc4';
                    stepButton.title = 'Click to advance to next turn';
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
                // Clear command lock on error so user can retry
                commandInProgress = false;
                stepButton.disabled = false;
                stepButton.style.background = '#4ecdc4';
                stepButton.title = 'Click to advance to next turn';
            }
        }
        
        async function runTurns() {
            if (commandInProgress) return; // Prevent rapid clicks
            commandInProgress = true;
            
            const resultDiv = document.getElementById('turnResult');
            try {
                const response = await fetch('/api/turn/run', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    resultDiv.innerHTML = `<span class="success">${result.message}</span>`;
                    // Button state will be updated by backend via turn_state_update message
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.message}</span>`;
                    // Clear command lock on error so user can retry
                    commandInProgress = false;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
                // Clear command lock on error so user can retry
                commandInProgress = false;
            }
        }
        
        async function stopTurns() {
            if (commandInProgress) return; // Prevent rapid clicks
            commandInProgress = true;
            
            const resultDiv = document.getElementById('turnResult');
            try {
                const response = await fetch('/api/turn/stop', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    resultDiv.innerHTML = `<span class="success">${result.message}</span>`;
                    // Button state will be updated by backend via turn_state_update message
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.message}</span>`;
                    // Clear command lock on error so user can retry
                    commandInProgress = false;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
                // Clear command lock on error so user can retry
                commandInProgress = false;
            }
        }
        
        async function saveAll() {
            const resultDiv = document.getElementById('turnResult');
            try {
                const response = await fetch('/api/save', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    resultDiv.innerHTML = `<span class="success">SAVED: ${result.message}</span>`;
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.message}</span>`;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
            }
        }
        
        async function exportToObsidian() {
            const resultDiv = document.getElementById('turnResult');
            try {
                resultDiv.innerHTML = '<span style="color: #7c3aed;">📓 Exporting to Obsidian...</span>';
                
                const response = await fetch('/api/export_to_obsidian', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    resultDiv.innerHTML = `<span class="success">📓 ${result.message}</span>`;
                    console.log('Exported to:', result.path);
                } else {
                    resultDiv.innerHTML = `<span class="error">Export failed: ${result.message}</span>`;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Export error: ${error.message}</span>`;
            }
        }
        
        function showShutdownDialog() {
            const choice = confirm("Save data before shutdown?\\n\\nOK = Save & Shutdown\\nCancel = Shutdown without saving\\n\\n(Press ESC or close dialog to cancel shutdown)");
            
            if (choice === true) {
                // User chose "OK" - Save & Shutdown
                saveAndShutdown();
            } else if (choice === false) {
                // User chose "Cancel" - Shutdown without saving
                if (confirm("Shutdown without saving? All unsaved data will be lost.")) {
                    shutdownOnly();
                }
            }
            // If user closes dialog or presses ESC, do nothing (cancel)
        }
        
        async function saveAndShutdown() {
            const resultDiv = document.getElementById('turnResult');
            resultDiv.innerHTML = '<span style="color: orange;">SAVING data...</span>';
            
            try {
                const response = await fetch('/api/save_and_shutdown', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    resultDiv.innerHTML = '<span style="color: orange;">SHUTTING DOWN system...</span>';
                } else {
                    resultDiv.innerHTML = `<span class="error">Save failed: ${result.message}</span>`;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
            }
        }
        
        async function shutdownOnly() {
            const resultDiv = document.getElementById('turnResult');
            resultDiv.innerHTML = '<span style="color: orange;">SHUTTING DOWN system...</span>';
            
            try {
                const response = await fetch('/api/shutdown', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    resultDiv.innerHTML = '<span style="color: orange;">SYSTEM SHUTDOWN initiated</span>';
                } else {
                    resultDiv.innerHTML = `<span class="error">Shutdown failed: ${result.message}</span>`;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
            }
        }
        
        // Time slider functions
        function initTimeSlider() {
            const slider = document.getElementById('timeSlider');
            const sliderValue = document.getElementById('timeSliderValue');
            
            // Update display when slider moves
            slider.addEventListener('input', function() {
                sliderValue.textContent = this.value;
            });
            
            // Send value to server on mouse up (not during drag)
            slider.addEventListener('mouseup', function() {
                updateTimeDelay(this.value);
            });
            
            // Also handle touchend for mobile devices
            slider.addEventListener('touchend', function() {
                updateTimeDelay(this.value);
            });
        }
        
        async function updateTimeDelay(minutes) {
            try {
                const response = await fetch('/api/time/delay', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ minutes: parseInt(minutes) })
                });
                
                const result = await response.json();
                if (!result.success) {
                    console.error('Failed to update time delay:', result.message);
                }
            } catch (error) {
                console.error('Error updating time delay:', error);
            }
        }
        

        
        // Initialize time slider when page loads
        document.addEventListener('DOMContentLoaded', function() {
            initTimeSlider();
            // Get initial simulation time
            getInitialSimulationTime();
            // Send initial delay value (0) to backend
            updateTimeDelay(0);
        });
        
        async function getInitialSimulationTime() {
            try {
                const response = await fetch('/api/time/initial');
                const result = await response.json();
                
                if (result.success && result.time_info) {
                    const timeInfo = result.time_info;
                    const dateTime = new Date(timeInfo.datetime);
                    const displayTime = dateTime.toLocaleString('en-US', {
                        weekday: 'short',
                        year: 'numeric',
                        month: 'short',
                        day: 'numeric',
                        hour: '2-digit',
                        minute: '2-digit'
                    });
                    document.getElementById('simulationTime').textContent = displayTime;
                    console.log('⏰ Initial time loaded:', displayTime);
                } else {
                    document.getElementById('simulationTime').textContent = 'Starting up...';
                }
            } catch (error) {
                document.getElementById('simulationTime').textContent = 'Starting up...';
                console.log('Initial time query failed, waiting for WebSocket updates');
            }
        }
    </script>
</body>
</html>
        """
    
    def run(self):
        """Start the FastAPI server."""
        try:
            print(f'FastAPI Action Display Node running on port {self.port}')
            print('Press Ctrl+C to stop')
            
            # Open browser automatically
            webbrowser.open(f'http://localhost:{self.port}')
            
            # Start FastAPI server
            uvicorn.run(self.app, host="0.0.0.0", port=self.port)
            
        except KeyboardInterrupt:
            print('FastAPI Action Display Node shutting down...')
        finally:
            self.shutdown()
    
    def _set_event_loop(self):
        """Set the event loop reference when FastAPI starts."""
        try:
            self.event_loop = asyncio.get_running_loop()
            pass
        except RuntimeError:
            pass
    
    def action_callback(self, sample):
        """Handle incoming actions."""
        try:
            action_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.action_counter += 1
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/action
            

            
            # Track active character
            self.active_characters.add(character_name)
            
            # Handle character announcements
            if action_data.get('type') == 'announcement':
                self._handle_character_announcement(action_data, character_name)
                # Send announcement to web clients so tabs can be created
                self._send_to_websockets(action_data, character_name)
                return
            
            # Log action to trace file
            self.tracer.log_action(action_data, character_name)
            
            # Store action in memory
            self._store_action_in_memory(action_data, character_name)
            
            # Send to web clients
            self._send_to_websockets(action_data, character_name)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def goal_callback(self, sample):
        """Handle incoming character goals."""
        try:
            goal_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/goal
            
            # Store goal for this character
            with self.character_goals_lock:
                self.character_goals[character_name] = goal_data.get('goal', '')
            
            # Send goal update to web clients
            self._send_goal_to_websockets(goal_data, character_name)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def decided_action_callback(self, sample):
        """Handle incoming character decided actions."""
        try:
            decided_action_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/decided_action
            
            # Store decided action for this character
            with self.character_decided_actions_lock:
                self.character_decided_actions[character_name] = decided_action_data.get('decided_action', '')
            
            # Send decided action update to web clients
            self._send_decided_action_to_websockets(decided_action_data, character_name)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def current_plan_callback(self, sample):
        """Handle incoming character current plans."""
        try:
            current_plan_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/current_plan
            
            # Store current plan for this character
            with self.character_current_plans_lock:
                self.character_current_plans[character_name] = current_plan_data.get('current_plan', '')
            
            # Send current plan update to web clients
            self._send_current_plan_to_websockets(current_plan_data, character_name)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def current_activity_callback(self, sample):
        """Handle incoming character current activities."""
        try:
            current_activity_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/current_activity
            
            # Store current activity for this character
            with self.character_current_activities_lock:
                self.character_current_activities[character_name] = current_activity_data
            
            # Send current activity update to web clients
            self._send_current_activity_to_websockets(current_activity_data, character_name)
            
        except Exception as e:
            import traceback
            traceback.print_exc()

    def current_state_callback(self, sample):
        """Handle incoming character current internal state."""
        try:
            current_state_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/current_state
            # Store current state for this character
            if not hasattr(self, 'character_current_states'):
                self.character_current_states = {}
            self.character_current_states[character_name] = current_state_data
            # Send current state update to web clients
            self._send_current_state_to_websockets(current_state_data, character_name)
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def situation_callback(self, sample):
        """Handle incoming character situation data."""
        try:
            situation_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/situation/update
            
            # Store situation data for this character
            with self.character_situation_data_lock:
                self.character_situation_data[character_name] = situation_data
            
            # Send situation data update to web clients
            self._send_situation_data_to_websockets(situation_data, character_name)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def step_complete_callback(self, sample):
        """Handle step complete events from map node."""
        try:
            step_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Update turn state to indicate step is complete
            with self.turn_state_lock:
                self.turn_state['active_characters'] = []
                self.turn_state['completed_characters'] = []
                self.turn_state['turn_start_time'] = None
            
            # Send step_complete message to re-enable Step button
            self._send_step_complete_to_websockets()
            
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def turn_start_callback(self, sample):
        """Handle turn start events from map node."""
        try:
            turn_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            with self.turn_state_lock:
                self.turn_state['turn_number'] = turn_data.get('turn_number', self.turn_state['turn_number'] + 1)
                self.turn_state['active_characters'] = turn_data.get('active_characters', [])
                self.turn_state['completed_characters'] = []
                self.turn_state['turn_start_time'] = time.time()
            
            # Send turn start message to web clients
            self._send_turn_start_to_websockets(turn_data)
            
            # Send turn state update to web clients
            self._send_turn_state_update()
            
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def turn_control_callback(self, sample):
        """Handle turn control status updates from map node."""
        try:
            turn_control_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            with self.turn_state_lock:
                # Update mode based on map node's actual state
                self.turn_state['mode'] = turn_control_data.get('mode', 'step')
                logger.info(f"🔄 Turn control update from map node: mode={self.turn_state['mode']}")
            
            # Send just the mode update to the UI
            self._send_turn_mode_update()
            
        except Exception as e:
            import traceback
            traceback.print_exc()
    
    def turn_state_update_callback(self, sample):
        """
        Handle unified turn state updates from map node (NEW).
        
        This receives comprehensive state including:
        - Turn information (number, mode, progress)
        - Computed button states (enabled/disabled with tooltips)
        - All state in one message - no race conditions
        """
        try:
            state_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            logger.info(f"🆕 Received turn_state_update: turn={state_data['turn']['number']}, "
                       f"mode={state_data['turn']['mode']}, "
                       f"step_enabled={state_data['buttons']['step']['enabled']}")
            
            # Forward complete state to websockets for UI rendering (broadcast to all)
            with self.websocket_lock:
                for ws in self.websocket_connections:
                    try:
                        asyncio.run(ws.send_json(state_data))
                    except Exception as e:
                        logger.error(f"Error sending turn_state_update to websocket: {e}")
            
        except Exception as e:
            logger.error(f"Error in turn_state_update_callback: {e}")
            import traceback
            traceback.print_exc()
    
    def ready_callback(self, sample):
        """Handle launcher ready signal."""
        try:
            ready_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            if ready_data.get('status') == 'ready':
                self.system_ready = True
                self.character_count = ready_data.get('character_count', 0)
                
                logger.info(f'🚀 System ready signal received: {self.character_count} characters active')
                
                # Send ready state update to web clients
                self._send_ready_state_update()
                
                # Send current time to all connected WebSocket clients if available
                if self.current_simulation_time:
                    self._send_time_update_to_websockets(self.current_simulation_time.get('time_info', {}))
                    logger.info(f'📤 Sent current time to all WebSocket clients: {self.current_simulation_time.get("time_info", {}).get("datetime", "unknown")}')
                else:
                    logger.info('📤 No simulation time available to send to WebSocket clients')
                
        except Exception as e:
            import traceback
            traceback.print_exc()
    

    
    def time_advanced_callback(self, sample):
        """Handle time advancement updates from map_node."""
        try:
            time_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Extract new time info
            new_time_info = time_data.get('new_time_info', {})
            if new_time_info and 'datetime' in new_time_info:
                # Cache the full time data for future requests
                self.current_simulation_time = time_data
                #logger.info(f'⏰ Time advanced: {new_time_info["datetime"]}')
                
                # Send time update to web clients
                self._send_time_update_to_websockets(new_time_info)
            else:
                logger.warning(f'Received time_advanced but no valid time_info in data')
                
        except Exception as e:
            logger.error(f'Error in time_advanced callback: {e}')
    

    
    def _send_to_websockets(self, action_data: Dict[str, Any], character_name: str):
        """Send action data to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        
        # Check if this is a Say or response action - only show text
        action_type = action_data.get('type', '')
        is_text_only = action_type.lower() in ['say', 'response']
        
        # Prepare data for web clients
        web_data = {
            'type': 'action',
            'character': character_name,
            'action_id': action_data.get('action_id', 'unknown'),
            'action_type': action_data.get('type', 'unknown'),
            'is_text_only': is_text_only,
            'input_text': action_data.get('input_text', ''),
            'llm_response': action_data.get('llm_response', ''),
            'text': action_data.get('text', ''),
            'confidence': action_data.get('confidence'),
            'timestamp': action_data.get('timestamp', ''),
            'action': action_data.get('action', '') if not is_text_only else '',
            'target': action_data.get('target', ''),
            'requested_target': action_data.get('requested_target', ''),
            'resolved_target': action_data.get('resolved_target', ''),
            'status': action_data.get('status', ''),
            'error': action_data.get('error', ''),
            'source': action_data.get('source', ''),
            'value': action_data.get('value', '') if not is_text_only else '',
            'metadata': action_data.get('metadata', {}),
            'raw_data': action_data
        }
        

        
        # Send to all connected clients
        if self.event_loop is None:
            return
            
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    # Use asyncio.run_coroutine_threadsafe to send from non-async context
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(web_data)), 
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        disconnected.append(websocket)
            
            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)

    def _send_goal_to_websockets(self, goal_data: Dict[str, Any], character_name: str):
        """Send goal data to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        
        # Prepare goal data for web clients
        web_data = {
            'type': 'goal',
            'character': character_name,
            'goal': goal_data.get('goal', ''),
            'timestamp': goal_data.get('timestamp', '')
        }
        
        # Send to all connected clients
        if self.event_loop is None:
            return
            
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    # Use asyncio.run_coroutine_threadsafe to send from non-async context
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(web_data)), 
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        disconnected.append(websocket)
            
            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)

    def _send_decided_action_to_websockets(self, decided_action_data: Dict[str, Any], character_name: str):
        """Send decided action data to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        
        # Prepare decided action data for web clients
        web_data = {
            'type': 'decided_action',
            'character': character_name,
            'decided_action': decided_action_data.get('decided_action', ''),
            'timestamp': decided_action_data.get('timestamp', '')
        }
        
        # Send to all connected clients
        if self.event_loop is None:
            return
            
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    # Use asyncio.run_coroutine_threadsafe to send from non-async context
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(web_data)), 
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        disconnected.append(websocket)
            
            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    def _send_current_plan_to_websockets(self, current_plan_data: Dict[str, Any], character_name: str):
        """Send current plan data to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        
        # Prepare current plan data for web clients
        web_data = {
            'type': 'current_plan',
            'character': character_name,
            'current_plan': current_plan_data.get('current_plan', ''),
            'timestamp': current_plan_data.get('timestamp', '')
        }
        
        # Send to all connected clients
        if self.event_loop is None:
            return
            
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    # Use asyncio.run_coroutine_threadsafe to send from non-async context
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(web_data)), 
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        disconnected.append(websocket)
            
            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    def _send_current_activity_to_websockets(self, current_activity_data: Dict[str, Any], character_name: str):
        """Send current activity data to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        
        # Prepare current activity data for web clients
        web_data = {
            'type': 'current_activity',
            'character': character_name,
            'current_activity': current_activity_data.get('current_activity', ''),
            'activity_data': current_activity_data.get('activity_data'),
            'current_step': current_activity_data.get('current_step'),
            'activity_state': current_activity_data.get('activity_state'),
            'timestamp': current_activity_data.get('timestamp', '')
        }
        
        # Send to all connected clients
        if self.event_loop is None:
            return
            
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    # Use asyncio.run_coroutine_threadsafe to send from non-async context
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(web_data)), 
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        disconnected.append(websocket)
            
            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)

    def _send_current_state_to_websockets(self, current_state_data: Dict[str, Any], character_name: str):
        """Send current internal state data to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        web_data = {
            'type': 'current_state',
            'character': character_name,
            'state': current_state_data.get('state', {}),
            'timestamp': current_state_data.get('timestamp', '')
        }
        if self.event_loop is None:
            return
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(web_data)),
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    if not isinstance(e, TimeoutError):
                        disconnected.append(websocket)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    def _send_situation_data_to_websockets(self, situation_data: Dict[str, Any], character_name: str):
        """Send situation data to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        
        # Prepare situation data for web clients
        web_data = {
            'type': 'situation_data',
            'character': character_name,
            'situation_data': situation_data,
            'timestamp': datetime.now().isoformat()
        }
        
        # Send to all connected clients
        if self.event_loop is None:
            return
            
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    # Use asyncio.run_coroutine_threadsafe to send from non-async context
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(web_data)), 
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        disconnected.append(websocket)
            
            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    def _send_time_update_to_websockets(self, time_info: Dict[str, Any]):
        """Send time update to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        
        # Prepare time data for web clients
        web_data = {
            'type': 'time_update',
            'time_info': time_info,
            'timestamp': datetime.now().isoformat()
        }
        
        # Send to all connected clients
        if self.event_loop is None:
            return
            
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    # Use asyncio.run_coroutine_threadsafe to send from non-async context
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(web_data)), 
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        disconnected.append(websocket)
            
            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    def _send_turn_state_update(self):
        """Send the current turn state to all WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return

        with self.turn_state_lock:
            turn_state_data = {
                'type': 'turn_state',
                'turn_number': self.turn_state['turn_number'],
                'mode': self.turn_state['mode'],
                'active_characters': self.turn_state['active_characters'],
                'completed_characters': self.turn_state['completed_characters'],
                'step_button_disabled': self.turn_state.get('step_button_disabled', False)
            }
            logger.info(f"📤 Sending turn state update: mode={self.turn_state['mode']}, turn={self.turn_state['turn_number']}")

        # Actually send the data to clients
        if self.event_loop:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self._send_turn_state_to_all_clients(turn_state_data), 
                    self.event_loop
                )
                future.result(timeout=1.0)
            except asyncio.TimeoutError:
                pass  # Keep clients, timeouts are expected
            except Exception as e:
                # Only log other errors
                pass
        else:
            pass
    
    def _send_turn_mode_update(self):
        """Send just the turn mode update to all WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return

        with self.turn_state_lock:
            turn_mode_data = {
                'type': 'turn_mode_update',
                'mode': self.turn_state['mode']
            }
            logger.info(f"📤 Sending turn mode update: mode={self.turn_state['mode']}")

        if self.event_loop:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self._send_turn_mode_to_all_clients(turn_mode_data), 
                    self.event_loop
                )
                future.result(timeout=1.0)
            except asyncio.TimeoutError:
                pass  # Keep clients, timeouts are expected
            except Exception as e:
                # Only log other errors
                pass
        else:
            pass
    
    def _send_ready_state_update(self):
        """Send the current ready state to all WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return

        ready_state_data = {
            'type': 'ready_state',
            'system_ready': self.system_ready,
            'character_count': self.character_count
        }

        if self.event_loop:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self._send_ready_state_to_all_clients(ready_state_data), 
                    self.event_loop
                )
                future.result(timeout=1.0)
            except asyncio.TimeoutError:
                pass  # Keep clients, timeouts are expected
            except Exception as e:
                # Only log other errors
                pass
        else:
            pass
    
    async def _send_turn_mode_to_all_clients(self, turn_mode_data: Dict[str, Any]):
        """Send turn mode update to all connected WebSocket clients."""
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    await websocket.send_text(json.dumps(turn_mode_data))
                except Exception as e:
                    disconnected.append(websocket)
            
            # Remove disconnected clients
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    async def _send_turn_state_to_all_clients(self, turn_state_data: Dict[str, Any]):
        """Send turn state to all connected WebSocket clients."""
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    await websocket.send_text(json.dumps(turn_state_data))
                except Exception as e:
                    disconnected.append(websocket)
            
            # Remove disconnected clients
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    async def _send_ready_state_to_all_clients(self, ready_state_data: Dict[str, Any]):
        """Send ready state to all connected WebSocket clients."""
        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    await websocket.send_text(json.dumps(ready_state_data))
                except Exception as e:
                    disconnected.append(websocket)
            
            # Remove disconnected clients
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    def _send_step_complete_to_websockets(self):
        """Send a dedicated step_complete message to all WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return

        step_complete_data = {
            'type': 'step_complete'
        }

        if self.event_loop is None:
            return

        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(step_complete_data)),
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        import traceback
                        traceback.print_exc()
                        disconnected.append(websocket)

            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)

    def _send_turn_start_to_websockets(self, turn_data: Dict[str, Any]):
        """Send a dedicated turn_start message to all WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return

        turn_start_data = {
            'type': 'turn_start',
            'turn_number': turn_data.get('turn_number', 0),
            'active_characters': turn_data.get('active_characters', []),
            'timestamp': datetime.now().isoformat()
        }

        if self.event_loop is None:
            return

        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(turn_start_data)),
                        self.event_loop
                    )
                    future.result(timeout=5.0 if not self.debug else 300.0)
                except Exception as e:
                    # Don't remove client on timeout - just log the error
                    if not isinstance(e, TimeoutError):
                        import traceback
                        traceback.print_exc()
                        disconnected.append(websocket)

            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)

    
    def _store_action_in_memory(self, action_data: Dict[str, Any], character_name: str):
        """Store action in memory system."""
        try:
            memory_data = {
                'type': 'action',
                'character': character_name,
                'action_data': action_data,
                'timestamp': datetime.now().isoformat()
            }
            self.memory_publisher.put(json.dumps(memory_data))
        except Exception as e:
            print(f'❌ Error storing action in memory: {e}')
    
    def _store_text_input_in_memory(self, text_input: str, character_name: str):
        """Store text input in memory system."""
        try:
            memory_data = {
                'type': 'text_input',
                'character': character_name,
                'text': text_input,
                'source': 'User',
                'timestamp': datetime.now().isoformat()
            }
            self.memory_publisher.put(json.dumps(memory_data))
        except Exception as e:
            print(f'❌ Error storing text input in memory: {e}')
    
    def _handle_character_announcement(self, action_data: Dict[str, Any], character_name: str):
        """Handle character announcement actions."""

        self.active_characters.add(character_name)
    
    def shutdown(self):
        """Shutdown the node."""
        try:
            print('FastAPI Action Display Node shutdown initiated...')
            
            # Set shutdown flag
            self.shutdown_requested = True
            
            # Close WebSocket connections
            with self.websocket_lock:
                for websocket in self.websocket_connections:
                    try:
                        websocket.close()
                    except:
                        pass
            
            # Close action tracer
            if hasattr(self, 'tracer'):
                self.tracer.close()
                print('Action trace file closed')
            
            # Close Zenoh session
            if hasattr(self, 'session'):
                time.sleep(1.0)  # Give time for cleanup
                self.session.close()
            
            print('✅ FastAPI Action Display Node shutdown complete')
        except Exception as e:
            print(f'❌ Error during shutdown: {e}')


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='FastAPI Action Display Node')
    parser.add_argument('--port', type=int, default=3000, help='Port for FastAPI server (default: 3000)')
    parser.add_argument('--scenario', type=str, default=None, help='Scenario name for trace file')
    parser.add_argument('--obsidian-vault', type=str, 
                       default=os.getenv('OBSIDIAN_VAULT_PATH', '/home/bruce/Documents/Obsidian Vault'),
                       help='Path to Obsidian vault for trace exports (default: /home/bruce/Documents/Obsidian Vault)')
    args = parser.parse_args()
    
    node = FastAPIActionDisplayNode(port=args.port, scenario_name=args.scenario, obsidian_vault=args.obsidian_vault)
    node.run()


if __name__ == "__main__":
    main() 