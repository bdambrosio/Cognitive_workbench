#!/usr/bin/env python3
"""
FastAPI Action Display Node - FIXED VERSION

This node displays actions via a web UI and provides text input interface.
Replaces the console-based action_display_node with a web interface.
"""

import zenoh
from zenoh import ConsolidationMode, QueryTarget
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
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, Body
from fastapi.responses import HTMLResponse, FileResponse
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
        self.start_time = datetime.now()
        self._trace_lines = [
            f"# Action Trace: {self.scenario_name}\n",
            f"Started: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}\n",
        ]
    
    def log_action(self, action_data: dict, character_name: str):
        """Log an action to the in-memory trace."""
        
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
            
            self._trace_lines.append(line)
            
        except Exception as e:
            logger.error(f"Error logging action to trace: {e}")
    
    def export_to_file(self, path: Path):
        """Write the in-memory trace to a file."""
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('\n'.join(self._trace_lines) + '\n')

    def close(self):
        """No-op, kept for API compatibility."""
        pass


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
        self.character_scheduler_defaults: Dict[str, Dict[str, Any]] = {}
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
        
        
        # Subscriber for execution state updates from executive node
        self.execution_state_subscriber = None  # Will be set when character is known
        
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
        
        # Subscriber for character world state updates
        self.world_state_subscriber = self.session.declare_subscriber(
            "cognitive/*/world_state",
            self.world_state_callback
        )
        
        # Subscriber for character current state
        # Subscriber for launcher ready signal
        self.ready_subscriber = self.session.declare_subscriber(
            "cognitive/launcher/ready",
            self.ready_callback
        )
        
        # Publisher for memory storage
        self.memory_publisher = self.session.declare_publisher("cognitive/memory/store")
        
        # Track active character name for direct control
        self.active_character_name = None
        
        # Publishers for direct execution control (will be set when character is known)
        self.control_step_publisher = None
        self.control_run_publisher = None
        self.control_autonomous_publisher = None
        self.control_stop_publisher = None
        self.control_continuous_publisher = None
        
        # Publisher for time management
        self.time_delay_publisher = self.session.declare_publisher("cognitive/map/time_delay_setting")
        
        
        # Publisher for save commands
        self.save_publisher = self.session.declare_publisher("cognitive/save_all")
        
        # Current time tracking (using datetime)
        from datetime import datetime
        self.current_time = datetime.now()
        
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
    
    async def _handle_websocket_message(self, websocket, msg: Dict):
        """Handle incoming websocket messages (test-related commands)."""
        import yaml
        msg_type = msg.get('type', '')
        
        if msg_type == 'list_test_files':
            # List available test files from tests/eval/ directory
            # Get absolute path: fastapi_action_display.py is in src/, tests/ is sibling to src/
            script_dir = Path(__file__).resolve().parent  # src/
            test_dir = script_dir.parent / 'tests' / 'eval'  # workspace/tests/eval
            test_files = []
            logger.info(f"🧪 Script location: {Path(__file__).resolve()}")
            logger.info(f"🧪 Looking for test files in: {test_dir}")
            logger.info(f"🧪 Directory exists: {test_dir.exists()}")
            if test_dir.exists():
                yaml_files = list(test_dir.glob('*.yaml'))
                logger.info(f"🧪 Found {len(yaml_files)} YAML files: {[f.name for f in yaml_files]}")
                for test_file in yaml_files:
                    test_files.append({
                        'name': test_file.name,
                        'path': str(test_file)
                    })
            else:
                logger.warning(f"🧪 Test directory does not exist: {test_dir}")
            
            await websocket.send_text(json.dumps({
                'type': 'test_files_list',
                'files': test_files
            }))
        
        elif msg_type == 'get_test_details':
            # Load and return test file details
            test_file = msg.get('test_file', '')
            try:
                with open(test_file, 'r') as f:
                    test_config = yaml.safe_load(f)
                
                await websocket.send_text(json.dumps({
                    'type': 'test_details',
                    'details': {
                        'name': test_config.get('name', 'Test'),
                        'description': test_config.get('description', ''),
                        'allowed_tools': test_config.get('allowed_tools'),
                        'test_goal': test_config.get('test_goal', '')
                    }
                }))
            except Exception as e:
                await websocket.send_text(json.dumps({
                    'type': 'error',
                    'message': f'Failed to load test file: {str(e)}'
                }))
        
        elif msg_type == 'run_test':
            # Run a test by sending goals to character with evaluation mode
            character = msg.get('character', '')
            test_file = msg.get('test_file', '')
            
            try:
                # Load test config
                with open(test_file, 'r') as f:
                    test_config = yaml.safe_load(f)
                
                # Log test start to UI action log (direct UI entry, bypasses action_data pub/sub)
                test_name = test_config.get('name', Path(test_file).stem)
                test_action_data = {
                    'type': 'evaluation_test_start',
                    'test_name': test_name,
                    'test_file': test_file,
                    'timestamp': datetime.now().isoformat()
                }
                self.tracer.log_action(test_action_data, character)
                self._send_to_websockets(test_action_data, character)
                
                # Enable compliance tracking via Zenoh message
                self.session.put(
                    f"cognitive/{character}/enable_compliance_tracking",
                    json.dumps({})
                )
                # Give it a moment to register
                await asyncio.sleep(0.2)
                
                # Send setup goals if any (as User text to character's sense_data)
                for setup_goal in test_config.get('setup_goals', []):
                    # sense_data_callback expects 'content' field, which can be JSON string or plain text
                    # Format as JSON string: {"text": "...", "source": "..."}
                    content_json = json.dumps({
                        'text': setup_goal,
                        'source': 'User'
                    })
                    sense_data = {
                        'content': content_json,
                        'timestamp': datetime.now().isoformat()
                    }
                    self.session.put(
                        f"cognitive/{character}/sense_data",
                        json.dumps(sense_data)
                    )
                    await asyncio.sleep(0.5)
                
                # Send main test goal (as User text to character's sense_data)
                test_goal = test_config.get('test_goal', '')
                # Format as JSON string: {"text": "...", "source": "..."}
                content_json = json.dumps({
                    'text': test_goal,
                    'source': 'User'
                })
                sense_data = {
                    'content': content_json,
                    'timestamp': datetime.now().isoformat()
                }
                self.session.put(
                    f"cognitive/{character}/sense_data",
                    json.dumps(sense_data)
                )
                
                # Give character a moment to receive the goal
                await asyncio.sleep(0.3)
                
                # Automatically trigger autonomous mode to start plan execution
                # This uses the same mechanism as the Autonomous button
                if self.active_character_name and self.control_autonomous_publisher:
                    with self.turn_state_lock:
                        self.turn_state['mode'] = 'autonomous'
                    self.control_autonomous_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                
                await websocket.send_text(json.dumps({
                    'type': 'test_status',
                    'status': {
                        'running': True,
                        'message': 'Test goal sent, step triggered automatically...'
                    }
                }))
                
            except Exception as e:
                await websocket.send_text(json.dumps({
                    'type': 'test_status',
                    'status': {
                        'error': str(e)
                    }
                }))
    
    def _setup_routes(self):
        """Setup FastAPI routes."""
        
        @self.app.get("/favicon.ico")
        async def favicon():
            """Serve favicon if it exists, otherwise return 204 No Content."""
            favicon_path = Path("static/favicon.ico")
            if not favicon_path.exists():
                # Try PNG format
                favicon_path = Path("static/favicon.png")
            
            if favicon_path.exists():
                return FileResponse(favicon_path)
            else:
                # Return 204 No Content to suppress error
                from fastapi import Response
                return Response(status_code=204)
        
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
                
                # Send current time
                from datetime import datetime
                current_time = datetime.now()
                time_info = {
                    'year': current_time.year,
                    'month': current_time.month,
                    'day': current_time.day,
                    'hour': current_time.hour,
                    'minute': current_time.minute,
                    'datetime': current_time.isoformat(),
                    'period': 'morning' if 5 <= current_time.hour < 12 else 'afternoon' if 12 <= current_time.hour < 17 else 'evening' if 17 <= current_time.hour < 21 else 'night',
                    'season': 'spring' if 3 <= current_time.month <= 5 else 'summer' if 6 <= current_time.month <= 8 else 'autumn' if 9 <= current_time.month <= 11 else 'winter'
                }
                time_data = {
                    'type': 'time_update',
                    'time_info': time_info,
                    'timestamp': current_time.isoformat()
                }
                await websocket.send_text(json.dumps(time_data))
                logger.info(f'📤 Sent current time to new WebSocket connection: {current_time.isoformat()}')
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
                                
                                # Try to parse JSON messages
                                try:
                                    msg = json.loads(text)
                                    await self._handle_websocket_message(websocket, msg)
                                except json.JSONDecodeError:
                                    pass
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
            
            # No conversation locks needed with single character
            
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

        @self.app.post("/api/interrupt")
        async def interrupt_character(data: Dict[str, Any]):
            """Request an interrupt for the specified character (pauses + interrupts current planner loop)."""
            character_name = data.get('character', '')

            # If no character specified, use last character (same behavior as /api/text_input)
            if not character_name:
                if not self.last_character_name:
                    return {"success": False, "error": "No character specified and no previous character"}
                character_name = self.last_character_name

            # Find actual character name (case-insensitive)
            actual_character_name = None
            for active_char in self.active_characters:
                if active_char.lower() == character_name.lower():
                    actual_character_name = active_char
                    break

            if not actual_character_name:
                return {"success": False, "error": f"Character '{character_name}' not found. Available: {', '.join(sorted(self.active_characters))}"}

            # Store as last character used
            self.last_character_name = actual_character_name

            # Publish interrupt control message (payload content is ignored by subscriber)
            payload = {
                "timestamp": datetime.now().isoformat(),
                "source": "ui",
                "command": "interrupt"
            }
            self.session.put(
                f"cognitive/{actual_character_name}/control/interrupt",
                json.dumps(payload).encode("utf-8")
            )

            return {"success": True, "message": f"Interrupt requested for {actual_character_name}"}
        
        @self.app.post("/api/execute_plan_sync")
        async def execute_plan_sync(data: Dict[str, Any]):
            """Execute a plan synchronously via executive_node."""
            character_name = data.get('character', '')
            plan = data.get('plan')
            
            if not plan:
                return {"success": False, "error": "Plan is required"}
            
            # If no character specified, use last character
            if not character_name:
                if not self.last_character_name:
                    return {"success": False, "error": "No character specified and no previous character"}
                character_name = self.last_character_name
            
            # Find actual character name (case-insensitive)
            actual_character_name = None
            for active_char in self.active_characters:
                if active_char.lower() == character_name.lower():
                    actual_character_name = active_char
                    break
            
            if not actual_character_name:
                return {"success": False, "error": f"Character '{character_name}' not found. Available: {', '.join(sorted(self.active_characters))}"}
            
            # Store as last character used
            self.last_character_name = actual_character_name
            
            # Query executive_node for sync plan execution
            try:
                request_payload = json.dumps({
                    'plan': plan,
                    'max_steps': 1000
                }).encode('utf-8')
                
                response_received = False
                result = None
                
                for reply in self.session.get(
                    f"cognitive/{actual_character_name}/execute_plan_sync",
                    target=QueryTarget.BEST_MATCHING,
                    consolidation=ConsolidationMode.NONE,
                    payload=request_payload,
                    timeout=300.0  # 5 minutes for long-running plans
                ):
                    if reply.ok:
                        result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        response_received = True
                        break
                
                if not response_received:
                    return {"success": False, "error": "No response from executive_node"}
                
                if result.get('success'):
                    return {
                        "success": True,
                        "status": result.get('status'),
                        "executed_steps": result.get('executed_steps', 0),
                        "bindings": result.get('bindings', {}),
                        "last_action_result": result.get('last_action_result'),  # Uniform format: {status, value, resource_id, reason}
                        "suspended": result.get('suspended', False),
                        "suspension_reason": result.get('suspension_reason')
                    }
                else:
                    return {
                        "success": False,
                        "error": result.get('error', 'Unknown error'),
                        "status": result.get('status'),
                        "reason": result.get('reason'),
                        "last_action_result": result.get('last_action_result')  # May contain last result even on failure
                    }
                    
            except Exception as e:
                logger.error(f'Error executing sync plan: {e}')
                import traceback
                logger.error(traceback.format_exc())
                return {"success": False, "error": str(e)}
        
        @self.app.post("/api/end_dialog")
        async def end_dialog(data: Dict[str, str]):
            """End dialog with a character - User-only action."""
            character_name = data.get('character', '')
            
            if not character_name:
                return {"error": "Character name is required"}
            
            # Find actual character name (case-insensitive)
            actual_character_name = None
            for active_char in self.active_characters:
                if active_char.lower() == character_name.lower():
                    actual_character_name = active_char
                    break
            
            if not actual_character_name:
                return {"error": f"Character '{character_name}' not found. Available: {', '.join(sorted(self.active_characters))}"}
            
            # No conversation locks needed with single character
            
            # No conversation locks needed with single character
            
            # Send close_dialog messages to both User and target character
            # Close User's dialog with target
            self.session.put(
                f"cognitive/User/memory/close_dialog",
                json.dumps({'entity_name': actual_character_name})
            )
            
            # Close target's dialog with User
            self.session.put(
                f"cognitive/{actual_character_name}/memory/close_dialog",
                json.dumps({'entity_name': 'User'})
            )

            # Also issue immediate stop/interrupt so any active ask/wait exits now.
            stop_payload = json.dumps({"timestamp": datetime.now().isoformat()}).encode()
            interrupt_payload = json.dumps({
                "timestamp": datetime.now().isoformat(),
                "source": "ui",
                "command": "interrupt"
            }).encode("utf-8")
            self.session.put(f"cognitive/{actual_character_name}/control/stop", stop_payload)
            self.session.put(f"cognitive/{actual_character_name}/control/interrupt", interrupt_payload)
            
            logger.info(f"User ended dialog with {actual_character_name}")
            return {"success": True, "message": f"Dialog ended with {actual_character_name}"}
        
        @self.app.get("/api/conversation_status")
        async def get_conversation_status():
            """Get User's conversation lock status (always unlocked with single character)."""
            return {"is_locked": False, "locked_with": []}
        
        @self.app.post("/api/turn/autonomous")
        async def autonomous_turns():
            """Start autonomous execution."""
            try:
                if self.active_character_name is None:
                    return {"success": False, "message": "No character available yet"}
                
                logger.info(f"🤖 Autonomous command received in FastAPI")
                
                with self.turn_state_lock:
                    self.turn_state['mode'] = 'autonomous'
                
                self.control_autonomous_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                logger.info(f"🤖 Autonomous command sent to {self.active_character_name}")
                
                return {"success": True, "message": "Autonomous command sent"}
            except Exception as e:
                logger.error(f"Error in autonomous_turns: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/turn/stop")
        async def stop_turns():
            """Stop execution and trigger interrupt for ALL active characters."""
            try:
                if not self.active_characters:
                    return {"success": False, "message": "No characters available yet"}
                
                logger.info(f"🛑 Stop command received in FastAPI")
                
                with self.turn_state_lock:
                    self.turn_state['mode'] = 'step'
                
                stop_payload = json.dumps({"timestamp": datetime.now().isoformat()}).encode()
                interrupt_payload = json.dumps({
                    "timestamp": datetime.now().isoformat(),
                    "source": "ui",
                    "command": "interrupt"
                }).encode("utf-8")
                
                # Broadcast stop and interrupt to ALL active characters
                for char_name in self.active_characters:
                    self.session.put(f"cognitive/{char_name}/control/stop", stop_payload)
                    self.session.put(f"cognitive/{char_name}/control/interrupt", interrupt_payload)
                logger.info(f"🛑 Stop and interrupt commands sent to all: {list(self.active_characters)}")
                
                return {"success": True, "message": f"Stop and interrupt sent to {len(self.active_characters)} character(s)"}
            except Exception as e:
                logger.error(f"Error in stop_turns: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/turn/continuous")
        async def toggle_continuous():
            """Toggle continuous mode on/off."""
            try:
                if self.active_character_name is None:
                    return {"success": False, "message": "No character available yet"}
                
                logger.info(f"🔄 Continuous toggle command received in FastAPI")
                
                # Publish continuous toggle command (enable=None means toggle)
                self.control_continuous_publisher.put(json.dumps({"timestamp": datetime.now().isoformat(), "enable": None}).encode())
                logger.info(f"🔄 Continuous toggle command sent to {self.active_character_name}")
                
                return {"success": True, "message": "Continuous toggle command sent"}
            except Exception as e:
                logger.error(f"Error in toggle_continuous: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/task_scheduler/{character}")
        async def toggle_task_scheduler(character: str, data: dict = Body(...)):
            """Control task scheduler (enable/disable, interval)."""
            try:
                payload = json.dumps(data).encode()
                self.session.put(f"cognitive/{character}/control/task_scheduler", payload)
                return {"success": True}
            except Exception as e:
                logger.error(f"Error in toggle_task_scheduler: {e}")
                return {"success": False, "message": str(e)}

        @self.app.post("/api/task_schedule_mode/{character}")
        async def set_task_schedule_mode(character: str, data: dict = Body(...)):
            """Set per-task schedule mode (manual/auto/recurring)."""
            try:
                payload = json.dumps(data).encode()
                self.session.put(f"cognitive/{character}/control/task_schedule_mode", payload)
                return {"success": True}
            except Exception as e:
                logger.error(f"Error in set_task_schedule_mode: {e}")
                return {"success": False, "message": str(e)}

        @self.app.post("/api/goal_schedule_mode/{character}")
        async def set_goal_schedule_mode(character: str, data: dict = Body(...)):
            """Set per-goal schedule mode (manual/auto/recurring/daily)."""
            try:
                payload = json.dumps(data).encode()
                self.session.put(f"cognitive/{character}/control/goal_schedule_mode", payload)
                return {"success": True}
            except Exception as e:
                logger.error(f"Error in set_goal_schedule_mode: {e}")
                return {"success": False, "message": str(e)}

        @self.app.post("/api/goal_rename/{character}")
        async def rename_goal(character: str, data: dict = Body(...)):
            """Rename a scheduled goal."""
            try:
                payload = json.dumps(data).encode()
                self.session.put(f"cognitive/{character}/control/goal_rename", payload)
                return {"success": True}
            except Exception as e:
                logger.error(f"Error in rename_goal: {e}")
                return {"success": False, "message": str(e)}

        @self.app.post("/api/goal_text_update/{character}")
        async def goal_text_update(character: str, data: dict = Body(...)):
            """Update the goal_text of a scheduled goal."""
            try:
                payload = json.dumps(data).encode()
                self.session.put(f"cognitive/{character}/control/goal_text_update", payload)
                return {"success": True}
            except Exception as e:
                logger.error(f"Error in goal_text_update: {e}")
                return {"success": False, "message": str(e)}

        @self.app.post("/api/goal_cache/{character}")
        async def goal_cache(character: str, data: dict = Body(...)):
            """Manage scheduled goal cache operations."""
            try:
                payload = json.dumps(data).encode()
                self.session.put(f"cognitive/{character}/control/goal_cache", payload)
                return {"success": True}
            except Exception as e:
                logger.error(f"Error in goal_cache: {e}")
                return {"success": False, "message": str(e)}

        @self.app.post("/api/goal_interrupt/{character}")
        async def goal_interrupt(character: str, data: dict = Body(...)):
            """Interrupt a scheduled goal without removing it."""
            try:
                payload = json.dumps(data).encode()
                self.session.put(f"cognitive/{character}/control/goal_interrupt", payload)
                return {"success": True}
            except Exception as e:
                logger.error(f"Error in goal_interrupt: {e}")
                return {"success": False, "message": str(e)}

        @self.app.post("/api/goal_remove/{character}")
        async def goal_remove(character: str, data: dict = Body(...)):
            """Remove a scheduled goal, interrupting first if needed."""
            try:
                payload = json.dumps(data).encode()
                self.session.put(f"cognitive/{character}/control/goal_remove", payload)
                return {"success": True}
            except Exception as e:
                logger.error(f"Error in goal_remove: {e}")
                return {"success": False, "message": str(e)}

        @self.app.post("/api/control/clear_world_model")
        async def clear_world_model():
            """Clear world model."""
            try:
                if self.active_character_name is None:
                    return {"success": False, "message": "No character available yet"}
                
                logger.info(f"🗑️ Clear world model command received")
                
                # Publish clear world model command
                topic = f"cognitive/{self.active_character_name}/control/clear_world_model"
                publisher = self.session.declare_publisher(topic)
                publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                logger.info(f"🗑️ Clear world model command sent to {self.active_character_name}")
                
                return {"success": True, "message": "World model clear command sent"}
            except Exception as e:
                logger.error(f"Error in clear_world_model: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/control/clear_map")
        async def clear_map():
            """Clear map."""
            try:
                if self.active_character_name is None:
                    return {"success": False, "message": "No character available yet"}
                
                logger.info(f"🗑️ Clear map command received")
                
                # Publish clear map command
                topic = f"cognitive/{self.active_character_name}/control/clear_map"
                publisher = self.session.declare_publisher(topic)
                publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                logger.info(f"🗑️ Clear map command sent to {self.active_character_name}")
                
                return {"success": True, "message": "Map clear command sent"}
            except Exception as e:
                logger.error(f"Error in clear_map: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/control/clear_transients")
        async def clear_transients(data: Dict[str, str] = {}):
            """Clear transients for a specific character."""
            try:
                character = data.get('character') or self.active_character_name
                if character is None:
                    return {"success": False, "message": "No character available yet"}
                
                logger.info(f"🗑️ Clear transients command received for {character}")
                
                # Publish clear transients command
                topic = f"cognitive/{character}/control/clear_transients"
                publisher = self.session.declare_publisher(topic)
                publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                logger.info(f"🗑️ Clear transients command sent to {character}")
                
                return {"success": True, "message": f"Clear transients command sent to {character}"}
            except Exception as e:
                logger.error(f"Error in clear_transients: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/control/clear_persistents")
        async def clear_persistents(data: Dict[str, str] = {}):
            """Clear persistents for a specific character."""
            try:
                character = data.get('character') or self.active_character_name
                if character is None:
                    return {"success": False, "message": "No character available yet"}
                
                logger.info(f"🗑️ Clear persistents command received for {character}")
                
                # Publish clear persistents command
                topic = f"cognitive/{character}/control/clear_persistents"
                publisher = self.session.declare_publisher(topic)
                publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                logger.info(f"🗑️ Clear persistents command sent to {character}")
                
                return {"success": True, "message": f"Clear persistents command sent to {character}"}
            except Exception as e:
                logger.error(f"Error in clear_persistents: {e}")
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
        
        @self.app.post("/api/save_plan")
        async def save_plan(request: Request):
            """Save executed plan as a reusable tool in saved_plans/ directory."""
            try:
                data = await request.json()
                tool_name = data.get('tool_name', '').strip()
                goal = data.get('goal', '').strip()
                plan_actions = data.get('plan', [])
                parameters = data.get('parameters', [])  # List of {name, value, description}
                overwrite = data.get('overwrite', False)
                
                # Validate inputs
                if not tool_name:
                    return {"success": False, "message": "Tool name is required"}
                if not goal:
                    return {"success": False, "message": "Goal is required"}
                if not plan_actions:
                    return {"success": False, "message": "Plan is empty"}
                
                # Validate tool name format (kebab-case)
                import re
                if not re.match(r'^[a-z][a-z0-9-]*$', tool_name):
                    return {"success": False, "message": "Tool name must be lowercase kebab-case (e.g., 'research-papers')"}
                
                # Create saved_plans directory if it doesn't exist
                saved_plans_dir = Path('saved_plans')
                saved_plans_dir.mkdir(exist_ok=True)
                
                # Create tool directory
                tool_dir = saved_plans_dir / tool_name
                if tool_dir.exists():
                    if not overwrite:
                        return {"success": False, "message": f"Tool '{tool_name}' already exists"}
                    # Overwrite - remove existing directory
                    import shutil
                    shutil.rmtree(tool_dir)
                tool_dir.mkdir()
                
                # Generate SKILL.md with frontmatter
                skill_content = f"""---
name: {tool_name}
description: {goal}
type: plan
manual_only: true
parameters:
"""
                for param in parameters:
                    param_name = param.get('name', '')
                    param_desc = param.get('description', '')
                    param_required = param.get('required', True)
                    skill_content += f"  - name: {param_name}\n"
                    if param_desc:
                        skill_content += f"    description: {param_desc}\n"
                    skill_content += f"    required: {param_required}\n"
                
                skill_content += """---

# {tool_name}

{goal}

## Parameters

"""
                for param in parameters:
                    param_name = param.get('name', '')
                    param_desc = param.get('description', 'No description')
                    param_default = param.get('value', '')
                    skill_content += f"- `{param_name}`: {param_desc}"
                    if param_default:
                        skill_content += f" (default: `{param_default}`)"
                    skill_content += "\n"
                
                skill_content += """
## Usage

```json
{{
  "type": "{tool_name}",
  "args": {{"param1": "value1", ...}},
  "out": "$result",
  "expect": "..."
}}
```

## Implementation

This is a saved plan generated from an executed goal. The plan is stored in `plan.json` and will be executed directly when this tool is invoked.

Original goal: {goal}

Generated: {generated_at}
"""
                
                # Write SKILL.md
                skill_path = tool_dir / 'SKILL.md'
                skill_path.write_text(skill_content.format(
                    tool_name=tool_name,
                    goal=goal,
                    generated_at=datetime.now().isoformat()
                ))
                
                # Generate plan.json
                plan_data = {
                    "goal": goal,
                    "plan": plan_actions,
                    "out": "$result",
                    "parameters": {p['name']: p.get('value', '') for p in parameters},
                    "saved_at": datetime.now().isoformat()
                }
                
                plan_path = tool_dir / 'plan.json'
                plan_path.write_text(json.dumps(plan_data, indent=2))
                
                logger.info(f"💾 Saved plan as tool: {tool_name} in {tool_dir}")
                
                return {
                    "success": True,
                    "message": f"Plan saved as tool: {tool_name}",
                    "path": str(tool_dir)
                }
                
            except Exception as e:
                logger.error(f"Error saving plan: {e}")
                import traceback
                traceback.print_exc()
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/saved_plans")
        async def list_saved_plans():
            """List all saved plans in the saved_plans directory."""
            try:
                saved_plans_dir = Path('saved_plans')
                if not saved_plans_dir.exists():
                    return {"success": True, "plans": []}
                
                plans = []
                for plan_dir in saved_plans_dir.iterdir():
                    if plan_dir.is_dir() and not plan_dir.name.startswith('.'):
                        skill_path = plan_dir / 'SKILL.md'
                        plan_path = plan_dir / 'plan.json'
                        
                        if skill_path.exists() and plan_path.exists():
                            # Read plan metadata
                            plan_data = json.loads(plan_path.read_text())
                            plans.append({
                                "name": plan_dir.name,
                                "goal": plan_data.get("goal", "No description"),
                                "saved_at": plan_data.get("saved_at", "Unknown"),
                                "action_count": len(plan_data.get("plan", []))
                            })
                
                # Sort by saved_at descending (most recent first)
                plans.sort(key=lambda x: x.get("saved_at", ""), reverse=True)
                
                return {"success": True, "plans": plans}
            except Exception as e:
                logger.error(f"Error listing saved plans: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/execute_saved_plan")
        async def execute_saved_plan(request: Request):
            """Execute a saved plan."""
            try:
                data = await request.json()
                plan_name = data.get("plan_name")
                character = data.get("character")
                
                if not plan_name:
                    return {"success": False, "message": "plan_name required"}
                
                if not character:
                    return {"success": False, "message": "character required"}
                
                plan_path = Path(f'saved_plans/{plan_name}/plan.json')
                if not plan_path.exists():
                    return {"success": False, "message": f"Plan {plan_name} not found"}
                
                topic = f"cognitive/{character}/execute_saved_plan"
                payload = json.dumps({"plan_name": plan_name})
                self.session.put(topic, payload.encode('utf-8'))
                
                logger.info(f"Executing saved plan {plan_name} for {character}")
                return {"success": True, "message": f"Executing {plan_name}"}
            except Exception as e:
                logger.error(f"Error executing saved plan: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/test_goals")
        async def list_test_goals():
            """List all goals from goals directory."""
            try:
                import yaml
                goals_dir = Path('goals')
                if not goals_dir.exists():
                    return {"success": True, "goals": []}
                
                goals = []
                for yaml_file in goals_dir.glob('*.yaml'):
                    try:
                        content = yaml.safe_load(yaml_file.read_text())
                        if content:
                            goals.append({
                                "name": yaml_file.stem,
                                "title": content.get("name", yaml_file.stem),
                                "description": content.get("description", ""),
                                "test_goal": content.get("test_goal", ""),
                                "filename": yaml_file.name
                            })
                    except Exception as e:
                        logger.warning(f"Error reading {yaml_file}: {e}")
                
                goals.sort(key=lambda x: x["name"])
                return {"success": True, "goals": goals}
            except Exception as e:
                logger.error(f"Error listing goals: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/goal_details/{goal_name}")
        async def get_goal_details(goal_name: str):
            """Get full details of a goal."""
            try:
                import yaml
                yaml_file = Path(f'goals/{goal_name}.yaml')
                if not yaml_file.exists():
                    return {"success": False, "message": "Goal not found"}
                
                content = yaml.safe_load(yaml_file.read_text())
                return {"success": True, "content": content, "raw": yaml_file.read_text()}
            except Exception as e:
                logger.error(f"Error reading goal {goal_name}: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/plan_details/{plan_name}")
        async def get_plan_details(plan_name: str):
            """Get full details of a saved plan."""
            try:
                plan_path = Path(f'saved_plans/{plan_name}/plan.json')
                if not plan_path.exists():
                    return {"success": False, "message": "Plan not found"}
                
                content = json.loads(plan_path.read_text())
                return {"success": True, "content": content, "raw": plan_path.read_text()}
            except Exception as e:
                logger.error(f"Error reading plan {plan_name}: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/execute_test_goal")
        async def execute_test_goal(request: Request):
            """Execute a goal."""
            try:
                import yaml
                data = await request.json()
                goal_file = data.get("goal_file")
                character = data.get("character")
                
                if not goal_file:
                    return {"success": False, "message": "goal_file required"}
                
                if not character:
                    return {"success": False, "message": "character required"}
                
                goal_path = Path(f'goals/{goal_file}')
                if not goal_path.exists():
                    return {"success": False, "message": f"Goal {goal_file} not found"}
                
                # Read goal from YAML
                goal_data = yaml.safe_load(goal_path.read_text())
                test_goal = goal_data.get('test_goal', '')
                
                if not test_goal:
                    return {"success": False, "message": "No test_goal found in YAML"}
                
                # Send goal via sense_data (same mechanism as text input)
                sense_data = {
                    'timestamp': datetime.now().isoformat(),
                    'sequence_id': 0,
                    'mode': 'text',
                    'content': json.dumps({
                        'text': test_goal,  # Goal text (already includes 'goal:' prefix in YAML)
                        'source': 'User'
                    })
                }
                
                topic = f"cognitive/{character}/sense_data"
                self.session.put(topic, json.dumps(sense_data).encode('utf-8'))
                
                # Publish User's action so it appears in UI/trace
                user_action = {
                    'type': 'say',
                    'action_id': f'user_goal_{int(time.time() * 1000)}',
                    'timestamp': datetime.now().isoformat(),
                    'text': test_goal,
                    'source': 'User',
                    'target': character
                }
                self.user_action_publisher.put(json.dumps(user_action))
                
                logger.info(f"Executing goal {goal_file} for {character}")
                return {"success": True, "message": f"Executing {goal_file}"}
            except Exception as e:
                logger.error(f"Error executing goal: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/plan_bindings/{character}")
        async def get_plan_bindings(character: str):
            """Get current plan bindings for a character."""
            try:
                selector = f"cognitive/{character}/plan_bindings"
                replies = self.session.get(selector, timeout=2.0)
                
                for reply in replies:
                    if hasattr(reply, 'ok') and reply.ok is not None:
                        payload_bytes = reply.ok.payload.to_bytes()
                        response = json.loads(payload_bytes.decode('utf-8'))
                        if response.get('success'):
                            return {"success": True, "bindings": response.get('bindings', {})}
                
                return {"success": True, "bindings": {}}
            except Exception as e:
                logger.error(f"Error getting plan bindings: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/tasks/{character}")
        async def get_tasks(character: str):
            """Get active tasks for a character."""
            try:
                selector = f"cognitive/{character}/tasks"
                replies = self.session.get(selector, timeout=2.0)
                for reply in replies:
                    if hasattr(reply, 'ok') and reply.ok is not None:
                        payload_bytes = reply.ok.payload.to_bytes()
                        response = json.loads(payload_bytes.decode('utf-8'))
                        if response.get('success'):
                            return {"success": True, "tasks": response.get('tasks', [])}
                return {"success": True, "tasks": []}
            except Exception as e:
                logger.error(f"Error getting tasks: {e}")
                return {"success": False, "message": str(e)}

        @self.app.get("/api/scheduled_goals/{character}")
        async def get_scheduled_goals(character: str):
            """Get scheduled goals for a character."""
            try:
                selector = f"cognitive/{character}/scheduled_goals"
                replies = self.session.get(selector, timeout=2.0)
                for reply in replies:
                    if hasattr(reply, 'ok') and reply.ok is not None:
                        payload_bytes = reply.ok.payload.to_bytes()
                        response = json.loads(payload_bytes.decode('utf-8'))
                        if response.get('success'):
                            return {"success": True, "goals": response.get("goals", [])}
                return {"success": True, "goals": []}
            except Exception as e:
                logger.error(f"Error getting scheduled goals: {e}")
                return {"success": False, "message": str(e)}
        
        @self.app.get("/api/characters")
        async def get_characters():
            """Get list of active characters that have announced themselves."""
            return {
                "characters": list(self.active_characters),
                "scheduler_defaults": self.character_scheduler_defaults,
            }
        
        @self.app.get("/api/world_state/{character}")
        async def get_world_state(character: str):
            """Get current world state for a character."""
            try:
                selector = f"cognitive/{character}/world_state"
                replies = self.session.get(selector, timeout=2.0)
                
                for reply in replies:
                    if hasattr(reply, 'ok') and reply.ok is not None:
                        payload_bytes = reply.ok.payload.to_bytes()
                        response = json.loads(payload_bytes.decode('utf-8'))
                        if response.get('success'):
                            return {"success": True, "world_state": response.get('world_state', {})}
                
                return {"success": True, "world_state": {}}
            except Exception as e:
                logger.error(f"Error getting world state: {e}")
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
                
                # Check if trace has content
                if not self.tracer._trace_lines:
                    return {"success": False, "message": "No trace data available"}

                # Create target directory
                target_dir = vault_path / "Cognitive_Workbench_Traces"
                target_dir.mkdir(exist_ok=True)

                # Generate timestamped filename
                timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                target_filename = f"action_trace_{self.scenario_name}_{timestamp}.md"
                target_path = target_dir / target_filename

                # Write trace to target
                self.tracer.export_to_file(target_path)
                
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
                self.shutdown_in_progress = True
                
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
                self.shutdown_in_progress = False
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/character/{character_name}/inventory")
        async def get_character_inventory(character_name: str):
            """Get inventory list for a character."""
            try:
                if not hasattr(self, 'session'):
                    return {"success": False, "message": "Zenoh session not available", "inventory": []}
                
                # Inventory removed - return empty list
                query_key = f"cognitive/{character_name}/memory/inventory"
                
                try:
                    replies = self.session.get(query_key, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=5.0 if not self.debug else 300.0)
                    for reply in replies:
                        try:
                            if hasattr(reply, 'ok') and reply.ok is not None:
                                payload_bytes = reply.ok.payload.to_bytes()
                            elif hasattr(reply, 'payload'):
                                payload_bytes = reply.payload.to_bytes()
                            else:
                                continue
                            
                            response = json.loads(payload_bytes.decode('utf-8'))
                            
                            if response.get('success'):
                                inventory_list = response.get('value', [])
                                if not isinstance(inventory_list, list):
                                    inventory_list = [inventory_list] if inventory_list else []
                                return {
                                    "success": True,
                                    "inventory": inventory_list
                                }
                            else:
                                return {"success": False, "message": response.get('error', 'Unknown error'), "inventory": []}
                        except Exception as parse_error:
                            logger.warning(f"Failed to parse inventory reply for {character_name}: {parse_error}")
                            break
                    
                    # No valid response - return empty inventory
                    return {"success": True, "inventory": []}
                except Exception as e:
                    logger.error(f"Error querying inventory for {character_name}: {e}")
                    return {"success": False, "message": str(e), "inventory": []}
            except Exception as e:
                logger.error(f"Error in get_character_inventory for {character_name}: {e}")
                return {"success": False, "message": str(e), "inventory": []}
        
        @self.app.post("/api/shutdown")
        async def shutdown_only():
            """Request centralized shutdown via launcher (no save)."""
            try:
                if self.shutdown_in_progress:
                    return {"success": False, "message": "Shutdown already in progress"}
                self.shutdown_in_progress = True
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
                self.shutdown_in_progress = False
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
            """Get initial time information."""
            try:
                from datetime import datetime
                current_time = datetime.now()
                time_info = {
                    'year': current_time.year,
                    'month': current_time.month,
                    'day': current_time.day,
                    'hour': current_time.hour,
                    'minute': current_time.minute,
                    'datetime': current_time.isoformat(),
                    'period': 'morning' if 5 <= current_time.hour < 12 else 'afternoon' if 12 <= current_time.hour < 17 else 'evening' if 17 <= current_time.hour < 21 else 'night',
                    'season': 'spring' if 3 <= current_time.month <= 5 else 'summer' if 6 <= current_time.month <= 8 else 'autumn' if 9 <= current_time.month <= 11 else 'winter'
                }
                return {"success": True, "time_info": time_info}
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/resource/{resource_id}")
        async def get_resource_content(resource_id: str):
            """Get Note or Collection content by resource ID."""
            try:
                if not hasattr(self, 'session'):
                    return {"success": False, "message": "Zenoh session not available"}
                
                # Query executive_node for resource content
                query_key = f"cognitive/{self.last_character_name}/resource/view/{resource_id}" if self.last_character_name else f"cognitive/*/resource/view/{resource_id}"
                
                replies = self.session.get(query_key, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=5.0 if not self.debug else 300.0)
                for reply in replies:
                    if hasattr(reply, 'ok') and reply.ok is not None:
                        payload_bytes = reply.ok.payload.to_bytes()
                        response = json.loads(payload_bytes.decode('utf-8'))
                        
                        if response.get('success'):
                            return {
                                "success": True,
                                "resource_id": resource_id,
                                "type": response.get('type'),
                                "content": response.get('content'),
                                "metadata": response.get('metadata', {})
                            }
                        else:
                            return {"success": False, "message": response.get('error', 'Resource not found')}
                    break
                
                return {"success": False, "message": "No response from executive_node"}
                    
            except Exception as e:
                logger.error(f"Error fetching resource {resource_id}: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.get("/api/relation/{character_name}/{target_character}")
        async def get_relation_data(character_name: str, target_character: str):
            """Get relation data (discourse_state and tom_model) for a character pair."""
            try:
                if not hasattr(self, 'session'):
                    return {"success": False, "message": "Zenoh session not available", "no_interaction": False}
                
                # Query executive_node memory module for relation data
                query_key = f"cognitive/{character_name}/memory/entity/{target_character}?query=relation"
                
                try:
                    replies = self.session.get(query_key, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=5.0 if not self.debug else 300.0)
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
                            break
                    
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
        
        @self.app.get("/api/activities/{character_name}")
        async def get_activities(character_name: str):
            """Get list of available activities for a character."""
            try:
                if not hasattr(self, 'session'):
                    return {"success": False, "message": "Zenoh session not available"}
                
                query_key = f"cognitive/{character_name}/activity/list"
                
                replies = self.session.get(query_key, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=5.0 if not self.debug else 300.0)
                for reply in replies:
                    if hasattr(reply, 'ok') and reply.ok is not None:
                        payload_bytes = reply.ok.payload.to_bytes()
                        response = json.loads(payload_bytes.decode('utf-8'))
                        
                        if response.get('success'):
                            return {
                                "success": True,
                                "activities": response.get('activities', [])
                            }
                    break
                
                return {"success": False, "message": "No response from executive_node"}
                    
            except Exception as e:
                logger.error(f"Error fetching activities for {character_name}: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/activity/set")
        async def set_activity(request: Request):
            """Set a character's activity manually."""
            try:
                data = await request.json()
                character_name = data.get('character_name')
                activity_name = data.get('activity_name')
                
                if not character_name or not activity_name:
                    return {"success": False, "message": "Missing character_name or activity_name"}
                
                if not hasattr(self, 'session'):
                    return {"success": False, "message": "Zenoh session not available"}
                
                # Publish activity selection to executive_node
                topic = f"cognitive/{character_name}/activity/set"
                payload = json.dumps({"activity_name": activity_name})
                self.session.put(topic, payload)
                
                return {"success": True, "message": f"Activity {activity_name} set for {character_name}"}
                    
            except Exception as e:
                logger.error(f"Error setting activity: {e}")
                return {"success": False, "message": f"Error: {str(e)}"}

    
    def _get_html_template(self) -> str:
        """Get the HTML template for the web UI."""
        return r"""
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
            transition: height 0.3s ease;
        }
        .action-log.collapsed {
            height: 0;
            padding: 0;
            overflow: hidden;
            border: none;
        }
        .log-toggle {
            background: #2d2d2d;
            border: 1px solid #404040;
            border-radius: 8px 8px 0 0;
            padding: 8px 12px;
            cursor: pointer;
            user-select: none;
            color: #00d4ff;
            font-size: 13px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .log-toggle:hover {
            background: #333;
        }
        .log-toggle::after {
            content: '▼';
            transition: transform 0.3s ease;
        }
        .log-toggle.collapsed::after {
            transform: rotate(-90deg);
        }
        .response-panel {
            background: #2d2d2d;
            border: 1px solid #404040;
            border-radius: 8px;
            padding: 10px;
            margin-bottom: 10px;
        }
        .response-panel h3 {
            margin: 0 0 8px 0;
            color: #00d4ff;
            font-size: 14px;
        }
        .response-meta {
            color: #888;
            font-size: 11px;
            margin-bottom: 8px;
        }
        .response-content {
            background: #1a1a1a;
            border: 1px solid #333;
            border-radius: 6px;
            padding: 10px;
            color: #e0e0e0;
            font-family: 'Consolas', 'Monaco', 'Courier New', monospace;
            font-size: 13px;
            line-height: 1.45;
            white-space: pre-wrap;
            min-height: 60px;
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
        #messageInput {
            transition: height 0.3s ease;
        }
        #messageInput:focus {
            height: 200px !important;
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
            cursor: pointer;
        }
        .action-entry:hover {
            background: #202020;
        }
        .action-entry.selected {
            border-left-color: #ffe66d;
            background: #252525;
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
        
        /* Modal Styles */
        .modal {
            display: none;
            position: fixed;
            z-index: 1000;
            left: 0;
            top: 0;
            width: 100%;
            height: 100%;
            overflow: auto;
            background-color: rgba(0,0,0,0.7);
        }
        
        .modal-content {
            background-color: #2b2b2b;
            margin: 5% auto;
            padding: 0;
            border: 1px solid #555;
            width: 80%;
            min-width: 600px;
            max-width: 95%;
            max-height: 85vh;
            border-radius: 8px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.5);
            position: relative;
            overflow: auto;
        }
        
        .modal-resize-handle {
            position: absolute;
            bottom: 0;
            right: 0;
            width: 20px;
            height: 20px;
            cursor: nwse-resize;
            background: linear-gradient(135deg, transparent 0%, transparent 40%, #555 40%, #555 45%, transparent 45%, transparent 55%, #555 55%, #555 60%, transparent 60%);
            border-radius: 0 0 8px 0;
            z-index: 1001;
        }
        
        .modal-resize-handle:hover {
            background: linear-gradient(135deg, transparent 0%, transparent 40%, #777 40%, #777 45%, transparent 45%, transparent 55%, #777 55%, #777 60%, transparent 60%);
        }
        
        .modal-header {
            padding: 15px 20px;
            background-color: #1e1e1e;
            border-bottom: 1px solid #555;
            border-radius: 8px 8px 0 0;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .modal-header h2 {
            margin: 0;
            color: #4ecdc4;
            font-size: 1.2em;
        }
        
        .modal-close {
            color: #aaa;
            font-size: 28px;
            font-weight: bold;
            cursor: pointer;
            transition: color 0.2s;
        }
        
        .modal-close:hover,
        .modal-close:focus {
            color: #fff;
        }
        
        .modal-body {
            padding: 20px;
            max-height: 70vh;
            overflow-y: auto;
        }
        
        .modal-body pre {
            background-color: #1e1e1e;
            color: #e0e0e0;
            padding: 15px;
            border-radius: 4px;
            white-space: pre-wrap;
            word-wrap: break-word;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            line-height: 1.5;
            margin: 0;
        }
        
        /* Resource Link Styles */
        .resource-link {
            color: #4a9eff;
            text-decoration: none;
            cursor: pointer;
            border-bottom: 1px dotted #4a9eff;
        }
        
        .resource-link:hover {
            color: #6ab7ff;
            border-bottom: 1px solid #6ab7ff;
        }
        
        /* Activities list styles */
        .activities-list {
            display: flex;
            flex-direction: column;
            gap: 4px;
        }
        
        .activity-item {
            padding: 6px 8px;
            background: #1a1a1a;
            border: 1px solid #404040;
            border-radius: 4px;
            cursor: pointer;
            transition: all 0.2s;
            font-size: 12px;
            color: #e0e0e0;
        }
        
        .activity-item:hover {
            background: #2a2a2a;
            border-color: #00d4ff;
            color: #00d4ff;
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
                    <div class="character-data-tab active" data-tab="plan">Plan</div>
                    <div class="character-data-tab" data-tab="bindings">Bindings</div>
                    <div class="character-data-tab" data-tab="goals">Goals</div>
                    <div class="character-data-tab" data-tab="plans">Plans</div>
                    <div class="character-data-tab" data-tab="state">State</div>
                    <div class="character-data-tab" data-tab="tasks">Schedule</div>
                </div>
                
                <!-- Character data content -->
                <div class="character-data-content">
                    <!-- Plan tab content -->
                    <div class="character-data-panel active" id="planPanel">
                        <div id="characterDataItems">
                            <div style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                                Select a character to view data
                            </div>
                        </div>
                    </div>
                    
                    <!-- Bindings tab content -->
                    <div class="character-data-panel" id="bindingsPanel">
                        <div id="bindingsList">
                            <div style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                                No bindings available.
                            </div>
                        </div>
                    </div>
                    
                    <!-- Goals tab content -->
                    <div class="character-data-panel" id="goalsPanel">
                        <div id="goalsList">
                            <div style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                                Loading test goals...
                            </div>
                        </div>
                    </div>
                    
                    <!-- Plans tab content -->
                    <div class="character-data-panel" id="plansPanel">
                        <div id="savedPlansList">
                            <div style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                                Loading saved plans...
                            </div>
                        </div>
                    </div>
                    
                    <!-- State tab content -->
                    <div class="character-data-panel" id="statePanel">
                        <div id="worldStateDisplay">
                            <div style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                                Select a character to view world state
                            </div>
                        </div>
                    </div>
                    
                    <!-- Tasks tab content -->
                    <div class="character-data-panel" id="tasksPanel">
                        <div id="taskSchedulerBar" style="padding: 6px 10px; border-bottom: 1px solid #404040; display: flex; align-items: center; gap: 8px; font-size: 11px;">
                            <label style="display: flex; align-items: center; gap: 4px; cursor: pointer;">
                                <input type="checkbox" id="schedulerToggle" onchange="toggleScheduler(this.checked)">
                                Auto-proceed
                            </label>
                            <span id="schedulerStatus" style="color: #888; font-size: 10px;">disabled</span>
                            <label style="margin-left: auto; display: flex; align-items: center; gap: 3px;">
                                Interval:
                                <input type="number" id="schedulerInterval" value="1" min="1" max="1440"
                                       style="width: 45px; background: #333; color: #ccc; border: 1px solid #555; font-size: 10px; padding: 2px;"
                                       onblur="updateSchedulerInterval(this.value)">m
                            </label>
                        </div>
                        <div id="schedulerEvents" style="max-height: 130px; overflow-y: auto; border-bottom: 1px solid #404040; padding: 6px 10px; font-size: 11px; color: #bbb;">
                            <div style="color: #777; font-style: italic;">No scheduler events yet.</div>
                        </div>
                        <div id="tasksList">
                            <div style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                                Select a character to view tasks
                            </div>
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

                <div class="response-panel">
                    <h3>Agent Response</h3>
                    <div class="response-meta" id="responseMeta">Click an action entry to view full content.</div>
                    <div class="response-content" id="responseContent">(no response selected)</div>
                </div>
                
                <div class="log-toggle" id="logToggle" onclick="toggleLog()">
                    <span>Action Log</span>
                </div>
                <div class="action-log" id="actionLog">
                    <div style="color: #888; font-style: italic;">Waiting for actions...</div>
                </div>
                
                <div class="input-section">
                    <h3>Turn Control</h3>
                    <div style="margin-bottom: 15px;">
                        <button id="autonomousButton" onclick="autonomousTurns()" style="background: #555; margin-right: 10px;" disabled title="Start autonomous execution">Autonomous</button>
                        <button onclick="stopTurns()" style="background: #ff6b6b; margin-right: 10px;" title="Pause execution">Stop</button>
                        <button onclick="openControlPanel()" style="background: #6c5ce7; color: white; margin-right: 10px;" title="Open control panel">⚙️ Control</button>
                        <button onclick="saveAll()" style="background: #95e1d3; color: #1a1a1a; margin-right: 10px;" title="Save all resources and memory to disk">Save</button>
                        <button onclick="exportToObsidian()" style="background: #7c3aed; color: white; margin-right: 10px;" title="Export action log to Obsidian vault">Obsidian</button>
                        <button onclick="openResourceBrowser()" style="background: #0e639c; color: white; margin-right: 10px;" title="Open resource browser in new tab to view Notes and Collections">🔍 Browser</button>
                        <button onclick="shutdownWithSave()" style="background: #ff4757; color: white;" title="Save all data and shutdown the system">Shutdown</button>
                    </div>
                    <div style="margin-bottom: 15px; padding: 10px; background: #333; border-radius: 5px;">
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
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px;">
                        <h3 style="margin: 0;">Send Text Input</h3>
                        <div style="display: flex; gap: 10px; align-items: center;">
                            <label style="display: flex; align-items: center; gap: 5px; font-size: 0.9em;">
                                <input type="radio" name="executionMode" value="turn" checked id="modeTurn">
                                Turn-based
                            </label>
                            <label style="display: flex; align-items: center; gap: 5px; font-size: 0.9em;">
                                <input type="radio" name="executionMode" value="sync" id="modeSync">
                                Sync
                            </label>
                            <button onclick="sendText()" style="background-color: #5fb85f;">Submit</button>
                            <button id="endConversationButton" onclick="endConversation()" style="background-color: #555; color: #ccc;" title="End conversation (or type goodbye, bye, etc.)">End</button>
                        </div>
                    </div>
                    <div id="activeConversation" style="color: #4ecdc4; font-size: 0.9em; margin-bottom: 10px; display: none;">
                        Active conversation with: <span id="activeConversationPartner"></span>
                    </div>
                    <input type="text" id="characterInput" placeholder="Character name" style="width: 200px; margin-bottom: 8px; display: none;">
                    <textarea id="messageInput" placeholder="Message or Plan (multi-line supported)" style="width: 100%; height: 80px; resize: vertical; background-color: #2b2b2b; color: #ffffff; border: 1px solid #555; padding: 8px; font-family: monospace; box-sizing: border-box;"></textarea>
                    <div id="askIndicator" style="display: none; color: #f39c12; padding: 8px; margin-top: 8px; background: #2a2a2a; border-radius: 4px; border: 1px solid #f39c12;">
                        ❓ <strong>Question:</strong> <span id="askQuestion"></span>
                        <div style="font-size: 11px; color: #888; margin-top: 4px;">Type your response above and click Send or press Step to continue</div>
                    </div>
                    <div id="sendResult" style="margin-top: 5px;"></div>
                </div>
            </div>
        </div>
    </div>

    <!-- Display Modal for formatted content -->
    <div id="displayModal" class="modal">
        <div class="modal-content" id="modalContentDiv">
            <div class="modal-header">
                <h2 id="modalTitle">Content Display</h2>
                <span class="modal-close" onclick="closeDisplayModal()">&times;</span>
            </div>
            <div class="modal-body">
                <pre id="modalContent"></pre>
            </div>
            <div class="modal-resize-handle" id="modalResizeHandle"></div>
        </div>
    </div>

    <!-- Save Plan Modal -->
    <div id="savePlanModal" class="modal">
        <div class="modal-content" style="max-width: 600px;">
            <div class="modal-header">
                <h2>💾 Save Plan as Tool</h2>
                <span class="modal-close" onclick="closeSavePlanModal()">&times;</span>
            </div>
            <div class="modal-body">
                <div style="margin-bottom: 15px;">
                    <label style="display: block; margin-bottom: 5px; color: #ccc; font-weight: bold;">Tool Name:</label>
                    <input type="text" id="toolNameInput" placeholder="e.g., research-papers-workflow" 
                           style="width: 100%; padding: 8px; background: #1a1a1a; color: #e0e0e0; border: 1px solid #555; border-radius: 4px; font-family: 'Courier New', monospace;">
                    <div style="font-size: 11px; color: #888; margin-top: 4px;">Use lowercase kebab-case (e.g., 'create-collection', 'search-papers')</div>
                </div>
                <div style="margin-bottom: 15px;">
                    <label style="display: block; margin-bottom: 5px; color: #ccc; font-weight: bold;">Goal/Description:</label>
                    <textarea id="toolGoalInput" rows="3" 
                              style="width: 100%; padding: 8px; background: #1a1a1a; color: #e0e0e0; border: 1px solid #555; border-radius: 4px; resize: vertical; font-family: 'Segoe UI', sans-serif;"></textarea>
                </div>
                <div style="margin-bottom: 15px;">
                    <label style="display: block; margin-bottom: 5px; color: #ccc; font-weight: bold;">Parameters (detected):</label>
                    <div id="toolParametersDisplay" style="padding: 8px; background: #1a1a1a; border: 1px solid #555; border-radius: 4px; min-height: 40px; max-height: 200px; overflow-y: auto;">
                        <div style="color: #888; font-style: italic;">No parameters detected</div>
                    </div>
                </div>
                <div style="margin-bottom: 15px;">
                    <label style="display: flex; align-items: center; color: #ccc; cursor: pointer;">
                        <input type="checkbox" id="overwriteCheckbox" style="margin-right: 8px; cursor: pointer;">
                        <span>Overwrite if tool already exists</span>
                    </label>
                </div>
                <div style="display: flex; gap: 10px;">
                    <button onclick="closeSavePlanModal()" style="flex: 1; background: #555; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; font-weight: bold;">
                        Cancel
                    </button>
                    <button onclick="executeSavePlan()" style="flex: 1; background: #00d4ff; color: #1a1a1a; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; font-weight: bold;">
                        💾 Save Tool
                    </button>
                </div>
                <div id="savePlanStatus" style="margin-top: 15px; padding: 10px; background: #2d2d2d; border-radius: 4px; min-height: 40px; display: none;">
                    <div style="color: #888;">Status will appear here...</div>
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
        let lastSelectedTestCharacter = null; // Sticky character selection for test runner
        
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
                
                // Query backend for any characters that may have been announced while tab was inactive
                syncCharactersFromBackend();
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
                
                // Handle display modal actions
                if (data.type === 'action' && data.is_display_modal) {
                    console.log('📺 Display modal action received:', data);
                    showDisplayModal(data);
                    return;
                }
                
                if (data.type === 'action') {
                    addActionEntry(data);
                    // Check if this is an announcement to create a character tab
                    if (data.action_type === 'announcement') {
                        createCharacterTab(data.character);
                        try {
                            const cfg = (data.raw_data && data.raw_data.character_config && data.raw_data.character_config.task_scheduler) || null;
                            if (cfg) {
                                const tabData = characterTabs.get(data.character);
                                if (tabData) {
                                    tabData.schedulerState = {
                                        enabled: !!cfg.enabled,
                                        interval: Math.max(60, parseInt((parseFloat(cfg.interval) || 15) * 60))
                                    };
                                }
                            }
                        } catch (e) {}
                        try { announcedCharacters.add(data.character); } catch (e) {}
                        // Auto-fill character input with the first (and only) character
                        const characterInput = document.getElementById('characterInput');
                        if (characterInput && !characterInput.value) {
                            characterInput.value = data.character;
                        }
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
                                    const autonomousButton = document.getElementById('autonomousButton');
                                    const testButton = document.getElementById('testButton');
                                    if (currentTurnMode !== 'run' && currentTurnMode !== 'autonomous') {
                                        if (autonomousButton) {
                                            autonomousButton.disabled = false;
                                            autonomousButton.style.background = '#4ecdc4';
                                            autonomousButton.title = 'Click to start autonomous execution';
                                        }
                                        if (testButton) {
                                            testButton.disabled = false;
                                            testButton.style.background = '#f39c12';
                                            testButton.style.color = 'white';
                                            testButton.title = 'Open test runner to run evaluation tests';
                                        }
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
                } else if (data.type === 'world_state_update') {
                    // Handle world state update
                    updateWorldStateDisplay(data);
                } else if (data.type === 'test_files_list') {
                    updateTestFileList(data.files);
                } else if (data.type === 'test_details') {
                    showTestDetails(data.details);
                } else if (data.type === 'test_status') {
                    updateTestStatus(data.status);
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
            const autonomousButton = document.getElementById('autonomousButton');
            const testButton = document.getElementById('testButton');
            
            if (autonomousButton) {
                autonomousButton.disabled = true;
                autonomousButton.style.background = '#555';
                autonomousButton.title = 'Waiting for system startup...';
            }
            
            const continuousButton = document.getElementById('continuousButton');
            if (continuousButton) {
                continuousButton.disabled = false;  // Always enabled (it's a toggle)
                continuousButton.style.background = '#555';
                continuousButton.style.color = '#888';
                continuousButton.title = 'Toggle continuous mode - goal will resubmit on completion';
            }
            
            if (testButton) {
                testButton.disabled = true;
                testButton.style.background = '#555';
                testButton.style.color = '#888';
                testButton.title = 'Waiting for system startup...';
            }
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
            
            if (tabName === 'plan') {
                document.getElementById('planPanel').classList.add('active');
                // Refresh plan display for current character
                if (activeCharacter) {
                    updateCharacterDataDisplay(activeCharacter);
                }
            } else if (tabName === 'bindings') {
                document.getElementById('bindingsPanel').classList.add('active');
                // Load bindings list
                const character = activeCharacter || 'Jill';
                loadBindingsList(character);
            } else if (tabName === 'goals') {
                document.getElementById('goalsPanel').classList.add('active');
                // Load test goals list
                loadGoalsList();
            } else if (tabName === 'plans') {
                document.getElementById('plansPanel').classList.add('active');
                // Load saved plans list
                loadSavedPlansList();
            } else if (tabName === 'state') {
                document.getElementById('statePanel').classList.add('active');
                // Load world state
                const character = activeCharacter || 'Jill';
                loadWorldState(character);
            } else if (tabName === 'tasks') {
                document.getElementById('tasksPanel').classList.add('active');
                const character = activeCharacter || 'Jill';
                loadTasksList(character);
            }
        }
        
        // Sync characters from backend (handles missed announcements when tab was inactive)
        async function syncCharactersFromBackend() {
            try {
                const response = await fetch('/api/characters');
                if (!response.ok) return;
                const data = await response.json();
                if (data.characters && Array.isArray(data.characters)) {
                    data.characters.forEach(function(character) {
                        if (!characterTabs.has(character)) {
                            createCharacterTab(character);
                            announcedCharacters.add(character);
                        }
                        const defaults = data.scheduler_defaults && data.scheduler_defaults[character];
                        if (defaults) {
                            const tabData = characterTabs.get(character);
                            if (tabData && !tabData.schedulerState) {
                                tabData.schedulerState = defaults;
                            }
                        }
                    });
                    // Auto-select first character if none selected
                    if (!activeCharacter && data.characters.length > 0) {
                        selectCharacterTab(data.characters[0]);
                    }
                }
            } catch (e) {
                console.log('Failed to sync characters from backend:', e);
            }
        }
        
        // Listen for page visibility changes (tab becomes active)
        document.addEventListener('visibilitychange', function() {
            if (!document.hidden) {
                syncCharactersFromBackend();
            }
        });
        
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
            updateCharacterDataDisplay(characterName);

            // Sync scheduler UI to this character's cached state (or reset to defaults)
            const tabData = characterTabs.get(characterName);
            updateSchedulerUI(tabData?.schedulerState || { enabled: false, interval: 60 });

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
            // Activity tabs removed - this callback is now unused
            const characterName = currentActivityData.character;
            console.log(`Current activity update for ${characterName} (ignored - activity tabs removed)`);
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
                const escapedCharName = characterName.replace(/'/g, "\\'");
                content += `
                    <div class="character-data-item">
                        <div class="character-data-label">Current Plan
                            <button onclick="showSavePlanModal('${escapedCharName}')" style="float: right; padding: 2px 8px; background: #00d4ff; color: #1a1a1a; border: none; border-radius: 3px; cursor: pointer; font-size: 11px; font-weight: bold;">💾 Save as Tool</button>
                        </div>
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
        

        function escapeHtml(text) {
            const div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }
        
        // Helper function to make resource IDs clickable in text
        function makeResourceIdsClickable(text) {
            if (!text || typeof text !== 'string') return text;
            // Match Note_ or Collection_ followed by word characters (digits, letters, underscore)
            // Use non-word-boundary pattern to catch Note_15, Collection_4, Note_null, etc.
            return text.replace(/(Note_|Collection_)\w+/g, function(match) {
                return `<a href="#" class="resource-link" data-resource-id="${match}">${match}</a>`;
            });
        }

        function truncateForLog(text, maxLen = 220) {
            if (!text) return '';
            const normalized = String(text).replace(/\s+/g, ' ').trim();
            if (normalized.length <= maxLen) return normalized;
            return `${normalized.slice(0, maxLen)}...`;
        }

        function extractFullResponseText(actionData) {
            if (!actionData) return '';
            if (actionData.is_text_only) {
                return String(actionData.text || actionData.input_text || actionData.llm_response || '');
            }
            if (actionData.llm_response) return String(actionData.llm_response);
            if (actionData.result !== undefined && actionData.result !== null && actionData.result !== '') {
                return typeof actionData.result === 'object' ? JSON.stringify(actionData.result, null, 2) : String(actionData.result);
            }
            if (actionData.value !== undefined && actionData.value !== null && actionData.value !== '') {
                return typeof actionData.value === 'object' ? JSON.stringify(actionData.value, null, 2) : String(actionData.value);
            }
            if (actionData.input_text) return String(actionData.input_text);
            return '';
        }

        function setSelectedResponse(entryElement, actionData, fullText) {
            document.querySelectorAll('.action-entry.selected').forEach(el => el.classList.remove('selected'));
            entryElement.classList.add('selected');
            const responseMeta = document.getElementById('responseMeta');
            const responseContent = document.getElementById('responseContent');
            const type = (actionData.action_type || 'action').toUpperCase();
            const character = (actionData.character || 'unknown').toUpperCase();
            responseMeta.textContent = `[${character}] ${type}`;
            responseContent.textContent = fullText || '(no response content)';
        }
        
        function addActionEntry(actionData) {
            console.log('Adding action entry:', actionData);
            
            const typeLower = (actionData.action_type || '').toLowerCase();
            
            // Handle ask action type - show indicator only when targeting User
            if (typeLower === 'ask' && (!actionData.target || actionData.target === 'User')) {
                const questionText = actionData.text || actionData.value || 'Awaiting your response...';
                document.getElementById('askQuestion').textContent = questionText;
                document.getElementById('askIndicator').style.display = 'block';
                // Clear message input to make it obvious user should type response
                document.getElementById('messageInput').value = '';
                // Also add to action log for completeness
            }
            
            // Handle response action type - hide ask indicator
            if (typeLower === 'response') {
                document.getElementById('askIndicator').style.display = 'none';
            }
            
            const entry = document.createElement('div');
            entry.className = 'action-entry';
            
            // Prefer per-action timestamp; fallback to simulation time, then local clock
            const timestamp = actionData.timestamp ? new Date(actionData.timestamp).toLocaleTimeString() : (latestSimTimeISO ? new Date(latestSimTimeISO).toLocaleTimeString() : new Date().toLocaleTimeString());
            let html = `<span class="timestamp">[${timestamp}]</span> `;
            html += `<span class="character-name">[${actionData.character.toUpperCase()}]</span> `;
            let actorLabel = '';
            if (actionData.is_text_only) {
                if (typeLower === 'say' && actionData.target) {
                    actorLabel = ` ${makeResourceIdsClickable(actionData.target)}:`;
                } else if (typeLower === 'ask' && actionData.target) {
                    actorLabel = ` ${makeResourceIdsClickable(actionData.target)}:`;
                } else if (typeLower === 'response' && actionData.source) {
                    actorLabel = ` ${makeResourceIdsClickable(actionData.source)}:`;
                }
            } else {
                if (typeLower === 'take' && actionData.target) {
                    actorLabel = ` ${makeResourceIdsClickable(actionData.target)}`;
                } else if (typeLower === 'scan' && actionData.target) {
                    actorLabel = ` ${makeResourceIdsClickable(actionData.target)}`;
                }
            }
            html += `<span class="action-type">${actionData.action_type}</span>${actorLabel}`;
            
            // For text-only actions (Say/response), only show the text
            if (actionData.is_text_only) {
                const textContent = actionData.text || actionData.input_text || actionData.llm_response || '';
                if (textContent) {
                    html += `<br><span class="response-text">"${makeResourceIdsClickable(truncateForLog(textContent))}"</span>`;
                }
            } else {
                // Display action details if available
                if (actionData.action || actionData.target || actionData.value || actionData.requested_target || actionData.result || actionData.error || actionData.status) {
                    let actionDetails = [];
                    if (actionData.action) {
                        // Handle objects properly - serialize if needed
                        const actionValue = typeof actionData.action === 'object' ? JSON.stringify(actionData.action) : actionData.action;
                        actionDetails.push(`Action: ${actionValue}`);
                    }
                    // Prefer resolved target if present, else show requested target, else target
                    const targetLabel = actionData.resolved_target || actionData.target || actionData.requested_target;
                    if (targetLabel) {
                        let targetDisplay = String(targetLabel);
                        // If target is a variable ($var) but we have a resolved_target that's a resource ID, make variable clickable
                        if (actionData.target && actionData.target.startsWith('$') && actionData.resolved_target) {
                            const resolvedValue = String(actionData.resolved_target);
                            // Check if resolved value is a resource ID
                            if (/^(Note_|Collection_)\w+/.test(resolvedValue)) {
                                // Make the variable clickable to the resolved resource
                                targetDisplay = `<a href="#" class="resource-link" data-resource-id="${resolvedValue}">${actionData.target}</a>`;
                            } else {
                                targetDisplay = makeResourceIdsClickable(targetDisplay);
                            }
                        } else {
                            targetDisplay = makeResourceIdsClickable(targetDisplay);
                        }
                        actionDetails.push(`Target: ${targetDisplay}`);
                    }
                    if (actionData.requested_target && (!actionData.resolved_target && !actionData.target)) {
                        const clickableRequested = makeResourceIdsClickable(String(actionData.requested_target));
                        actionDetails.push(`Requested: ${clickableRequested}`);
                    }
                    // Display result field for all actions (not just scan)
                    // Check result field first, then value field as fallback
                    // Also display resource_id if present (from uniform format)
                    if (actionData.result !== undefined && actionData.result !== null && actionData.result !== '') {
                        // Handle objects properly - serialize if needed, otherwise convert to string
                        const resultStr = typeof actionData.result === 'object' ? JSON.stringify(actionData.result) : String(actionData.result);
                        const clickableResult = makeResourceIdsClickable(resultStr);
                        if (typeLower === 'scan') {
                            actionDetails.push(`Found: ${clickableResult}`);
                        } else if (typeLower === 'createnote') {
                            actionDetails.push(`Created: ${clickableResult}`);
                        } else if (typeLower === 'createcollection') {
                            actionDetails.push(`Created: ${clickableResult}`);
                        } else {
                            actionDetails.push(`result: ${clickableResult}`);
                        }
                    } else if (actionData.value) {
                        // Fallback to value field if result is not set
                        // Handle objects properly - serialize if needed, otherwise convert to string
                        const valueStr = typeof actionData.value === 'object' ? JSON.stringify(actionData.value) : String(actionData.value);
                        const clickableValue = makeResourceIdsClickable(valueStr);
                        if (typeLower === 'createnote') {
                            actionDetails.push(`Created: ${clickableValue}`);
                        } else if (typeLower === 'createcollection') {
                            actionDetails.push(`Created: ${clickableValue}`);
                        } else {
                            actionDetails.push(`Value: ${clickableValue}`);
                        }
                    }
                    // Display resource_id if present (from uniform format)
                    if (actionData.resource_id) {
                        const clickableResourceId = makeResourceIdsClickable(String(actionData.resource_id));
                        actionDetails.push(`Resource: ${clickableResourceId}`);
                    }
                    // Add scan-specific details
                    if (typeLower === 'scan') {
                        if (actionData.out) actionDetails.push(`Variable: ${actionData.out}`);
                        if (actionData.variable_bound && actionData.bound_value) {
                            const clickableBound = makeResourceIdsClickable(String(actionData.bound_value));
                            actionDetails.push(`Bound: ${actionData.variable_bound} = ${clickableBound}`);
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
                    const clickableInput = makeResourceIdsClickable(String(actionData.input_text));
                    html += `<br><span class="input-text">Input: "${clickableInput}"</span>`;
                }
                
                // Display LLM response if available
                if (actionData.llm_response) {
                    const clickableResponse = makeResourceIdsClickable(truncateForLog(String(actionData.llm_response)));
                    html += `<br><span class="response-text">Response: ${clickableResponse}</span>`;
                }
                
                // Display any additional fields from raw_data for future flexibility
                if (actionData.raw_data) {
                    const additionalFields = [];
                    for (const [key, value] of Object.entries(actionData.raw_data)) {
                        // Skip fields we've already displayed
                        if (!['action', 'target', 'value', 'input_text', 'llm_response', 'action_type', 'action_id', 'timestamp', 'confidence'].includes(key)) {
                            if (value && value !== '') {
                                const clickableValue = makeResourceIdsClickable(String(value));
                                additionalFields.push(`${key}: ${clickableValue}`);
                            }
                        }
                    }
                    if (additionalFields.length > 0) {
                        html += `<br><span class="additional-fields">${additionalFields.join(' | ')}</span>`;
                    }
                }
            }
            
            entry.innerHTML = html;
            const fullResponseText = extractFullResponseText(actionData);
            entry.addEventListener('click', function() {
                setSelectedResponse(entry, actionData, fullResponseText);
            });
            const actionLog = document.getElementById('actionLog');
            actionLog.appendChild(entry);
            actionLog.scrollTop = actionLog.scrollHeight;
            // Auto-focus full response panel for user-facing text actions.
            if (actionData.is_text_only && fullResponseText) {
                setSelectedResponse(entry, actionData, fullResponseText);
            }
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
                       `autonomous_enabled=${stateData.buttons.autonomous ? stateData.buttons.autonomous.enabled : 'N/A'}, ` +
                       `progress=${stateData.turn.completed_count}/${stateData.turn.active_count}`);
            
            // Store state for control panel access
            window.lastTurnState = stateData;
            
            // Clear command lock when backend sends state update
            commandInProgress = false;
            
            // Apply button states directly from backend computation
            const autonomousButton = document.getElementById('autonomousButton');
            const continuousButton = document.getElementById('continuousButton');
            const stopButton = document.getElementById('stopButton');
            
            if (autonomousButton) {
                const shouldBeEnabled = stateData.buttons.autonomous ? stateData.buttons.autonomous.enabled : true;
                autonomousButton.disabled = !shouldBeEnabled;
                autonomousButton.style.background = shouldBeEnabled ? '#4ecdc4' : '#555';
                autonomousButton.title = stateData.buttons.autonomous ? stateData.buttons.autonomous.tooltip : 'Click to start autonomous execution';
                console.log(`🔘 Autonomous button: enabled=${shouldBeEnabled}, disabled=${autonomousButton.disabled}, background=${autonomousButton.style.background}, commandInProgress=${commandInProgress}`);
            }
            
            if (continuousButton && stateData.buttons.continuous) {
                const isActive = stateData.buttons.continuous.active || false;
                continuousButton.disabled = !stateData.buttons.continuous.enabled;
                continuousButton.textContent = stateData.buttons.continuous.label;
                continuousButton.style.background = isActive ? '#4ecdc4' : '#555';
                continuousButton.style.color = isActive ? '#1a1a1a' : '#888';
                continuousButton.title = stateData.buttons.continuous.tooltip;
            }
            
            // Always update control panel button if continuous state exists
            if (stateData.buttons.continuous) {
                updateControlPanelContinuousButton();
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
            
            // End button: red when dialog active (User or agent can end), grey when no active dialog
            const endBtn = document.getElementById('endConversationButton');
            if (endBtn) {
                const hasActiveDialog = stateData.active_dialog === true;
                endBtn.style.background = hasActiveDialog ? '#c0392b' : '#555';
                endBtn.style.color = hasActiveDialog ? '#fff' : '#ccc';
                endBtn.title = hasActiveDialog ? 'End conversation (click to close)' : 'End conversation (or type goodbye, bye, etc.)';
            }
            
            console.log(`✅ Applied button states: autonomous=${stateData.buttons.autonomous ? stateData.buttons.autonomous.enabled : 'N/A'}, ` +
                       `stop=${stateData.buttons.stop.enabled}, active_dialog=${stateData.active_dialog}`);

            // Cache scheduler state per character and update UI if it's the active character
            if (stateData.task_scheduler && stateData.character) {
                const tabData = characterTabs.get(stateData.character);
                if (tabData) tabData.schedulerState = stateData.task_scheduler;
                if (stateData.character === activeCharacter) {
                    updateSchedulerUI(stateData.task_scheduler);
                }
            }
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

                // Sync characters from readyData.characters_active if available
                if (Array.isArray(readyData.characters_active) && readyData.characters_active.length > 0) {
                    readyData.characters_active.forEach(function(character) {
                        if (!characterTabs.has(character)) {
                            createCharacterTab(character);
                            announcedCharacters.add(character);
                        }
                    });
                }

                // Auto-select first character if none selected yet
                try {
                    const expected = Array.isArray(readyData.characters_active) ? readyData.characters_active.length : (readyData.character_count || 0);
                    const allAnnounced = (expected > 0) && (announcedCharacters.size >= expected);
                    if (!activeCharacter && allAnnounced && Array.isArray(readyData.characters_active) && readyData.characters_active.length > 0) {
                        selectCharacterTab(readyData.characters_active[0]);
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
            let character = activeCharacter || document.getElementById('characterInput').value;
            // If no character specified, use the first (and only) character from characterTabs
            if (!character && characterTabs.size > 0) {
                character = Array.from(characterTabs.keys())[0];
            }
            const message = document.getElementById('messageInput').value;
            const resultDiv = document.getElementById('sendResult');
            const mode = document.querySelector('input[name="executionMode"]:checked').value;
            
            if (!message) {
                resultDiv.innerHTML = '<span class="error">Message is required</span>';
                return;
            }
            
            if (!character) {
                resultDiv.innerHTML = '<span class="error">No character available</span>';
                return;
            }
            
            // Check if sync mode and message looks like JSON plan
            if (mode === 'sync') {
                try {
                    // Strip 'plan:' prefix if present
                    let jsonText = message.trim();
                    if (jsonText.toLowerCase().startsWith('plan:')) {
                        jsonText = jsonText.substring(5).trim();
                    }
                    
                    // Parse JSON
                    const parsed = JSON.parse(jsonText);
                    
                    // Extract plan structure (handle both {"plan": [...]} and [...] formats)
                    let planStructure = null;
                    if (parsed && typeof parsed === 'object') {
                        if (Array.isArray(parsed)) {
                            // Array format: [...]
                            planStructure = parsed;
                        } else if (parsed.plan) {
                            // Object with plan key: {"plan": [...]}
                            planStructure = parsed.plan;
                        }
                    }
                    
                    if (planStructure && Array.isArray(planStructure)) {
                        // Execute sync plan with clean structure
                        const response = await fetch('/api/execute_plan_sync', {
                            method: 'POST',
                            headers: {
                                'Content-Type': 'application/json',
                            },
                            body: JSON.stringify({
                                character: character,
                                plan: planStructure
                            })
                        });
                        
                        const result = await response.json();
                        
                        if (result.success) {
                            let resultMsg = `Plan executed: ${result.executed_steps} steps completed`;
                            // Display last action result if available (uniform format)
                            if (result.last_action_result) {
                                const lastResult = result.last_action_result;
                                if (lastResult.value) {
                                    resultMsg += ` | Last: ${lastResult.value}`;
                                }
                                if (lastResult.resource_id) {
                                    resultMsg += ` | Resource: ${lastResult.resource_id}`;
                                }
                            }
                            resultDiv.innerHTML = `<span class="success">${resultMsg}</span>`;
                            document.getElementById('messageInput').value = '';
                        } else {
                            let errorMsg = `Plan execution failed: ${result.error || result.reason}`;
                            // Display last action result if available even on failure
                            if (result.last_action_result && result.last_action_result.value) {
                                errorMsg += ` | Last result: ${result.last_action_result.value}`;
                            }
                            resultDiv.innerHTML = `<span class="error">${errorMsg}</span>`;
                        }
                        return;
                    }
                } catch (e) {
                    // Not valid JSON, fall through to regular text input
                }
            }
            
            // Regular text input (turn-based or non-plan sync)
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
                    // Hide ask indicator when user responds
                    document.getElementById('askIndicator').style.display = 'none';
                    // Update conversation indicator
                    updateConversationIndicator();
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.error}</span>`;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
            }
        }

        async function endConversation() {
            let character = activeCharacter || document.getElementById('characterInput').value;
            if (!character && characterTabs.size > 0) {
                character = Array.from(characterTabs.keys())[0];
            }
            if (!character) {
                document.getElementById('sendResult').innerHTML = '<span class="error">No character selected</span>';
                return;
            }
            try {
                const response = await fetch('/api/end_dialog', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ character: character })
                });
                const result = await response.json();
                if (result.success) {
                    document.getElementById('sendResult').innerHTML = `<span class="success">${result.message}</span>`;
                    updateConversationIndicator();
                } else {
                    document.getElementById('sendResult').innerHTML = `<span class="error">${result.error || 'Failed'}</span>`;
                }
            } catch (error) {
                document.getElementById('sendResult').innerHTML = `<span class="error">${error.message}</span>`;
            }
        }
        
        
        async function updateConversationIndicator() {
            try {
                const response = await fetch('/api/conversation_status', {
                    method: 'GET'
                });
                
                const result = await response.json();
                
                const indicator = document.getElementById('activeConversation');
                const partner = document.getElementById('activeConversationPartner');
                
                if (result.is_locked && result.locked_with && result.locked_with.length > 0) {
                    partner.textContent = result.locked_with.join(', ');
                    indicator.style.display = 'block';
                } else {
                    indicator.style.display = 'none';
                }
            } catch (error) {
                console.error('Error updating conversation indicator:', error);
            }
        }
        
        // Poll conversation status every 5 seconds
        setInterval(updateConversationIndicator, 5000);
        // Initial check
        setTimeout(updateConversationIndicator, 1000);
        
        async function toggleContinuous() {
            if (commandInProgress) return; // Prevent rapid clicks
            commandInProgress = true;
            
            const resultDiv = document.getElementById('turnResult');
            try {
                const response = await fetch('/api/turn/continuous', {
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
        
        function openControlPanel() {
            console.log('🔧 openControlPanel() called');
            try {
                let modal = document.getElementById('controlPanelModal');
                
                // If modal doesn't exist or isn't in body, move it to body
                if (!modal) {
                    console.error('🔧 ERROR: controlPanelModal element not found!');
                    alert('Control panel modal not found. Check console for details.');
                    return;
                }
                
                // Ensure modal is a direct child of body for proper stacking
                if (modal.parentElement !== document.body) {
                    console.log('🔧 Moving modal to body');
                    document.body.appendChild(modal);
                }
                
                // Set all necessary styles explicitly for full-screen overlay
                modal.style.display = 'flex';
                modal.style.position = 'fixed';
                modal.style.top = '0px';
                modal.style.left = '0px';
                modal.style.right = '0px';
                modal.style.bottom = '0px';
                modal.style.width = '100vw';
                modal.style.height = '100vh';
                modal.style.margin = '0';
                modal.style.padding = '0';
                modal.style.background = 'rgba(0, 0, 0, 0.7)';
                modal.style.zIndex = '99999'; // Very high z-index
                modal.style.justifyContent = 'center';
                modal.style.alignItems = 'center';
                modal.style.visibility = 'visible';
                modal.style.opacity = '1';
                modal.style.pointerEvents = 'auto';
                
                // Check computed styles
                const computed = window.getComputedStyle(modal);
                console.log('🔧 Modal display:', computed.display, 'z-index:', computed.zIndex);
                console.log('🔧 Modal rect:', modal.getBoundingClientRect());
                
                updateControlPanelContinuousButton();
            } catch (error) {
                console.error('🔧 ERROR in openControlPanel:', error);
                alert('Error opening control panel: ' + error.message);
            }
        }
        
        // Make sure function is globally accessible
        window.openControlPanel = openControlPanel;
        
        function closeControlPanel() {
            const modal = document.getElementById('controlPanelModal');
            const status = document.getElementById('controlPanelStatus');
            if (modal) modal.style.display = 'none';
            if (status) status.style.display = 'none';
        }
        
        function updateControlPanelContinuousButton() {
            const panelButton = document.getElementById('controlPanelContinuousButton');
            if (!panelButton) return;
            
            // Try to get state from turn state data if available
            if (window.lastTurnState && window.lastTurnState.buttons && window.lastTurnState.buttons.continuous) {
                const continuous = window.lastTurnState.buttons.continuous;
                const isActive = continuous.active || false;
                panelButton.textContent = continuous.label || '🔄 Continuous';
                panelButton.style.background = isActive ? '#4ecdc4' : '#555';
                panelButton.style.color = isActive ? '#1a1a1a' : '#888';
                panelButton.title = continuous.tooltip || 'Toggle continuous mode';
            } else {
                // Fallback to default state
                panelButton.textContent = '🔄 Continuous';
                panelButton.style.background = '#555';
                panelButton.style.color = '#888';
                panelButton.title = 'Toggle continuous mode - goal will resubmit on completion';
            }
        }
        
        async function toggleContinuousFromPanel() {
            await toggleContinuous();
            updateControlPanelContinuousButton();
        }
        
        async function clearWorldModel() {
            if (!confirm('Are you sure you want to clear the World Model? This cannot be undone.')) {
                return;
            }
            
            const statusDiv = document.getElementById('controlPanelStatus');
            statusDiv.style.display = 'block';
            statusDiv.innerHTML = '<span style="color: #888;">Clearing world model...</span>';
            
            try {
                const response = await fetch('/api/control/clear_world_model', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    statusDiv.innerHTML = '<span style="color: #4ecdc4;">✓ World model cleared successfully</span>';
                } else {
                    statusDiv.innerHTML = `<span style="color: #ff6b6b;">✗ Error: ${result.message || 'Unknown error'}</span>`;
                }
            } catch (error) {
                statusDiv.innerHTML = `<span style="color: #ff6b6b;">✗ Error: ${error.message}</span>`;
            }
        }
        
        async function clearMap() {
            if (!confirm('Are you sure you want to clear the Map? This will delete all map observations and cannot be undone.')) {
                return;
            }
            
            const statusDiv = document.getElementById('controlPanelStatus');
            statusDiv.style.display = 'block';
            statusDiv.innerHTML = '<span style="color: #888;">Clearing map...</span>';
            
            try {
                const response = await fetch('/api/control/clear_map', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    statusDiv.innerHTML = '<span style="color: #4ecdc4;">✓ Map cleared successfully</span>';
                } else {
                    statusDiv.innerHTML = `<span style="color: #ff6b6b;">✗ Error: ${result.message || 'Unknown error'}</span>`;
                }
            } catch (error) {
                statusDiv.innerHTML = `<span style="color: #ff6b6b;">✗ Error: ${error.message}</span>`;
            }
        }
        
        async function clearTransients() {
            const character = activeCharacter || 'Unknown';
            if (!confirm(`Clear all transient resources for ${character}? This cannot be undone.`)) {
                return;
            }
            
            const statusDiv = document.getElementById('controlPanelStatus');
            statusDiv.style.display = 'block';
            statusDiv.innerHTML = '<span style="color: #888;">Clearing transients...</span>';
            
            try {
                const response = await fetch('/api/control/clear_transients', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ character: character })
                });
                
                const result = await response.json();
                
                if (result.success) {
                    const deleted = result.deleted_notes || 0;
                    const collections = result.deleted_collections || 0;
                    const bindings = result.bindings_cleared || 0;
                    statusDiv.innerHTML = `<span style="color: #4ecdc4;">✓ Cleared ${deleted} Notes, ${collections} Collections, ${bindings} bindings</span>`;
                } else {
                    statusDiv.innerHTML = `<span style="color: #ff6b6b;">✗ Error: ${result.message || 'Unknown error'}</span>`;
                }
            } catch (error) {
                statusDiv.innerHTML = `<span style="color: #ff6b6b;">✗ Error: ${error.message}</span>`;
            }
        }
        
        async function clearPersistents() {
            const character = activeCharacter || 'Unknown';
            if (!confirm(`Clear all PERSISTENT resources for ${character}? This cannot be undone.`)) {
                return;
            }
            
            const statusDiv = document.getElementById('controlPanelStatus');
            statusDiv.style.display = 'block';
            statusDiv.innerHTML = '<span style="color: #888;">Clearing persistents...</span>';
            
            try {
                const response = await fetch('/api/control/clear_persistents', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ character: character })
                });
                
                const result = await response.json();
                
                if (result.success) {
                    const deleted = result.deleted_notes || 0;
                    const collections = result.deleted_collections || 0;
                    statusDiv.innerHTML = `<span style="color: #4ecdc4;">✓ Cleared ${deleted} persistent Notes, ${collections} persistent Collections</span>`;
                } else {
                    statusDiv.innerHTML = `<span style="color: #ff6b6b;">✗ Error: ${result.message || 'Unknown error'}</span>`;
                }
            } catch (error) {
                statusDiv.innerHTML = `<span style="color: #ff6b6b;">✗ Error: ${error.message}</span>`;
            }
        }
        
        async function autonomousTurns() {
            if (commandInProgress) return; // Prevent rapid clicks
            commandInProgress = true;
            
            const resultDiv = document.getElementById('turnResult');
            const autonomousButton = document.getElementById('autonomousButton');
            
            // Immediately disable and shade the button for responsive UI
            if (autonomousButton) {
                autonomousButton.disabled = true;
                autonomousButton.style.background = '#555';
                autonomousButton.title = 'Autonomous mode active...';
            }
            
            try {
                const response = await fetch('/api/turn/autonomous', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    }
                });
                
                const result = await response.json();
                
                if (result.success) {
                    if (resultDiv) resultDiv.innerHTML = `<span class="success">${result.message}</span>`;
                    // Button state will be updated by backend via turn_state_update message
                } else {
                    if (resultDiv) resultDiv.innerHTML = `<span class="error">Error: ${result.message}</span>`;
                    // Clear command lock on error so user can retry
                    commandInProgress = false;
                    if (autonomousButton) {
                        autonomousButton.disabled = false;
                        autonomousButton.style.background = '#4ecdc4';
                        autonomousButton.title = 'Click to start autonomous execution';
                    }
                }
            } catch (error) {
                if (resultDiv) resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
                // Clear command lock on error so user can retry
                commandInProgress = false;
                if (autonomousButton) {
                    autonomousButton.disabled = false;
                    autonomousButton.style.background = '#4ecdc4';
                    autonomousButton.title = 'Click to start autonomous execution';
                }
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
        
        function openResourceBrowser() {
            // Open Resource Browser in new tab
            // Just open it - browser will show connection error if not running
            window.open('http://localhost:3001', '_blank');
        }
        
        // Test Runner Functions
        function openTestRunner() {
            const modal = document.getElementById('testModal');
            modal.style.display = 'flex';
            
            // Populate character dropdown
            const charSelect = document.getElementById('testCharacterSelect');
            charSelect.innerHTML = '<option value="">-- Select Character --</option>';
            const characterNames = [];
            for (const [charName, _] of characterTabs) {
                charSelect.innerHTML += `<option value="${charName}">${charName}</option>`;
                characterNames.push(charName);
            }
            
            // Restore sticky character selection if available, otherwise select first character
            if (lastSelectedTestCharacter && characterNames.includes(lastSelectedTestCharacter)) {
                charSelect.value = lastSelectedTestCharacter;
            } else if (characterNames.length > 0) {
                charSelect.value = characterNames[0];
            }
            
            // Save selection when changed
            charSelect.onchange = () => {
                lastSelectedTestCharacter = charSelect.value || null;
            };
            
            // Request test file list from server
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'list_test_files'
                }));
            }
        }
        
        function closeTestModal() {
            const modal = document.getElementById('testModal');
            modal.style.display = 'none';
        }
        
        function updateTestFileList(testFiles) {
            const select = document.getElementById('testFileSelect');
            if (testFiles.length === 0) {
                select.innerHTML = '<option value="">No test files found</option>';
            } else {
                select.innerHTML = testFiles.map(f => 
                    `<option value="${f.path}">${f.name}</option>`
                ).join('');
            }
            
            // Add change handler to show test details
            select.onchange = () => {
                const selected = select.value;
                if (selected && ws && ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({
                        type: 'get_test_details',
                        test_file: selected
                    }));
                }
            };
        }
        
        function showTestDetails(details) {
            const statusDiv = document.getElementById('testStatus');
            statusDiv.innerHTML = `
                <div style="color: #00d4ff; font-weight: bold; margin-bottom: 8px;">${details.name || 'Test'}</div>
                <div style="color: #ccc; margin-bottom: 8px; font-size: 12px;">${details.description || 'No description'}</div>
                <div style="color: #888; font-size: 11px;">
                    <div>Goal: ${details.test_goal || 'N/A'}</div>
                </div>
            `;
        }
        
        function runSelectedTest() {
            const charSelect = document.getElementById('testCharacterSelect');
            const fileSelect = document.getElementById('testFileSelect');
            const character = charSelect.value;
            const testFile = fileSelect.value;
            
            if (!character) {
                alert('Please select a character');
                return;
            }
            
            if (!testFile) {
                alert('Please select a test file');
                return;
            }
            
            // Save character selection for sticky behavior
            lastSelectedTestCharacter = character;
            
            const statusDiv = document.getElementById('testStatus');
            statusDiv.innerHTML = '<div style="color: #f39c12;">⏳ Running test...</div>';
            
            // Send test run request via WebSocket
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'run_test',
                    character: character,
                    test_file: testFile
                }));
            }
        }
        
        function updateTestStatus(status) {
            const statusDiv = document.getElementById('testStatus');
            if (status.error) {
                statusDiv.innerHTML = `<div style="color: #ff4757;">❌ Error: ${status.error}</div>`;
            } else if (status.running) {
                statusDiv.innerHTML = `<div style="color: #f39c12;">⏳ ${status.message}</div>`;
            } else if (status.complete) {
                const violations = status.violations || 0;
                const checks = status.checks || 0;
                const color = violations === 0 ? '#27ae60' : '#ff4757';
                statusDiv.innerHTML = `
                    <div style="color: ${color}; font-weight: bold;">${violations === 0 ? '✅' : '❌'} Test Complete</div>
                    <div style="color: #ccc; margin-top: 8px; font-size: 12px;">
                        <div>Type violations: ${violations}</div>
                        <div>Compatibility checks: ${checks}</div>
                    </div>
                `;
            }
        }
        
        // Save Plan Modal Functions
        let currentSavePlanCharacter = null;
        
        function showSavePlanModal(characterName) {
            currentSavePlanCharacter = characterName;
            const tabData = characterTabs.get(characterName);
            
            if (!tabData || !tabData.currentPlan) {
                alert('No plan available for this character');
                return;
            }
            
            // Parse the plan
            let planData;
            try {
                planData = JSON.parse(tabData.currentPlan);
            } catch (e) {
                alert('Failed to parse plan JSON');
                return;
            }
            
            // Auto-suggest tool name from goal
            const goal = tabData.goal || '';
            let suggestedName = goal.toLowerCase()
                .replace(/[^a-z0-9\s-]/g, '')
                .replace(/\s+/g, '-')
                .substring(0, 50);
            if (!suggestedName) {
                suggestedName = 'saved-plan';
            }
            
            // Pre-fill form
            document.getElementById('toolNameInput').value = suggestedName;
            document.getElementById('toolGoalInput').value = goal;
            
            // Extract parameters from plan (look for literals and variables)
            const parameters = extractPlanParameters(planData);
            displayParameters(parameters);
            
            // Show modal
            document.getElementById('savePlanModal').style.display = 'block';
            document.getElementById('savePlanStatus').style.display = 'none';
        }
        
        function closeSavePlanModal() {
            document.getElementById('savePlanModal').style.display = 'none';
            currentSavePlanCharacter = null;
        }
        
        function extractPlanParameters(planData) {
            // Extract unique literal values and variable references from plan
            const parameters = [];
            const seen = new Set();
            
            function traverse(obj, path = '') {
                if (Array.isArray(obj)) {
                    obj.forEach((item, idx) => traverse(item, `${path}[${idx}]`));
                } else if (typeof obj === 'object' && obj !== null) {
                    for (const [key, value] of Object.entries(obj)) {
                        if (key === 'args' && typeof value === 'object') {
                            // Found args object - extract parameters
                            for (const [argKey, argValue] of Object.entries(value)) {
                                const paramKey = `${obj.type || 'action'}.${argKey}`;
                                if (!seen.has(paramKey)) {
                                    seen.add(paramKey);
                                    parameters.push({
                                        name: argKey,
                                        value: typeof argValue === 'string' ? argValue : JSON.stringify(argValue),
                                        description: `Parameter for ${obj.type || 'action'}`,
                                        required: true
                                    });
                                }
                            }
                        }
                        traverse(value, `${path}.${key}`);
                    }
                }
            }
            
            traverse(planData);
            return parameters;
        }
        
        function displayParameters(parameters) {
            const display = document.getElementById('toolParametersDisplay');
            if (parameters.length === 0) {
                display.innerHTML = '<div style="color: #888; font-style: italic;">No parameters detected</div>';
                return;
            }
            
            let html = '<div style="font-family: Courier New, monospace; font-size: 12px;">';
            parameters.forEach((param, idx) => {
                html += `
                    <div style="margin-bottom: 8px; padding: 6px; background: #2d2d2d; border-radius: 3px; border-left: 3px solid #00d4ff;">
                        <div style="color: #00d4ff; font-weight: bold;">${param.name}</div>
                        <div style="color: #888; font-size: 11px; margin-top: 2px;">${param.description}</div>
                        <div style="color: #ccc; margin-top: 4px;">Default: <span style="color: #4ecdc4;">${param.value}</span></div>
                    </div>
                `;
            });
            html += '</div>';
            display.innerHTML = html;
        }
        
        async function executeSavePlan() {
            const toolName = document.getElementById('toolNameInput').value.trim();
            const goal = document.getElementById('toolGoalInput').value.trim();
            const statusDiv = document.getElementById('savePlanStatus');
            
            // Validate inputs
            if (!toolName) {
                alert('Please enter a tool name');
                return;
            }
            
            if (!/^[a-z][a-z0-9-]*$/.test(toolName)) {
                alert('Tool name must be lowercase kebab-case (e.g., "research-papers")');
                return;
            }
            
            if (!goal) {
                alert('Please enter a goal/description');
                return;
            }
            
            const tabData = characterTabs.get(currentSavePlanCharacter);
            if (!tabData || !tabData.currentPlan) {
                alert('Plan data not available');
                return;
            }
            
            // Parse plan
            let planData;
            try {
                planData = JSON.parse(tabData.currentPlan);
            } catch (e) {
                alert('Failed to parse plan JSON');
                return;
            }
            
            // Extract parameters
            const parameters = extractPlanParameters(planData);
            
            // Get overwrite flag
            const overwrite = document.getElementById('overwriteCheckbox').checked;
            
            // Show status
            statusDiv.style.display = 'block';
            statusDiv.innerHTML = '<div style="color: #f39c12;">⏳ Saving plan...</div>';
            
            try {
                const response = await fetch('/api/save_plan', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        tool_name: toolName,
                        goal: goal,
                        plan: planData,
                        parameters: parameters,
                        overwrite: overwrite
                    })
                });
                
                const result = await response.json();
                
                if (result.success) {
                    statusDiv.innerHTML = `<div style="color: #27ae60;">✅ ${result.message}</div>`;
                    setTimeout(() => {
                        closeSavePlanModal();
                    }, 2000);
                } else {
                    statusDiv.innerHTML = `<div style="color: #ff4757;">❌ Error: ${result.message}</div>`;
                }
            } catch (error) {
                statusDiv.innerHTML = `<div style="color: #ff4757;">❌ Error: ${error.message}</div>`;
            }
        }
        
        // Close modal when clicking outside
        window.onclick = function(event) {
            const savePlanModal = document.getElementById('savePlanModal');
            if (event.target === savePlanModal) {
                closeSavePlanModal();
            }
        }
        
        // Notification function
        function showNotification(message, type = 'info') {
            const notification = document.createElement('div');
            notification.textContent = message;
            notification.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                padding: 12px 20px;
                background: ${type === 'error' ? '#ff4757' : type === 'success' ? '#2ed573' : '#00d4ff'};
                color: #1a1a1a;
                border-radius: 6px;
                font-weight: bold;
                z-index: 10000;
                box-shadow: 0 4px 12px rgba(0,0,0,0.3);
                animation: slideIn 0.3s ease-out;
            `;
            document.body.appendChild(notification);
            setTimeout(() => {
                notification.style.animation = 'slideOut 0.3s ease-out';
                setTimeout(() => notification.remove(), 300);
            }, 3000);
        }
        
        // Saved Plans List Functions
        async function executeSavedPlan(planName) {
            const character = activeCharacter || 'Jill';
            
            try {
                showNotification(`▶️  Executing ${planName}...`, 'info');
                
                const response = await fetch('/api/execute_saved_plan', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({plan_name: planName, character: character})
                });
                
                const result = await response.json();
                
                if (result.success) {
                    showNotification(`✅ ${result.message}`, 'success');
                    switchCharacterDataTab('plan');
                } else {
                    showNotification(`❌ ${result.message}`, 'error');
                }
            } catch (error) {
                showNotification(`❌ Error: ${error.message}`, 'error');
            }
        }
        
        async function loadBindingsList(character) {
            const listDiv = document.getElementById('bindingsList');
            if (!listDiv) {
                return;
            }
            listDiv.innerHTML = '<div style="color: #f39c12; text-align: center; padding: 20px;">⏳ Loading...</div>';
            
            try {
                const response = await fetch(`/api/plan_bindings/${character}`);
                const result = await response.json();
                
                if (!result.success) {
                    listDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ Error: ${result.message}</div>`;
                    return;
                }
                
                const bindings = result.bindings || {};
                const bindingKeys = Object.keys(bindings);
                
                if (bindingKeys.length === 0) {
                    listDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No bindings set. Execute a plan to create bindings.</div>';
                    return;
                }
                
                let html = '<div style="margin-bottom: 10px; color: #00d4ff; font-weight: bold; font-size: 12px;">Variables in Scope:</div>';
                bindingKeys.forEach(varName => {
                    const binding = bindings[varName];
                    let valueDisplay = '';
                    
                    if (binding.type === 'resource_id') {
                        valueDisplay = `<a href="#" class="resource-link" data-resource-id="${binding.value}" style="color: #2ed573;">${binding.value}</a>`;
                    } else if (binding.type === 'string') {
                        valueDisplay = `"${binding.value}"`;
                    } else if (binding.type === 'dict') {
                        valueDisplay = `{${binding.keys.join(', ')}}`;
                    } else if (binding.type === 'list') {
                        valueDisplay = `[${binding.length} items]`;
                    } else {
                        valueDisplay = `${binding.value || binding.type}`;
                    }
                    
                    html += `
                        <div class="character-data-item">
                            <div style="display: flex; justify-content: space-between; align-items: center;">
                                <div class="character-data-label" style="color: #f39c12;">${varName}</div>
                                <div style="font-size: 10px; color: #666;">${binding.type}</div>
                            </div>
                            <div style="color: #ccc; font-size: 11px; font-family: monospace; word-break: break-all;">${valueDisplay}</div>
                        </div>
                    `;
                });
                
                listDiv.innerHTML = html;
            } catch (error) {
                listDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ Error: ${error.message}</div>`;
            }
        }
        
        async function loadTasksList(character) {
            const listDiv = document.getElementById('tasksList');
            if (!listDiv) return;
            listDiv.innerHTML = '<div style="color: #f39c12; text-align: center; padding: 20px;">⏳ Loading...</div>';
            try {
                const response = await fetch(`/api/scheduled_goals/${character}`);
                const result = await response.json();
                if (!result.success) {
                    listDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ ${result.message || 'Error'}</div>`;
                    return;
                }
                const goals = result.goals || [];
                if (goals.length === 0) {
                    listDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No scheduled goals. Type "goal: &lt;description&gt;" to create one.</div>';
                    return;
                }
                const escapedChar = character.replace(/'/g, "\\'");
                window._goalTextCache = {};
                let html = '';
                goals.forEach((g, i) => {
                    window._goalTextCache[g.goal_id] = g.goal_text || '';
                    const status = g.status || 'ready';
                    const taskId = `goal-acc-${i}`;
                    const hasCache = Array.isArray(g.cached_plan_actions) && g.cached_plan_actions.length > 0;
                    const cacheLabel = hasCache ? `${g.cached_plan_actions.length} cached action(s)` : 'no cache';
                    const isRunning = !!g.is_running;
                    const actionLabel = isRunning ? 'Interrupt' : 'Remove';
                    const actionColor = isRunning ? '#ff9f43' : '#ff6b6b';
                    html += `
                        <div class="task-accordion-item" style="border: 1px solid #404040; border-radius: 4px; margin-bottom: 8px; overflow: hidden;">
                            <div class="task-accordion-header" onclick="toggleTaskAccordion('${taskId}')" style="padding: 8px 10px; cursor: pointer; display: flex; align-items: stretch; gap: 8px;">
                                <div style="flex: 1; min-width: 0; padding-right: 4px;">
                                    <input type="text" value="${escapeHtml(g.name || g.goal_id || 'Goal')}" onchange="event.stopPropagation(); setGoalName('${escapedChar}', '${g.goal_id || ''}', this.value)" style="width: 100%; background: #2a2a2a; color: #00d4ff; border: 1px solid #555; padding: 3px 4px; border-radius: 3px; font-size: 11px; font-weight: bold;" />
                                    <span style="margin-left: 6px; font-size: 10px; padding: 2px 6px; border-radius: 3px; background: #404040;">${status}</span>
                                    <span style="margin-left: 4px; color: #888; font-size: 11px;">${escapeHtml(g.goal_id || '')}</span>
                                    <div style="margin-top: 3px; color: #a8b3c7; font-size: 10px;">${cacheLabel}</div>
                                    <div onclick="event.stopPropagation(); openGoalTextEditor('${escapedChar}', '${g.goal_id}')" style="margin-top: 3px; color: #8f9bb3; font-size: 10px; cursor: pointer;" title="Click to edit goal text">${escapeHtml((g.goal_text || '').slice(0, 140))}${(g.goal_text || '').length > 140 ? '...' : ''}</div>
                                </div>
                                <div onclick="event.stopPropagation();" style="display: flex; flex-direction: column; align-items: stretch; gap: 3px; min-width: 90px;">
                                    <select onchange="event.stopPropagation(); onScheduleModeChange('${escapedChar}', '${g.goal_id || ''}', this)" style="background: #2a2a2a; color: #ccc; border: 1px solid #555; padding: 3px 4px; border-radius: 3px; font-size: 11px; cursor: pointer; width: 100%;">
                                        <option value="manual" ${(g.schedule_mode || 'manual') === 'manual' ? 'selected' : ''}>Manual</option>
                                        <option value="auto" ${g.schedule_mode === 'auto' ? 'selected' : ''}>Auto</option>
                                        <option value="recurring" ${g.schedule_mode === 'recurring' ? 'selected' : ''}>Recurring</option>
                                        <option value="daily" ${g.schedule_mode === 'daily' ? 'selected' : ''}>Daily</option>
                                    </select>
                                    <input type="time" value="${g.run_at || ''}" onchange="event.stopPropagation(); setTaskRunAt('${escapedChar}', '${g.goal_id || ''}', this.value)" style="display:${g.schedule_mode === 'daily' ? 'block' : 'none'}; background: #2a2a2a; color: #ccc; border: 1px solid #555; padding: 3px 4px; border-radius: 3px; font-size: 11px; width: 100%; box-sizing: border-box;" title="Daily run time (24h)" />
                                    <button onclick="event.stopPropagation(); sendTaskCommand('${escapedChar}', 'proceed ${g.goal_id || ''}')" style="background: #00d4ff; color: #1a1a1a; border: none; padding: 4px 0; border-radius: 3px; font-size: 11px; cursor: pointer; width: 100%;">Proceed</button>
                                    <button onclick="event.stopPropagation(); sendTaskCommand('${escapedChar}', 'reuse ${g.goal_id || ''}')" style="background: #9b59b6; color: white; border: none; padding: 4px 0; border-radius: 3px; font-size: 11px; cursor: pointer; width: 100%;">Reuse</button>
                                    <button onclick="event.stopPropagation(); onGoalPrimaryAction('${escapedChar}', '${g.goal_id || ''}', ${isRunning})" style="background: ${actionColor}; color: white; border: none; padding: 4px 0; border-radius: 3px; font-size: 11px; cursor: pointer; width: 100%;">${actionLabel}</button>
                                </div>
                            </div>
                            <div id="${taskId}" class="task-accordion-body" style="display: none; padding: 10px; border-top: 1px solid #404040; background: #1e1e1e; font-size: 11px;">
                                <pre style="white-space: pre-wrap; margin: 0; font-size: 10px; overflow-x: auto;">${escapeHtml(JSON.stringify(g, null, 2))}</pre>
                            </div>
                        </div>
                    `;
                });
                listDiv.innerHTML = html;
            } catch (error) {
                listDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ ${error.message}</div>`;
            }
        }
        
        function toggleTaskAccordion(id) {
            const el = document.getElementById(id);
            if (el) el.style.display = el.style.display === 'none' ? 'block' : 'none';
        }
        
        async function sendTaskCommand(character, cmd) {
            try {
                const response = await fetch('/api/text_input', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ character: character, message: cmd })
                });
                const result = await response.json();
                if (result.success) {
                    // Delay refresh to allow backend to process the command
                    if (activeCharacter === character) {
                        setTimeout(() => loadTasksList(character), 1200);
                    }
                    return true;
                }
                return false;
            } catch (e) {
                console.error('sendTaskCommand:', e);
                return false;
            }
        }

        async function sendTaskCommandSequence(character, commands) {
            for (const cmd of commands) {
                const ok = await sendTaskCommand(character, cmd);
                if (!ok) return false;
            }
            return true;
        }

        async function setTaskScheduleMode(character, taskId, mode, runAt) {
            try {
                const body = { goal_id: taskId, schedule_mode: mode };
                if (runAt) body.run_at = runAt;
                await fetch(`/api/goal_schedule_mode/${character}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(body)
                });
                loadTasksList(character);
            } catch (e) {
                console.error('setTaskScheduleMode:', e);
            }
        }

        function onScheduleModeChange(character, taskId, selectEl) {
            const mode = selectEl.value;
            // Show/hide the time picker sibling
            const timeInput = selectEl.nextElementSibling;
            if (timeInput && timeInput.type === 'time') {
                timeInput.style.display = mode === 'daily' ? 'inline' : 'none';
            }
            const runAt = (mode === 'daily' && timeInput) ? timeInput.value : undefined;
            setTaskScheduleMode(character, taskId, mode, runAt);
        }

        async function setTaskRunAt(character, taskId, runAt) {
            try {
                await fetch(`/api/goal_schedule_mode/${character}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ goal_id: taskId, schedule_mode: 'daily', run_at: runAt })
                });
            } catch (e) {
                console.error('setTaskRunAt:', e);
            }
        }

        async function interruptGoal(character, goalId) {
            if (!goalId) return false;
            try {
                const response = await fetch(`/api/goal_interrupt/${character}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ goal_id: goalId })
                });
                const result = await response.json();
                return !!result.success;
            } catch (e) {
                console.error('interruptGoal:', e);
                return false;
            }
        }

        async function removeGoal(character, goalId) {
            if (!goalId) return false;
            try {
                const response = await fetch(`/api/goal_remove/${character}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ goal_id: goalId })
                });
                const result = await response.json();
                return !!result.success;
            } catch (e) {
                console.error('removeGoal:', e);
                return false;
            }
        }

        async function onGoalPrimaryAction(character, goalId, isRunning) {
            const label = goalId || 'this goal';
            if (isRunning) {
                if (!confirm(`Interrupt ${label}?`)) return;
                const ok = await interruptGoal(character, goalId);
                if (ok && activeCharacter === character) {
                    setTimeout(() => loadTasksList(character), 300);
                }
                return;
            }
            if (!confirm(`Remove ${label}?`)) return;
            const ok = await removeGoal(character, goalId);
            if (ok && activeCharacter === character) {
                loadTasksList(character);
                setTimeout(() => loadTasksList(character), 500);
            }
        }

        async function setGoalName(character, goalId, name) {
            try {
                await fetch(`/api/goal_rename/${character}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ goal_id: goalId, name: name })
                });
                loadTasksList(character);
            } catch (e) {
                console.error('setGoalName:', e);
            }
        }

        let _goalEditorState = {};

        function openGoalTextEditor(character, goalId) {
            _goalEditorState = { character, goalId };
            const goalText = (window._goalTextCache && window._goalTextCache[goalId]) || '';

            let modal = document.getElementById('goalTextModal');
            if (!modal) {
                modal = document.createElement('div');
                modal.id = 'goalTextModal';
                modal.innerHTML = `
                    <div id="goalTextModalBackdrop" onclick="closeGoalTextEditor()" style="position:fixed; top:0; left:0; width:100%; height:100%; background:rgba(0,0,0,0.5); z-index:9998;"></div>
                    <div id="goalTextModalDialog" style="position:fixed; top:50%; left:50%; transform:translate(-50%,-50%); width:600px; max-width:90vw; max-height:80vh; background:#1e1e1e; border:1px solid #555; border-radius:8px; z-index:9999; display:flex; flex-direction:column; resize:both; overflow:auto; min-width:300px; min-height:200px;">
                        <div style="padding:10px 14px; border-bottom:1px solid #404040; display:flex; justify-content:space-between; align-items:center;">
                            <span style="color:#00d4ff; font-size:13px; font-weight:bold;">Edit Goal Text</span>
                            <button onclick="closeGoalTextEditor()" style="background:none; border:none; color:#888; font-size:18px; cursor:pointer; padding:0 4px;">&times;</button>
                        </div>
                        <textarea id="goalTextModalArea" style="flex:1; margin:10px 14px; background:#2a2a2a; color:#ddd; border:1px solid #555; padding:10px; border-radius:4px; font-size:13px; font-family:monospace; line-height:1.5; resize:none; min-height:200px;"></textarea>
                        <div style="padding:8px 14px; border-top:1px solid #404040; display:flex; justify-content:flex-end; gap:8px;">
                            <button onclick="closeGoalTextEditor()" style="background:#555; color:#ccc; border:none; padding:6px 16px; border-radius:4px; font-size:12px; cursor:pointer;">Cancel</button>
                            <button onclick="saveGoalTextFromModal()" style="background:#2ecc71; color:#fff; border:none; padding:6px 16px; border-radius:4px; font-size:12px; cursor:pointer;">Save</button>
                        </div>
                    </div>`;
                document.body.appendChild(modal);
            }
            document.getElementById('goalTextModalArea').value = goalText;
            modal.style.display = 'block';
            document.getElementById('goalTextModalArea').focus();
        }

        function closeGoalTextEditor() {
            const modal = document.getElementById('goalTextModal');
            if (modal) modal.style.display = 'none';
            _goalEditorState = {};
        }

        async function saveGoalTextFromModal() {
            const { character, goalId } = _goalEditorState;
            const textarea = document.getElementById('goalTextModalArea');
            if (!character || !goalId || !textarea) return;
            try {
                await fetch(`/api/goal_text_update/${character}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ goal_id: goalId, goal_text: textarea.value })
                });
                closeGoalTextEditor();
                loadTasksList(character);
            } catch (e) {
                console.error('saveGoalText:', e);
            }
        }

        async function clearGoalCache(character, goalId) {
            try {
                await fetch(`/api/goal_cache/${character}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ goal_id: goalId, action: 'clear' })
                });
                sendTaskCommand(character, `clear-cache ${goalId}`);
                loadTasksList(character);
            } catch (e) {
                console.error('clearGoalCache:', e);
            }
        }

        async function toggleScheduler(enabled) {
            if (!activeCharacter) return;
            try {
                await fetch(`/api/task_scheduler/${activeCharacter}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ enable: enabled })
                });
            } catch (e) { console.error('toggleScheduler:', e); }
        }

        async function updateSchedulerInterval(minutes) {
            if (!activeCharacter) return;
            const mins = Math.max(1, Math.min(1440, parseInt(minutes) || 1));
            try {
                await fetch(`/api/task_scheduler/${activeCharacter}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ interval: mins * 60 })
                });
            } catch (e) { console.error('updateSchedulerInterval:', e); }
        }

        function updateSchedulerUI(schedulerState) {
            if (!schedulerState) return;
            const toggle = document.getElementById('schedulerToggle');
            const status = document.getElementById('schedulerStatus');
            const interval = document.getElementById('schedulerInterval');
            const eventsDiv = document.getElementById('schedulerEvents');
            if (toggle) toggle.checked = schedulerState.enabled;
            if (interval) interval.value = Math.round((schedulerState.interval || 60) / 60);
            if (status) {
                if (!schedulerState.enabled) {
                    status.textContent = 'disabled';
                    status.style.color = '#888';
                } else if (schedulerState.executing_task_id) {
                    status.textContent = 'running: ' + schedulerState.executing_task_id;
                    status.style.color = '#4ecdc4';
                } else {
                    status.textContent = 'waiting';
                    status.style.color = '#f39c12';
                }
            }
            if (eventsDiv) {
                const events = Array.isArray(schedulerState.events) ? schedulerState.events : [];
                if (!events.length) {
                    eventsDiv.innerHTML = '<div style="color: #777; font-style: italic;">No scheduler events yet.</div>';
                } else {
                    const rows = events.slice(-12).reverse().map((evt) => {
                        const t = evt.ts ? evt.ts.split('T').pop().slice(0, 8) : '--:--:--';
                        const head = `${t} ${String(evt.event || '').toUpperCase()} ${evt.goal_id || ''}`.trim();
                        const tail = evt.status ? ` [${evt.status}]` : (evt.reason ? ` (${evt.reason})` : '');
                        const details = evt.result ? ` - ${evt.result}` : '';
                        return `<div style="white-space: nowrap; overflow: hidden; text-overflow: ellipsis;">${escapeHtml(head + tail + details)}</div>`;
                    });
                    eventsDiv.innerHTML = rows.join('');
                }
            }
        }

        function updateWorldStateDisplay(data) {
            // Only update if state tab is active and matches current character
            const activeTab = document.querySelector('.character-data-tab.active');
            if (!activeTab || activeTab.getAttribute('data-tab') !== 'state') {
                return;
            }
            
            // Check if this update is for the currently selected character
            if (data.character !== activeCharacter) {
                return;
            }
            
            const stateDiv = document.getElementById('worldStateDisplay');
            if (!stateDiv) {
                return;
            }
            
            const worldState = data.world_state || {};
            
            if (Object.keys(worldState).length === 0) {
                stateDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No world state data available.</div>';
                return;
            }
            
            // Use formatted_json if provided, otherwise format it
            const formattedJson = data.formatted_json || JSON.stringify(worldState, null, 2);
            
            stateDiv.innerHTML = `
                <div style="background: #1e1e1e; border: 1px solid #3e3e42; border-radius: 4px; padding: 15px; font-family: 'Courier New', monospace; font-size: 12px; line-height: 1.6; white-space: pre-wrap; word-wrap: break-word; color: #d4d4d4; max-height: 600px; overflow-y: auto;">
                    ${escapeHtml(formattedJson)}
                </div>
            `;
        }
        
        async function loadWorldState(character) {
            const stateDiv = document.getElementById('worldStateDisplay');
            if (!stateDiv) {
                return;
            }
            stateDiv.innerHTML = '<div style="color: #f39c12; text-align: center; padding: 20px;">⏳ Loading...</div>';
            
            try {
                const response = await fetch(`/api/world_state/${character}`);
                const result = await response.json();
                
                if (!result.success) {
                    stateDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ Error: ${result.message}</div>`;
                    return;
                }
                
                const worldState = result.world_state || {};
                
                if (Object.keys(worldState).length === 0) {
                    stateDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No world state data available.</div>';
                    return;
                }
                
                // Format world_state as JSON with indentation
                const formattedJson = JSON.stringify(worldState, null, 2);
                
                stateDiv.innerHTML = `
                    <div style="background: #1e1e1e; border: 1px solid #3e3e42; border-radius: 4px; padding: 15px; font-family: 'Courier New', monospace; font-size: 12px; line-height: 1.6; white-space: pre-wrap; word-wrap: break-word; color: #d4d4d4; max-height: 600px; overflow-y: auto;">
                        ${escapeHtml(formattedJson)}
                    </div>
                `;
            } catch (error) {
                stateDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ Error: ${error.message}</div>`;
            }
        }
        
        async function executeGoal(goalName) {
            const character = activeCharacter || 'Jill';
            
            try {
                showNotification(`▶️  Executing ${goalName}...`, 'info');
                
                const response = await fetch('/api/execute_test_goal', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({goal_file: goalName + '.yaml', character: character})
                });
                
                const result = await response.json();
                
                if (result.success) {
                    showNotification(`✅ ${result.message}`, 'success');
                    switchCharacterDataTab('plan');
                } else {
                    showNotification(`❌ ${result.message}`, 'error');
                }
            } catch (error) {
                showNotification(`❌ Error: ${error.message}`, 'error');
            }
        }
        
        async function loadGoalsList() {
            const listDiv = document.getElementById('goalsList');
            listDiv.innerHTML = '<div style="color: #f39c12; text-align: center; padding: 20px;">⏳ Loading...</div>';
            
            try {
                const response = await fetch('/api/test_goals');
                const result = await response.json();
                
                if (!result.success) {
                    listDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ Error: ${result.message}</div>`;
                    return;
                }
                
                const goals = result.goals || [];
                
                if (goals.length === 0) {
                    listDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No goals found.</div>';
                    return;
                }
                
                let html = '';
                goals.forEach((goal, idx) => {
                    const description = goal.description || 'No description';
                    const testGoal = goal.test_goal || '';
                    const preview = testGoal.substring(0, 100) + (testGoal.length > 100 ? '...' : '');
                    
                    html += `
                        <div class="character-data-item" style="transition: background 0.2s; cursor: pointer;" 
                             onmouseover="this.style.background='#333'" 
                             onmouseout="this.style.background='#1a1a1a'"
                             onclick="event.target.tagName !== 'BUTTON' && showGoalDetails('${goal.name}')">
                            <div style="display: flex; justify-content: space-between; align-items: start; margin-bottom: 4px;">
                                <div class="character-data-label" style="flex: 1; margin-bottom: 0;">${goal.title}</div>
                                <button onclick="event.stopPropagation(); executeGoal('${goal.name}')" 
                                        style="background: #00d4ff; color: #1a1a1a; border: none; border-radius: 4px; padding: 4px 8px; font-size: 11px; cursor: pointer; font-weight: bold;"
                                        onmouseover="this.style.background='#00a8cc'" 
                                        onmouseout="this.style.background='#00d4ff'">
                                    ▶️ Execute
                                </button>
                            </div>
                            <div style="color: #ccc; font-size: 12px; line-height: 1.3; margin-bottom: 4px;">${description}</div>
                            <div style="color: #666; font-size: 10px;">${preview}</div>
                        </div>
                    `;
                });
                
                listDiv.innerHTML = html;
            } catch (error) {
                listDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ Error: ${error.message}</div>`;
            }
        }
        
        async function loadSavedPlansList() {
            const listDiv = document.getElementById('savedPlansList');
            listDiv.innerHTML = '<div style="color: #f39c12; text-align: center; padding: 20px;">⏳ Loading...</div>';
            
            try {
                const response = await fetch('/api/saved_plans');
                const result = await response.json();
                
                if (!result.success) {
                    listDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ Error: ${result.message}</div>`;
                    return;
                }
                
                const plans = result.plans || [];
                
                if (plans.length === 0) {
                    listDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">No saved plans yet. Execute a goal and click "Save as Tool" to create one.</div>';
                    return;
                }
                
                let html = '';
                plans.forEach((plan, idx) => {
                    const savedDate = plan.saved_at ? new Date(plan.saved_at).toLocaleString() : 'Unknown';
                    html += `
                        <div class="character-data-item" style="transition: background 0.2s; cursor: pointer;" 
                             onmouseover="this.style.background='#333'" 
                             onmouseout="this.style.background='#1a1a1a'"
                             onclick="event.target.tagName !== 'BUTTON' && showPlanDetails('${plan.name}')">
                            <div style="display: flex; justify-content: space-between; align-items: start; margin-bottom: 4px;">
                                <div class="character-data-label" style="flex: 1; margin-bottom: 0;">${plan.name}</div>
                                <div style="display: flex; gap: 8px; align-items: center;">
                                    <div style="font-size: 10px; color: #666;">${plan.action_count} actions</div>
                                    <button onclick="event.stopPropagation(); executeSavedPlan('${plan.name}')" 
                                            style="background: #00d4ff; color: #1a1a1a; border: none; border-radius: 4px; padding: 4px 8px; font-size: 11px; cursor: pointer; font-weight: bold;"
                                            onmouseover="this.style.background='#00a8cc'" 
                                            onmouseout="this.style.background='#00d4ff'">
                                        ▶️ Execute
                                    </button>
                                </div>
                            </div>
                            <div style="color: #ccc; font-size: 12px; line-height: 1.3; margin-bottom: 4px;">${plan.goal}</div>
                            <div style="color: #666; font-size: 10px;">${savedDate}</div>
                        </div>
                    `;
                });
                
                listDiv.innerHTML = html;
            } catch (error) {
                listDiv.innerHTML = `<div style="color: #ff4757; text-align: center; padding: 20px;">❌ Error: ${error.message}</div>`;
            }
        }
        
        function shutdownWithSave() {
            if (confirm('Save and shutdown the system?')) {
                fetch('/api/save_and_shutdown', {method: 'POST'})
                    .then(response => response.json())
                    .then(data => {
                        if (data.success) {
                            alert('Save initiated, shutting down...');
                        } else {
                            alert('Error: ' + data.message);
                        }
                    })
                    .catch(err => alert('Error: ' + err));
            }
        }
        
        // Time slider functions
        function initTimeSlider() {
            const slider = document.getElementById('timeSlider');
            const sliderValue = document.getElementById('timeSliderValue');
            
            if (!slider || !sliderValue) {
                return; // Elements don't exist, skip initialization
            }
            
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
        
        function toggleLog() {
            const log = document.getElementById('actionLog');
            const toggle = document.getElementById('logToggle');
            log.classList.toggle('collapsed');
            toggle.classList.toggle('collapsed');
        }
        

        
        // Initialize time slider when page loads
        document.addEventListener('DOMContentLoaded', function() {
            initTimeSlider();
            // Get initial simulation time
            getInitialSimulationTime();
            // Send initial delay value (0) to backend
            updateTimeDelay(0);
            
            // Set up control panel button event listener
            const controlButton = document.querySelector('button[onclick="openControlPanel()"]');
            if (controlButton) {
                controlButton.removeAttribute('onclick');
                controlButton.addEventListener('click', function(e) {
                    e.preventDefault();
                    console.log('🔧 Control button clicked via event listener');
                    openControlPanel();
                });
                console.log('🔧 Control panel button event listener attached');
            } else {
                console.warn('🔧 Control panel button not found for event listener setup');
            }
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
        
        // Display Modal Functions
        function showDisplayModal(actionData) {
            const modal = document.getElementById('displayModal');
            const modalTitle = document.getElementById('modalTitle');
            const modalContent = document.getElementById('modalContent');
            
            // Set title with character and target
            const character = actionData.character || 'Agent';
            const target = actionData.target || 'User';
            modalTitle.textContent = `${character} → ${target}`;
            
            // Set content from text, value, or llm_response
            const content = actionData.text || actionData.value || actionData.llm_response || 'No content';
            modalContent.textContent = content;
            
            // Show modal
            modal.style.display = 'block';
            
            // Close on background click
            modal.onclick = function(event) {
                if (event.target === modal) {
                    closeDisplayModal();
                }
            };
        }
        
        function closeDisplayModal() {
            const modal = document.getElementById('displayModal');
            modal.style.display = 'none';
            modal.onclick = null;
        }
        
        // Modal resize functionality
        let isResizingModal = false;
        let modalStartX, modalStartY, modalStartWidth, modalStartHeight;
        
        const modalResizeHandle = document.getElementById('modalResizeHandle');
        const modalContentDiv = document.getElementById('modalContentDiv');
        
        if (modalResizeHandle && modalContentDiv) {
            modalResizeHandle.addEventListener('mousedown', function(e) {
                isResizingModal = true;
                modalStartX = e.clientX;
                modalStartY = e.clientY;
                modalStartWidth = parseInt(document.defaultView.getComputedStyle(modalContentDiv).width, 10);
                modalStartHeight = parseInt(document.defaultView.getComputedStyle(modalContentDiv).height, 10);
                e.preventDefault();
            });
            
            document.addEventListener('mousemove', function(e) {
                if (!isResizingModal) return;
                const width = modalStartWidth + e.clientX - modalStartX;
                const height = modalStartHeight + e.clientY - modalStartY;
                modalContentDiv.style.width = Math.max(600, Math.min(width, window.innerWidth * 0.95)) + 'px';
                modalContentDiv.style.height = Math.max(400, Math.min(height, window.innerHeight * 0.85)) + 'px';
            });
            
            document.addEventListener('mouseup', function() {
                isResizingModal = false;
            });
        }
        
        // Close modal on Escape key
        document.addEventListener('keydown', function(event) {
            if (event.key === 'Escape') {
                closeDisplayModal();
            }
        });
        
        // Resource Viewer Functions
        async function showResourceModal(resourceId) {
            const modal = document.getElementById('displayModal');
            const modalTitle = document.getElementById('modalTitle');
            const modalContent = document.getElementById('modalContent');
            
            // Set loading state
            modalTitle.textContent = 'Resource Viewer';
            modalContent.textContent = 'Loading...';
            modal.style.display = 'block';
            
            try {
                // Fetch resource content from API
                const response = await fetch(`/api/resource/${resourceId}`);
                const data = await response.json();
                
                if (data.success) {
                    // Set title with resource ID and type
                    modalTitle.textContent = `${data.type}: ${resourceId}`;
                    
                    // Format content for display
                    let formattedContent = '';
                    
                    // Add metadata
                    const metadata = data.metadata || {};
                    if (metadata.creator) formattedContent += `Creator: ${metadata.creator}\\n`;
                    if (metadata.created_at) formattedContent += `Created: ${metadata.created_at}\\n`;
                    if (metadata.format) formattedContent += `Format: ${metadata.format}\\n`;
                    if (metadata.source_skill) formattedContent += `Source: ${metadata.source_skill}\\n`;
                    if (metadata.item_count !== undefined) formattedContent += `Items: ${metadata.item_count}\\n`;
                    if (metadata.persistent !== undefined) formattedContent += `Persistent: ${metadata.persistent}\\n`;
                    formattedContent += '\\n---\\n\\n';
                    
                    // Add content (format JSON if needed)
                    let content = data.content;
                    
                    // If content is null or undefined, show placeholder
                    if (content === null || content === undefined) {
                        formattedContent += '(empty)';
                    } else if (typeof content === 'object') {
                        // Already an object, stringify it
                        formattedContent += JSON.stringify(content, null, 2);
                    } else if (typeof content === 'string') {
                        // Try to parse as JSON if it looks like JSON
                        if (content.trim().startsWith('{') || content.trim().startsWith('[')) {
                            try {
                                const parsed = JSON.parse(content);
                                formattedContent += JSON.stringify(parsed, null, 2);
                            } catch (e) {
                                // Not valid JSON, display as-is
                                formattedContent += content;
                            }
                        } else {
                            formattedContent += content;
                        }
                    } else {
                        formattedContent += String(content);
                    }
                    
                    modalContent.textContent = formattedContent;
                } else {
                    modalTitle.textContent = 'Error';
                    modalContent.textContent = data.message || 'Failed to load resource';
                }
            } catch (error) {
                modalTitle.textContent = 'Error';
                modalContent.textContent = `Failed to fetch resource: ${error.message}`;
            }
            
            // Close on background click
            modal.onclick = function(event) {
                if (event.target === modal) {
                    closeDisplayModal();
                }
            };
        }
        
        // Add click handlers to resource links (delegated event listener)
        document.addEventListener('click', function(event) {
            if (event.target.classList.contains('resource-link')) {
                event.preventDefault();
                const resourceId = event.target.getAttribute('data-resource-id');
                if (resourceId) {
                    showResourceModal(resourceId);
                }
            }
        });
        
        // Goal/Plan details modal functions
        async function showGoalDetails(goalName) {
            const modal = document.getElementById('detailsModal');
            const title = document.getElementById('detailsModalTitle');
            const content = document.getElementById('detailsModalContent');
            
            title.textContent = `Goal: ${goalName}`;
            content.innerHTML = '<div style="color: #f39c12; text-align: center; padding: 20px;">⏳ Loading...</div>';
            modal.style.display = 'flex';
            
            try {
                const response = await fetch(`/api/goal_details/${goalName}`);
                const result = await response.json();
                
                if (result.success) {
                    content.innerHTML = `<pre style="background: #1a1a1a; padding: 15px; border-radius: 5px; overflow-x: auto; color: #e0e0e0; font-size: 12px; line-height: 1.4;">${escapeHtml(result.raw)}</pre>`;
                } else {
                    content.innerHTML = `<div style="color: #ff4757;">Error: ${result.message}</div>`;
                }
            } catch (error) {
                content.innerHTML = `<div style="color: #ff4757;">Error: ${error.message}</div>`;
            }
        }
        
        async function showPlanDetails(planName) {
            const modal = document.getElementById('detailsModal');
            const title = document.getElementById('detailsModalTitle');
            const content = document.getElementById('detailsModalContent');
            
            title.textContent = `Plan: ${planName}`;
            content.innerHTML = '<div style="color: #f39c12; text-align: center; padding: 20px;">⏳ Loading...</div>';
            modal.style.display = 'flex';
            
            try {
                const response = await fetch(`/api/plan_details/${planName}`);
                const result = await response.json();
                
                if (result.success) {
                    content.innerHTML = `<pre style="background: #1a1a1a; padding: 15px; border-radius: 5px; overflow-x: auto; color: #e0e0e0; font-size: 12px; line-height: 1.4;">${escapeHtml(JSON.stringify(result.content, null, 2))}</pre>`;
                } else {
                    content.innerHTML = `<div style="color: #ff4757;">Error: ${result.message}</div>`;
                }
            } catch (error) {
                content.innerHTML = `<div style="color: #ff4757;">Error: ${error.message}</div>`;
            }
        }
        
        function closeDetailsModal() {
            document.getElementById('detailsModal').style.display = 'none';
        }
        
        function escapeHtml(text) {
            const div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }
    </script>
    
    <!-- Details Modal -->
    <div id="detailsModal" style="display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.8); z-index: 1000; justify-content: center; align-items: center;">
        <div style="background: #252525; border-radius: 8px; max-width: 800px; max-height: 80vh; width: 90%; display: flex; flex-direction: column; box-shadow: 0 4px 20px rgba(0,0,0,0.5);">
            <div style="padding: 15px; border-bottom: 1px solid #404040; display: flex; justify-content: space-between; align-items: center;">
                <h3 id="detailsModalTitle" style="margin: 0; color: #00d4ff;">Details</h3>
                <button onclick="closeDetailsModal()" style="background: transparent; border: none; color: #888; font-size: 24px; cursor: pointer; padding: 0; width: 30px; height: 30px; display: flex; align-items: center; justify-content: center;" onmouseover="this.style.color='#fff'" onmouseout="this.style.color='#888'">×</button>
            </div>
            <div id="detailsModalContent" style="padding: 20px; overflow-y: auto; flex: 1;"></div>
        </div>
    </div>
    
    <!-- Control Panel Modal - Must be outside all containers for proper z-index -->
    <div id="controlPanelModal" style="display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0, 0, 0, 0.7); z-index: 10000; justify-content: center; align-items: center;">
        <div style="background: #2a2d2e; border-radius: 8px; padding: 30px; max-width: 500px; width: 90%; box-shadow: 0 4px 20px rgba(0,0,0,0.5);">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px;">
                <h2 style="margin: 0; color: #d4d4d4;">⚙️ Control Panel</h2>
                <button onclick="closeControlPanel()" style="background: #555; color: #d4d4d4; border: none; padding: 5px 15px; border-radius: 4px; cursor: pointer; font-size: 18px;">×</button>
            </div>
            
            <div style="margin-bottom: 20px;">
                <h3 style="color: #d4d4d4; font-size: 16px; margin-bottom: 10px;">Execution Control</h3>
                <button id="controlPanelContinuousButton" onclick="toggleContinuousFromPanel()" style="width: 100%; padding: 12px; margin-bottom: 10px; background: #555; color: #888; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">🔄 Continuous</button>
            </div>
            
            <div style="margin-bottom: 20px; padding-top: 20px; border-top: 1px solid #3e3e42;">
                <h3 style="color: #d4d4d4; font-size: 16px; margin-bottom: 10px;">Clear Data</h3>
                <p style="color: #888; font-size: 12px; margin-bottom: 15px;">⚠️ These actions cannot be undone</p>
                
                <button onclick="clearWorldModel()" style="width: 100%; padding: 12px; margin-bottom: 10px; background: #d32f2f; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">🗑️ Clear World Model</button>
                <button onclick="clearMap()" style="width: 100%; padding: 12px; margin-bottom: 10px; background: #d32f2f; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">🗑️ Clear Map</button>
                <button onclick="clearTransients()" style="width: 100%; padding: 12px; margin-bottom: 10px; background: #ff9800; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">🗑️ Clear Transients</button>
                <button onclick="clearPersistents()" style="width: 100%; padding: 12px; background: #d32f2f; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">🗑️ Clear Persistents</button>
            </div>
            
            <div id="controlPanelStatus" style="margin-top: 15px; padding: 10px; background: #333; border-radius: 4px; min-height: 20px; font-size: 12px; color: #888; display: none;"></div>
        </div>
    </div>
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
            
            # Start FastAPI server with custom access log filter
            # Suppress logs for conversation_status endpoint (polling keeps websocket alive)
            import logging
            class FilteredAccessLogger(logging.Filter):
                def filter(self, record):
                    # Suppress access logs for conversation_status endpoint
                    # Check the log message itself (uvicorn logs in format: "IP - "METHOD PATH" STATUS")
                    msg = record.getMessage() if hasattr(record, 'getMessage') else str(record.msg)
                    if '/api/conversation_status' in msg:
                        return False
                    return True
            
            # Configure uvicorn access logger
            uvicorn_access_logger = logging.getLogger("uvicorn.access")
            uvicorn_access_logger.addFilter(FilteredAccessLogger())
            
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
            payload = sample.payload.to_bytes().decode('utf-8')
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/goal
            
            # Try to parse as JSON first, otherwise treat as plain text
            try:
                goal_data = json.loads(payload)
                goal_text = goal_data.get('goal', '')
            except json.JSONDecodeError:
                # Plain text goal
                goal_text = payload
                goal_data = {'goal': goal_text}
            
            # Store goal for this character
            with self.character_goals_lock:
                self.character_goals[character_name] = goal_text
            
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
    
    def execution_state_callback(self, sample):
        """Handle execution state updates from executive node."""
        try:
            state_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            paused = state_data.get('paused', True)
            mode = state_data.get('mode', 'step')
            continuous_mode = state_data.get('continuous_mode', False)
            active_dialog = state_data.get('active_dialog', False)
            
            with self.turn_state_lock:
                self.turn_state['mode'] = mode
            
            # Compute button states
            # Step/Run enabled when paused and in step mode (ready to advance)
            # Stop enabled when running (not paused) or in run mode
            step_enabled = paused and mode == 'step'
            run_enabled = paused and mode == 'step'
            stop_enabled = not paused or mode == 'run' or continuous_mode
            
            # Format as turn_state_update for UI compatibility
            continuous_enabled = True  # Continuous button is always enabled (it's a toggle)
            continuous_label = '✅ Continuous' if continuous_mode else '🔄 Continuous'
            continuous_tooltip = 'Continuous mode ON - click to disable' if continuous_mode else 'Continuous mode OFF - click to enable'
            
            turn_state_update = {
                'type': 'turn_state_update',
                'turn': {
                    'number': 0,  # No turn numbers anymore
                    'mode': mode,
                    'auto_progression': mode == 'run' or continuous_mode,
                    'active_count': 0 if paused else 1,
                    'completed_count': 0,
                    'active_characters': [] if paused else [state_data.get('character', '')],
                    'completed_characters': [],
                    'is_active': not paused
                },
                'active_dialog': active_dialog,
                'buttons': {
                    'autonomous': {
                        'enabled': step_enabled or run_enabled,  # Enable if either step or run would be enabled
                        'label': '🤖 Autonomous',
                        'tooltip': 'Ready - click to start autonomous execution' if (step_enabled or run_enabled) else 'Execution in progress'
                    },
                    'continuous': {
                        'enabled': continuous_enabled,
                        'label': continuous_label,
                        'tooltip': continuous_tooltip,
                        'active': continuous_mode
                    },
                    'stop': {
                        'enabled': stop_enabled,
                        'label': '⏹️ Stop',
                        'tooltip': 'Click to pause execution' if stop_enabled else 'Not running'
                    }
                },
                'task_scheduler': state_data.get('task_scheduler'),
                'character': state_data.get('character', ''),
                'timestamp': state_data.get('timestamp', time.time())
            }

            logger.info(f"🆕 Received execution_state: paused={paused}, mode={mode}, autonomous_enabled={step_enabled or run_enabled}")
            
            # Forward formatted state to websockets for UI rendering
            with self.websocket_lock:
                for ws in self.websocket_connections:
                    try:
                        asyncio.run(ws.send_json(turn_state_update))
                    except Exception as e:
                        logger.error(f"Error sending execution_state to websocket: {e}")
            
        except Exception as e:
            logger.error(f"Error in execution_state_callback: {e}")
            import traceback
            traceback.print_exc()
    
    def world_state_callback(self, sample):
        """Handle incoming world state updates."""
        try:
            world_state_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/world_state
            
            # Send world state update to web clients via WebSocket
            self._send_world_state_to_websockets(world_state_data, character_name)
            
        except Exception as e:
            logger.error(f"Error in world_state_callback: {e}")
            import traceback
            traceback.print_exc()
    
    def _send_world_state_to_websockets(self, world_state_data: Dict[str, Any], character_name: str):
        """Send world state update to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                return
        
        world_state = world_state_data.get('world_state', {})
        
        # Format as JSON
        formatted_json = json.dumps(world_state, indent=2)
        
        web_data = {
            'type': 'world_state_update',
            'character': character_name,
            'world_state': world_state,
            'formatted_json': formatted_json,
            'timestamp': datetime.now().isoformat()
        }
        
        # Send to all WebSocket connections
        with self.websocket_lock:
            disconnected = []
            for ws in self.websocket_connections:
                try:
                    import asyncio
                    if hasattr(self, '_event_loop') and self._event_loop:
                        asyncio.run_coroutine_threadsafe(
                            ws.send_text(json.dumps(web_data)),
                            self._event_loop
                        )
                    else:
                        # Fallback: try direct send (may not work if event loop not set)
                        pass
                except Exception as e:
                    logger.warning(f"Error sending world_state to WebSocket: {e}")
                    disconnected.append(ws)
            
            # Remove disconnected clients
            for ws in disconnected:
                if ws in self.websocket_connections:
                    self.websocket_connections.remove(ws)
    
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
                
                # Send current time to all connected WebSocket clients
                from datetime import datetime
                current_time = datetime.now()
                time_info = {
                    'year': current_time.year,
                    'month': current_time.month,
                    'day': current_time.day,
                    'hour': current_time.hour,
                    'minute': current_time.minute,
                    'datetime': current_time.isoformat(),
                    'period': 'morning' if 5 <= current_time.hour < 12 else 'afternoon' if 12 <= current_time.hour < 17 else 'evening' if 17 <= current_time.hour < 21 else 'night',
                    'season': 'spring' if 3 <= current_time.month <= 5 else 'summer' if 6 <= current_time.month <= 8 else 'autumn' if 9 <= current_time.month <= 11 else 'winter'
                }
                self._send_time_update_to_websockets(time_info)
                logger.info(f'📤 Sent current time to all WebSocket clients: {current_time.isoformat()}')
                
        except Exception as e:
            import traceback
            traceback.print_exc()    

    
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
    
    def _auto_release_user_lock(self, target_character: str):
        """Auto-release User's previous conversation lock (no-op with single character)."""
        # No conversation locks needed with single character
        pass
    
    def _handle_character_announcement(self, action_data: Dict[str, Any], character_name: str):
        """Handle character announcement actions."""
        self.active_characters.add(character_name)
        try:
            sched_cfg = (action_data.get('character_config') or {}).get('task_scheduler', {})
            interval_min = float(sched_cfg.get('interval', 15))
            self.character_scheduler_defaults[character_name] = {
                "enabled": bool(sched_cfg.get('enabled', False)),
                "interval": max(60, int(interval_min * 60)),
            }
        except Exception:
            self.character_scheduler_defaults[character_name] = {"enabled": False, "interval": 60}
        
        # Set active character name for direct control (first announced character)
        if self.active_character_name is None:
            self.active_character_name = character_name
            # Create publishers for direct control
            self.control_step_publisher = self.session.declare_publisher(
                f"cognitive/{character_name}/control/step"
            )
            self.control_run_publisher = self.session.declare_publisher(
                f"cognitive/{character_name}/control/run"
            )
            self.control_autonomous_publisher = self.session.declare_publisher(
                f"cognitive/{character_name}/control/autonomous"
            )
            self.control_stop_publisher = self.session.declare_publisher(
                f"cognitive/{character_name}/control/stop"
            )
            self.control_continuous_publisher = self.session.declare_publisher(
                f"cognitive/{character_name}/control/continuous"
            )
            # Subscribe to execution state updates
            self.execution_state_subscriber = self.session.declare_subscriber(
                f"cognitive/{character_name}/execution_state",
                self.execution_state_callback
            )
            logger.info(f"📡 Direct control publishers and execution state subscriber created for {character_name}")
            
            # Send initial button state to UI (character ready, paused, step mode)
            # This enables buttons immediately on startup
            initial_state = {
                'type': 'turn_state_update',
                'turn': {
                    'number': 0,
                    'mode': 'step',
                    'auto_progression': False,
                    'active_count': 0,
                    'completed_count': 0,
                    'active_characters': [],
                    'completed_characters': [],
                    'is_active': False
                },
                'buttons': {
                    'step': {
                        'enabled': True,
                        'label': '🎯 Step',
                        'tooltip': 'Ready - click to advance one OODA cycle'
                    },
                    'run': {
                        'enabled': True,
                        'label': '🏃 Run',
                        'tooltip': 'Ready - click to run continuously'
                    },
                    'stop': {
                        'enabled': False,
                        'label': '⏹️ Stop',
                        'tooltip': 'Not running'
                    }
                },
                'timestamp': time.time()
            }
            
            # Send to all connected websockets
            with self.websocket_lock:
                for ws in self.websocket_connections:
                    try:
                        asyncio.run(ws.send_json(initial_state))
                    except Exception as e:
                        logger.error(f"Error sending initial state to websocket: {e}")
    
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
