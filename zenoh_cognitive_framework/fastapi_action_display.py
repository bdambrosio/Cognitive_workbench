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
from datetime import datetime
from typing import Dict, List, Any, Set
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn
from pathlib import Path
from concurrent.futures import TimeoutError


class FastAPIActionDisplayNode:
    """
    The FastAPI Action Display node provides:
    - Web UI for displaying incoming actions
    - Text input interface via web
    - Action history and statistics
    """
    
    def __init__(self, port: int = 3000):
        # Initialize FastAPI app
        self.app = FastAPI(title="Zenoh Action Display")
        self.port = port
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Track active characters
        self.active_characters: Set[str] = set()
        self.character_publishers: Dict[str, Any] = {}
        self.last_character_name: str = None
        
        # Turn state tracking with proper locking
        self.turn_state_lock = threading.Lock()
        self.turn_state = {
            'active_characters': [],
            'completed_characters': [],
            'turn_number': 0,
            'turn_start_time': None,
            'mode': 'step'  # 'step' or 'run'
        }
        
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
        
        # Publisher for memory storage
        self.memory_publisher = self.session.declare_publisher("cognitive/memory/store")
        
        # Publishers for turn control
        self.turn_step_publisher = self.session.declare_publisher("cognitive/map/turn/step")
        self.turn_run_publisher = self.session.declare_publisher("cognitive/map/turn/run")
        self.turn_stop_publisher = self.session.declare_publisher("cognitive/map/turn/stop")
        
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
        print('   - Subscribing to: cognitive/map/step_complete (step completion)')
        print('   - Subscribing to: cognitive/map/turn (turn start)')
        print('   - Publishing to: cognitive/{character}/text_input (dynamic)')
        print('   - Publishing to: cognitive/memory/store')
        print(f'   - Web UI available at: http://localhost:{port}')
    
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
                print(f"WebSocket client connected. Total clients: {len(self.websocket_connections)}")
            
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
                print("Test message sent to WebSocket client")
            except Exception as e:
                print(f"Error sending test message: {e}")
            
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
                    except Exception as e:
                        print(f"WebSocket receive error: {e}")
                        break
            except WebSocketDisconnect:
                pass
            except Exception as e:
                print(f"WebSocket error: {e}")
            finally:
                with self.websocket_lock:
                    if websocket in self.websocket_connections:
                        self.websocket_connections.remove(websocket)
                print(f"WebSocket client disconnected. Total clients: {len(self.websocket_connections)}")
        
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
                    f"cognitive/{actual_character_name}/text_input"
                )
            
            # Publish text input
            text_input_data = {
                'source': 'ui',
                'text': message
            }
            self.character_publishers[actual_character_name].put(json.dumps(text_input_data))
            
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
                print('🎯 Step Turn command sent')
                
                return {"success": True, "message": "Step Turn command sent"}
            except Exception as e:
                print(f'❌ Error sending step turn command: {e}')
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/turn/run")
        async def run_turns():
            """Start automatic turn progression."""
            try:
                with self.turn_state_lock:
                    self.turn_state['mode'] = 'run'
                
                self.turn_run_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                print('🏃 Run Turns command sent')
                
                # Send turn state update to enable/disable buttons
                self._send_turn_state_update()
                return {"success": True, "message": "Run Turns command sent"}
            except Exception as e:
                print(f'❌ Error sending run turns command: {e}')
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/turn/stop")
        async def stop_turns():
            """Stop automatic turn progression."""
            try:
                with self.turn_state_lock:
                    self.turn_state['mode'] = 'step'
                
                self.turn_stop_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                print('⏹️ Stop Turns command sent')
                
                # Send turn state update to enable/disable buttons
                self._send_turn_state_update()
                return {"success": True, "message": "Stop Turns command sent"}
            except Exception as e:
                print(f'❌ Error sending stop turns command: {e}')
                return {"success": False, "message": f"Error: {str(e)}"}
    
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
            padding: 20px; 
            background-color: #1a1a1a; 
            color: #e0e0e0; 
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto; 
        }
        .header { 
            background: #2d2d2d; 
            padding: 20px; 
            border-radius: 8px; 
            margin-bottom: 20px; 
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
            padding: 15px; 
            height: 500px; 
            overflow-y: auto; 
            font-family: 'Consolas', 'Monaco', 'Courier New', monospace; 
            font-size: 13px;
            color: #e0e0e0;
        }
        .input-section { 
            background: #2d2d2d; 
            padding: 20px; 
            border-radius: 8px; 
            margin-top: 20px; 
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
            margin: 8px 0; 
            padding: 10px; 
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
        .action-log::-webkit-scrollbar {
            width: 8px;
        }
        .action-log::-webkit-scrollbar-track {
            background: #1a1a1a;
        }
        .action-log::-webkit-scrollbar-thumb {
            background: #404040;
            border-radius: 4px;
        }
        .action-log::-webkit-scrollbar-thumb:hover {
            background: #555;
        }
    </style>
</head>
<body>
    <div class="container">
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
                <button id="stepButton" onclick="stepTurn()" style="background: #4ecdc4; margin-right: 10px;">🎯 Step Turn</button>
                <button onclick="runTurns()" style="background: #ffe66d; color: #1a1a1a; margin-right: 10px;">🏃 Run</button>
                <button onclick="stopTurns()" style="background: #ff6b6b;">⏹️ Stop</button>
            </div>
            <div id="turnStatus" style="margin-bottom: 10px; font-size: 12px; color: #888;">
                <span id="turnMode">Mode: Step</span> | 
                <span id="turnNumber">Turn: 0</span> | 
                <span id="turnProgress">Progress: 0/0</span>
            </div>
            <div id="turnResult" style="margin-top: 10px;"></div>
        </div>
        
        <div class="input-section">
            <h3>Send Text Input</h3>
            <input type="text" id="characterInput" placeholder="Character name (optional)" style="width: 150px;">
            <input type="text" id="messageInput" placeholder="Message" style="width: 300px;">
            <button onclick="sendText()">Send</button>
            <div id="sendResult"></div>
        </div>
    </div>

    <script>
        let ws = null;
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 5;
        const reconnectDelay = 2000;
        
        function connectWebSocket() {
            ws = new WebSocket('ws://localhost:3000/ws');
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
                    entry.innerHTML = `<span class="timestamp">[${new Date().toLocaleTimeString()}]</span> <span style="color: #ff6b6b;">❌ WebSocket connection lost - please refresh page</span>`;
                    actionLog.appendChild(entry);
                    actionLog.scrollTop = actionLog.scrollHeight;
                }
            };
            
            ws.onmessage = function(event) {
                console.log('WebSocket message received:', event.data);
                const data = JSON.parse(event.data);
                if (data.type === 'action') {
                    addActionEntry(data);
                } else if (data.type === 'turn_state') {
                    updateTurnState(data);
                } else if (data.type === 'step_complete') {
                    // Re-enable the Step button when step is complete
                    const stepButton = document.getElementById('stepButton');
                    stepButton.disabled = false;
                    stepButton.style.background = '#4ecdc4';
                    stepButton.title = 'Click to advance to next turn';
                    console.log('Step complete - re-enabled Step button');
                } else if (data.type === 'test') {
                    console.log('Test message received:', data.message);
                    // Add a test entry to the action log
                    const entry = document.createElement('div');
                    entry.className = 'action-entry';
                    entry.innerHTML = `<span class="timestamp">[${new Date().toLocaleTimeString()}]</span> <span style="color: #4ecdc4;">🔗 ${data.message}</span>`;
                    actionLog.appendChild(entry);
                    actionLog.scrollTop = actionLog.scrollHeight;
                }
            };
        }
        
        // Connect WebSocket on page load
        connectWebSocket();
        
        function addActionEntry(actionData) {
            console.log('Adding action entry:', actionData);
            const entry = document.createElement('div');
            entry.className = 'action-entry';
            
            const timestamp = new Date().toLocaleTimeString();
            let html = `<span class="timestamp">[${timestamp}]</span> `;
            html += `<span class="character-name">[${actionData.character.toUpperCase()}]</span> `;
            html += `<span class="action-type">${actionData.action_type}</span>`;
            
            // For text-only actions (Say/response), only show the text
            if (actionData.is_text_only) {
                const textContent = actionData.text || actionData.input_text || actionData.llm_response || '';
                if (textContent) {
                    html += `<br><span class="response-text">"${textContent}"</span>`;
                }
            } else {
                // Display action details if available
                if (actionData.action || actionData.target || actionData.value) {
                    let actionDetails = [];
                    if (actionData.action) actionDetails.push(`Action: ${actionData.action}`);
                    if (actionData.target) actionDetails.push(`Target: ${actionData.target}`);
                    if (actionData.value) actionDetails.push(`Value: ${actionData.value}`);
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
        
        function updateTurnState(turnData) {
            const turnMode = document.getElementById('turnMode');
            const turnNumber = document.getElementById('turnNumber');
            const turnProgress = document.getElementById('turnProgress');
            
            // Update display
            turnMode.textContent = `Mode: ${turnData.mode.charAt(0).toUpperCase() + turnData.mode.slice(1)}`;
            turnNumber.textContent = `Turn: ${turnData.turn_number}`;
            turnProgress.textContent = `Progress: ${turnData.completed_characters.length}/${turnData.active_characters.length}`;
            
            console.log(`Turn state update: mode=${turnData.mode}, active=${turnData.active_characters.length}, completed=${turnData.completed_characters.length}`);
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
        
        // Allow Enter key to send message
        document.getElementById('messageInput').addEventListener('keypress', function(e) {
            if (e.key === 'Enter') {
                sendText();
            }
        });
        
        async function stepTurn() {
            const resultDiv = document.getElementById('turnResult');
            const stepButton = document.getElementById('stepButton');
            
            // Immediately disable and shade the button for responsive UI
            stepButton.disabled = true;
            stepButton.style.background = '#555';
            stepButton.title = 'Waiting for all characters to complete their turns...';
            
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
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.message}</span>`;
                    // Re-enable button if API call failed
                    stepButton.disabled = false;
                    stepButton.style.background = '#4ecdc4';
                    stepButton.title = 'Click to advance to next turn';
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
                // Re-enable button if API call failed
                stepButton.disabled = false;
                stepButton.style.background = '#4ecdc4';
                stepButton.title = 'Click to advance to next turn';
            }
        }
        
        async function runTurns() {
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
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.message}</span>`;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
            }
        }
        
        async function stopTurns() {
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
                } else {
                    resultDiv.innerHTML = `<span class="error">Error: ${result.message}</span>`;
                }
            } catch (error) {
                resultDiv.innerHTML = `<span class="error">Error: ${error.message}</span>`;
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
            print(f"Event loop set: {self.event_loop}")
        except RuntimeError:
            print("No running event loop found")
    
    def action_callback(self, sample):
        """Handle incoming actions."""
        try:
            action_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.action_counter += 1
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/action
            
            print(f'📥 Received action from {character_name}: {action_data.get("type", "unknown")}')
            print(f'   Raw action data: {action_data}')
            
            # Track active character
            self.active_characters.add(character_name)
            
            # Handle character announcements
            if action_data.get('type') == 'announcement':
                self._handle_character_announcement(action_data, character_name)
                return
            
            # Store action in memory
            self._store_action_in_memory(action_data, character_name)
            
            # Send to web clients
            self._send_to_websockets(action_data, character_name)
            
        except Exception as e:
            print(f'❌ Error handling action: {e}')
            import traceback
            traceback.print_exc()
    
    def step_complete_callback(self, sample):
        """Handle step complete events from map node."""
        try:
            step_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            print(f'🎯 Step complete received: {step_data}')
            
            # Update turn state to indicate step is complete
            with self.turn_state_lock:
                self.turn_state['active_characters'] = []
                self.turn_state['completed_characters'] = []
                self.turn_state['turn_start_time'] = None
            print('✅ Step button will be re-enabled')
            
            # Send step_complete message to re-enable Step button
            self._send_step_complete_to_websockets()
            
        except Exception as e:
            print(f'❌ Error handling step complete: {e}')
            import traceback
            traceback.print_exc()
    
    def turn_start_callback(self, sample):
        """Handle turn start events from map node."""
        try:
            turn_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            print(f'🚦 Turn start received: {turn_data}')
            
            with self.turn_state_lock:
                self.turn_state['turn_number'] = turn_data.get('turn_number', self.turn_state['turn_number'] + 1)
                self.turn_state['active_characters'] = turn_data.get('active_characters', [])
                self.turn_state['completed_characters'] = []
                self.turn_state['turn_start_time'] = time.time()
            
            print(f'🚦 Turn {self.turn_state["turn_number"]} started for: {", ".join(self.turn_state["active_characters"])}')
            
            # Send turn state update to web clients
            self._send_turn_state_update()
            
        except Exception as e:
            print(f'❌ Error handling turn start: {e}')
            import traceback
            traceback.print_exc()
    
    def _send_to_websockets(self, action_data: Dict[str, Any], character_name: str):
        """Send action data to all connected WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                print("No WebSocket clients connected")
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
            'target': action_data.get('target', '') if not is_text_only else '',
            'value': action_data.get('value', '') if not is_text_only else '',
            'metadata': action_data.get('metadata', {}),
            'raw_data': action_data
        }
        
        print(f"Sending action to {len(self.websocket_connections)} WebSocket clients")
        
        # Send to all connected clients
        if self.event_loop is None:
            print("No event loop available, skipping WebSocket send")
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
                    future.result(timeout=5.0)
                except Exception as e:
                    print(f"WebSocket send error for action: {e}")
                    # Don't remove client on timeout - just log the error
                    if isinstance(e, TimeoutError):
                        print(f"WebSocket send timeout for action - keeping client")
                    else:
                        print(f"WebSocket send error - removing client")
                        disconnected.append(websocket)
            
            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
                    print(f"Removed disconnected WebSocket client. Remaining: {len(self.websocket_connections)}")
    
    def _send_turn_state_update(self):
        """Send the current turn state to all WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                print("No WebSocket clients connected, skipping turn state update.")
                return

        with self.turn_state_lock:
            turn_state_data = {
                'type': 'turn_state',
                'turn_number': self.turn_state['turn_number'],
                'active_characters': self.turn_state['active_characters'],
                'completed_characters': self.turn_state['completed_characters'],
                'mode': self.turn_state['mode'],
                'turn_start_time': self.turn_state['turn_start_time']
            }

        print(f"Sending turn state update to {len(self.websocket_connections)} WebSocket clients")

        if self.event_loop is None:
            print("No event loop available, skipping turn state update WebSocket send")
            return

        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(turn_state_data)),
                        self.event_loop
                    )
                    future.result(timeout=5.0)
                except Exception as e:
                    print(f"WebSocket send error for turn state update: {e}")
                    # Don't remove client on timeout - just log the error
                    if isinstance(e, TimeoutError):
                        print(f"WebSocket send timeout for turn state update - keeping client")
                    else:
                        print(f"WebSocket send error - removing client")
                        import traceback
                        traceback.print_exc()
                        disconnected.append(websocket)

            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
                    print(f"Removed disconnected WebSocket client during turn state update. Remaining: {len(self.websocket_connections)}")
    
    def _send_step_complete_to_websockets(self):
        """Send a dedicated step_complete message to all WebSocket clients."""
        with self.websocket_lock:
            if not self.websocket_connections:
                print("No WebSocket clients connected, skipping step complete message.")
                return

        step_complete_data = {
            'type': 'step_complete'
        }

        print(f"Sending step complete message to {len(self.websocket_connections)} WebSocket clients")

        if self.event_loop is None:
            print("No event loop available, skipping step complete message WebSocket send")
            return

        with self.websocket_lock:
            disconnected = []
            for websocket in self.websocket_connections:
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        websocket.send_text(json.dumps(step_complete_data)),
                        self.event_loop
                    )
                    future.result(timeout=5.0)
                except Exception as e:
                    print(f"WebSocket send error for step complete message: {e}")
                    # Don't remove client on timeout - just log the error
                    if isinstance(e, TimeoutError):
                        print(f"WebSocket send timeout for step complete message - keeping client")
                    else:
                        print(f"WebSocket send error - removing client")
                        import traceback
                        traceback.print_exc()
                        disconnected.append(websocket)

            # Remove only truly disconnected clients (not timeout errors)
            for websocket in disconnected:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
                    print(f"Removed disconnected WebSocket client during step complete message. Remaining: {len(self.websocket_connections)}")
    
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
                'source': 'ui',
                'timestamp': datetime.now().isoformat()
            }
            self.memory_publisher.put(json.dumps(memory_data))
        except Exception as e:
            print(f'❌ Error storing text input in memory: {e}')
    
    def _handle_character_announcement(self, action_data: Dict[str, Any], character_name: str):
        """Handle character announcement actions."""
        print(f'🎉 Character announced: {character_name}')
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
    args = parser.parse_args()
    
    node = FastAPIActionDisplayNode(port=args.port)
    node.run()


if __name__ == "__main__":
    main() 