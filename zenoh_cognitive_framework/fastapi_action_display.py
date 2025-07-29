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
        
        # Character goals tracking
        self.character_goals: Dict[str, str] = {}  # character_name -> current_goal_string
        self.character_goals_lock = threading.Lock()
        
        # Character decided actions tracking
        self.character_decided_actions: Dict[str, str] = {}  # character_name -> decided_action_string
        self.character_decided_actions_lock = threading.Lock()
        
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
        print('   - Subscribing to: cognitive/*/goal (character goals)')
        print('   - Subscribing to: cognitive/*/decided_action (character decided actions)')
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
                
                return {"success": True, "message": "Step Turn command sent"}
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/turn/run")
        async def run_turns():
            """Start automatic turn progression."""
            try:
                with self.turn_state_lock:
                    self.turn_state['mode'] = 'run'
                
                self.turn_run_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                
                # Send turn state update to enable/disable buttons
                self._send_turn_state_update()
                return {"success": True, "message": "Run Turns command sent"}
            except Exception as e:
                return {"success": False, "message": f"Error: {str(e)}"}
        
        @self.app.post("/api/turn/stop")
        async def stop_turns():
            """Stop automatic turn progression."""
            try:
                with self.turn_state_lock:
                    self.turn_state['mode'] = 'step'
                
                self.turn_stop_publisher.put(json.dumps({"timestamp": datetime.now().isoformat()}).encode())
                
                # Send turn state update to enable/disable buttons
                self._send_turn_state_update()
                return {"success": True, "message": "Stop Turns command sent"}
            except Exception as e:
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
            width: 250px;
            background: #252525;
            border-right: 1px solid #404040;
            display: flex;
            flex-direction: column;
            overflow-y: auto;
        }
        
        .sidebar-header {
            background: #2d2d2d;
            padding: 15px;
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
            padding: 15px;
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
            margin-bottom: 8px;
            padding: 6px 8px;
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
        
        .character-tab {
            padding: 12px 15px;
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
            padding: 15px 20px;
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
            padding: 20px;
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
            height: 400px; 
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
        <div class="character-sidebar">
            <div class="sidebar-header">
                👥 Characters
            </div>
            <div class="character-tabs" id="characterTabs">
                <!-- Character tabs will be added dynamically -->
            </div>
            
            <!-- Character data area -->
            <div class="character-data-area" id="characterDataArea">
                <div class="character-data-header">Character Data</div>
                <div id="characterDataItems">
                    <div style="color: #888; font-style: italic; text-align: center; padding: 20px;">
                        Select a character to view data
                    </div>
                </div>
            </div>
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
                    <textarea id="messageInput" placeholder="Message or Plan (multi-line supported)" style="width: 300px; height: 80px; resize: vertical;"></textarea>
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
        let activeCharacter = null;
        
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
                    // Check if this is an announcement to create a character tab
                    if (data.action_type === 'announcement') {
                        createCharacterTab(data.character);
                    }
                } else if (data.type === 'goal') {
                    handleGoalUpdate(data);
                } else if (data.type === 'decided_action') {
                    handleDecidedActionUpdate(data);
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
                decidedAction: null
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
            
            // Update character data display
            updateCharacterDataDisplay(characterName);
            
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
                content += `<div class="character-goal">🎯 Goal: ${tabData.goal}</div>`;
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
        
        function updateCharacterDataDisplay(characterName) {
            const dataItemsDiv = document.getElementById('characterDataItems');
            const tabData = characterTabs.get(characterName);
            
            if (!tabData) {
                dataItemsDiv.innerHTML = '<div style="color: #888; font-style: italic; text-align: center; padding: 20px;">Character not found</div>';
                return;
            }
            
            let content = '';
            
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
            
            # Send turn state update to web clients
            self._send_turn_state_update()
            
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
            'target': action_data.get('target', '') if not is_text_only else '',
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
                    future.result(timeout=5.0)
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
                    future.result(timeout=5.0)
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
                    future.result(timeout=5.0)
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
                    future.result(timeout=5.0)
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
                'source': 'ui',
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