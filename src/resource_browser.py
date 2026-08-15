#!/usr/bin/env python3
"""
Standalone Resource Browser - Browse Notes and Collections in running map_node.

Connects to map_node via Zenoh and provides web UI for viewing raw resource content.
Read-only, manual refresh.
"""

import argparse
import json
import logging
import signal
import sys
import webbrowser
from pathlib import Path
from typing import Dict, List, Optional

import zenoh
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse
import uvicorn

_console_handler = logging.StreamHandler()
_console_handler.setLevel(logging.WARNING)
# Anchor log path to the repo root so we land in <repo>/logs/ no
# matter where the launcher subprocess starts us from.
_LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
_LOG_DIR.mkdir(parents=True, exist_ok=True)
_file_handler = logging.FileHandler(_LOG_DIR / 'resource_browser.log', mode='w')
_file_handler.setLevel(logging.INFO)
_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
_console_handler.setFormatter(_formatter)
_file_handler.setFormatter(_formatter)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[_console_handler, _file_handler],
    force=True
)
logger = logging.getLogger(__name__)


class ResourceBrowser:
    """Browse Notes and Collections from map_node."""

    def __init__(self, map_name: str = "infolab", port: int = 3001, open_browser: bool = True, character: str = ""):
        self.map_name = map_name
        self.port = port
        self.open_browser = open_browser
        self.character_name = character
        self.app = FastAPI(title="Resource Browser")
        self.shutdown_requested = False
        
        # Zenoh session (localhost only)
        from utils.zenoh_utils import make_localhost_config
        self.session = zenoh.open(make_localhost_config())
        logger.info(f"Connected to Zenoh for map: {map_name}")
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Setup routes
        self.setup_routes()
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.info(f'Received signal {signum}, initiating shutdown...')
        self.shutdown()
    
    def shutdown(self):
        """Clean shutdown."""
        if getattr(self, '_shutting_down', False):
            return
        self._shutting_down = True
        self.shutdown_requested = True
        try:
            if hasattr(self, 'session'):
                self.session.close()
            logger.info('Resource Browser shutdown complete')
        except Exception as e:
            logger.error(f'Error during shutdown: {e}')
    
    def setup_routes(self):
        """Setup FastAPI routes."""
        
        @self.app.get("/", response_class=HTMLResponse)
        async def index():
            return self.get_html()
        
        @self.app.get("/api/resources")
        async def list_resources(character: str = ""):
            """Get list of all resources from map_node."""
            return self.query_resources(character)
        
        @self.app.get("/api/resource/{resource_id}")
        async def get_resource(resource_id: str, character: str = ""):
            """Get specific resource content."""
            return self.query_resource(resource_id, character)
        
        @self.app.delete("/api/resource/{resource_id}")
        async def delete_resource(resource_id: str, character: str = ""):
            """Delete a resource."""
            return self.delete_resource_via_zenoh(resource_id, character)

        @self.app.put("/api/resource/{resource_id}")
        async def update_resource(resource_id: str, request: Request,
                                  character: str = ""):
            """Update a Note's content."""
            body = await request.json()
            content = body.get('content', '')
            return self.update_resource_via_zenoh(resource_id, content, character)

        @self.app.get("/api/graph/entities")
        async def graph_entities(character: str = ""):
            """Get entity index summary for graph explorer."""
            return self.query_graph_entities(character)

        @self.app.post("/api/graph/subgraph")
        async def graph_subgraph(request: Request, character: str = ""):
            """Get subgraph expansion or search results."""
            body = await request.json()
            return self.query_graph_subgraph(body, character)

        @self.app.get("/api/concerns")
        async def list_concerns(character: str = ""):
            """Get concerns from executive_node."""
            return self.query_concerns(character)

        @self.app.get("/api/context")
        async def get_context(character: str = ""):
            """Return browser context (map, character) for frontend.

            Echoes the requested character so the page reports the agent it
            is actually showing, not the one the process was launched with.
            """
            return {"map": self.map_name,
                    "character": self._effective_character(character)}

        @self.app.post("/api/concern/{character}/manage")
        async def manage_concern(character: str, request: Request):
            """Manage a concern (close/resolve/abandon)."""
            body = await request.json()
            return self.manage_concern_via_zenoh(character, body)
    
    def _key_prefix(self, character: str = "") -> str:
        """Return Zenoh key prefix for the target character (or wildcard fallback).

        `character` is the per-request override supplied as a ?character=
        query param; it falls back to the one passed at launch. Per-request
        rather than per-process so one browser on one port can switch
        agents — a multi-agent session otherwise needs a process and a port
        each, and the edit path breaks entirely without a concrete name.
        """
        c = self._effective_character(character)
        if c:
            return f"cognitive/{c}"
        return "cognitive/*"

    def _effective_character(self, character: str = "") -> str:
        return (character or self.character_name or "").strip()

    def _target(self, character: str = ""):
        """BEST_MATCHING when a character is known, ALL when wildcarding."""
        from zenoh import QueryTarget
        return (QueryTarget.BEST_MATCHING if self._effective_character(character)
                else QueryTarget.ALL)

    def query_resources(self, character: str = "") -> Dict:
        """Query executive_node for resource list."""
        key = f"{self._key_prefix(character)}/resources"
        logger.info(f"Querying: {key}")

        from zenoh import QueryTarget, ConsolidationMode
        target = self._target(character)
        best = None
        best_count = -1
        for reply in self.session.get(key, target=target, consolidation=ConsolidationMode.NONE, timeout=2.0):
            if reply.ok:
                try:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                if data.get('success'):
                    resources = data.get('resources', [])
                    if len(resources) > best_count:
                        best = data
                        best_count = len(resources)

        if best:
            resources = best.get('resources', [])
            notes = [r for r in resources if r.get('id', '').startswith('Note_')]
            collections = [r for r in resources if r.get('id', '').startswith('Collection_')]
            logger.info(f"Found {len(notes)} Notes, {len(collections)} Collections (from {best_count} total resources)")
            return {
                'success': True,
                'notes': notes,
                'collections': collections
            }

        return {'success': False, 'error': 'No response from executive_node'}
    
    def query_resource(self, resource_id: str, character: str = "") -> Dict:
        """Query executive_node for specific resource."""
        key = f"{self._key_prefix(character)}/resource/{resource_id}"
        logger.info(f"Querying: {key}")

        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(key, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                return data

        return {'success': False, 'error': f'Resource {resource_id} not found'}
    
    def delete_resource_via_zenoh(self, resource_id: str, character: str = "") -> Dict:
        """Delete resource via Zenoh query to executive_node."""
        key = f"{self._key_prefix(character)}/resource/remove/{resource_id}"
        logger.info(f"Deleting resource: {key}")
        
        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(key, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                return data
        
        return {'success': False, 'error': f'No response from executive_node for deletion'}

    def update_resource_via_zenoh(self, resource_id: str, content: str, character: str = "") -> Dict:
        """Update Note content via Zenoh query to executive_node."""
        key = f"{self._key_prefix(character)}/resource/update/{resource_id}"
        logger.info(f"Updating resource: {key}")

        payload = json.dumps({'content': content}).encode('utf-8')

        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(key, payload=payload, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                return data

        return {'success': False, 'error': f'No response from executive_node for update'}

    def query_graph_entities(self, character: str = "") -> Dict:
        """Query executive_node for entity index summary."""
        key = f"{self._key_prefix(character)}/graph/entities"
        logger.info(f"Querying graph entities: {key}")

        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(key, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=5.0):
            if reply.ok:
                return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))

        return {'success': False, 'error': 'No response from executive_node for graph entities'}

    def query_graph_subgraph(self, params: Dict, character: str = "") -> Dict:
        """Query executive_node for cognitive graph subgraph."""
        key = f"{self._key_prefix(character)}/graph/subgraph"
        logger.info(f"Querying graph subgraph: {key}")
        payload = json.dumps(params).encode('utf-8')

        from zenoh import QueryTarget, ConsolidationMode
        for reply in self.session.get(key, payload=payload, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=5.0):
            if reply.ok:
                return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))

        return {'success': False, 'error': 'No response from executive_node for graph subgraph'}

    def query_concerns(self, character: str = "") -> Dict:
        """Query executive_node for concerns (user + derived + activations)."""
        key = f"{self._key_prefix(character)}/concerns"
        logger.info(f"Querying concerns: {key}")

        from zenoh import QueryTarget, ConsolidationMode
        target = self._target(character)
        for reply in self.session.get(key, target=target, consolidation=ConsolidationMode.NONE, timeout=2.0):
            if reply.ok:
                try:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        return data
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue

        return {'success': False, 'error': 'No response from executive_node for concerns'}

    def manage_concern_via_zenoh(self, character: str, params: Dict) -> Dict:
        """Send a concern management command to the chat loop and wait
        for a typed reply. The chat-side handler is registered as a
        queryable, so we use session.get (not session.put — that would
        be a fire-and-forget publish to a topic with no subscriber)."""
        key = f"cognitive/{character}/control/concern_manage"
        payload = json.dumps(params).encode('utf-8')

        from zenoh import QueryTarget, ConsolidationMode
        try:
            for reply in self.session.get(
                key, payload=payload,
                target=QueryTarget.BEST_MATCHING,
                consolidation=ConsolidationMode.NONE,
                timeout=5.0,
            ):
                if reply.ok:
                    return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
        except Exception as e:
            return {'success': False, 'error': str(e)}

        return {'success': False,
                'error': f'No response from {character} for concern_manage'}

    def get_html(self) -> str:
        """Generate HTML UI."""
        return """
<!DOCTYPE html>
<html>
<head>
    <title>Resource Browser</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #1e1e1e;
            color: #d4d4d4;
            height: 100vh;
            display: flex;
            flex-direction: column;
        }
        header {
            background: #252526;
            padding: 12px 20px;
            border-bottom: 1px solid #3e3e42;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        h1 { 
            font-size: 18px; 
            font-weight: 500;
            color: #cccccc;
        }
        button {
            background: #0e639c;
            color: white;
            border: none;
            padding: 6px 14px;
            border-radius: 2px;
            cursor: pointer;
            font-size: 13px;
        }
        button:hover { background: #1177bb; }
        button:active { background: #0d5585; }
        
        .container {
            display: flex;
            flex: 1;
            overflow: hidden;
        }
        .sidebar {
            width: 300px;
            background: #252526;
            border-right: 1px solid #3e3e42;
            overflow-y: auto;
            padding: 10px;
        }
        .section {
            margin-bottom: 20px;
        }
        .section-title {
            font-size: 11px;
            font-weight: 600;
            text-transform: uppercase;
            color: #888;
            margin-bottom: 8px;
            letter-spacing: 0.5px;
        }
        .resource-item {
            padding: 6px 10px;
            margin: 2px 0;
            cursor: pointer;
            border-radius: 3px;
            font-size: 13px;
            font-family: 'Consolas', monospace;
        }
        .resource-item:hover {
            background: #2a2d2e;
        }
        .resource-item.selected {
            background: #094771;
        }
        .content-area {
            flex: 1;
            padding: 20px;
            overflow-y: auto;
        }
        .content-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 15px;
            padding-bottom: 10px;
            border-bottom: 1px solid #3e3e42;
        }
        .content-title {
            font-size: 16px;
            font-weight: 500;
            font-family: 'Consolas', monospace;
            color: #4ec9b0;
        }
        .content-body {
            background: #1e1e1e;
            border: 1px solid #3e3e42;
            border-radius: 4px;
            padding: 15px;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 13px;
            line-height: 1.6;
            white-space: pre-wrap;
            word-wrap: break-word;
        }
        .empty-state {
            text-align: center;
            color: #666;
            padding: 60px 20px;
        }
        .metadata {
            background: #2d2d30;
            padding: 10px;
            border-radius: 3px;
            margin-bottom: 15px;
            font-size: 12px;
        }
        .metadata-item {
            margin: 5px 0;
            color: #888;
            text-align: left;
        }
        .metadata-item span {
            color: #dcdcaa;
        }
        .metadata-key {
            color: #888;
            margin-bottom: 2px;
        }
        .metadata-value {
            margin: 0;
            padding: 8px 10px;
            background: #232326;
            border: 1px solid #3e3e42;
            border-radius: 3px;
            color: #dcdcaa;
            white-space: pre-wrap;
            word-break: break-word;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 12px;
            line-height: 1.45;
            text-align: left;
        }
        .copy-btn {
            background: #3e3e42;
            font-size: 12px;
            padding: 4px 10px;
        }
        .copy-btn:hover { background: #4e4e52; }
        .edit-textarea {
            width: 100%;
            min-height: 200px;
            background: #1e1e1e;
            border: 1px solid #3e3e42;
            border-radius: 4px;
            padding: 15px;
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 13px;
            line-height: 1.6;
            color: #d4d4d4;
            resize: vertical;
        }
        .edit-textarea:focus {
            outline: none;
            border-color: #007acc;
        }
        .edit-actions {
            display: flex;
            gap: 8px;
            margin-top: 10px;
        }
        .save-btn {
            background: #388a34;
            font-size: 12px;
            padding: 6px 14px;
        }
        .save-btn:hover { background: #45a340; }
        .cancel-btn {
            background: #6c6c6c;
            font-size: 12px;
            padding: 6px 14px;
        }
        .cancel-btn:hover { background: #7e7e7e; }
        
        .context-menu {
            position: fixed;
            background: #252526;
            border: 1px solid #3e3e42;
            border-radius: 3px;
            padding: 4px 0;
            z-index: 1000;
            min-width: 150px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.3);
            display: none;
        }
        .context-menu-item {
            padding: 6px 16px;
            cursor: pointer;
            font-size: 13px;
            color: #d4d4d4;
        }
        .context-menu-item:hover {
            background: #2a2d2e;
        }
        .context-menu-item.delete {
            color: #f48771;
        }
        .context-menu-item.delete:hover {
            background: #3a1f1a;
        }
        
        ::-webkit-scrollbar { width: 10px; }
        ::-webkit-scrollbar-track { background: #1e1e1e; }
        ::-webkit-scrollbar-thumb { background: #424242; border-radius: 5px; }
        ::-webkit-scrollbar-thumb:hover { background: #4e4e4e; }

        /* Tab switcher */
        .tab-bar { display: flex; gap: 2px; margin-bottom: 10px; }
        .tab-btn { background: #2d2d2d; color: #888; border: none; padding: 5px 12px; cursor: pointer; font-size: 11px; text-transform: uppercase; letter-spacing: 0.5px; border-radius: 2px; }
        .tab-btn.active { background: #094771; color: #d4d4d4; }
        .tab-btn:hover { background: #3e3e42; color: #d4d4d4; }
        .tab-panel { display: none; }
        .tab-panel.active { display: block; }

        /* Graph explorer */
        #graph-panel { display: none; position: relative; width: 100%; height: 100%; }
        #graph-panel.active { display: flex; flex-direction: column; }
        #graph-controls { padding: 8px; background: #252526; border-bottom: 1px solid #3e3e42; display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
        #graph-search { background: #3c3c3c; border: 1px solid #3e3e42; color: #d4d4d4; padding: 4px 8px; font-size: 12px; border-radius: 2px; width: 180px; }
        .filter-label { font-size: 10px; color: #888; cursor: pointer; display: flex; align-items: center; gap: 2px; }
        .filter-label input { cursor: pointer; }
        #graph-svg { flex: 1; background: #1a1a1a; }
        .graph-node { cursor: pointer; }
        .graph-node:hover { filter: brightness(1.3); }
        .graph-edge { stroke: #888; stroke-opacity: 0.8; marker-end: url(#arrowhead); }
        .graph-edge-label { fill: #999; font-size: 8px; pointer-events: none; text-anchor: middle; }
        .graph-tooltip { position: absolute; background: #333; color: #d4d4d4; padding: 6px 10px; border-radius: 3px; font-size: 12px; pointer-events: none; max-width: 300px; z-index: 100; border: 1px solid #555; }
    </style>
    <script src="https://d3js.org/d3.v7.min.js"></script>
</head>
<body>
    <header>
        <h1>🔍 Resource Browser - <span id="map-name">infolab</span></h1>
        <button onclick="refreshResources()">↻ Refresh</button>
    </header>
    
    <div class="container">
        <div class="sidebar">
            <div class="tab-bar">
                <button class="tab-btn active" onclick="switchTab('resources')">Resources</button>
                <button class="tab-btn" onclick="switchTab('graph')">Graph</button>
                <button class="tab-btn" onclick="switchTab('concerns')">Concerns</button>
            </div>
            <div id="resources-tab" class="tab-panel active">
                <div class="section">
                    <div class="section-title">Notes (<span id="notes-count">0</span>)</div>
                    <div id="notes-list"></div>
                </div>
                <div class="section">
                    <div class="section-title">Collections (<span id="collections-count">0</span>)</div>
                    <div id="collections-list"></div>
                </div>
            </div>
            <div id="graph-sidebar" class="tab-panel">
                <div class="section">
                    <div class="section-title">Entities (<span id="entity-count">0</span>)</div>
                    <div id="entity-list"></div>
                </div>
            </div>
            <div id="concerns-tab" class="tab-panel">
                <div class="section">
                    <div class="section-title">User Concerns (<span id="user-concerns-count">0</span>)</div>
                    <div id="user-concerns-list"></div>
                </div>
                <div class="section">
                    <div class="section-title">Derived Concerns (<span id="derived-concerns-count">0</span>)</div>
                    <div id="derived-concerns-list"></div>
                </div>
            </div>
        </div>

        <div class="content-area" id="resource-content">
            <div id="content-display" class="empty-state">
                <p>← Select a resource to view its content</p>
            </div>
        </div>
        <div id="graph-panel">
            <div id="graph-controls" style="display:flex;flex-wrap:wrap;align-items:center;gap:4px 10px;">
                <input type="text" id="graph-search" placeholder="Search entities/nodes..." onkeydown="if(event.key==='Enter')graphSearch()">
                <button onclick="graphSearch()" style="font-size:11px;padding:4px 8px;">Search</button>
                <button onclick="loadGraphByTypes()" style="font-size:11px;padding:4px 8px;background:#2d6b3f;">Load</button>
                <span style="color:#666;font-size:11px;">show:</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="entity" checked onchange="applyTypeFilter()"> entities</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="goal_launch,goal_outcome" onchange="applyTypeFilter()"> goals</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="concern_created,concern_change" onchange="applyTypeFilter()"> concerns</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="conversation_turn" onchange="applyTypeFilter()"> turns</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="tom_update" onchange="applyTypeFilter()"> ToM</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="task_created" onchange="applyTypeFilter()"> tasks</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="decision,action_result" onchange="applyTypeFilter()"> decisions</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="event" onchange="applyTypeFilter()"> events</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="assessment" onchange="applyTypeFilter()"> assessments</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="note" onchange="applyTypeFilter()"> notes</span>
                <span class="filter-label"><input type="checkbox" class="seed-type" value="consolidation" onchange="applyTypeFilter()"> consolidations</span>
            </div>
            <svg id="graph-svg"></svg>
            <div id="graph-tooltip" class="graph-tooltip" style="display:none;"></div>
        </div>
    </div>
    
    <div id="context-menu" class="context-menu">
        <div class="context-menu-item delete" onclick="handleDelete()">Delete</div>
    </div>
    
    <script>
        let currentResources = {notes: [], collections: []};
        let selectedResource = null;
        let contextMenuResource = null;
        
        async function refreshResources() {
            try {
                const response = await fetch(apiUrl('/api/resources'));
                const data = await response.json();
                
                if (data.success) {
                    currentResources = data;
                    renderResourceLists();
                } else {
                    alert('Error: ' + data.error);
                }
            } catch (e) {
                alert('Failed to fetch resources: ' + e.message);
            }
        }
        
        function renderResourceLists() {
            const notesList = document.getElementById('notes-list');
            const collectionsList = document.getElementById('collections-list');
            
            document.getElementById('notes-count').textContent = currentResources.notes.length;
            document.getElementById('collections-count').textContent = currentResources.collections.length;
            
            function resourceItemHtml(r) {
                const resId = r.id || r.name;
                const props = r.properties || {};
                const friendlyName = props.note_name || props.collection_name || '';
                const nameHtml = friendlyName
                    ? `<div style="color:#9cdcfe;font-size:11px;margin-top:1px;">${escapeHtml(friendlyName)}</div>`
                    : '';
                return `<div class="resource-item" data-resource-id="${resId}" onclick="selectResource('${resId}')" oncontextmenu="showContextMenu(event, '${resId}'); return false;">${resId}${nameHtml}</div>`;
            }

            notesList.innerHTML = currentResources.notes.map(resourceItemHtml).join('');
            collectionsList.innerHTML = currentResources.collections.map(resourceItemHtml).join('');
        }
        
        function showContextMenu(event, resourceId) {
            const menu = document.getElementById('context-menu');
            contextMenuResource = resourceId;
            menu.style.display = 'block';
            menu.style.left = event.pageX + 'px';
            menu.style.top = event.pageY + 'px';
            
            // Close menu on click outside
            setTimeout(() => {
                document.addEventListener('click', function closeMenu() {
                    menu.style.display = 'none';
                    document.removeEventListener('click', closeMenu);
                });
            }, 0);
        }
        
        async function handleDelete() {
            if (!contextMenuResource) return;
            
            const resourceId = contextMenuResource;
            if (!confirm(`Delete ${resourceId}?`)) {
                return;
            }
            
            try {
                const response = await fetch(apiUrl(`/api/resource/${resourceId}`), {
                    method: 'DELETE'
                });
                const data = await response.json();
                
                if (data.success) {
                    // Clear selection if deleted resource was selected
                    if (selectedResource === resourceId) {
                        selectedResource = null;
                        document.getElementById('content-display').innerHTML = '<div class="empty-state"><p>← Select a resource to view its content</p></div>';
                    }
                    // Refresh resource list
                    refreshResources();
                } else {
                    alert('Error: ' + (data.error || 'Failed to delete resource'));
                }
            } catch (e) {
                alert('Failed to delete resource: ' + e.message);
            }
            
            document.getElementById('context-menu').style.display = 'none';
        }
        
        async function selectResource(resourceId) {
            // Update selected styling
            document.querySelectorAll('.resource-item').forEach(el => {
                el.classList.toggle('selected', el.dataset.resourceId === resourceId);
            });
            
            selectedResource = resourceId;
            
            try {
                const response = await fetch(apiUrl(`/api/resource/${resourceId}`));
                const data = await response.json();
                
                console.log('Received data:', data);
                
                if (!data) {
                    alert('Error: Empty response from server');
                    return;
                }
                
                if (data.success === false) {
                    alert('Error: ' + (data.error || 'Unknown error'));
                    return;
                }
                
                // Check if wrapped format
                if (data.resource) {
                    console.log('Using wrapped format, resource:', data.resource);
                    displayResource(data.resource);
                } else if (data.id || data.name) {
                    // Direct resource format (has id or name field)
                    console.log('Using direct format, resource:', data);
                    displayResource(data);
                } else {
                    console.error('Unexpected data format:', data);
                    alert('Error: Unexpected response format. Check console for details.');
                }
            } catch (e) {
                console.error('Fetch error:', e);
                alert('Failed to fetch resource: ' + e.message);
            }
        }
        
        // Track original content for cancel
        let originalContent = '';

        function displayResource(resource) {
            const display = document.getElementById('content-display');

            // Validate resource structure (accept either id or name field)
            const resourceId = resource.id || resource.name;
            if (!resource || !resourceId) {
                display.innerHTML = '<div class="empty-state"><p>Error: Invalid resource data</p></div>';
                console.error('Invalid resource (missing id/name):', resource);
                return;
            }

            const isNote = resourceId.startsWith('Note_');
            const content = resource.content || '';
            originalContent = content;
            const metadata = resource.properties || {};
            const friendlyName = metadata.note_name || metadata.collection_name || '';

            let metadataHtml = '';
            if (Object.keys(metadata).length > 0) {
                metadataHtml = '<div class="metadata">';
                metadataHtml += '<div class="metadata-item"><strong>Properties:</strong></div>';
                for (const [key, value] of Object.entries(metadata)) {
                    metadataHtml += `<div class="metadata-item"><div class="metadata-key">${escapeHtml(key)}:</div><pre class="metadata-value">${escapeHtml(formatMetadataValue(value))}</pre></div>`;
                }
                metadataHtml += '</div>';
            }

            const headerButtons = isNote
                ? `<button class="copy-btn" onclick="copyContent()">📋 Copy</button>`
                : `<button class="copy-btn" onclick="copyContent()">📋 Copy</button>`;

            const contentHtml = isNote
                ? `<textarea class="edit-textarea" id="content-text">${escapeHtml(content)}</textarea>
                   <div class="edit-actions">
                       <button class="save-btn" onclick="saveContent('${resourceId}')">Save</button>
                       <button class="cancel-btn" onclick="cancelEdit()">Cancel</button>
                   </div>`
                : `<div class="content-body" id="content-text">${escapeHtml(content)}</div>`;

            display.innerHTML = `
                <div class="content-header">
                    <div class="content-title">${escapeHtml(resourceId)}${friendlyName ? `<span style="color:#9cdcfe;font-size:13px;font-weight:normal;margin-left:8px;">${escapeHtml(friendlyName)}</span>` : ''}</div>
                    ${headerButtons}
                </div>
                ${metadataHtml}
                ${contentHtml}
            `;
        }

        async function saveContent(resourceId) {
            const textarea = document.getElementById('content-text');
            const newContent = textarea.value;
            try {
                const response = await fetch(apiUrl(`/api/resource/${resourceId}`), {
                    method: 'PUT',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({content: newContent})
                });
                const data = await response.json();
                if (data.success) {
                    originalContent = newContent;
                    // Brief visual feedback
                    const btn = document.querySelector('.save-btn');
                    btn.textContent = 'Saved';
                    btn.style.background = '#2d7d2a';
                    setTimeout(() => { btn.textContent = 'Save'; btn.style.background = ''; }, 1500);
                } else {
                    alert('Save failed: ' + (data.error || 'Unknown error'));
                }
            } catch (e) {
                alert('Save failed: ' + e.message);
            }
        }

        function cancelEdit() {
            const textarea = document.getElementById('content-text');
            if (textarea) {
                textarea.value = originalContent;
            }
        }
        
        function copyContent() {
            const el = document.getElementById('content-text');
            const text = el.tagName === 'TEXTAREA' ? el.value : el.textContent;
            navigator.clipboard.writeText(text).then(() => {
                alert('Content copied to clipboard');
            });
        }
        
        function escapeHtml(text) {
            const div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }

        function formatMetadataValue(value) {
            if (value === null || value === undefined) return String(value);
            if (typeof value === 'object') {
                try {
                    return JSON.stringify(value, null, 2);
                } catch (e) {
                    return String(value);
                }
            }
            if (typeof value === 'string') {
                const trimmed = value.trim();
                if ((trimmed.startsWith('{') && trimmed.endsWith('}')) || (trimmed.startsWith('[') && trimmed.endsWith(']'))) {
                    try {
                        const parsed = JSON.parse(trimmed);
                        return JSON.stringify(parsed, null, 2);
                    } catch (e) {
                        // Fall back to raw string when not valid JSON
                    }
                }
                return value;
            }
            return String(value);
        }
        
        // ── Tab switching ──────────────────────────────────────────
        function switchTab(tab) {
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            event.target.classList.add('active');
            document.getElementById('resources-tab').classList.toggle('active', tab === 'resources');
            document.getElementById('graph-sidebar').classList.toggle('active', tab === 'graph');
            document.getElementById('concerns-tab').classList.toggle('active', tab === 'concerns');
            document.getElementById('resource-content').style.display = tab === 'resources' ? '' : 'none';
            const gp = document.getElementById('graph-panel');
            if (tab === 'graph') {
                gp.classList.add('active');
                if (!graphInitialized) initGraph();
                loadEntityOverview();
            } else {
                gp.classList.remove('active');
            }
            if (tab === 'concerns') {
                loadConcerns();
            }
        }

        // ── Graph explorer ───────────────────────────────────────────
        let graphInitialized = false;
        let graphSim = null;
        let graphSvg, graphG, graphZoom;
        let graphNodes = [], graphEdges = [];
        const NODE_COLORS = {
            entity: '#4ec9b0', goal_launch: '#569cd6', goal_outcome: '#569cd6',
            conversation_turn: '#888', concern_created: '#ce9178', concern_change: '#ce9178',
            assessment: '#dcdcaa', decision: '#dcdcaa', tom_update: '#c586c0',
            task_created: '#6a9955', consolidation: '#555', event: '#666',
        };
        const NODE_RADIUS = {
            entity: d => 6 + Math.min((d.attrs?.mention_count || 1) * 2, 18),
            goal_launch: () => 10, goal_outcome: () => 10,
            conversation_turn: () => 4, concern_created: () => 8, concern_change: () => 8,
            assessment: () => 5, decision: () => 5, tom_update: () => 7,
            task_created: () => 8, consolidation: () => 4, event: () => 4,
        };

        function initGraph() {
            graphInitialized = true;
            const svg = d3.select('#graph-svg');
            const width = svg.node().parentElement.clientWidth;
            const height = svg.node().parentElement.clientHeight - 40;
            svg.attr('width', width).attr('height', height);

            graphZoom = d3.zoom().scaleExtent([0.1, 5]).on('zoom', e => graphG.attr('transform', e.transform));
            svg.call(graphZoom);

            // Arrowhead marker for directed edges
            const defs = svg.append('defs');
            defs.append('marker')
                .attr('id', 'arrowhead')
                .attr('viewBox', '0 -4 8 8')
                .attr('refX', 16).attr('refY', 0)
                .attr('markerWidth', 6).attr('markerHeight', 6)
                .attr('orient', 'auto')
                .append('path').attr('d', 'M0,-4L8,0L0,4').attr('fill', '#888');

            graphG = svg.append('g');
            graphSvg = svg;

            graphSim = d3.forceSimulation()
                .force('charge', d3.forceManyBody().strength(-80))
                .force('link', d3.forceLink().id(d => d.node_id).distance(80))
                .force('center', d3.forceCenter(width / 2, height / 2))
                .force('collision', d3.forceCollide().radius(15))
                .on('tick', graphTick);
            graphSim.stop();
        }

        function graphTick() {
            graphG.selectAll('.graph-edge')
                .attr('x1', d => d.source.x).attr('y1', d => d.source.y)
                .attr('x2', d => d.target.x).attr('y2', d => d.target.y);
            graphG.selectAll('.graph-edge-label')
                .attr('x', d => (d.source.x + d.target.x) / 2)
                .attr('y', d => (d.source.y + d.target.y) / 2)
                .attr('transform', d => {
                    const mx = (d.source.x + d.target.x) / 2;
                    const my = (d.source.y + d.target.y) / 2;
                    let angle = Math.atan2(d.target.y - d.source.y, d.target.x - d.source.x) * 180 / Math.PI;
                    if (angle > 90 || angle < -90) angle += 180;
                    return `rotate(${angle},${mx},${my})`;
                });
            graphG.selectAll('.graph-node')
                .attr('transform', d => `translate(${d.x},${d.y})`)
                .style('display', d => getHiddenTypes().has(d.type) ? 'none' : '');
        }

        function renderGraph(nodes, edges) {
            // Merge with existing (keep positions for nodes already rendered)
            const existingMap = {};
            graphNodes.forEach(n => { existingMap[n.node_id] = n; });
            nodes.forEach(n => {
                if (existingMap[n.node_id]) {
                    Object.assign(existingMap[n.node_id], n);
                } else {
                    graphNodes.push(n);
                }
            });
            // Deduplicate edges
            const edgeKey = e => `${e.source?.node_id||e.source}-${e.target?.node_id||e.target}-${e.type}`;
            const existingEdges = new Set(graphEdges.map(edgeKey));
            edges.forEach(e => {
                const k = edgeKey(e);
                if (!existingEdges.has(k)) { graphEdges.push(e); existingEdges.add(k); }
            });

            // Render edges
            const edgeSel = graphG.selectAll('.graph-edge').data(graphEdges, edgeKey);
            edgeSel.exit().remove();
            edgeSel.enter().append('line').attr('class', 'graph-edge');

            // Render edge labels
            const labelSel = graphG.selectAll('.graph-edge-label').data(graphEdges, edgeKey);
            labelSel.exit().remove();
            labelSel.enter().append('text').attr('class', 'graph-edge-label')
                .text(d => d.type || '');

            // Render nodes
            const nodeSel = graphG.selectAll('.graph-node').data(graphNodes, d => d.node_id);
            nodeSel.exit().remove();
            const nodeEnter = nodeSel.enter().append('g').attr('class', 'graph-node')
                .call(d3.drag()
                    .on('start', (e, d) => { if (!e.active) graphSim.alphaTarget(0.3).restart(); d.fx = d.x; d.fy = d.y; })
                    .on('drag', (e, d) => { d.fx = e.x; d.fy = e.y; })
                    .on('end', (e, d) => { if (!e.active) graphSim.alphaTarget(0); d.fx = null; d.fy = null; })
                )
                .on('click', (e, d) => expandFromNode(d))
                .on('dblclick', (e, d) => {
                    if (d.type === 'entity') return;
                    // Try to navigate to resource
                    const rid = d.attrs?.resource_id || d.attrs?.primary_product || '';
                    if (rid && rid.startsWith('Note_')) { switchTab('resources'); selectResource(rid); }
                })
                .on('mouseover', (e, d) => showTooltip(e, d))
                .on('mouseout', () => { document.getElementById('graph-tooltip').style.display = 'none'; });

            nodeEnter.append('circle')
                .attr('r', d => (NODE_RADIUS[d.type] || (() => 5))(d))
                .attr('fill', d => NODE_COLORS[d.type] || '#666');
            // Label for entity nodes
            nodeEnter.filter(d => d.type === 'entity')
                .append('text')
                .attr('dy', d => -(NODE_RADIUS.entity(d) + 3))
                .attr('text-anchor', 'middle')
                .attr('fill', '#aaa').attr('font-size', '9px')
                .text(d => (d.content || d.attrs?.display || d.node_id).slice(0, 20));

            // Update simulation
            graphSim.nodes(graphNodes);
            graphSim.force('link').links(graphEdges);
            graphSim.alpha(0.5).restart();
        }

        function showTooltip(event, d) {
            const tip = document.getElementById('graph-tooltip');
            const label = d.content || d.attrs?.display || d.node_id;
            let html = `<strong>${d.type}</strong>: ${escapeHtml(label.slice(0, 120))}`;
            if (d.attrs?.goal_id) html += `<br>Goal: ${d.attrs.goal_id}`;
            if (d.attrs?.concern_id) html += `<br>Concern: ${d.attrs.concern_id}`;
            if (d.attrs?.entity) html += `<br>Entity: ${d.attrs.entity}`;
            tip.innerHTML = html;
            tip.style.display = 'block';
            tip.style.left = (event.pageX + 12) + 'px';
            tip.style.top = (event.pageY - 10) + 'px';
        }

        async function expandFromNode(d) {
            try {
                const resp = await fetch(apiUrl('/api/graph/subgraph'), {
                    method: 'POST', headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({seed_ids: [d.node_id], max_hops: 1})
                });
                const data = await resp.json();
                if (data.success) renderGraph(data.nodes, data.edges);
            } catch (e) { console.error('Expand failed:', e); }
        }

        async function loadGraphByTypes() {
            // Collect all checked seed types
            const seedTypes = [];
            document.querySelectorAll('.seed-type:checked').forEach(cb => {
                cb.value.split(',').forEach(t => seedTypes.push(t.trim()));
            });
            if (seedTypes.length === 0) return;

            // Also load entity sidebar if entities are among the checked types
            if (seedTypes.includes('entity')) {
                try {
                    const resp = await fetch(apiUrl('/api/graph/entities'));
                    const data = await resp.json();
                    if (data.success) {
                        const linked = data.entities.filter(e => e.graph_node_id);
                        document.getElementById('entity-count').textContent = linked.length;
                        const list = document.getElementById('entity-list');
                        list.innerHTML = linked
                            .sort((a, b) => b.mention_count - a.mention_count)
                            .map(e => `<div class="resource-item" onclick="expandEntity('${e.graph_node_id}','${escapeHtml(e.name)}')" style="color:#4ec9b0;">${escapeHtml(e.name)} <span style="color:#888;font-size:10px;">(${e.mention_count})</span></div>`)
                            .join('');
                    }
                } catch (e) { console.error('Entity sidebar load failed:', e); }
            }

            // Fetch subgraph seeded by selected node types
            graphNodes = []; graphEdges = [];
            graphG.selectAll('*').remove();
            try {
                const sgResp = await fetch(apiUrl('/api/graph/subgraph'), {
                    method: 'POST', headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({seed_types: seedTypes, max_hops: 1})
                });
                const sgData = await sgResp.json();
                if (sgData.success) renderGraph(sgData.nodes, sgData.edges);
            } catch (e) { console.error('Type-seeded subgraph load failed:', e); }
        }

        // Backward compat alias
        async function loadEntityOverview() { return loadGraphByTypes(); }

        async function expandEntity(graphNodeId, name) {
            if (!graphNodeId) { alert('Entity has no graph node yet'); return; }
            try {
                graphNodes = []; graphEdges = [];
                graphG.selectAll('*').remove();
                const resp = await fetch(apiUrl('/api/graph/subgraph'), {
                    method: 'POST', headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({seed_ids: [graphNodeId], max_hops: 1})
                });
                const data = await resp.json();
                if (data.success) renderGraph(data.nodes, data.edges);
            } catch (e) { console.error('Expand entity failed:', e); }
        }

        async function graphSearch() {
            const q = document.getElementById('graph-search').value.trim();
            if (!q) return;
            try {
                const resp = await fetch(apiUrl('/api/graph/subgraph'), {
                    method: 'POST', headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({query: q, k: 10})
                });
                const data = await resp.json();
                if (data.success) {
                    graphNodes = []; graphEdges = [];
                    graphG.selectAll('*').remove();
                    renderGraph(data.nodes, data.edges);
                }
            } catch (e) { console.error('Search failed:', e); }
        }

        function getHiddenTypes() {
            // Build set of all unchecked types for display filtering
            const hidden = new Set();
            document.querySelectorAll('.seed-type').forEach(cb => {
                if (!cb.checked) {
                    cb.value.split(',').forEach(t => hidden.add(t.trim()));
                }
            });
            return hidden;
        }

        function applyTypeFilter() {
            const hidden = getHiddenTypes();
            graphG.selectAll('.graph-node').style('display', d => hidden.has(d.type) ? 'none' : '');
            // Also hide edges where both endpoints are hidden
            graphG.selectAll('.graph-edge').style('display', function(d) {
                const srcHidden = hidden.has((graphNodes.find(n => n.node_id === d.source?.node_id || n.node_id === d.source) || {}).type);
                const tgtHidden = hidden.has((graphNodes.find(n => n.node_id === d.target?.node_id || n.node_id === d.target) || {}).type);
                return (srcHidden && tgtHidden) ? 'none' : '';
            });
        }

        // ── Concerns tab ──────────────────────────────────────────
        async function loadConcerns() {
            try {
                const resp = await fetch(apiUrl('/api/concerns'));
                const data = await resp.json();
                if (!data.success) return;

                const activations = data.activations || {};

                // User concerns
                const ucList = document.getElementById('user-concerns-list');
                const uc = data.user_concerns || [];
                document.getElementById('user-concerns-count').textContent = uc.length;
                ucList.innerHTML = uc.map(c => {
                    const label = c.concern_label || c.name || '?';
                    const status = c.status || 'open';
                    const desc = c.concern_description || c.description || '';
                    const weight = c.weight || 0;
                    const statusColor = status === 'open' ? '#4ec9b0' : status === 'closed' ? '#888' : '#ce9178';
                    return `<div class="resource-item" onclick="showConcernDetail(${JSON.stringify(c).replace(/"/g, '&quot;')}, 'user')" style="padding:4px 6px;margin:1px 0;cursor:pointer">
                        <div style="display:flex;justify-content:space-between;align-items:center">
                            <span style="color:#d4d4d4;font-size:11px">${label}</span>
                            <span style="color:${statusColor};font-size:9px;text-transform:uppercase">${status}</span>
                        </div>
                        <div style="color:#888;font-size:10px;margin-top:2px">${desc.substring(0,80)}${desc.length > 80 ? '...' : ''}</div>
                        <div style="background:#333;height:3px;margin-top:3px;border-radius:1px">
                            <div style="background:#569cd6;height:3px;width:${Math.min(weight * 100, 100)}%;border-radius:1px"></div>
                        </div>
                    </div>`;
                }).join('');

                // Derived concerns
                const dcList = document.getElementById('derived-concerns-list');
                const dc = data.derived_concerns || [];
                document.getElementById('derived-concerns-count').textContent = dc.length;
                dcList.innerHTML = dc.map(c => {
                    const cid = c.concern_id || '?';
                    const label = c.concern_label || cid;
                    const status = c.status || '?';
                    const desc = c.concern_description || '';
                    // Activation: prefer the new field directly on the concern
                    // record (agent_concerns carry .activation), fall back to
                    // the legacy executive activations map, then to weight.
                    const isSeed = c.seed === true;
                    const directAct = (typeof c.activation === 'number') ? c.activation : null;
                    const act = activations[cid] || {};
                    const activation = directAct !== null
                        ? directAct
                        : (typeof act.activation === 'number' ? act.activation
                           : (typeof c.weight === 'number' ? c.weight : 0));
                    const trend = act.trend || 'stable';
                    const trendIcon = trend === 'rising' ? '↑' : trend === 'falling' ? '↓' : '→';
                    const statusColor = status === 'active' ? '#4ec9b0' : status === 'satisfied' ? '#569cd6' : status === 'abandoned' ? '#888' : '#ce9178';
                    const barColor = activation > 0.7 ? '#ce9178' : activation > 0.4 ? '#dcdcaa' : '#4ec9b0';
                    const seedBadge = isSeed
                        ? `<span style="color:#dcdcaa;font-size:9px;margin-left:6px;border:1px solid #5a4a30;padding:0 3px;border-radius:2px">SEED</span>`
                        : '';
                    return `<div class="resource-item" onclick="showConcernDetail(${JSON.stringify(c).replace(/"/g, '&quot;')}, 'derived')" style="padding:4px 6px;margin:1px 0;cursor:pointer">
                        <div style="display:flex;justify-content:space-between;align-items:center">
                            <span style="color:#d4d4d4;font-size:11px">${label}${seedBadge}</span>
                            <span style="color:${statusColor};font-size:9px">${status} ${trendIcon}</span>
                        </div>
                        <div style="color:#888;font-size:10px;margin-top:2px">${desc.substring(0,80)}${desc.length > 80 ? '...' : ''}</div>
                        <div style="display:flex;align-items:center;gap:4px;margin-top:3px">
                            <div style="flex:1;background:#333;height:3px;border-radius:1px">
                                <div style="background:${barColor};height:3px;width:${Math.min(activation * 100, 100)}%;border-radius:1px"></div>
                            </div>
                            <span style="color:#888;font-size:9px">${activation.toFixed(2)}</span>
                        </div>
                    </div>`;
                }).join('');
            } catch (e) {
                console.error('Failed to load concerns:', e);
            }
        }

        function showConcernDetail(concern, type) {
            const content = document.getElementById('resource-content');
            const display = document.getElementById('content-display');
            if (!content || !display) return;
            content.style.display = '';

            const isUser = type === 'user';
            const cid = isUser ? (concern.concern_id || concern.id || '?') : (concern.concern_id || '?');
            const label = concern.concern_label || concern.name || '?';
            const desc = concern.concern_description || concern.description || '';
            const status = concern.status || (isUser ? 'open' : '?');
            const weight = concern.weight;
            const weightFmt = (typeof weight === 'number') ? weight.toFixed(3) : weight;
            const category = concern.category || '';
            const origin = concern.origin || '';
            const rationale = concern.status_rationale || '';
            const history = concern.history_summary || '';
            const recency = concern.recency || '';
            const created = concern.created || concern.created_at || '';
            // Phase B firing fields (chat-mode concerns)
            const cadenceHours = concern.cadence_hours;
            const cadenceAllowed = Array.isArray(concern.cadence_hours_allowed)
                                   ? concern.cadence_hours_allowed
                                   : [1, 2, 4, 8, 12, 24, 168];
            const lifetime = concern.lifetime_days;
            const instruction = concern.instruction || '';
            const lastActed = concern.last_acted_at || '';
            const provenance = concern.provenance || '';
            const seed = concern.seed === true;
            const fmtHours = (h) => (h === null || h === undefined) ? '∞'
                                  : (h < 24 ? `${h}h`
                                           : (h % 24 === 0 ? `${h/24}d` : `${h}h`));
            const fmtDays = (d) => (d === null || d === undefined) ? '∞'
                                 : (d < 1 ? `${(d*24).toFixed(1)}h`
                                          : (d % 1 === 0 ? `${d}d` : `${d.toFixed(1)}d`));

            // Build action buttons based on type
            let actions = '';
            if (isUser) {
                if (status === 'open') {
                    actions = `
                        <button onclick="manageConcern('${cid}', 'close', '${type}')" style="background:#5a4a30;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer;margin-right:6px">Close</button>
                        <button onclick="manageConcern('${cid}', 'delete', '${type}')" style="background:#5a2d2d;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer">Delete</button>
                    `;
                } else {
                    actions = `
                        <button onclick="manageConcern('${cid}', 'reopen', '${type}')" style="background:#2d5a4a;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer;margin-right:6px">Reopen</button>
                        <button onclick="manageConcern('${cid}', 'delete', '${type}')" style="background:#5a2d2d;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer">Delete</button>
                    `;
                }
            } else {
                // Derived concern
                if (status === 'active' || status === 'surfaced') {
                    actions = `
                        <button onclick="manageConcern('${cid}', 'satisfy', '${type}')" style="background:#2d5a4a;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer;margin-right:6px">Satisfy</button>
                        <button onclick="manageConcern('${cid}', 'abandon', '${type}')" style="background:#5a4a30;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer;margin-right:6px">Abandon</button>
                        <button onclick="manageConcern('${cid}', 'delete', '${type}')" style="background:#5a2d2d;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer">Delete</button>
                    `;
                } else {
                    actions = `
                        <button onclick="manageConcern('${cid}', 'activate', '${type}')" style="background:#2d5a4a;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer;margin-right:6px">Activate</button>
                        <button onclick="manageConcern('${cid}', 'delete', '${type}')" style="background:#5a2d2d;color:#d4d4d4;border:none;padding:5px 12px;cursor:pointer">Delete</button>
                    `;
                }
            }

            const statusColor = (status === 'open' || status === 'active') ? '#4ec9b0'
                              : (status === 'closed' || status === 'satisfied') ? '#569cd6'
                              : (status === 'abandoned') ? '#888' : '#ce9178';

            display.classList.remove('empty-state');
            display.innerHTML = `
                <div style="padding:15px;color:#d4d4d4">
                    <div style="border-bottom:1px solid #333;padding-bottom:10px;margin-bottom:12px">
                        <div style="color:#888;font-size:10px;text-transform:uppercase;letter-spacing:0.5px;margin-bottom:3px">
                            ${isUser ? 'User Concern' : 'Derived Concern'} · ${cid}
                        </div>
                        <h2 style="margin:0;font-size:16px;color:#d4d4d4">${label}</h2>
                        <div style="margin-top:6px">
                            <span style="color:${statusColor};font-size:11px;text-transform:uppercase;letter-spacing:0.5px">${status}</span>
                            ${weight !== undefined ? `<span style="color:#888;font-size:11px;margin-left:10px">weight: ${weightFmt}</span>` : ''}
                            ${category ? `<span style="color:#888;font-size:11px;margin-left:10px">category: ${category}</span>` : ''}
                            ${origin ? `<span style="color:#888;font-size:11px;margin-left:10px">origin: ${origin}</span>` : ''}
                            ${seed ? `<span style="color:#dcdcaa;font-size:11px;margin-left:10px">seed</span>` : ''}
                        </div>
                        ${(cadenceHours !== undefined) ? `
                            <div style="margin-top:4px;display:flex;align-items:center;gap:10px;flex-wrap:wrap">
                                <span style="color:#888;font-size:11px">rhythm:</span>
                                <select onchange="setCadenceHours('${cid}', this.value, '${type}')" style="background:#2d2d2d;color:#d4d4d4;border:1px solid #3e3e42;font-size:11px;padding:1px 4px">
                                    <option value="" ${cadenceHours === null || cadenceHours === undefined ? 'selected' : ''}>(none — won't fire)</option>
                                    ${cadenceAllowed.map(h =>
                                        `<option value="${h}" ${cadenceHours === h ? 'selected' : ''}>${fmtHours(h)}</option>`
                                    ).join('')}
                                </select>
                                ${concern.rhythm_source ? `<span style="color:#888;font-size:11px">source: ${concern.rhythm_source}</span>` : ''}
                                ${typeof concern.activation === 'number' ? `<span style="color:#888;font-size:11px">activation: ${concern.activation.toFixed(3)}</span>` : ''}
                                ${typeof concern.strength === 'number' ? `<span style="color:#888;font-size:11px">strength: ${concern.strength.toFixed(3)}</span>` : ''}
                                ${provenance ? `<span style="color:#888;font-size:11px">provenance: ${provenance}</span>` : ''}
                            </div>
                        ` : ''}
                    </div>

                    <div style="margin-bottom:12px">
                        <div style="color:#888;font-size:10px;text-transform:uppercase;margin-bottom:4px">Description</div>
                        <div style="color:#d4d4d4;font-size:12px;line-height:1.5">${desc || '(none)'}</div>
                    </div>

                    ${(cadenceHours !== undefined) ? `
                        <div style="margin-bottom:12px">
                            <div style="color:#888;font-size:10px;text-transform:uppercase;margin-bottom:4px">Instruction (procedure run when this concern fires; multi-line OK)</div>
                            <textarea id="instr-${cid}" rows="6" style="width:100%;background:#1e1e1e;color:#d4d4d4;border:1px solid #3e3e42;font-family:ui-monospace,monospace;font-size:12px;line-height:1.5;padding:6px;box-sizing:border-box;resize:vertical">${htmlEscape(instruction)}</textarea>
                            <div style="margin-top:4px;display:flex;align-items:center;gap:8px">
                                <button onclick="setInstruction('${cid}', '${type}')" style="background:#2d2d2d;color:#d4d4d4;border:1px solid #3e3e42;font-size:11px;padding:2px 10px;cursor:pointer">Save</button>
                                ${instruction ? '' : '<span style="color:#666;font-size:11px">(empty — concern will not fire)</span>'}
                            </div>
                        </div>
                    ` : ''}

                    ${rationale ? `
                        <div style="margin-bottom:12px">
                            <div style="color:#888;font-size:10px;text-transform:uppercase;margin-bottom:4px">Status rationale</div>
                            <div style="color:#d4d4d4;font-size:12px;line-height:1.5">${rationale}</div>
                        </div>
                    ` : ''}

                    ${history ? `
                        <div style="margin-bottom:12px">
                            <div style="color:#888;font-size:10px;text-transform:uppercase;margin-bottom:4px">History</div>
                            <div style="color:#d4d4d4;font-size:12px;line-height:1.5;white-space:pre-wrap">${history}</div>
                        </div>
                    ` : ''}

                    ${created || recency || lastActed ? `
                        <div style="color:#888;font-size:10px;margin-bottom:12px">
                            ${created ? `created: ${created}` : ''}
                            ${recency ? ` · last engaged: ${recency}` : ''}
                            ${lastActed ? ` · last acted: ${lastActed}` : ''}
                        </div>
                    ` : ''}

                    <div style="border-top:1px solid #333;padding-top:10px;margin-top:15px">
                        ${actions}
                    </div>

                    <details style="margin-top:15px">
                        <summary style="color:#888;font-size:10px;text-transform:uppercase;cursor:pointer">Raw JSON</summary>
                        <pre style="color:#888;font-size:10px;white-space:pre-wrap;margin-top:6px">${JSON.stringify(concern, null, 2)}</pre>
                    </details>
                </div>
            `;
        }

        async function manageConcern(concernId, action, type) {
            // Confirm destructive actions
            if (action === 'delete' && !confirm(`Delete concern ${concernId}? This cannot be undone.`)) {
                return;
            }
            const character = window.characterName || '';
            if (!character) {
                alert('No character context available for concern management');
                return;
            }
            try {
                const resp = await fetch(`/api/concern/${character}/manage`, {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({concern_id: concernId, action: action, type: type}),
                });
                const data = await resp.json();
                if (data.success) {
                    // Refresh concerns list and clear detail pane
                    loadConcerns();
                    document.getElementById('content-display').innerHTML = '<div class="empty-state"><p>← Select a concern to view its content</p></div>';
                    document.getElementById('content-display').classList.add('empty-state');
                } else {
                    alert('Action failed: ' + (data.error || 'unknown error'));
                }
            } catch (e) {
                alert('Action failed: ' + e.message);
            }
        }

        // Minimal HTML escaping so multi-line instructions with `<`, `>`,
        // or `</textarea>` content don't break the editor element. Not a
        // full XSS shield — concerns are authored by the user or by Jill,
        // not external input — just element-boundary correctness.
        function htmlEscape(s) {
            return String(s == null ? '' : s)
                .replace(/&/g, '&amp;')
                .replace(/</g, '&lt;')
                .replace(/>/g, '&gt;');
        }

        async function setInstruction(concernId, type) {
            // Read the textarea, post raw text. Backend writes verbatim
            // to concern.properties.instruction (empty string → null so
            // the concern stops firing rather than firing with no body).
            const ta = document.getElementById(`instr-${concernId}`);
            if (!ta) return;
            const value = ta.value;
            const character = window.characterName || '';
            if (!character) {
                alert('No character context');
                return;
            }
            try {
                const resp = await fetch(`/api/concern/${character}/manage`, {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({concern_id: concernId, action: 'set_instruction',
                                          type: type, value: value}),
                });
                const data = await resp.json();
                if (!data.success) {
                    alert('set instruction failed: ' + (data.error || 'unknown error'));
                    return;
                }
                loadConcerns();
            } catch (e) {
                alert('set instruction failed: ' + e.message);
            }
        }

        async function setCadenceHours(concernId, rawValue, type) {
            // Combo-box → backend snap-and-write. Empty string → null (no fire).
            const character = window.characterName || '';
            if (!character) {
                alert('No character context available');
                return;
            }
            const value = (rawValue === '' || rawValue === null) ? null : Number(rawValue);
            try {
                const resp = await fetch(`/api/concern/${character}/manage`, {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({concern_id: concernId, action: 'set_cadence_hours',
                                          type: type, value: value}),
                });
                const data = await resp.json();
                if (!data.success) {
                    alert('set cadence failed: ' + (data.error || 'unknown error'));
                }
                // Refresh list so the row reflects the snapped value.
                loadConcerns();
            } catch (e) {
                alert('set cadence failed: ' + e.message);
            }
        }

        // Character comes from the ?character= query param, falling back to
        // whatever the process was launched with. Set before any fetch so
        // every request carries it — and so the edit paths below always
        // have a concrete name (they alert and bail without one).
        window.characterName =
            new URLSearchParams(window.location.search).get('character') || '';

        // Append the active character to an API path.
        function apiUrl(path) {
            const c = window.characterName || '';
            if (!c) return path;
            return path + (path.includes('?') ? '&' : '?')
                        + 'character=' + encodeURIComponent(c);
        }

        // Load context (character name) on startup
        fetch(apiUrl('/api/context')).then(r => r.json()).then(data => {
            window.characterName = window.characterName || data.character || '';
            if (data.character) {
                document.getElementById('map-name').textContent = `${data.map} · ${data.character}`;
            }
        }).catch(() => {});

        // Auto-load on page load
        refreshResources();
    </script>
</body>
</html>
"""
    
    def run(self):
        """Start the browser server."""
        logger.info(f"Starting Resource Browser on http://localhost:{self.port}")
        
        if self.open_browser:
            webbrowser.open(f'http://localhost:{self.port}')
        
        try:
            uvicorn.run(self.app, host="0.0.0.0", port=self.port, log_level="warning")
        except KeyboardInterrupt:
            logger.info('Resource Browser shutting down...')
        finally:
            self.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Browse Notes and Collections from running map_node")
    parser.add_argument('--map', type=str, default='infolab', help='Map name (default: infolab)')
    parser.add_argument('--port', type=int, default=3001, help='Web server port (default: 3001)')
    parser.add_argument('--character', type=str, default='', help='Character name to browse (avoids multi-agent ambiguity)')
    parser.add_argument('--no-browser', action='store_true', help='Do not open browser automatically')

    args = parser.parse_args()

    browser = ResourceBrowser(map_name=args.map, port=args.port, open_browser=not args.no_browser, character=args.character)
    browser.run()


if __name__ == "__main__":
    main()

