#!/usr/bin/env python3
"""
Standalone Resource Browser - Browse Notes and Collections in running map_node.

Connects to map_node via Zenoh and provides web UI for viewing raw resource content.
Read-only, manual refresh.
"""

import argparse
import json
import logging
import sys
import webbrowser
from pathlib import Path
from typing import Dict, List, Optional

import zenoh
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
import uvicorn

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ResourceBrowser:
    """Browse Notes and Collections from map_node."""
    
    def __init__(self, map_name: str = "infolab", port: int = 3001, open_browser: bool = True):
        self.map_name = map_name
        self.port = port
        self.open_browser = open_browser
        self.app = FastAPI(title="Resource Browser")
        
        # Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        logger.info(f"Connected to Zenoh for map: {map_name}")
        
        # Setup routes
        self.setup_routes()
    
    def setup_routes(self):
        """Setup FastAPI routes."""
        
        @self.app.get("/", response_class=HTMLResponse)
        async def index():
            return self.get_html()
        
        @self.app.get("/api/resources")
        async def list_resources():
            """Get list of all resources from map_node."""
            return self.query_resources()
        
        @self.app.get("/api/resource/{resource_id}")
        async def get_resource(resource_id: str):
            """Get specific resource content."""
            return self.query_resource(resource_id)
    
    def query_resources(self) -> Dict:
        """Query map_node for resource list."""
        key = "cognitive/map/resources"
        logger.info(f"Querying: {key}")
        
        for reply in self.session.get(key, timeout=2.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_string())
                if data.get('success'):
                    resources = data.get('resources', [])
                    
                    # Separate Notes and Collections
                    notes = [r for r in resources if r.get('id', '').startswith('Note_')]
                    collections = [r for r in resources if r.get('id', '').startswith('Collection_')]
                    
                    logger.info(f"Found {len(notes)} Notes, {len(collections)} Collections")
                    return {
                        'success': True,
                        'notes': notes,
                        'collections': collections
                    }
                return data
        
        return {'success': False, 'error': 'No response from map_node'}
    
    def query_resource(self, resource_id: str) -> Dict:
        """Query map_node for specific resource."""
        key = f"cognitive/map/resource/{resource_id}"
        logger.info(f"Querying: {key}")
        
        for reply in self.session.get(key, timeout=2.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_string())
                return data
        
        return {'success': False, 'error': f'Resource {resource_id} not found'}
    
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
        }
        .metadata-item span {
            color: #dcdcaa;
        }
        .copy-btn {
            background: #3e3e42;
            font-size: 12px;
            padding: 4px 10px;
        }
        .copy-btn:hover { background: #4e4e52; }
        
        ::-webkit-scrollbar { width: 10px; }
        ::-webkit-scrollbar-track { background: #1e1e1e; }
        ::-webkit-scrollbar-thumb { background: #424242; border-radius: 5px; }
        ::-webkit-scrollbar-thumb:hover { background: #4e4e4e; }
    </style>
</head>
<body>
    <header>
        <h1>🔍 Resource Browser - <span id="map-name">infolab</span></h1>
        <button onclick="refreshResources()">↻ Refresh</button>
    </header>
    
    <div class="container">
        <div class="sidebar">
            <div class="section">
                <div class="section-title">Notes (<span id="notes-count">0</span>)</div>
                <div id="notes-list"></div>
            </div>
            
            <div class="section">
                <div class="section-title">Collections (<span id="collections-count">0</span>)</div>
                <div id="collections-list"></div>
            </div>
        </div>
        
        <div class="content-area">
            <div id="content-display" class="empty-state">
                <p>← Select a resource to view its content</p>
            </div>
        </div>
    </div>
    
    <script>
        let currentResources = {notes: [], collections: []};
        let selectedResource = null;
        
        async function refreshResources() {
            try {
                const response = await fetch('/api/resources');
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
            
            notesList.innerHTML = currentResources.notes
                .map(r => {
                    const resId = r.id || r.name;
                    return `<div class="resource-item" onclick="selectResource('${resId}')">${resId}</div>`;
                })
                .join('');
            
            collectionsList.innerHTML = currentResources.collections
                .map(r => {
                    const resId = r.id || r.name;
                    return `<div class="resource-item" onclick="selectResource('${resId}')">${resId}</div>`;
                })
                .join('');
        }
        
        async function selectResource(resourceId) {
            // Update selected styling
            document.querySelectorAll('.resource-item').forEach(el => {
                el.classList.toggle('selected', el.textContent === resourceId);
            });
            
            selectedResource = resourceId;
            
            try {
                const response = await fetch(`/api/resource/${resourceId}`);
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
        
        function displayResource(resource) {
            const display = document.getElementById('content-display');
            
            // Validate resource structure (accept either id or name field)
            const resourceId = resource.id || resource.name;
            if (!resource || !resourceId) {
                display.innerHTML = '<div class="empty-state"><p>Error: Invalid resource data</p></div>';
                console.error('Invalid resource (missing id/name):', resource);
                return;
            }
            
            const type = resourceId.startsWith('Note_') ? 'Note' : 'Collection';
            const content = resource.content || '';
            const metadata = resource.properties || {};
            
            let metadataHtml = '';
            if (Object.keys(metadata).length > 0) {
                metadataHtml = '<div class="metadata">';
                metadataHtml += '<div class="metadata-item"><strong>Properties:</strong></div>';
                for (const [key, value] of Object.entries(metadata)) {
                    metadataHtml += `<div class="metadata-item">  ${key}: <span>${JSON.stringify(value)}</span></div>`;
                }
                metadataHtml += '</div>';
            }
            
            display.innerHTML = `
                <div class="content-header">
                    <div class="content-title">${escapeHtml(resourceId)}</div>
                    <button class="copy-btn" onclick="copyContent()">📋 Copy</button>
                </div>
                ${metadataHtml}
                <div class="content-body" id="content-text">${escapeHtml(content)}</div>
            `;
        }
        
        function copyContent() {
            const text = document.getElementById('content-text').textContent;
            navigator.clipboard.writeText(text).then(() => {
                alert('Content copied to clipboard');
            });
        }
        
        function escapeHtml(text) {
            const div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }
        
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
        
        uvicorn.run(self.app, host="0.0.0.0", port=self.port, log_level="warning")


def main():
    parser = argparse.ArgumentParser(description="Browse Notes and Collections from running map_node")
    parser.add_argument('--map', type=str, default='infolab', help='Map name (default: infolab)')
    parser.add_argument('--port', type=int, default=3001, help='Web server port (default: 3001)')
    parser.add_argument('--no-browser', action='store_true', help='Do not open browser automatically')
    
    args = parser.parse_args()
    
    browser = ResourceBrowser(map_name=args.map, port=args.port, open_browser=not args.no_browser)
    browser.run()


if __name__ == "__main__":
    main()

