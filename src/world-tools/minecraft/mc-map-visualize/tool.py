"""
Minecraft map visualization tool.
Generates an HTML file with 2D visualization of the spatial map and opens it in browser.
"""

import logging
import json
import webbrowser
from pathlib import Path
from typing import Any, Dict, List, Optional
from datetime import datetime

logger = logging.getLogger(__name__)

DEFAULT_MAP_NAME = "minecraft_map"


def _get_content(resource_id: str, resource_manager) -> Any:
    """Fetch content for a resource ID."""
    if resource_id == "Note_null":
        return None
    
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    
    resource = resource_manager.get_resource(resource_id)
    if not resource:
        return None
    
    return resource.get('properties', {}).get('content')


def _round_coordinate(coord: float) -> int:
    """Round coordinate to nearest block (integer)."""
    return int(round(coord))


def _compile_map_forward(map_collection_id: str, resource_manager) -> List[Dict]:
    """
    Compile forward: Load all Notes from Collection, deduplicate by (x,y,z), keep latest.
    
    Args:
        map_collection_id: Collection ID containing map Notes
        resource_manager: Resource manager instance
        
    Returns:
        List of compiled map entries (one per unique location, latest observation)
    """
    if not resource_manager:
        return []
    
    # Load Collection
    map_resource = resource_manager.get_resource(map_collection_id)
    if not map_resource:
        return []
    
    # Get Note IDs from Collection content
    note_ids = map_resource.get('properties', {}).get('content', [])
    if not isinstance(note_ids, list):
        return []
    
    # Load all Notes and compile
    location_map = {}  # (x,y,z) -> entry with latest timestamp
    waypoint_map = {}  # (x,y,z) -> set of waypoint names
    
    for note_id in note_ids:
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            continue
        
        note_resource = resource_manager.get_resource(note_id)
        if not note_resource:
            continue
        
        entry = note_resource.get('properties', {}).get('content')
        if not isinstance(entry, dict):
            continue
        
        # Extract coordinates
        x = _round_coordinate(entry.get('x', 0))
        y = _round_coordinate(entry.get('y', 0))
        z = _round_coordinate(entry.get('z', 0))
        key = (x, y, z)
        
        # Collect waypoints
        waypoints = entry.get('waypoints', [])
        if isinstance(waypoints, list) and waypoints:
            if key not in waypoint_map:
                waypoint_map[key] = set()
            waypoint_map[key].update(waypoints)
        
        # Get timestamp
        timestamp = entry.get('timestamp', '')
        
        # Keep latest entry for each location
        if key not in location_map:
            location_map[key] = entry
        else:
            existing_timestamp = location_map[key].get('timestamp', '')
            if timestamp > existing_timestamp:
                location_map[key] = entry
    
    # Merge waypoints into compiled entries
    compiled = []
    for key, entry in location_map.items():
        compiled_entry = entry.copy()
        if key in waypoint_map:
            compiled_entry['waypoints'] = sorted(list(waypoint_map[key]))
        compiled.append(compiled_entry)
    
    return compiled


def _generate_html_visualization(map_content: List[Dict], map_name: str) -> str:
    """
    Generate HTML visualization of the map.
    
    Args:
        map_content: List of location entries from map Collection
        map_name: Name of the map Collection
        
    Returns:
        HTML string
    """
    if not map_content:
        return """
<!DOCTYPE html>
<html>
<head>
    <title>Map Visualization - Empty</title>
    <style>
        body { font-family: sans-serif; padding: 40px; text-align: center; }
        .empty { color: #666; font-size: 18px; }
    </style>
</head>
<body>
    <div class="empty">Map is empty - no locations explored yet</div>
</body>
</html>
"""
    
    # Extract coordinates and metadata
    locations = []
    observed_blocks = []  # Blocks observed but not visited
    observed_set = set()  # Track (x,y,z) to avoid duplicates
    waypoints = []
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')
    
    for entry in map_content:
        if isinstance(entry, dict):
            x = entry.get('x', 0)
            y = entry.get('y', 0)
            z = entry.get('z', 0)
            
            if isinstance(x, (int, float)) and isinstance(y, (int, float)) and isinstance(z, (int, float)):
                # Extract observation metadata for hover details
                observed = entry.get('observed', {})
                observation_meta = {}
                if isinstance(observed, dict):
                    # Extract affordances, geometry, blocks for hover display
                    observation_meta = {
                        'aff': observed.get('aff', {}),
                        'geom': observed.get('geom', {}),
                        'blocks': observed.get('blocks', {}),
                        'support': observed.get('support', {}),
                        'clear': observed.get('clear', {})
                    }
                
                # Create location dict with observation metadata
                loc_dict = {
                    'x': x,
                    'y': y,
                    'z': z,
                    'visit_count': entry.get('visit_count', 0),
                    'waypoints': entry.get('waypoints', []),
                    'first_visit': entry.get('first_visit', ''),
                    'last_visit': entry.get('last_visit', '')
                }
                if observation_meta and any(observation_meta.values()):
                    loc_dict['observation'] = observation_meta
                
                locations.append(loc_dict)
                
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
                min_z = min(min_z, z)
                max_z = max(max_z, z)
                
                # Track visited location to avoid showing as observed
                visited_key = (_round_coordinate(x), _round_coordinate(y), _round_coordinate(z))
                observed_set.add(visited_key)
                
                # Extract nearby_blocks from observation data (check both paths for backward compatibility)
                nearby = None
                if isinstance(observed, dict):
                    # New path: observed.nearby_blocks (top-level, filtered by map-update)
                    nearby = observed.get('nearby_blocks')
                    if not nearby:
                        # Old path: observed.metadata.perception.nearby_blocks (nested)
                        metadata = observed.get('metadata', {})
                        if isinstance(metadata, dict):
                            perception = metadata.get('perception', {})
                            if isinstance(perception, dict):
                                nearby = perception.get('nearby_blocks')
                
                if isinstance(nearby, list):
                    for block in nearby:
                        if isinstance(block, dict):
                            pos = block.get('position')
                            if isinstance(pos, dict):
                                bx = pos.get('x')
                                by = pos.get('y')
                                bz = pos.get('z')
                                if isinstance(bx, (int, float)) and isinstance(by, (int, float)) and isinstance(bz, (int, float)):
                                    bx_round = _round_coordinate(bx)
                                    by_round = _round_coordinate(by)
                                    bz_round = _round_coordinate(bz)
                                    block_key = (bx_round, by_round, bz_round)
                                    
                                    # Only add if not already visited
                                    if block_key not in observed_set:
                                        observed_set.add(block_key)
                                        # Extract dy for layer indication
                                        dy = block.get('dy', 0)
                                        observed_blocks.append({
                                            'x': bx_round,
                                            'y': by_round,
                                            'z': bz_round,
                                            'name': block.get('name', 'unknown'),
                                            'dy': int(dy) if isinstance(dy, (int, float)) else 0
                                        })
                                        
                                        min_x = min(min_x, bx_round)
                                        max_x = max(max_x, bx_round)
                                        min_y = min(min_y, by_round)
                                        max_y = max(max_y, by_round)
                                        min_z = min(min_z, bz_round)
                                        max_z = max(max_z, bz_round)
                
                # Collect waypoints
                for wp in entry.get('waypoints', []):
                    if wp not in [w['name'] for w in waypoints]:
                        waypoints.append({'name': wp, 'x': x, 'y': y, 'z': z})
    
    if not locations:
        return """
<!DOCTYPE html>
<html>
<head>
    <title>Map Visualization - Invalid Data</title>
    <style>
        body { font-family: sans-serif; padding: 40px; text-align: center; }
        .error { color: #d32f2f; font-size: 18px; }
    </style>
</head>
<body>
    <div class="error">Map contains no valid location data</div>
</body>
</html>
"""
    
    # Calculate bounds and scale
    range_x = max_x - min_x if max_x != min_x else 1
    range_z = max_z - min_z if max_z != min_z else 1
    
    # Generate HTML with canvas visualization
    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Map Visualization - {map_name}</title>
    <meta charset="UTF-8">
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #1e1e1e;
            color: #d4d4d4;
            padding: 20px;
        }}
        .header {{
            background: #252526;
            padding: 15px 20px;
            border-radius: 4px;
            margin-bottom: 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }}
        h1 {{
            font-size: 20px;
            font-weight: 500;
            color: #cccccc;
        }}
        .stats {{
            font-size: 14px;
            color: #888;
        }}
        .controls {{
            background: #252526;
            padding: 15px;
            border-radius: 4px;
            margin-bottom: 20px;
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }}
        button {{
            background: #0e639c;
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 3px;
            cursor: pointer;
            font-size: 13px;
        }}
        button:hover {{ background: #1177bb; }}
        button:active {{ background: #0d5585; }}
        .container {{
            display: flex;
            gap: 20px;
        }}
        .map-area {{
            flex: 1;
            background: #252526;
            border-radius: 4px;
            padding: 20px;
            position: relative;
        }}
        canvas {{
            border: 1px solid #3e3e42;
            border-radius: 4px;
            background: #1e1e1e;
            cursor: crosshair;
        }}
        .sidebar {{
            width: 300px;
            background: #252526;
            border-radius: 4px;
            padding: 15px;
            max-height: 800px;
            overflow-y: auto;
        }}
        .section {{
            margin-bottom: 20px;
        }}
        .section-title {{
            font-size: 12px;
            font-weight: 600;
            text-transform: uppercase;
            color: #888;
            margin-bottom: 10px;
            letter-spacing: 0.5px;
        }}
        .waypoint-item {{
            padding: 6px 10px;
            margin: 4px 0;
            background: #2a2d2e;
            border-radius: 3px;
            font-size: 13px;
            cursor: pointer;
        }}
        .waypoint-item:hover {{
            background: #3a3d3e;
        }}
        .location-item {{
            padding: 6px 10px;
            margin: 4px 0;
            background: #2a2d2e;
            border-radius: 3px;
            font-size: 12px;
            font-family: 'Consolas', monospace;
        }}
        .info-panel {{
            position: absolute;
            top: 10px;
            right: 10px;
            background: rgba(30, 30, 30, 0.9);
            padding: 10px;
            border-radius: 4px;
            font-size: 12px;
            display: none;
            border: 1px solid #3e3e42;
        }}
        .info-panel.visible {{
            display: block;
        }}
        ::-webkit-scrollbar {{ width: 8px; }}
        ::-webkit-scrollbar-track {{ background: #1e1e1e; }}
        ::-webkit-scrollbar-thumb {{ background: #424242; border-radius: 4px; }}
        ::-webkit-scrollbar-thumb:hover {{ background: #4e4e4e; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>🗺️ Map Visualization: {map_name}</h1>
        <div class="stats">
            {len(locations)} visited • {len(observed_blocks)} observed • {len(waypoints)} waypoints
        </div>
    </div>
    
    <div class="controls">
        <button onclick="resetView()">Reset View</button>
        <button onclick="fitToBounds()">Fit All</button>
        <button onclick="toggleWaypoints()">Toggle Waypoints</button>
        <button onclick="exportData()">Export JSON</button>
    </div>
    
    <div class="container">
        <div class="map-area">
            <canvas id="mapCanvas" width="800" height="600"></canvas>
            <div id="infoPanel" class="info-panel"></div>
        </div>
        
        <div class="sidebar">
            <div class="section">
                <div class="section-title">Waypoints ({len(waypoints)})</div>
                <div id="waypointsList">
"""
    
    # Add waypoints to sidebar
    for wp in waypoints:
        html += f"""
                    <div class="waypoint-item" onclick="focusLocation({wp['x']}, {wp['z']})">
                        📍 {wp['name']}<br>
                        <span style="color: #888; font-size: 11px;">({wp['x']}, {wp['y']}, {wp['z']})</span>
                    </div>
"""
    
    html += """
                </div>
            </div>
            
            <div class="section">
                <div class="section-title">Recent Locations</div>
                <div id="locationsList">
"""
    
    # Add recent locations (sorted by last_visit)
    sorted_locations = sorted(locations, key=lambda l: l.get('last_visit', ''), reverse=True)[:20]
    for loc in sorted_locations:
        wp_str = ', '.join(loc['waypoints']) if loc['waypoints'] else ''
        html += f"""
                    <div class="location-item" onclick="focusLocation({loc['x']}, {loc['z']})">
                        ({loc['x']}, {loc['y']}, {loc['z']})<br>
                        <span style="color: #888; font-size: 11px;">
                            Visits: {loc['visit_count']}
                            {(' • ' + wp_str) if wp_str else ''}
                        </span>
                    </div>
"""
    
    html += f"""
                </div>
            </div>
        </div>
    </div>
    
    <script>
        const canvas = document.getElementById('mapCanvas');
        const ctx = canvas.getContext('2d');
        const infoPanel = document.getElementById('infoPanel');
        
        // Map data
        const locations = {json.dumps(locations)};
        const observedBlocks = {json.dumps(observed_blocks)};
        const waypoints = {json.dumps(waypoints)};
        const bounds = {{
            minX: {min_x},
            maxX: {max_x},
            minZ: {min_z},
            maxZ: {max_z},
            rangeX: {range_x},
            rangeZ: {range_z}
        }};
        
        // View state
        let scale = 1;
        let offsetX = 0;
        let offsetZ = 0;
        let showWaypoints = true;
        let isDragging = false;
        let dragStart = {{x: 0, z: 0}};
        
        // Convert world coordinates to canvas coordinates
        function worldToCanvas(x, z) {{
            const canvasX = ((x - bounds.minX) / bounds.rangeX) * canvas.width * scale + offsetX;
            const canvasZ = ((z - bounds.minZ) / bounds.rangeZ) * canvas.height * scale + offsetZ;
            return {{x: canvasX, z: canvasZ}};
        }}
        
        // Convert canvas coordinates to world coordinates
        function canvasToWorld(canvasX, canvasZ) {{
            const worldX = ((canvasX - offsetX) / scale) * bounds.rangeX / canvas.width + bounds.minX;
            const worldZ = ((canvasZ - offsetZ) / scale) * bounds.rangeZ / canvas.height + bounds.minZ;
            return {{x: worldX, z: worldZ}};
        }}
        
        // Draw map
        function drawMap() {{
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Draw grid
            ctx.strokeStyle = '#2a2d2e';
            ctx.lineWidth = 1;
            const gridSpacing = 50 * scale;
            for (let x = -offsetX % gridSpacing; x < canvas.width; x += gridSpacing) {{
                ctx.beginPath();
                ctx.moveTo(x, 0);
                ctx.lineTo(x, canvas.height);
                ctx.stroke();
            }}
            for (let z = -offsetZ % gridSpacing; z < canvas.height; z += gridSpacing) {{
                ctx.beginPath();
                ctx.moveTo(0, z);
                ctx.lineTo(canvas.width, z);
                ctx.stroke();
            }}
            
            // Draw observed blocks (smaller, lighter)
            observedBlocks.forEach(block => {{
                const pos = worldToCanvas(block.x, block.z);
                ctx.fillStyle = 'rgba(150, 150, 150, 0.4)';
                ctx.beginPath();
                ctx.arc(pos.x, pos.z, 1.5, 0, Math.PI * 2);
                ctx.fill();
            }});
            
            // Draw visited locations (larger, colored)
            locations.forEach(loc => {{
                const pos = worldToCanvas(loc.x, loc.z);
                const radius = Math.max(3, Math.min(8, 3 + loc.visit_count * 0.5));
                
                // Color based on visit count
                const intensity = Math.min(255, 100 + loc.visit_count * 30);
                ctx.fillStyle = `rgb(${{intensity}}, 150, 100)`;
                ctx.beginPath();
                ctx.arc(pos.x, pos.z, radius, 0, Math.PI * 2);
                ctx.fill();
                
                // Border
                ctx.strokeStyle = '#4ec9b0';
                ctx.lineWidth = 1;
                ctx.stroke();
            }});
            
            // Draw waypoints
            if (showWaypoints) {{
                waypoints.forEach(wp => {{
                    const pos = worldToCanvas(wp.x, wp.z);
                    ctx.fillStyle = '#ffd700';
                    ctx.beginPath();
                    ctx.arc(pos.x, pos.z, 6, 0, Math.PI * 2);
                    ctx.fill();
                    ctx.strokeStyle = '#ffaa00';
                    ctx.lineWidth = 2;
                    ctx.stroke();
                    
                    // Label
                    ctx.fillStyle = '#ffd700';
                    ctx.font = '12px sans-serif';
                    ctx.fillText(wp.name, pos.x + 8, pos.z - 8);
                }});
            }}
        }}
        
        // Mouse events
        canvas.addEventListener('mousedown', (e) => {{
            isDragging = true;
            dragStart.x = e.offsetX;
            dragStart.z = e.offsetY;
        }});
        
        canvas.addEventListener('mousemove', (e) => {{
            if (isDragging) {{
                offsetX += e.offsetX - dragStart.x;
                offsetZ += e.offsetY - dragStart.z;
                dragStart.x = e.offsetX;
                dragStart.z = e.offsetY;
                drawMap();
            }} else {{
                // Show info panel
                const worldPos = canvasToWorld(e.offsetX, e.offsetY);
                const nearby = locations.filter(loc => 
                    Math.abs(loc.x - worldPos.x) < 1 && Math.abs(loc.z - worldPos.z) < 1
                );
                if (nearby.length > 0) {{
                    const loc = nearby[0];
                    let infoHtml = `<strong>Location</strong><br>X: ${{loc.x}}, Y: ${{loc.y}}, Z: ${{loc.z}}<br>Visits: ${{loc.visit_count}}<br>`;
                    
                    if (loc.waypoints && loc.waypoints.length > 0) {{
                        infoHtml += `Waypoints: ${{loc.waypoints.join(', ')}}<br>`;
                    }}
                    
                    // Show observation details if available
                    if (loc.observation) {{
                        const obs = loc.observation;
                        infoHtml += `<br><strong>Observation:</strong><br>`;
                        
                        // Affordances
                        if (obs.aff) {{
                            const affs = [];
                            if (obs.aff.step) affs.push('Step');
                            if (obs.aff.jump) affs.push('Jump');
                            if (obs.aff.descend) affs.push('Descend');
                            if (obs.aff.sky) affs.push('Sky');
                            if (affs.length > 0) {{
                                infoHtml += `Affordances: ${{affs.join(', ')}}<br>`;
                            }}
                        }}
                        
                        // Geometry
                        if (obs.geom) {{
                            const geoms = [];
                            if (obs.geom.pit) geoms.push('Pit');
                            if (obs.geom.stair) geoms.push('Stair');
                            if (obs.geom.slope) geoms.push('Slope');
                            if (geoms.length > 0) {{
                                infoHtml += `Geometry: ${{geoms.join(', ')}}<br>`;
                            }}
                        }}
                        
                        // Block types
                        if (obs.blocks && obs.blocks.seen && obs.blocks.seen.length > 0) {{
                            const blockTypes = obs.blocks.seen.slice(0, 5);
                            infoHtml += `Blocks: ${{blockTypes.join(', ')}}`;
                            if (obs.blocks.seen.length > 5) {{
                                infoHtml += ` (+${{obs.blocks.seen.length - 5}} more)`;
                            }}
                            infoHtml += `<br>`;
                        }}
                    }}
                    
                    infoPanel.innerHTML = infoHtml;
                    infoPanel.classList.add('visible');
                }} else {{
                    // Check if hovering over observed block
                    const nearbyBlocks = observedBlocks.filter(block => 
                        Math.abs(block.x - worldPos.x) < 0.5 && Math.abs(block.z - worldPos.z) < 0.5
                    );
                    if (nearbyBlocks.length > 0) {{
                        const block = nearbyBlocks[0];
                        let infoHtml = `<strong>Block</strong><br>X: ${{block.x}}, Y: ${{block.y}}, Z: ${{block.z}}<br>Type: ${{block.name}}<br>`;
                        if (block.dy !== undefined) {{
                            const layerNames = {{'-1': 'Support (y-1)', '0': 'Body (y)', '1': 'Head (y+1)'}};
                            infoHtml += `Layer: ${{layerNames[block.dy] || 'y+' + block.dy}}<br>`;
                        }}
                        infoPanel.innerHTML = infoHtml;
                        infoPanel.classList.add('visible');
                    }} else {{
                        infoPanel.classList.remove('visible');
                    }}
                }}
            }}
        }});
        
        canvas.addEventListener('mouseup', () => {{
            isDragging = false;
        }});
        
        canvas.addEventListener('wheel', (e) => {{
            e.preventDefault();
            const delta = e.deltaY > 0 ? 0.9 : 1.1;
            scale *= delta;
            scale = Math.max(0.1, Math.min(5, scale));
            drawMap();
        }});
        
        // Control functions
        function resetView() {{
            scale = 1;
            offsetX = 0;
            offsetZ = 0;
            drawMap();
        }}
        
        function fitToBounds() {{
            const padding = 20;
            const scaleX = (canvas.width - padding * 2) / canvas.width;
            const scaleZ = (canvas.height - padding * 2) / canvas.height;
            scale = Math.min(scaleX, scaleZ);
            offsetX = padding;
            offsetZ = padding;
            drawMap();
        }}
        
        function toggleWaypoints() {{
            showWaypoints = !showWaypoints;
            drawMap();
        }}
        
        function focusLocation(x, z) {{
            const pos = worldToCanvas(x, z);
            offsetX = canvas.width / 2 - pos.x;
            offsetZ = canvas.height / 2 - pos.z;
            scale = 2;
            drawMap();
        }}
        
        function exportData() {{
            const data = {{
                map_name: '{map_name}',
                locations: locations,
                observed_blocks: observedBlocks,
                waypoints: waypoints,
                bounds: bounds,
                exported_at: new Date().toISOString()
            }};
            const blob = new Blob([JSON.stringify(data, null, 2)], {{type: 'application/json'}});
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = '{map_name}_export.json';
            a.click();
            URL.revokeObjectURL(url);
        }}
        
        // Initial draw
        fitToBounds();
    </script>
</body>
</html>
"""
    
    return html


def tool(input_value=None, **kwargs):
    """
    Generate HTML visualization of the spatial map and open in browser.
    
    Args:
        input_value: Ignored
        target: Map Collection name or variable (e.g., "minecraft_map" or "$map")
        map_name: Optional explicit map name (alternative to target)
        output_file: Optional output file path (default: auto-generated)
        resource_manager: Resource manager instance (from executor)
        agent_name: Agent name (for logging)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    resource_manager = kwargs.get('resource_manager')
    agent_name = kwargs.get('agent_name', 'system')
    target = kwargs.get('target') or kwargs.get('map_name')
    output_file = kwargs.get('output_file')
    
    if not resource_manager:
        return executor._create_uniform_return('failed', reason="Resource manager not available")
    
    # Resolve target (could be variable or literal name)
    # Default to agent-specific map name if not provided
    default_map_name = f"{agent_name}-minecraft_map"
    map_collection_id = None
    map_name = None
    
    if not target:
        # Default to agent-specific map name
        map_name = default_map_name
    elif isinstance(target, str) and target.startswith('$'):
        # Variable reference - strip $ and use as literal name
        # (executor should have resolved it, but if not, treat as literal)
        map_name = target.lstrip('$')
    else:
        map_name = str(target)
    
    # Try to get Collection by name
    map_collection_id = resource_manager.named_collections.get(map_name)
    
    if not map_collection_id:
        reason = f"Map Collection '{map_name}' not found"
        result_text = f"Failed to visualize map: Collection '{map_name}' not found. Use mc-map-update to create it first."
        return executor._create_uniform_return('failed', reason=reason, value=result_text)
    
    # Compile forward: Load all Notes, deduplicate by (x,y,z), keep latest
    map_content = _compile_map_forward(map_collection_id, resource_manager)
    
    # Generate HTML
    try:
        html_content = _generate_html_visualization(map_content, map_name)
    except Exception as e:
        logger.error(f"Failed to generate HTML visualization: {e}", exc_info=True)
        reason = f"HTML generation failed: {str(e)}"
        result_text = f"Failed to generate visualization: {str(e)}"
        return executor._create_uniform_return('failed', reason=reason, value=result_text)
    
    # Determine output file path
    if not output_file:
        # Generate filename based on map name and timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_map_name = map_name.replace('/', '_').replace('\\', '_')
        output_file = f"map_visualization_{safe_map_name}_{timestamp}.html"
    
    output_path = Path(output_file)
    
    # If relative path, save to /tmp
    if not output_path.is_absolute():
        # Ensure /tmp exists
        tmp_dir = Path("/tmp")
        tmp_dir.mkdir(parents=True, exist_ok=True)
        output_path = tmp_dir / output_path
    
    # Write HTML file
    try:
        output_path.write_text(html_content, encoding='utf-8')
        logger.info(f"Generated map visualization: {output_path}")
    except Exception as e:
        logger.error(f"Failed to write HTML file: {e}", exc_info=True)
        reason = f"File write failed: {str(e)}"
        result_text = f"Failed to save visualization file: {str(e)}"
        return executor._create_uniform_return('failed', reason=reason, value=result_text)
    
    # Open in browser
    try:
        file_url = f"file://{output_path.absolute()}"
        webbrowser.open(file_url)
        logger.info(f"Opened map visualization in browser: {file_url}")
    except Exception as e:
        logger.warning(f"Failed to open browser automatically: {e}. File saved at: {output_path}")
    
    result_text = f"Map visualization generated: {output_path.absolute()}\n{len(map_content)} locations visualized"
    
    # Build structured data dict
    structured_data = {
        "map_name": map_name,
        "map_id": map_collection_id,
        "file_path": str(output_path.absolute()),
        "output_file": str(output_path.absolute()),
        "locations_count": len(map_content),
        "waypoints_count": len([loc for loc in map_content if isinstance(loc, dict) and loc.get('waypoints')])
    }
    
    return executor._create_uniform_return('success', value=result_text, data=structured_data)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Test with mock data
    result = tool(
        target="minecraft_map",
        resource_manager=None,
        agent_name="test"
    )
    print(result)

