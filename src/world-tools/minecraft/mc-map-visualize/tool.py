"""
Minecraft map visualization tool.
Generates an HTML file with 2D visualization of the spatial map and opens it in browser.
Displays cell-based SpatialMap data with zoomable, pannable interface.
"""

import logging
import json
import webbrowser
from pathlib import Path
from typing import Any, Dict, List, Optional
from datetime import datetime

logger = logging.getLogger(__name__)

DEFAULT_MAP_NAME = "minecraft_map"

# Import SpatialMap
try:
    from ..spatial_map import SpatialMap
except ImportError:
    # Fallback for direct execution
    import sys
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from spatial_map import SpatialMap


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


# Cache for SpatialMap instances (one per agent)
_spatial_map_cache: Dict[str, SpatialMap] = {}


def _get_spatial_map(agent_name: str, world_name: str, base_dir: Optional[Path] = None) -> SpatialMap:
    """Get or create SpatialMap for agent."""
    cache_key = f"{agent_name}:{world_name}"
    if cache_key not in _spatial_map_cache:
        _spatial_map_cache[cache_key] = SpatialMap(agent_name, world_name, base_dir)
    return _spatial_map_cache[cache_key]


def _generate_html_visualization(cells: List[Dict], map_name: str, stats: Dict) -> str:
    """
    Generate HTML visualization of the spatial map.
    
    Args:
        cells: List of cell entries from SpatialMap
        map_name: Name of the map
        stats: Map statistics
        
    Returns:
        HTML string
    """
    if not cells:
        return """
<!DOCTYPE html>
<html>
<head>
    <title>Spatial Map Visualization - Empty</title>
    <style>
        body { font-family: sans-serif; padding: 40px; text-align: center; background: #1e1e1e; color: #d4d4d4; }
        .empty { color: #888; font-size: 18px; }
    </style>
</head>
<body>
    <div class="empty">Spatial map is empty - no cells mapped yet</div>
</body>
</html>
"""
    
    # Process cells into visualization data
    cell_data = []
    waypoints = []
    
    bounds = stats.get('bounds', {})
    min_x = bounds.get('min_x', 0)
    max_x = bounds.get('max_x', 0)
    min_z = bounds.get('min_z', 0)
    max_z = bounds.get('max_z', 0)
    range_x = max(1, max_x - min_x)
    range_z = max(1, max_z - min_z)
    
    for cell in cells:
        cell_id = cell.get('cell_id', {})
        x = cell_id.get('x', 0)
        z = cell_id.get('z', 0)
        
        support = cell.get('support', {})
        surface = cell.get('surface', {})
        hazards = cell.get('hazards', {})
        resources = cell.get('resources', {})
        observability = cell.get('observability', {})
        environment = cell.get('environment', {})
        
        cell_entry = {
            'x': x,
            'z': z,
            'support_y': support.get('support_y'),
            # Note: delta_y_from_agent removed - it was agent-relative data that becomes stale
            # Use absolute support_y instead
            'walkable': support.get('walkable', False),
            'movement_class': support.get('movement_class', 'unknown'),
            'surface_class': surface.get('surface_block_class', 'unknown'),
            'support_block': surface.get('support_block'),
            'is_fluid': surface.get('is_fluid', False),
            'hazard_flags': hazards.get('flags', []),
            'exposure_risk': hazards.get('exposure_risk', 'unknown'),
            'escape_difficulty': hazards.get('escape_difficulty', 'unknown'),
            'resources': resources.get('resources_visible', []),
            'confidence': observability.get('confidence', 0),
            'observation_mode': observability.get('observation_mode', 'unknown'),
            'last_observed': observability.get('last_observed_at'),
            'light_level': environment.get('light_level', 'unknown')
        }
        
        cell_data.append(cell_entry)
    
    # Generate HTML
    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Spatial Map Visualization - {map_name}</title>
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
            align-items: center;
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
        button.active {{ background: #388a34; }}
        .legend {{
            display: flex;
            gap: 15px;
            margin-left: 20px;
            font-size: 12px;
        }}
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 5px;
        }}
        .legend-color {{
            width: 16px;
            height: 16px;
            border-radius: 2px;
            border: 1px solid #444;
        }}
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
            width: 320px;
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
        .stat-item {{
            padding: 6px 10px;
            margin: 4px 0;
            background: #2a2d2e;
            border-radius: 3px;
            font-size: 13px;
            display: flex;
            justify-content: space-between;
        }}
        .stat-value {{
            color: #4ec9b0;
            font-weight: 500;
        }}
        .info-panel {{
            position: absolute;
            top: 10px;
            right: 10px;
            background: rgba(30, 30, 30, 0.95);
            padding: 12px;
            border-radius: 4px;
            font-size: 12px;
            display: none;
            border: 1px solid #3e3e42;
            min-width: 200px;
            max-width: 280px;
        }}
        .info-panel.visible {{
            display: block;
        }}
        .info-row {{
            display: flex;
            justify-content: space-between;
            padding: 3px 0;
            border-bottom: 1px solid #333;
        }}
        .info-row:last-child {{
            border-bottom: none;
        }}
        .info-label {{
            color: #888;
        }}
        .info-value {{
            color: #4ec9b0;
        }}
        .info-value.hazard {{
            color: #f14c4c;
        }}
        .info-value.safe {{
            color: #89d185;
        }}
        .info-value.warning {{
            color: #cca700;
        }}
        ::-webkit-scrollbar {{ width: 8px; }}
        ::-webkit-scrollbar-track {{ background: #1e1e1e; }}
        ::-webkit-scrollbar-thumb {{ background: #424242; border-radius: 4px; }}
        ::-webkit-scrollbar-thumb:hover {{ background: #4e4e4e; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>🗺️ Spatial Map: {map_name}</h1>
        <div class="stats">
            {stats.get('cell_count', 0)} cells • 
            {stats.get('walkable_count', 0)} walkable • 
            {stats.get('hazard_count', 0)} hazards • 
            {stats.get('resource_count', 0)} resources
        </div>
    </div>
    
    <div class="controls">
        <button onclick="resetView()">Reset View</button>
        <button onclick="fitToBounds()">Fit All</button>
        <button id="btnWalkable" class="active" onclick="toggleLayer('walkable')">Walkable</button>
        <button id="btnHazards" class="active" onclick="toggleLayer('hazards')">Hazards</button>
        <button id="btnResources" class="active" onclick="toggleLayer('resources')">Resources</button>
        <button id="btnConfidence" onclick="toggleLayer('confidence')">Confidence</button>
        <button onclick="exportData()">Export JSON</button>
        
        <div class="legend">
            <div class="legend-item">
                <div class="legend-color" style="background: #4ec9b0;"></div>
                <span>Walkable</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #666;"></div>
                <span>Blocked</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #f14c4c;"></div>
                <span>Hazard</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #ffd700;"></div>
                <span>Resource</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #3794ff;"></div>
                <span>Water</span>
            </div>
        </div>
    </div>
    
    <div class="container">
        <div class="map-area">
            <canvas id="mapCanvas" width="900" height="700"></canvas>
            <div id="infoPanel" class="info-panel"></div>
        </div>
        
        <div class="sidebar">
            <div class="section">
                <div class="section-title">Map Statistics</div>
                <div class="stat-item">
                    <span>Total Cells</span>
                    <span class="stat-value">{stats.get('cell_count', 0)}</span>
                </div>
                <div class="stat-item">
                    <span>Walkable</span>
                    <span class="stat-value">{stats.get('walkable_count', 0)}</span>
                </div>
                <div class="stat-item">
                    <span>Hazards</span>
                    <span class="stat-value">{stats.get('hazard_count', 0)}</span>
                </div>
                <div class="stat-item">
                    <span>Resources</span>
                    <span class="stat-value">{stats.get('resource_count', 0)}</span>
                </div>
            </div>
            
            <div class="section">
                <div class="section-title">Bounds</div>
                <div class="stat-item">
                    <span>X Range</span>
                    <span class="stat-value">{bounds.get('min_x', 0)} to {bounds.get('max_x', 0)}</span>
                </div>
                <div class="stat-item">
                    <span>Z Range</span>
                    <span class="stat-value">{bounds.get('min_z', 0)} to {bounds.get('max_z', 0)}</span>
                </div>
                <div class="stat-item">
                    <span>Size</span>
                    <span class="stat-value">{bounds.get('range_x', 0)} x {bounds.get('range_z', 0)}</span>
                </div>
            </div>
            
            <div class="section">
                <div class="section-title">Movement Classes</div>
                <div id="movementStats"></div>
            </div>
            
            <div class="section">
                <div class="section-title">Surface Types</div>
                <div id="surfaceStats"></div>
            </div>
        </div>
    </div>
    
    <script>
        const canvas = document.getElementById('mapCanvas');
        if (!canvas) {{
            console.error('Canvas element not found!');
        }}
        const ctx = canvas ? canvas.getContext('2d') : null;
        const infoPanel = document.getElementById('infoPanel');
        
        if (!ctx) {{
            console.error('Could not get 2D context from canvas!');
        }}
        
        // Map data
        const cells = {json.dumps(cell_data)};
        console.log('Loaded cells:', cells.length, 'cells');
        console.log('Cell data sample:', cells.length > 0 ? cells[0] : 'none');
        
        const bounds = {{
            minX: {min_x},
            maxX: {max_x},
            minZ: {min_z},
            maxZ: {max_z},
            rangeX: {range_x},
            rangeZ: {range_z}
        }};
        console.log('Bounds:', bounds);
        
        // View state
        let scale = 1;
        let offsetX = 0;
        let offsetZ = 0;
        let isDragging = false;
        let dragStart = {{x: 0, z: 0}};
        
        // Layer visibility
        let showWalkable = true;
        let showHazards = true;
        let showResources = true;
        let showConfidence = false;
        
        // Color schemes
        const surfaceColors = {{
            'stone': '#808080',
            'dirt': '#8B4513',
            'sand': '#C2B280',
            'wood': '#A0522D',
            'foliage': '#228B22',
            'water': '#3794ff',
            'lava': '#FF4500',
            'ore': '#FFD700',
            'unknown': '#555555'
        }};
        
        // Convert world coordinates to canvas coordinates
        function worldToCanvas(x, z) {{
            const padding = 40;
            const canvasX = padding + ((x - bounds.minX) / bounds.rangeX) * (canvas.width - padding * 2) * scale + offsetX;
            const canvasZ = padding + ((z - bounds.minZ) / bounds.rangeZ) * (canvas.height - padding * 2) * scale + offsetZ;
            return {{x: canvasX, z: canvasZ}};
        }}

        function getCellRect(x, z) {{
            const topLeft = worldToCanvas(x, z);
            const bottomRight = worldToCanvas(x + 1, z + 1);
            const left = Math.min(topLeft.x, bottomRight.x);
            const right = Math.max(topLeft.x, bottomRight.x);
            const top = Math.min(topLeft.z, bottomRight.z);
            const bottom = Math.max(topLeft.z, bottomRight.z);
            return {{left, top, width: right - left, height: bottom - top}};
        }}
        
        // Convert canvas coordinates to world coordinates
        function canvasToWorld(canvasX, canvasZ) {{
            const padding = 40;
            const worldX = ((canvasX - offsetX - padding) / scale) * bounds.rangeX / (canvas.width - padding * 2) + bounds.minX;
            const worldZ = ((canvasZ - offsetZ - padding) / scale) * bounds.rangeZ / (canvas.height - padding * 2) + bounds.minZ;
            return {{x: worldX, z: worldZ}};
        }}
        
        // Create lookup map for mapped cells
        const mappedCells = new Map();
        cells.forEach(cell => {{
            const key = `${{cell.x}},${{cell.z}}`;
            mappedCells.set(key, cell);
        }});
        
        // Helper function to lighten a color
        function lightenColor(color, amount = 0.3) {{
            // Handle hex colors
            if (color && color.startsWith('#')) {{
                const num = parseInt(color.slice(1), 16);
                if (isNaN(num)) return '#4ec9b0';  // Fallback to cyan if invalid
                const r = Math.min(255, ((num >> 16) & 0xff) + Math.floor(255 * amount));
                const g = Math.min(255, ((num >> 8) & 0xff) + Math.floor(255 * amount));
                const b = Math.min(255, (num & 0xff) + Math.floor(255 * amount));
                const result = `#${{r.toString(16).padStart(2, '0')}}${{g.toString(16).padStart(2, '0')}}${{b.toString(16).padStart(2, '0')}}`;
                // Ensure result is visible (not too dark)
                if (r < 50 && g < 50 && b < 50) {{
                    return '#4ec9b0';  // Force cyan if still too dark
                }}
                return result;
            }}
            // Handle rgb colors
            if (color && color.startsWith('rgb')) {{
                const matches = color.match(/\\d+/g);
                if (matches && matches.length >= 3) {{
                    const r = Math.min(255, parseInt(matches[0]) + Math.floor(255 * amount));
                    const g = Math.min(255, parseInt(matches[1]) + Math.floor(255 * amount));
                    const b = Math.min(255, parseInt(matches[2]) + Math.floor(255 * amount));
                    return `rgb(${{r}}, ${{g}}, ${{b}})`;
                }}
            }}
            // Fallback to visible color
            return '#4ec9b0';
        }}
        
        // Get cell color based on properties and active layers
        function getCellColor(cell, isMapped = true) {{
            // Unmapped cells use darker color (slightly lighter than background for visibility)
            if (!isMapped) {{
                return '#2a2d2e';  // Dark gray, slightly lighter than background #1e1e1e
            }}
            
            // Mapped cells use lighter shades
            // Priority: hazards > resources > walkability > surface
            if (showHazards && cell.hazard_flags && cell.hazard_flags.length > 0) {{
                return lightenColor('#f14c4c', 0.2);
            }}
            
            if (cell.is_fluid) {{
                return cell.surface_class === 'lava' ? lightenColor('#FF4500', 0.2) : lightenColor('#3794ff', 0.2);
            }}
            
            if (showResources && cell.resources && cell.resources.length > 0) {{
                return lightenColor('#ffd700', 0.2);
            }}
            
            if (showWalkable) {{
                if (cell.walkable) {{
                    // Color by surface type - lighten all surface colors
                    const baseColor = surfaceColors[cell.surface_class] || '#4ec9b0';
                    return lightenColor(baseColor, 0.3);
                }} else {{
                    return lightenColor('#444444', 0.3);
                }}
            }}
            
            if (showConfidence) {{
                const conf = cell.confidence || 0;
                const r = Math.min(255, Math.floor(255 * (1 - conf)) + Math.floor(255 * 0.3));
                const g = Math.min(255, Math.floor(255 * conf) + Math.floor(255 * 0.3));
                const b = Math.min(255, 100 + Math.floor(255 * 0.3));
                return `rgb(${{r}}, ${{g}}, ${{b}})`;
            }}
            
            // Fallback: ensure all mapped cells get a visible color
            // Default to a light cyan/teal that's clearly visible
            const baseColor = surfaceColors[cell.surface_class] || '#4ec9b0';
            return lightenColor(baseColor, 0.3);
        }}
        
        // Draw map
        function drawMap() {{
            if (!ctx || !canvas) {{
                console.error('Canvas or context not available!');
                return;
            }}
            
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            if (cells.length === 0) {{
                console.warn('No cells to draw!');
                return;
            }}
            
            console.log('Drawing map with', cells.length, 'cells, scale:', scale, 'bounds:', bounds);
            
            // Draw background grid aligned to 1x1 block boundaries (integer coordinates)
            ctx.strokeStyle = '#2a2d2e';
            ctx.lineWidth = 0.5;
            
            // Draw vertical grid lines at integer x coordinates
            const minXInt = Math.floor(bounds.minX);
            const maxXInt = Math.ceil(bounds.maxX);
            for (let xInt = minXInt; xInt <= maxXInt; xInt++) {{
                const canvasPos = worldToCanvas(xInt, 0);
                // Only draw if within visible area
                if (canvasPos.x >= -10 && canvasPos.x <= canvas.width + 10) {{
                    ctx.beginPath();
                    ctx.moveTo(canvasPos.x, 0);
                    ctx.lineTo(canvasPos.x, canvas.height);
                    ctx.stroke();
                }}
            }}
            
            // Draw horizontal grid lines at integer z coordinates
            const minZInt = Math.floor(bounds.minZ);
            const maxZInt = Math.ceil(bounds.maxZ);
            for (let zInt = minZInt; zInt <= maxZInt; zInt++) {{
                const canvasPos = worldToCanvas(0, zInt);
                // Only draw if within visible area
                if (canvasPos.z >= -10 && canvasPos.z <= canvas.height + 10) {{
                    ctx.beginPath();
                    ctx.moveTo(0, canvasPos.z);
                    ctx.lineTo(canvas.width, canvasPos.z);
                    ctx.stroke();
                }}
            }}
            
            // Draw mapped cells first (always draw these, regardless of bounds or visibility)
            let cellsDrawn = 0;
            cells.forEach(cell => {{
                // Validate cell has coordinates
                if (cell.x === undefined || cell.z === undefined || cell.x === null || cell.z === null) {{
                    console.warn('Cell missing coordinates:', cell);
                    return;
                }}
                
                const rect = getCellRect(cell.x, cell.z);
                
                // Validate rectangle is valid
                if (isNaN(rect.left) || isNaN(rect.top) || isNaN(rect.width) || isNaN(rect.height)) {{
                    console.warn('Invalid rectangle for cell:', cell, 'rect:', rect);
                    return;
                }}
                
                const color = getCellColor(cell, true);
                
                ctx.fillStyle = color;
                ctx.fillRect(rect.left, rect.top, rect.width, rect.height);
                cellsDrawn++;
                
                // Draw border based on movement class
                if (cell.movement_class === 'step_up') {{
                    ctx.strokeStyle = '#89d185';
                    ctx.lineWidth = 2;
                    ctx.strokeRect(rect.left, rect.top, rect.width, rect.height);
                }} else if (cell.movement_class === 'drop') {{
                    ctx.strokeStyle = '#cca700';
                    ctx.lineWidth = 2;
                    ctx.strokeRect(rect.left, rect.top, rect.width, rect.height);
                }} else if (cell.observation_mode === 'inferred') {{
                    ctx.strokeStyle = '#666';
                    ctx.lineWidth = 1;
                    ctx.setLineDash([2, 2]);
                    ctx.strokeRect(rect.left, rect.top, rect.width, rect.height);
                    ctx.setLineDash([]);
                }}
            }});
            
            console.log('Drew', cellsDrawn, 'mapped cells');
            
            // Draw unmapped cells in bounds (for visual contrast)
            for (let xInt = minXInt; xInt <= maxXInt; xInt++) {{
                for (let zInt = minZInt; zInt <= maxZInt; zInt++) {{
                    const key = `${{xInt}},${{zInt}}`;
                    const cell = mappedCells.get(key);
                    
                    // Skip if already drawn as mapped cell
                    if (cell !== undefined) {{
                        continue;
                    }}
                    
                    const rect = getCellRect(xInt, zInt);
                    
                    // Skip if outside visible area
                    if (rect.left + rect.width < -10 || rect.left > canvas.width + 10 ||
                        rect.top + rect.height < -10 || rect.top > canvas.height + 10) {{
                        continue;
                    }}
                    
                    // Draw unmapped cell with darker color
                    ctx.fillStyle = '#2a2d2e';
                    ctx.fillRect(rect.left, rect.top, rect.width, rect.height);
                }}
            }}
            
            // Draw axis labels
            ctx.fillStyle = '#888';
            ctx.font = '11px sans-serif';
            ctx.fillText(`X: ${{bounds.minX}} to ${{bounds.maxX}}`, 10, canvas.height - 10);
            ctx.fillText(`Z: ${{bounds.minZ}} to ${{bounds.maxZ}}`, canvas.width - 120, 20);
        }}
        
        // Mouse events
        canvas.addEventListener('mousedown', (e) => {{
            isDragging = true;
            dragStart.x = e.offsetX;
            dragStart.z = e.offsetY;
        }});
        
        // Track currently hovered cell to avoid unnecessary updates
        let hoveredCellKey = null;
        
        canvas.addEventListener('mousemove', (e) => {{
            if (isDragging) {{
                offsetX += e.offsetX - dragStart.x;
                offsetZ += e.offsetY - dragStart.z;
                dragStart.x = e.offsetX;
                dragStart.z = e.offsetY;
                drawMap();
                // Clear hover when dragging
                if (hoveredCellKey !== null) {{
                    hoveredCellKey = null;
                    infoPanel.classList.remove('visible');
                }}
            }} else {{
                // Convert canvas coordinates to world coordinates, then to integer cell coordinates
                const worldPos = canvasToWorld(e.offsetX, e.offsetY);
                const cellX = Math.floor(worldPos.x);
                const cellZ = Math.floor(worldPos.z);
                const currentCellKey = `${{cellX}},${{cellZ}}`;
                
                // Only update if hovering over a different cell
                if (currentCellKey === hoveredCellKey) {{
                    return;  // Already showing this cell's info
                }}
                
                hoveredCellKey = currentCellKey;
                
                // Look up exact cell at these coordinates using mappedCells Map
                const cell = mappedCells.get(currentCellKey);
                
                if (cell) {{
                    let infoHtml = `
                        <div class="info-row">
                            <span class="info-label">Position</span>
                            <span class="info-value">(${{cell.x}}, ${{cell.z}})</span>
                        </div>
                        ${{cell.support_y !== null && cell.support_y !== undefined ? `
                        <div class="info-row">
                            <span class="info-label">Surface Height</span>
                            <span class="info-value">Y = ${{cell.support_y.toFixed(1)}}</span>
                        </div>
                        ` : ''}}
                        <!-- Note: Height Delta removed - was agent-relative data that becomes stale. Use absolute support_y instead. -->
                        <div class="info-row">
                            <span class="info-label">Walkable</span>
                            <span class="info-value ${{cell.walkable ? 'safe' : 'warning'}}">${{cell.walkable ? 'Yes' : 'No'}}</span>
                        </div>
                        <div class="info-row">
                            <span class="info-label">Movement</span>
                            <span class="info-value">${{cell.movement_class}}</span>
                        </div>
                        <div class="info-row">
                            <span class="info-label">Surface</span>
                            <span class="info-value">${{cell.surface_class}}</span>
                        </div>
                    `;
                    
                    if (cell.support_block) {{
                        infoHtml += `
                            <div class="info-row">
                                <span class="info-label">Block</span>
                                <span class="info-value">${{cell.support_block}}</span>
                            </div>
                        `;
                    }}
                    
                    if (cell.hazard_flags && cell.hazard_flags.length > 0) {{
                        infoHtml += `
                            <div class="info-row">
                                <span class="info-label">Hazards</span>
                                <span class="info-value hazard">${{cell.hazard_flags.join(', ')}}</span>
                            </div>
                        `;
                    }}
                    
                    if (cell.resources && cell.resources.length > 0) {{
                        infoHtml += `
                            <div class="info-row">
                                <span class="info-label">Resources</span>
                                <span class="info-value">${{cell.resources.join(', ')}}</span>
                            </div>
                        `;
                    }}
                    
                    infoHtml += `
                        <div class="info-row">
                            <span class="info-label">Confidence</span>
                            <span class="info-value">${{(cell.confidence * 100).toFixed(0)}}%</span>
                        </div>
                        <div class="info-row">
                            <span class="info-label">Observation</span>
                            <span class="info-value">${{cell.observation_mode}}</span>
                        </div>
                    `;
                    
                    infoPanel.innerHTML = infoHtml;
                    infoPanel.classList.add('visible');
                }} else {{
                    // No cell at this location - clear the info panel
                    hoveredCellKey = null;
                    infoPanel.classList.remove('visible');
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
            scale = Math.max(0.2, Math.min(10, scale));
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
            // worldToCanvas: padding + ((x - minX) / rangeX) * availableSpace * scale
            // With scale=1: minX maps to padding, maxX maps to padding + availableSpace (fits exactly)
            // Use scale slightly less than 1 for padding around the map
            scale = 0.9;  // 10% padding on all sides
            offsetX = 0;
            offsetZ = 0;
            drawMap();
        }}
        
        function toggleLayer(layer) {{
            const btn = document.getElementById('btn' + layer.charAt(0).toUpperCase() + layer.slice(1));
            
            switch(layer) {{
                case 'walkable':
                    showWalkable = !showWalkable;
                    break;
                case 'hazards':
                    showHazards = !showHazards;
                    break;
                case 'resources':
                    showResources = !showResources;
                    break;
                case 'confidence':
                    showConfidence = !showConfidence;
                    break;
            }}
            
            if (btn) {{
                btn.classList.toggle('active');
            }}
            
            drawMap();
        }}
        
        function exportData() {{
            const data = {{
                map_name: '{map_name}',
                cells: cells,
                bounds: bounds,
                stats: {json.dumps(stats)},
                exported_at: new Date().toISOString()
            }};
            const blob = new Blob([JSON.stringify(data, null, 2)], {{type: 'application/json'}});
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = '{map_name}_spatial_export.json';
            a.click();
            URL.revokeObjectURL(url);
        }}
        
        // Compute and display statistics
        function computeStats() {{
            // Movement class distribution
            const movementCounts = {{}};
            const surfaceCounts = {{}};
            
            cells.forEach(cell => {{
                const mc = cell.movement_class || 'unknown';
                movementCounts[mc] = (movementCounts[mc] || 0) + 1;
                
                const sc = cell.surface_class || 'unknown';
                surfaceCounts[sc] = (surfaceCounts[sc] || 0) + 1;
            }});
            
            // Display movement stats
            const movementDiv = document.getElementById('movementStats');
            movementDiv.innerHTML = Object.entries(movementCounts)
                .sort((a, b) => b[1] - a[1])
                .map(([key, value]) => `
                    <div class="stat-item">
                        <span>${{key}}</span>
                        <span class="stat-value">${{value}}</span>
                    </div>
                `).join('');
            
            // Display surface stats
            const surfaceDiv = document.getElementById('surfaceStats');
            surfaceDiv.innerHTML = Object.entries(surfaceCounts)
                .sort((a, b) => b[1] - a[1])
                .slice(0, 8)
                .map(([key, value]) => `
                    <div class="stat-item">
                        <span>${{key}}</span>
                        <span class="stat-value">${{value}}</span>
                    </div>
                `).join('');
        }}
        
        // Initial draw - ensure canvas is ready
        // Use requestAnimationFrame to ensure DOM is fully loaded
        if (document.readyState === 'loading') {{
            document.addEventListener('DOMContentLoaded', () => {{
                fitToBounds();
                computeStats();
            }});
        }} else {{
            // DOM already loaded
            fitToBounds();
            computeStats();
        }}
    </script>
</body>
</html>
"""
    
    return html


def tool(input_value=None, **kwargs):
    """
    Generate HTML visualization of the spatial map and open in browser.
    
    Displays cell-based SpatialMap data with zoomable, pannable interface.
    Shows walkability, hazards, resources, and observation confidence.
    
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
    world_name = kwargs.get('world_name', 'minecraft')
    target = kwargs.get('target') or kwargs.get('map_name')
    output_file = kwargs.get('output_file')
    
    if not resource_manager:
        return executor._create_uniform_return(
            'failed',
            value="Resource manager not available",
            reason="no_resource_manager"
        )
    
    # Resolve map name
    default_map_name = f"{agent_name}-minecraft_map"
    
    if not target:
        map_name = default_map_name
    elif isinstance(target, str) and target.startswith('$'):
        map_name = target.lstrip('$')
    else:
        map_name = str(target)
    
    # Get base_dir from resource_manager if available
    base_dir = None
    if resource_manager and hasattr(resource_manager, 'base_dir'):
        base_dir = resource_manager.base_dir
    
    # Get SpatialMap instance
    spatial_map = _get_spatial_map(agent_name, world_name, base_dir)
    
    # Get all cells and stats
    cells = spatial_map.get_all_cells()
    stats = spatial_map.get_stats()
    
    if not cells:
        result_text = f"Spatial map '{map_name}' is empty - no cells mapped yet. Use mc-map-update to add observations."
        return executor._create_uniform_return(
            'success',
            value=result_text,
            extra={"map_name": map_name, "cell_count": 0, "message": "empty_map"}
        )
    
    # Generate HTML
    try:
        html_content = _generate_html_visualization(cells, map_name, stats)
    except Exception as e:
        logger.error(f"Failed to generate HTML visualization: {e}", exc_info=True)
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to generate visualization: {str(e)}",
            reason="html_generation_failed"
        )
    
    # Determine output file path
    if not output_file:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_map_name = map_name.replace('/', '_').replace('\\', '_')
        output_file = f"spatial_map_{safe_map_name}_{timestamp}.html"
    
    output_path = Path(output_file)
    
    # If relative path, save to /tmp
    if not output_path.is_absolute():
        tmp_dir = Path("/tmp")
        tmp_dir.mkdir(parents=True, exist_ok=True)
        output_path = tmp_dir / output_path
    
    # Write HTML file
    try:
        output_path.write_text(html_content, encoding='utf-8')
        logger.info(f"Generated spatial map visualization: {output_path}")
    except Exception as e:
        logger.error(f"Failed to write HTML file: {e}", exc_info=True)
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to save visualization file: {str(e)}",
            reason="file_write_failed"
        )
    
    # Open in browser
    try:
        file_url = f"file://{output_path.absolute()}"
        webbrowser.open(file_url)
        logger.info(f"Opened spatial map visualization in browser: {file_url}")
    except Exception as e:
        logger.warning(f"Failed to open browser automatically: {e}. File saved at: {output_path}")
    
    result_text = f"Spatial map visualization generated: {output_path.absolute()}\n{stats['cell_count']} cells visualized"
    
    # Extract metadata fields for extra
    extra_metadata = {
        "map_name": map_name,
        "file_path": str(output_path.absolute()),
        "output_file": str(output_path.absolute()),
        "cell_count": stats['cell_count'],
        "stats": stats
    }
    
    return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Test with mock data
    result = tool(
        target="minecraft_map",
        resource_manager=None,
        agent_name="test"
    )
    print(result)
