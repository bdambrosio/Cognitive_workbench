"""
Minecraft local grid visualization tool.
Generates HTML file with 3D visualization of the session-local occupancy grid and opens it in browser.
"""

import json
import logging
import math
import tempfile
import webbrowser
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


def _block_color(name: Optional[str]) -> str:
    """Map block name to hex color."""
    if not name:
        return "#888888"
    n = name.split(":")[-1] if ":" in name else str(name).lower()
    # Common Minecraft blocks
    colors = {
        "air": "#ffffff",
        "stone": "#808080",
        "dirt": "#8b4513",
        "grass": "#7cba3d",
        "cobblestone": "#808080",
        "wood": "#8b4513",
        "log": "#654321",
        "sand": "#c2b280",
        "gravel": "#808080",
        "water": "#3794ff",
        "lava": "#ff4500",
        "coal_ore": "#2d2d2d",
        "iron_ore": "#d4af37",
        "gold_ore": "#ffd700",
        "diamond_ore": "#00ffff",
    }
    for key, color in colors.items():
        if key in n:
            return color
    return "#888888"  # Default gray


def _generate_html(grid: Dict[str, Any]) -> str:
    """Generate Three.js HTML visualization."""
    center = grid.get("center", {})
    radius = int(grid.get("radius", 10))
    cells = grid.get("cells", {})
    
    cx = int(center.get("x", 0))
    cy = int(center.get("y", 0))
    cz = int(center.get("z", 0))
    
    # Parse cells into voxel list
    voxels = []
    for key, entry in cells.items():
        if not isinstance(entry, dict):
            continue
        name = entry.get("name", "unknown")
        if not isinstance(name, str):
            continue
        try:
            x, y, z = map(int, key.split(","))
            voxels.append({"x": x, "y": y, "z": z, "name": name})
        except Exception:
            continue
    
    voxel_data_json = json.dumps(voxels)
    center_json = json.dumps({"x": cx, "y": cy, "z": cz})
    
    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Local Grid 3D Visualization</title>
    <style>
        body {{ margin: 0; overflow: hidden; background: #1a1a1a; font-family: monospace; }}
        #info {{ position: absolute; top: 10px; left: 10px; color: #fff; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; font-size: 12px; }}
        #info h3 {{ margin: 0 0 5px 0; }}
        #controls {{ position: absolute; bottom: 10px; left: 10px; color: #fff; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; font-size: 11px; }}
    </style>
</head>
<body>
    <div id="info">
        <h3>Local Grid 3D</h3>
        <div>Center: ({cx}, {cy}, {cz})</div>
        <div>Agent Position: ({cx + 0.5:.1f}, {cy + 0.5:.1f}, {cz + 0.5:.1f})</div>
        <div>Radius: {radius}</div>
        <div>Voxels: {len(voxels)}</div>
        <div style="color: #ff0000; margin-top: 5px;">🔴 Red sphere = Agent location</div>
    </div>
    <div id="controls">
        <strong>Controls:</strong><br>
        Left-drag: Rotate<br>
        Right-drag: Pan<br>
        Scroll: Zoom<br>
        <button onclick="location.reload()" style="margin-top: 5px;">Refresh</button>
    </div>
    
    <script type="importmap">
        {{"imports": {{"three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js", "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}}}
    </script>
    <script type="module">
        import * as THREE from 'three';
        import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';
        
        const voxels = {voxel_data_json};
        const center = {center_json};
        const radius = {radius};
        
        // Scene setup
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x1a1a1a);
        
        const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.set(center.x + radius * 1.5, center.y + radius * 1.5, center.z + radius * 1.5);
        
        const renderer = new THREE.WebGLRenderer({{ antialias: true }});
        renderer.setSize(window.innerWidth, window.innerHeight);
        document.body.appendChild(renderer.domElement);
        
        // Controls
        const controls = new OrbitControls(camera, renderer.domElement);
        controls.target.set(center.x, center.y, center.z);
        controls.enableDamping = true;
        controls.dampingFactor = 0.1;
        controls.minDistance = 0.5;
        controls.maxDistance = radius * 5;
        controls.zoomToCursor = true;
        
        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(radius, radius * 2, radius);
        scene.add(directionalLight);
        
        // Grid helper (optional, shows block grid)
        const gridHelper = new THREE.GridHelper(radius * 2, radius * 2, 0x444444, 0x222222);
        gridHelper.position.set(center.x, center.y, center.z);
        scene.add(gridHelper);
        
        // Radius boundary wireframe
        const boundaryGeometry = new THREE.BoxGeometry(radius * 2 + 1, radius * 2 + 1, radius * 2 + 1);
        const boundaryMaterial = new THREE.MeshBasicMaterial({{ color: 0x00ff00, wireframe: true, opacity: 0.3, transparent: true }});
        const boundary = new THREE.Mesh(boundaryGeometry, boundaryMaterial);
        boundary.position.set(center.x, center.y, center.z);
        scene.add(boundary);
        
        // Agent center marker (red sphere, larger and always visible)
        const centerGeometry = new THREE.SphereGeometry(0.5, 16, 16);
        const centerMaterial = new THREE.MeshBasicMaterial({{ color: 0xff0000, depthTest: false, depthWrite: false }});
        const centerMarker = new THREE.Mesh(centerGeometry, centerMaterial);
        centerMarker.position.set(center.x + 0.5, center.y + 0.5, center.z + 0.5);
        centerMarker.renderOrder = 999; // Render on top
        scene.add(centerMarker);
        
        // Add a helper line pointing up from agent position
        const lineGeometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(center.x + 0.5, center.y + 0.5, center.z + 0.5),
            new THREE.Vector3(center.x + 0.5, center.y + 2.0, center.z + 0.5)
        ]);
        const lineMaterial = new THREE.LineBasicMaterial({{ color: 0xff0000, linewidth: 2 }});
        const centerLine = new THREE.Line(lineGeometry, lineMaterial);
        centerLine.renderOrder = 999;
        scene.add(centerLine);
        
        // Block name to color mapping
        function blockColor(name) {{
            if (!name) return 0x888888;
            const n = name.toLowerCase();
            if (n.includes('air')) return 0xffffff;
            if (n.includes('stone') || n.includes('cobble')) return 0x808080;
            if (n.includes('dirt')) return 0x8b4513;
            if (n.includes('grass')) return 0x7cba3d;
            if (n.includes('wood') || n.includes('log')) return 0x8b4513;
            if (n.includes('sand')) return 0xc2b280;
            if (n.includes('water')) return 0x3794ff;
            if (n.includes('lava')) return 0xff4500;
            if (n.includes('coal')) return 0x2d2d2d;
            if (n.includes('iron')) return 0xd4af37;
            if (n.includes('gold')) return 0xffd700;
            if (n.includes('diamond')) return 0x00ffff;
            return 0x888888;
        }}
        
        // Voxel cubes
        const voxelGroup = new THREE.Group();
        voxels.forEach(v => {{
            const geometry = new THREE.BoxGeometry(1, 1, 1);
            const isAir = v.name && v.name.toLowerCase().includes('air');
            const material = new THREE.MeshLambertMaterial({{
                color: blockColor(v.name),
                transparent: isAir,
                opacity: isAir ? 0.2 : 1.0
            }});
            const cube = new THREE.Mesh(geometry, material);
            cube.position.set(v.x + 0.5, v.y + 0.5, v.z + 0.5);
            cube.userData = {{ name: v.name, x: v.x, y: v.y, z: v.z }};
            voxelGroup.add(cube);
        }});
        scene.add(voxelGroup);
        
        // Raycaster for hover info
        const raycaster = new THREE.Raycaster();
        const mouse = new THREE.Vector2();
        let hovered = null;
        
        function onMouseMove(event) {{
            mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
            mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
            raycaster.setFromCamera(mouse, camera);
            const intersects = raycaster.intersectObjects(voxelGroup.children);
            if (intersects.length > 0) {{
                const obj = intersects[0].object;
                if (hovered !== obj) {{
                    if (hovered) hovered.material.emissive.setHex(0x000000);
                    hovered = obj;
                    obj.material.emissive.setHex(0x444444);
                    const info = document.getElementById('info');
                    info.innerHTML = `
                        <h3>Local Grid 3D</h3>
                        <div>Center: ({cx}, {cy}, {cz})</div>
                        <div>Radius: {radius}</div>
                        <div>Voxels: {len(voxels)}</div>
                        <div style="margin-top: 10px; color: #4ec9b0;">
                            Hover: ${{obj.userData.name}}<br>
                            Position: (${{obj.userData.x}}, ${{obj.userData.y}}, ${{obj.userData.z}})
                        </div>
                    `;
                }}
            }} else {{
                if (hovered) {{
                    hovered.material.emissive.setHex(0x000000);
                    hovered = null;
                    const info = document.getElementById('info');
                    info.innerHTML = `
                        <h3>Local Grid 3D</h3>
                        <div>Center: ({cx}, {cy}, {cz})</div>
                        <div>Radius: {radius}</div>
                        <div>Voxels: {len(voxels)}</div>
                    `;
                }}
            }}
        }}
        
        window.addEventListener('mousemove', onMouseMove);
        window.addEventListener('resize', () => {{
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        }});
        
        // Animation loop
        function animate() {{
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }}
        animate();
    </script>
</body>
</html>
"""
    return html


def tool(input_value=None, **kwargs):
    """
    Generate HTML visualization of the session-local 3D occupancy grid and open in browser.
    
    Args:
        input_value: Ignored
        output_file: Optional output file path (default: /tmp/local_grid_viz.html)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    # Read grid from world_state
    try:
        from local_grid import get_or_init
        grid = get_or_init(executor)
    except Exception as e:
        logger.warning(f"Failed to import/read local_grid: {e}")
        grid = executor.get_world_state("local_grid")
        if not isinstance(grid, dict):
            return executor._create_uniform_return(
                "failed",
                value="Local grid not available (empty or not initialized)",
                reason="grid_not_available"
            )
    
    # Generate HTML
    try:
        html = _generate_html(grid)
    except Exception as e:
        logger.error(f"Failed to generate HTML: {e}", exc_info=True)
        return executor._create_uniform_return(
            "failed",
            value=f"Failed to generate visualization: {e}",
            reason="generation_failed"
        )
    
    # Write to file
    output_file = kwargs.get("output_file")
    if output_file:
        output_path = Path(output_file)
    else:
        output_path = Path(tempfile.gettempdir()) / "local_grid_viz.html"
    
    try:
        output_path.write_text(html, encoding="utf-8")
        logger.info(f"Generated local grid visualization: {output_path}")
    except Exception as e:
        logger.error(f"Failed to write HTML file: {e}")
        return executor._create_uniform_return(
            "failed",
            value=f"Failed to write visualization file: {e}",
            reason="write_failed"
        )
    
    # Open in browser
    try:
        file_url = f"file://{output_path.absolute()}"
        webbrowser.open(file_url)
        logger.info(f"Opened local grid visualization in browser: {file_url}")
    except Exception as e:
        logger.warning(f"Failed to open browser automatically: {e}. File saved at: {output_path}")
    
    cells = grid.get("cells", {})
    cell_count = len(cells) if isinstance(cells, dict) else 0
    center = grid.get("center", {})
    
    result_text = f"Local grid visualization generated: {output_path.absolute()}\n{cell_count} voxels visualized"
    
    return executor._create_uniform_return(
        "success",
        value=result_text,
        extra={
            "file_path": str(output_path.absolute()),
            "cell_count": cell_count,
            "center": center,
            "radius": grid.get("radius", 10),
        }
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("mc-local-grid-visualize module loaded")
