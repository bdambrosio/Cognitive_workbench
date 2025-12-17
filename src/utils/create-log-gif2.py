import re
from datetime import datetime, timedelta
from PIL import Image, ImageDraw, ImageFont
import imageio

# --- CONFIGURATION ---
LOG_DATA = """
2025-12-14 18:39:23 - executive_node - INFO - Cleared 3 Notes, 1 Collections, 5 planner bindings
2025-12-14 18:39:23 - executive_node - INFO - 📥 Jill Queued: "goal: subject: high_school_physics..."
2025-12-14 18:39:23 - memory - INFO - 💬 Added conversation entry for User
2025-12-14 18:39:23 - executive_node - INFO - 🚀 Goal received, unpausing execution
2025-12-14 18:39:23 - executive_node - INFO - 📥 Jill Received goal: "goal: subject: high_school_physics..."
2025-12-14 18:39:23 - executive_node - INFO - 🛑 Jill interrupting existing plan for new goal
2025-12-14 18:39:23 - executive_node - INFO - 🎯 Published goal for Jill
2025-12-14 18:39:23 - executive_node - INFO - 🧩 Jill infospace planning for goal
2025-12-14 18:39:24 - incremental_planner - INFO - Found 1 similar plans
2025-12-14 18:39:24 - incremental_planner - INFO - Stage 1: Analysis + tool selection
2025-12-14 18:39:25 - incremental_planner - INFO - SELECTED_TOOLS_JSON:  ["generate-note", "think", "coerce", "calculate"]
2025-12-14 18:39:25 - incremental_planner - INFO - FIRST_TASK:  Generate the relevant physics formula
2025-12-14 18:39:25 - infospace_executor - INFO - Executing action: {"type": "generate-note"}
2025-12-14 18:39:27 - tool_generate_note - INFO - generate-note complete: output_len=1095
2025-12-14 18:39:27 - incremental_planner - INFO - THOUGHTS: I have generated the physics concept explanation...
2025-12-14 18:39:27 - incremental_planner - INFO - NEXT_TASK: Extract the formula from the generated note
2025-12-14 18:39:28 - infospace_executor - INFO - Executing action: {"type": "refine", "target": "$physics_concept"}
2025-12-14 18:39:28 - tool_refine - INFO - refine complete: output_len=6
2025-12-14 18:39:29 - infospace_executor - ERROR - Unbound variable: $capacitance_mf
2025-12-14 18:39:29 - incremental_planner - INFO - Step 2: coerce -> ERROR | coerce failed
2025-12-14 18:39:29 - incremental_planner - INFO - THOUGHTS: I need to create a note for the capacitance value first...
2025-12-14 18:39:30 - incremental_planner - INFO - REQUEST_TOOLS: ["create-note", "coerce"]
2025-12-14 18:39:30 - infospace_executor - INFO - Executing action: {"type": "create-note", "value": "2"}
2025-12-14 18:39:31 - infospace_executor - INFO - Executing action: {"type": "create-note", "value": "mF"}
2025-12-14 18:39:32 - infospace_executor - INFO - Executing action: {"type": "calculate", "value": "2 * 1e-3"}
2025-12-14 18:39:33 - infospace_executor - INFO - Executing action: {"type": "calculate", "value": "0.002 * 5"}
2025-12-14 18:39:34 - infospace_executor - INFO - Executing action: {"type": "think", "value": "Calculated charge 0.01 C matches choice B"}
2025-12-14 18:39:35 - incremental_planner - INFO - FINAL_ANSWER: ANSWER: B
2025-12-14 18:39:35 - executive_node - INFO - 📋 Published current plan for Jill
2025-12-14 18:39:35 - executive_node - INFO - 📤 Published FINAL_ANSWER to action log
"""

# Visual Settings
WIDTH = 800
HEIGHT = 500
BG_COLOR = (15, 15, 20)
TEXT_COLOR = (200, 200, 200)
FONT_SIZE = 14
LINE_SPACING = 4
PADDING = 20

# TIME SETTINGS (THE FIX)
FPS = 15  # Fixed frames per second. High enough to look smooth, low enough to save size.
FINAL_PAUSE_SECONDS = 4.0 

# Colors
COLORS = {
    "timestamp": (100, 100, 100),
    "node": (0, 255, 136),
    "INFO": (100, 200, 255),
    "ERROR": (255, 80, 80),
    "Jill": (255, 0, 255),
    "THOUGHTS": (255, 200, 80)
}

def parse_logs(log_text):
    lines = log_text.strip().split('\n')
    parsed = []
    pattern = re.compile(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) - (.*?) - (.*?) - (.*)')
    
    # We add a micro-delay counter to handle lines with the EXACT same second
    # so they cascade in rather than popping in all at once.
    last_dt = None
    micro_offset = 0
    
    for line in lines:
        match = pattern.match(line)
        if match:
            ts_str, node, level, msg = match.groups()
            dt = datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S")
            
            # Artificial cascade for logs in the same second
            if dt == last_dt:
                micro_offset += 0.1 # Add 100ms offset
            else:
                micro_offset = 0
                last_dt = dt
                
            display_dt = dt + timedelta(seconds=micro_offset)

            parsed.append({
                'dt': display_dt,
                'ts_str': ts_str.split(' ')[1],
                'node': node,
                'level': level,
                'msg': msg
            })
    return parsed

def create_realtime_gif(parsed_logs):
    try:
        font = ImageFont.truetype("Arial", FONT_SIZE)
    except IOError:
        font = ImageFont.load_default()

    start_time = parsed_logs[0]['dt']
    end_time = parsed_logs[-1]['dt']
    
    # Calculate total duration needed
    total_duration = (end_time - start_time).total_seconds() + FINAL_PAUSE_SECONDS
    total_frames = int(total_duration * FPS)
    
    frames = []
    
    print(f"Generating {total_frames} frames (Fixed {FPS} FPS)...")
    
    max_lines = (HEIGHT - (2 * PADDING)) // (FONT_SIZE + LINE_SPACING)

    for f in range(total_frames):
        # Current 'simulation' time
        sim_time = start_time + timedelta(seconds=f / FPS)
        
        # Get all logs that have occurred by this time
        visible_logs = [entry for entry in parsed_logs if entry['dt'] <= sim_time]
        
        # Scroll Logic: Keep only the last N logs
        if len(visible_logs) > max_lines:
            visible_logs = visible_logs[-max_lines:]
            
        # Draw Frame
        img = Image.new('RGB', (WIDTH, HEIGHT), BG_COLOR)
        draw = ImageDraw.Draw(img)
        
        y = PADDING
        for line_data in visible_logs:
            # Draw Timestamp
            draw.text((PADDING, y), line_data['ts_str'], font=font, fill=COLORS["timestamp"])
            x_cursor = PADDING + 80
            
            # Draw Node
            draw.text((x_cursor, y), line_data['node'], font=font, fill=COLORS["node"])
            x_cursor += 150
            
            # Draw Level
            level_color = COLORS.get(line_data['level'], TEXT_COLOR)
            draw.text((x_cursor, y), line_data['level'], font=font, fill=level_color)
            x_cursor += 60
            
            # Draw Message
            msg = line_data['msg']
            msg_color = TEXT_COLOR
            if "Jill" in msg: msg_color = COLORS["Jill"]
            if "THOUGHTS" in msg: msg_color = COLORS["THOUGHTS"]
            if "ERROR" in line_data['level']: msg_color = COLORS["ERROR"]
            
            draw.text((x_cursor, y), msg, font=font, fill=msg_color)
            y += FONT_SIZE + LINE_SPACING
            
        frames.append(img)

    print("Saving GIF...")
    # duration here implies "seconds per frame". 1/15 = 0.066s
    imageio.mimsave('jill_fixed_fps.gif', frames, duration=1/FPS, loop=0)
    print("Done! Saved as jill_fixed_fps.gif")

if __name__ == "__main__":
    parsed = parse_logs(LOG_DATA)
    create_realtime_gif(parsed)