import re
from datetime import datetime
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
BG_COLOR = (15, 15, 20)  # Dark terminal background
TEXT_COLOR = (200, 200, 200)
FONT_SIZE = 14
LINE_SPACING = 4
PADDING = 20
FPS_SCALE = 1.0  # 1.0 = Real time. 2.0 = 2x speed.
FINAL_PAUSE_DURATION = 3.0  # Seconds to hold the final frame

# Colors for highlighting
COLORS = {
    "timestamp": (100, 100, 100),
    "node": (0, 255, 136),  # Greenish for nodes
    "INFO": (100, 200, 255), # Light Blue
    "ERROR": (255, 80, 80),  # Red
    "Jill": (255, 0, 255),   # Magenta for the persona
    "THOUGHTS": (255, 200, 80) # Orange/Gold
}

def parse_logs(log_text):
    lines = log_text.strip().split('\n')
    parsed = []
    
    # Regex to extract Time, Node, Level, Message
    # Matches: "2025-12-14 18:39:23 - node_name - LEVEL - Message"
    pattern = re.compile(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) - (.*?) - (.*?) - (.*)')
    
    for line in lines:
        match = pattern.match(line)
        if match:
            ts_str, node, level, msg = match.groups()
            dt = datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S")
            parsed.append({
                'dt': dt,
                'ts_str': ts_str.split(' ')[1], # Just keep HH:MM:SS
                'node': node,
                'level': level,
                'msg': msg,
                'raw': line
            })
    return parsed

def create_mp4(parsed_logs):
    try:
        font = ImageFont.truetype("Arial", FONT_SIZE)
    except IOError:
        font = ImageFont.load_default()

    frames = []

    visible_lines = []
    max_lines = (HEIGHT - (2 * PADDING)) // (FONT_SIZE + LINE_SPACING)

    for i, entry in enumerate(parsed_logs):

        # compute time delta for pacing
        if i < len(parsed_logs) - 1:
            next_dt = parsed_logs[i+1]['dt']
            diff = (next_dt - entry['dt']).total_seconds()
            duration = max(0.1, diff) / FPS_SCALE
        else:
            duration = FINAL_PAUSE_DURATION

        # scrolling logic
        visible_lines.append(entry)
        if len(visible_lines) > max_lines:
            visible_lines.pop(0)

        # draw frame
        img = Image.new('RGB', (WIDTH, HEIGHT), BG_COLOR)
        draw = ImageDraw.Draw(img)

        y = PADDING
        for line_data in visible_lines:
            draw.text((PADDING, y), line_data['ts_str'], font=font, fill=COLORS["timestamp"])
            x_cursor = PADDING + 80
            draw.text((x_cursor, y), line_data['node'], font=font, fill=COLORS["node"])
            x_cursor += 150
            level_color = COLORS.get(line_data['level'], TEXT_COLOR)
            draw.text((x_cursor, y), line_data['level'], font=font, fill=level_color)
            x_cursor += 60

            msg = line_data['msg']
            msg_color = TEXT_COLOR
            if "Jill" in msg: msg_color = COLORS["Jill"]
            if "THOUGHTS" in msg: msg_color = COLORS["THOUGHTS"]
            if "ERROR" in line_data['level']: msg_color = COLORS["ERROR"]

            draw.text((x_cursor, y), msg, font=font, fill=msg_color)
            y += FONT_SIZE + LINE_SPACING

        # --- THE KEY CHANGE: expand each log event into many frames ---
        num_frames = max(1, int(30 * duration))  # 30 fps * duration
        for _ in range(num_frames):
            frames.append(img)

    print("Saving MP4...")
    imageio.mimsave("jill_log_scroll.mp4", frames, fps=30)
    print("Done.")

if __name__ == "__main__":
    parsed = parse_logs(LOG_DATA)
    create_mp4(parsed)