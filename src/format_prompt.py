from ast import literal_eval
from PyQt6.QtWidgets import (QApplication, QTextEdit, QVBoxLayout, QWidget, 
                            QPushButton, QDialog, QProgressDialog, QMessageBox,
                            QFileDialog, QComboBox, QLabel, QHBoxLayout, QLineEdit, QCheckBox, QMenu)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QUrl, QByteArray, QTimer
from PyQt6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply
from PyQt6.QtGui import QFont, QTextCursor, QKeySequence, QClipboard, QShortcut
from PyQt6.QtGui import QTextDocument
from pathlib import Path
import json
import re
import sys
import os
import re
from typing import Optional
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


def format_python_structure(text):
    """Format Python literal structures (dicts, lists) with line breaks for readability.
    
    Safely parses Python literals and adds line breaks after top-level key-value pairs
    or list items to improve readability. Outputs clean, unescaped quotes.
    """
    try:
        # Try to parse as Python literal
        parsed = literal_eval(text.strip())
        
        if isinstance(parsed, dict):
            # Format dict with line breaks after each top-level key-value pair
            formatted_lines = []
            for i, (key, value) in enumerate(parsed.items()):
                if i > 0:
                    formatted_lines.append('')  # Add blank line between items
                # Use clean quotes instead of repr() for better readability
                formatted_lines.append(f"'{key}': {_format_value_clean(value)}")
            return '\n'.join(formatted_lines)
            
        elif isinstance(parsed, list):
            # Format list with line breaks after each item
            formatted_lines = []
            for i, item in enumerate(parsed):
                if i > 0:
                    formatted_lines.append('')  # Add blank line between items
                formatted_lines.append(_format_value_clean(item))
            return '\n'.join(formatted_lines)
            
        else:
            # Not a dict or list, return as-is
            return text
            
    except (ValueError, SyntaxError, RecursionError):
        # If parsing fails, return original text unchanged
        return text


def _format_value_clean(value):
    """Format a value with clean, unescaped quotes for better readability."""
    if isinstance(value, str):
        return f"'{value}'"
    elif isinstance(value, dict):
        # Format nested dicts inline but clean
        items = [f"'{k}': {_format_value_clean(v)}" for k, v in value.items()]
        return '{' + ', '.join(items) + '}'
    elif isinstance(value, list):
        # Format nested lists inline but clean
        items = [_format_value_clean(item) for item in value]
        return '[' + ', '.join(items) + ']'
    else:
        # For numbers, booleans, None, etc., use str() for clean output
        return str(value)


def _is_scalar_json(v):
    return isinstance(v, (str, int, float, bool)) or v is None


def _render_json_compact(node, indent=0, indent_step=2):
    sp = ' ' * indent
    if isinstance(node, dict):
        # Leaf dict: all values scalar → single line preserving insertion order
        if all(_is_scalar_json(v) for v in node.values()):
            items = [f'"{k}": {json.dumps(v, ensure_ascii=False)}' for k, v in node.items()]
            return sp + '{' + ', '.join(items) + '}'
        # Expanded dict
        parts = [sp + '{']
        first = True
        for k, v in node.items():
            if not first:
                parts[-1] += ','
            first = False
            parts.append(' ' * (indent + indent_step) + f'"{k}": ' + _render_json_compact(v, 0, indent_step).lstrip())
        parts.append(sp + '}')
        return '\n'.join(parts)
    elif isinstance(node, list):
        # Leaf list: all elements scalar → single line
        if all(_is_scalar_json(x) for x in node):
            items = [json.dumps(x, ensure_ascii=False) for x in node]
            return sp + '[' + ', '.join(items) + ']'
        # Expanded list
        parts = [sp + '[']
        for i, x in enumerate(node):
            line = _render_json_compact(x, indent + indent_step, indent_step)
            if i < len(node) - 1:
                line += ','
            parts.append(line)
        parts.append(sp + ']')
        return '\n'.join(parts)
    else:
        return sp + json.dumps(node, ensure_ascii=False)


def _format_embedded_json_blocks(text: str) -> str:
    """Scan text, find JSON objects/arrays outside quotes, and reprint them with custom rules.
    Safely replaces only spans that json.loads can parse.
    """
    n = len(text)
    i = 0
    out = []
    in_str = False
    esc = False
    stack = []  # holds opening chars '[' or '{' and start index
    spans = []

    while i < n:
        ch = text[i]
        if in_str:
            out.append(ch)
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
            i += 1
            continue
        else:
            if ch == '"':
                in_str = True
                out.append(ch)
                i += 1
                continue
            if ch in '{[':
                stack.append((ch, len(out), i))  # remember output buffer index and source index
                out.append(ch)
                i += 1
                continue
            if ch in '}]' and stack:
                open_ch, out_pos, src_pos = stack[-1]
                if (open_ch == '{' and ch == '}') or (open_ch == '[' and ch == ']'):
                    # tentatively close
                    out.append(ch)
                    stack.pop()
                    if not stack:
                        # We have a top-level JSON span in out buffer from out_pos to current end
                        span_text = ''.join(out[out_pos:])
                        # Try parse this span only
                        try:
                            obj = json.loads(span_text)
                            pretty = _render_json_compact(obj)
                            # replace segment in out
                            out = out[:out_pos]
                            out.append(pretty)
                        except Exception:
                            # leave as-is
                            pass
                    i += 1
                    continue
            out.append(ch)
            i += 1
    return ''.join(out)


def import_file():
    file_path, _ = QFileDialog.getOpenFileName(
        window,
        "Select Text File",
        "",
        "Text Files (*.txt *.log);;All Files (*)"
    )
    if file_path:
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()
                text_edit.setPlainText(content)
        except Exception as e:
            QMessageBox.critical(window, "Error", f"Failed to read file: {str(e)}")

def format_text():
    raw_text = text_edit.toPlainText()
    
    # First pass: format embedded JSON blocks with custom rules
    formatted_text = _format_embedded_json_blocks(raw_text)
    
    # Second pass: convert escaped newlines and tabs globally
    formatted_text = formatted_text.replace("\\t", "\t").replace("\\n", "\n")
    
    text_edit.setPlainText(formatted_text)

def clear_text():
    text_edit.clear()
    text_edit.setFont(textFont)


# API configuration
API_URL = "http://localhost:5000"
_model_id_cache = None
_network_manager = None


def get_network_manager():
    """Get or create QNetworkAccessManager."""
    global _network_manager
    if _network_manager is None:
        _network_manager = QNetworkAccessManager()
    return _network_manager


def get_model_id(url: str = API_URL, callback=None):
    """Get model ID from /v1/models endpoint. Uses Qt networking."""
    global _model_id_cache
    if _model_id_cache:
        if callback:
            callback(_model_id_cache)
        return _model_id_cache
    
    manager = get_network_manager()
    request = QNetworkRequest(QUrl(f"{url}/v1/models"))
    request.setHeader(QNetworkRequest.KnownHeaders.ContentTypeHeader, "application/json")
    
    def handle_reply(reply):
        global _model_id_cache
        if reply.error() == QNetworkReply.NoError:
            try:
                data = json.loads(reply.readAll().data().decode('utf-8'))
                if data.get("data") and len(data["data"]) > 0:
                    _model_id_cache = data["data"][0]["id"]
                    if callback:
                        callback(_model_id_cache)
            except Exception as e:
                print(f"[WARNING] Failed to parse model ID: {e}")
                if callback:
                    callback(None)
        else:
            print(f"[WARNING] Failed to get model ID: {reply.errorString()}")
            if callback:
                callback(None)
        reply.deleteLater()
    
    reply = manager.get(request)
    reply.finished.connect(lambda: handle_reply(reply))
    
    return None  # Async, will call callback


def submit_text():
    """Submit current text to API and display response."""
    raw_text = text_edit.toPlainText().strip()
    if not raw_text:
        QMessageBox.warning(window, "Warning", "No text to submit.")
        return
    
    # Disable submit button during request
    submit_button.setEnabled(False)
    submit_button.setText("Submitting...")
    
    def send_chat_request(model_id, retry_count=0):
        """Send chat request with given model ID."""
        messages = [{"role": "user", "content": raw_text}]
        payload = {
            "model": model_id,
            "messages": messages,
            "temperature": 0.7,
            "max_tokens": 2000
        }
        
        manager = get_network_manager()
        request = QNetworkRequest(QUrl(f"{API_URL}/v1/chat/completions"))
        request.setHeader(QNetworkRequest.KnownHeaders.ContentTypeHeader, "application/json")
        
        payload_bytes = QByteArray(json.dumps(payload).encode('utf-8'))
        
        def handle_chat_reply(reply):
            submit_button.setEnabled(True)
            submit_button.setText("Submit")
            
            if reply.error() == QNetworkReply.NoError:
                try:
                    data = json.loads(reply.readAll().data().decode('utf-8'))
                    if "choices" in data and len(data["choices"]) > 0:
                        assistant_message = data["choices"][0]["message"]["content"]
                        # Append response below the original text
                        cursor = text_edit.textCursor()
                        cursor.movePosition(QTextCursor.MoveOperation.End)
                        cursor.insertText("\n\n--- Response ---\n\n")
                        cursor.insertText(assistant_message)
                        text_edit.setTextCursor(cursor)
                    else:
                        QMessageBox.critical(window, "Error", f"Unexpected response format: {json.dumps(data, indent=2)}")
                except Exception as e:
                    QMessageBox.critical(window, "Error", f"Failed to parse response: {str(e)}")
            else:
                # Check if it's a model mismatch error
                try:
                    error_data = json.loads(reply.readAll().data().decode('utf-8'))
                    error_detail = error_data.get("detail", "")
                    
                    # Check for model mismatch pattern: "Model 'X' not found. Available model: 'Y'"
                    if "not found" in error_detail.lower() and "available model" in error_detail.lower():
                        # Extract available model from error message
                        import re
                        match = re.search(r"Available model: ['\"]([^'\"]+)['\"]", error_detail)
                        if match and retry_count == 0:
                            available_model = match.group(1)
                            global _model_id_cache
                            _model_id_cache = available_model  # Update cache
                            # Retry with correct model
                            send_chat_request(available_model, retry_count=1)
                            reply.deleteLater()
                            return
                    
                    QMessageBox.critical(window, "Error", f"HTTP Error: {json.dumps(error_data, indent=2)}")
                except:
                    QMessageBox.critical(window, "Error", f"Request failed: {reply.errorString()}")
            reply.deleteLater()
        
        reply = manager.post(request, payload_bytes)
        reply.finished.connect(lambda: handle_chat_reply(reply))
    
    def on_model_id_received(model_id):
        """Handle model ID and send chat request."""
        if not model_id:
            submit_button.setEnabled(True)
            submit_button.setText("Submit")
            QMessageBox.critical(window, "Error", "Could not determine model ID. Is the API server running?")
            return
        
        send_chat_request(model_id)
    
    # Get model ID first (async)
    get_model_id(API_URL, on_model_id_received)


class ResponseDialog(QDialog):
    def __init__(self, response_text, parent=None):
        super().__init__(parent)
        self.setWindowTitle("LLM Response")
        layout = QVBoxLayout()
        
        response_edit = QTextEdit()
        response_edit.setPlainText(response_text)
        response_edit.setReadOnly(True)
        response_edit.setStyleSheet("background-color: #2E2E2E; color: ivory;")
        layout.addWidget(response_edit)
        
        close_button = QPushButton("Close")
        close_button.clicked.connect(self.accept)
        layout.addWidget(close_button)
        
        self.setLayout(layout)
        self.resize(600, 400)

# Create the PyQt application
app = QApplication([])

# Create the main window and layout
window = QWidget()
layout = QVBoxLayout()

# Create the QTextEdit widget for text input and output
text_edit = QTextEdit()
text_edit.setStyleSheet("background-color: #2E2E2E; color: ivory;")  # Dark blue-grey background and ivory text
text_edit.setPlaceholderText("Paste your text here...")
text_edit.setTabStopDistance(40)  # Set tab stop distance (in pixels)
textFont = QFont(); textFont.setPointSize(16)
text_edit.setFont(textFont)  
text_edit.setAcceptRichText(False)

# Enable context menu with paste support
def show_context_menu(pos):
    # Preserve existing selection: only move cursor if no selection exists
    current_cursor = text_edit.textCursor()
    has_selection = current_cursor.hasSelection()
    
    if not has_selection:
        # No selection: move cursor to click position (normal behavior)
        cursor = text_edit.cursorForPosition(pos)
        text_edit.setTextCursor(cursor)
    # else: preserve existing selection (don't move cursor)
    
    menu = QMenu()
    
    # Add standard actions
    undo_action = menu.addAction("Undo")
    undo_action.setEnabled(text_edit.document().isUndoAvailable())
    undo_action.triggered.connect(text_edit.undo)
    
    redo_action = menu.addAction("Redo")
    redo_action.setEnabled(text_edit.document().isRedoAvailable())
    redo_action.triggered.connect(text_edit.redo)
    
    menu.addSeparator()
    
    # Check selection again (may have changed if cursor was moved above)
    has_selection_now = text_edit.textCursor().hasSelection()
    
    cut_action = menu.addAction("Cut")
    cut_action.setEnabled(has_selection_now)
    cut_action.triggered.connect(text_edit.cut)
    
    copy_action = menu.addAction("Copy")
    copy_action.setEnabled(has_selection_now)
    copy_action.triggered.connect(text_edit.copy)
    
    # Paste: explicitly use CLIPBOARD mode (not Selection/PRIMARY) to avoid clearing clipboard
    paste_action = menu.addAction("Paste")
    clipboard = QApplication.clipboard()
    # Check CLIPBOARD mode explicitly (not Selection mode)
    has_clipboard_text = clipboard.text(QClipboard.Mode.Clipboard)
    paste_action.setEnabled(bool(has_clipboard_text))
    
    def handle_paste():
        # Ensure we're using CLIPBOARD mode, not Selection mode
        text_edit.setFocus()
        # Use QTimer to ensure paste happens after menu closes and focus is restored
        QTimer.singleShot(10, lambda: text_edit.paste())
    
    paste_action.triggered.connect(handle_paste)
    
    menu.addSeparator()
    
    select_all_action = menu.addAction("Select All")
    select_all_action.triggered.connect(text_edit.selectAll)
    
    menu.exec(text_edit.mapToGlobal(pos))

text_edit.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
text_edit.customContextMenuRequested.connect(show_context_menu)

layout.addWidget(text_edit)

# Create search bar
search_layout = QHBoxLayout()
search_line_edit = QLineEdit()
search_line_edit.setPlaceholderText("Search...")
search_line_edit.setStyleSheet("background-color: #3E3E3E; color: ivory; padding: 4px;")
case_checkbox = QCheckBox("Case")
case_checkbox.setStyleSheet("color: ivory;")
find_next_button = QPushButton("Find Next")
find_next_button.setStyleSheet("padding: 4px 8px;")
search_layout.addWidget(QLabel("Search:"))
search_layout.addWidget(search_line_edit)
search_layout.addWidget(case_checkbox)
search_layout.addWidget(find_next_button)
layout.addLayout(search_layout)

# Search functionality
def find_next():
    """Find next occurrence of search term."""
    search_term = search_line_edit.text()
    if not search_term:
        return
    
    flags = QTextDocument.FindFlag(0)
    if case_checkbox.isChecked():
        flags |= QTextDocument.FindFlag.FindCaseSensitively
    
    # Start search from current cursor position
    found = text_edit.find(search_term, flags)
    
    if not found:
        # Wrap around: move to start and try again
        cursor = text_edit.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.Start)
        text_edit.setTextCursor(cursor)
        found = text_edit.find(search_term, flags)
        
        if not found:
            QMessageBox.information(window, "Search", f"'{search_term}' not found.")

def show_search():
    """Show/focus search box."""
    search_line_edit.setFocus()
    search_line_edit.selectAll()

find_next_button.clicked.connect(find_next)
search_line_edit.returnPressed.connect(find_next)

# Keyboard shortcuts
QShortcut(QKeySequence("Ctrl+F"), window, show_search)
QShortcut(QKeySequence("F3"), window, find_next)

# Create a QPushButton to import files
import_button = QPushButton("Import File")
import_button.clicked.connect(import_file)
layout.addWidget(import_button)

# Create a QPushButton to trigger text formatting
format_button = QPushButton("Format Text")
layout.addWidget(format_button)

# Connect the button's clicked signal to the function
format_button.clicked.connect(format_text)

# Add simple zoom controls (A+ / A-)
zoom_in_button = QPushButton("A+")
layout.addWidget(zoom_in_button)

zoom_out_button = QPushButton("A-")
layout.addWidget(zoom_out_button)

def _zoom_all(delta: int):
    try:
        cur = text_edit.textCursor()
        sel = QTextCursor(cur)
        sel.select(QTextCursor.SelectionType.Document)
        text_edit.setTextCursor(sel)
        if delta > 0:
            text_edit.zoomIn(delta)
        elif delta < 0:
            text_edit.zoomOut(-delta)
        text_edit.setTextCursor(cur)
    except Exception:
        pass

zoom_in_button.clicked.connect(lambda: _zoom_all(1))
zoom_out_button.clicked.connect(lambda: _zoom_all(-1))

clear_button = QPushButton("Clear Text")
layout.addWidget(clear_button)

# Connect the button's clicked signal to the function
clear_button.clicked.connect(clear_text)

def load_trace_section():
    """Load section(s) of planner_trace_{agent}.txt starting from last n ProgramState lines in trace file."""
    trace_dir = Path(__file__).parent / "logs"
    
    # Create dialog to ask for agent name and number of sections
    dialog = QDialog(window)
    dialog.setWindowTitle("Load Trace Sections")
    dialog_layout = QVBoxLayout()
    
    # Agent name
    agent_label = QLabel("Agent name:")
    dialog_layout.addWidget(agent_label)
    agent_combo = QComboBox()
    agent_combo.setEditable(True)
    # Populate from existing planner_trace_*.txt files
    if trace_dir.exists():
        for f in sorted(trace_dir.glob("planner_trace_*.txt")):
            agent = f.stem.replace("planner_trace_", "")
            if agent and agent not in [agent_combo.itemText(i) for i in range(agent_combo.count())]:
                agent_combo.addItem(agent)
    if agent_combo.count() == 0:
        agent_combo.addItems(["Jill", "Jack"])
    agent_combo.setCurrentIndex(0)
    dialog_layout.addWidget(agent_combo)
    
    # Number of sections
    label = QLabel("How many sections to load?")
    dialog_layout.addWidget(label)
    
    combo = QComboBox()
    combo.addItems(["1", "2", "3", "4", "5"])
    combo.setCurrentIndex(0)  # Default to 1
    dialog_layout.addWidget(combo)
    
    button_layout = QHBoxLayout()
    ok_button = QPushButton("OK")
    cancel_button = QPushButton("Cancel")
    button_layout.addWidget(ok_button)
    button_layout.addWidget(cancel_button)
    dialog_layout.addLayout(button_layout)
    
    dialog.setLayout(dialog_layout)
    
    ok_button.clicked.connect(dialog.accept)
    cancel_button.clicked.connect(dialog.reject)
    
    if dialog.exec() != QDialog.DialogCode.Accepted:
        return
    
    agent_name = agent_combo.currentText().strip() or "Jill"
    num_sections = int(combo.currentText())
    trace_file_path = trace_dir / f"planner_trace_{agent_name}.txt"
    
    # Check if trace file exists
    if not trace_file_path.exists():
        QMessageBox.critical(window, "Error", f"Trace file not found: {trace_file_path}")
        return
    
    # Read trace file and find matching lines
    try:
        with open(trace_file_path, 'r', encoding='utf-8') as f:
            trace_lines = f.readlines()
    except Exception as e:
        QMessageBox.critical(window, "Error", f"Failed to read trace file: {str(e)}")
        return
    
    # Find all lines starting with supported section markers in trace file
    search_prefixes = ["ProgramState(<|im_start|>system", "Planning session:"]
    match_indices = []
    for i in range(len(trace_lines) - 1, -1, -1):
        if any(trace_lines[i].startswith(prefix) for prefix in search_prefixes):
            match_indices.append(i)
    
    if not match_indices:
        QMessageBox.warning(window, "Error", f"No line starting with any of {search_prefixes} found in trace file.")
        return
    
    # Get the last n sections (or all if fewer than n exist)
    if len(match_indices) < num_sections:
        QMessageBox.information(window, "Info", f"Only {len(match_indices)} section(s) found, loading all.")
        num_sections = len(match_indices)
    
    # Get the starting index (first of the last n sections)
    start_index = match_indices[num_sections - 1]
    
    # Load from start_index to end of file
    section_lines = trace_lines[start_index:]
    section_text = ''.join(section_lines)
    
    # Get current window content
    current_text = text_edit.toPlainText()
    
    # If window is empty, clear and load; otherwise append to end
    if not current_text.strip():
        text_edit.clear()
        text_edit.setPlainText(section_text)
        text_edit.setFont(textFont)
    else:
        cursor = text_edit.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        cursor.insertText('\n\n' + section_text)
        text_edit.setTextCursor(cursor)

load_trace_button = QPushButton("Load Trace Section")
layout.addWidget(load_trace_button)

# Connect the button's clicked signal to the function
load_trace_button.clicked.connect(load_trace_section)


_CHAT_TRACE_HEADER_RE = re.compile(
    r'^(?:\[\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}[^\]]*\]|Chat session:)'
)


_FORMAT_PROMPT_STATE_FILE = Path.home() / ".cw_format_prompt.json"


def _scenarios_dir() -> Path:
    """Project-root scenarios/ — sibling of src/."""
    return Path(__file__).resolve().parent.parent / "scenarios"


def _list_worlds_with_traces(scenarios_dir: Path) -> list:
    """Worlds (dirs under scenarios/) that contain at least one agent
    memory dir with a chat_trace.txt. Sorted, mtime-descending so the
    most recently active world floats to the top."""
    if not scenarios_dir.is_dir():
        return []
    candidates = []
    for world_dir in scenarios_dir.iterdir():
        if not world_dir.is_dir():
            continue
        for trace in world_dir.glob("*/memory/chat_trace.txt"):
            candidates.append((trace.stat().st_mtime, world_dir.name))
            break
    candidates.sort(reverse=True)
    seen = set()
    out = []
    for _, name in candidates:
        if name not in seen:
            seen.add(name)
            out.append(name)
    return out


def _list_agents_in_world(scenarios_dir: Path, world: str) -> list:
    """Agent names under scenarios/<world>/ that have a memory/chat_trace.txt."""
    world_dir = scenarios_dir / world
    if not world_dir.is_dir():
        return []
    return sorted(
        d.name for d in world_dir.iterdir()
        if d.is_dir() and (d / "memory" / "chat_trace.txt").exists()
    )


def _load_format_prompt_state() -> dict:
    try:
        if _FORMAT_PROMPT_STATE_FILE.exists():
            return json.loads(_FORMAT_PROMPT_STATE_FILE.read_text())
    except Exception:
        pass
    return {}


def _save_format_prompt_state(state: dict) -> None:
    try:
        _FORMAT_PROMPT_STATE_FILE.write_text(json.dumps(state, indent=2))
    except Exception:
        pass


def load_chat_trace_section():
    """Load section(s) of <world>/<agent>/memory/chat_trace.txt — written
    by the chat ReAct loop, one section per turn. Mirrors load_trace_section's
    UI but reads a different file pattern. Section markers:
      • new format: a line that begins '[YYYY-MM-DD HH:MM:SS UTC] source=…'
      • legacy format: a line that begins 'Chat session:'
    Both forms recognized so old + new entries in the same file both load.
    World and agent selections are session-sticky via ~/.cw_format_prompt.json."""
    scenarios_dir = _scenarios_dir()
    state = _load_format_prompt_state()

    dialog = QDialog(window)
    dialog.setWindowTitle("Load Chat Trace Sections")
    dialog_layout = QVBoxLayout()

    world_label = QLabel("World:")
    dialog_layout.addWidget(world_label)
    world_combo = QComboBox()
    world_combo.setEditable(True)
    worlds = _list_worlds_with_traces(scenarios_dir)
    for w in worlds:
        world_combo.addItem(w)
    last_world = state.get("world")
    if last_world and last_world in worlds:
        world_combo.setCurrentText(last_world)
    elif worlds:
        world_combo.setCurrentIndex(0)
    dialog_layout.addWidget(world_combo)

    agent_label = QLabel("Agent name:")
    dialog_layout.addWidget(agent_label)
    agent_combo = QComboBox()
    agent_combo.setEditable(True)

    def _refresh_agents():
        agent_combo.clear()
        for a in _list_agents_in_world(scenarios_dir, world_combo.currentText().strip()):
            agent_combo.addItem(a)
        if agent_combo.count() == 0:
            agent_combo.addItem("Jill")
        last_agent = state.get("agent")
        if last_agent:
            idx = agent_combo.findText(last_agent)
            if idx >= 0:
                agent_combo.setCurrentIndex(idx)

    _refresh_agents()
    world_combo.currentTextChanged.connect(lambda _: _refresh_agents())
    dialog_layout.addWidget(agent_combo)

    label = QLabel("How many turns (sections) to load?")
    dialog_layout.addWidget(label)
    combo = QComboBox()
    combo.addItems(["1", "2", "3", "4", "5", "10"])
    last_n = str(state.get("num_sections", "1"))
    idx = combo.findText(last_n)
    combo.setCurrentIndex(idx if idx >= 0 else 0)
    dialog_layout.addWidget(combo)

    button_layout = QHBoxLayout()
    ok_button = QPushButton("OK")
    cancel_button = QPushButton("Cancel")
    button_layout.addWidget(ok_button)
    button_layout.addWidget(cancel_button)
    dialog_layout.addLayout(button_layout)

    dialog.setLayout(dialog_layout)
    ok_button.clicked.connect(dialog.accept)
    cancel_button.clicked.connect(dialog.reject)

    if dialog.exec() != QDialog.DialogCode.Accepted:
        return

    world_name = world_combo.currentText().strip()
    agent_name = agent_combo.currentText().strip() or "Jill"
    num_sections = int(combo.currentText())
    _save_format_prompt_state({
        "world": world_name, "agent": agent_name, "num_sections": num_sections,
    })

    trace_file_path = scenarios_dir / world_name / agent_name / "memory" / "chat_trace.txt"

    if not trace_file_path.exists():
        QMessageBox.critical(window, "Error", f"Chat trace file not found: {trace_file_path}")
        return

    try:
        with open(trace_file_path, 'r', encoding='utf-8') as f:
            trace_lines = f.readlines()
    except Exception as e:
        QMessageBox.critical(window, "Error", f"Failed to read trace file: {str(e)}")
        return

    # Each turn's section starts with either a bracketed-timestamp header
    # (new format) or 'Chat session:' (legacy). Walk in reverse so the
    # most-recent turn is first in match_indices.
    match_indices = []
    for i in range(len(trace_lines) - 1, -1, -1):
        if _CHAT_TRACE_HEADER_RE.match(trace_lines[i]):
            match_indices.append(i)

    if not match_indices:
        QMessageBox.warning(window, "Error", "No turn header found in chat trace file.")
        return

    if len(match_indices) < num_sections:
        QMessageBox.information(window, "Info", f"Only {len(match_indices)} turn(s) found, loading all.")
        num_sections = len(match_indices)

    # match_indices is reverse-ordered (last turn first). Pick the start of
    # the last n turns; include the '=' separator line above the header.
    start_index = match_indices[num_sections - 1]
    if start_index > 0 and trace_lines[start_index - 1].startswith('='):
        start_index -= 1

    section_lines = trace_lines[start_index:]
    section_text = ''.join(section_lines)

    current_text = text_edit.toPlainText()
    if not current_text.strip():
        text_edit.clear()
        text_edit.setPlainText(section_text)
        text_edit.setFont(textFont)
    else:
        cursor = text_edit.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        cursor.insertText('\n\n' + section_text)
        text_edit.setTextCursor(cursor)


load_chat_trace_button = QPushButton("Load Chat Trace")
layout.addWidget(load_chat_trace_button)
load_chat_trace_button.clicked.connect(load_chat_trace_section)

# Create submit button for API calls
submit_button = QPushButton("Submit")
layout.addWidget(submit_button)

# Connect submit button
submit_button.clicked.connect(submit_text)

# Set up the window
window.setLayout(layout)
window.setWindowTitle("Text Formatter")
window.show()

# Run the app
app.exec()
