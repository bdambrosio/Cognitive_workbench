from ast import literal_eval
from PyQt5.QtWidgets import (QApplication, QTextEdit, QVBoxLayout, QWidget, 
                            QPushButton, QDialog, QProgressDialog, QMessageBox,
                            QFileDialog)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QUrl, QByteArray
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply
from PyQt5.QtGui import QFont, QTextCursor
from pathlib import Path
import json
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
    request.setHeader(QNetworkRequest.ContentTypeHeader, "application/json")
    
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
        request.setHeader(QNetworkRequest.ContentTypeHeader, "application/json")
        
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
                        cursor.movePosition(QTextCursor.End)
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
layout.addWidget(text_edit)

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
        sel.select(QTextCursor.Document)
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
app.exec_()
