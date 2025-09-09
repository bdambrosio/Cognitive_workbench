from ast import literal_eval
from PyQt5.QtWidgets import (QApplication, QTextEdit, QVBoxLayout, QWidget, 
                            QPushButton, QDialog, QProgressDialog, QMessageBox,
                            QFileDialog)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QTextCursor
from pathlib import Path
import json, requests
import sys
import os
import re
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

# Set up the window
window.setLayout(layout)
window.setWindowTitle("Text Formatter")
window.show()

# Run the app
app.exec_()
