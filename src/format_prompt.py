from ast import literal_eval
from PyQt5.QtWidgets import (QApplication, QTextEdit, QVBoxLayout, QWidget, 
                            QPushButton, QDialog, QProgressDialog, QMessageBox,
                            QFileDialog)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
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
    
    # First: Apply existing \n literal replacement
    formatted_text = raw_text.replace("\\t", "\t").replace("\\n", "\n")
    
    # Second: Apply Python structure formatting to improve readability
    formatted_text = format_python_structure(formatted_text)
    
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
