"""JSON repair helpers for tolerant parsing of LLM output."""

import json
from typing import Dict, Optional


def repair_json_string(json_str: str) -> Optional[Dict]:
    """
    Attempt to repair malformed JSON from LLM output.

    Handles common LLM JSON errors:
    - Trailing extra braces
    - Missing closing braces
    - Code fences
    - Newlines in wrong places
    - Arithmetic expressions in numeric fields (e.g., "1.0236+3.14159")

    Args:
        json_str: Potentially malformed JSON string

    Returns:
        Parsed dict if successful, None if repair fails
    """
    if not json_str:
        return None

    response = json_str.strip()

    # Remove markdown code fences if present
    response = response.replace("```json", "").replace("```", "").strip()

    # Pre-process: Evaluate arithmetic expressions in numeric fields before parsing
    # This handles cases like {"yaw": 1.0236+3.14159} which are invalid JSON
    def eval_arithmetic_in_json(text):
        """Find and evaluate arithmetic expressions in JSON-like strings."""
        import re
        # Pattern: "key": value where value might be an arithmetic expression
        # Match unquoted numeric expressions after colons
        # Pattern matches: "key": 1.23+4.56 or "key": "1.23+4.56"
        pattern = r'"([^"]+)":\s*("?)([0-9.+\-*/().\s]+)("?)'

        def replace(match):
            key = match.group(1)
            open_quote = match.group(2)
            expr = match.group(3).strip()
            close_quote = match.group(4)

            # Only process if it contains arithmetic operators and isn't a variable reference
            if any(op in expr for op in ['+', '-', '*', '/']) and not expr.startswith('$'):
                try:
                    import ast
                    result = ast.literal_eval(expr)
                    if isinstance(result, (int, float)):
                        return f'"{key}": {result}'
                except (ValueError, SyntaxError, TypeError):
                    pass

            # Return original if evaluation fails
            return match.group(0)

        return re.sub(pattern, replace, text)

    # Try evaluating arithmetic expressions before parsing
    response = eval_arithmetic_in_json(response)

    # Try direct parse first
    try:
        return json.loads(response)
    except json.JSONDecodeError:
        pass

    # Repair attempt 1: Extract JSON if not at start
    if not response.startswith('{') and '{' in response:
        start = response.find('{')
        end = response.rfind('}')
        if start >= 0 and end >= start:
            response = response[start:end+1]

    # Repair attempt 2: Remove newlines outside string values
    in_string = False
    result = []
    i = 0
    while i < len(response):
        if response[i] == '"' and (i == 0 or response[i-1] != '\\'):
            in_string = not in_string
        if not in_string and response[i] == '\n':
            i += 1
            continue
        result.append(response[i])
        i += 1
    response = ''.join(result)

    # Repair attempt 3: Find first complete JSON object by brace counting
    brace_count = 0
    json_end = 0
    in_string = False
    for i, char in enumerate(response):
        if char == '"' and (i == 0 or response[i-1] != '\\'):
            in_string = not in_string
        if not in_string:
            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0:
                    json_end = i + 1
                    break

    if json_end > 0:
        response = response[:json_end]
        try:
            return json.loads(response)
        except json.JSONDecodeError:
            pass

    # Repair attempt 4: Add missing closing braces
    if brace_count > 0:
        response = response + ('}' * brace_count)
        try:
            return json.loads(response)
        except json.JSONDecodeError:
            pass

    # Repair attempt 5: Strip trailing extra braces
    while response.endswith('}}'):
        trimmed = response[:-1]
        try:
            return json.loads(trimmed)
        except json.JSONDecodeError:
            response = trimmed

    return None
