"""JSON repair helpers for tolerant parsing of LLM output."""

import json
import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)


def unparseable_action_note(truncated: bool) -> str:
    """Retry instruction to feed back after repair_json_string() returned
    None, steered by WHY the parse failed.

    A token-limit truncation and genuinely malformed output need opposite
    advice: the first means "you wrote too much", the second means "you
    wrote the wrong shape". Telling a truncated emitter only that its
    output was "unparseable" invites it to re-emit the same oversized
    payload verbatim — observed 2026-08-16, seven identical retries
    pasting 300 lines of source into a `respond` text field.

    Callers get `truncated` from the backend's last_finish_reason
    ('length' / 'max_tokens').
    """
    if truncated:
        return ("Your previous emission was cut off by the token limit before the "
                "JSON action closed — it was too long. Emit ONE COMPLETE, COMPACT "
                "JSON action now: a SHORT thought (a single clause), the `tool`, and "
                "concise args. Keep EVERY field brief — do not dump analysis, source "
                "code or step-by-step work into `thought` or into any argument value. "
                "Summarize or cite line ranges instead of quoting, and work the "
                "problem in small steps across iterations.")
    return ("Previous output was not valid JSON. Do NOT apologize — emit ONE JSON "
            "action now: a single object with a short `thought` and a `tool`.")


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

    # Strip markdown bold/italic emphasis markers (`**`, `__`) that some
    # instruction-tuned models leak into structured output, e.g.
    # `{"a": 1, **"b": 2,**}`. String-aware: only stripped outside of
    # double-quoted JSON strings, so legitimate `**` inside string values
    # is preserved.
    if "**" in response or "__" in response:
        out = []
        i = 0
        in_string = False
        while i < len(response):
            c = response[i]
            if c == '"' and (i == 0 or response[i-1] != '\\'):
                in_string = not in_string
                out.append(c)
                i += 1
                continue
            if not in_string and (
                response[i:i+2] == "**" or response[i:i+2] == "__"
            ):
                i += 2
                continue
            out.append(c)
            i += 1
        response = ''.join(out)

    # Pre-process: escape unescaped backslashes inside string literals.
    # LLMs frequently emit LaTeX (`$v = \sqrt{T/\mu}$`) or other
    # backslash-heavy content inside `text` fields. Only `\" \\ \/ \b
    # \f \n \r \t \uXXXX` are valid JSON escapes — anything else (e.g.
    # `\s`, `\m`, `\l`) makes json.loads reject the whole object. This
    # pass walks the string character by character, and inside any
    # JSON string literal, doubles any backslash that isn't followed
    # by a valid escape character. Outside string literals, leaves
    # backslashes alone. Valid `\\` sequences are detected and
    # advanced over intact, so this is idempotent on already-valid
    # JSON.
    if '\\' in response:
        _VALID_ESCAPES = set('"\\/bfnrtu')
        out = []
        i = 0
        in_str = False
        while i < len(response):
            c = response[i]
            if not in_str:
                out.append(c)
                if c == '"':
                    in_str = True
                i += 1
                continue
            # Inside a string literal.
            if c == '\\':
                nxt = response[i+1] if i + 1 < len(response) else ''
                if nxt in _VALID_ESCAPES:
                    # Valid escape — keep both characters as-is. The
                    # `\"` case correctly leaves in_str = True because
                    # we never see the `"` as a standalone char.
                    out.append(c)
                    out.append(nxt)
                    i += 2
                    continue
                # Invalid (or trailing) backslash — double it.
                out.append('\\')
                out.append('\\')
                i += 1
                continue
            if c == '"':
                in_str = False
            out.append(c)
            i += 1
        response = ''.join(out)

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
                except (ValueError, SyntaxError, TypeError) as e:
                    logger.debug(f"repair_json_string: arithmetic eval failed for {expr!r}: {e}")

            # Return original if evaluation fails
            return match.group(0)

        return re.sub(pattern, replace, text)

    # Try evaluating arithmetic expressions before parsing
    response = eval_arithmetic_in_json(response)

    # Try direct parse first
    try:
        return json.loads(response)
    except json.JSONDecodeError as e:
        logger.debug(f"repair_json_string: direct parse failed ({e}); attempting repair")

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
        except json.JSONDecodeError as e:
            logger.debug(f"repair_json_string: brace-truncation parse failed ({e})")

    # Repair attempt 4: Add missing closing braces
    if brace_count > 0:
        response = response + ('}' * brace_count)
        try:
            return json.loads(response)
        except json.JSONDecodeError as e:
            logger.debug(f"repair_json_string: missing-brace fill parse failed ({e})")

    # Repair attempt 5: Strip trailing extra braces
    while response.endswith('}}'):
        trimmed = response[:-1]
        try:
            return json.loads(trimmed)
        except json.JSONDecodeError as e:
            logger.debug(f"repair_json_string: trailing-brace strip parse failed ({e})")
            response = trimmed

    # Repair attempt 6: Quote bareword keys ({key: 1} → {"key": 1}).
    # Matches a bareword that follows `{` or `,` (with optional whitespace)
    # and is followed by `:`. This is a regex-only pass — it doesn't track
    # string boundaries, so a value-string containing `, foo:` could be
    # mis-rewritten. Acceptable as a last-resort heuristic; LLM output
    # rarely contains such strings, and earlier attempts already covered
    # the common shapes.
    import re
    quoted = re.sub(
        r'([{,]\s*)([A-Za-z_][A-Za-z0-9_]*)(\s*:)',
        r'\1"\2"\3',
        response,
    )
    # Also strip trailing commas before } or ], which often co-occur
    # with unquoted keys in the same drift class.
    quoted = re.sub(r',(\s*[}\]])', r'\1', quoted)
    if quoted != response:
        try:
            return json.loads(quoted)
        except json.JSONDecodeError as e:
            logger.debug(f"repair_json_string: bareword-key quoting parse failed ({e})")

    logger.debug(
        f"repair_json_string: all repair attempts exhausted; returning None. "
        f"Input head: {json_str[:200]!r}")
    return None
