#!/usr/bin/env python3
"""
Interactive CLI for Cognitive Workbench.

Sends /commands as structured JSON to the command channel.
Sends chat as sense_data. Uses prompt_toolkit for non-blocking input.
"""

import json
import re
import sys
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# ANSI color helpers
# ---------------------------------------------------------------------------

class C:
    """ANSI color codes."""
    RESET   = '\033[0m'
    BOLD    = '\033[1m'
    DIM     = '\033[2m'
    CYAN    = '\033[36m'
    GREEN   = '\033[32m'
    YELLOW  = '\033[33m'
    RED     = '\033[31m'
    MAGENTA = '\033[35m'
    BLUE    = '\033[34m'
    WHITE   = '\033[37m'


def _ts():
    return datetime.now().strftime('%H:%M:%S')


def _elapsed_since(iso_str: str) -> str:
    """Render elapsed time since an ISO timestamp as "12s" / "3m 14s" /
    "1h 22m". Returns the raw string on parse failure so /status never
    silently swallows a malformed timestamp."""
    try:
        from datetime import timezone
        t = datetime.fromisoformat(str(iso_str))
        if t.tzinfo is None:
            t = t.replace(tzinfo=timezone.utc)
        now = datetime.now(timezone.utc)
        secs = max(0, int((now - t).total_seconds()))
    except Exception:
        return str(iso_str)
    if secs < 60:
        return f"{secs}s"
    if secs < 3600:
        return f"{secs // 60}m {secs % 60}s"
    return f"{secs // 3600}h {(secs % 3600) // 60}m"


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def _clean_agent_text(text: str) -> str:
    """Strip LLM prompt/response artifacts from agent text."""
    for end_marker in ('</end>', '</end'):
        idx = text.find(end_marker)
        if idx >= 0:
            text = text[:idx]
    for marker in ('\nUSER:', '\nASSISTANT:', '\nTheir move:', '\n## ORIENTATION',
                   '\nRECENT DIALOG:', '\nMessage from ',
                   '\n#Objectives', '\n#Constraints', '\n#Format'):
        idx = text.find(marker)
        if idx > 0:
            text = text[:idx]
    return text.strip()


def _print_agent(character: str, text: str):
    text = _clean_agent_text(text)
    if not text:
        return
    print(f"\n{C.CYAN}{C.BOLD}{character}{C.RESET} {C.DIM}[{_ts()}]{C.RESET}")
    for line in text.split('\n'):
        print(f"  {line}")
    print()


def _print_system(text: str):
    print(f"{C.YELLOW}{text}{C.RESET}")


def _print_error(text: str):
    print(f"{C.RED}{text}{C.RESET}")


def _print_info(text: str):
    print(f"{C.DIM}{text}{C.RESET}")


# ---------------------------------------------------------------------------
# Formatters for structured data
# ---------------------------------------------------------------------------

def _format_concern_row(concern: dict, activation: dict = None) -> str:
    label = concern.get('label') or concern.get('concern_label', '?')
    cid = concern.get('concern_id', '')
    status = concern.get('status', '')
    weight = concern.get('weight', 0)
    act_info = ''
    if activation:
        act_val = activation.get('activation', 0)
        act_info = f'  act={act_val:.2f}'
    return f"  {C.BOLD}{cid}{C.RESET}  {label}  w={weight}  {status}{act_info}"


# ---------------------------------------------------------------------------
# Zenoh helpers
# ---------------------------------------------------------------------------

def _zenoh_get(session, key_expr: str, timeout: float = 3.0) -> Optional[dict]:
    try:
        for reply in session.get(key_expr, timeout=timeout):
            if hasattr(reply, 'ok') and reply.ok is not None:
                return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
    except Exception as e:
        _print_error(f"Query failed ({key_expr}): {e}")
    return None


def _zenoh_put(session, key_expr: str, payload: dict):
    try:
        session.put(key_expr, json.dumps(payload).encode('utf-8'))
    except Exception as e:
        _print_error(f"Put failed ({key_expr}): {e}")


# ---------------------------------------------------------------------------
# Image input helpers (/img <path|url> [caption], /paste [caption])
# ---------------------------------------------------------------------------

_DEFAULT_IMAGE_CAPTION = "Describe this image."


def _detect_clipboard_reader() -> Optional[List[str]]:
    """Best-available command vector that reads PNG bytes from the system
    clipboard to stdout. Wayland preferred when both are present (X11
    clipboard on a Wayland session is XWayland and usually empty for
    images). Returns None if no supported tool is found."""
    import os
    import shutil
    if os.environ.get('WAYLAND_DISPLAY') and shutil.which('wl-paste'):
        return ['wl-paste', '-t', 'image/png']
    if os.environ.get('DISPLAY') and shutil.which('xclip'):
        return ['xclip', '-selection', 'clipboard', '-t', 'image/png', '-o']
    return None


def _bytes_to_data_uri(data: bytes, mime: str) -> str:
    import base64
    return f"data:{mime};base64,{base64.b64encode(data).decode('ascii')}"


def _sniff_image_mime(head: bytes) -> Optional[str]:
    """Magic-byte sniff. Returns the MIME or None if it doesn't look
    like a recognized image format. Used in preference to extension
    sniffing because drag-dropped paths can have any extension."""
    if head.startswith(b'\x89PNG\r\n\x1a\n'):
        return 'image/png'
    if head.startswith(b'\xff\xd8\xff'):
        return 'image/jpeg'
    if head[:6] in (b'GIF87a', b'GIF89a'):
        return 'image/gif'
    if head.startswith(b'RIFF') and head[8:12] == b'WEBP':
        return 'image/webp'
    return None


def _path_to_data_uri(path: str) -> str:
    import os
    if not os.path.isfile(path):
        raise FileNotFoundError(f"not a file: {path}")
    with open(path, 'rb') as f:
        data = f.read()
    mime = _sniff_image_mime(data[:16])
    if mime is None:
        raise ValueError(f"not a recognized image format: {path}")
    return _bytes_to_data_uri(data, mime)


def _clipboard_to_data_uri() -> str:
    import subprocess
    cmd = _detect_clipboard_reader()
    if cmd is None:
        raise RuntimeError(
            "no clipboard reader found — install wl-clipboard (Wayland) "
            "or xclip (X11)")
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=5)
    except Exception as e:
        raise RuntimeError(f"clipboard read failed: {e}")
    if result.returncode != 0:
        stderr = (result.stderr or b'').decode('utf-8', errors='replace').strip()
        raise RuntimeError(
            f"clipboard has no image (run after a screenshot/image copy). "
            f"reader said: {stderr or 'no detail'}")
    data = result.stdout or b''
    mime = _sniff_image_mime(data[:16])
    if mime is None:
        raise RuntimeError("clipboard payload is not a recognized image format")
    return _bytes_to_data_uri(data, mime)


def _parse_img_args(rest: str) -> tuple:
    """Split '/img <path-or-url> [caption...]' into (url_or_path, caption).
    Uses shlex so drag-dropped paths with quoted whitespace survive.
    Caption is everything after the first token, joined with spaces and
    stripped. Empty caption falls back to _DEFAULT_IMAGE_CAPTION at the
    call site."""
    import shlex
    try:
        tokens = shlex.split(rest)
    except ValueError as e:
        raise ValueError(f"could not parse /img args (unbalanced quotes?): {e}")
    if not tokens:
        raise ValueError("usage: /img <path-or-url> [caption]")
    return tokens[0], ' '.join(tokens[1:]).strip()


def _build_image_envelope_inner(text: str, image_url: str) -> str:
    """Inner-content JSON for sense_data — same shape as the text-only
    path plus an optional image field."""
    return json.dumps({
        'source': 'User',
        'text': text,
        'image': {'url': image_url},
    })


# ---------------------------------------------------------------------------
# Command parser
# ---------------------------------------------------------------------------

def _parse_command(line: str) -> Optional[dict]:
    """Parse a /command line into a structured dict for the command channel.

    Returns None if the line is not a recognized command.
    """
    parts = line[1:].split()  # strip leading /
    if not parts:
        return None

    cmd = parts[0].lower()
    args = parts[1:]

    # -- Concerns --
    if cmd == 'concern' and args:
        sub = args[0].lower()
        rest = args[1:]
        if sub in ('close', 'reopen', 'resolve', 'delete', 'activate') and rest:
            d = {'cmd': f'/concern {sub}', 'concern_id': rest[0]}
            if len(rest) > 1:
                d['type'] = rest[1]
            return d
        if sub == 'weight' and len(rest) >= 2:
            d = {'cmd': '/concern weight', 'concern_id': rest[0], 'weight': float(rest[1])}
            if len(rest) > 2:
                d['type'] = rest[2]
            return d
        if sub == 'revisit' and len(rest) >= 2:
            d = {'cmd': '/concern revisit', 'concern_id': rest[0], 'revisit_hours': float(rest[1])}
            if len(rest) > 2:
                d['type'] = rest[2]
            return d
        _print_error("Usage: /concern <close|reopen|resolve|delete|activate|weight|revisit> <id> [args]")
        return None

    # -- System --
    if cmd == 'shutdown':
        return {'cmd': '/shutdown'}

    # -- Notes --
    if cmd == 'note' and args:
        sub = args[0].lower()
        if sub == 'show' and len(args) > 1:
            return {'cmd': '_query', 'query': 'note_show', 'resource_id': args[1]}
        # /note <id> as shorthand for /note show <id>
        return {'cmd': '_query', 'query': 'note_show', 'resource_id': args[0]}

    # -- Recall (direct subagent query, bypasses Jill's ReAct loop) --
    if cmd == 'recall':
        if not args:
            _print_error("Usage: /recall <natural-language query>")
            return None
        return {'cmd': '_query', 'query': 'recall', 'text': ' '.join(args)}

    # -- External-repo binding (sticky across session; sets the geofence
    # for the inspect_external tool). One repo at a time per character.
    if cmd == 'set-external-repo':
        if not args:
            _print_error("Usage: /set-external-repo <absolute-path-to-repo>")
            return None
        return {'cmd': '_query', 'query': 'set_external_repo',
                'path': ' '.join(args)}
    if cmd == 'clear-external-repo':
        return {'cmd': '_query', 'query': 'clear_external_repo'}
    if cmd == 'external-repo':
        return {'cmd': '_query', 'query': 'get_external_repo'}

    # -- Status (is the agent ready for new input?) --
    if cmd == 'status':
        return {'cmd': '_query', 'query': 'status'}

    # -- Read-only queries (handled locally, not sent to command channel) --
    if cmd == 'concerns':
        # Subcommand: /concerns wipe → bulk-delete every non-seed concern.
        # Routed locally through the concern_manage queryable rather than
        # the (currently unsubscribed) command channel.
        if args and args[0].lower() == 'wipe':
            return {'cmd': '_query', 'query': 'concerns_wipe'}
        owner = args[0] if args else None
        return {'cmd': '_query', 'query': 'concerns', 'owner': owner}
    if cmd == 'help':
        return {'cmd': '_query', 'query': 'help'}
    if cmd == 'verbose':
        return {'cmd': '_query', 'query': 'verbose'}

    # -- Navigation --
    if cmd == 'char':
        return {'cmd': '_char', 'name': args[0] if args else ''}
    if cmd == 'ui':
        return {'cmd': '_open', 'url': 'http://localhost:3000'}
    if cmd == 'resources':
        return {'cmd': '_open', 'url': 'http://localhost:3001'}

    _print_error(f"Unknown command: /{cmd}  (type /help for commands)")
    return None


# ---------------------------------------------------------------------------
# Query handlers (read-only, executed locally)
# ---------------------------------------------------------------------------

def _handle_query(session, character: str, data: dict, state: dict):
    query = data.get('query')

    if query == 'concerns':
        owner_filter = data.get('owner')
        result = _zenoh_get(session, f"cognitive/{character}/concerns")
        if not result or not result.get('success'):
            _print_info("No concern data available.")
            return
        activations = result.get('activations', {})
        uc = result.get('user_concerns', [])
        dc = result.get('derived_concerns', [])

        # Filter by owner if specified
        if owner_filter:
            low_filter = owner_filter.lower()
            if low_filter == 'user':
                dc = []  # show only user concerns
            elif low_filter == character.lower():
                uc = []  # show only character's derived concerns
            else:
                # Try matching as a character name — derived concerns belong to the character
                uc = []
                _print_info(f"Showing derived concerns (owner: {owner_filter})")

        if uc:
            _print_info(f"User concerns ({len(uc)}):")
            for c in uc:
                act = activations.get(c.get('concern_id') or c.get('label'))
                print(_format_concern_row(c, act))
        if dc:
            _print_info(f"Derived concerns ({len(dc)}):")
            for c in dc:
                act = activations.get(c.get('concern_id') or c.get('label'))
                print(_format_concern_row(c, act))
        if not uc and not dc:
            _print_info("No concerns.")

    elif query == 'note_show':
        resource_id = data.get('resource_id', '')
        # Normalize: accept "3940" or "Note_3940"
        if resource_id and not resource_id.startswith('Note_'):
            resource_id = f'Note_{resource_id}'
        result = _zenoh_get(session, f"cognitive/{character}/resource/view/{resource_id}")
        if not result or not result.get('success', True) == True:
            # Try as-is if normalization failed
            if result and result.get('error'):
                _print_error(result['error'])
            else:
                _print_error(f"Note '{resource_id}' not found.")
            return
        # Display note content
        name = result.get('name', resource_id)
        rtype = result.get('type', '')
        _print_info(f"{C.BOLD}{name}{C.RESET}  ({rtype}, {resource_id})")
        content = result.get('content', '')
        if isinstance(content, dict):
            content = json.dumps(content, indent=2)
        elif isinstance(content, list):
            content = json.dumps(content, indent=2)
        if content:
            print(content)
        else:
            _print_info("(empty)")

    elif query == 'concerns_wipe':
        _print_info(f"→ wiping non-seed concerns from {character} …")
        payload = json.dumps({'action': 'wipe_non_seed'}).encode('utf-8')
        result = None
        try:
            for reply in session.get(
                f"cognitive/{character}/control/concern_manage",
                payload=payload, timeout=10.0,
            ):
                if hasattr(reply, 'ok') and reply.ok is not None:
                    result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    break
        except Exception as e:
            _print_error(f"Wipe failed: {e}")
            return
        if not result:
            _print_error("(no response — chat loop may not be reachable)")
            return
        if not result.get('success'):
            _print_error(result.get('error', 'wipe failed'))
            return
        n_deleted = result.get('deleted_count', 0)
        n_kept = result.get('kept_seed_count', 0)
        errors = result.get('errors') or []
        _print_info(f"Deleted {n_deleted} non-seed concern(s); kept {n_kept} seed concern(s).")
        if errors:
            _print_error(f"{len(errors)} error(s) during wipe:")
            for e in errors:
                _print_error(f"  {e}")

    elif query == 'recall':
        q_text = data.get('text', '').strip()
        if not q_text:
            _print_error("Usage: /recall <query>")
            return
        _print_info(f"→ recall: {q_text} (subagent runs up to 10 iters; may take a minute)")
        payload = json.dumps({'query': q_text}).encode('utf-8')
        result = None
        try:
            for reply in session.get(
                f"cognitive/{character}/recall",
                payload=payload, timeout=120.0,
            ):
                if hasattr(reply, 'ok') and reply.ok is not None:
                    result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    break
        except Exception as e:
            _print_error(f"Query failed: {e}")
            return
        if not result:
            _print_error("(no response — subagent may have timed out, or chat loop isn't reachable)")
            return
        if not result.get('success'):
            _print_error(result.get('error', 'remember query failed'))
            return
        print(result.get('answer', '(no answer)'))
        trace_dir = result.get('trace_dir')
        if trace_dir:
            _print_info(f"(full subagent trace under {trace_dir})")

    elif query in ('set_external_repo', 'clear_external_repo', 'get_external_repo'):
        action_map = {
            'set_external_repo': 'set',
            'clear_external_repo': 'clear',
            'get_external_repo': 'get',
        }
        action = action_map[query]
        body: dict = {'action': action}
        if action == 'set':
            body['path'] = data.get('path', '')
        payload = json.dumps(body).encode('utf-8')
        result = None
        try:
            for reply in session.get(
                f"cognitive/{character}/control/external_repo",
                payload=payload, timeout=5.0,
            ):
                if hasattr(reply, 'ok') and reply.ok is not None:
                    result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    break
        except Exception as e:
            _print_error(f"external_repo query failed: {e}")
            return
        if not result:
            _print_error("(no response — chat loop may not be reachable)")
            return
        if not result.get('success'):
            _print_error(result.get('error', 'external_repo command failed'))
            return
        if action == 'set':
            _print_info(f"external_repo bound: {result.get('path')}")
        elif action == 'clear':
            if result.get('was_bound'):
                _print_info(f"external_repo cleared (was: {result.get('path')})")
            else:
                _print_info("external_repo was not bound; nothing to clear")
        else:  # get
            p = result.get('path')
            if p:
                _print_info(f"external_repo: {p}")
            else:
                _print_info("external_repo: (none)")

    elif query == 'status':
        result = _zenoh_get(session, f"cognitive/{character}/status")
        if not result:
            _print_error("(no response — chat loop may not be reachable)")
            return
        if not result.get('success'):
            _print_error(result.get('error', 'status query failed'))
            return
        ready = result.get('ready', False)
        action = result.get('action', '?')
        if ready:
            _print_info(f"{C.BOLD}{character} is ready{C.RESET}  ({action})")
        else:
            _print_info(f"{C.BOLD}{character} is busy{C.RESET}: {action}")
            ct = result.get('current_turn') or {}
            started = ct.get('started_at')
            if started:
                _print_info(f"  started: {_elapsed_since(started)} ago")
            preview = ct.get('text_preview')
            if preview:
                _print_info(f"  input: {preview!r}")
        if result.get('post_turn_busy') and ready:
            _print_info(f"  (post-turn reflection still running)")
        backlog = int(result.get('inbox_backlog', 0) or 0)
        if backlog:
            _print_info(f"  inbox backlog: {backlog}")
        last = result.get('last_reply_at')
        if last:
            _print_info(f"  last reply: {_elapsed_since(last)} ago")
        backend = result.get('backend') or {}
        if backend:
            _print_info(
                f"  backend: {backend.get('server','?')}@"
                f"{backend.get('base_url','?')} model={backend.get('model') or '(default)'}"
            )
        log_dir = result.get('log_dir')
        if log_dir:
            _print_info(f"  logs: {log_dir}/")

    elif query == 'verbose':
        state['verbose'] = not state.get('verbose', False)
        _print_system(f"Verbose mode: {'on' if state['verbose'] else 'off'}")

    elif query == 'help':
        _print_help()


def _print_help():
    print(f"""{C.BOLD}Cognitive Workbench CLI{C.RESET}

{C.BOLD}Input:{C.RESET}  Plain text is sent to the active character as chat.
        Slash commands (below) are dispatched directly.

{C.BOLD}Concerns:{C.RESET}
  /concerns [owner]              List concerns (owner: User, <character>, or all)
  /concerns wipe                 Hard-delete every non-seed concern (seeds preserved)
  /concern close <id>            Close a user concern
  /concern reopen <id>           Reopen a user concern
  /concern resolve <id>          Resolve/satisfy a derived concern
  /concern delete <id>           Delete a concern
  /concern activate <id>         Reactivate a derived concern
  /concern weight <id> <0-1>     Set concern weight
  /concern revisit <id> <hours>  Set revisit hours

{C.BOLD}Notes:{C.RESET}
  /note <id>                     Show note content (e.g. /note 3940)

{C.BOLD}Memory:{C.RESET}
  /recall <query>                Active-recall subagent query (reads agent's memory dir)

{C.BOLD}Image input:{C.RESET}
  /img <path|url> [caption]      Send an image (local file or URL) with optional caption
  /paste [caption]               Send the image currently in the system clipboard

{C.BOLD}External code:{C.RESET}
  /set-external-repo <path>      Bind a project repo for the inspect_external tool
  /clear-external-repo           Unbind the external repo
  /external-repo                 Show the currently bound external repo (if any)

{C.BOLD}Navigation:{C.RESET}
  @<agent> /<command>            Send command to a specific agent
  /char <name>                   Switch active character
  /ui                            Open web UI in browser
  /resources                     Open resource browser in browser
  /verbose                       Toggle verbose mode
  /help                          Show this help

{C.BOLD}State:{C.RESET}
  /status                        Show whether the agent is ready for new input

{C.BOLD}System:{C.RESET}
  /shutdown                      Save and shutdown
  Ctrl+D                         Exit CLI""")


# ---------------------------------------------------------------------------
# Main CLI loop
# ---------------------------------------------------------------------------

def run_cli(zenoh_session, character_names: List[str], shutdown_event: threading.Event):
    """Run the interactive CLI.

    Args:
        zenoh_session: Active Zenoh session.
        character_names: List of character names.
        shutdown_event: Threading event to signal shutdown.
    """
    if not character_names:
        _print_error("No characters available.")
        return

    # Save terminal state before prompt_toolkit touches it
    _saved_term_attrs = None
    try:
        import termios
        _saved_term_attrs = termios.tcgetattr(sys.stdin.fileno())
    except Exception:
        pass

    # Use prompt_toolkit for non-blocking, safe terminal input
    from prompt_toolkit import PromptSession
    from prompt_toolkit.history import FileHistory
    from prompt_toolkit.formatted_text import HTML
    from prompt_toolkit.patch_stdout import patch_stdout
    from prompt_toolkit.key_binding import KeyBindings
    from prompt_toolkit.keys import Keys

    active_character = character_names[0]

    # Publishers
    sense_publisher = zenoh_session.declare_publisher(
        f"cognitive/{active_character}/sense_data"
    )
    command_publisher = zenoh_session.declare_publisher(
        f"cognitive/{active_character}/command"
    )

    # Shared state
    state: Dict[str, Any] = {
        'verbose': False,
        'agent_state': {},
        'active_character': active_character,
        'awaiting_ask': False,  # True while agent is blocking on an ask
    }
    state_lock = threading.Lock()

    # Subscribe to agent actions (say + announcement)
    def _action_callback(sample):
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            action_type = data.get('type', '')
            character = str(sample.key_expr).split('/')[1]

            if action_type == 'announcement':
                if 'character_name' in data and 'character_config' in data:
                    return
                text = data.get('text', '')
                if text:
                    _print_system(f"[{character}] {text}")
                return

            if action_type == 'say':
                source = data.get('source', '')
                if source == 'User':
                    return
                text = data.get('text', '')
                if text:
                    # Dedup: skip if this text was recently displayed (auto-delivery
                    # may duplicate an in-plan say — suppress the repeat).
                    text_key = text[:500].strip()
                    with state_lock:
                        recent = state.setdefault('_recent_say', [])
                        if text_key in recent:
                            return
                        recent.append(text_key)
                        # Keep only last 5 to bound memory
                        if len(recent) > 5:
                            del recent[0]
                    _print_agent(character, text)
                return

            if action_type == 'ask':
                text = data.get('text', '')
                if text:
                    with state_lock:
                        state['awaiting_ask'] = True
                    print(f"\n{C.YELLOW}{C.BOLD}{character} asks:{C.RESET} {C.DIM}[{_ts()}]{C.RESET}")
                    for line in text.split('\n'):
                        print(f"  {line}")
                    print(f"  {C.DIM}(type your reply and press Enter){C.RESET}")
                    print()
                return

            with state_lock:
                if state.get('verbose'):
                    text = data.get('text', data.get('llm_response', ''))
                    if text:
                        summary = str(text)[:120]
                        _print_info(f"  [{character}] {action_type}: {summary}")
        except Exception:
            pass

    action_sub = zenoh_session.declare_subscriber("cognitive/*/action", _action_callback)

    def _execution_state_callback(sample):
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            with state_lock:
                prev = state['agent_state']
                prev_goal = prev.get('executing_goal_id')
                prev_dialog = prev.get('active_dialog', False)
                state['agent_state'].update(data)
                curr_goal = data.get('executing_goal_id')
                curr_dialog = data.get('active_dialog', False)
                curr_paused = data.get('paused', False)
                # Detect goal start: clear say dedup so replayed plans can re-display
                if not prev_goal and curr_goal:
                    state.pop('_recent_say', None)
                # Detect goal completion: was running, now not
                if prev_goal and not curr_goal:
                    _print_system(f"[{_ts()}] Ready.")
                # Detect dialog completion: was in dialog, now idle
                # (only when paused — avoids false Ready when a goal is about to start)
                elif prev_dialog and not curr_dialog and not curr_goal and curr_paused:
                    _print_system(f"[{_ts()}] Ready.")
        except Exception:
            pass

    exec_state_sub = zenoh_session.declare_subscriber(
        "cognitive/*/execution_state", _execution_state_callback
    )

    # Banner
    print()
    print(f"{C.BOLD}Cognitive Workbench CLI{C.RESET}")
    print(f"Character: {C.CYAN}{active_character}{C.RESET}  |  Type /help for commands  |  /img <path|url> [caption], /paste [caption] for image  |  Alt+Enter for newline  |  Ctrl+D to exit")
    print()

    import os
    history_path = os.path.expanduser('~/.cognitive_workbench_history')

    # Multiline prompt: Enter submits, Alt+Enter inserts a newline. Default
    # single-line mode would bind Enter to submit and offer no way to enter
    # a newline at all — no Alt+Enter, no Esc+Enter, nothing.
    kb = KeyBindings()

    @kb.add(Keys.Enter)
    def _(event):
        event.current_buffer.validate_and_handle()

    @kb.add(Keys.Escape, Keys.Enter)
    def _(event):
        event.current_buffer.insert_text('\n')

    session = PromptSession(
        history=FileHistory(str(history_path)),
        multiline=True,
        key_bindings=kb,
    )
    last_ctrl_c = 0.0

    try:
      with patch_stdout(raw=True):
        while not shutdown_event.is_set():
            try:
                prompt_text = HTML(f'<aaa fg="ansigreen">{active_character}</aaa>&gt; ')
                line = session.prompt(prompt_text)
                last_ctrl_c = 0.0
            except EOFError:
                break
            except KeyboardInterrupt:
                now = time.monotonic()
                if now - last_ctrl_c < 1.0:
                    break  # double Ctrl+C exits
                last_ctrl_c = now
                print()
                _print_info("(Ctrl+C again to exit, Ctrl+D to exit, or keep typing)")
                continue

            line = line.strip()
            if not line:
                continue

            # /img and /paste are CLI-local: they build a sense_data
            # envelope (like a normal text turn) rather than a command.
            # Intercept here so the generic command parser doesn't
            # see them. Active-character-only — @agent retargeting
            # isn't supported in v1.
            if line.startswith('/img ') or line == '/img' or line.startswith('/paste'):
                head, _, rest = line.partition(' ')
                head = head.strip()
                rest = rest.strip()
                try:
                    if head == '/img':
                        if not rest:
                            _print_error("usage: /img <path-or-url> [caption]")
                            continue
                        target, caption = _parse_img_args(rest)
                        if target.startswith(('http://', 'https://')):
                            image_url = target
                        else:
                            image_url = _path_to_data_uri(target)
                    elif head == '/paste':
                        # Whole rest (after /paste) is the caption.
                        caption = rest
                        image_url = _clipboard_to_data_uri()
                    else:
                        _print_error(f"unknown image command: {head}")
                        continue
                except (FileNotFoundError, ValueError, RuntimeError) as e:
                    _print_error(str(e))
                    continue

                if not caption:
                    caption = _DEFAULT_IMAGE_CAPTION

                sense_data = {
                    'timestamp': datetime.now().isoformat(),
                    'sequence_id': 0,
                    'mode': 'text',
                    'content': _build_image_envelope_inner(caption, image_url),
                }
                sense_publisher.put(json.dumps(sense_data))
                # Don't echo the data URI; it can be megabytes.
                tag = 'url' if image_url.startswith(('http://', 'https://')) else 'data-uri'
                _print_info(f"→ sent to {active_character} (image: {tag}, caption: {caption!r})")
                continue

            # Slash-command (with optional @agent targeting)
            if line.startswith('/'):
                # Check for @agent prefix: /@agent /command or @agent /command
                target_agent = None
                cmd_line = line
                if line.startswith('/@') or line.startswith('@'):
                    # Extract @agent and the rest
                    stripped = line.lstrip('/')
                    at_match = re.match(r'@([\w-]+)\s+(/.+)', stripped)
                    if at_match:
                        target_name = at_match.group(1)
                        cmd_line = at_match.group(2)
                        # Resolve agent name (case-insensitive, prefix match)
                        for cn in character_names:
                            if cn.lower() == target_name.lower() or cn.lower().startswith(target_name.lower()):
                                target_agent = cn
                                break
                        if not target_agent:
                            _print_error(f"Unknown agent: @{target_name}. Available: {', '.join(character_names)}")
                            continue
                    else:
                        # Malformed @agent prefix — treat as normal command
                        pass

                parsed = _parse_command(cmd_line)
                if parsed is None:
                    continue

                cmd = parsed.get('cmd', '')

                # Local queries
                if cmd == '_query':
                    _handle_query(zenoh_session, active_character, parsed, state)
                    continue

                # Character switch
                if cmd == '_char':
                    new_char = parsed.get('name', '').strip()
                    if not new_char:
                        _print_info(f"Available characters: {', '.join(character_names)}")
                        continue
                    matched = None
                    for cn in character_names:
                        if cn.lower() == new_char.lower():
                            matched = cn
                            break
                    if not matched:
                        _print_error(f"Unknown character: {new_char}. Available: {', '.join(character_names)}")
                        continue
                    active_character = matched
                    state['active_character'] = matched
                    sense_publisher = zenoh_session.declare_publisher(
                        f"cognitive/{active_character}/sense_data"
                    )
                    command_publisher = zenoh_session.declare_publisher(
                        f"cognitive/{active_character}/command"
                    )
                    _print_system(f"Switched to character: {active_character}")
                    continue

                # Open URL
                if cmd == '_open':
                    import webbrowser
                    webbrowser.open(parsed['url'])
                    _print_system(f"Opened {parsed['url']}")
                    continue

                # Send command to the command channel (or targeted agent)
                parsed['source'] = 'User'
                if target_agent:
                    zenoh_session.put(
                        f"cognitive/{target_agent}/command",
                        json.dumps(parsed).encode('utf-8'),
                    )
                    _print_info(f"→ @{target_agent} {parsed['cmd']}")
                else:
                    command_publisher.put(json.dumps(parsed))
                    _print_info(f"→ {parsed['cmd']}")

                # Handle shutdown locally too
                if cmd == '/shutdown':
                    time.sleep(2)
                    _zenoh_put(zenoh_session, "cognitive/launcher/shutdown",
                               {'timestamp': datetime.now().isoformat(), 'source': 'cli', 'mode': 'save_and_shutdown'})
                    _print_system("Shutdown requested.")
                continue

            # If agent is waiting for an ask response, send directly — no interpret
            with state_lock:
                is_ask_reply = state.get('awaiting_ask')
            if is_ask_reply:
                with state_lock:
                    state['awaiting_ask'] = False
                sense_data = {
                    'timestamp': datetime.now().isoformat(),
                    'sequence_id': 0,
                    'mode': 'text',
                    'content': json.dumps({
                        'source': 'User',
                        'text': line
                    })
                }
                sense_publisher.put(json.dumps(sense_data))
                _print_info(f"→ reply sent to {active_character}")
                continue

            # Plain text — send directly to agent (chat handler decides
            # whether to respond conversationally, escalate to a goal, or
            # dispatch a system command).
            sense_data = {
                'timestamp': datetime.now().isoformat(),
                'sequence_id': 0,
                'mode': 'text',
                'content': json.dumps({
                    'source': 'User',
                    'text': line
                })
            }
            sense_publisher.put(json.dumps(sense_data))
            _print_info(f"→ sent to {active_character}")

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal state — prompt_toolkit raw mode can leave
        # echo disabled and line buffering off if cleanup is interrupted.
        if _saved_term_attrs is not None:
            try:
                import termios
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, _saved_term_attrs)
            except Exception:
                pass
        # Belt-and-suspenders: stty sane in case the above didn't work
        try:
            import subprocess
            subprocess.run(['stty', 'sane'], stdin=sys.stdin, check=False)
        except Exception:
            pass
        print()
        _print_system("Shutting down...")
        shutdown_event.set()
