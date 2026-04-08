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

def _format_task_row(t: dict) -> str:
    name = t.get('_note_name', '?')
    status = t.get('status', '?')
    phase = t.get('phase', '')
    intention = (t.get('intention', '') or '')[:80]
    is_active = ' *' if t.get('_is_active') else ''
    sc = {
        'proposed': C.YELLOW, 'in_progress': C.GREEN, 'active': C.GREEN,
        'completed': C.DIM, 'abandoned': C.RED, 'interrupted': C.MAGENTA,
    }.get(status, '')
    return f"  {C.BOLD}{name}{C.RESET}{is_active}  {sc}{status}{C.RESET}/{phase}  {intention}"


def _format_goal_row(g: dict) -> str:
    gid = g.get('goal_id', '?')
    name = g.get('name', '')[:40]
    status = g.get('status', '?')
    mode = g.get('schedule_mode', '?')
    exec_mode = g.get('execution_mode', '')
    running = g.get('is_running', False)
    sc = {'ready': C.GREEN, 'running': C.CYAN, 'paused': C.YELLOW, 'completed': C.DIM}.get(status, '')
    run_mark = f' {C.CYAN}[running]{C.RESET}' if running else ''
    exec_tag = f' {exec_mode}' if exec_mode else ''
    return f"  {C.BOLD}{gid}{C.RESET}  {sc}{status}{C.RESET}  {mode}{exec_tag}{run_mark}  {name}"


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


def _format_task_detail(t: dict) -> str:
    lines = [f"{C.BOLD}Task: {t.get('_note_name', '?')}{C.RESET}"]
    for key in ('task_wip_id', 'status', 'phase', 'intention', 'proposal_reason',
                'linked_concern_id', 'created', 'updated'):
        val = t.get(key)
        if val:
            lines.append(f"  {key}: {val}")
    milestones = t.get('milestones_completed', [])
    if milestones:
        lines.append(f"  milestones_completed: {len(milestones)}")
        for m in milestones[-3:]:
            lines.append(f"    - {m}")
    current = t.get('current_milestone')
    if current:
        lines.append(f"  current_milestone: {current}")
    return '\n'.join(lines)


def _format_goal_detail(g: dict) -> str:
    lines = [f"{C.BOLD}Goal: {g.get('goal_id', '?')}{C.RESET}"]
    for key in ('name', 'status', 'schedule_mode', 'execution_mode', 'is_running',
                'goal_text', 'last_result', 'primary_product', 'created', 'updated',
                'run_at', 'last_run_date'):
        val = g.get(key)
        if val is not None and val != '':
            display = str(val)[:200] if key in ('goal_text', 'last_result', 'primary_product') else val
            lines.append(f"  {key}: {display}")
    return '\n'.join(lines)


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

    # -- Goals --
    if cmd == 'goal' and args:
        sub = args[0].lower()
        rest = args[1:]

        if sub == 'add' and rest:
            return {'cmd': '/goal add', 'goal_text': ' '.join(rest)}
        if sub == 'run' and rest:
            d = {'cmd': '/goal run', 'goal_id': rest[0]}
            if len(rest) > 1 and rest[1] in ('replan', 'replay'):
                d['execution_mode'] = rest[1]
            return d
        if sub == 'terminate' and rest:
            return {'cmd': '/goal terminate', 'goal_id': rest[0]}
        if sub == 'rename' and rest:
            if len(rest) >= 2:
                return {'cmd': '/goal rename', 'goal_id': rest[0], 'name': ' '.join(rest[1:])}
            return {'cmd': '_interactive', 'type': 'goal_rename', 'goal_id': rest[0]}
        if sub == 'edit' and rest:
            if len(rest) >= 2:
                return {'cmd': '/goal edit', 'goal_id': rest[0], 'goal_text': ' '.join(rest[1:])}
            return {'cmd': '_interactive', 'type': 'goal_edit', 'goal_id': rest[0]}
        if sub in ('delete', 'remove') and rest:
            return {'cmd': '/goal delete', 'goal_id': rest[0]}
        if sub == 'mode' and len(rest) >= 2:
            d = {'cmd': '/goal mode', 'goal_id': rest[0], 'schedule_mode': rest[1]}
            if len(rest) > 2:
                d['run_at'] = rest[2]
            return d
        if sub == 'exec' and len(rest) >= 2:
            return {'cmd': '/goal exec', 'goal_id': rest[0], 'execution_mode': rest[1]}
        if sub == 'plan':
            # /goal plan <id>, /goal plan show <id>, /goal plan edit <id>, /goal plan approve <id>
            if not rest:
                _print_error("Usage: /goal plan [show|edit|approve] <goal_id>")
                return None
            if rest[0] in ('show', 'edit', 'load', 'approve', 'review', 'commit'):
                if len(rest) < 2:
                    _print_error(f"Usage: /goal plan {rest[0]} <goal_id>")
                    return None
                return {'cmd': f'/goal plan {rest[0]}', 'goal_id': rest[1]}
            # /goal plan <id>  →  shorthand for show
            return {'cmd': '/goal plan', 'goal_id': rest[0]}
        if sub == 'cache' and rest:
            # /goal cache clear <id> or /goal cache <id>
            if rest[0] == 'clear' and len(rest) > 1:
                return {'cmd': '/goal cache clear', 'goal_id': rest[1]}
            return {'cmd': '/goal cache clear', 'goal_id': rest[0]}
        if sub == 'show' and rest:
            return {'cmd': '_query', 'query': 'goal_show', 'goal_id': rest[0]}
        _print_error(f"Usage: /goal <add|run|terminate|rename|edit|delete|mode|exec|cache|plan|show> ...")
        return None

    # -- Tasks --
    if cmd == 'task' and args:
        sub = args[0].lower()
        rest = args[1:]

        if sub == 'add' and rest:
            return {'cmd': '/task add', 'intention': ' '.join(rest)}
        if sub == 'propose':
            return {'cmd': '/task propose'}
        if sub == 'approve' and rest:
            d = {'cmd': '/task approve', 'note_name': rest[0]}
            if len(rest) > 1:
                d['intention'] = ' '.join(rest[1:])
            return d
        if sub == 'edit' and rest:
            if len(rest) >= 2:
                return {'cmd': '/task edit', 'note_name': rest[0], 'intention': ' '.join(rest[1:])}
            return {'cmd': '_interactive', 'type': 'task_edit', 'note_name': rest[0]}
        if sub == 'abandon' and rest:
            d = {'cmd': '/task abandon', 'note_name': rest[0]}
            if len(rest) > 1:
                d['reason'] = ' '.join(rest[1:])
            return d
        if sub == 'delete' and rest:
            return {'cmd': '/task delete', 'note_name': rest[0]}
        if sub == 'interrupt' and rest:
            return {'cmd': '/task interrupt', 'note_name': rest[0]}
        if sub == 'run' and rest:
            return {'cmd': '/task run', 'note_name': rest[0]}
        if sub == 'cooldown' and rest:
            d = {'cmd': '/task cooldown', 'note_name': rest[0]}
            d['cooldown_seconds'] = int(rest[1]) if len(rest) > 1 else 300
            return d
        if sub == 'show' and rest:
            return {'cmd': '_query', 'query': 'task_show', 'note_name': rest[0]}
        _print_error(f"Usage: /task <add|propose|approve|edit|abandon|delete|interrupt|run|cooldown|show> ...")
        return None

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
    if cmd == 'stop':
        return {'cmd': '/stop'}
    if cmd == 'continuous':
        return {'cmd': '/continuous'}
    if cmd == 'llm':
        return {'cmd': '/llm'}
    if cmd == 'delay' and args:
        return {'cmd': '/delay', 'delay': float(args[0])}
    if cmd == 'scheduler':
        if not args:
            return {'cmd': '/scheduler'}
        sub = args[0].lower()
        if sub in ('on', 'off'):
            return {'cmd': '/scheduler', 'action': sub}
        if sub == 'interval' and len(args) > 1:
            return {'cmd': '/scheduler', 'action': 'interval', 'interval': float(args[1])}
        return {'cmd': '/scheduler'}
    if cmd == 'clear' and args:
        return {'cmd': '/clear', 'target': args[0]}
    if cmd == 'save':
        return {'cmd': '/save'}
    if cmd == 'shutdown':
        return {'cmd': '/shutdown'}
    if cmd == 'bye':
        return {'cmd': '/bye'}
    if cmd == 'done':
        return {'cmd': '/done'}
    if cmd == 'next':
        return {'cmd': '/next'}
    if cmd == 'action' and args:
        return {'cmd': '/action', 'json_text': ' '.join(args)}

    # -- Notes --
    if cmd == 'note' and args:
        sub = args[0].lower()
        if sub == 'show' and len(args) > 1:
            return {'cmd': '_query', 'query': 'note_show', 'resource_id': args[1]}
        # /note <id> as shorthand for /note show <id>
        return {'cmd': '_query', 'query': 'note_show', 'resource_id': args[0]}

    # -- Interpret (experimental LLM command parsing) --
    if cmd == 'interpret':
        if not args:
            _print_error("Usage: /interpret <text to interpret>")
            return None
        return {'cmd': '_interpret', 'text': ' '.join(args)}

    # -- Read-only queries (handled locally, not sent to command channel) --
    if cmd == 'tasks':
        return {'cmd': '_query', 'query': 'tasks'}
    if cmd == 'goals':
        return {'cmd': '_query', 'query': 'goals'}
    if cmd == 'concerns':
        owner = args[0] if args else None
        return {'cmd': '_query', 'query': 'concerns', 'owner': owner}
    if cmd == 'triage':
        return {'cmd': '_query', 'query': 'triage'}
    if cmd == 'status':
        return {'cmd': '_query', 'query': 'status'}
    if cmd == 'ooda':
        return {'cmd': '_query', 'query': 'ooda'}
    if cmd == 'help':
        return {'cmd': '_query', 'query': 'help'}
    if cmd == 'verbose':
        return {'cmd': '_query', 'query': 'verbose'}

    # -- Navigation --
    if cmd == 'char':
        return {'cmd': '_char', 'name': args[0] if args else ''}
    if cmd == 'ui':
        return {'cmd': '_open', 'url': 'http://localhost:3000'}
    if cmd in ('tasks-ui', 'tasksui'):
        return {'cmd': '_open', 'url': 'http://localhost:3002'}
    if cmd == 'resources':
        return {'cmd': '_open', 'url': 'http://localhost:3001'}

    _print_error(f"Unknown command: /{cmd}  (type /help for commands)")
    return None


# ---------------------------------------------------------------------------
# Query handlers (read-only, executed locally)
# ---------------------------------------------------------------------------

def _handle_query(session, character: str, data: dict, state: dict):
    query = data.get('query')

    if query == 'tasks':
        result = _zenoh_get(session, f"cognitive/{character}/task_wips")
        if not result or not result.get('success'):
            _print_info("No task data available.")
            return
        tasks = result.get('tasks', [])
        if not tasks:
            _print_info("No tasks.")
            return
        _print_info(f"Tasks for {character} ({len(tasks)}):")
        for t in tasks:
            print(_format_task_row(t))

    elif query == 'task_show':
        name = data.get('note_name', '')
        if not name.startswith('_task_wip_'):
            name = f'_task_wip_{name}'
        result = _zenoh_get(session, f"cognitive/{character}/task_wips")
        if not result:
            return
        for t in result.get('tasks', []):
            if t.get('_note_name') == name:
                print(_format_task_detail(t))
                return
        _print_error(f"Task '{name}' not found.")

    elif query == 'goals':
        result = _zenoh_get(session, f"cognitive/{character}/scheduled_goals")
        if not result or not result.get('success'):
            _print_info("No goal data available.")
            return
        goals = result.get('goals', [])
        if not goals:
            _print_info("No goals.")
            return
        _print_info(f"Goals for {character} ({len(goals)}):")
        for g in goals:
            print(_format_goal_row(g))

    elif query == 'goal_show':
        gid = data.get('goal_id', '')
        result = _zenoh_get(session, f"cognitive/{character}/scheduled_goals")
        if not result:
            return
        for g in result.get('goals', []):
            if g.get('goal_id') == gid:
                print(_format_goal_detail(g))
                return
        _print_error(f"Goal '{gid}' not found.")

    elif query == 'concerns':
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

    elif query == 'triage':
        result = _zenoh_get(session, f"cognitive/{character}/triage_status")
        if not result or not result.get('success'):
            _print_info("No triage data available.")
            return
        _print_info("Triage status:")
        for key, val in result.items():
            if key == 'success':
                continue
            print(f"  {key}: {val}")

    elif query == 'status':
        _print_info(f"Active character: {C.BOLD}{character}{C.RESET}")
        _print_info(f"Verbose: {state.get('verbose', False)}")
        agent_state = state.get('agent_state', {})
        if agent_state:
            mode = agent_state.get('mode', '?')
            paused = agent_state.get('paused', False)
            continuous = agent_state.get('continuous_mode', False)
            llm = agent_state.get('llm_mode', '?')
            dialog = agent_state.get('active_dialog', False)
            parts = [f"mode={mode}"]
            if paused:
                parts.append(f"{C.YELLOW}paused{C.RESET}")
            if continuous:
                parts.append(f"{C.GREEN}continuous{C.RESET}")
            parts.append(f"llm={llm}")
            if dialog:
                parts.append(f"{C.CYAN}in dialog{C.RESET}")
            print(f"  {', '.join(parts)}")
            scheduler = agent_state.get('goal_scheduler', {})
            if scheduler:
                running = scheduler.get('executing_goal_id')
                if running:
                    print(f"  {C.CYAN}Running goal: {running}{C.RESET}")
                else:
                    print(f"  No goal running")
        else:
            print(f"  {C.DIM}(no state received yet){C.RESET}")

    elif query == 'ooda':
        result = _zenoh_get(session, f"cognitive/{character}/ooda_feed")
        if not result or not result.get('success'):
            _print_info("No OODA feed data available.")
            return
        events = result.get('events', [])
        if not events:
            _print_info("No recent OODA events.")
            return
        _print_info(f"Recent OODA events ({len(events)}):")
        for ev in events[-10:]:
            ts = ev.get('timestamp', '')
            if ts:
                ts = ts.split('T')[1][:8] if 'T' in ts else ts
            phase = ev.get('phase', '')
            summary = ev.get('summary', ev.get('text', ''))[:100]
            print(f"  {C.DIM}{ts}{C.RESET} {C.BOLD}{phase}{C.RESET} {summary}")

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

    elif query == 'verbose':
        state['verbose'] = not state.get('verbose', False)
        _print_system(f"Verbose mode: {'on' if state['verbose'] else 'off'}")

    elif query == 'help':
        _print_help()


# ---------------------------------------------------------------------------
# /interpret — experimental LLM command parsing
# ---------------------------------------------------------------------------

# Command vocabulary for the LLM prompt (static — matches the registry)
_COMMAND_VOCABULARY = """\
/goal add <goal_text>          Create and run a new goal
/goal run <goal_id> [replan|replay]  Execute goal (override mode)
/goal terminate <goal_id>      Stop a running goal
/goal rename <goal_id> <name>  Rename a goal
/goal edit <goal_id> <text>    Update goal text
/goal delete <goal_id>         Delete a goal
/goal mode <goal_id> <mode>    Set schedule: manual|auto|recurring|daily
/goal cache clear <goal_id>    Clear cached plan
/task add <intention>           Create a new task
/task propose                   Propose task from conversation context
/task approve <note_name>       Approve a proposed task
/task edit <note_name> <text>   Edit task intention
/task abandon <note_name>       Abandon a task
/task delete <note_name>        Delete task
/task interrupt <note_name>     Pause active task
/task run <note_name>           Run task now (clear cooldown)
/task cooldown <name> <secs>    Set cooldown seconds
/concern close <id>             Close a user concern
/concern reopen <id>            Reopen a user concern
/concern resolve <id>           Satisfy a derived concern
/concern delete <id>            Delete a concern
/concern activate <id>          Reactivate a derived concern
/stop                           Halt current execution
/continuous                     Toggle continuous mode
/llm                            Toggle LLM (primary/alt)
/delay <seconds>                Set turn delay
/scheduler on|off               Enable/disable goal scheduler
/scheduler interval <secs>      Set scheduler check interval
/clear <target>                 Clear: world-model|map|transients|persistents
/save                           Save all data
/shutdown                       Save and shutdown
/bye                            End conversation
/note <id>                      Show note content"""


def _interpret_text(session, character: str, text: str) -> Optional[Dict]:
    """Use LLM to interpret free-form text as /commands or chat.

    Returns the parsed interpretation dict, or None on failure.
    """

    # Gather current state for context
    goals_data = _zenoh_get(session, f"cognitive/{character}/concerns")  # for concern IDs
    goals_result = _zenoh_get(session, f"cognitive/{character}/scheduled_goals")
    tasks_result = _zenoh_get(session, f"cognitive/{character}/task_wips")

    # Build goals context
    goals_ctx = "None"
    if goals_result and goals_result.get('success'):
        lines = []
        for g in goals_result.get('goals', []):
            gid = g.get('goal_id', '?')
            name = g.get('name', '')[:60]
            status = g.get('status', '?')
            running = ' [RUNNING]' if g.get('is_running') else ''
            lines.append(f"  {gid} [{status}]{running} \"{name}\"")
        if lines:
            goals_ctx = '\n'.join(lines)

    # Build tasks context
    tasks_ctx = "None"
    if tasks_result and tasks_result.get('success'):
        lines = []
        for t in tasks_result.get('tasks', []):
            name = t.get('_note_name', '?')
            status = t.get('status', '?')
            lifecycle = t.get('lifecycle', '')
            intention = (t.get('intention', '') or '')[:60]
            active = ' [ACTIVE]' if t.get('_is_active') else ''
            lines.append(f"  {name} [{status}/{lifecycle}]{active} \"{intention}\"")
        if lines:
            tasks_ctx = '\n'.join(lines)

    # Build concerns context
    concerns_ctx = "None"
    if goals_data and goals_data.get('success'):
        lines = []
        for c in goals_data.get('user_concerns', []):
            cid = c.get('concern_id', '')
            label = c.get('label', '')
            status = c.get('status', '')
            lines.append(f"  {cid} [user, {status}] \"{label}\"")
        for c in goals_data.get('derived_concerns', []):
            cid = c.get('concern_id', '')
            label = c.get('concern_label', '')
            status = c.get('status', '')
            lines.append(f"  {cid} [derived, {status}] \"{label}\"")
        if lines:
            concerns_ctx = '\n'.join(lines)

    prompt = f"""\
You are a command interpreter for a cognitive agent system. Given user text,
determine if it contains one or more system commands, or if it is conversational
chat directed at the agent.

RULES:
- Only emit commands from the vocabulary below. Do not invent commands.
- Resolve references like "the weather goal" or "goal 1" to actual goal_ids from
  the current state. If you cannot resolve a reference, treat the text as chat.
- Default to chat when uncertain. False command detection is worse than missed detection.
- A single input may map to multiple commands (e.g., "stop and save" → two commands).
- Requests like "tell me about X" or "what do you think about X" are chat, not commands.
- Requests like "check email" or "run the health check" are /goal run if a matching goal exists,
  or /goal add if the user seems to want something new done.
- "make it so" or "do it" with no prior conversational context suggesting a specific action is chat.

COMMAND VOCABULARY:
{_COMMAND_VOCABULARY}

CURRENT GOALS:
{goals_ctx}

CURRENT TASKS:
{tasks_ctx}

CURRENT CONCERNS:
{concerns_ctx}

USER TEXT: "{text}"

Respond with a JSON object. Examples:
{{"interpretation": "command", "commands": [{{"cmd": "/goal run", "goal_id": "goal_2"}}]}}
{{"interpretation": "command", "commands": [{{"cmd": "/stop"}}, {{"cmd": "/save"}}]}}
{{"interpretation": "chat", "text": "How are you feeling today?"}}
{{"interpretation": "command", "commands": [{{"cmd": "/goal add", "goal_text": "research quantum computing breakthroughs"}}]}}

JSON response:"""

    # Call LLM via Zenoh queryable
    messages = [
        {'role': 'user', 'content': prompt},
    ]
    payload = json.dumps({
        'messages': messages,
        'max_tokens': 500,
        'temperature': 0.1,
        'is_json': True,
    })

    _print_info(f"Interpreting: \"{text}\"")
    try:
        result = None
        for reply in session.get(f"cognitive/{character}/llm/generate",
                                  payload=payload.encode('utf-8'), timeout=30.0):
            if hasattr(reply, 'ok') and reply.ok is not None:
                result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                break

        if not result or not result.get('success'):
            _print_error(f"LLM call failed: {result.get('error', 'no response') if result else 'timeout'}")
            return None

        llm_text = result.get('text', '')
        # LLM may return pre-parsed dict (when is_json=True) or raw string
        if isinstance(llm_text, dict):
            interpretation = llm_text
        else:
            llm_text = str(llm_text).strip()
            # Handle cases where LLM wraps in markdown code blocks
            if llm_text.startswith('```'):
                llm_text = llm_text.split('\n', 1)[-1].rsplit('```', 1)[0].strip()
            try:
                interpretation = json.loads(llm_text)
            except json.JSONDecodeError:
                _print_error(f"LLM returned invalid JSON: {llm_text[:200]}")
                return None

        return interpretation

    except Exception as e:
        _print_error(f"Interpret failed: {e}")
        return None


def _format_commands(commands: List[Dict]) -> str:
    """Format a list of interpreted commands for display.

    Shows commands as they would actually be executed (positional args),
    not as keyword=value pairs.
    """
    parts = []
    for cmd_data in commands:
        cmd = cmd_data.get('cmd', '?')
        args_parts = []
        for k, v in cmd_data.items():
            if k != 'cmd':
                args_parts.append(str(v))
        args_str = ' '.join(args_parts)
        parts.append(f"  {C.BOLD}{cmd}{C.RESET} {args_str}")
    return '\n'.join(parts)


def _handle_interpret(session, character: str, text: str):
    """Use LLM to interpret free-form text and display results (for /interpret command)."""
    interpretation = _interpret_text(session, character, text)
    if interpretation is None:
        return

    interp_type = interpretation.get('interpretation', 'unknown')

    if interp_type == 'chat':
        orig = interpretation.get('text', text)
        _print_info(f"  → {C.CYAN}chat{C.RESET}: \"{orig}\"")
    elif interp_type == 'command':
        commands = interpretation.get('commands', [])
        if not commands:
            _print_info(f"  → {C.CYAN}chat{C.RESET} (no commands extracted)")
            return
        _print_info(f"  → {C.GREEN}{len(commands)} command(s):{C.RESET}")
        print(_format_commands(commands))
    else:
        _print_info(f"  → {C.YELLOW}unknown interpretation{C.RESET}")


def _print_help():
    print(f"""{C.BOLD}Cognitive Workbench CLI{C.RESET}

{C.BOLD}Input:{C.RESET}  Type naturally — input is interpreted as commands or chat automatically.
        Commands are shown for confirmation (Enter to run, anything else to cancel).

{C.BOLD}Tasks:{C.RESET}
  /tasks                         List all tasks
  /task show <name>              Show task detail
  /task add <intention>          Create a new task
  /task propose                  Propose task from conversation
  /task approve <name> [text]    Approve proposed task
  /task edit <name> <text>       Edit task intention
  /task abandon <name> [reason]  Abandon task
  /task delete <name>            Delete task
  /task interrupt <name>         Pause task
  /task run <name>               Run task now (clear cooldown)
  /task cooldown <name> [secs]   Set cooldown (default 300s)

{C.BOLD}Goals:{C.RESET}
  /goals                         List all goals
  /goal show <id>                Show goal detail
  /goal add <text>               Create and run new goal
  /goal run <id> [replan|replay]  Execute goal now (override mode)
  /goal terminate <id>           Stop a running goal
  /goal rename <id> <name>       Rename goal
  /goal edit <id> <text>         Update goal text
  /goal delete <id>              Delete goal
  /goal mode <id> <mode> [time]  Set schedule mode (manual|auto|recurring|daily)
  /goal exec <id> <mode>         Set execution mode (replan|replay)
  /goal cache clear <id>         Clear cached plan
  /goal plan <id>                Show cached plan steps
  /goal plan edit <id>           Write plan to file for editing
  /goal plan load <id>           Load edited plan from file
  /goal plan approve <id>        Approve plan (set replay mode)
  /goal plan review <id>         Generate review bundle for analysis
  /goal plan commit <id>         Load + approve + inject learnings + run

{C.BOLD}Concerns:{C.RESET}
  /concerns [owner]              List concerns (owner: User, <character>, or all)
  /concern close <id>            Close a user concern
  /concern reopen <id>           Reopen a user concern
  /concern resolve <id>          Resolve/satisfy a derived concern
  /concern delete <id>           Delete a concern
  /concern activate <id>         Reactivate a derived concern
  /concern weight <id> <0-1>     Set concern weight
  /concern revisit <id> <hours>  Set revisit hours

{C.BOLD}System:{C.RESET}
  /stop                          Halt current execution
  /continuous                    Toggle continuous execution
  /llm                           Toggle LLM mode (primary/alt)
  /delay <seconds>               Set turn delay
  /scheduler [on|off|interval N] Goal scheduler config
  /clear <target>                Clear (world-model|map|transients|persistents)
  /save                          Save all data
  /shutdown                      Save and shutdown
  /bye                           End conversation
  /action <json>                 Execute a direct JSON action

{C.BOLD}Notes:{C.RESET}
  /note <id>                     Show note content (e.g. /note 3940)

{C.BOLD}Info:{C.RESET}
  /status                        Show system status
  /triage                        Show triage pipeline status
  /ooda                          Show recent OODA events

{C.BOLD}Navigation:{C.RESET}
  @<agent> /<command>             Send command to a specific agent (e.g. @offline /stop)
  /char <name>                   Switch active character
  /ui                            Open web UI in browser
  /tasks-ui                      Open task manager in browser
  /resources                     Open resource browser in browser
  /verbose                       Toggle verbose mode
  /help                          Show this help

{C.BOLD}Experimental:{C.RESET}
  /interpret <text>              LLM-parse text into /commands (display only, no execute)

{C.BOLD}Note:{C.RESET} Plain text is auto-interpreted. If commands are detected, confirm with
Enter to execute or type anything to cancel. Chat is sent directly to the agent.""")


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
                prev_goal = prev.get('goal_scheduler', {}).get('executing_goal_id')
                prev_dialog = prev.get('active_dialog', False)
                state['agent_state'].update(data)
                curr_goal = (data.get('goal_scheduler') or {}).get('executing_goal_id')
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
    print(f"Character: {C.CYAN}{active_character}{C.RESET}  |  Type /help for commands  |  Ctrl+D to exit")
    print()

    import os
    history_path = os.path.expanduser('~/.cognitive_workbench_history')
    session = PromptSession(history=FileHistory(str(history_path)))
    last_ctrl_c = 0.0

    try:
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

                # LLM-based command interpretation (experimental)
                if cmd == '_interpret':
                    _handle_interpret(zenoh_session, active_character, parsed['text'])
                    continue

                # Interactive commands (fetch current value, prompt to edit)
                if cmd == '_interactive':
                    itype = parsed.get('type')
                    try:
                        if itype == 'goal_edit':
                            gid = parsed['goal_id']
                            result = _zenoh_get(zenoh_session, f"cognitive/{active_character}/scheduled_goals")
                            current_text = ''
                            if result:
                                for g in result.get('goals', []):
                                    if g.get('goal_id') == gid:
                                        current_text = g.get('goal_text', '')
                                        break
                            new_text = session.prompt(
                                HTML(f'<aaa fg="ansicyan">goal text for {gid}:</aaa> '),
                                default=current_text,
                            )
                            if new_text.strip() and new_text.strip() != current_text:
                                command_publisher.put(json.dumps({
                                    'cmd': '/goal edit', 'goal_id': gid,
                                    'goal_text': new_text.strip(), 'source': 'User',
                                }))
                                _print_info(f"→ /goal edit {gid}")
                            else:
                                _print_info("(no change)")

                        elif itype == 'goal_rename':
                            gid = parsed['goal_id']
                            result = _zenoh_get(zenoh_session, f"cognitive/{active_character}/scheduled_goals")
                            current_name = ''
                            if result:
                                for g in result.get('goals', []):
                                    if g.get('goal_id') == gid:
                                        current_name = g.get('name', '')
                                        break
                            new_name = session.prompt(
                                HTML(f'<aaa fg="ansicyan">name for {gid}:</aaa> '),
                                default=current_name,
                            )
                            if new_name.strip() and new_name.strip() != current_name:
                                command_publisher.put(json.dumps({
                                    'cmd': '/goal rename', 'goal_id': gid,
                                    'name': new_name.strip(), 'source': 'User',
                                }))
                                _print_info(f"→ /goal rename {gid}")
                            else:
                                _print_info("(no change)")

                        elif itype == 'task_edit':
                            note_name = parsed['note_name']
                            if not note_name.startswith('_task_wip_'):
                                note_name = f'_task_wip_{note_name}'
                            result = _zenoh_get(zenoh_session, f"cognitive/{active_character}/task_wips")
                            current_intention = ''
                            if result:
                                for t in result.get('tasks', []):
                                    if t.get('_note_name') == note_name:
                                        current_intention = t.get('intention', '')
                                        break
                            new_intention = session.prompt(
                                HTML(f'<aaa fg="ansicyan">intention for {note_name}:</aaa> '),
                                default=current_intention,
                            )
                            if new_intention.strip() and new_intention.strip() != current_intention:
                                command_publisher.put(json.dumps({
                                    'cmd': '/task edit', 'note_name': note_name,
                                    'intention': new_intention.strip(), 'source': 'User',
                                }))
                                _print_info(f"→ /task edit {note_name}")
                            else:
                                _print_info("(no change)")
                    except (EOFError, KeyboardInterrupt):
                        _print_info("(cancelled)")
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
