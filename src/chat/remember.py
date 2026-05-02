"""Active-recall subagent — a thin, persona-less ReAct loop that answers
queries by reading files in Jill's per-world memory directory.

Used by Jill's chat ReAct loop via the `remember` tool. From Jill's vantage,
a `remember` call is one step: she emits a query string, gets a synthesized
answer back. The subagent's internal multi-iteration reasoning is opaque to
her trace, but written to a per-call trace file under subagent_traces/ for
debugging.

Primitives are intentionally minimal and read-only: list, read (with optional
line range), grep, respond. All file paths resolve within memory_dir; path
traversal is rejected.
"""

from __future__ import annotations

import json
import logging
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

_MAX_ITERS = 10
_MAX_READ_CHARS = 10_000
_MAX_GREP_HITS = 50


def _build_system_prompt(memory_dir: Path) -> str:
    """Static system prompt — describes the file types in the memory dir,
    their content/role, expected size, and the right read strategy for each.
    Stable across all calls in a session, so caches well on the anthropic
    route (cache_control already attached in _ChatBackend)."""
    return (
        "You are a memory-search subagent. You answer questions about an "
        "agent's memory by reading files in a fixed directory. You have "
        "no persona, no goals beyond the current query, and no memory of "
        "your own past calls — each invocation is independent.\n"
        "\n"
        f"Memory directory: {memory_dir}\n"
        "\n"
        "## File types — content, size, and read strategy\n"
        "\n"
        "- `conversation.txt` — APPEND-ONLY free text. Each entry is a "
        "timestamped header (`[ts] entity -> Jill (...)` for user input, "
        "`[ts] entity <- Jill (...)` for Jill's reply) followed by the "
        "message text. CONTENT: verbatim dialogue — what the user said, "
        "what Jill replied. SIZE: small (~1KB per turn, readable whole for "
        "thousands of turns). **Read whole** for any user-content query "
        "(\"what did the user say / mention / ask about\", \"what do I "
        "need to do\", \"did we discuss X\"). This is the primary substrate "
        "for content queries.\n"
        "\n"
        "- `companion_state_<entity>.txt` — CURRENT-VALUE ONLY, overwritten "
        "on each update. CONTENT: rolling fair-witness profile (CURRENT "
        "CHAPTER, STATE OF MIND, WHAT MATTERS TO THEM, OBSERVED DEFAULTS, "
        "ON THEIR MIND). SIZE: small (~3KB), always readable whole. **Read "
        "whole** for queries about how the user thinks, current preferences, "
        "or what's currently on their mind. Use `list` for mtime if you "
        "need \"when was this last updated.\"\n"
        "\n"
        "- `discourse_state_<entity>.txt` — CURRENT-VALUE ONLY, overwritten "
        "on each update. CONTENT: ACTIVE COMMITMENTS, CURRENT AGREEMENTS, "
        "KEY DECISIONS MADE, unresolved issues. SIZE: small (~5-10KB), "
        "always readable whole. **Read whole** for queries about what's "
        "been agreed, decided, or left unresolved between user and Jill.\n"
        "\n"
        "- `chat_trace.txt` — APPEND-ONLY free text. Each ReAct turn writes "
        "a section with header `[<timestamp>] source=... iters=... "
        "exit=...` followed by SYSTEM:, USER:, ASSISTANT: blocks. CONTENT: "
        "byte-stream of LLM I/O — Jill's intermediate reasoning (thoughts, "
        "tool calls, observations) per iteration. The SYSTEM block repeats "
        "every turn and dominates the file. SIZE: large (~5-15KB per turn, "
        "grows quickly). **Never read whole — too large.** Use `grep` with "
        "literal strings, then `read` narrow line ranges around hits. Most "
        "user-content queries should NOT touch this file; `conversation.txt` "
        "is the right source. This file is for queries about Jill's prior "
        "internal reasoning (\"what tool did Jill use to answer Y\", \"what "
        "did Jill think before saying Z\").\n"
        "\n"
        "## Tools (one JSON object per emission)\n"
        "\n"
        '1. {"thought": "<one sentence>", "tool": "list"} — list files in '
        "memory dir with size and mtime.\n"
        '2. {"thought": "...", "tool": "read", "file": "<name>", '
        '"start_line": <int?>, "end_line": <int?>} — read a file. Omit '
        "start_line/end_line to read the whole file (capped). Lines are "
        "1-indexed; output format is `lineno|content`.\n"
        '3. {"thought": "...", "tool": "grep", "pattern": "<regex>", '
        '"file": "<name>?"} — grep a single file or, if file is omitted, '
        "across all files in the dir. Output format is "
        "`file:lineno|content` per hit.\n"
        '4. {"thought": "...", "tool": "respond", "text": "<answer>"} — '
        "final answer to the query, exits the loop.\n"
        "\n"
        "## Discipline\n"
        "\n"
        "- **No keyword unions for concept search.** Patterns like "
        "`chore|cleaning|wash|tidy|...` are keyword matching, which misses "
        "synonyms, items vs. activities, and indirect references. You are "
        "the semantic filter — read the relevant whole file and reason over "
        "its content. Reserve `grep` for LITERAL STRINGS you expect to "
        "appear verbatim (a name, a ticker, an explicit phrase the user "
        "used).\n"
        "- **Pick the right file for the query type before any tool call:**\n"
        "  * Retrieval / list queries (\"what did the user say about X\", "
        "\"what do I need to do\", \"list all Y\") → start with "
        "`conversation.txt` whole. Add companion/discourse state if the "
        "query touches preferences or commitments.\n"
        "  * State / preference queries (\"how does the user think\", "
        "\"what's on their mind\", \"what have we agreed on\") → start "
        "with `companion_state_*.txt` and/or `discourse_state_*.txt`, "
        "whole.\n"
        "  * Reasoning-introspection queries (\"why did Jill say X\", "
        "\"what was Jill thinking when Y\") → grep `chat_trace.txt` for "
        "the specific term, then read a narrow line range around hits.\n"
        "- **Confidence requires completeness for retrieval.** \"I found "
        "one match\" is not enough for list-style queries. Read the whole "
        "relevant file before responding.\n"
        "- **If the files do not contain what's asked, say so plainly** — "
        "do not speculate beyond the evidence.\n"
        "- Quote specific lines when answering, with line numbers or "
        "timestamps to anchor the claim.\n"
        "- Output ONLY one JSON object. No prose, no markdown fences."
    )


def _safe_resolve(memory_dir: Path, name: str) -> Optional[Path]:
    """Resolve a filename within memory_dir; reject path traversal."""
    if not name:
        return None
    try:
        candidate = (memory_dir / name).resolve()
        memory_resolved = memory_dir.resolve()
        if candidate != memory_resolved and not str(candidate).startswith(
                str(memory_resolved) + '/'):
            return None
        return candidate
    except Exception:
        return None


def _tool_list(memory_dir: Path) -> str:
    if not memory_dir.is_dir():
        return f"(memory dir does not exist: {memory_dir})"
    rows = []
    for p in sorted(memory_dir.iterdir()):
        if p.is_dir():
            continue
        try:
            stat = p.stat()
            mtime = datetime.fromtimestamp(
                stat.st_mtime, tz=timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
            rows.append(f"{p.name}\t{stat.st_size} bytes\t{mtime}")
        except Exception:
            continue
    return "\n".join(rows) if rows else "(empty)"


def _tool_read(memory_dir: Path, name: str,
               start_line: Optional[int], end_line: Optional[int]) -> str:
    path = _safe_resolve(memory_dir, name)
    if path is None:
        return f"(read: invalid or out-of-scope file: {name!r})"
    if not path.is_file():
        return f"(read: file not found: {name})"
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            lines = f.readlines()
    except Exception as e:
        return f"(read error: {e})"
    n = len(lines)
    s = max(1, int(start_line)) if start_line else 1
    e = min(n, int(end_line)) if end_line else n
    if s > n:
        return f"(read: start_line {s} > file length {n})"
    selected = lines[s - 1:e]
    out = ''.join(f"{s + i}|{ln}" for i, ln in enumerate(selected))
    if len(out) > _MAX_READ_CHARS:
        return (out[:_MAX_READ_CHARS] +
                f"\n…(truncated; {len(out)} chars total, capped at "
                f"{_MAX_READ_CHARS}; use line ranges to slice)")
    if not out:
        return f"(read: empty range {s}..{e}, file has {n} lines)"
    return out


def _tool_grep(memory_dir: Path, pattern: str,
               file: Optional[str]) -> str:
    if not pattern:
        return "(grep: empty pattern)"
    try:
        rgx = re.compile(pattern)
    except re.error as e:
        return f"(grep: invalid regex: {e})"

    targets: List[Path] = []
    if file:
        path = _safe_resolve(memory_dir, file)
        if path is None or not path.is_file():
            return f"(grep: file not found or out-of-scope: {file!r})"
        targets = [path]
    else:
        if not memory_dir.is_dir():
            return "(grep: memory dir does not exist)"
        targets = sorted(p for p in memory_dir.iterdir() if p.is_file())

    hits: List[str] = []
    for path in targets:
        try:
            with open(path, 'r', encoding='utf-8', errors='replace') as f:
                for i, ln in enumerate(f, start=1):
                    if rgx.search(ln):
                        hits.append(f"{path.name}:{i}|{ln.rstrip()}")
                        if len(hits) >= _MAX_GREP_HITS:
                            break
        except Exception as e:
            hits.append(f"{path.name}:0|(read error: {e})")
        if len(hits) >= _MAX_GREP_HITS:
            break
    if not hits:
        return "(no matches)"
    out = "\n".join(hits)
    if len(hits) >= _MAX_GREP_HITS:
        out += f"\n(grep capped at {_MAX_GREP_HITS} hits)"
    return out


def _parse_action(raw: str) -> Optional[Dict[str, Any]]:
    """Parse a JSON action emission. Returns None if unparseable."""
    s = (raw or '').strip()
    if not s:
        return None
    if s.startswith('```'):
        s = re.sub(r'^```[a-zA-Z]*\n', '', s)
        s = re.sub(r'\n```\s*$', '', s)
    try:
        return json.loads(s)
    except json.JSONDecodeError:
        m = re.search(r'\{.*\}', s, re.DOTALL)
        if m:
            try:
                return json.loads(m.group(0))
            except json.JSONDecodeError:
                return None
        return None


def _write_trace(trace_dir: Path, query: str,
                 iters: List[Dict[str, Any]], answer: str,
                 exit_reason: str) -> Optional[Path]:
    try:
        trace_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H-%M-%SZ')
        path = trace_dir / f'remember_{ts}.txt'
        lines = [
            '=' * 80,
            f'[remember] {ts} exit={exit_reason} iters={len(iters)}',
            '=' * 80,
            f'Query: {query}',
            '',
        ]
        for i, it in enumerate(iters, start=1):
            lines.append(f'--- iter {i} ---')
            lines.append('ACTION:')
            if it.get('action') is not None:
                lines.append(json.dumps(it['action'], indent=2))
            else:
                lines.append('(unparseable; raw follows)')
                lines.append(it.get('raw', ''))
            obs = it.get('observation', '')
            if obs:
                lines.append('OBSERVATION:')
                lines.append(obs)
            lines.append('')
        lines.append('FINAL ANSWER:')
        lines.append(answer)
        path.write_text('\n'.join(lines), encoding='utf-8')
        return path
    except Exception as e:
        logger.warning(f"remember: trace write failed: {e}")
        return None


def remember(query: str, memory_dir: Path, llm_backend,
             trace_dir: Path) -> str:
    """Run the active-recall subagent. Returns the synthesized answer to
    the query, suitable for binding to a $stepN observation in Jill's
    parent ReAct loop. Side effect: writes a trace file under trace_dir.

    Args:
        query: natural-language question to answer.
        memory_dir: per-world per-agent memory directory.
        llm_backend: a _ChatBackend instance used to generate actions.
        trace_dir: where to write the per-call subagent trace.
    """
    if not query or not query.strip():
        return "(remember: empty query)"
    sys_prompt = _build_system_prompt(memory_dir)
    user_prefix = f"Query: {query.strip()}\n\n## Working log\n"
    log_lines: List[str] = []
    iters: List[Dict[str, Any]] = []

    def _build_user_msg() -> str:
        body = user_prefix + ('\n'.join(log_lines) + '\n' if log_lines else '')
        return body + '\nEmit next action:\n'

    answer = ''
    exit_reason = 'max_iters'
    for i in range(_MAX_ITERS):
        messages = [
            {'role': 'system', 'content': sys_prompt},
            {'role': 'user', 'content': _build_user_msg()},
        ]
        try:
            raw = llm_backend.chat(messages, max_tokens=600, temperature=0.2)
        except Exception as e:
            logger.warning(f"remember: llm call failed at iter {i+1}: {e}")
            answer = f"(remember: llm error at iter {i+1}: {e})"
            exit_reason = 'llm_error'
            break

        action = _parse_action(raw)
        iter_rec: Dict[str, Any] = {'raw': raw, 'action': action}
        iters.append(iter_rec)
        if action is None:
            log_lines.append(
                "NOTE: previous output was unparseable; emit ONE JSON action now.")
            iter_rec['observation'] = '(unparseable)'
            continue

        tool = action.get('tool')
        if tool == 'respond':
            answer = str(action.get('text', '') or '').strip() or '(no answer)'
            exit_reason = 'respond'
            iter_rec['observation'] = '(respond)'
            break

        binding = f'$step{i+1}'
        if tool == 'list':
            obs = _tool_list(memory_dir)
        elif tool == 'read':
            obs = _tool_read(
                memory_dir, action.get('file', ''),
                action.get('start_line'), action.get('end_line'),
            )
        elif tool == 'grep':
            obs = _tool_grep(
                memory_dir, action.get('pattern', ''),
                action.get('file'),
            )
        else:
            obs = (f"(unknown tool {tool!r}; available: list, read, grep, respond)")

        iter_rec['observation'] = obs
        log_lines.append(f"ACTION {i+1}: {json.dumps(action)}")
        log_lines.append(f"{binding}:")
        log_lines.append(obs)
        log_lines.append('')

    if exit_reason == 'max_iters' and not answer:
        answer = (
            "(remember: hit max iterations without responding; "
            "consider narrowing the query)")

    _write_trace(trace_dir, query, iters, answer, exit_reason)
    return answer
