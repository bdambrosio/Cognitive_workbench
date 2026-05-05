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
        "**Read cap: ~10K characters per `read` call.** Long files truncate. "
        "For any file with more than ~5-10 turns of history, prefer `grep` "
        "for a literal phrase / name / value, then a narrow `read` line "
        "range around the hits. Whole-file reads are only safe on small "
        "current-value files (companion_state, discourse_state).\n"
        "\n"
        "- `conversation.txt` — APPEND-ONLY free text. Each entry is a "
        "timestamped header (`[ts] entity -> Jill (...)` for user input, "
        "`[ts] entity <- Jill (...)` for Jill's reply) followed by the "
        "message text. CONTENT: verbatim dialogue — what the user said, "
        "what Jill replied. SIZE: ~2-4KB per turn, grows linearly with "
        "session length. Primary substrate for user-content queries. For "
        "early-session lookups, can read whole (~3-5 turns). For mature "
        "sessions, use `grep` with a literal phrase/name from the user, "
        "then `read` a narrow range around the hit (e.g., 10-line window) "
        "to capture both directions of that exchange.\n"
        "\n"
        "- `companion_state_<entity>.txt` — CURRENT-VALUE ONLY, overwritten "
        "on each update. CONTENT: rolling fair-witness profile (CURRENT "
        "CHAPTER, STATE OF MIND, WHAT MATTERS TO THEM, OBSERVED DEFAULTS, "
        "ON THEIR MIND). SIZE: small (~3KB), readable whole. **Read whole** "
        "for queries about how the user thinks, current preferences, or "
        "what's currently on their mind. Use `list` for mtime if you need "
        "\"when was this last updated.\"\n"
        "\n"
        "- `discourse_state_<entity>.txt` — CURRENT-VALUE ONLY, overwritten "
        "on each update. CONTENT: ACTIVE COMMITMENTS, CURRENT AGREEMENTS, "
        "KEY DECISIONS MADE, unresolved issues. SIZE: small (~5-10KB), "
        "readable whole. **Read whole** for queries about what's been "
        "agreed, decided, or left unresolved between user and Jill.\n"
        "\n"
        "- `chat_trace.txt` — APPEND-ONLY free text. Each ReAct turn writes "
        "a section with header `[<timestamp>] source=... iters=... "
        "exit=...` followed by SYSTEM:, USER:, ASSISTANT: blocks. CONTENT: "
        "byte-stream of LLM I/O — Jill's intermediate reasoning (thoughts, "
        "tool calls, observations) per iteration. The SYSTEM block repeats "
        "every turn and dominates the file. SIZE: large (~5-15KB per turn, "
        "grows quickly). **Never read whole.** Use `grep` with literal "
        "strings, then `read` narrow line ranges around hits. Most "
        "user-content queries should NOT touch this file — "
        "`reasoning_trace.jsonl` is more structured and easier to navigate "
        "for reasoning-introspection, and `conversation.txt` is the right "
        "source for what was said.\n"
        "\n"
        "- `reasoning_trace.jsonl` — APPEND-ONLY JSONL. **One JSON record "
        "per line, line N = record N (= turn_seq N, 1-indexed).** Each "
        "record is a structured per-turn snapshot: turn_seq, ts, source, "
        "user_input, orientation (per-turn evaluator output — what Jill "
        "was 'operating under' at this turn), active_concerns (state at "
        "this turn), recall_hits, companion_state, discourse_state, "
        "prefix_trace_refs (list of earlier turn_seqs that were visible "
        "in this turn's awareness window), working_log (literal ReAct "
        "iter actions and observations), raw_response. SIZE: ~1-5KB per "
        "record. To read record N, call `read(file='reasoning_trace.jsonl', "
        "start_line=N, end_line=N)`. `grep` matches one whole record per "
        "hit. **Auto-dereference: when `read` or `grep` returns a record, "
        "any `prefix_trace_refs` are auto-expanded one level to one-line "
        "summaries (`#N: '<user>' → '<reply>'`). One level only — to see "
        "a referenced record in full, read its line explicitly.** "
        "Primary source for reasoning-introspection queries (\"what was "
        "Jill operating under at T11\", \"what concerns were active when "
        "she made decision X\").\n"
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
        "\"what do I need to do\", \"list all Y\") → `conversation.txt`. "
        "For early-session, can read whole; for mature sessions, `grep` "
        "with a literal phrase from the user, then `read` a narrow line "
        "window around hits. Add companion/discourse state if the query "
        "touches preferences or commitments.\n"
        "  * State / preference queries (\"how does the user think\", "
        "\"what's on their mind\", \"what have we agreed on\") → start "
        "with `companion_state_*.txt` and/or `discourse_state_*.txt`, "
        "whole.\n"
        "  * Reasoning-introspection queries (\"why did Jill say X\", "
        "\"what was Jill thinking when Y\", \"what was she operating "
        "under at turn N\") → `reasoning_trace.jsonl`. For a specific "
        "turn, read line N (= turn_seq N) — the record comes back "
        "structured with auto-dereferenced prefix_trace_refs. For a "
        "search across turns, `grep` for a literal phrase (each hit is "
        "one whole record, also structured + dereferenced). Fall back "
        "to `chat_trace.txt` only if the JSONL record doesn't have the "
        "detail you need (rare).\n"
        "- **For long files, default to grep + targeted read.** Read cap "
        "is ~10K chars; whole-file reads on long files truncate and the "
        "context window fills fast across multiple ReAct iterations. "
        "Reserve whole-reads for current-value snapshot files.\n"
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


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    """Read a .jsonl file into a list of dicts indexed by 1-based line
    number. Empty/unparseable lines become {} placeholders so list
    indices stay aligned with file line numbers."""
    out: List[Dict[str, Any]] = []
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            for line in f:
                s = line.strip()
                if not s:
                    out.append({})
                    continue
                try:
                    out.append(json.loads(s))
                except json.JSONDecodeError:
                    out.append({})
    except Exception:
        return []
    return out


# Fields in JSONL records that hold lists of references to other records
# in the same file (by turn_seq / line number). When _tool_read or
# _tool_grep returns a record, each ref in these fields is auto-expanded
# to a one-line summary of the referenced record. ONE LEVEL ONLY — no
# recursion (which would re-introduce the O(N^2) blowup we just fixed
# in storage). Add new ref field names here as the schema grows.
_JSONL_REF_FIELDS = ('prefix_trace_refs',)


def _format_jsonl_record(rec: Dict[str, Any],
                         all_records: List[Dict[str, Any]]) -> str:
    """Render a JSONL record as a structured block. Auto-expands ref
    fields (one level, summary form: '#N: <user> → <reply>') so the
    caller doesn't have to issue extra reads to know what each ref
    points at."""
    if not rec:
        return "(empty record)"
    lines: List[str] = []
    seq = rec.get('turn_seq', '?')
    ts = rec.get('ts', '')
    src = rec.get('source', '?')
    lines.append(f"=== record #{seq} (turn_seq={seq}, ts={ts}, source={src}) ===")
    for field, label in (
        ('user_input', 'USER INPUT'),
        ('orientation', 'ORIENTATION'),
    ):
        val = rec.get(field)
        if val:
            lines.append(f"{label}:")
            lines.append(str(val).rstrip())
    for field, label in (
        ('active_concerns', 'ACTIVE CONCERNS'),
        ('recall_hits', 'RECALL HITS'),
    ):
        items = rec.get(field) or []
        if items:
            lines.append(f"{label}:")
            for it in items:
                lines.append(f"  - {it}")
    for field, label in (
        ('working_log', 'WORKING LOG'),
        ('raw_response', 'RAW RESPONSE'),
    ):
        val = rec.get(field)
        if val:
            lines.append(f"{label}:")
            lines.append(str(val).rstrip())
    # One-level dereference for ref fields. Build a quick lookup by
    # turn_seq into the same file's records.
    by_seq: Dict[int, Dict[str, Any]] = {}
    for r in all_records:
        seq_key = r.get('turn_seq') if isinstance(r, dict) else None
        if isinstance(seq_key, int):
            by_seq[seq_key] = r
    for ref_field in _JSONL_REF_FIELDS:
        refs = rec.get(ref_field) or []
        if not refs:
            continue
        lines.append(f"{ref_field.upper()}: {refs}")
        for ref in refs:
            try:
                ref_int = int(ref)
            except (TypeError, ValueError):
                lines.append(f"  → {ref}: (non-int ref)")
                continue
            target = by_seq.get(ref_int)
            if target is None:
                lines.append(f"  → #{ref_int}: (not found in this file)")
                continue
            user_head = (target.get('user_input') or '').replace('\n', ' ')[:80]
            resp_head = (target.get('raw_response') or '').replace('\n', ' ')[:80]
            lines.append(f"  → #{ref_int}: '{user_head}' → '{resp_head}'")
    # Surface any remaining non-empty fields for completeness — schema
    # may grow with fields we haven't explicitly rendered here.
    rendered = {'turn_seq', 'ts', 'source', 'user_input', 'orientation',
                'active_concerns', 'recall_hits', 'working_log',
                'raw_response'} | set(_JSONL_REF_FIELDS)
    for k, v in rec.items():
        if k in rendered or not v:
            continue
        if isinstance(v, (list, dict)):
            lines.append(f"{k.upper()}: {json.dumps(v, ensure_ascii=False)}")
        else:
            lines.append(f"{k.upper()}: {v}")
    return "\n".join(lines)


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

    # JSONL: line N = record N. Render structured with one-level deref
    # of any refs (currently prefix_trace_refs).
    if path.suffix == '.jsonl':
        records = _load_jsonl(path)
        n = len(records)
        if n == 0:
            return "(read: file empty)"
        s = max(1, int(start_line)) if start_line else 1
        e = min(n, int(end_line)) if end_line else n
        if s > n:
            return f"(read: start_line {s} > file length {n} records)"
        chunks = []
        for i in range(s, e + 1):
            chunks.append(_format_jsonl_record(records[i - 1], records))
        out = "\n\n".join(chunks)
        if len(out) > _MAX_READ_CHARS:
            return (out[:_MAX_READ_CHARS]
                    + f"\n…(truncated; {len(out)} chars total, capped at "
                    f"{_MAX_READ_CHARS}; narrow the line range)")
        return out

    # Text path: line-prefixed verbatim, capped.
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
        is_jsonl = (path.suffix == '.jsonl')
        records = _load_jsonl(path) if is_jsonl else None
        try:
            with open(path, 'r', encoding='utf-8', errors='replace') as f:
                for i, ln in enumerate(f, start=1):
                    if rgx.search(ln):
                        if is_jsonl and records and i <= len(records) and records[i - 1]:
                            # Structured render with one-level deref so
                            # the caller doesn't have to re-read the
                            # matched record to use it.
                            hits.append(
                                f"{path.name}:{i}\n"
                                + _format_jsonl_record(records[i - 1], records))
                        else:
                            hits.append(f"{path.name}:{i}|{ln.rstrip()}")
                        if len(hits) >= _MAX_GREP_HITS:
                            break
        except Exception as e:
            hits.append(f"{path.name}:0|(read error: {e})")
        if len(hits) >= _MAX_GREP_HITS:
            break
    if not hits:
        return "(no matches)"
    out = "\n\n".join(hits)
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
            raw = llm_backend.chat(messages, max_tokens=4096, temperature=0.2)
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
