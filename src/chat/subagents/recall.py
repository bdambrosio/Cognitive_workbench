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
from typing import Any, Callable, Dict, List, Optional

from chat.subagents.subagent import Subagent

logger = logging.getLogger(__name__)

_MAX_ITERS = 10
_MAX_READ_CHARS = 10_000
_MAX_GREP_HITS = 50


def _build_system_prompt(memory_dir: Path, thread_context: str = "") -> str:
    """Static system prompt — output discipline first, then a tight
    procedural sequence, then minimal tool/file detail. Stable across
    all calls in a session, so caches well on the anthropic route
    (cache_control already attached in _ChatBackend).

    `thread_context` (Stage 4b) is the activity-level threads
    inventory injected by the chat loop. When non-empty, it's prepended
    as a separate section so the subagent sees the topical structure
    of the conversation before procedural instructions. When empty
    (no threads, or older bench harnesses bypassing chat_loop), the
    prompt collapses to the original form. Per-prompt content varies
    only when the threads list changes, so the cache stability
    property is preserved across most calls."""
    base = (
        "You are a memory-search subagent for "
        f"{memory_dir}.\n"
        "\n"
        "## OUTPUT (read first)\n"
        "ONE JSON object per emission. Starts with `{`, ends with `}`. No "
        "prose. No markdown fences. No multiple objects per emission.\n"
        "\n"
        "## PROCEDURE\n"
        "1. First iter: read `conversation.txt` whole. Most questions "
        "are answerable from there.\n"
        "2. If you find the answer → respond with line citation. Done.\n"
        "3. Only if not found, switch file by question type:\n"
        "   - user state / preferences / what's on their mind → "
        "`companion_state_<entity>.txt` (read whole, ~3KB)\n"
        "   - agreements / decisions / commitments → "
        "`discourse_state_<entity>.txt` (read whole, ~5-10KB)\n"
        "   - explicit memories Jill has saved / 'what does Jill "
        "remember about X' / 'when did Jill learn X' → "
        "`memories.jsonl` (append-only event log; one record per "
        "memory write — text, category, entity, ts, source_turn_seq)\n"
        "   - 'why did Jill say X at turn N' → "
        "`reasoning_trace.jsonl` line N\n"
        "4. If conversation.txt was too long to read whole (>200 "
        "lines), grep a LITERAL phrase from the question first, then "
        "narrow read.\n"
        "5. If no file has it → respond \"I don't see that in memory\" "
        "— DO NOT fabricate.\n"
        "\n"
        "## Tools\n"
        '- `{"thought":"...","tool":"list"}`\n'
        '- `{"thought":"...","tool":"read","file":"<name>","start_line":<int?>,"end_line":<int?>}`\n'
        '- `{"thought":"...","tool":"grep","pattern":"<regex>","file":"<name>?"}`\n'
        '- `{"thought":"...","tool":"respond","text":"<answer>"}`\n'
        "\n"
        "## Rules\n"
        "- grep is for LITERAL strings only (names, tickers, exact "
        "phrases). For concept/synonym matching → read whole files.\n"
        "- Cite line numbers when answering.\n"
        "- Read cap ~10K chars/call. 10 iters total.\n"
        "- `reasoning_trace.jsonl`: line N = turn N. Reads/greps return "
        "structured records with refs auto-dereferenced one level.\n"
        "\n"
        "## Freshness\n"
        "`companion_state_*.txt` and `discourse_state_*.txt` are "
        "overwritten snapshots — always current. `memories.jsonl` and "
        "`reasoning_trace.jsonl` are append-only history; each entry "
        "reflects state at the moment it was written, not now. For "
        "'what is true now' questions, prefer the state files. In "
        "`reasoning_trace.jsonl`, fields like `recall_hits` and "
        "`active_concerns` show what was injected into Jill's prompt "
        "at that turn — use them to explain 'why did Jill respond "
        "this way at turn N', not as evidence of what Jill currently "
        "thinks or remembers.\n"
    )
    if thread_context.strip():
        return thread_context.strip() + "\n\n---\n\n" + base
    return base


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






class RecallSubagent(Subagent):
    """Active recall over the per-world memory directory.

    Primitives are plain-filesystem, NOT the git-aware ones in
    code_subagent: the memory dir matches `scenarios/*/` in .gitignore, so
    `git ls-files` would list nothing and `git check-ignore` would refuse
    every read. Same three verb names, different implementations, on
    purpose.
    """

    label = 'recall'
    max_iters = _MAX_ITERS

    def __init__(self, memory_dir: Path, llm_backend, trace_dir: Path, *,
                 system_prompt_builder: Optional[Callable[[Path], str]] = None,
                 thread_context: str = "",
                 reasoning_effort: Optional[str] = None):
        super().__init__(llm_backend, trace_dir,
                         reasoning_effort=reasoning_effort)
        self.memory_dir = Path(memory_dir)
        self._prompt_builder = system_prompt_builder
        self._thread_context = thread_context

    def precheck(self, query: str) -> Optional[str]:
        if not query or not query.strip():
            return f"({self.label}: empty query)"
        return None

    def system_prompt(self) -> str:
        if self._prompt_builder is not None:
            return self._prompt_builder(self.memory_dir)
        return _build_system_prompt(self.memory_dir,
                                    thread_context=self._thread_context)

    def primitives(self):
        return {
            'list': lambda a: _tool_list(self.memory_dir),
            'read': lambda a: _tool_read(self.memory_dir, a.get('file', ''),
                                         a.get('start_line'),
                                         a.get('end_line')),
            'grep': lambda a: _tool_grep(self.memory_dir,
                                         a.get('pattern', ''), a.get('file')),
        }


def recall(query: str, memory_dir: Path, llm_backend,
           trace_dir: Path,
           system_prompt_builder: Optional[Callable[[Path], str]] = None,
           thread_context: str = "",
           reasoning_effort: Optional[str] = None
           ) -> str:
    """Run the active-recall subagent. Returns the synthesized answer to
    the query, suitable for binding to a $stepN observation in the parent
    ReAct loop. Side effect: writes a trace file under trace_dir.

    Args:
        query: natural-language question to answer.
        memory_dir: per-world per-agent memory directory.
        llm_backend: a _ChatBackend instance used to generate actions.
        trace_dir: where to write the per-call subagent trace.
        system_prompt_builder: optional override for the system-prompt
            constructor (default `_build_system_prompt`). Used by
            bench/remember_prompt_optimization to A/B candidate prompts
            without going through chat_loop's call site. Leave None for
            production behavior. When a custom builder is supplied,
            `thread_context` is ignored — the bench harness controls
            its own prompt entirely.
        thread_context: Optional activity-level threads block prepended
            to the default system prompt (Stage 4b). chat_loop builds
            this from the agent_threads Collection at invocation time
            and passes the rendered string in. Empty string disables.
        reasoning_effort: forwarded to the backend per call; None means
            the field is never sent (launcher --reasoning sets it).
    """
    return RecallSubagent(
        memory_dir, llm_backend, trace_dir,
        system_prompt_builder=system_prompt_builder,
        thread_context=thread_context,
        reasoning_effort=reasoning_effort).run(query)


# Back-compat alias: the tool is `recall`, but this function was named
# remember() for its first three months and the bench harness imports it
# under that name.
remember = recall
