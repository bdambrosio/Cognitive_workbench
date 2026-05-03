"""Codebase-inspection subagent — a thin, persona-less ReAct loop that
answers queries about a source tree by reading files within a fixed root.

Used by Jill's chat ReAct loop via the `inspect` tool. From Jill's vantage,
an `inspect` call is one step: she emits a query string, gets a synthesized
answer back. The subagent's internal multi-iteration reasoning is opaque
to her trace, but written to a per-call trace file under inspect_traces/
for debugging.

Architectural notes:
- Backend hardcoded to Sonnet (caller passes a Sonnet-configured
  _ChatBackend), independent of Jill's per-scenario llm_config — code
  reasoning needs a stronger model than memory recall.
- Primitives are intentionally minimal and read-only: list, read (with
  optional line range), grep (via ripgrep), respond.
- All file paths resolve within repo_root; path traversal AND symlink
  escape are rejected.
- grep shells out to `rg` (ripgrep). Required at runtime; absence
  produces an ERROR observation rather than a silent fallback so the
  failure is loud.
"""

from __future__ import annotations

import json
import logging
import re
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

_MAX_ITERS = 12
_MAX_READ_CHARS = 10_000
_MAX_GREP_HITS = 50
_MAX_LIST_ENTRIES = 200

# Files / dirs the LLM should never need to navigate. Filter at list time
# so the subagent doesn't waste iterations on cache or compiled artifacts.
_LIST_DENY_NAMES = {
    '__pycache__', '.git', '.mypy_cache', '.pytest_cache', '.ruff_cache',
    'node_modules',
}
_LIST_DENY_SUFFIXES = ('.pyc', '.pyo', '.so', '.bak', '.faiss', '.meta')


def _build_system_prompt(repo_root: Path) -> str:
    """Static system prompt — describes the subagent's role, the geofence
    root, and read-strategy discipline for code. Stable across all calls
    in a session, caches well on the anthropic route."""
    return (
        "You are a codebase-inspection subagent. You answer questions "
        "about a source tree by reading its files. You have no persona, "
        "no goals beyond the current query, and no memory of your own "
        "past calls — each invocation is independent.\n"
        "\n"
        f"Repo root (geofence): {repo_root}\n"
        "All file paths are relative to this root. Navigation outside "
        "the root is not possible.\n"
        "\n"
        "## Read strategy\n"
        "\n"
        "**Read cap: ~10K characters per `read` call.** Long files "
        "truncate. For source files larger than ~300 lines, prefer "
        "`grep` for a literal symbol / phrase / definition keyword, "
        "then a narrow `read` line range (e.g., 30-line window) around "
        "the hits. Whole-file reads are appropriate for short modules "
        "(~<300 lines) where you need full context.\n"
        "\n"
        "Code-specific reading patterns that work well:\n"
        "- 'Where is X defined?' → grep `def X\\b|class X\\b`, then read "
        "  a window around the hit.\n"
        "- 'What calls Y?' → grep `\\bY\\(` (or just `Y` for broader "
        "  matches), inspect each call site.\n"
        "- 'How does Z work?' → list the directory, identify the right "
        "  file from filenames, then read the function body or the "
        "  whole file if short.\n"
        "- 'Show me the test for W' → list the tests/ or bench/ tree, "
        "  then grep for `W` within tests.\n"
        "\n"
        "## Tools (one JSON object per emission)\n"
        "\n"
        '1. {"thought": "<one sentence>", "tool": "list", "path": '
        '"<relative_dir>?"} — list files and subdirs. `path` is '
        "optional, defaults to repo root. Pass a relative subdir "
        "(e.g. `chat`, `tools/search-web`) to navigate deeper. "
        "Output: one entry per line, format `<name>\\t<size_or_DIR>` "
        "for each file/subdir; cache and compiled-artifact names are "
        "filtered out.\n"
        '2. {"thought": "...", "tool": "read", "file": "<relative_path>", '
        '"start_line": <int?>, "end_line": <int?>} — read a file. Omit '
        "start_line/end_line to read the whole file (capped at ~10K "
        "chars; use line ranges for larger files). Lines are 1-indexed; "
        "output format is `lineno|content`.\n"
        '3. {"thought": "...", "tool": "grep", "pattern": "<regex>", '
        '"path": "<relative_path>?"} — ripgrep over the repo. `path` is '
        "optional and may be a single file OR a subdirectory. Pattern "
        "is a regex (rg's default syntax). Output format is "
        "`<relative_path>:<lineno>:<content>` per hit, capped at "
        f"{_MAX_GREP_HITS} hits.\n"
        '4. {"thought": "...", "tool": "respond", "text": "<answer>"} — '
        "final answer to the query, exits the loop. Cite specific paths "
        "and line numbers (`src/chat/chat_loop.py:1860`) when grounding "
        "claims in code — the caller wants to verify against the source.\n"
        "\n"
        "## Discipline\n"
        "\n"
        "- **You have no model of the codebase prior to this call.** Do "
        "  not assume conventions, file locations, or APIs from training "
        "  data — verify by reading. The repo may have idiosyncratic "
        "  structure.\n"
        "- **Pick the right primitive for the query type before any "
        "  tool call:**\n"
        "    * 'Where is X' / 'what calls Y' → grep first.\n"
        "    * 'How does this directory work' → list first, then read the "
        "  obvious entry-point file (e.g. `__init__.py`, `main.py`, the "
        "  largest file by name relevance).\n"
        "    * 'Show me the implementation of Z' → if you know the file, "
        "  read directly; if not, grep then read around the hit.\n"
        "- **Don't loop blindly.** If grep returns 50 hits, narrow the "
        "  pattern rather than reading every hit. If a read truncates, "
        "  use a tighter line range.\n"
        "- **Cite paths and line numbers in the final answer.** Format: "
        "  `<relative_path>:<lineno>` or `<relative_path>:<start>-<end>`. "
        "  The caller verifies against actual source.\n"
        "- **If the codebase does not contain what's asked, say so "
        "  plainly** — do not speculate beyond evidence.\n"
        "- Output ONLY one JSON object per emission. No prose, no "
        "  markdown fences."
    )


def _safe_resolve(repo_root: Path, name: str,
                  must_be_dir: bool = False,
                  must_be_file: bool = False) -> Optional[Path]:
    """Resolve a path within repo_root; reject path traversal AND symlink
    escape. Returns the resolved path or None on rejection."""
    if name is None:
        return None
    name = str(name).strip()
    # Empty / dot-only refers to the root itself.
    if name in ('', '.', './'):
        candidate = repo_root.resolve()
    else:
        try:
            candidate = (repo_root / name).resolve()
        except Exception:
            return None
    try:
        root = repo_root.resolve()
    except Exception:
        return None
    # Inclusive: candidate may equal root (when name is empty/dot).
    try:
        candidate.relative_to(root)
    except ValueError:
        return None
    if must_be_dir and not candidate.is_dir():
        return None
    if must_be_file and not candidate.is_file():
        return None
    return candidate


def _rel_to_root(repo_root: Path, p: Path) -> str:
    """Render a resolved path as a string relative to repo_root."""
    try:
        return str(p.relative_to(repo_root.resolve()))
    except ValueError:
        return str(p)


def _is_hidden_or_denied(p: Path) -> bool:
    """Filter for list output: hide caches, compiled artifacts, hidden
    dotfiles. Symlinks pointing outside the geofence are caught earlier
    by _safe_resolve; here we only filter for noise reduction."""
    if p.name.startswith('.'):
        return True
    if p.name in _LIST_DENY_NAMES:
        return True
    if p.is_file() and any(p.name.endswith(s) for s in _LIST_DENY_SUFFIXES):
        return True
    return False


def _tool_list(repo_root: Path, path: Optional[str]) -> str:
    target = _safe_resolve(repo_root, path or '', must_be_dir=True)
    if target is None:
        return f"ERROR: list invalid or out-of-scope path: {path!r}"
    rows = []
    try:
        entries = sorted(target.iterdir(),
                         key=lambda p: (not p.is_dir(), p.name.lower()))
    except Exception as e:
        return f"ERROR: list failed: {e}"
    for p in entries:
        if _is_hidden_or_denied(p):
            continue
        if p.is_dir():
            rows.append(f"{p.name}/\tDIR")
        else:
            try:
                size = p.stat().st_size
            except Exception:
                size = 0
            rows.append(f"{p.name}\t{size} bytes")
        if len(rows) >= _MAX_LIST_ENTRIES:
            rows.append(f"(list capped at {_MAX_LIST_ENTRIES} entries)")
            break
    rel = _rel_to_root(repo_root, target) or '.'
    if not rows:
        return f"OK: {rel}/\n(empty after filtering)"
    return f"OK: {rel}/\n" + "\n".join(rows)


def _tool_read(repo_root: Path, name: str,
               start_line: Optional[int], end_line: Optional[int]) -> str:
    if not name:
        return "ERROR: read requires a `file` argument"
    path = _safe_resolve(repo_root, name, must_be_file=True)
    if path is None:
        return f"ERROR: read invalid or out-of-scope file: {name!r}"
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            lines = f.readlines()
    except Exception as e:
        return f"ERROR: read failed: {e}"
    n = len(lines)
    if n == 0:
        return f"EMPTY: {name} is empty"
    s = max(1, int(start_line)) if start_line else 1
    e = min(n, int(end_line)) if end_line else n
    if s > n:
        return f"EMPTY: read start_line {s} > file length {n}"
    selected = lines[s - 1:e]
    out = ''.join(f"{s + i}|{ln}" for i, ln in enumerate(selected))
    if len(out) > _MAX_READ_CHARS:
        return ('OK: ' + out[:_MAX_READ_CHARS]
                + f"\n…(truncated; {len(out)} chars total, capped at "
                f"{_MAX_READ_CHARS}; narrow the line range)")
    if not out.strip():
        return f"EMPTY: read range {s}..{e} of {name} contains only blank lines"
    return 'OK: ' + out


def _tool_grep(repo_root: Path, pattern: str,
               path: Optional[str]) -> str:
    if not pattern:
        return "ERROR: grep requires a `pattern` argument"
    if shutil.which('rg') is None:
        return ("ERROR: ripgrep (`rg`) is not on PATH — install with "
                "`apt install ripgrep`")
    target = _safe_resolve(repo_root, path or '')
    if target is None:
        return f"ERROR: grep invalid or out-of-scope path: {path!r}"
    cmd = [
        'rg',
        '--line-number',
        '--with-filename',     # always emit path:line:content (single-file
                               # grep otherwise omits the filename, which
                               # would be inconsistent with directory mode)
        '--no-heading',
        '--color=never',
        f'--max-count={_MAX_GREP_HITS}',
        '--max-columns=300',
        '--max-filesize=2M',
        '--glob=!__pycache__',
        '--glob=!*.pyc',
        '--glob=!*.bak',
        '--glob=!*.faiss',
        '--glob=!*.meta',
        '--',
        pattern,
        str(target),
    ]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=20.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return "ERROR: grep timed out (>20s) — narrow the pattern or scope"
    except Exception as e:
        return f"ERROR: grep failed to launch: {e}"
    # rg exit codes: 0 = matches, 1 = no matches, 2 = error.
    if proc.returncode == 1:
        return f"EMPTY: no matches for pattern {pattern!r} in {path or '.'}"
    if proc.returncode != 0:
        msg = (proc.stderr or '').strip().splitlines()[:3]
        return f"ERROR: grep returned {proc.returncode}: " + ' | '.join(msg)
    # Rewrite absolute paths to repo-relative for stable output across
    # invocations and to keep observation lines short.
    root_str = str(repo_root.resolve())
    out_lines: List[str] = []
    total = 0
    for raw_line in (proc.stdout or '').splitlines():
        if not raw_line:
            continue
        if raw_line.startswith(root_str + '/'):
            raw_line = raw_line[len(root_str) + 1:]
        elif raw_line.startswith(root_str):
            raw_line = raw_line[len(root_str):]
        out_lines.append(raw_line)
        total += 1
        if total >= _MAX_GREP_HITS:
            out_lines.append(f"(grep capped at {_MAX_GREP_HITS} hits — "
                             "narrow the pattern for full coverage)")
            break
    if not out_lines:
        return f"EMPTY: no matches for pattern {pattern!r} in {path or '.'}"
    return 'OK: ' + "\n".join(out_lines)


def _parse_action(raw: str) -> Optional[Dict[str, Any]]:
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
        path = trace_dir / f'inspect_{ts}.txt'
        lines = [
            '=' * 80,
            f'[inspect] {ts} exit={exit_reason} iters={len(iters)}',
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
        logger.warning(f"inspect: trace write failed: {e}")
        return None


def inspect(query: str, repo_root: Path, llm_backend,
            trace_dir: Path) -> str:
    """Run the codebase-inspection subagent. Returns the synthesized
    answer string, suitable for binding to a $stepN observation in
    Jill's parent ReAct loop. Side effect: writes a per-call trace file
    under trace_dir.

    Args:
        query: natural-language question about the codebase.
        repo_root: directory the subagent is geofenced to (typically
            src/). All file primitives reject paths outside this root.
        llm_backend: a _ChatBackend instance used to generate actions.
            Caller is responsible for using a strong model (Sonnet).
        trace_dir: where to write the per-call subagent trace.
    """
    if not query or not query.strip():
        return "(inspect: empty query)"
    repo_root = Path(repo_root)
    if not repo_root.is_dir():
        return f"(inspect: repo_root does not exist: {repo_root})"
    sys_prompt = _build_system_prompt(repo_root)
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
            raw = llm_backend.chat(messages, max_tokens=800, temperature=0.2)
        except Exception as e:
            logger.warning(f"inspect: llm call failed at iter {i+1}: {e}")
            answer = f"(inspect: llm error at iter {i+1}: {e})"
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
            obs = _tool_list(repo_root, action.get('path'))
        elif tool == 'read':
            obs = _tool_read(
                repo_root, action.get('file', ''),
                action.get('start_line'), action.get('end_line'),
            )
        elif tool == 'grep':
            obs = _tool_grep(
                repo_root, action.get('pattern', ''),
                action.get('path'),
            )
        else:
            obs = (f"ERROR: unknown tool {tool!r}; available: "
                   "list, read, grep, respond")

        iter_rec['observation'] = obs
        log_lines.append(f"ACTION {i+1}: {json.dumps(action)}")
        log_lines.append(f"{binding}:")
        log_lines.append(obs)
        log_lines.append('')

    if exit_reason == 'max_iters' and not answer:
        answer = ("(inspect: hit max iterations without responding; "
                  "consider narrowing the query)")

    _write_trace(trace_dir, query, iters, answer, exit_reason)
    return answer
