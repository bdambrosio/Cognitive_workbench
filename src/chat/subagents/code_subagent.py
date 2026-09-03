"""Codebase-inspection subagent — a thin, persona-less ReAct loop that
answers queries about a source tree by reading files within a fixed root.

Two entry points share the same loop, parser, and primitives:

  - inspect(...)          — self-introspection over the agent's own src/.
  - inspect_external(...) — read an external project repo bound for the
                            session (sticky binding via Note + YAML).

Both go through Jill's ReAct loop. The ReAct LLM picks tools by referent
("about myself" → inspect, "about <named repo>" → inspect_external) and
calls them with the same shape ({tool, query}); the geofence and prompt
framing differ, the rest is shared.

Architectural notes:
- Backend is whatever the caller passes — by default this is the main
  character's backend (self.backend in chat_loop), so per-scenario YAML
  decides the model. No per-subagent backend overrides.
- Primitives are intentionally minimal and read-only: list, read (with
  optional line range), grep (via ripgrep), respond.
- All file paths resolve within repo_root; path traversal AND symlink
  escape are rejected.
- list is git-aware when repo_root is a git checkout (uses
  `git ls-files -co --exclude-standard` so tracked + uncommitted-not-
  ignored files are visible, gitignored noise is hidden). Falls back to
  raw filesystem with a small deny-list for non-git directories.
- grep shells out to `rg` (ripgrep), which already respects .gitignore
  inside git checkouts and skips binaries by default. Required at
  runtime; absence produces an ERROR observation rather than a silent
  fallback so the failure is loud.
- read refuses gitignored paths (via `git check-ignore`) so all three
  primitives respect the same boundary. The geofence (_safe_resolve)
  is the authoritative path-traversal layer; the gitignore check is an
  additional discipline layer keeping generated / runtime artifacts
  out of read output.
"""

from __future__ import annotations

import json
import logging
import re
import shutil
import subprocess

from chat.subagents.subagent import Subagent
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

_MAX_ITERS = 12
_MAX_READ_CHARS = 10_000
_MAX_GREP_HITS = 50
_MAX_LIST_ENTRIES = 200

# Filesystem-fallback deny list (used when repo_root isn't a git checkout).
# Inside git checkouts, .gitignore handles this and the deny list is
# unused. Names match common cache / build / vendored-deps directories
# across language ecosystems.
_LIST_DENY_NAMES = {
    '__pycache__', '.git', '.mypy_cache', '.pytest_cache', '.ruff_cache',
    'node_modules', '.venv', 'venv', 'dist', 'build', 'target',
    '.next', '.cache', '.tox',
}
_LIST_DENY_SUFFIXES = ('.pyc', '.pyo', '.so', '.bak', '.faiss', '.meta')


# ---------------------------------------------------------------------------
# System prompt
# ---------------------------------------------------------------------------

def _build_system_prompt(repo_root: Path, mode: str) -> str:
    """Static system prompt — describes the subagent's role, the geofence
    root, and read-strategy discipline for code. Stable across all calls
    in a session, caches well on the anthropic route.

    `mode` is 'self' (the agent's own substrate) or 'external' (a project
    repo bound for this session). The two differ only in the framing line
    above the geofence; primitives and discipline are identical.
    """
    if mode == 'external':
        framing = (
            "You are a codebase-inspection subagent. You answer questions "
            "about an external project repo by reading its files. You have "
            "no persona, no goals beyond the current query, and no memory "
            "of your own past calls — each invocation is independent.\n"
            "\n"
            "This is an external project, not your own substrate. Read it "
            "as documentation: when the question is about overall shape "
            "or unfamiliar terrain, list the root and read README.md / "
            "similar top-level docs first; then drill in. The repo's "
            "conventions may not match anything in your training — verify "
            "by reading."
        )
    else:
        framing = (
            "You are a codebase-inspection subagent. You answer questions "
            "about a source tree by reading its files. You have no persona, "
            "no goals beyond the current query, and no memory of your own "
            "past calls — each invocation is independent."
        )
    return (
        f"{framing}\n"
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
        "for each file/subdir; in git checkouts, gitignored entries "
        "are hidden automatically.\n"
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


# ---------------------------------------------------------------------------
# Path resolution and helpers
# ---------------------------------------------------------------------------

def _safe_resolve(repo_root: Path, name: str,
                  must_be_dir: bool = False,
                  must_be_file: bool = False) -> Optional[Path]:
    """Resolve a path within repo_root; reject path traversal AND symlink
    escape. Returns the resolved path or None on rejection."""
    if name is None:
        return None
    name = str(name).strip()
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
    """Filesystem-fallback filter for non-git directories."""
    if p.name.startswith('.'):
        return True
    if p.name in _LIST_DENY_NAMES:
        return True
    if p.is_file() and any(p.name.endswith(s) for s in _LIST_DENY_SUFFIXES):
        return True
    return False


def _is_git_checkout(repo_root: Path) -> bool:
    """True if `<repo_root>/.git` exists (as a dir for a normal checkout
    or as a file for a worktree). The `git` binary is also required."""
    try:
        if not (repo_root / '.git').exists():
            return False
    except Exception:
        return False
    return shutil.which('git') is not None


def _is_gitignored(repo_root: Path, target: Path) -> bool:
    """Return True if `target` is gitignored relative to repo_root.
    Used by `_tool_read` so the read primitive respects the same
    gitignore boundary that list (via `git ls-files -co
    --exclude-standard`) and grep (ripgrep, which respects .gitignore
    by default in git checkouts) already do.

    Returns False — fail-open — when:
      - repo_root isn't a git checkout, or
      - the git binary is unavailable, or
      - `git check-ignore` itself errors (rare; transient git issues).
    The geofence (`_safe_resolve`) is the authoritative path-traversal
    boundary; this is an additional discipline layer to keep generated/
    runtime artifacts out of read output."""
    if not _is_git_checkout(repo_root):
        return False
    try:
        proc = subprocess.run(
            ['git', 'check-ignore', '--quiet', '--', str(target)],
            cwd=str(repo_root.resolve()),
            capture_output=True, timeout=5.0, check=False,
        )
    except Exception as e:
        logger.warning(f"code_subagent: git check-ignore failed: {e}")
        return False
    # Exit 0 = ignored. Exit 1 = not ignored. Exit 128 = error / not in
    # repo. Treat anything other than 0 as not-ignored (fail-open).
    return proc.returncode == 0


# ---------------------------------------------------------------------------
# Tool: list
# ---------------------------------------------------------------------------

def _list_via_git(repo_root: Path, target: Path) -> Optional[List[Tuple[str, bool]]]:
    """List entries directly under `target` using git ls-files. Returns a
    list of (name, is_dir) sorted (dirs first, then files, both alphabetic),
    or None if the git invocation failed.

    Strategy: `git ls-files -co --exclude-standard` lists tracked +
    cached + untracked-not-ignored files relative to repo_root. We filter
    to entries whose first path segment under `target` is the entry, and
    derive directories from prefixes."""
    rel_target = _rel_to_root(repo_root, target)
    prefix = '' if rel_target in ('.', '') else rel_target.rstrip('/') + '/'
    try:
        proc = subprocess.run(
            ['git', 'ls-files', '-co', '--exclude-standard', '--', '.'],
            cwd=str(repo_root.resolve()),
            capture_output=True, text=True, timeout=10.0, check=False,
        )
    except Exception as e:
        logger.warning(f"code_subagent: git ls-files failed: {e}")
        return None
    if proc.returncode != 0:
        return None
    files: set = set()
    dirs: set = set()
    for raw in (proc.stdout or '').splitlines():
        path = raw.strip()
        if not path:
            continue
        if prefix and not path.startswith(prefix):
            continue
        rest = path[len(prefix):]
        if not rest:
            continue
        head, sep, _tail = rest.partition('/')
        if sep:
            dirs.add(head)
        else:
            files.add(head)
    rows: List[Tuple[str, bool]] = []
    for d in sorted(dirs, key=str.lower):
        rows.append((d, True))
    for f in sorted(files, key=str.lower):
        rows.append((f, False))
    return rows


def _tool_list(repo_root: Path, path: Optional[str]) -> str:
    target = _safe_resolve(repo_root, path or '', must_be_dir=True)
    if target is None:
        return f"ERROR: list invalid or out-of-scope path: {path!r}"
    rel = _rel_to_root(repo_root, target) or '.'

    rows: List[str] = []
    if _is_git_checkout(repo_root):
        git_rows = _list_via_git(repo_root, target)
        if git_rows is not None:
            for name, is_dir in git_rows:
                if is_dir:
                    rows.append(f"{name}/\tDIR")
                else:
                    full = target / name
                    try:
                        size = full.stat().st_size
                    except Exception:
                        size = 0
                    rows.append(f"{name}\t{size} bytes")
                if len(rows) >= _MAX_LIST_ENTRIES:
                    rows.append(f"(list capped at {_MAX_LIST_ENTRIES} entries)")
                    break
            if not rows:
                return f"OK: {rel}/\n(empty)"
            return f"OK: {rel}/\n" + "\n".join(rows)
        # Fall through to filesystem on git failure.

    # Filesystem fallback (non-git or git invocation failed).
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
    if not rows:
        return f"OK: {rel}/\n(empty after filtering)"
    return f"OK: {rel}/\n" + "\n".join(rows)


# ---------------------------------------------------------------------------
# Tool: read
# ---------------------------------------------------------------------------

def _tool_read(repo_root: Path, name: str,
               start_line: Optional[int], end_line: Optional[int]) -> str:
    if not name:
        return "ERROR: read requires a `file` argument"
    path = _safe_resolve(repo_root, name, must_be_file=True)
    if path is None:
        return f"ERROR: read invalid or out-of-scope file: {name!r}"
    if _is_gitignored(repo_root, path):
        return (f"ERROR: read refused: {name} is gitignored "
                f"(generated or runtime artifact, not source). list and "
                f"grep already hide such files; pick a tracked file.")
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


# ---------------------------------------------------------------------------
# Tool: grep (ripgrep — already respects .gitignore inside git checkouts
# and skips binaries by default)
# ---------------------------------------------------------------------------

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
        '--with-filename',
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
    ]
    # THE SAME BOUNDARY AS list AND read. Those two apply git's ignore
    # rules only when repo_root is itself a checkout; ripgrep applies the
    # rules of any checkout ABOVE the root, so a root that is an ignored
    # subdirectory of one — an engagement's merged/ and runs/ under this
    # repo — grepped as empty while list showed the files and read opened
    # them (continuation, 2026-09-03: "Redis" found nowhere in a record with
    # 45 occurrences). Outside a checkout root, search everything.
    if not _is_git_checkout(repo_root):
        cmd.append('--no-ignore')
    cmd += ['--', pattern, str(target)]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=20.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return "ERROR: grep timed out (>20s) — narrow the pattern or scope"
    except Exception as e:
        return f"ERROR: grep failed to launch: {e}"
    if proc.returncode == 1:
        return f"EMPTY: no matches for pattern {pattern!r} in {path or '.'}"
    if proc.returncode != 0:
        msg = (proc.stderr or '').strip().splitlines()[:3]
        return f"ERROR: grep returned {proc.returncode}: " + ' | '.join(msg)
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


# ---------------------------------------------------------------------------
# Loop core
# ---------------------------------------------------------------------------

class CodeSubagent(Subagent):
    """Read-only navigation of a source tree under a fixed root.

    `inspect` and `inspect_external` are the SAME class with different
    configuration, not two subclasses: identical primitives, identical
    budgets, differing only in geofence root, prompt framing, and label.
    A subclass pair here would have carried one string each.

    Primitives are git-aware (`git ls-files`, `git check-ignore`,
    ripgrep). That is correct for a checkout and wrong for the recall
    subagent's gitignored world directory, which is why the two do not
    share primitive implementations.
    """

    max_iters = _MAX_ITERS

    def __init__(self, repo_root: Path, llm_backend, trace_dir: Path, *,
                 mode: str = 'self',
                 reasoning_effort: Optional[str] = None):
        super().__init__(llm_backend, trace_dir,
                         reasoning_effort=reasoning_effort)
        self.repo_root = Path(repo_root)
        self.mode = mode
        self.label = 'inspect_external' if mode == 'external' else 'inspect'

    def precheck(self, query: str) -> Optional[str]:
        if not query or not query.strip():
            return f"({self.label}: empty query)"
        if not self.repo_root.is_dir():
            return (f"({self.label}: repo_root does not exist: "
                    f"{self.repo_root})")
        return None

    def system_prompt(self) -> str:
        return _build_system_prompt(self.repo_root, self.mode)

    def primitives(self):
        return {
            'list': lambda a: _tool_list(self.repo_root, a.get('path')),
            'read': lambda a: _tool_read(self.repo_root, a.get('file', ''),
                                         a.get('start_line'),
                                         a.get('end_line')),
            'grep': lambda a: _tool_grep(self.repo_root,
                                         a.get('pattern', ''), a.get('path')),
        }


def inspect(query: str, repo_root: Path, llm_backend,
            trace_dir: Path, reasoning_effort: Optional[str] = None) -> str:
    """Self-introspection: navigate the agent's own codebase under
    `repo_root` (typically the repo root) and answer the query.

    Args:
        query: natural-language question about the codebase.
        repo_root: directory the subagent is geofenced to. All file
            primitives reject paths outside this root.
        llm_backend: a _ChatBackend instance used to generate actions.
            By convention this is the main character backend; the
            per-scenario llm_config decides the model.
        trace_dir: where to write the per-call subagent trace.
        reasoning_effort: forwarded to the backend per call; None means
            the field is never sent (launcher --reasoning sets it).
    """
    return CodeSubagent(repo_root, llm_backend, trace_dir, mode='self',
                        reasoning_effort=reasoning_effort).run(query)


def inspect_external(query: str, repo_root: Path, llm_backend,
                     trace_dir: Path,
                     reasoning_effort: Optional[str] = None) -> str:
    """External-codebase inspection: navigate a project repo bound for
    this session (sticky binding via `/set-external-repo` or the YAML
    `external_repo` field). Same primitives as `inspect`, neutral prompt
    framing — this is reading documentation, not introspection.

    Args:
        query: natural-language question about the external repo.
        repo_root: bound external repo root.
        llm_backend: same conventions as `inspect`.
        trace_dir: where to write the per-call subagent trace (same
            directory as `inspect`; filename prefix differentiates).
        reasoning_effort: same conventions as `inspect`.
    """
    return CodeSubagent(repo_root, llm_backend, trace_dir, mode='external',
                        reasoning_effort=reasoning_effort).run(query)
