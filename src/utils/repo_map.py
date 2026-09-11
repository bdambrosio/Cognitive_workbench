"""Repository map: the shape of a source tree, generated once per tree state
and cached, for the code subagent's `map` primitive and the claims runner's
warm-up.

What it is. A mechanical tree (every file the subagent's own `list` would
show, with sizes and line counts, from the same git listing) plus a ROLE per
directory written by a model: a category word for the kind of files the
directory holds ("route handlers", "tests", "dependency manifest"). Roles
describe how the tree is organized, never what the code does; the rendered
text says so, because a role that reads as a function claim would stop the
subagent checking.

Why a cache keyed on the tree's shape. Design settled with Jill 2026-09-11:
the key is a hash over (path, size) for every listed file, so any added,
removed or edited file is a miss and the map is regenerated. A stale map is
never served, which is stronger than a hash the reader could refuse. The
cache is never written inside the target; the caller names a directory
outside it (a world directory, a run's working record).

Excludes are not in the map. They are query-time state and stay in the
subagent's list/read/grep.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import re
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

#: Files larger than this are not line-counted (their size is still shown).
_MAX_LINECOUNT_BYTES = 2_000_000
#: Directories labelled per model call.
_ROLE_BATCH = 40
#: File names shown to the labeller per directory.
_SAMPLE_NAMES = 8
_ROLE_MAX_WORDS = 4
_ROLE_MAX_CHARS = 40
#: Rendered slice cap; a larger tree is asked for by subtree.
_RENDER_MAX_LINES = 200

#: Filesystem-fallback skip list for a root that is not a git checkout.
#: Inside a checkout the git listing applies .gitignore instead.
_WALK_SKIP_NAMES = {
    '__pycache__', '.git', '.mypy_cache', '.pytest_cache', '.ruff_cache',
    'node_modules', '.venv', 'venv', 'dist', 'build', 'target',
    '.next', '.cache', '.tox',
}

_ROLE_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "additionalProperties": {"type": "string", "maxLength": _ROLE_MAX_CHARS},
}

_ROLE_SYSTEM = (
    "You label directories of a source tree by the KIND of files they hold. "
    "A label is one to three words naming a category or shape of content, "
    "for example: 'route handlers', 'tests', 'dependency manifest', "
    "'database migrations', 'static assets', 'vendored code', 'build config', "
    "'CI workflows', 'documentation', 'shell scripts', 'type definitions', "
    "'fixtures', 'package root'. Describe how the tree is organized, never "
    "what the code does: a label such as 'handles authentication' or "
    "'sends email' is wrong even when true. Where the kind is unclear, "
    "write 'mixed' or 'unknown' rather than guessing a function. Answer "
    "with one JSON object mapping each directory path, exactly as given, "
    "to its label. Nothing else."
)


# ---------------------------------------------------------------------------
# Listing and hashing
# ---------------------------------------------------------------------------

def list_tree(root: Path) -> List[Tuple[str, int]]:
    """Every file under `root` as (relative path, size), sorted. Uses
    `git ls-files -co --exclude-standard` in a checkout, so tracked and
    untracked-not-ignored files are included and ignored ones are not;
    falls back to a walk with a small skip list elsewhere."""
    root = Path(root).resolve()
    paths: Optional[List[str]] = None
    try:
        proc = subprocess.run(
            ['git', 'ls-files', '-co', '--exclude-standard', '--', '.'],
            cwd=str(root), capture_output=True, text=True, timeout=30.0,
            check=False)
        if proc.returncode == 0:
            paths = [p.strip() for p in (proc.stdout or '').splitlines()
                     if p.strip()]
    except Exception as e:                                       # noqa: BLE001
        logger.warning("repo_map: git ls-files failed under %s: %s", root, e)
    if paths is None:
        paths = []
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = sorted(d for d in dirnames
                                 if not d.startswith('.')
                                 and d not in _WALK_SKIP_NAMES)
            for fn in filenames:
                if fn.startswith('.'):
                    continue
                paths.append(os.path.relpath(os.path.join(dirpath, fn), root))
    entries: List[Tuple[str, int]] = []
    for p in sorted(set(paths)):
        fp = root / p
        try:
            if not fp.is_file():
                continue          # a submodule shows in ls-files as a dir
            entries.append((p.replace(os.sep, '/'), fp.stat().st_size))
        except OSError:
            continue
    return entries


def tree_hash(entries: List[Tuple[str, int]]) -> str:
    """Hash of the tree's shape: every path with its size."""
    h = hashlib.sha256()
    for p, s in entries:
        h.update(f"{p}\t{s}\n".encode('utf-8'))
    return h.hexdigest()[:16]


def _line_count(fp: Path, size: int) -> Optional[int]:
    """Lines in a text file; None for a binary or an oversized file."""
    if size > _MAX_LINECOUNT_BYTES:
        return None
    try:
        with open(fp, 'rb') as f:
            data = f.read()
    except OSError:
        return None
    if b'\x00' in data[:8192]:
        return None
    if not data:
        return 0
    n = data.count(b'\n')
    if not data.endswith(b'\n'):
        n += 1
    return n


def _new_dir() -> Dict[str, Any]:
    return {'files': [], 'subdirs': set(), 'n_files': 0, 'n_lines': 0,
            'n_binary': 0, 'role': None}


def build_dirs(root: Path, entries: List[Tuple[str, int]]) -> Dict[str, Dict[str, Any]]:
    """Directory table keyed by relative path ('' is the root). Each entry
    holds its direct files as [name, size, lines], its direct subdirectory
    names, and recursive file, line and binary counts."""
    dirs: Dict[str, Dict[str, Any]] = {'': _new_dir()}
    for p, size in entries:
        parts = p.split('/')
        for i in range(1, len(parts)):
            d = '/'.join(parts[:i])
            parent = '/'.join(parts[:i - 1])
            if d not in dirs:
                dirs[d] = _new_dir()
            dirs[parent]['subdirs'].add(parts[i - 1])
        lines = _line_count(root / p, size)
        dirs['/'.join(parts[:-1])]['files'].append([parts[-1], size, lines])
        for i in range(len(parts)):
            d = dirs['/'.join(parts[:i])]
            d['n_files'] += 1
            if lines is None:
                d['n_binary'] += 1
            else:
                d['n_lines'] += lines
    for d in dirs.values():
        d['subdirs'] = sorted(d['subdirs'], key=str.lower)
        d['files'].sort(key=lambda f: f[0].lower())
    return dirs


# ---------------------------------------------------------------------------
# Roles
# ---------------------------------------------------------------------------

def _clean_role(label: Any) -> Optional[str]:
    """Shape check only: a short phrase. What the phrase says is the
    labeller's responsibility and a reviewer's to catch."""
    if not isinstance(label, str):
        return None
    text = re.sub(r'\s+', ' ', label).strip().strip('.').strip()
    if not text:
        return None
    words = text.split(' ')
    if len(words) > _ROLE_MAX_WORDS:
        text = ' '.join(words[:_ROLE_MAX_WORDS])
    return text[:_ROLE_MAX_CHARS]


def _describe_dir(path: str, d: Dict[str, Any]) -> str:
    names = [f[0] for f in d['files']][:_SAMPLE_NAMES]
    subs = d['subdirs'][:_SAMPLE_NAMES]
    parts = [f"{path}/  ({d['n_files']} files, {d['n_lines']} lines)"]
    if subs:
        parts.append("subdirs: " + ", ".join(subs))
    if names:
        parts.append("files: " + ", ".join(names))
    return "  ".join(parts)


def label_roles(dirs: Dict[str, Dict[str, Any]], llm_backend) -> Dict[str, Any]:
    """Ask the model for a role per directory, in batches. Writes into
    `dirs[path]['role']`; a batch that fails leaves its roles None and the
    map stays usable. Returns counts for the record."""
    paths = [p for p in sorted(dirs) if p]
    calls = 0
    labelled = 0
    t0 = time.monotonic()
    for i in range(0, len(paths), _ROLE_BATCH):
        batch = paths[i:i + _ROLE_BATCH]
        user = ("Directories, one per line, with recursive counts and a "
                "sample of names:\n\n"
                + "\n".join(_describe_dir(p, dirs[p]) for p in batch))
        calls += 1
        try:
            raw = llm_backend.chat(
                [{'role': 'system', 'content': _ROLE_SYSTEM},
                 {'role': 'user', 'content': user}],
                max_tokens=4000, cot_profile='none',
                reasoning_effort='none', response_schema=_ROLE_SCHEMA)
        except Exception as e:                                   # noqa: BLE001
            logger.warning("repo_map: role labelling call %d failed: %s", calls, e)
            continue
        from utils.json_utils import repair_json_string
        parsed = repair_json_string(raw or '')
        if not isinstance(parsed, dict):
            logger.warning("repo_map: role labelling call %d returned no "
                           "object: %r", calls, (raw or '')[:120])
            continue
        for p in batch:
            role = _clean_role(parsed.get(p) or parsed.get(p + '/'))
            if role:
                dirs[p]['role'] = role
                labelled += 1
    return {'directories': len(paths), 'labelled': labelled, 'calls': calls,
            'seconds': round(time.monotonic() - t0, 1)}


# ---------------------------------------------------------------------------
# Generate, cache, render
# ---------------------------------------------------------------------------

def generate(root: Path, entries: List[Tuple[str, int]], h: str,
             llm_backend) -> Dict[str, Any]:
    root = Path(root).resolve()
    dirs = build_dirs(root, entries)
    roles = label_roles(dirs, llm_backend) if llm_backend is not None else {
        'directories': len(dirs) - 1, 'labelled': 0, 'calls': 0, 'seconds': 0.0}
    return {
        'root': root.name,
        'root_path': str(root),
        'tree_hash': h,
        'generated_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
        'n_files': dirs['']['n_files'],
        'n_lines': dirs['']['n_lines'],
        'dirs': dirs,
        'roles': roles,
    }


def get_map(root: Path, cache_dir: Path, llm_backend) -> Dict[str, Any]:
    """The map for `root`: the cached one when the tree's shape matches,
    else a fresh one, cached under its hash in `cache_dir` (created if
    absent; must not lie inside the target)."""
    root = Path(root).resolve()
    entries = list_tree(root)
    h = tree_hash(entries)
    cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    cached = cache_dir / f"{h}.json"
    if cached.is_file():
        try:
            m = json.loads(cached.read_text(encoding='utf-8'))
            if m.get('tree_hash') == h:
                return m
        except Exception as e:                                   # noqa: BLE001
            logger.warning("repo_map: cached map %s unreadable (%s); "
                           "regenerating", cached, e)
    t0 = time.monotonic()
    m = generate(root, entries, h, llm_backend)
    cached.write_text(json.dumps(m, indent=1), encoding='utf-8')
    (cache_dir / f"{h}.md").write_text(render_map(m, full=True), encoding='utf-8')
    logger.info("repo_map: built %s (%d files, %s of %s directories "
                "labelled) in %.1fs", h, m['n_files'], m['roles']['labelled'],
                m['roles']['directories'], time.monotonic() - t0)
    return m


def _dir_line(path: str, d: Dict[str, Any], indent: int) -> str:
    counts = f"{d['n_files']} files, {d['n_lines']} lines"
    if d['n_binary']:
        counts += f", {d['n_binary']} binary"
    role = f"  role: {d['role']}" if d.get('role') else ""
    return f"{'  ' * indent}{path}/  ({counts}){role}"


def _file_line(f: List[Any], indent: int) -> str:
    name, size, lines = f[0], f[1], f[2]
    tail = f"{lines} lines" if lines is not None else "binary or oversized"
    return f"{'  ' * indent}{name}  {size:,} bytes, {tail}"


def render_map(m: Dict[str, Any], path: Optional[str] = None,
               full: bool = False) -> str:
    """Text for the subagent: the whole tree, files included, when it fits
    in _RENDER_MAX_LINES (observed 2026-09-11: a two-level directory view
    of a 76-file repository sent the subagent back to `list` for every
    directory); otherwise the root view (root files, top-level directories
    and their children) or one subtree (`path`) to the same cap. `full`
    renders everything for the record, uncapped."""
    if not full:
        whole = render_map(m, path, full=True)
        if whole.startswith('ERROR:') or whole.count('\n') <= _RENDER_MAX_LINES + 3:
            return whole
    dirs = m['dirs']
    head = [
        f"Repository map: {m['root']}  tree hash {m['tree_hash']}  "
        f"{m['n_files']} files, {m['n_lines']} lines  generated {m['generated_at']}",
        "Roles are a model's guess about how the tree is ORGANIZED, the kind "
        "of files a directory holds. They say nothing about what the code "
        "does; verify by reading. A role says where to look first, never "
        "where not to look: no directory is out of a search because of its "
        "role, and a claim that something is absent still needs every place "
        "it could occur. Counts are recursive. Ask `map` with `path` for one "
        "subtree.",
        "",
    ]
    start = (path or '').strip().strip('/')
    if start and start not in dirs:
        return f"ERROR: no directory {start!r} in the map"
    lines: List[str] = []

    def walk(p: str, depth: int, max_depth: Optional[int]) -> None:
        d = dirs[p]
        if p != start or start:
            lines.append(_dir_line(p, d, depth))
        file_depth = depth + (1 if (p != start or start) else 0)
        if p == start or full:
            for f in d['files']:
                lines.append(_file_line(f, file_depth))
        for name in d['subdirs']:
            child = f"{p}/{name}" if p else name
            if max_depth is None or depth + 1 <= max_depth:
                walk(child, depth + 1, max_depth)

    if start == '':
        for f in dirs['']['files']:
            lines.append(_file_line(f, 0))
        for name in dirs['']['subdirs']:
            walk(name, 0, None if full else 1)
    else:
        walk(start, 0, None if full else 2)
    if not full and len(lines) > _RENDER_MAX_LINES:
        omitted = len(lines) - _RENDER_MAX_LINES
        lines = lines[:_RENDER_MAX_LINES] + [
            f"…[{omitted} more entries not shown; ask `map` for a subtree]"]
    return "\n".join(head + lines)
