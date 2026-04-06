"""Obsidian Web Clipper sensor — detects new clippings in the vault."""

import os
import re
import yaml
from pathlib import Path

# Track files we've already reported (survives across poll cycles)
_seen_files: set = set()
_initialized = False


def _parse_frontmatter(text: str) -> dict:
    """Extract YAML frontmatter from markdown content."""
    m = re.match(r'^---\s*\n(.*?)\n---\s*\n', text, re.DOTALL)
    if not m:
        return {}
    try:
        return yaml.safe_load(m.group(1)) or {}
    except yaml.YAMLError:
        return {}


def _body_after_frontmatter(text: str) -> str:
    """Return markdown body after frontmatter."""
    m = re.match(r'^---\s*\n.*?\n---\s*\n', text, re.DOTALL)
    if m:
        return text[m.end():]
    return text


def run(context):
    global _seen_files, _initialized

    params = context.get('parameters', {})
    vault_path = params.get('vault_path', '/home/bruce/Documents/Obsidian Vault')
    clippings_folder = params.get('clippings_folder', 'Clippings')

    clippings_dir = Path(vault_path) / clippings_folder

    if not clippings_dir.is_dir():
        return {'status': 'nothing', 'content': '', 'metadata': {}}

    # Scan for .md files
    current_files = set()
    for f in clippings_dir.iterdir():
        if f.is_file() and f.suffix == '.md':
            current_files.add(str(f))

    # First run: seed seen set without reporting (avoid re-reporting old clips)
    if not _initialized:
        _seen_files = set(current_files)
        _initialized = True
        return {'status': 'nothing', 'content': '', 'metadata': {}}

    new_files = current_files - _seen_files
    _seen_files = current_files

    if not new_files:
        return {'status': 'nothing', 'content': '', 'metadata': {}}

    clippings = []
    for filepath in sorted(new_files):
        try:
            text = Path(filepath).read_text(encoding='utf-8', errors='replace')
        except OSError:
            continue

        fm = _parse_frontmatter(text)
        body = _body_after_frontmatter(text).strip()

        title = fm.get('title', '') or Path(filepath).stem
        source_url = fm.get('source', '') or fm.get('url', '') or ''
        tags = fm.get('tags', []) or []
        if isinstance(tags, str):
            tags = [t.strip() for t in tags.split(',')]
        created = fm.get('created', '') or fm.get('date', '') or ''

        # Content preview (first ~500 chars of body)
        preview = body[:500] if body else ''

        clippings.append({
            'title': title,
            'source_url': source_url,
            'tags': tags,
            'created': str(created),
            'preview': preview,
            'filename': Path(filepath).name,
        })

    n = len(clippings)
    return {
        'status': 'ok',
        'content': {
            'summary': f"User clipped {n} page{'s' if n != 1 else ''} to Obsidian",
            'data': clippings,
        },
        'metadata': {'clipping_count': n},
    }
