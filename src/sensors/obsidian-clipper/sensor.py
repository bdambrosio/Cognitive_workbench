"""Obsidian Web Clipper sensor — detects new clippings in the vault.

Fires when the user clips a page into the vault's Clippings folder, so the
agent hears about it as an event rather than having to go looking for it.

Per-character seen-sets live in module state and are deliberately not
persisted: a fresh process seeds its baseline silently on the first poll,
which is what keeps a restart from re-reporting the whole folder as new
clippings.

Emits a prose situation report, not structured content: non-tick sensors
reach the chat loop through the `text` field of the sense_data envelope,
and a {'summary': ..., 'data': ...} payload has no `text` for it to read.
"""

import logging
import re
import yaml
from pathlib import Path

logger = logging.getLogger(__name__)

# character -> set of clipping paths already reported. Absent means the
# baseline has not been seeded yet for this process.
_seen: dict = {}

_NOTHING = {'status': 'nothing', 'content': '', 'metadata': {}}

_PREVIEW_CHARS = 400


def _parse_frontmatter(text: str, filepath: str) -> dict:
    """Extract YAML frontmatter from markdown content."""
    m = re.match(r'^---\s*\n(.*?)\n---\s*\n', text, re.DOTALL)
    if not m:
        return {}
    try:
        return yaml.safe_load(m.group(1)) or {}
    except yaml.YAMLError as e:
        logger.warning(f"obsidian-clipper: bad frontmatter in {filepath}: {e}")
        return {}


def _body_after_frontmatter(text: str) -> str:
    """Return markdown body after frontmatter."""
    m = re.match(r'^---\s*\n.*?\n---\s*\n', text, re.DOTALL)
    if m:
        return text[m.end():]
    return text


def _describe(clippings: list, folder: str) -> str:
    """A self-contained report of what was clipped.

    Non-tick sensors arrive as user-like turns, so this has to read as an
    event the agent is noticing rather than as something anyone said. It
    carries enough of each clipping — and the vault path — that the woken
    turn can decide whether to care without first spending an iteration
    reading the file.
    """
    n = len(clippings)
    lines = [f"The user clipped {n} page{'s' if n != 1 else ''} "
             f"into the Obsidian vault."]
    lines.append("")

    for c in clippings:
        lines.append(f"  \"{c['title']}\"")
        if c['source_url']:
            lines.append(f"    from {c['source_url']}")
        if c['tags']:
            lines.append(f"    tags: {', '.join(str(t) for t in c['tags'])}")
        lines.append(f"    readable at {folder}/{c['filename']}")
        if c['preview']:
            lines.append(f"    {c['preview']}")
        lines.append("")

    lines.append("Nobody has said anything — you noticed this yourself. "
                 "Clipping something is not a request to discuss it. "
                 "Reply only if it genuinely connects to what you and the "
                 "user are working on; otherwise stay silent.")
    return "\n".join(lines)


def run(context):
    me = context.get('character_name') or ''
    if not me:
        return _NOTHING

    params = context.get('parameters', {})
    vault_path = params.get('vault_path', '/home/bruce/Documents/Obsidian Vault')
    clippings_folder = params.get('clippings_folder', 'Clippings')

    clippings_dir = Path(vault_path) / clippings_folder

    if not clippings_dir.is_dir():
        logger.debug(f"obsidian-clipper: no clippings dir at {clippings_dir}")
        return _NOTHING

    # Scan for .md files
    current_files = set()
    try:
        for f in clippings_dir.iterdir():
            if f.is_file() and f.suffix == '.md':
                current_files.add(str(f))
    except OSError as e:
        logger.warning(f"obsidian-clipper: cannot scan {clippings_dir}: {e}")
        return _NOTHING

    prior = _seen.get(me)
    if prior is None:
        # Cold start: seed the baseline, say nothing. What was already
        # clipped is not news; only what arrives from here is.
        _seen[me] = current_files
        logger.info(f"obsidian-clipper[{me}]: baseline seeded "
                    f"({len(current_files)} clippings), no event emitted")
        return _NOTHING

    new_files = current_files - prior
    _seen[me] = current_files

    if not new_files:
        return _NOTHING

    clippings = []
    for filepath in sorted(new_files):
        try:
            text = Path(filepath).read_text(encoding='utf-8', errors='replace')
        except OSError as e:
            logger.warning(f"obsidian-clipper: cannot read {filepath}: {e}")
            continue

        fm = _parse_frontmatter(text, filepath)
        body = _body_after_frontmatter(text).strip()

        title = fm.get('title', '') or Path(filepath).stem
        source_url = fm.get('source', '') or fm.get('url', '') or ''
        tags = fm.get('tags', []) or []
        if isinstance(tags, str):
            tags = [t.strip() for t in tags.split(',')]
        created = fm.get('created', '') or fm.get('date', '') or ''

        clippings.append({
            'title': title,
            'source_url': source_url,
            'tags': tags,
            'created': str(created),
            'preview': body[:_PREVIEW_CHARS] if body else '',
            'filename': Path(filepath).name,
        })

    if not clippings:
        return _NOTHING

    n = len(clippings)
    logger.info(f"obsidian-clipper[{me}]: {n} new clipping(s): "
                f"{[c['filename'] for c in clippings]}")
    return {
        'status': 'ok',
        'content': _describe(clippings, clippings_folder),
        'metadata': {'clipping_count': n,
                     'titles': [c['title'] for c in clippings]},
    }
