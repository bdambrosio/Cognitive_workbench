"""doc-read — read a PDF that is already on this machine.

The gap this closes, observed 2026-08-12: an OpenReview link 403'd, the
user downloaded the PDF instead, and there was no way to read it.
`inspect_external` is text-only, `exec-script` is broken from the chat
path, and `fetch-text` speaks HTTP, not file paths. The extraction logic
already existed in utils/doc_extract and utils/grobid — it simply was not
reachable as an action. What followed was a review of a different paper
found by title similarity, so the cost of the gap was not a missing
feature but a confident wrong answer.

Presentation deliberately mirrors fetch-text on a paper: section index
first, named section after. Two readers that behave differently on the
same document teach an agent that document reading is unpredictable.
"""
import logging
import os
import sys
from pathlib import Path

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.doc_sections import resolve_section  # noqa: E402
from utils.tool_helpers import run_tool  # noqa: E402

_log = logging.getLogger(__name__)

# Where a local read is allowed to look. A document reader that accepts
# any absolute path is a file-exfiltration primitive in an agent whose
# turns can be sourced by RSS feeds and mail — the roots below are where
# documents actually arrive, and nothing else is readable through here.
_ROOTS = [Path.home() / 'Downloads', Path(_SRC).parent]

_OBS_CAP = 8000            # matches fetch-text's ReAct observation cap
_INLINE_ABSTRACT_CAP = 2500


def _resolve_path(raw: str):
    """Absolute path inside an allowed root, or (None, reason).

    Resolves symlinks before the containment check so a link planted
    inside a root cannot reach outside it."""
    p = Path(raw).expanduser()
    candidates = [p] if p.is_absolute() else [r / p for r in _ROOTS]
    for cand in candidates:
        try:
            real = cand.resolve()
        except OSError as e:
            _log.debug(f"doc-read: cannot resolve {cand}: {e}")
            continue
        if not any(real == r.resolve() or r.resolve() in real.parents
                   for r in _ROOTS):
            continue
        if real.is_file():
            return real, None
    roots = ', '.join(str(r) for r in _ROOTS)
    if any((r / p).exists() for r in _ROOTS) or p.exists():
        return None, (f"{raw!r} is outside the folders I can read. "
                      f"Readable roots: {roots}")
    return None, (f"no such file: {raw!r}. I look in {roots} — if it is "
                  f"elsewhere, give me the full path or move it there.")


def _sections_from_chunks(chunks):
    """GROBID returns [(section_title, text), ...] in document order and
    may split one section across several chunks. Rejoin by title, keeping
    first-appearance order so the index reads like the paper."""
    order, bodies = [], {}
    for name, text in chunks or []:
        name = (name or '').strip() or '(untitled section)'
        if name not in bodies:
            order.append(name)
            bodies[name] = []
        bodies[name].append(text or '')
    return order, {n: '\n\n'.join(bodies[n]).strip() for n in order}


def _impl(args):
    raw = str(args.get('path') or '').strip()
    if not raw:
        return {'status': 'error',
                'text': 'doc-read needs a `path` — which file to read'}
    path, err = _resolve_path(raw)
    if err:
        return {'status': 'error', 'text': err}
    if path.suffix.lower() != '.pdf':
        return {'status': 'error',
                'text': f"doc-read handles PDFs; {path.name} is "
                        f"{path.suffix or 'extensionless'}"}

    parsed = None
    try:
        from utils.grobid import parse_pdf_grobid
        parsed = parse_pdf_grobid(pdf_filepath=str(path))
    except Exception as e:
        # GROBID is optional structure, not the text itself. Falling back
        # to flat pages is worse to navigate but still readable.
        _log.info(f"doc-read: GROBID unavailable for {path.name} ({e}); "
                  f"falling back to flat extraction")

    order, bodies = _sections_from_chunks((parsed or {}).get('chunks'))
    requested = args.get('section')

    if not order:
        from utils.doc_extract import extract_to_markdown
        text = extract_to_markdown(path) or ''
        if not text.strip():
            return {'status': 'empty',
                    'text': f"no text extracted from {path.name} — it may "
                            f"be a scan with no text layer"}
        note = (f"[{path.name}: no section structure available, so this is "
                f"flat page text]\n\n")
        if len(text) > _OBS_CAP:
            text = text[:_OBS_CAP].rstrip() + \
                f"\n…[truncated at {_OBS_CAP} chars of a longer document]"
        return {'status': 'ok', 'text': note + text}

    if isinstance(requested, str) and requested.strip():
        name = resolve_section(requested, order)
        if name is None:
            return {'status': 'error',
                    'text': (f"no section named {requested!r} in this "
                             f"document. Available sections: "
                             + ', '.join(order))}
        body = bodies[name]
        if len(body) > _OBS_CAP:
            body = (body[:_OBS_CAP].rstrip()
                    + f"\n…[section truncated at {_OBS_CAP} chars]")
        return {'status': 'ok', 'text': f"## {name}\n\n{body}"}

    title = (parsed or {}).get('title') or path.name
    abstract = ((parsed or {}).get('abstract') or '').strip()
    total = sum(len(bodies[n]) for n in order)
    lines = [f"# {title}", ""]
    if abstract:
        lines += ["## Abstract", "",
                  abstract[:_INLINE_ABSTRACT_CAP]
                  + ("…" if len(abstract) > _INLINE_ABSTRACT_CAP else ""), ""]
    lines.append(f"Read from {path}. Section index ({len(order)} sections, "
                 f"{total:,} chars of body text — not shown). Request one "
                 f"with the same path plus `\"section\": \"<name>\"`:")
    lines += [f"  - {n}  ({len(bodies[n]):,} chars)" for n in order]
    return {'status': 'ok', 'text': '\n'.join(lines)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    return run_tool(lambda: _impl(args or {}), logger or _log)
