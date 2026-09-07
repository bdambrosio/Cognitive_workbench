"""The engagement's materials as the site shows and changes them: the
files under the target, uploaded, arranged and deleted by the seller or the
practice, and marked by the practice as claim sources or as excluded from
evidence.

WHERE. Everything here acts on `engagement_state.target_dir(eng_dir)`, which
is the engagement's own `target/` unless engagement.yaml points elsewhere.
A target outside the engagement directory (a fixture, a checkout the
practice manages by hand) is shown but never written from the site.

WHEN. Writes are refused while a job holds the engagement (the runner reads
the files live) and, for the seller, once the practice has marked the
materials ready: the record a run pins must not move under it. The practice
may write at any time no job is running, and re-marks the stage if it does.

PATHS. Every path a request names is relative to the target root, with no
empty, `.` or `..` segments, and must resolve to a place under the root
(a symlink that points out is refused). The listing shows every entry,
dotfiles included: `.github/workflows` is evidence.
"""
from __future__ import annotations

import logging
import re
import shutil
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

from workflowsv2 import engagement_state as state

logger = logging.getLogger("client_ui.materials")

_SEGMENT = re.compile(r"[^A-Za-z0-9._ +@()\[\]-]+")


class Refused(Exception):
    """A request the materials page cannot carry out; the message is shown."""


def is_own_target(eng_dir: Path) -> bool:
    """True when the materials live in the engagement's own target/: the
    default, whether or not the directory exists yet."""
    t = str(state._engagement_yaml(eng_dir).get("target") or state.TARGET)
    return t == state.TARGET or (eng_dir / t).resolve() == (eng_dir / state.TARGET).resolve()


def root(eng_dir: Path) -> Path:
    """The materials root. `engagement_state.target_dir` falls back to a
    repo path while the engagement's own target/ does not exist yet; the
    page never does."""
    return (eng_dir / state.TARGET) if is_own_target(eng_dir) else state.target_dir(eng_dir)


def clean_segments(rel: str, sanitize: bool = False) -> List[str]:
    """The segments of a relative path as the page names it. A segment with
    a character outside the accepted set is refused, or with `sanitize`
    (an upload's own file name) rewritten with `_` in its place."""
    parts = [p for p in re.split(r"[\\/]+", (rel or "").strip()) if p]
    out = []
    for p in parts:
        if p in (".", ".."):
            raise Refused(f"'{p}' is not a name the materials page accepts")
        q = _SEGMENT.sub("_", p).strip(" .") if sanitize else p
        if not q or _SEGMENT.sub("", q) != q or q != q.strip(" ."):
            raise Refused(f"'{p}' is not a name the materials page accepts "
                          f"(letters, digits, space and . _ - + @ ( ) [ ])")
        out.append(q)
    return out


def resolve(eng_dir: Path, rel: str) -> Tuple[Path, str]:
    """The absolute path of `rel` under the target root and its clean
    relative form. Raises Refused when it would leave the root."""
    r = root(eng_dir)
    parts = clean_segments(rel)
    p = r.joinpath(*parts) if parts else r
    try:
        inside = p.resolve().is_relative_to(r.resolve())
    except OSError as e:
        raise Refused(f"cannot resolve '{rel}': {e}")
    if not inside:
        raise Refused(f"'{rel}' is outside the materials")
    return p, "/".join(parts)


# ---- the write gate ------------------------------------------------------------

def writable(eng_dir: Path, roles: Iterable[str]) -> Tuple[bool, str]:
    """Whether these roles may change the materials now, and if not, why."""
    roles = set(roles)
    if not is_own_target(eng_dir):
        return False, ("the materials for this engagement are held outside its "
                       "directory and are managed by the practice by hand")
    job = state.running_job(eng_dir)
    if job:
        return False, f"a {job.get('kind')} job is running; the materials are locked until it ends"
    if "practice" not in roles and state.stage_value(eng_dir, "materials") == "ready":
        return False, ("the practice has marked the materials ready; write to the "
                      "practice if something must change")
    return True, ""


def _check_writable(eng_dir: Path, roles: Iterable[str]) -> None:
    ok, why = writable(eng_dir, roles)
    if not ok:
        raise Refused(why)


# ---- reading -------------------------------------------------------------------

def _marks(eng_dir: Path) -> Tuple[Set[str], Set[str]]:
    return set(state.claim_sources(eng_dir)), set(state.evidence_excludes(eng_dir))


def listing(eng_dir: Path, rel: str, roles: Iterable[str]) -> Dict[str, Any]:
    """One directory of the materials, as the page shows it."""
    p, clean = resolve(eng_dir, rel)
    if not p.is_dir() and (clean or p.exists()):
        raise Refused(f"'{clean or '/'}' is not a folder in the materials")
    sources, excludes = _marks(eng_dir)
    entries = []
    for child in sorted(p.iterdir(), key=lambda c: (not c.is_dir(), c.name.lower())) if p.is_dir() else []:
        crel = f"{clean}/{child.name}" if clean else child.name
        under_excluded = any(crel == x or crel.startswith(x.rstrip("/") + "/") for x in excludes)
        st = child.stat()
        entries.append({"name": child.name, "path": crel, "dir": child.is_dir(),
                        "size": 0 if child.is_dir() else st.st_size,
                        "mtime": int(st.st_mtime),
                        "claim_source": crel in sources,
                        "excluded": crel in excludes,
                        "under_excluded": under_excluded and crel not in excludes})
    ok, why = writable(eng_dir, roles)
    return {"path": clean, "entries": entries, "writable": ok, "why_not_writable": why,
            "own_target": is_own_target(eng_dir), "root_exists": root(eng_dir).is_dir(),
            "claim_sources": sorted(sources), "evidence_excludes": sorted(excludes),
            "excludes_explicit": "evidence_excludes" in state._engagement_yaml(eng_dir),
            "materials": state.stage_value(eng_dir, "materials"),
            "count": sum(1 for _ in root(eng_dir).rglob("*") if _.is_file()) if root(eng_dir).is_dir() else 0}


def file_path(eng_dir: Path, rel: str) -> Path:
    p, clean = resolve(eng_dir, rel)
    if not p.is_file():
        raise Refused(f"'{clean}' is not a file in the materials")
    return p


# ---- writing -------------------------------------------------------------------

def save(eng_dir: Path, roles: Iterable[str], into: str, rel_name: str, data: bytes) -> str:
    """Write one uploaded file at `into`/`rel_name` (rel_name may carry the
    folders of a folder upload), replacing a file of that name. Returns the
    clean relative path."""
    _check_writable(eng_dir, roles)
    parts = clean_segments(into) + clean_segments(rel_name, sanitize=True)
    if not parts:
        raise Refused("an upload needs a file name")
    p, clean = resolve(eng_dir, "/".join(parts))
    if p.is_dir():
        raise Refused(f"'{clean}' is a folder")
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(data)
    logger.info("materials %s: wrote %s (%d bytes)", eng_dir.name, clean, len(data))
    return clean


def mkdir(eng_dir: Path, roles: Iterable[str], rel: str) -> str:
    _check_writable(eng_dir, roles)
    p, clean = resolve(eng_dir, rel)
    if not clean:
        raise Refused("a folder needs a name")
    if p.exists() and not p.is_dir():
        raise Refused(f"'{clean}' is a file")
    p.mkdir(parents=True, exist_ok=True)
    return clean


def delete(eng_dir: Path, roles: Iterable[str], rel: str) -> str:
    """Remove a file, or a folder and everything under it. Marks naming it
    are removed from engagement.yaml too."""
    _check_writable(eng_dir, roles)
    p, clean = resolve(eng_dir, rel)
    if not clean:
        raise Refused("the materials root cannot be deleted")
    if not p.exists():
        raise Refused(f"'{clean}' is not in the materials")
    if p.is_dir() and not p.is_symlink():
        shutil.rmtree(p)
    else:
        p.unlink()
    sources, excludes = _marks(eng_dir)
    gone = lambda x: x == clean or x.startswith(clean + "/")           # noqa: E731
    fields: Dict[str, Any] = {}
    if any(gone(x) for x in sources):
        fields["claim_sources"] = [x for x in state.claim_sources(eng_dir) if not gone(x)]
    if any(gone(x) for x in excludes):
        fields["evidence_excludes"] = [x for x in state.evidence_excludes(eng_dir) if not gone(x)]
    if fields:
        state.update_engagement(eng_dir, **fields)
    logger.info("materials %s: deleted %s", eng_dir.name, clean)
    return clean


# ---- marks (the practice) ----------------------------------------------------------

def mark(eng_dir: Path, rel: str, claim_source: Optional[bool] = None,
         excluded: Optional[bool] = None) -> Dict[str, List[str]]:
    """Set or clear the marks on one path. A claim source is one document,
    so marking a folder marks the files directly in it; an exclusion names
    the folder itself and covers everything under it. Marks may change at
    any time: the runner reads them when a job starts."""
    p, clean = resolve(eng_dir, rel)
    if not p.exists():
        raise Refused(f"'{clean}' is not in the materials")
    fields: Dict[str, Any] = {}
    if claim_source is not None:
        names = ([f"{clean}/{c.name}" if clean else c.name
                  for c in sorted(p.iterdir()) if c.is_file()] if p.is_dir() else [clean])
        cur = state.claim_sources(eng_dir)
        if claim_source:
            fields["claim_sources"] = cur + [n for n in names if n not in cur]
        else:
            fields["claim_sources"] = [x for x in cur if x not in names]
    if excluded is not None:
        cur = state.evidence_excludes(eng_dir)
        if excluded:
            fields["evidence_excludes"] = cur + ([clean] if clean not in cur else [])
        else:
            fields["evidence_excludes"] = [x for x in cur if x != clean]
    if fields:
        state.update_engagement(eng_dir, **fields)
    return {"claim_sources": state.claim_sources(eng_dir),
            "evidence_excludes": state.evidence_excludes(eng_dir)}
