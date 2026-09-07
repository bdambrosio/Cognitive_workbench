"""The security audit's record: what the agent emits, what the runner checks.

THE RECORD IS DATA, NOT PROSE. Until 2026-09-07 the auditor wrote four prose
blocks for a reader, and the one run that completed (sec_2, 2026-08-29) was
conformant by every mechanical measure and unreadable by the person it was
for. The split the claims audit already made applies here: the agent fills
fields under METHOD.md; a renderer (report.py) writes the document under
REPORT.md; a query session (continuation.py) answers over both.

Two emissions, both schema-constrained calls outside the ReAct loop
(workflowsv2/emit.py):

  surface   — the attack surface, one element per way in (METHOD §3). The
              runner assigns the labels S1..Sn and freezes it.
  findings  — one finding per examined element that carries one, plus the
              limitations and the gap map (METHOD §§6, 7, 14).

The runner enforces what the method promises: every citation resolves into
the collection, every finding names a frozen label, the disposition is one of
§7's four, and the §10 conclusion is computed from the dispositions rather
than written by anyone.
"""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

#: METHOD §7, the whole vocabulary.
DISPOSITIONS: Tuple[str, ...] = ("confirmed", "mitigated", "unreachable", "uncertain")

#: METHOD §3, the kinds of element the enumeration test admits, with the
#: line of §3 each answers to. Handed to the surface call, because sec_3
#: (2026-09-07) typed a Wi-Fi interface, a token file and a package source
#: all as "service".
KIND_WORDS = {
    "socket": "a listening socket, per address and port",
    "container_port": "a published container port, and the container that publishes it",
    "service": "a service or unit that starts on boot and accepts input",
    "account": "an account that can authenticate, local or remote",
    "interface": "a physical or wireless interface: USB, Bluetooth, Wi-Fi, Ethernet",
    "extension_set": "a browser profile's installed extensions, taken together",
    "outbound_trust": "an auto-updater, a package source, a remote API whose response the host acts on",
    "credential": "a credential at rest, by location and permissions, never by contents",
}
KINDS: Tuple[str, ...] = tuple(KIND_WORDS)

#: The form of `identity`, per kind. METHOD §8 compares two reviews of one
#: system by identity, and the first pair (sec_3, sec_4, 2026-09-07) matched
#: nothing: one run wrote "sshd on port 22", the next "0.0.0.0:22/tcp". A
#: comparison needs a form, not a description; the description field is
#: where the words go.
IDENTITY_FORM = {
    "socket": "<proto>/<address>:<port>, e.g. tcp/0.0.0.0:22 or udp/[::]:5353",
    "container_port": "<container name>:<host port>, e.g. qwen38-flash-next:5000",
    "service": "<unit name>, e.g. bluetooth.service or apt-daily.timer",
    "account": "<user name>, e.g. bruce",
    "interface": "<interface name>, e.g. wlp68s0 or bluetooth",
    "extension_set": "<browser>/<profile>, e.g. chromium/Default",
    "outbound_trust": "<program or source name>, e.g. cloudflared or unattended-upgrades",
    "credential": "<file path>, e.g. /etc/cloudflared/token",
}


def identity_key(e: Dict[str, Any]) -> str:
    """What two reviews compare on: kind plus the identity, lowercased, with
    spaces removed."""
    return f"{e.get('kind')}:{re.sub(r'\s+', '', str(e.get('identity') or '').lower())}"

#: METHOD §4, the order of work, by what an attacker reaches first. An
#: element's `reach` is the runner's reading of where it sits; the conclusion
#: (§10) turns on the first two.
REACH: Tuple[str, ...] = ("beyond_lan", "lan_unauthenticated", "authenticated",
                          "at_rest", "outbound", "other")

#: METHOD §10, computed.
CONCLUSIONS = ("Exposed", "Weak", "Hardened for what was examined")

_CITE = {"type": "object",
         "properties": {"artifact": {"type": "string"},
                        "lines": {"type": "array", "items": {"type": "integer"},
                                  "minItems": 2, "maxItems": 2},
                        "quote": {"type": "string"}},
         "required": ["artifact", "lines", "quote"]}

_STEP = {"type": "object",
         "properties": {"cite": _CITE, "text": {"type": "string"}},
         "required": ["cite", "text"]}


def surface_schema() -> Dict[str, Any]:
    return {"type": "object",
            "properties": {
                "elements": {"type": "array", "items": {
                    "type": "object",
                    "properties": {
                        "kind": {"enum": list(KINDS)},
                        "identity": {"type": "string"},
                        "description": {"type": "string"},
                        "reach": {"enum": list(REACH)},
                        "cite": _CITE},
                    "required": ["kind", "identity", "description", "reach", "cite"]}},
                "unsettled_cases": {"type": "array", "items": {"type": "string"}},
                "not_completed": {"type": "string"}},
            "required": ["elements"]}


def findings_schema() -> Dict[str, Any]:
    return {"type": "object",
            "properties": {
                "findings": {"type": "array", "items": {
                    "type": "object",
                    "properties": {
                        "title": {"type": "string"},
                        "element": {"type": "string"},
                        "disposition": {"enum": list(DISPOSITIONS)},
                        "exposure": _STEP,
                        "path": {"type": "array", "items": _STEP},
                        "consequence": _STEP,
                        "assessment": {"type": "string"},
                        "remedy_locus": {"type": "string"}},
                    "required": ["title", "element", "disposition", "exposure",
                                 "path", "consequence", "assessment"]}},
                "examined": {"type": "array", "items": {"type": "string"}},
                "limitations": {"type": "array", "items": {
                    "type": "object",
                    "properties": {"text": {"type": "string"}, "cite": _CITE},
                    "required": ["text"]}},
                "gaps": {"type": "array", "items": {
                    "type": "object",
                    "properties": {"unknown": {"type": "string"},
                                   "why": {"type": "string"},
                                   "settles": {"type": "string"},
                                   "element": {"type": "string"}},
                    "required": ["unknown", "why", "settles", "element"]}},
                "not_completed": {"type": "string"}},
            "required": ["findings", "examined", "limitations", "gaps"]}


# ---- the frozen surface --------------------------------------------------------

def freeze(elements: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Label the elements S1..Sn in the order emitted. The label is the
    report's name for an element and nothing else (METHOD §3)."""
    out = []
    for i, e in enumerate(elements, 1):
        row = dict(e)
        row["label"] = f"S{i}"
        out.append(row)
    return out


def surface_lines(frozen: Sequence[Dict[str, Any]]) -> str:
    """The frozen surface as the agent is handed it back."""
    rows = []
    for e in frozen:
        c = e.get("cite") or {}
        rows.append(f"{e['label']}  [{e.get('kind')}, {e.get('reach')}]  "
                    f"{e.get('identity')} — {e.get('description')}  "
                    f"(collection/{c.get('artifact')}:{_lines(c.get('lines'))})")
    return "\n".join(rows)


def _lines(lines: Any) -> str:
    if isinstance(lines, list) and len(lines) == 2:
        return str(lines[0]) if lines[0] == lines[1] else f"{lines[0]}-{lines[1]}"
    return "?"


# ---- citations ------------------------------------------------------------------

def _flat(s: str) -> str:
    return " ".join((s or "").split())


def _alnum(s: str) -> str:
    """Letters and digits only. A quote that differs from its line in
    punctuation — a doubled parenthesis dropped, a tab collapsed — still
    names text that exists; sec_3 (2026-09-07) lost eight citations that
    way, all at the right line."""
    return re.sub(r"[^A-Za-z0-9]+", "", s or "")


LOOSE = "quote differs from the line in punctuation or spacing"
RELOCATED = "quote found at other lines; the citation's lines were corrected"


def resolve_cite(cite: Dict[str, Any], collection: Path) -> Optional[str]:
    """None when the citation resolves exactly; LOOSE when the quote's first
    line matches the cited lines once punctuation and spacing are ignored;
    RELOCATED when the quote is real text at other lines of the same file,
    in which case `cite["lines"]` is corrected in place; else one line
    saying why not. Callers treat LOOSE and RELOCATED as resolved and say
    so beside the finding.

    The agent reads the collection through evidence requests that show it
    slices, so its line numbers are reconstructed and its quotes are
    retyped; sec_3 (2026-09-07) had eight of thirteen findings fail on one
    or the other with the text plainly present. The runner corrects what a
    file operation can correct and records that it did. What it does not
    do is judge: a resolved citation is text that exists, not text that
    supports the finding (METHOD §5)."""
    if not isinstance(cite, dict):
        return "citation is not an object"
    art = str(cite.get("artifact") or "").strip()
    art = re.sub(r"^collection/", "", art)
    if not art or "/" in art or art.startswith("."):
        return f"artifact {art!r} is not a collection file name"
    f = collection / art
    if not f.is_file():
        return f"collection/{art} does not exist"
    try:
        text = f.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as e:
        return f"collection/{art} unreadable: {e}"
    lines = cite.get("lines")
    if not (isinstance(lines, list) and len(lines) == 2
            and all(isinstance(x, int) for x in lines)):
        return "lines is not [start, end]"
    a, b = lines
    if a < 1 or b < a or b > len(text):
        return f"lines {a}-{b} outside collection/{art} ({len(text)} lines)"
    q = _flat(cite.get("quote"))
    if not q:
        return "empty quote"
    span = _flat(" ".join(text[a - 1:b]))
    if q in span:
        return None
    first = _alnum(str(cite.get("quote") or "").strip().splitlines()[0])
    if first and first in _alnum(span):
        return LOOSE
    # Real text, wrong lines: find the first line of the quote in the file.
    hits = [i for i, line in enumerate(text, 1) if first and first in _alnum(line)]
    if len(hits) == 1:
        n = max(1, len(str(cite.get("quote") or "").strip().splitlines()))
        cite["lines"] = [hits[0], min(len(text), hits[0] + n - 1)]
        return RELOCATED
    if len(hits) > 1:
        return (f"quote not found in collection/{art}:{a}-{b}, and its first line "
                f"appears at {len(hits)} other places")
    return f"quote not found in collection/{art}:{a}-{b}"


def _cites_of(finding: Dict[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    out = []
    ex = finding.get("exposure") or {}
    if ex.get("cite") is not None:
        out.append(("exposure", ex["cite"]))
    for i, step in enumerate(finding.get("path") or []):
        if isinstance(step, dict) and step.get("cite") is not None:
            out.append((f"path[{i}]", step["cite"]))
    cq = finding.get("consequence") or {}
    if cq.get("cite") is not None:
        out.append(("consequence", cq["cite"]))
    return out


# ---- checks ----------------------------------------------------------------------

def check_surface(obj: Optional[Dict[str, Any]], collection: Path) -> Dict[str, Any]:
    problems: List[str] = []
    if not isinstance(obj, dict):
        return {"ok": False, "problems": ["no parseable surface"]}
    els = obj.get("elements") or []
    if not els and not obj.get("not_completed"):
        problems.append("no elements and no not_completed")
    seen = set()
    for i, e in enumerate(els, 1):
        w = f"element {i}"
        if e.get("kind") not in KINDS:
            problems.append(f"{w}: kind {e.get('kind')!r} not in METHOD §3")
        if e.get("reach") not in REACH:
            problems.append(f"{w}: reach {e.get('reach')!r} not in METHOD §4")
        ident = _flat(e.get("identity"))
        if not ident:
            problems.append(f"{w}: empty identity")
        elif ident in seen:
            problems.append(f"{w}: identity {ident!r} repeats an earlier element "
                            f"(one element is one way in, METHOD §3)")
        seen.add(ident)
        bad = resolve_cite(e.get("cite"), collection)
        if bad and bad not in (LOOSE, RELOCATED):
            problems.append(f"{w}: {bad}")
    return {"ok": not problems, "problems": problems}


def check_findings(obj: Optional[Dict[str, Any]], frozen: Sequence[Dict[str, Any]],
                   collection: Path) -> Dict[str, Any]:
    """METHOD §§5-7, 14 as file operations. Citation problems are also
    written onto each finding as `citation_problems`, so the renderer can
    say so beside it."""
    problems: List[str] = []
    if not isinstance(obj, dict):
        return {"ok": False, "problems": ["no parseable findings"], "citations": {}}
    labels = {e["label"] for e in frozen}
    gaps_by_el = {g.get("element") for g in (obj.get("gaps") or []) if g.get("element")}
    cites: Dict[str, List[str]] = {}
    for i, f in enumerate(obj.get("findings") or [], 1):
        w = f"finding {i} ({f.get('title', '')[:40]})"
        if f.get("element") not in labels:
            problems.append(f"{w}: element {f.get('element')!r} is not on the frozen surface")
        d = f.get("disposition")
        if d not in DISPOSITIONS:
            problems.append(f"{w}: disposition {d!r} not in METHOD §7")
        if d == "uncertain" and f.get("element") not in gaps_by_el:
            problems.append(f"{w}: [uncertain] with no gap naming what would settle it (METHOD §7)")
        if d == "mitigated" and not (f.get("path") or []):
            problems.append(f"{w}: [mitigated] cites no control in its path (METHOD §7)")
        bad, loose, moved = [], [], []
        for where, c in _cites_of(f):
            r = resolve_cite(c, collection)
            if r == LOOSE:
                loose.append(where)
            elif r == RELOCATED:
                moved.append(where)
            elif r:
                bad.append(f"{where}: {r}")
        f["citation_problems"] = bad
        f["citation_loose"] = loose
        f["citation_relocated"] = moved
        if bad:
            cites[w] = bad
            problems.extend(f"{w}: {b}" for b in bad)
    for i, g in enumerate(obj.get("gaps") or [], 1):
        if not _flat(g.get("settles")):
            problems.append(f"gap {i}: names nothing that would settle it (METHOD §14)")
    for lab in obj.get("examined") or []:
        if lab not in labels:
            problems.append(f"examined: {lab!r} is not on the frozen surface")
    for i, lim in enumerate(obj.get("limitations") or [], 1):
        if lim.get("cite") is not None:
            r = resolve_cite(lim["cite"], collection)
            if r and r not in (LOOSE, RELOCATED):
                problems.append(f"limitation {i}: {r}")
    return {"ok": not problems, "problems": problems, "citations": cites}


# ---- the conclusion, computed --------------------------------------------------

def conclusion(findings: Sequence[Dict[str, Any]], frozen: Sequence[Dict[str, Any]],
               outcomes: Dict[str, Any]) -> Dict[str, Any]:
    """METHOD §10, from the dispositions and the probe outcomes. Never
    written by the agent."""
    reach = {e["label"]: e.get("reach") for e in frozen}
    confirmed = [f for f in findings if f.get("disposition") == "confirmed"]
    front = [f for f in confirmed
             if reach.get(f.get("element")) in ("beyond_lan", "lan_unauthenticated")]
    if front:
        word = "Exposed"
    elif confirmed:
        word = "Weak"
    else:
        word = "Hardened for what was examined"
    withheld = sorted(k for k, v in (outcomes or {}).items()
                      if v.get("outcome") == "unauthorised")
    return {"conclusion": word, "confirmed": len(confirmed),
            "confirmed_without_credentials": len(front),
            "withheld_grants": withheld,
            "caveat_required": word.startswith("Hardened") and bool(withheld)}


# ---- run layout ------------------------------------------------------------------

def load_run(run_dir: Path) -> Dict[str, Any]:
    """Everything the document and the query stages read, from one run."""
    run_dir = Path(run_dir)
    def _j(name):
        p = run_dir / name
        return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else None
    outcomes = _j("collection/outcomes.json") or {}
    surface = _j("surface.json") or {}
    findings = _j("findings.json") or {}
    checks = _j("checks.json") or {}
    meta = _j("run_meta.json") or {}
    frozen = surface.get("elements") or []
    return {"dir": run_dir, "collection": run_dir / "collection",
            "outcomes": outcomes, "surface": surface, "frozen": frozen,
            "findings": findings, "checks": checks, "meta": meta,
            "conclusion": conclusion(findings.get("findings") or [], frozen, outcomes)}


def previous_run(run_dir: Path) -> Optional[Path]:
    """The run before this one in the same engagement that produced a
    record, for METHOD §8."""
    runs = sorted(p for p in Path(run_dir).parent.iterdir()
                  if p.is_dir() and (p / "findings.json").is_file())
    prior = [p for p in runs if p.name < Path(run_dir).name]
    return prior[-1] if prior else None
