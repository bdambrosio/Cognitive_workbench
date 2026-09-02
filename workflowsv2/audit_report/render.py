"""The client's document, assembled from the record. Mechanical.

Reads `merged.json` and `materiality.json` from a materiality output directory
and writes the document REPORT.md §6 describes. The six passages the agent
writes are placed where §6 names them; without them the document still
assembles, with a marker where each passage would go, and that is what the
agent is shown.

THREE CLASSES, AND ONLY ONE IS A MARK AGAINST THE SELLER. Findings with a
verdict about the claim are what the audit showed, ordered by materiality.
`unverifiable` findings are unsettled, ordered by exposure, and among them
those the searches named files for that nobody opened are set apart as not
examined. Nothing here re-judges: every verdict, gap, rating and basis is the
record's, copied.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from workflowsv2.audit_materiality import schemas as ms
from workflowsv2.claims_audit.schemas import NOT_EXAMINED

#: Highest consequence first.
_ORDER = {m: i for i, m in enumerate(reversed(ms.MATERIALITY))}

VERDICT_WORDS = {
    "contradicted": "contradicted — the materials show otherwise",
    "partial": "partly true — a named part of the claim is not borne out",
    "real_with_caveat": "true, with something the buyer must know",
    "unverifiable": "unsettled — the materials cannot settle it",
    "real": "holds",
}

DISPOSITION_WORDS = {
    "not_in_the_materials": "material of the right kind was supplied and "
                            "none of it settles the claim",
    "present_but_not_readable": "the material is supplied and cannot be read "
                                "as given",
    "outside_the_materials": "the kind of material needed was not supplied",
    NOT_EXAMINED: "the searches named files that were not opened",
}

RATING_WORDS = {
    "not_material": "would change neither the price, the terms, nor the "
                    "decision to close",
    "material": "would change the price or the terms",
    "decisive": "on its own would change the decision to close",
}


def load(merged_dir: Path) -> Dict[str, Any]:
    merged_dir = Path(merged_dir)
    merged = json.loads((merged_dir / "merged.json").read_text(encoding="utf-8"))
    ratings = json.loads((merged_dir / "materiality.json").read_text(encoding="utf-8"))
    return {"merged": merged, "ratings": ratings, "dir": merged_dir}


def _key(f: Dict[str, Any]) -> str:
    return f"{f.get('claim_source')}#{f.get('claim_id')}"


def classify(merged: Dict[str, Any]) -> Dict[str, List[Dict[str, Any]]]:
    """shown / unsettled / not_examined / holds, from the verdicts."""
    out: Dict[str, List[Dict[str, Any]]] = {
        "shown": [], "unsettled": [], "not_examined": [], "holds": []}
    for f in merged.get("findings") or []:
        adj = f.get("adjudication") or {}
        if ms.rateable(f):
            out["shown"].append(f)
        elif ms.exposable(f):
            if adj.get("unresolved_because") == NOT_EXAMINED:
                out["not_examined"].append(f)
            else:
                out["unsettled"].append(f)
        else:
            out["holds"].append(f)
    return out


def _ratings_by_key(ratings: Dict[str, Any], array: str
                    ) -> Dict[str, Dict[str, Any]]:
    return {f"{r.get('claim_source')}#{r.get('claim_id')}": r
            for r in ratings.get(array) or []}


def _ordered(findings: Sequence[Dict[str, Any]], rated: Dict[str, Dict[str, Any]],
             field: str) -> List[Dict[str, Any]]:
    def rank(f):
        r = rated.get(_key(f))
        return (_ORDER.get(r.get(field), 99) if r else 99,
                f.get("claim_source") or "", f.get("claim_id") or 0)
    return sorted(findings, key=rank)


def _lines(lines: Any) -> str:
    if isinstance(lines, list) and len(lines) == 2:
        return f"line {lines[0]}" if lines[0] == lines[1] \
            else f"lines {lines[0]}–{lines[1]}"
    return ""


def _evidence(items: Sequence[Dict[str, Any]]) -> List[str]:
    out = []
    for e in items or []:
        if not isinstance(e, dict):
            continue
        form = e.get("form")
        if form == "citation":
            out.append(f"- `{e.get('document')}`, {_lines(e.get('lines'))}: "
                       f"\"{(e.get('quote') or '').strip()}\" — "
                       f"{(e.get('shows') or '').strip()}")
        elif form == "derived":
            basis = "; ".join(f"`{b.get('document')}` {_lines(b.get('lines'))}"
                              for b in (e.get("basis") or []) if isinstance(b, dict))
            out.append(f"- derived from {basis}: {(e.get('derivation') or '').strip()} "
                       f"— {(e.get('consequence') or '').strip()}")
        elif form == "search":
            cands = ", ".join(f"`{c}`" for c in (e.get("candidates") or []))
            out.append(f"- searched ({e.get('kind')}): "
                       f"{(e.get('performed') or '').strip()} — "
                       f"{(e.get('result') or '').strip()}"
                       + (f" Files named: {cands}." if cands else ""))
    return out


def _review_line(f: Dict[str, Any]) -> str:
    rv = f.get("review") or {}
    line = {"holds": "the review upheld this finding",
            "does_not_hold": "the review did not uphold this finding",
            "unreviewed": "not reviewed"}.get(rv.get("outcome"), "not reviewed")
    if rv.get("adverse_observations"):
        line += " (" + ", ".join(rv["adverse_observations"]) + ")"
    if f.get("citation_problems"):
        line += f"; {len(f['citation_problems'])} citation problem(s) recorded"
    return line


def _finding(f: Dict[str, Any], rating: Optional[Dict[str, Any]],
             field: str) -> List[str]:
    adj = f.get("adjudication") or {}
    v = adj.get("verdict")
    head = f"### {f.get('claim_source')}, claim {f.get('claim_id')}"
    if rating:
        head += f" — {field}: {rating.get(field)}"
    out = [head, "",
           f"> \"{(f.get('quote') or '').strip()}\" "
           f"({f.get('claim_source')}, {_lines(f.get('lines'))})", "",
           f"**Verdict:** {VERDICT_WORDS.get(v, v)}."]
    if adj.get("gap"):
        out.append(f"**The gap:** {adj['gap'].strip()}")
    if adj.get("unresolved_because"):
        out.append(f"**Why unsettled:** "
                   f"{DISPOSITION_WORDS.get(adj['unresolved_because'], adj['unresolved_because'])}.")
    if rating:
        out.append(f"**{field.capitalize()} — {rating.get(field)}:** "
                   f"{RATING_WORDS.get(rating.get(field), '')}. "
                   f"{(rating.get('basis') or '').strip()}")
    if f.get("correction"):
        out.append(f"**Correction:** {f['correction'].strip()}")
    out += ["", "Evidence:", ""] + (_evidence(f.get("evidence")) or ["- (none)"])
    out += ["", f"Review: {_review_line(f)}.", ""]
    return out


def coverage(merged: Dict[str, Any], ratings: Dict[str, Any],
             classes: Dict[str, List[Dict[str, Any]]]) -> List[str]:
    """The figures, computed. Never written by the agent."""
    fig = merged.get("figures") or {}
    rf = ratings.get("figures") or {}
    out = ["| claim source | claims | findings | reviewed | files read | "
           "gathering legs | model |", "|---|---|---|---|---|---|---|"]
    for r in merged.get("runs") or []:
        out.append(f"| {r.get('claim_source')} | {r.get('claims')} | "
                   f"{r.get('findings')} | {'yes' if r.get('reviewed') else 'no'} | "
                   f"{r.get('files_read', '')} | {r.get('gathering_legs', '')} | "
                   f"{r.get('resolved_model') or ''} |")
    v = fig.get("verdicts") or {}
    out += ["",
            f"Findings: {fig.get('findings', 0)} over {fig.get('claims', 0)} claims. "
            + ", ".join(f"{k} {v[k]}" for k in sorted(v)) + ".",
            "",
            f"Shown, by materiality: "
            + (", ".join(f"{k} {n}" for k, n in sorted((rf.get('materiality') or {}).items()))
               or "none") + ".",
            f"Unsettled and not examined, by exposure: "
            + (", ".join(f"{k} {n}" for k, n in sorted((rf.get('exposure') or {}).items()))
               or "none") + ".",
            f"Not examined: {len(classes['not_examined'])} claim(s)."]
    unopened = sorted({f for r in merged.get("runs") or []
                       for f in (r.get("unopened_candidates") or [])})
    if unopened:
        out += ["", f"Files the searches named that were not opened "
                    f"({len(unopened)}):", ""]
        out += [f"- `{f}`" for f in unopened]
    return out


def _slot(name: str, prose: Optional[Dict[str, Any]]) -> List[str]:
    text = ((prose or {}).get(name) or "").strip()
    if text:
        return [text, ""]
    return [f"[[{name}]]", ""]


def assemble(record: Dict[str, Any], prose: Optional[Dict[str, Any]] = None,
             transaction: Optional[str] = None,
             engagement: Optional[str] = None) -> str:
    """REPORT.md §6, in order. `prose` absent leaves a `[[field]]` marker in
    each slot, which is the form the agent is shown."""
    merged, ratings = record["merged"], record["ratings"]
    classes = classify(merged)
    by_m = _ratings_by_key(ratings, "ratings")
    by_e = _ratings_by_key(ratings, "exposures")
    when = ""
    for r in merged.get("runs") or []:
        when = when or (r.get("dir") or "")[-40:]
    out = [f"# Technical claims audit — {engagement or merged.get('engagement') or 'engagement'}",
           "", "Limited assurance; see Limitations. Every finding cites the "
               "material that settles it.", ""]
    if transaction:
        out += ["## The transaction", "", transaction.strip(), ""]
    else:
        out += ["## The transaction", "",
                "The engagement states nothing about the transaction. Ratings "
                "assume a buyer paying a price that assumes every claim holds.", ""]
    out += ["## Summary", ""] + _slot("summary", prose)
    out += ["## How to read a finding", "",
            "Each finding names the seller's claim as the claim source states "
            "it, the verdict, the evidence that settles it — a file and line "
            "numbers in the materials — and, where the audit showed a gap, "
            "what that gap would change for this transaction.", "",
            "Three classes follow. **What the audit showed** is the only one "
            "that counts against the seller. **Unsettled** claims are ones the "
            "supplied materials cannot settle; nothing was found against them. "
            "**Not examined** claims are ones whose settling files the "
            "engagement did not open.", ""]
    out += ["## What the audit showed", ""] + _slot("shown_note", prose)
    for f in _ordered(classes["shown"], by_m, "materiality"):
        out += _finding(f, by_m.get(_key(f)), "materiality")
    if not classes["shown"]:
        out += ["No finding showed a gap.", ""]
    out += ["## Unsettled claims", ""] + _slot("unsettled_note", prose)
    for f in _ordered(classes["unsettled"], by_e, "exposure"):
        out += _finding(f, by_e.get(_key(f)), "exposure")
    if not classes["unsettled"]:
        out += ["None.", ""]
    if classes["not_examined"]:
        out += ["## Claims not examined", ""] + _slot("not_examined_note", prose)
        for f in _ordered(classes["not_examined"], by_e, "exposure"):
            out += _finding(f, by_e.get(_key(f)), "exposure")
    out += ["## Claims that hold", "",
            "| claim source | id | claim | evidence |", "|---|---|---|---|"]
    for f in sorted(classes["holds"], key=lambda x: (x.get("claim_source") or "",
                                                     x.get("claim_id") or 0)):
        cites = "; ".join(f"`{e.get('document')}` {_lines(e.get('lines'))}"
                          for e in f.get("evidence") or []
                          if isinstance(e, dict) and e.get("form") == "citation")
        q = (f.get("quote") or "").replace("|", "\\|").replace("\n", " ")
        out.append(f"| {f.get('claim_source')} | {f.get('claim_id')} | {q[:200]} "
                   f"| {cites} |")
    if not classes["holds"]:
        out.append("| | | none | |")
    out.append("")
    if merged.get("questions"):
        out += ["## Questions for the seller", ""]
        out += [f"- ({q.get('claim_source')}) {q.get('question')}"
                for q in merged["questions"]] + [""]
    if merged.get("unclaimed"):
        out += ["## Observations the seller did not claim", ""]
        for u in merged["unclaimed"]:
            out.append(f"- ({u.get('claim_source')}) {u.get('note')}")
            out += ["  " + x for x in _evidence([u.get("evidence") or {}])]
        out.append("")
    out += ["## Coverage", ""] + coverage(merged, ratings, classes) + [""]
    out += _slot("coverage_note", prose)
    out += ["## Limitations", ""] + _slot("limitations", prose)
    return "\n".join(out).rstrip() + "\n"
