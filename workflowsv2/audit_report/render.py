"""The client's document, assembled from the record. Mechanical.

Reads `merged.json` and `materiality.json` from a materiality output directory
and writes the document REPORT.md §6 describes. The passages the agent
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
import re
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


def _md_safe(text: Any) -> str:
    """Quoted text placed into the markdown as text, never as markup. A
    ChatterMate quote carried a code fence, which opened a code block that
    swallowed seven of the document's twelve sections when rendered
    (2026-09-02). Whitespace collapses to one line; backtick runs and pipes
    are escaped; a leading `#`, `>` or list marker cannot start a block
    because the text never starts a line on its own."""
    s = " ".join(str(text or "").split())
    s = s.replace("```", "\\`\\`\\`").replace("|", "\\|")
    return s


def _lines(lines: Any) -> str:
    if isinstance(lines, list) and len(lines) == 2:
        return f"line {lines[0]}" if lines[0] == lines[1] \
            else f"lines {lines[0]}–{lines[1]}"
    return ""


def _lines_bare(lines: Any) -> str:
    """The range alone, for a table column headed `lines`: the word cost a
    row of height in nearly every appendix entry (Bruce, 2026-09-02)."""
    if isinstance(lines, list) and len(lines) == 2:
        return str(lines[0]) if lines[0] == lines[1] else f"{lines[0]}–{lines[1]}"
    return ""


def _evidence(items: Sequence[Dict[str, Any]]) -> List[str]:
    out = []
    for e in items or []:
        if not isinstance(e, dict):
            continue
        form = e.get("form")
        if form == "citation":
            out.append(f"- `{e.get('document')}`, {_lines(e.get('lines'))}: "
                       f"\"{_md_safe(e.get('quote'))}\" — "
                       f"{_md_safe(e.get('shows'))}")
        elif form == "derived":
            basis = "; ".join(f"`{b.get('document')}` {_lines(b.get('lines'))}"
                              for b in (e.get("basis") or []) if isinstance(b, dict))
            out.append(f"- derived from {basis}: {_md_safe(e.get('derivation'))} "
                       f"— {_md_safe(e.get('consequence'))}")
        elif form == "search":
            cands = ", ".join(f"`{c}`" for c in (e.get("candidates") or []))
            out.append(f"- searched ({e.get('kind')}): "
                       f"{_md_safe(e.get('performed'))} — "
                       f"{_md_safe(e.get('result'))}"
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


_URL = re.compile(r"https?://[^\s)\]>\"']+")
#: A markdown image's source: the picture a badge shows, not where it points.
_IMAGE_SRC = re.compile(r"!\[[^\]]*\]\((https?://[^\s)]+)\)")
_LOOPBACK = ("localhost", "127.0.0.1", "0.0.0.0", "[::1]")


def _links(f: Dict[str, Any]) -> List[str]:
    """Every link the claim source itself gives where this claim is made,
    in order of first appearance. THE LINK IS QUOTED, NOT INFERRED: a
    package name in a shell command is not turned into a registry URL.
    Two kinds are left out on their syntax alone: an image's source (a
    badge picture) and an address on the reader's own machine."""
    texts = [f.get("quote") or ""] + [loc.get("quote") or ""
                                       for loc in f.get("locations") or []]
    out: List[str] = []
    for t in texts:
        images = set(_IMAGE_SRC.findall(t))
        for u in _URL.findall(t):
            u = u.rstrip(".,;:")
            host = u.split("//", 1)[1].split("/", 1)[0].split(":")[0]
            if u in images or host in _LOOPBACK or u in out:
                continue
            out.append(u)
    return out


def _finding(f: Dict[str, Any], rating: Optional[Dict[str, Any]],
             field: str) -> List[str]:
    adj = f.get("adjudication") or {}
    v = adj.get("verdict")
    head = f"### {f.get('claim_source')}, claim {f.get('claim_id')}"
    if rating:
        head += f" — {field}: {rating.get(field)}"
    out = [head, "",
           f"> \"{_md_safe(f.get('quote'))}\" "
           f"({f.get('claim_source')}, {_lines(f.get('lines'))})"]
    for loc in f.get("locations") or []:
        out.append(f"> also, at {_lines(loc.get('lines'))}: "
                   f"\"{_md_safe(loc.get('quote'))}\"")
    out += [""]
    if f.get("about") == "seller":
        out += ["This assertion is about the seller's own activity or "
                "hosted service, which the supplied materials are not "
                "expected to reach.", ""]
    out += [f"**Verdict:** {VERDICT_WORDS.get(v, v)}.", ""]
    if adj.get("gap"):
        out += [f"**The gap:** {_md_safe(adj['gap'])}", ""]
    if adj.get("unresolved_because"):
        out += [f"**Why unsettled:** "
                f"{DISPOSITION_WORDS.get(adj['unresolved_because'], adj['unresolved_because'])}.", ""]
    if adj.get("unresolved_because") == "outside_the_materials":
        links = _links(f)
        if links:
            out += ["**Where the claim source points:** " + ", ".join(
                f"<{u}>" for u in links) + ". The audit did not follow "
                "these links; the buyer can confirm what is there directly.", ""]
    if rating:
        agree = rating.get("agreement")
        tag = ""
        if agree:
            tag = (f" (rated {agree.split(' of ')[1]} times, {agree.split(' of ')[0]} agree"
                   + ("; **borderline**" if rating.get("borderline") else "") + ")")
        out += [f"**{field.capitalize()} — {rating.get(field)}{tag}:** "
                f"{RATING_WORDS.get(rating.get(field), '')}. "
                f"{_md_safe(rating.get('basis'))}", ""]
        if rating.get("borderline"):
            out += ["The replicates split on this rating. The other readings:", ""]
            out += [f"- *{sm.get(field)}* — {_md_safe(sm.get('basis'))}"
                    for sm in rating.get("samples") or []
                    if sm.get(field) != rating.get(field)] + [""]
    if f.get("correction"):
        out += [f"**Correction:** {_md_safe(f['correction'])}", ""]
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


def _first_sentence(text: str) -> str:
    t = " ".join((text or "").split())
    for stop in (". ", "; "):
        i = t.find(stop)
        if 0 < i < 220:
            return t[:i + 1]
    return t[:220]


def key_findings(classes: Dict[str, List[Dict[str, Any]]],
                 by_m: Dict[str, Dict[str, Any]]) -> List[str]:
    """The shown findings rated material or decisive, one line each, in
    rating order. Computed, so the executive summary lists what the ratings
    say and not what a writer chose to mention."""
    rows = []
    for f in _ordered(classes["shown"], by_m, "materiality"):
        r = by_m.get(_key(f))
        if not r or r.get("materiality") not in ("material", "decisive"):
            continue
        adj = f.get("adjudication") or {}
        rows.append(f"- **{f.get('claim_source')}, claim {f.get('claim_id')}** "
                    f"({VERDICT_WORDS.get(adj.get('verdict'), adj.get('verdict')).split(' — ')[0]}; "
                    f"{r.get('materiality')}): {_md_safe(_first_sentence(adj.get('gap') or ''))}")
    return rows


def _front_matter(engagement: str, dates: List[str], revs: List[str],
                  sources: List[str]) -> List[str]:
    """Title, the materials' date and version, the assurance given, and who
    is responsible for what. Fixed text from the record; defines the terms it
    uses at first use."""
    when = ", ".join(dates) if dates else "an undated run"
    rev = (" at commit " + ", ".join(r[:12] for r in revs)) if revs else ""
    src = ", ".join(f"`{s}`" for s in sources) or "the claim sources named by the engagement"
    return [
        f"# Technical claims audit — {engagement}", "",
        f"Materials as of {when}{rev}. Claim sources: {src}.", "",
        "**What this document is.** A claims audit: the assertions the seller "
        "makes in the claim sources are tested, one by one, against the "
        "materials the seller supplied, and each is reported with the "
        "evidence that settles it. A *claim source* is a document in which "
        "the seller asserts things about the target; the *materials* are "
        "everything supplied, including the claim sources, source code and "
        "configuration. The audit examined what was supplied and nothing "
        "else.", "",
        "**The assurance given is limited.** The audit reports what the "
        "materials show about each claim. It did not perform procedures "
        "beyond examining the materials, so a claim the materials cannot "
        "settle is reported as unsettled, not as false, and the document "
        "states no overall conclusion on the target or the transaction. An "
        "overall assessment is given only against thresholds the buyer has "
        "stated; where this document carries none, none was recorded.", "",
        "**Responsibilities.** The seller made the claims and was not "
        "consulted; the seller has not confirmed the audit's reading of any "
        "claim. The practice performed the audit under its written method, "
        "at the version each run received, retained with the record, and is "
        "responsible for the findings and their ratings. The buyer is "
        "responsible for decisions taken on them.", ""]


def _how_to_read() -> List[str]:
    """Every term a finding uses, defined once, before the first finding."""
    return [
        "## How to read a finding", "",
        "Each finding names one claim as the claim source states it, gives a "
        "*verdict*, cites the *evidence* — a file and line numbers in the "
        "materials, quoted — and, where a rating applies, says what the gap "
        "would change for this transaction. A *review* is an independent "
        "second pass over each finding's evidence, and a *retest* is a "
        "second reviewer, blind to the first, on the findings the review "
        "questioned. Each finding ends with the review's outcome.", "",
        "**Verdicts**, and the class each puts a claim in:", "",
        "| verdict | meaning | class |", "|---|---|---|",
        "| contradicted | " + VERDICT_WORDS["contradicted"] + " | shown |",
        "| partial | " + VERDICT_WORDS["partial"] + " | shown |",
        "| real_with_caveat | " + VERDICT_WORDS["real_with_caveat"] + " | shown |",
        "| unverifiable | " + VERDICT_WORDS["unverifiable"] + " | unsettled, or not examined |",
        "| real | the materials bear the claim out | holds |", "",
        "**The three classes.** *Shown* findings are gaps the audit "
        "demonstrated in the materials; they are the only findings that count "
        "against the seller, and each carries a *materiality* rating. "
        "*Unsettled* claims are ones the supplied materials cannot settle; "
        "nothing was found against them, and each carries an *exposure* "
        "rating. *Not examined* claims are unsettled claims whose searches "
        "named files the engagement did not open; they are listed apart and "
        "are the first thing a further pass would settle. Claims that hold "
        "are listed after the three classes.", "",
        "**Where the claim source points.** An unsettled claim whose "
        "materials were not supplied — a listing, a published package, a "
        "hosted service — shows any link the claim source itself gives for "
        "it. The audit did not follow those links and says nothing about "
        "what is there; the buyer can look.", "",
        "**Ratings.** Materiality and exposure use one scale, read for a gap "
        "the audit showed or for a claim assumed false:", "",
        "| rating | meaning |", "|---|---|",
        "| not_material | " + RATING_WORDS["not_material"] + " |",
        "| material | " + RATING_WORDS["material"] + " |",
        "| decisive | " + RATING_WORDS["decisive"] + " |", "",
        "Materiality and exposure are never added together: one counts what "
        "the audit showed, the other what rests on what it could not settle.", "",
        "**How stable a rating is.** Each rating was made twice, independently; "
        "where the two readings differed, the finding was rated five times and "
        "the rating shown is the one most of them gave, with the count. A rating "
        "marked *borderline* is one the readings split on: the other readings are "
        "shown beneath it, and the practice has decided which to carry. The count "
        "measures how stable one reading of the buyer's thresholds is, not "
        "whether it is right.", "",
        "**Why a claim is unsettled** is recorded as one of: " + "; ".join(
            f"*{k.replace('_', ' ')}*, {v}" for k, v in DISPOSITION_WORDS.items()) + ".", ""]


def assemble(record: Dict[str, Any], prose: Optional[Dict[str, Any]] = None,
             transaction: Optional[str] = None,
             engagement: Optional[str] = None,
             thresholds: Optional[str] = None,
             conclusion: bool = False) -> str:
    """REPORT.md §6, in order. `prose` absent leaves a `[[field]]` marker in
    each slot, which is the form the agent is shown."""
    merged, ratings = record["merged"], record["ratings"]
    classes = classify(merged)
    by_m = _ratings_by_key(ratings, "ratings")
    by_e = _ratings_by_key(ratings, "exposures")
    runs = merged.get("runs") or []
    dates = sorted({(r.get("captured_at_utc") or "")[:10] for r in runs} - {""})
    revs = sorted({r.get("target_rev") or "" for r in runs} - {""})
    sources = [r.get("claim_source") for r in runs if r.get("claim_source")]
    out = _front_matter(engagement or merged.get("engagement") or "engagement",
                        dates, revs, sources)

    out += ["## The transaction", ""]
    out += ["  \n".join(transaction.strip().splitlines()), ""] if transaction else [
        "The engagement states nothing about the transaction. Ratings assume "
        "a buyer paying a price that assumes every claim holds.", ""]
    out += ["**The buyer's thresholds.**  \n" + (
        "  \n".join(thresholds.strip().splitlines()) if thresholds else
        "None recorded. Ratings are read against the rating scale alone, not "
        "against what the buyer said would change the price or end the deal."), ""]

    out += ["## Executive summary", ""] + _slot("summary", prose)
    kf = key_findings(classes, by_m)
    out += ["**The material findings**, ordered by rating — each is set out in "
            "full under *What the audit showed*:", ""]
    out += kf + [""] if kf else ["No finding was rated material or decisive.", ""]

    # THE CONCLUSION IS OPT-IN AND CONDITIONAL. Present only when the
    # engagement asked for one and the buyer's thresholds are recorded; the
    # front matter says an overall assessment is given only against stated
    # thresholds, and this is where it is given.
    if conclusion and thresholds:
        out += ["## Conclusion", "",
                "The engagement asked for a conclusion. It is read against "
                "the buyer's thresholds stated above and against nothing "
                "else; it counts what the audit showed, not what it could "
                "not settle.", ""] + _slot("conclusion", prose)

    out += ["## Scope and approach", "",
            "| claim source | claims | findings | reviewed | files read | "
            "gathering legs | model |", "|---|---|---|---|---|---|---|"]
    for r in runs:
        out.append(f"| {r.get('claim_source')} | {r.get('claims')} | "
                   f"{r.get('findings')} | {'yes' if r.get('reviewed') else 'no'} | "
                   f"{r.get('files_read', '')} | {r.get('gathering_legs', '')} | "
                   f"{r.get('resolved_model') or ''} |")
    out += ["", "*Claims* are the seller's assertions as enumerated from the "
                "claim source, one finding each. *Files read* counts the "
                "target's files the audit opened while gathering evidence, "
                "over the number of *gathering legs* it took. *Reviewed* says "
                "whether the independent review ran on that claim source.", ""]
    subs = sorted({m for r in runs for m in ((r.get("materials") or {}).get("submodules") or [])})
    if subs:
        out += ["**Not supplied:** the materials name "
                + ("one submodule" if len(subs) == 1 else f"{len(subs)} submodules")
                + " kept elsewhere by the seller — " + ", ".join(f"`{m}`" for m in subs)
                + ". Nothing under them was examined; a claim whose evidence "
                "would sit there is reported as unsettled, with that reason.", ""]
    out += _slot("scope_note", prose)

    out += _how_to_read()

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
        q = _md_safe(f.get("quote"))
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

    out += ["## Coverage", "", "Every claim received one finding. By verdict "
            "and class:", "", "| verdict | class | claims |", "|---|---|---|"]
    counts = {"contradicted": 0, "partial": 0, "real_with_caveat": 0,
              "unverifiable": 0, "real": 0}
    for f in merged.get("findings") or []:
        v = (f.get("adjudication") or {}).get("verdict")
        counts[v] = counts.get(v, 0) + 1
    cls = {"contradicted": "shown", "partial": "shown", "real_with_caveat": "shown",
           "unverifiable": "unsettled", "real": "holds"}
    for v, n in counts.items():
        out.append(f"| {v} | {cls.get(v, '')} | {n} |")
    rf = ratings.get("figures") or {}
    out += ["",
            "Shown findings by materiality: "
            + (", ".join(f"{k} {n}" for k, n in sorted((rf.get('materiality') or {}).items()))
               or "none") + ". Unsettled claims by exposure: "
            + (", ".join(f"{k} {n}" for k, n in sorted((rf.get('exposure') or {}).items()))
               or "none") + f". Not examined: {len(classes['not_examined'])}.", ""]
    unopened = sorted({f for r in runs for f in (r.get("unopened_candidates") or [])})
    if unopened:
        out += [f"Files the searches named that were not opened ({len(unopened)}):", ""]
        out += [f"- `{f}`" for f in unopened] + [""]

    out += ["## Limitations", ""] + _slot("limitations", prose)
    out += ["The party that made these claims was not consulted and has not "
            "confirmed the audit's interpretation of them. Every verdict and "
            "rating is defined by the practice's method at the version each "
            "run received, which is retained with the record; a later method "
            "may define a term differently.", ""]

    out += ["## Appendix — every claim and its verdict", "",
            "The claim surface as the audit froze it, in document order, with "
            "the verdict each claim received and, where rated, its "
            "materiality or exposure.", "",
            "| source | id | lines | claim | verdict | rating |",
            "|---|---|---|---|---|---|"]
    for f in sorted(merged.get("findings") or [],
                    key=lambda x: (x.get("claim_source") or "", x.get("claim_id") or 0)):
        k = _key(f)
        r = by_m.get(k) or by_e.get(k) or {}
        rating = r.get("materiality") or r.get("exposure") or ""
        q = _md_safe(f.get("quote"))
        out.append(f"| {f.get('claim_source')} | {f.get('claim_id')} | "
                   f"{_lines_bare(f.get('lines'))} | {q[:120]} | "
                   f"{(f.get('adjudication') or {}).get('verdict')} | {rating} |")
    return "\n".join(out).rstrip() + "\n"


def worklist(merged: Dict[str, Any], merged_dir: Optional[Path] = None) -> str:
    """worklist.md: what a person still has to look at, gathered from every
    stage's issues file — each run directory's, its review's, and the merged
    directory's. v1's editor notes did this from one run; v2 spread the
    record across stages and consolidated nothing until 2026-09-02."""
    rows: List[Dict[str, Any]] = []
    places = []
    for r in merged.get("runs") or []:
        d = Path(r.get("dir") or "")
        places += [(r.get("claim_source"), d / "issues.jsonl"),
                   (r.get("claim_source"), d / "review" / "issues.jsonl")]
    if merged_dir is not None:
        places.append(("merged", Path(merged_dir) / "issues.jsonl"))
    for source, path in places:
        if not path.is_file():
            continue
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                d = json.loads(line)
            except ValueError:
                continue
            d["source"] = source
            rows.append(d)
    order = {"blocking": 0, "check": 1, "note": 2}
    rows.sort(key=lambda d: (order.get(d.get("severity"), 9), d.get("stage") or "",
                             d.get("code") or ""))
    out = ["# Worklist", "",
           f"{len(rows)} item(s) recorded by the stages. `blocking`: a client "
           "must not see this as it stands. `check`: a person decides; it may "
           "be fine. `note`: recorded so it is not lost.", ""]
    if not rows:
        out.append("Nothing recorded.")
    last = None
    for d in rows:
        if d.get("severity") != last:
            last = d.get("severity")
            out += ["", f"## {last}", ""]
        out.append(f"- **{d.get('stage')} / {d.get('code')}** ({d.get('source')}): "
                   f"{d.get('text')}")
    return "\n".join(out) + "\n"
