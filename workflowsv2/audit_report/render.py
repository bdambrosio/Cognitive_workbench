"""The client's document, assembled from the record. Mechanical.

Reads `merged.json` and `materiality.json` from a materiality output directory
and writes the document REPORT.md §6 describes. The passages the agent
writes are placed where §6 names them; without them the document still
assembles, with a marker where each passage would go, and that is what the
agent is shown.

THREE CLASSES, AND ONLY ONE IS A MARK AGAINST THE SELLER. Findings with a
verdict about the claim are what the review showed, ordered by materiality.
`unverifiable` findings are unsettled, ordered by exposure, and among them
those the searches named files for that nobody opened are set apart as not
examined. Unsettled claims tagged `about: seller` at enumeration — promises
about the seller's conduct or services, which code is not expected to settle
— are listed under their own heading within the unsettled class, each with
the question the review puts to the seller (2026-09-07). Nothing here
re-judges: every verdict, gap, rating and basis is the record's, copied.
"""
from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from workflowsv2.audit_materiality import schemas as ms
from workflowsv2.audit_report import schemas as rs
from workflowsv2.claims_audit.schemas import NOT_EXAMINED

logger = logging.getLogger("audit_report.render")

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

#: A rating as the reader sees it outside the definitions and the appendices.
RATING_LABEL = {"not_material": "not material", "material": "material",
                "decisive": "decisive"}

#: MATERIALITY.md §5a's bands, as the reader sees them. `none` is not shown.
BAND_WORDS = {"setting": "a setting", "one_place": "one place in the code",
              "several_places": "several places in the code",
              "design": "a design change"}

RATING_WORDS = {
    "not_material": "would change neither the price, the terms, nor the "
                    "decision to close",
    "material": "would change the price or the terms materially: the buyer "
                "would go back to the seller over it",
    "decisive": "on its own would change the decision to close",
}


DISPOSITIONS = "dispositions.json"
PRACTICE_REVIEW_WORDS = {"retained": "retained after review by the practice",
                         "unresolved": "unresolved; the practice has not decided between the finding and the check",
                         "withdrawn": "withdrawn by the practice after review; it is not counted among "
                                      "the findings of this document"}


def apply_dispositions(merged: Dict[str, Any], merged_dir: Path) -> None:
    """Put the practice's decision on each finding the check did not uphold
    into that finding as `disposition`. The decisions are written by hand,
    after reading the finding and the check, in DISPOSITIONS beside
    merged.json: {"<claim source>#<claim id>": {"disposition": "retained" |
    "unresolved" | "withdrawn", "note": "..."}}. `withdrawn` (Bruce,
    2026-10-14): the practice agrees the finding is wrong; it stays in its
    section, marked, and is named in no summary or conclusion."""
    p = Path(merged_dir) / DISPOSITIONS
    disp = json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}
    for f in merged.get("findings") or []:
        if _key(f) in disp:
            f["disposition"] = disp[_key(f)]


def load(merged_dir: Path) -> Dict[str, Any]:
    merged_dir = Path(merged_dir)
    merged = json.loads((merged_dir / "merged.json").read_text(encoding="utf-8"))
    ratings = json.loads((merged_dir / "materiality.json").read_text(encoding="utf-8"))
    apply_dispositions(merged, merged_dir)
    from workflowsv2.composition import scan as composition
    return {"merged": merged, "ratings": ratings, "dir": merged_dir,
            "covered": covered_by(merged_dir), "not_tested": not_tested(merged_dir),
            # None unless the engagement enables composition analysis and has
            # a scan (<engagement>/composition/); the merged directory sits
            # at <engagement>/merged/<name>.
            "composition": composition.latest(merged_dir.resolve().parents[1])}


def not_tested(merged_dir: Path) -> List[Dict[str, Any]]:
    """The claims listed and not tested: rated tier 2 or 3 before the freeze
    (TIERS.md §1). Read from the engagement's frozen surfaces, where
    covered_by reads the duplicates record, because a claim source whose
    every claim is tier 2 or 3 has no run to read them from. Each row
    carries its claim source."""
    from workflowsv2.claims_audit.schemas import split_by_tier
    rows: List[Dict[str, Any]] = []
    for f in sorted((Path(merged_dir).resolve().parents[1] / "surface").glob("*.surface.json")):
        try:
            doc = json.loads(f.read_text(encoding="utf-8"))
        except Exception as e:                                 # noqa: BLE001
            logger.warning("frozen surface %s not read: %s", f, e)
            continue
        rows += [dict(c, claim_source=doc.get("claim_source"))
                 for c in split_by_tier(doc.get("claims") or [])[1]]
    return rows


def covered_by(merged_dir: Path) -> Dict[str, Dict[str, Any]]:
    """{finding key: the wider claim} for every claim the duplicates pass
    marked as within another (claims_audit/duplicates.py, record at
    <engagement>/surface/duplicates.json; the merged directory sits at
    <engagement>/merged/<name>). A covered claim is audited on its own like
    any other (Bruce, 2026-09-17: its verdict is never derived from the
    wider claim's); the report says which claim covers it and how that one
    came out, so the reader sees the two together."""
    f = Path(merged_dir).resolve().parents[1] / "surface" / "duplicates.json"
    if not f.is_file():
        return {}
    try:
        pairs = json.loads(f.read_text(encoding="utf-8")).get("pairs") or []
    except Exception as e:                                     # noqa: BLE001
        logger.warning("duplicates record %s not read: %s", f, e)
        return {}
    return {f"{p['source']}#{p['id']}": p["other"]
            for p in pairs if p.get("relation") == "within" and p.get("other")}


def _key(f: Dict[str, Any]) -> str:
    return f"{f.get('claim_source')}#{f.get('claim_id')}"


def classify(merged: Dict[str, Any]) -> Dict[str, List[Dict[str, Any]]]:
    """shown / unsettled / seller_unsettled / not_examined / holds, from the
    verdicts. `seller_unsettled` is the unsettled claims tagged `about:
    seller`; a seller-tagged claim the materials did settle stays with its
    verdict, and an unexamined one stays with that class.

    `asked_buyer` and `asked_practice` are the findings on statements from a
    question source (`asked_by`), whatever their verdict: the seller did not
    make them, so none of the classes above, which are read as a record of
    what the seller said, holds them."""
    out: Dict[str, List[Dict[str, Any]]] = {
        "shown": [], "unsettled": [], "seller_unsettled": [], "not_examined": [],
        "holds": [], "asked_buyer": [], "asked_practice": []}
    for f in merged.get("findings") or []:
        adj = f.get("adjudication") or {}
        if f.get("asked_by"):
            out["asked_buyer" if f["asked_by"] == "buyer" else "asked_practice"].append(f)
        elif ms.rateable(f):
            out["shown"].append(f)
        elif ms.exposable(f):
            if adj.get("unresolved_because") == NOT_EXAMINED:
                out["not_examined"].append(f)
            elif f.get("about") == "seller":
                out["seller_unsettled"].append(f)
            else:
                out["unsettled"].append(f)
        else:
            out["holds"].append(f)
    return out


#: The line before the seller's promises. Fixed text, not a prose slot.
SELLER_LINE = ("These claims are promises about what the seller does or offers — "
               "maintenance, a hosted service, commercial terms, what is planned — "
               "not about what the software does. The supplied materials are not "
               "expected to settle them, and nothing in the materials was found "
               "against them. Each carries its exposure rating, what would change "
               "if the promise were not kept, and the question the review puts to "
               "the seller. Ordered by exposure.")


#: The lines before the questions the buyer and the practice asked. Fixed
#: text, not prose slots.
ASKED_LINES = {
    "asked_buyer": (
        "Questions the buyer asked",
        "The buyer asked these questions at intake. The practice wrote each one as "
        "a statement of what the target is expected to have, and the review tested "
        "it as it tests a claim. The seller did not make these statements, so a gap "
        "here is a finding about the target, not about what the seller said. A gap "
        "carries its materiality rating; a statement the materials cannot settle "
        "carries its exposure rating. Where the buyer said at intake what a \"no\" "
        "would do to the deal, that is the rating, and it is marked as the buyer's. "
        "Gaps come first, then unsettled statements, then those that hold."),
    "asked_practice": (
        "Questions the practice asked",
        "The practice asks these of every engagement, or raised them from a claim "
        "the seller made. Each is a statement of what the target is expected to "
        "have, tested as a claim is tested. The seller did not make these "
        "statements, so a gap here is a finding about the target, not about what "
        "the seller said. A gap carries its materiality rating; a statement the "
        "materials cannot settle carries its exposure rating. Gaps come first, then "
        "unsettled statements, then those that hold."),
}


def _asked_ordered(findings: Sequence[Dict[str, Any]], by_m: Dict[str, Dict[str, Any]],
                   by_e: Dict[str, Dict[str, Any]]) -> List[Tuple[Dict[str, Any], Optional[Dict[str, Any]], str]]:
    """(finding, rating, rating field): gaps by materiality, then unsettled
    statements by exposure, then those that hold."""
    gaps = [f for f in findings if ms.rateable(f)]
    open_ = [f for f in findings if not ms.rateable(f) and ms.exposable(f)]
    held = [f for f in findings if not ms.rateable(f) and not ms.exposable(f)]
    return ([(f, by_m.get(_key(f)), "materiality") for f in _ordered(gaps, by_m, "materiality")]
            + [(f, by_e.get(_key(f)), "exposure") for f in _ordered(open_, by_e, "exposure")]
            + [(f, None, "") for f in _ordered(held, {}, "")])


def _ratings_by_key(ratings: Dict[str, Any], array: str
                    ) -> Dict[str, Dict[str, Any]]:
    return {f"{r.get('claim_source')}#{r.get('claim_id')}": r
            for r in ratings.get(array) or []}


def _questions_by_key(merged: Dict[str, Any], dropped: Optional[set] = None
                      ) -> Dict[str, List[str]]:
    """The questions for the seller that name a claim, by that claim's key,
    in record order. `dropped`: the numbers (from 1, in record order) of
    questions that repeat an earlier one, left out."""
    out: Dict[str, List[str]] = {}
    for i, q in enumerate(merged.get("questions") or [], 1):
        if i in (dropped or set()) or q.get("claim_id") is None or not q.get("question"):
            continue
        out.setdefault(f"{q.get('claim_source')}#{q.get('claim_id')}", []).append(q["question"])
    return out


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
    # A `<` starts raw HTML in CommonMark; a quoted `<script>` tag swallowed
    # the rest of the rendered report (2026-09-16). Escaped, it stays text.
    s = s.replace("<", "\\<")
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


#: How many words of a quoted line the report shows; the evidence document
#: carries the whole of it (Bruce, 2026-10-14).
OPENING_WORDS = 8


def _opening(quote: Any) -> str:
    """The first words of a quoted line, enough to find it at the cited lines."""
    words = _md_safe(quote).split()
    return " ".join(words[:OPENING_WORDS]) + (" …" if len(words) > OPENING_WORDS else "")


def _evidence(items: Sequence[Dict[str, Any]], full: bool = False) -> List[str]:
    """Citations and derived facts first, in record order, then the
    searches: what settles a claim before the record of where else the
    practice looked.

    The report shows each item short: a citation's place, the first words of
    its quote and what it shows; a derived fact's sources and consequence; a
    search's result. `full` gives every field, for the evidence document."""
    items = [e for e in items or [] if isinstance(e, dict)]
    items = ([e for e in items if e.get("form") != "search"]
             + [e for e in items if e.get("form") == "search"])
    out = []
    for e in items:
        form = e.get("form")
        if form == "citation":
            shows = _md_safe(e.get("shows"))
            quote = _md_safe(e.get("quote")) if full else _opening(e.get("quote"))
            out.append(f"- `{e.get('document')}`, {_lines(e.get('lines'))}: "
                       f"\"{quote}\"" + (f" — {shows}" if shows else ""))
        elif form == "derived":
            basis = "; ".join(f"`{b.get('document')}` {_lines(b.get('lines'))}"
                              + (f": \"{_md_safe(b.get('quote'))}\"" if full else "")
                              for b in (e.get("basis") or []) if isinstance(b, dict))
            if e.get("from_knowledge") is True:
                head = ("- derived **from knowledge of the dependency, not from the "
                        f"supplied materials**, with {basis} as its basis: ")
            else:
                head = f"- derived from {basis}: "
            out.append(head + (f"{_md_safe(e.get('derivation'))} — " if full else "")
                       + _md_safe(e.get("consequence")))
        elif form == "search":
            if full:
                cands = ", ".join(f"`{c}`" for c in (e.get("candidates") or []))
                out.append(f"- searched ({e.get('kind')}): "
                           f"{_md_safe(e.get('performed'))} — "
                           f"{_md_safe(e.get('result'))}"
                           + (f" Files named: {cands}." if cands else ""))
            else:
                out.append(f"- searched: {_md_safe(e.get('result'))}")
    return out


def _from_knowledge(f: Dict[str, Any]) -> bool:
    """Whether any evidence item states what a dependency does from knowledge
    of it (METHOD §7, `from_knowledge`). Only a declared statement is seen: a
    gap that relies on such knowledge without the item is not."""
    return any(isinstance(e, dict) and e.get("form") == "derived"
               and e.get("from_knowledge") is True for e in f.get("evidence") or [])


KNOWLEDGE_MARK = "rests in part on knowledge of a dependency"


def _review_line(f: Dict[str, Any]) -> str:
    rv = f.get("review") or {}
    line = {"holds": "the check upheld this finding",
            "does_not_hold": "the check did not uphold this finding",
            "unreviewed": "not checked"}.get(rv.get("outcome"), "not checked")
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


def _verdict_short(v: Any) -> str:
    """The verdict in words, without its explanation: "partly true"."""
    return VERDICT_WORDS.get(v, str(v)).split(" — ")[0]


def _finding(f: Dict[str, Any], rating: Optional[Dict[str, Any]],
             field: str, questions: Optional[List[str]] = None,
             covered: Optional[Dict[str, Any]] = None,
             verdicts: Optional[Dict[str, str]] = None,
             level: int = 3) -> List[str]:
    adj = f.get("adjudication") or {}
    v = adj.get("verdict")
    # THE HEADING SAYS WHAT WAS TESTED AND HOW IT CAME OUT, in the record's
    # words: the claim's plain statement and the verdict. Where the claim
    # came from goes on the line below (Bruce, 2026-10-14).
    head = f"{'#' * level} {_md_safe(f.get('statement') or f.get('quote'))} — {_verdict_short(v)}"
    where = f"*{f.get('claim_source')}, claim {f.get('claim_id')}"
    if rating:
        where += f" · {field}: {RATING_LABEL.get(rating.get(field), rating.get(field))}"
    where += "*"
    if f.get("asked_by"):
        # The quote is the statement: a question source holds nothing else.
        who = "buyer" if f["asked_by"] == "buyer" else "practice"
        out = [head, "", where, ""]
        if rating and rating.get("in_short"):
            out += [f"**In short:** {_md_safe(rating['in_short'])}", ""]
        out += [f"Asked by the {who}."]
    else:
        out = [head, "", where, ""]
        if rating and rating.get("in_short"):
            out += [f"**In short:** {_md_safe(rating['in_short'])}", ""]
        out += [f"> \"{_md_safe(f.get('quote'))}\" "
                f"({f.get('claim_source')}, {_lines(f.get('lines'))})"]
    for loc in f.get("locations") or []:
        out.append(f"> also, at {_lines(loc.get('lines'))}: "
                   f"\"{_md_safe(loc.get('quote'))}\"")
    out += [""]
    if f.get("implied_by") is not None:
        out += [f"This is the practice's reading of claim {f['implied_by']}: a property "
                f"a reasonable buyer would take those words to assert, written down "
                f"before testing. The quoted words are claim {f['implied_by']}'s; "
                f"the finding tests the statement: {_md_safe(f.get('statement'))}"]
    if f.get("about") == "document":
        out += ["This assertion is about a document in the materials rather than "
                "about what the software does; the document itself settles it."]
    if f.get("about") == "seller":
        out += ["This assertion is about the seller's own activity or "
                "hosted service, which the supplied materials are not "
                "expected to reach.", ""]
    out += [f"**Verdict:** {VERDICT_WORDS.get(v, v)}.", ""]
    if _from_knowledge(f):
        out += [f"**{KNOWLEDGE_MARK.capitalize()}.** Part of the evidence for this verdict is "
                "a statement of what a third-party library does, made from the practice's "
                "knowledge of that library. The supplied materials show where the library "
                "is called and which version is used; they do not show what it does.", ""]
    if covered:
        wk = f"{covered.get('source')}#{covered.get('id')}"
        wv = (verdicts or {}).get(wk)
        out += [f"**Covered by:** {covered.get('source')}, claim {covered.get('id')}, a wider "
                f"claim of which this one is a part: \"{_md_safe(covered.get('statement'))}\". "
                + (f"That claim's verdict: {VERDICT_WORDS.get(wv, wv)}. " if wv else
                   "That claim is not in this report. ")
                + "This claim was tested on its own; the verdict above is its own.", ""]
    if adj.get("gap"):
        out += [f"**The gap:** {_md_safe(adj['gap'])}", ""]
    if adj.get("unresolved_because"):
        out += [f"**Why unsettled:** "
                f"{DISPOSITION_WORDS.get(adj['unresolved_because'], adj['unresolved_because'])}.", ""]
    if adj.get("unresolved_because") == "outside_the_materials":
        links = _links(f)
        if links:
            out += ["**Where the claim source points:** " + ", ".join(
                f"<{u}>" for u in links) + ". The practice did not follow "
                "these links; the buyer can confirm what is there directly.", ""]
    if rating:
        agree = rating.get("agreement")
        tag = ""
        if rating.get("by") == "buyer":
            tag = " (the buyer's own rating, given at intake)"
        elif agree:
            tag = (f" (rated {agree.split(' of ')[1]} times, {agree.split(' of ')[0]} agree"
                   + ("; **borderline**" if rating.get("borderline") else "") + ")")
        out += [f"**{field.capitalize()}: {RATING_LABEL.get(rating.get(field), rating.get(field))}**"
                f"{tag}. {_md_safe(rating.get('basis'))}", ""]
        if rating.get("borderline"):
            out += ["The replicates split on this rating. The other readings:", ""]
            out += [f"- *{RATING_LABEL.get(sm.get(field), sm.get(field))}* — {_md_safe(sm.get('basis'))}"
                    for sm in rating.get("samples") or []
                    if sm.get(field) != rating.get(field)] + [""]
        out += _remedy_line(rating.get("remedy"))
    # A `correction` is the audit's note on its own process; it stays in the
    # record and is not shown to the buyer.
    for q in questions or []:
        out += [f"**Question for the seller:** {_md_safe(q)}", ""]
    out += ["Evidence:", ""] + (_evidence(f.get("evidence")) or ["- (none)"])
    out += ["", f"Check: {_review_line(f)}.", ""]
    d = f.get("disposition")
    if d:
        out += [f"Practice review: {PRACTICE_REVIEW_WORDS[d['disposition']]}."
                + (f" {_md_safe(d['note'])}" if d.get("note") else ""), ""]
    return out


def _remedy_line(remedy: Optional[Dict[str, Any]]) -> List[str]:
    """MATERIALITY.md §5a's remedy, unless the band says nothing needs to
    change."""
    if not remedy or remedy.get("band") not in BAND_WORDS:
        return []
    return [f"**What would close it:** {_md_safe(remedy.get('change'))} — scope: "
            f"{BAND_WORDS[remedy['band']]}. The practice's estimate; not checked.", ""]


def observation_groups(merged: Dict[str, Any], ratings: Dict[str, Any]
                       ) -> List[Dict[str, Any]]:
    """The rated groups of observations, highest rating first, each with its
    number as the document shows it (1, 2, ...) and its members' records."""
    unclaimed = merged.get("unclaimed") or []
    groups = [dict(g, items=[unclaimed[m - 1] for m in g.get("members") or []
                             if 1 <= m <= len(unclaimed)])
              for g in ratings.get("observations") or []]
    groups = [g for g in groups if g["items"]]
    groups.sort(key=lambda g: _ORDER.get(g.get("materiality"), 99))
    for n, g in enumerate(groups, 1):
        g["number"] = n
    return groups


OBSERVATIONS_LINE = ("The review recorded these while testing claims. The seller asserted "
                     "nothing about them, so none is a claim that failed. Each is rated as a "
                     "gap the review showed, and they are ordered by rating.")


def _observation(g: Dict[str, Any]) -> List[str]:
    first = g["items"][0]
    out = [f"### Observation {g['number']} — {RATING_LABEL.get(g.get('materiality'), g.get('materiality'))}", ""]
    if g.get("in_short"):
        out += [f"**In short:** {_md_safe(g['in_short'])}", ""]
    out += [_md_safe(first.get("note")), "",
           f"**Materiality: {RATING_LABEL.get(g.get('materiality'), g.get('materiality'))}** "
           f"(rated once). {_md_safe(g.get('basis'))}", ""]
    out += _remedy_line(g.get("remedy"))
    seen, items = set(), []
    for u in g["items"]:
        e = u.get("evidence") or {}
        k = (e.get("document"), json.dumps(e.get("lines")), e.get("form"))
        if k not in seen:
            seen.add(k)
            items.append(e)
    srcs = sorted({u.get("claim_source") for u in g["items"]} - {None})
    out += ["Evidence:", ""] + (_evidence(items) or ["- (none)"])
    out += ["", f"Recorded while testing {', '.join(f'`{x}`' for x in srcs)}.", ""]
    return out


def grouped_sections(classes: Dict[str, List[Dict[str, Any]]]) -> Dict[str, set]:
    """The finding keys of each section whose findings may be grouped by
    subject (REPORT.md §6)."""
    return {"shown": {_key(f) for f in classes["shown"]},
            "buyer": {_key(f) for f in classes["asked_buyer"]},
            "practice": {_key(f) for f in classes["asked_practice"]}}


def ungrouped_allowed(classes: Dict[str, List[Dict[str, Any]]]) -> set:
    """The keys of the grouped sections' withdrawn findings, which the
    groups may leave out: a withdrawn finding is counted nowhere."""
    return {_key(f) for cls in ("shown", "asked_buyer", "asked_practice")
            for f in classes[cls] if withdrawn(f)}


def _in_groups(rows: List[Tuple[Dict[str, Any], Optional[Dict[str, Any]], str]],
               groups: Optional[List[Dict[str, Any]]], render_one) -> List[str]:
    """`rows` (finding, rating, field), already in order, under their
    subject headings when `groups` is given: each group keeps the rows' order,
    and the groups follow the position of their first row."""
    if not groups:
        out = []
        for row in rows:
            out += render_one(row, 3)
        return out
    pos = {_key(r[0]): i for i, r in enumerate(rows)}
    groups = sorted(groups, key=lambda g: min(pos.get(k, 1 << 30) for k in g["findings"]))
    by_key = {_key(r[0]): r for r in rows}
    out = []
    for g in groups:
        out += [f"### {_md_safe(g['title'])}", ""]
        for k in sorted(g["findings"], key=lambda k: pos.get(k, 1 << 30)):
            out += render_one(by_key[k], 4)
    # Only a withdrawn finding can be in no group (valid_groups).
    grouped = {k for g in groups for k in g["findings"]}
    rest = [r for r in rows if _key(r[0]) not in grouped]
    if rest:
        out += ["### Withdrawn by the practice", ""]
        for row in rest:
            out += render_one(row, 4)
    return out


def coverage(merged: Dict[str, Any], ratings: Dict[str, Any],
             classes: Dict[str, List[Dict[str, Any]]]) -> List[str]:
    """The figures, computed. Never written by the agent."""
    fig = merged.get("figures") or {}
    rf = ratings.get("figures") or {}
    out = ["| claim source | claims | findings | checked | files read | "
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
            f"Unsettled and unexamined, by exposure: "
            + (", ".join(f"{k} {n}" for k, n in sorted((rf.get('exposure') or {}).items()))
               or "none") + f"; of which {len(classes['seller_unsettled'])} about the seller.",
            f"Unexamined: {len(classes['not_examined'])} claim(s)."]
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


def awaiting_practice(f: Dict[str, Any]) -> bool:
    """The check did not uphold this finding and the practice has not
    retained it. Such a finding stays in its section, with the check's
    outcome, but is not named among the material findings or in the
    conclusion until the practice decides (Bruce, 2026-10-14: a finding the
    check rejected was the answer page's one contradiction)."""
    return ((f.get("review") or {}).get("outcome") == "does_not_hold"
            and (f.get("disposition") or {}).get("disposition") not in ("retained", "withdrawn"))


def withdrawn(f: Dict[str, Any]) -> bool:
    return (f.get("disposition") or {}).get("disposition") == "withdrawn"


def held_back(classes: Dict[str, List[Dict[str, Any]]],
              by_m: Dict[str, Dict[str, Any]]) -> List[Dict[str, Any]]:
    """The findings rated material or decisive that awaiting_practice holds
    back from the summary and the conclusion, in rating order."""
    fs = classes["shown"] + [f for k in ("asked_buyer", "asked_practice")
                             for f in classes.get(k) or [] if ms.rateable(f)]
    return [f for f in _ordered(fs, by_m, "materiality")
            if awaiting_practice(f)
            and (by_m.get(_key(f)) or {}).get("materiality") in ("material", "decisive")]


def key_findings(classes: Dict[str, List[Dict[str, Any]]],
                 by_m: Dict[str, Dict[str, Any]],
                 observations: Optional[List[Dict[str, Any]]] = None) -> List[str]:
    """The shown findings rated material or decisive, one line each, in
    rating order: the rating and the rater's plain account of the finding
    (MATERIALITY §5b), which a buyer can read on its own (Bruce,
    2026-10-14). The gap is under the finding; the summary carried a second
    copy of every gap until 2026-10-14, a page and a half of it. Computed, so the executive
    summary lists what the ratings say and not what a writer chose to
    mention."""
    rows = []
    asked = [f for k in ("asked_buyer", "asked_practice") for f in classes.get(k) or []
             if ms.rateable(f)]
    for f in _ordered(classes["shown"] + asked, by_m, "materiality"):
        r = by_m.get(_key(f))
        if not r or r.get("materiality") not in ("material", "decisive") \
                or awaiting_practice(f) or withdrawn(f):
            continue
        adj = f.get("adjudication") or {}
        head = f"- **{f.get('claim_source')}, claim {f.get('claim_id')}** ({RATING_LABEL.get(r.get('materiality'))}): "
        if r.get("in_short"):
            rows.append(head + _md_safe(r["in_short"]))
        else:                       # a rating made before MATERIALITY §5b
            rows.append(head + f"{_md_safe(f.get('statement') or f.get('quote'))} — "
                        f"{_verdict_short(adj.get('verdict'))}"
                        + (f" — *{KNOWLEDGE_MARK}*" if _from_knowledge(f) else ""))
    for g in observations or []:
        if g.get("materiality") in ("material", "decisive"):
            rows.append(f"- **Observation {g['number']}** ({RATING_LABEL[g['materiality']]}; "
                        f"the seller claimed nothing about it): "
                        f"{_md_safe(g.get('in_short') or g['items'][0].get('note'))}")
    return rows


#: Whose questions a question source holds, by its kind (asked_by).
ASKED_BY_WORDS = {"buyer": "the buyer's questions", "standard": "the practice's standard questions",
                  "practice": "the practice's questions"}


def _front_matter(engagement: str, dates: List[str], revs: List[str],
                  sources: List[str], tiered: bool = False,
                  asked: Optional[Dict[str, str]] = None) -> Tuple[List[str], List[str]]:
    """(the title and the materials' date and version; what the document
    is, the assurance given, and who is responsible for what). Fixed text
    from the record; defines the terms it uses at first use. The second part
    follows the answer and the summary (Bruce, 2026-10-14). `tiered`: some
    claims were listed and not tested. `asked`: question sources by claim
    source, with their kind; they are named apart from the seller's claim
    sources, as statements the seller did not make."""
    asked = asked or {}
    when = ", ".join(dates) if dates else "an undated run"
    rev = (" at commit " + ", ".join(r[:12] for r in revs)) if revs else ""
    src = (", ".join(f"`{s}`" for s in sources if s not in asked)
           or "the claim sources named by the engagement")
    questions = ", ".join(f"`{s}` ({ASKED_BY_WORDS.get(k, 'questions')})"
                          for s, k in asked.items())
    return [
        f"# Claims review — {engagement}", "",
        f"Materials as of {when}{rev}. Claim sources: {src}."
        + (f" Questions tested: {questions}." if questions else ""), ""], [
        "## What this document is", "",
        "A claims review: "
        + ("every assertion the seller makes in the claim sources is listed, "
           "and each is rated by what it would change for the buyer if it were "
           "untrue. The claims the buyer's decision rests on are tested, one by "
           "one, against the materials the seller supplied, and each is "
           "reported with the evidence that settles it; the others are listed "
           "as not tested, with the reason. " if tiered else
           "the assertions the seller makes in the claim sources are tested, "
           "one by one, against the materials the seller supplied, and each is "
           "reported with the evidence that settles it. ")
        + "A *claim source* is a document in which "
        "the seller asserts things about the target; the *materials* are "
        "everything supplied, including the claim sources, source code and "
        "configuration. The review examined what was supplied and nothing "
        "else."
        + (" The review also tested statements the seller did not make: "
           "questions the buyer raised and questions the practice asks, each "
           "written as a statement about the target and tested like a claim. A "
           "gap in one is a finding about the target, not about what the seller "
           "said." if asked else ""), "",
        "**The assurance given is limited.** The review reports what the "
        f"materials show about each{' tested' if tiered else ''} claim. It did not perform procedures "
        "beyond examining the materials, so a claim the materials cannot "
        "settle is reported as unsettled, not as false, and the document "
        "states no overall conclusion on the target or the transaction. An "
        "overall assessment is given only against thresholds the buyer has "
        "stated; where this document carries none, none was recorded.", "",
        "**Responsibilities.** The seller made the claims and was not "
        "consulted; the seller has not confirmed the review's reading of any "
        "claim. "
        + ("The buyer and the practice wrote the questions tested. " if asked else "")
        + "The practice performed the review under its written method, "
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
        "would change for this transaction. A *check* is an independent "
        "second pass over each finding's evidence, and a *retest* is a "
        "second check, blind to the first, on the findings the check "
        "questioned. Each finding ends with the check's outcome; where the "
        "check did not uphold a finding, a *practice review* line may follow, "
        "saying whether the practice retained it, withdrew it, or left it unresolved. "
        "A withdrawn finding is shown where it was, marked, and counted nowhere. A finding "
        "marked *covered by* names a wider claim of which this one is a "
        "part, with that claim's verdict beside it; the two were tested "
        "separately and each verdict is its own.", "",
        "**Evidence, in short and in full.** A finding's evidence here gives "
        "the file and lines, the first words of the quoted text and what it "
        "shows; for a search, what it found. The quoted text in full, the "
        "reasoning behind each derived fact, and every search with what was "
        "searched and the files it named are in the companion *evidence* "
        "document, under the same claim source and claim id.", "",
        "**Verdicts**, and the class each puts a claim in:", "",
        "| verdict | meaning | class |", "|---|---|---|",
        "| contradicted | " + VERDICT_WORDS["contradicted"] + " | shown |",
        "| partial | " + VERDICT_WORDS["partial"] + " | shown |",
        "| real_with_caveat | " + VERDICT_WORDS["real_with_caveat"] + " | shown |",
        "| unverifiable | " + VERDICT_WORDS["unverifiable"] + " | unsettled, or unexamined |",
        "| real | the materials bear the claim out | holds |", "",
        "**The three classes.** *Shown* findings are gaps the review "
        "demonstrated in the materials; they are the only findings that count "
        "against the seller, and each carries a *materiality* rating. "
        "*Unsettled* claims are ones the supplied materials cannot settle; "
        "nothing was found against them, and each carries an *exposure* "
        "rating. *Unexamined* claims are unsettled claims whose searches "
        "named files the engagement did not open; they are listed apart and "
        "are the first thing a further pass would settle. Unsettled claims "
        "about the seller's own conduct or services — a promise to maintain, "
        "a hosted service, commercial terms — are listed under their own "
        "heading within the unsettled class, each with the question the "
        "review puts to the seller. Claims that hold are listed after the "
        "three classes.", "",
        "**Where the claim source points.** An unsettled claim whose "
        "materials were not supplied — a listing, a published package, a "
        "hosted service — shows any link the claim source itself gives for "
        "it. The practice did not follow those links and says nothing about "
        "what is there; the buyer can look.", "",
        "**Knowledge of a dependency.** Some verdicts depend on what a "
        "third-party library does, when the supplied materials show only that "
        "the library is called and which version. The practice may then state "
        "that behaviour from its knowledge of the library. The evidence line "
        "says so, and the finding is marked *" + KNOWLEDGE_MARK + "*. The check "
        "takes such a statement as written: the materials do not show it, and "
        "nothing in this report confirms it. A finding without the mark may "
        "still rely on such knowledge without saying so.", "",
        "**Ratings.** Materiality and exposure use one scale, read for a gap "
        "the review showed or for a claim assumed false:", "",
        "| rating | meaning |", "|---|---|",
        "| not_material | " + RATING_WORDS["not_material"] + " |",
        "| material | " + RATING_WORDS["material"] + " |",
        "| decisive | " + RATING_WORDS["decisive"] + " |", "",
        "Materiality and exposure are never added together: one counts what "
        "the review showed, the other what rests on what it could not settle.", "",
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
    dropped, _ = rs.dropped_questions(prose, len(merged.get("questions") or []))
    groups, _ = rs.valid_groups(prose, grouped_sections(classes),
                                ungrouped_allowed(classes))
    by_q = _questions_by_key(merged, dropped)
    observations = observation_groups(merged, ratings)
    covered = record.get("covered") or {}
    verdicts = {_key(f): (f.get("adjudication") or {}).get("verdict")
                for f in merged.get("findings") or []}
    runs = merged.get("runs") or []
    dates = sorted({(r.get("captured_at_utc") or "")[:10] for r in runs} - {""})
    revs = sorted({r.get("target_rev") or "" for r in runs} - {""})
    sources = [r.get("claim_source") for r in runs if r.get("claim_source")]
    untested = record.get("not_tested") or []
    # A claim source whose every claim was listed and not tested has no run.
    unrun = sorted({c.get("claim_source") for c in untested} - set(sources) - {None})
    sources = sources + unrun
    asked = {f["claim_source"]: f["asked_by"] for f in merged.get("findings") or []
             if f.get("asked_by")}
    out, about = _front_matter(engagement or merged.get("engagement") or "engagement",
                               dates, revs, sources, tiered=bool(untested), asked=asked)

    out += ["## The transaction", ""]
    out += ["  \n".join(transaction.strip().splitlines()), ""] if transaction else [
        "The engagement states nothing about the transaction. Ratings assume "
        "a buyer paying a price that assumes every claim holds.", ""]
    out += ["**The buyer's thresholds.**  \n" + (
        "  \n".join(thresholds.strip().splitlines()) if thresholds else
        "None recorded. Ratings are read against the rating scale alone, not "
        "against what the buyer said would change the price or end the deal."), ""]

    # THE CONCLUSION IS OPT-IN AND CONDITIONAL. Present only when the
    # engagement asked for one and the buyer's thresholds are recorded; the
    # front matter says an overall assessment is given only against stated
    # thresholds, and this is where it is given. It comes first, right after
    # the thresholds it is read against (Bruce, 2026-10-14).
    if conclusion and thresholds:
        out += ["## Against the buyer's thresholds", "",
                "The engagement asked for a conclusion. It is read against "
                "the buyer's thresholds stated above and against nothing "
                "else; it counts what the review showed, not what it could "
                "not settle.", ""] + _slot("conclusion", prose)

    out += ["## Executive summary", ""] + _slot("summary", prose)
    kf = key_findings(classes, by_m, observations)
    out += ["**The material findings**, ordered by rating — each is set out in "
            "full under *What the review showed*"
            + (", or under the questions asked" if classes["asked_buyer"] or classes["asked_practice"]
               else "")
            + (", or under *Found without a claim*" if any(
                g.get("materiality") in ("material", "decisive") for g in observations) else "")
            + ":", ""]
    out += kf + [""] if kf else ["No finding was rated material or decisive.", ""]
    hb = held_back(classes, by_m)
    if hb:
        out += [("Not listed: " + ", ".join(f"{f.get('claim_source')}, claim {f.get('claim_id')}" for f in hb)
                 + ", rated " + " or ".join(sorted({RATING_LABEL[(by_m.get(_key(f)) or {}).get("materiality")]
                                                    for f in hb}, reverse=True)) + ". "
                 "The check did not uphold " + ("these findings" if len(hb) > 1 else "this finding")
                 + " and the practice has not yet decided " + ("them" if len(hb) > 1 else "it")
                 + "; each is set out in its section with the check's outcome."), ""]

    out += about

    listed = {s: sum(1 for c in untested if c.get("claim_source") == s) for s in sources}
    out += ["## Scope and approach", "",
            "| claim source | " + ("tested | not tested" if untested else "claims")
            + " | findings | checked | files read | gathering legs | model |",
            "|---|---|---|---|---|---|---|" + ("---|" if untested else "")]
    for r in runs:
        out.append(f"| {r.get('claim_source')} | {r.get('claims')} | "
                   + (f"{listed.get(r.get('claim_source'), 0)} | " if untested else "")
                   + f"{r.get('findings')} | {'yes' if r.get('reviewed') else 'no'} | "
                   f"{r.get('files_read', '')} | {r.get('gathering_legs', '')} | "
                   f"{r.get('resolved_model') or ''} |")
    for s in unrun:
        out.append(f"| {s} | 0 | {listed[s]} | 0 | | | | |")
    out += ["", ("*Tested* counts the seller's assertions, as enumerated from the "
                 "claim source, that the review tested, one finding each. *Not "
                 "tested* counts those rated before testing as claims whose being "
                 "untrue in any way would not lead the buyer to reopen the price "
                 "or the terms; they are listed with their reasons in the second "
                 "appendix, and a claim source with nothing to test has no other "
                 "figures. " if untested else
                 "*Claims* are the seller's assertions as enumerated from the "
                 "claim source, one finding each. ")
                + "".join(f"`{src}` holds statements the {who} asked, not assertions of the "
                          "seller; its figures count those statements. "
                          for src, who in sorted({(f.get("claim_source"),
                                                   "buyer" if f["asked_by"] == "buyer" else "practice")
                                                  for f in merged.get("findings") or []
                                                  if f.get("asked_by")}))
                + "*Files read* counts the "
                "target's files the practice opened while gathering evidence, "
                "over the number of *gathering legs* it took. *Checked* says "
                "whether the independent check ran on that claim source.", ""]
    subs = sorted({m for r in runs for m in ((r.get("materials") or {}).get("submodules") or [])})
    if subs:
        out += ["**Not supplied:** the materials name "
                + ("one submodule" if len(subs) == 1 else f"{len(subs)} submodules")
                + " kept elsewhere by the seller — " + ", ".join(f"`{m}`" for m in subs)
                + ". Nothing under them was examined; a claim whose evidence "
                "would sit there is reported as unsettled, with that reason.", ""]
    out += _slot("scope_note", prose)

    out += _how_to_read()

    def one(row, level):
        f, r, field = row
        return _finding(f, r, field, by_q.get(_key(f)), covered.get(_key(f)), verdicts, level)

    out += ["## What the review showed", ""] + _slot("shown_note", prose)
    out += _in_groups([(f, by_m.get(_key(f)), "materiality")
                       for f in _ordered(classes["shown"], by_m, "materiality")],
                      groups.get("shown"), one)
    if not classes["shown"]:
        out += ["No finding showed a gap.", ""]
    if observations:
        out += ["## Found without a claim", "", OBSERVATIONS_LINE, ""]
        for g in observations:
            out += _observation(g)
    out += ["## Unsettled claims", ""] + _slot("unsettled_note", prose)
    for f in _ordered(classes["unsettled"], by_e, "exposure"):
        out += _finding(f, by_e.get(_key(f)), "exposure", by_q.get(_key(f)), covered.get(_key(f)), verdicts)
    if not classes["unsettled"]:
        out += ["None.", ""]
    if classes["seller_unsettled"]:
        out += ["## Unsettled claims about the seller", "", SELLER_LINE, ""]
        for f in _ordered(classes["seller_unsettled"], by_e, "exposure"):
            out += _finding(f, by_e.get(_key(f)), "exposure", by_q.get(_key(f)), covered.get(_key(f)), verdicts)
    if classes["not_examined"]:
        out += ["## Unexamined claims", ""] + _slot("not_examined_note", prose)
        for f in _ordered(classes["not_examined"], by_e, "exposure"):
            out += _finding(f, by_e.get(_key(f)), "exposure", by_q.get(_key(f)), covered.get(_key(f)), verdicts)
    for cls, (title, line) in ASKED_LINES.items():
        if classes[cls]:
            out += [f"## {title}", "", line, ""]
            out += _in_groups(_asked_ordered(classes[cls], by_m, by_e),
                              groups.get("buyer" if cls == "asked_buyer" else "practice"), one)
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
        # Numbered while the passages are being written, so the writer can
        # name the questions that repeat one another (REPORT.md §6); in the
        # document, a repeat is left out.
        out += ["## Questions for the seller", ""]
        out += [(f"{i}. " if prose is None else "- ") + f"({q.get('claim_source')}"
                + (f", claim {q['claim_id']}" if q.get("claim_id") is not None else "")
                + f") {q.get('question')}"
                for i, q in enumerate(merged["questions"], 1) if i not in dropped] + [""]
    if merged.get("unclaimed") and not observations:
        out += ["## Observations the seller did not claim", ""]
        for u in merged["unclaimed"]:
            out.append(f"- ({u.get('claim_source')}) {u.get('note')}")
            out += ["  " + x for x in _evidence([u.get("evidence") or {}])]
        out.append("")

    out += ["## Coverage", "", ("Every claim received one finding. " if not untested else
            f"Every claim that was tested received one finding. {len(untested)} further "
            f"claim(s) were rated before testing as claims whose being untrue in any way "
            f"would not lead the buyer to reopen the price or the terms; they are listed in the "
            f"second appendix, each with the reason, and were not tested. ")
            + "By verdict:", "", "| verdict | claims |", "|---|---|"]
    counts = {"contradicted": 0, "partial": 0, "real_with_caveat": 0,
              "unverifiable": 0, "real": 0}
    for f in merged.get("findings") or []:
        v = (f.get("adjudication") or {}).get("verdict")
        counts[v] = counts.get(v, 0) + 1
    for v, n in counts.items():
        out.append(f"| {_verdict_short(v)} | {n} |")
    rf = ratings.get("figures") or {}
    out += ["",
            "Gaps by materiality: "
            + (", ".join(f"{RATING_LABEL.get(k, k)} {n}" for k, n in sorted((rf.get('materiality') or {}).items()))
               or "none") + ". Unsettled claims by exposure: "
            + (", ".join(f"{RATING_LABEL.get(k, k)} {n}" for k, n in sorted((rf.get('exposure') or {}).items()))
               or "none") + f", of which {len(classes['seller_unsettled'])} about the seller. "
            f"Unexamined: {len(classes['not_examined'])}."
            + (f" Of all the findings, {len(classes['asked_buyer'])} are on statements the "
               f"buyer asked and {len(classes['asked_practice'])} on statements the practice "
               "asked; they are counted above by verdict and set out under the questions "
               "asked." if classes["asked_buyer"] or classes["asked_practice"] else ""), ""]
    gone = [f for f in merged.get("findings") or [] if withdrawn(f)]
    if gone:
        out += ["Withdrawn by the practice, and counted above as the review recorded them: "
                + ", ".join(f"{f.get('claim_source')}, claim {f.get('claim_id')}" for f in gone) + ".", ""]
    unopened = sorted({f for r in runs for f in (r.get("unopened_candidates") or [])})
    if unopened:
        out += [f"Files the searches named that were not opened ({len(unopened)}):", ""]
        out += [f"- `{f}`" for f in unopened] + [""]

    out += ["## Limitations", ""] + _slot("limitations", prose)
    out += ["The seller was not consulted and has not confirmed the review's "
            "interpretation of its claims. Every verdict and "
            "rating is defined by the practice's method at the version each "
            "run received, which is retained with the record; a later method "
            "may define a term differently.", ""]

    out += ["## Appendix — every claim and its verdict", "",
            "The claim surface as the review froze it, in document order, with "
            "the verdict each claim received and, where rated, its "
            "materiality or exposure.", "",
            "| source | id | lines | claim | verdict | rating | covered by |",
            "|---|---|---|---|---|---|---|"]
    for f in sorted(merged.get("findings") or [],
                    key=lambda x: (x.get("claim_source") or "", x.get("claim_id") or 0)):
        k = _key(f)
        r = by_m.get(k) or by_e.get(k) or {}
        rating = r.get("materiality") or r.get("exposure") or ""
        q = _md_safe(f.get("quote"))
        c = covered.get(k) or {}
        out.append(f"| {f.get('claim_source')} | {f.get('claim_id')} | "
                   f"{_lines_bare(f.get('lines'))} | {q[:120]} | "
                   f"{(f.get('adjudication') or {}).get('verdict')} | {rating} | "
                   f"{(c.get('source') + ' ' + str(c.get('id'))) if c else ''} |")
    if untested:
        out += ["", "## Appendix — claims listed, not tested", "",
                "Rated before testing, against the transaction and the buyer's "
                "thresholds, and not tested. Tier 2: a detail that a person "
                "setting up or operating the software would act on and would "
                "meet in use, whose failure on its own changes nothing for the "
                "buyer. Tier 3: neither. The buyer may ask "
                "for any of them to be tested.", "",
                "| source | id | lines | claim | tier | reason |",
                "|---|---|---|---|---|---|"]
        for c in sorted(untested, key=lambda x: (x.get("claim_source") or "", x.get("id") or 0)):
            out.append(f"| {c.get('claim_source')} | {c.get('id')} | "
                       f"{_lines_bare(c.get('lines'))} | {_md_safe(c.get('quote'))[:120]} | "
                       f"{c.get('tier')} | {_md_safe(c.get('tier_basis'))} |")
    if record.get("composition"):
        from workflowsv2.composition import appendix
        out += [""] + appendix.render(record["composition"])
        out += [""] + appendix.render_secrets(record["composition"])
    return "\n".join(out).rstrip() + "\n"


def evidence_document(record: Dict[str, Any], engagement: Optional[str] = None) -> str:
    """evidence.md: every finding's and every observation's evidence in
    full, the companion to the report, for a reader who checks a finding
    against the materials. Findings in claim-source and claim-id order."""
    merged, ratings = record["merged"], record["ratings"]
    name = engagement or merged.get("engagement") or "engagement"
    out = [f"# Evidence — claims review — {name}", "",
           "The evidence for every finding of the claims review, in full: each quoted "
           "line as it appears in the materials, the reasoning behind each derived "
           "fact, and every search, with what was searched and the files it named. "
           "The report shows the same evidence in short. Findings are in order of "
           "claim source and claim id.", ""]
    for f in sorted(merged.get("findings") or [],
                    key=lambda x: (x.get("claim_source") or "", x.get("claim_id") or 0)):
        out += [f"## {f.get('claim_source')}, claim {f.get('claim_id')}", "",
                f"{_md_safe(f.get('statement') or f.get('quote'))} — "
                f"{_verdict_short((f.get('adjudication') or {}).get('verdict'))}", ""]
        out += (_evidence(f.get("evidence"), full=True) or ["- (none)"]) + [""]
    for g in observation_groups(merged, ratings):
        out += [f"## Observation {g['number']}", "", _md_safe(g["items"][0].get("note")), ""]
        out += _evidence([u.get("evidence") or {} for u in g["items"]], full=True) + [""]
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
