"""The record of one claim, served whole by the consultation's `claim` action.

WHY THIS EXISTS (2026-09-16). A post-delivery question about a named claim
("why is claim 44 rated material?") was answered correctly, but the turn spent
42 of its 65 seconds in 18 inspect-subagent steps locating and reading one
block of materiality.json, whose path is fixed when the session starts. A
second question ("do the cited lines still say that?") spent 60 seconds in two
subagent runs re-reading six citations. Both are lookups over a record that is
small, structured and fully known at session start, so this module builds
that lookup once and the ReAct loop reaches it in one step.

WHAT ONE RECORD HOLDS, per (claim source, claim id): the finding as delivered
(merged.json: quote, lines, statement, verdict, gap, every evidence item,
the check's outcome, the correction after the check), the rating and its
basis (materiality.json, both raters' bases included), the lines of report.md
that carry the claim, and the questions put to the seller about it. Every
citation is also resolved against the materials AT CALL TIME: the cited file
is read then and the quote checked at the cited lines by the same rule the
audit's output check uses (`schemas.quote_at`). Reading at call time rather
than at session start costs milliseconds and never serves a stale answer.

A CLAIM LISTED AND NOT TESTED (2026-09-19). A claim rated tier 2 or 3 before
the freeze has no finding (TIERS.md §1); it is on the frozen surface and in
the report's second appendix. Until this date the action answered "no claim N"
for one, which is false, and with no `source` given it could return the
tested claim of another source that has the same id. The record now holds
those claims too, read from the frozen surfaces the way the report reads them
(`audit_report.render.not_tested`), and returns the claim's words, its tier
and the reason for the tier, and says that it was not tested.

The record text is for the model, which then answers the client in the
client's words (CONTINUATION.md §6); labels here use those words where the
method defines them ("check", not "review").

REGISTRATION. The ReAct loop's discovered-tool registry (src/chat/tools.py)
is filled from src/tools/ at construction. `register` adds this action to a
built loop's registry and module cache directly, so the action exists for the
consultation and for nothing else: it needs a bound record, and a tool under
src/tools/ would appear in every character's catalog.
"""
from __future__ import annotations

import json
import logging
import re
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

from workflowsv2.claims_audit import schemas

logger = logging.getLogger("continuation.record")

TOOL_NAME = "claim"
#: Claims per call. One record is several thousand characters and the
#: observation goes into the prompt.
MAX_CLAIMS = 4
#: When a citation no longer resolves at its lines, the record shows what is
#: at those lines now, up to this many lines.
CURRENT_LINES_CAP = 40

DESCRIPTION = (
    "the complete record of one to four of this engagement's claims, by id: "
    "the claim's words and lines, the statement tested, the verdict and gap "
    "as delivered, the correction after the check if there was one, the "
    "check's outcome, every evidence item in full (a citation with its quote "
    "and whether that quote is at those lines in the materials now; a search "
    "with what was performed and what it found), the rating with its basis "
    "and each rater's basis, the lines of report.md that carry the claim, and "
    "the questions put to the seller about it. For a claim that was listed and "
    "not tested: its words and lines, its statement, its tier and the reason "
    "for the tier, and that no finding exists. Use it first for any question "
    "about a named claim. `inspect` is for the record beyond the findings "
    "(traces, run_meta, the method); `inspect_external` for the materials "
    "beyond the cited lines.")
ARGS = {
    "ids": "list of claim ids, e.g. [44, 45]; at most 4 per call",
    "source": "the claim source document, e.g. README.md; may be omitted "
              "when the engagement has one claim source",
}

_HEADING = re.compile(r"^(#{2,3}) (.*)$")
_CLAIM_HEADING = re.compile(r"^### (?P<src>.+?), claim (?P<id>\d+)(?: — |$)")
_TABLE_ROW = re.compile(r"^\| (?P<src>[^|]+?) \| (?P<id>\d+) \|")

Key = Tuple[str, int]


def report_index(report_text: str) -> Dict[Key, List[str]]:
    """Where report.md carries each claim: the line range of its finding
    section, and each row of a table that lists it, named by the section
    the lines sit under."""
    out: Dict[Key, List[str]] = defaultdict(list)
    lines = report_text.splitlines()
    section = ""
    open_span: Optional[Tuple[Key, int, str]] = None

    def close(last: int) -> None:
        nonlocal open_span
        if open_span:
            key, start, sec = open_span
            out[key].append(f"lines {start}-{last}, the finding's section under '{sec}'")
            open_span = None

    for n, line in enumerate(lines, 1):
        m = _HEADING.match(line)
        if m:
            close(n - 1)
            if m.group(1) == "##":
                section = m.group(2).strip()
            cm = _CLAIM_HEADING.match(line)
            if cm:
                open_span = ((cm.group("src"), int(cm.group("id"))), n, section)
            continue
        rm = _TABLE_ROW.match(line)
        if rm:
            out[(rm.group("src").strip(), int(rm.group("id")))].append(
                f"line {n}, a row of the table under '{section}'")
    close(len(lines))
    return out


def _indent(text: Any, prefix: str = "      ") -> str:
    return "\n".join(prefix + ln for ln in str(text or "").splitlines()) or prefix


class ClaimRecord:
    """Every claim of one merged run, and the materials it was checked against."""

    def __init__(self, merged: Dict[str, Any], merged_dir: Path, target: Path) -> None:
        self.target = Path(target)
        self.findings: Dict[Key, Dict[str, Any]] = {}
        for f in merged.get("findings") or []:
            self.findings[(str(f.get("claim_source")), int(f.get("claim_id")))] = f
        self.ratings: Dict[Key, Dict[str, Any]] = {}
        mp = merged_dir / "materiality.json"
        if mp.is_file():
            m = json.loads(mp.read_text(encoding="utf-8"))
            for kind in ("ratings", "exposures"):
                for r in m.get(kind) or []:
                    self.ratings[(str(r.get("claim_source")), int(r.get("claim_id")))] = r
        self.questions: Dict[Key, List[str]] = defaultdict(list)
        for q in merged.get("questions") or []:
            self.questions[(str(q.get("claim_source")), int(q.get("claim_id")))].append(
                str(q.get("question") or ""))
        rp = merged_dir / "report.md"
        self.report = report_index(rp.read_text(encoding="utf-8")) if rp.is_file() else {}
        # The materials' paths, for resolving a cited document the way the
        # audit does (a bare filename resolves when unique). Contents are
        # read at call time, so the index holds keys only.
        self._docs: Dict[str, List[str]] = {
            rel: [] for rel in schemas.corpus_view(self.target)["materials"]}
        # Claims listed and not tested, from the frozen surfaces. A key that
        # has a finding is a tested claim, whatever a later surface says.
        from workflowsv2.audit_report.render import not_tested
        self.not_tested: Dict[Key, Dict[str, Any]] = {}
        for c in not_tested(merged_dir):
            key = (str(c.get("claim_source")), int(c.get("id")))
            if key not in self.findings:
                self.not_tested[key] = c
        # A listed claim the practice tested after delivery (supplement.py):
        # only a result a person approved, and only one made for this run.
        from workflowsv2.claims_audit.supplement import records
        self.supplements: Dict[Key, Dict[str, Any]] = {}
        for rec in records(merged_dir.resolve().parents[1]):
            if rec.get("approved") and not rec.get("error") and rec.get("delivered_run") == merged_dir.resolve().name:
                self.supplements[(str(rec["claim_source"]), int(rec["claim"]["id"]))] = rec
        self.sources = sorted({k[0] for k in self.findings} | {k[0] for k in self.not_tested})

    # -- one citation, against the materials now ---------------------------

    def citation_now(self, item: Dict[str, Any]) -> str:
        """What the materials hold at a citation's lines at this moment."""
        key, why = schemas.resolve_document(self._docs, item.get("document"))
        if key is None:
            return f"in the materials now: {why}"
        lines = item.get("lines")
        if (not isinstance(lines, list) or len(lines) != 2
                or not all(isinstance(n, int) for n in lines) or lines[0] < 1
                or lines[1] < lines[0]):
            return f"in the materials now: lines {lines!r} is not a range"
        lo, hi = lines
        try:
            body = (self.target / key).read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError as e:
            logger.warning("record: unreadable %s (%s)", key, e)
            return f"in the materials now: {key} could not be read ({e})"
        if hi > len(body):
            return (f"in the materials now: {key} has {len(body)} lines, "
                    f"fewer than the cited {lo}-{hi}")
        quote = item.get("quote")
        if not isinstance(quote, str) or not schemas._norm(quote):
            return f"in the materials now: no quote to check at {key}:{lo}-{hi}"
        status, detail = schemas.quote_at(body, lo, hi, quote)
        if status == "exact":
            return (f"in the materials now: the quote is at {key}:{lo}-{hi}; "
                    f"the quote above is what those lines hold now")
        if status == "joined":
            return (f"in the materials now: each line of the quote is within "
                    f"{key}:{lo}-{hi}, with lines between them left out")
        if status == "prefixed":
            return (f"in the materials now: the quote is at {key}:{lo}-{hi} "
                    f"once its copied line-number prefixes are stripped")
        head = ("the quote is in the file but not at those lines"
                if status == "elsewhere" else
                f"the quote is not in the file; first part not found: {(detail or '')[:80]!r}")
        shown = body[lo - 1:min(hi, lo - 1 + CURRENT_LINES_CAP)]
        more = "" if hi - lo + 1 <= CURRENT_LINES_CAP else f"\n        … {hi - lo + 1 - CURRENT_LINES_CAP} more lines"
        return (f"in the materials now: {head}. Lines {lo}-{hi} of {key} read now:\n"
                + _indent("\n".join(shown), "        ") + more)

    # -- one claim, as text ------------------------------------------------

    def text_not_tested(self, key: Key) -> str:
        c = self.not_tested[key]
        src, cid = key
        lines = c.get("lines")
        where = f"{src} lines {lines[0]}-{lines[1]}" if isinstance(lines, list) and len(lines) == 2 else src
        out = [f"{src} #{cid} — listed, not tested (tier {c.get('tier')})",
               f"  quote ({where}): {c.get('quote')}",
               f"  statement: {c.get('statement')}",
               f"  about: {c.get('about')}"]
        if c.get("implied_by") is not None:
            out.append(f"  implied by: {c.get('implied_by')}")
        out.append("  reason for the tier"
                   + (f" (set by {c.get('tier_by')})" if c.get("tier_by") else "") + ":\n"
                   + _indent(c.get("tier_basis"), "    "))
        out.append("  The review did not test this claim: there is no finding, no "
                   "evidence, no check and no rating for it.")
        refs = self.report.get(key) or []
        out.append("  in report.md: " + ("; ".join(refs) if refs else "not named"))
        rec = self.supplements.get(key)
        if rec:
            adj = (rec.get("finding") or {}).get("adjudication") or {}
            rev = rec.get("review") or {}
            obs = rev.get("adverse_observations") or []
            out.append(f"  Tested after delivery, on {str(rec.get('at'))[:10]}, by the practice. Not part of "
                       f"the delivered report, and not rated for materiality.")
            out.append(f"    verdict: {adj.get('verdict')}")
            if adj.get("gap"):
                out.append("    gap:\n" + _indent(adj.get("gap"), "      "))
            if adj.get("unresolved_because"):
                out.append(f"    unresolved because: {adj.get('unresolved_because')}")
            out.append("    check: " + ("holds" if rev.get("holds") else "does not hold")
                       + (f" (observations: {', '.join(map(str, obs))})" if obs else "")
                       + (f"; retest: {rev.get('standing')}" if rev.get("standing") else ""))
            out += ["  " + ln for ln in self._evidence_lines((rec.get("finding") or {}).get("evidence") or [])]
        return "\n".join(out)

    def _evidence_lines(self, ev: List[Dict[str, Any]]) -> List[str]:
        """A finding's evidence items as the record's text, each citation
        with what the materials hold at its lines now."""
        out = [f"  evidence ({len(ev)} items):"]
        for i, e in enumerate(ev, 1):
            form = e.get("form")
            if form == "citation":
                ln = e.get("lines")
                rng = f"{ln[0]}-{ln[1]}" if isinstance(ln, list) and len(ln) == 2 else repr(ln)
                out.append(f"    {i}. citation {e.get('document')}:{rng}")
                out.append("      quote:\n" + _indent(e.get("quote"), "        "))
                if e.get("shows"):
                    out.append(f"      shows: {e.get('shows')}")
                out.append("      " + self.citation_now(e))
            elif form == "search":
                out.append(f"    {i}. search ({e.get('kind')})")
                out.append(f"      performed: {e.get('performed')}")
                out.append(f"      result: {e.get('result')}")
                if e.get("candidates"):
                    out.append("      candidates: " + ", ".join(map(str, e["candidates"])))
            else:
                out.append(f"    {i}. {form or 'derived'}")
                for k in ("derivation", "consequence"):
                    if e.get(k):
                        out.append(f"      {k}: {e.get(k)}")
        return out

    def text(self, key: Key) -> str:
        if key in self.not_tested:
            return self.text_not_tested(key)
        f = self.findings[key]
        src, cid = key
        adj = f.get("adjudication") or {}
        rev = f.get("review") or {}
        out: List[str] = []
        lines = f.get("lines")
        where = f"{src} lines {lines[0]}-{lines[1]}" if isinstance(lines, list) and len(lines) == 2 else src
        out.append(f"{src} #{cid} — verdict: {adj.get('verdict')}")
        out.append(f"  quote ({where}): {f.get('quote')}")
        out.append(f"  statement tested: {f.get('statement')}")
        out.append(f"  about: {f.get('about')}")
        if f.get("implied_by") is not None:
            out.append(f"  implied by: {f.get('implied_by')}")
        if f.get("approved_by") is not None:
            out.append(f"  approved by: {f.get('approved_by')}")
        if adj.get("gap"):
            out.append("  gap:\n" + _indent(adj.get("gap"), "    "))
        if adj.get("unresolved_because"):
            out.append(f"  unresolved because: {adj.get('unresolved_because')}")
        if f.get("correction"):
            out.append("  correction after the check:\n" + _indent(f.get("correction"), "    "))
        if rev:
            obs = rev.get("adverse_observations") or []
            out.append(f"  check: {rev.get('outcome')}"
                       + (f" (observations: {', '.join(map(str, obs))})" if obs else ""))
        else:
            out.append("  check: none recorded")
        if f.get("citation_problems"):
            out.append("  citation problems recorded at delivery:\n"
                       + _indent("\n".join(map(str, f["citation_problems"])), "    "))
        out += self._evidence_lines(f.get("evidence") or [])
        r = self.ratings.get(key)
        if r:
            kind = "materiality" if "materiality" in r else "exposure"
            out.append(f"  rating: {kind} {r.get(kind)}"
                       + (f" (agreement {r.get('agreement')}" if r.get("agreement") else " (")
                       + (", borderline)" if r.get("borderline") else ", not borderline)"))
            out.append("    basis:\n" + _indent(r.get("basis"), "      "))
            samples = r.get("samples") or []
            if samples:
                out.append(f"    each rater's basis ({len(samples)}):")
                for j, s in enumerate(samples, 1):
                    out.append(f"      {j}. {s.get(kind)}: {s.get('basis')}")
        else:
            out.append("  rating: none recorded")
        refs = self.report.get(key) or []
        out.append("  in report.md: " + ("; ".join(refs) if refs else "not named"))
        qs = self.questions.get(key) or []
        if qs:
            out.append("  questions put to the seller:")
            for q in qs:
                out.append(f"    - {q}")
        return "\n".join(out)

    # -- the action --------------------------------------------------------

    def _keys_for(self, ids: Any, source: Any) -> Tuple[List[Key], Optional[str]]:
        if isinstance(ids, (int, str)):
            ids = [ids]
        if not isinstance(ids, list) or not ids:
            return [], "`ids` must be a list of claim ids, e.g. [44, 45]"
        try:
            wanted = [int(x) for x in ids]
        except (TypeError, ValueError):
            return [], f"`ids` must be integers; got {ids!r}"
        if len(wanted) > MAX_CLAIMS:
            return [], f"at most {MAX_CLAIMS} claims per call; asked for {len(wanted)}"
        src = str(source).strip() if source else None
        if src is None and len(self.sources) == 1:
            src = self.sources[0]
        keys: List[Key] = []
        for cid in wanted:
            if src is not None:
                if (src, cid) in self.findings or (src, cid) in self.not_tested:
                    keys.append((src, cid))
                else:
                    return [], (f"no claim {cid} under {src}; claim sources: "
                                f"{', '.join(self.sources)}")
            else:
                hits = [k for k in list(self.findings) + list(self.not_tested) if k[1] == cid]
                if len(hits) != 1:
                    return [], (f"claim {cid} is under {len(hits)} sources "
                                f"({', '.join(h[0] for h in hits) or 'none'}); "
                                f"give `source`")
                keys.append(hits[0])
        return keys, None

    def invoke(self, args: Dict[str, Any]) -> Dict[str, Any]:
        keys, problem = self._keys_for(args.get("ids"), args.get("source"))
        if problem:
            return {"status": "error", "text": problem}
        return {"status": "ok", "text": "\n\n".join(self.text(k) for k in keys)}


def register(loop: Any, record: ClaimRecord) -> None:
    """Add the `claim` action to a built ChatLoop: the catalog entry in its
    discovered-tool registry and the callable in its module cache, so the
    loop dispatches it like a tool under src/tools/."""
    loop._discovered_tools[TOOL_NAME] = {
        "description": DESCRIPTION, "args": dict(ARGS), "module_path": None, "body": ""}
    loop._tool_module_cache[TOOL_NAME] = SimpleNamespace(
        react_invoke=lambda args, **_kw: record.invoke(args))
    logger.info("claim record: %d claims under %s; %d material paths",
                len(record.findings), ", ".join(record.sources), len(record._docs))
