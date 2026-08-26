#!/usr/bin/env python3
"""Score a dataroom run against the answer key.

    python3 measure/fixtures/dataroom/score.py --world dataroom_1 --agent Jill
    python3 measure/fixtures/dataroom/score.py --world dataroom_1 --dry-run

Reports a VECTOR, and reports the highest difficulty tier reached rather
than a pass/fail — a fixture that everyone passes is not an instrument, and
the ladder is what stops this one saturating.

WHAT IS MECHANICAL AND WHAT IS NOT. Which documents were opened, how many
iterations ran, whether the word ceiling held, whether the key was read —
all read straight off the trace, no model involved. Deciding whether a
paragraph in the memo *is* finding P2 is a judgement about meaning, so it
runs through ONE LLM call on the pinned grader, held constant across runs
for the same reason `regrade.py` pins it: if the arm under test also did the
matching, a difference in matching quality would surface as the metric.

THE KEY IS READABLE BY THE AGENT. `inspect` is geofenced to the repo root
(176b4029), so `answer_key.md` sits inside reach. Rather than hide it, this
scorer looks for it in the trace and marks the run INVALID if it was opened.
That is a third outcome, not a zero: a run that read the answers measured
nothing, and reporting it as a bad score would be worse than reporting
nothing.
"""

from __future__ import annotations

import argparse
import json
import logging
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from measure.trace import load_turns                            # noqa: E402

logger = logging.getLogger("dataroom.score")

# Where a run's world lives. Same layout load_turns walks; named here so the
# subagent trace files can be found without a second convention.
SCEN = REPO / "scenarios"

CORPUS_DOCS = 9

# TWO DIFFERENT AXES, and conflating them was the first bug in this file.
#
# Axis 1 — PROVENANCE of the finding (the answer key's own grouping):
#   planted / derived / supported-but-unplanted. Tier 3 is credited and
#   NEVER penalised, because entries 4-5 of the original exchange were
#   discovered rather than seeded — an agent finding something real that
#   nobody planted is being correct, and a grader that calls that a false
#   positive is measuring the wrong thing.
TIER1 = ["P1", "P2", "P3"]                       # planted, must-find
TIER2 = ["F1", "F2"]                             # derived, planted
TIER3 = ["B1", "B2", "B3", "B4", "B5", "B6", "B7"]   # unplanted, supported

# Axis 2 — DIFFICULTY of noticing it (the README's ladder). This is the one
# that keeps the fixture from saturating: report the highest rung reached
# rather than pass/fail, and retire the fixture when every arm tops out.
#   stated         a fact sitting in one document
#   cross_document a seller claim contradicted by another document
#   derived        true only after arithmetic on stated figures
LADDER = {
    "stated": ["B3", "B4", "B5"],
    "cross_document": ["P1", "P2", "P3", "B1", "B2", "B6"],
    "derived": ["F1", "F2", "B7"],
}
RUNGS = ["stated", "cross_document", "derived"]

# From the key. Placement is scored against this, because for a one-pager
# the forced choice about what earns space IS the measurement.
SEVERITY = ["P2", "F2", "P3", "P1", "F1", "B3", "B2", "B1", "B7", "B5",
            "B4", "B6"]

FINDINGS_BRIEF = """\
P1 vocabulary_gap — seller claims "redundancy and automatic failover" (Doc 9);
   Doc 4 shows a single standard-1x dyno with Postgres on the SAME dyno, no
   read replicas. A dyno restart is not failover.
P2 operational_degradation — seller claims "daily automated database backups"
   (Doc 9); Doc 4 records backup failures for the last 21 days, last success
   2026-07-30, alerting none configured.
P3 structural_omission — seller claims all integrations are standard with no
   lock-in and never names DataEnrich.io (Docs 1/2/9); Doc 8 shows
   DataEnrich.io at $400/mo, 40% of features depend on it, 90-day termination,
   no fallback.
F1 derived — Stripe MRR $16,000 (Doc 5) vs three enterprise contracts at
   $8,000/mo paid by wire and not in Stripe (Doc 7): ~60% of revenue is
   unverifiable from the processor; wire revenue is 150% of Stripe MRR.
   (28% is NOT this finding.)
F2 derived — 30-day retention against 21 days of failures puts the last
   recoverable backup (2026-07-30) at total loss around 2026-08-29.
B1 "comprehensive test coverage" vs 12 unit tests all in test/utils/, 0
   integration, 0 payment-path, no CI/CD, no staging, no review.
B2 "we monitor uptime at 99.9%" vs Doc 4 "Uptime Monitor: None", while two
   customers hold 99.9% SLAs with credits.
B3 key person — dave is 73% of commits, 0 in 30 days, last commit 2026-07-30;
   DNS managed personally by dave with no secondary.
B4 SLA heterogeneity — ShopStream at 99.5% where the other two are 99.9%.
B5 legal — 15 accounts with no recorded ToS acceptance; 8 pilots expiring Q3
   2026 with no payment obligation.
B6 "scale horizontally, adding dynos is one-click" vs the co-located database,
   which adding dynos does not scale.
B7 concentration — 3 customers are ~60% of revenue; GlobalMart renews first
   (2026-11-01), inside the 90-day non-renewal notice window.
"""

_MATCH_SYS = """\
You are scoring a due-diligence memo against a fixed answer key. You are not \
judging quality, only identity: which of the key's findings does each of the \
memo's findings correspond to?

Here is the key:

{findings}

Read the memo. For EACH distinct finding the memo reports, in the order the \
memo presents them, output one entry:
- "id": the key id it matches (P1 P2 P3 F1 F2 B1..B7), or "unsupported", or \
"other". See the test below for which.

Use "unsupported" ONLY when the memo states a fact that contradicts the data \
room, or a fact that appears nowhere in it. Use "other" when the underlying \
facts are in the documents and the memo has drawn its own conclusion from \
them, even if you disagree with the conclusion, and when the finding is a \
real observation matching no key entry. If you are unsure which applies, use \
"other": "unsupported" is a claim about the memo inventing a fact, not about \
the memo reasoning differently from you.
- "quote": a short verbatim span from the memo identifying that finding.

Match on SUBSTANCE, not wording. A memo saying "the database shares a dyno \
with the app so restarting it is not real failover" is P1 whatever words it \
uses. Do NOT match on a topic being mentioned in passing — the memo must \
actually report it as a finding. If the memo mentions backups only to say \
they exist, that is not P2.

A finding may match only one id. Do not invent ids.

Output ONLY: {{"findings": [{{"id": "...", "quote": "..."}}]}}"""


GAP_MARK = "=== GAP MAP ==="
LIMITS_MARK = "=== LIMITATIONS ==="
SURFACE_MARK = "=== CLAIM SURFACE ==="


def marker_re(mark: str) -> "re.Pattern":
    """A marker matcher tolerant of the whitespace between its tokens.

    WHY NOT AN EXACT MATCH. 2026-08-25, on a real target: an arm emitted a
    complete and correct limitations statement — all three required lines —
    under `===\nLIMITATIONS ===`. A line wrap in the middle of the literal.
    Exact matching reported the requirement as absent, which is the opposite
    of what the record showed.

    The repository already had this lesson. claims.py normalises whitespace
    before its quote-verbatim check, with the comment: "so an honest copy
    isn't invalidated by line wrapping in the observation." Same failure,
    same fix, one subsystem later.

    A pattern rather than normalising the whole text, because claim_surface
    splits on the marker's POSITION and normalising would move every offset
    after it.
    """
    return re.compile(r"\s+".join(re.escape(t) for t in mark.split()))


_GAP_RE = marker_re(GAP_MARK)
_LIMITS_RE = marker_re(LIMITS_MARK)
_SURFACE_RE = marker_re(SURFACE_MARK)


def agent_emissions(working_log: str) -> str:
    """The agent's own lines, with tool observations removed.

    A working log interleaves what the agent emitted (`ACTION: ...`) with what
    its tools returned (`$stepN: ...`). A claim surface is CLOSED BY THE
    AUDITOR. A marker inside a tool's output is the tool handing the
    instruction back, not a closure, and reading the two as equivalent
    recorded a denominator the agent had itself rejected: on 2026-08-25 an arm
    got `=== CLAIM SURFACE === / 14 claims` from a process_text call, said in
    its next thought that the output was unreliable because "it says 14 claims
    but lists 18", re-enumerated, and closed at 32. The scorer recorded 14 and
    reported 28 findings against it.

    An observation runs from its `$stepN:` line until the next `ACTION:`,
    `$stepN:` or `--- iter` line, because tool output may contain newlines.
    """
    keep, in_observation = [], False
    for line in working_log.splitlines():
        if line.startswith("$step"):
            in_observation = True
            continue
        if line.startswith("ACTION:") or line.startswith("--- iter"):
            in_observation = False
        if not in_observation:
            keep.append(line)
    return "\n".join(keep)


def claim_surface(world: str, agent: str) -> Dict[str, Any]:
    """When enumeration closed, and what denominator it committed to (§12.2).

    WHY THE TRACE AND NOT THE REPORT. The marker's whole point is ORDERING —
    that verification followed enumeration rather than running alongside it —
    and only the trace knows which leg each thing happened in. The report is a
    single artifact with no time in it.

    Mechanical, and DELIBERATELY STRICT. §12.2 fixes the shape — the marker,
    then a count and the word `claims` on the next line — so this matches that
    and nothing else. The first version took "the first integer on or after
    the marker", which on first exposure returned n=1 for an arm that listed
    its surface ("1) Asking $480k...") and n=463 for an arm that quoted the
    instruction back without ever closing anything. A confident wrong
    denominator is worse than none: every coverage figure divides by it.

    Not matched means NOT CLOSED, which is the honest reading — the method
    asks for a specific artifact and it is absent.
    """
    def _count(body: str) -> Optional[int]:
        hit = _SURFACE_RE.search(body)
        if not hit:
            return None
        tail = body[hit.end():hit.end() + 300]
        # <N> claims — tolerating "micro-claims", "stated claims", bold marks.
        m = re.search(r"(\d[\d,]*)\s+\S*claims\b", tail, re.I)
        # Marker present but no count is not a closure, same rule as before.
        return int(m.group(1).replace(",", "")) if m else None

    turns = load_turns(world, agent)
    for i, t in enumerate(turns, 1):
        raw = t.raw.get("raw_response") or ""
        # THE AGENT'S OWN WORDS FIRST, then anything it was handed.
        #
        # An arm may close the surface THROUGH a tool — delegate enumeration to
        # process_text and adopt the result — and that is a real closure. An arm
        # may also be handed a marker it then rejects. 2026-08-25: one run got
        # `14 claims` from process_text, said in its next thought that the
        # output was unreliable because "it says 14 claims but lists 18",
        # re-enumerated, and closed at 32 in its own words. The scorer recorded
        # 14, and reported 28 findings against a 14-claim surface.
        #
        # Excluding tool observations outright was tried first and was worse:
        # it turned three legitimate tool-delegated closures into no count at
        # all, which fails the threshold. Preferring the agent's own statement
        # and falling back to what it was handed gets both cases right without
        # having to detect whether the agent adopted or rejected the output.
        n = _count(f"{agent_emissions(t.working_log)}\n{raw}")
        if n is None:
            n = _count(f"{t.working_log}\n{raw}")
        if n is None:
            continue
        return {"declared": True, "leg": i, "total_legs": len(turns),
                "count": n}
    return {"declared": False, "leg": None, "total_legs": len(turns),
            "count": None}

# §6's closed set, after `[real, with a structural note]` was retired 2026-08-24
# for zero uses across three engagements. A finding wearing a label outside this
# set is not using the method's vocabulary — the same failure §9's check catches
# at report level, one level down, where nothing was watching.
_VERDICTS = {"real", "real, minor caveat", "real, operational caveat",
             "partial", "delta", "unverifiable", "non-delta", "derived"}

# §5's heading shape: **Finding N: <title> — [verdict]**. Tolerant of the dash
# an arm chooses and of bold markers, because those vary and the verdict does
# not.
_FINDING_RE = re.compile(r"^\s*\**\s*Finding\s+\d+[^\n\[]*\[([^\]]+)\]",
                         re.M | re.I)


def verdict_conformance(report: str) -> Dict[str, Any]:
    """Which §6 verdicts the findings used, and which are not §6 verdicts.

    Mechanical, and a CONFORMANCE check on a closed vocabulary the method
    defines — the same standing as recommendation_of, not classification of
    free text. Added 2026-08-24 on noticing that removing a verdict from §6 had
    no scorer consequence whatever: nothing checked per-finding labels, so an
    arm could invent them freely and still score clean.
    """
    found = [v.strip().lower() for v in _FINDING_RE.findall(report)]
    return {"findings": len(found),
            "used": sorted(set(v for v in found if v in _VERDICTS)),
            "off_vocabulary": sorted(set(v for v in found if v not in _VERDICTS))}


# The opening of run.py's Gap Map request, verbatim. Matching THIS is safe in a
# way matching model output is not: the harness emitted it, so it is byte-exact
# every time. If run.py's GAP_MAP_REQUEST changes, change this with it.
GAP_MAP_REQUEST_MARK = "The report is received. Now the Gap Map"


def read_memo(world: str, agent: str) -> Optional[str]:
    """The report, which since §16 went to two turns is not the last reply.

    THE LAST REPLY IS THE GAP MAP. A Gap Map runs ~150 words, comfortably over
    the 400-char floor below, so taking the last substantial turn would hand
    the grader a one-page summary to score against the answer key — silently,
    and with recall that looks merely poor rather than wrong.

    The runner ASKED for the Gap Map, so the record knows which turn it is: the
    turn whose user_input carries the request. The report is the substantial
    turn before it. Runs made before the two-turn change have no such turn and
    fall through to the old rule.

    The Gap Map repeats the top findings by design, so matching over both would
    count them twice and inflate recall. It is reported separately — presence,
    length and §15 elements — rather than scored.
    """
    all_turns = load_turns(world, agent)
    gap_leg = next((i for i, t in enumerate(all_turns)
                    if GAP_MAP_REQUEST_MARK in (t.raw.get("user_input") or "")),
                   None)
    if gap_leg is not None:
        all_turns = all_turns[:gap_leg]
    turns = [t for t in all_turns if t.produced_chars > 400]
    if not turns:
        return None
    return str(turns[-1].raw.get("raw_response") or "")


def gap_map_elements(gap_map: str) -> Dict[str, bool]:
    """Which of §15's required elements a Gap Map actually contains.

    WHY THIS EXISTS. `=== GAP MAP ===` was taken as proof a Gap Map had been
    produced, and it proves only that five characters appeared. The whole
    validation was the marker plus a word count. Measured 2026-08-25 across 25
    runs: two shipped a Gap Map missing a required element — one with no
    report-link line, one with no scope disclaimer — and both passed the
    threshold.

    ITS OWN RECOMMENDATION LOCATOR, and this is the trap. A first version
    reused recommendation_of and reported nine of twenty-five Gap Maps as
    missing the recommendation. All nine were one arm, and all nine were wrong:
    recommendation_of requires the term to open the document because §16 puts
    the recommendation first in a REPORT. §15 puts target identity first, so
    that arm's recommendation sits on line 2 and a report-shaped locator cannot
    see it. Here the term may stand as a verdict anywhere in the first few
    lines.

    Not gated. Reported, because two runs is not enough to know whether a
    missing disclaimer is an arm's habit or an accident.
    """
    head = "\n".join(gap_map.strip().splitlines()[:6])
    rec = None
    for term in _REC_VOCAB:
        m = _REC_TERM_RE[term].search(head)
        if m and (rec is None or m.start() < rec[0]):
            rec = (m.start(), term)
    low = gap_map.lower()
    return {
        "recommendation": rec is not None,
        # §1a: the recommendation and the number it rests on travel together.
        "coverage": bool(re.search(r"\bcoverage\b|\bof\s+\d+\s+claims"
                                  r"|\d+\s*(?:of|/)\s*\d+", low)),
        "report link": "full report" in low,
        "disclaimer": "pen-test" in low or "pen test" in low,
    }


def read_gap_map(world: str, agent: str) -> Optional[str]:
    """The Gap Map turn's reply, for runs delivered in two turns (§16).

    Returns None for a one-turn run, where the Gap Map is inside the report's
    reply behind the marker and split_deliverables recovers it.

    Same principle as read_memo: the runner asked for this turn, so the record
    identifies it without anything being parsed out of the model's prose.
    """
    for t in load_turns(world, agent):
        if GAP_MAP_REQUEST_MARK in (t.raw.get("user_input") or ""):
            return str(t.raw.get("raw_response") or "").strip() or None
    return None


def split_deliverables(reply: str) -> tuple:
    """(report, gap_map_or_None). Split on the marker the brief specifies."""
    hit = _GAP_RE.search(reply)
    if hit:
        return reply[:hit.start()].rstrip(), reply[hit.end():].strip()
    return reply, None


_TRACE_HDR = re.compile(r'^\[(?P<label>[\w-]+)\][^\n]*?exit=(?P<exit>\w+)\s+'
                        r'iters=(?P<iters>\d+)', re.M)


def subagent_compliance(world: str, agent: str) -> Dict[str, Any]:
    """How often a subagent call came back with no answer at all.

    WHY THIS IS COUNTED RATHER THAN REPAIRED. The subagent contract is one
    line: end by emitting `respond` with your answer in `text`. A call that
    returns nothing has failed that contract, and the parent sees only
    `EMPTY:` — so it retries, narrower, and the cost lands in the iteration
    count where nothing distinguishes it from thorough work. Measured
    2026-08-24: one arm returned no answer on 12 of 33 `inspect_external`
    calls while two others managed 0 of 9 and 0 of 7. That difference took
    three passes of hand-reading traces to find, because no number reported
    it.

    NOT SALVAGED, DELIBERATELY. `subagent._salvage` reconstructs an answer
    from the working log for the paths where the loop never GOT to answer —
    `max_iters`, `format_failed`. These are different: the subagent chose to
    respond and responded with nothing. Substituting salvaged observations
    there would put words in its mouth, which in an audit is manufacturing
    evidence to cover a contract violation. Count it; do not paper over it.

    Mechanical: reads the trace files the subagents already write. No model,
    no network, and no change to anything the agent sees.
    """
    d = (SCEN / world / agent / "inspect_traces")
    calls, empty, by_exit = 0, 0, {}
    if not d.is_dir():
        return {"calls": 0, "empty": 0, "by_exit": {}}
    for f in sorted(d.glob("*.txt")):
        try:
            body = f.read_text(errors="replace")
        except OSError as e:
            logger.warning("subagent trace unreadable: %s (%s)", f, e)
            continue
        m = _TRACE_HDR.search(body)
        if not m:
            continue
        calls += 1
        by_exit[m.group("exit")] = by_exit.get(m.group("exit"), 0) + 1
        _, _, answer = body.partition("FINAL ANSWER:")
        if not answer.strip():
            empty += 1
    return {"calls": calls, "empty": empty, "by_exit": by_exit}


def trace_facts(world: str, agent: str) -> Dict[str, Any]:
    """Everything computable without a model."""
    turns = load_turns(world, agent)
    log = "\n".join(t.working_log for t in turns)
    docs = set(re.findall(r'doc(\d)_[a-z0-9_]+\.md', log))
    return {
        "turns": len(turns),
        "iterations": sum(t.iterations for t in turns),
        "docs_opened": sorted(int(d) for d in docs),
        "exit_reasons": {},
        "subagent": subagent_compliance(world, agent),
        "claim_surface": claim_surface(world, agent),
        # The run is void if the answers were in reach and got read.
        "read_answer_key": "answer_key" in log,
    }


# The five values §9 permits, longest first so "Clear with caveats" is not
# matched as "Clear". This is a CONFORMANCE check on a closed vocabulary the
# method defines — not classification of free text, which is what the
# no-keyword-matching rule exists to prevent.
_REC_VOCAB = ["Clear with caveats", "Conditional", "Material", "Walk", "Clear"]


# A term counts only where it stands as a verdict — the clause ENDS on it.
# "Recommendation: **Material**", "Verdict: Material." and "Material — the
# backups have failed" all match. "Recommendation: Clear the backup failures"
# and "we should walk away from this deal" do not, and that is the whole
# point of the trailing guard: `Clear` and `Walk` are ordinary English verbs,
# and a substring test reads both as verdicts. Both false positives were
# caught by testing the fix rather than reading it.
#
# A CONSEQUENCE WORTH NAMING: "Recommendation: Walk away from this deal"
# returns None. That is not a miss. §2 forbids recommending in the buyer's
# action language, and "walk away from this deal" is exactly that; the
# conforming spelling is the bare term. A report that means the taxonomy
# writes the taxonomy.
_REC_TERM_RE = {
    term: re.compile(
        r"(?<![A-Za-z])\*{0,2}" + re.escape(term) + r"\*{0,2}(?![A-Za-z])"
        # HORIZONTAL space only, then a terminator or END OF LINE. `\s*` here
        # instead of `[^\S\n]*` swallows the newline so `$` can never reach
        # it, and re.M is what makes `$` mean end-of-line rather than end of
        # the whole report. Getting either wrong turned 13 correct rows into
        # false negatives — caught by re-scoring all 19, not by reading it.
        #
        # `(` IS A TERMINATOR, and its absence was the THIRD bug in this
        # function — 2026-08-25, same discovery method as the first two. An
        # arm wrote "**Recommendation: Material** (coverage: 28 of 62 claims
        # individually verified)." and scored FAIL on §9. The closing paren
        # was already here; the opening one was not, so a verdict followed by
        # a parenthetical qualifier failed the guard. Semantically an opening
        # paren is the same event as `—` or `.`: the clause ended on the term
        # and a qualifier follows. The guard's real target — `Clear` and
        # `Walk` used as ordinary verbs — is untouched, because "Clear the
        # backup failures" puts a LETTER after the term, not a bracket.
        r"(?=[^\S\n]*(?:[.,;:()\]|—–-]|$))", re.I | re.M)
    for term in _REC_VOCAB
}


def recommendation_of(report: str) -> Optional[str]:
    """Return the §9 recommendation the report states, or None if it used
    some other vocabulary. A report that recommends in the buyer's action
    language ("pause", "do not proceed") violates §2 and fails here.

    WHAT WAS WRONG, AND WHAT WAS NOT. This matched four hardcoded spellings
    of the surrounding label — "Recommendation: X", "Recommendation:** X",
    "Recommendation**: X", "recommendation is **X**" — and returned None for
    anything else. Measured across the 19-row campaign 2026-08-24: FOUR false
    negatives, every `—` in the §9 column. wm4 wrote "**Section 1:
    RECOMMENDATION**" then "**Verdict: Material.**"; hd1 put the term on the
    next line; hd3 wrote "Recommendation: **Material**", where the emphasis
    markers fall between the colon and the term. All four state Material. The
    column had never once discriminated between arms — it only ever reported
    label formatting.

    The CLOSED VOCABULARY is not the bug and is kept: §9 defines five literal
    values and the method requires one of them verbatim, so testing for them
    is conformance, not classification of free text. That distinction is why
    this stays mechanical rather than becoming an LLM call. The stronger
    reason is noise: this gates a PASS/FAIL, and the campaign has already
    watched grader noise flip `unsupported` 0/1 over identical text. A
    threshold criterion that is itself sampled would put that noise on a
    second gate.

    What changed is only the LOCATOR: find each mention of "recommendation",
    then look for a vocabulary term standing as a verdict in the 120
    characters that follow. Earliest match wins, longest term breaking a tie
    so "Clear with caveats" is never read as "Clear".
    """
    # WHOLE REPORT, NOT THE FIRST 4,000 CHARS. The head window was the second
    # locator bug in this function, found 2026-08-24 the same way as the
    # first — by re-scoring real runs. It was safe while reports ran ~7,000
    # chars and led with the recommendation, as §16 requires. An arm that
    # wrote 2,851 words and put "The recommendation is Material" at the END
    # scored "§9 recommendation used: FAIL" — reporting that it never used the
    # taxonomy when it had, at character ~16,000.
    #
    # Note what that mislabels: putting the recommendation last IS a §16
    # violation, and worth catching. But it is an ORDERING failure, and this
    # criterion answers "was the vocabulary used". Reporting one as the other
    # hides both. Ordering wants its own check.
    def _in(window: str) -> Optional[str]:
        hits = []
        for term in _REC_VOCAB:
            tm = _REC_TERM_RE[term].search(window)
            if tm:
                hits.append((tm.start(), -len(term), term))
        return min(hits)[2] if hits else None

    for m in re.finditer(r"recommendation\b", report, re.I):
        found = _in(report[m.end():m.end() + 120])
        if found:
            return found

    # NO LABEL IS NOT NO RECOMMENDATION. §16 requires "the recommendation, in
    # §9's vocabulary and no other" and never requires the word
    # "Recommendation" in front of it. 2026-08-25, on a real target: a report
    # opened "**Conditional** (of 42 claims examined out of 118 identified)"
    # and scored as having stated no recommendation. That was the fourth miss
    # by this function and the first where the report was entirely conformant —
    # the locator was enforcing a label the method does not ask for.
    #
    # THE FIRST LINE, AND THE TERM MUST OPEN IT. Without the label as an anchor
    # the trailing guard is doing all the work, and `Clear` and `Walk` are
    # ordinary English. Searching the opening 200 characters was tried and read
    # "Findings worst first.\n\nThe team should walk." as a Walk
    # recommendation — the guard accepts `walk.` because a period follows.
    #
    # §8 and §16 both put the recommendation first, so requiring the term to
    # BEGIN the report costs nothing a conformant report needs and removes the
    # whole class: a §9 term in a sentence is prose, a §9 term opening the
    # document is the verdict.
    head = next((ln for ln in report.splitlines() if ln.strip()), "")
    head = head.lstrip("#*_> \t")
    found = _in(head)
    return found if found and head.lower().startswith(found.lower()) else None


# REPORTED, NOT GATED — removed from the threshold 2026-08-24.
#
# It was a gate, at 2,000 words, and it could not survive being asked where
# the number came from. Three values existed in three places: the fixture
# README says 900, METHOD.md §16 says "AIM FOR 2,000 or under", and this file
# enforced a hard 2,000. The scorer was stricter than the method it scores.
#
# The README's rationale is a PROBE-DESIGN argument and a sound one — "the cap
# forces a choice about what earns space, and that choice is the thing worth
# measuring." That justifies scarcity as an instrument. It does not justify
# the threshold's claim to test "what a delivered report would be judged on",
# which is what it silently became.
#
# NO ASSURANCE STANDARD CAPS REPORT LENGTH. ISAE 3000 and AT-C 205 specify
# report CONTENTS and are silent on length; SOC 2 reports run to 100+ pages.
# Length is a consequence of the subject matter's complexity, and bounding it
# would force omission of required content.
#
# And it was actively inverting a measurement: an arm failed two of three runs
# on length alone while finding all three must-find items with zero
# unsupported claims. A longer report covers more findings, so the gate
# penalised coverage — the one thing §4 says must never be silently traded.
#
# The scarcity idea belongs where METHOD.md already justifies it: §15's Gap
# Map, which has a stated reader model and a real one-page constraint.
_LEN_THIN, _LEN_BLOATED = 500, 5000


def _length_note(wc: int) -> str:
    """A flag, never a gate. The bounds are deliberately very loose: below
    _LEN_THIN a 50-claim surface cannot have been covered, above
    _LEN_BLOATED the stated audience (§8: a partner reading for two minutes)
    will not read it. Between them, length is the subject matter's business."""
    if wc < _LEN_THIN:
        return f"THIN — under {_LEN_THIN}, check coverage"
    if wc > _LEN_BLOATED:
        return f"LONG — over {_LEN_BLOATED}, check the §8 audience holds"
    return "reported, not gated"


def word_count(memo: str) -> int:
    return len(re.findall(r"\S+", memo))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--agent", default="Jill")
    ap.add_argument("--dry-run", action="store_true",
                    help="mechanical metrics only; no LLM call, no cost")
    args = ap.parse_args()

    facts = trace_facts(args.world, args.agent)
    memo = read_memo(args.world, args.agent)

    print(f"\n=== dataroom — {args.world}/{args.agent} ===")
    if facts["read_answer_key"]:
        print("  MEASUREMENT INVALID — the trace shows answer_key was opened.")
        print("  This is not a zero. The run measured nothing; discard it.")
        return 2
    if not memo:
        print("  no memo found (no reply over 400 chars)")
        return 1

    # TWO TURNS OR ONE. A two-turn run (§16) has the Gap Map in its own reply;
    # an older run has it inside the memo behind the marker. read_gap_map
    # answers the first case from the record, split_deliverables the second.
    gap_map = read_gap_map(args.world, args.agent)
    report, embedded = split_deliverables(memo)
    if gap_map is None:
        gap_map = embedded
    memo = report          # findings are matched over the report only
    wc = word_count(report)
    print(f"  turns {facts['turns']}   iterations {facts['iterations']}")
    if gap_map is None:
        print("  Gap Map            ABSENT — the brief asked for one")
    else:
        el = gap_map_elements(gap_map)
        missing = [k for k, ok in el.items() if not ok]
        print(f"  Gap Map            present, {word_count(gap_map)} words"
              + ("" if not missing
                 else f"   <-- §15 missing: {', '.join(missing)}"))
    print(f"  corpus docs opened  {len(facts['docs_opened'])}/{CORPUS_DOCS}  "
          f"{facts['docs_opened']}")
    print(f"  report length       {wc} words  "
          f"({_length_note(wc)})")

    vc = verdict_conformance(report)
    if vc["findings"]:
        print(f"  finding verdicts     {vc['findings']} findings, "
              f"{len(vc['used'])} distinct §6 verdicts"
              + (f"   OFF-VOCABULARY: {vc['off_vocabulary']}"
                 if vc["off_vocabulary"] else ""))
    else:
        print("  finding verdicts     none parsed — check the §5 heading shape")

    cs = facts["claim_surface"]
    if cs["declared"]:
        early = cs["leg"] <= max(1, cs["total_legs"] // 2)
        if not cs["count"]:
            note = ("   <-- n=0 is not a closure; every coverage figure "
                    "divides by it")
        elif early:
            note = ""
        else:
            note = ("   <-- closed late; verification ran before the "
                    "denominator was fixed")
        print(f"  claim surface        closed leg {cs['leg']} of "
              f"{cs['total_legs']}, n={cs['count']}" + note)
    else:
        print("  claim surface        NOT CLOSED — §12.2 requires "
              "=== CLAIM SURFACE ===")

    print(f"  limitations stmt     "
          + ("present" if _LIMITS_RE.search(report) else
             "ABSENT — §16 requires it after === LIMITATIONS ==="))

    sa = facts["subagent"]
    if sa["calls"]:
        pct = sa["empty"] / sa["calls"] * 100
        print(f"  subagent calls      {sa['calls']}  "
              f"returned NO answer: {sa['empty']} ({pct:.0f}%)"
              f"{'   <-- contract violations' if sa['empty'] else ''}")
        if sa["by_exit"]:
            print(f"                      exits {sa['by_exit']}")

    if args.dry_run:
        print("  dry run — no finding match attempted.")
        return 0

    from measure.regrade import make_grader
    _, llm = make_grader()
    raw = llm([
        {"role": "system",
         "content": _MATCH_SYS.format(findings=FINDINGS_BRIEF)},
        {"role": "user", "content": memo},
    ])
    # repair_json_string already returns the parsed dict, not a string —
    # json.loads on its result raises TypeError. Guard shape the way the
    # retired bench/common.py did, for the same reason.
    from utils.json_utils import repair_json_string
    parsed = repair_json_string(raw) if isinstance(raw, str) else raw
    if not isinstance(parsed, dict) or "findings" not in parsed:
        print(f"  match pass returned no usable JSON: {str(raw)[:200]}")
        return 1
    got = parsed["findings"]
    if not isinstance(got, list):
        print(f"  match pass 'findings' was {type(got).__name__}, not a list")
        return 1

    order = [f.get("id") for f in got]
    found = set(order)
    t1 = [x for x in TIER1 if x in found]
    t2 = [x for x in TIER2 if x in found]
    t3 = [x for x in TIER3 if x in found]
    unsupported = order.count("unsupported")

    # Difficulty ladder — the anti-saturation measure. Highest rung with at
    # least one finding on it, reported alongside the count on every rung so
    # a single lucky hit is visible as one.
    rungs = {r: [x for x in LADDER[r] if x in found] for r in RUNGS}
    top = next((r for r in reversed(RUNGS) if rungs[r]), None)
    print(f"\n  ladder rung reached {top or 'none'}")
    for r in RUNGS:
        print(f"      {r:15} {len(rungs[r])}/{len(LADDER[r])}  {rungs[r]}")

    print(f"\n  Tier 1 (must-find)  {len(t1)}/3  {t1}")
    print(f"  Tier 2 (derived)    {len(t2)}/2  {t2}")
    print(f"  Tier 3 (credit)     {len(t3)}    {t3}")
    print(f"  unsupported claims  {unsupported}"
          f"   (only these are penalised)")

    ranked = [x for x in order if x in SEVERITY]
    if ranked:
        top = ranked[0]
        print(f"\n  memo's top finding  {top}  "
              f"(key ranks it #{SEVERITY.index(top) + 1} of {len(SEVERITY)})")
        ideal = [x for x in SEVERITY if x in found]
        print(f"  placement           memo order {ranked[:6]}")
        print(f"                      key order  {ideal[:6]}")
    # ---- THE THRESHOLD ----------------------------------------------
    # Tier counts are a SCORE. A business case needs a FLOOR: did this run
    # clear the bar, yes or no. The distinction matters because the tail is
    # noisy and the bar is not — across nine runs on one configuration the
    # Tier 3 count spanned 2 to 6 while every run found all three must-find
    # items. Averaging the tail hides that; a pass rate does not.
    #
    # Each criterion is something a delivered report would be judged on, not
    # something the fixture happens to be able to count. `unsupported` is the
    # one with teeth: a claim in a delivered report that cannot produce its
    # source is the §5 provenance failure the whole method exists to prevent,
    # and it is a liability rather than a lost point.
    rec = recommendation_of(report)
    checks = [
        ("all three must-find items", len(t1) == 3),
        ("Gap Map produced",          gap_map is not None),
        ("§9 recommendation used",    rec is not None),
        ("leads with a top-3 finding",
         bool(ranked) and ranked[0] in SEVERITY[:3]),
        ("no unsupported claims",     unsupported == 0),
        ("§6 verdicts only",           not vc["off_vocabulary"]),
        ("limitations statement",      bool(_LIMITS_RE.search(report))),
        # A DECLARED ZERO IS NOT A CLOSURE. 2026-08-25: an arm's first
        # external listing returned no entries, so it emitted the marker with
        # `0 claims`, recovered, and re-froze at 273 — which its report then
        # used throughout. This criterion tested `declared` alone and passed
        # it, recording a denominator of 0 for a report that never used one.
        # Zero fails the thing the marker exists to fix: it is the number
        # every coverage figure divides by.
        ("claim surface closed",       facts["claim_surface"]["declared"]
                                       and bool(facts["claim_surface"]["count"])),
    ]
    passed = all(ok for _, ok in checks)
    print(f"\n  THRESHOLD           {'PASS' if passed else 'FAIL'}"
          f"{'' if rec is None else '   (recommendation: ' + rec + ')'}")
    for label, ok in checks:
        print(f"      {'ok  ' if ok else 'FAIL'}  {label}")

    print("\n  n=1 is anecdote. Run the fixture at least three times per arm.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
