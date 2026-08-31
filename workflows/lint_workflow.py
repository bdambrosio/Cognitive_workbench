#!/usr/bin/env python3
"""Mechanical consistency checks on a workflow method document.

    python3 workflows/lint_workflow.py                 # every known document
    python3 workflows/lint_workflow.py path/to/DOC.md

WHY THIS EXISTS. On 2026-08-27 METHOD.md and REVIEW.md were audited by hand
five times, by two readers. Each round found six to nineteen defects, and
roughly half of each round's findings had been *introduced by the previous
round's fixes*. The defect classes repeated: a token retired in one section and
still named in another, a counted list whose count no longer matched its items,
a cross-reference to a section the reader never receives, and a changelog
sitting in the prompt naming vocabulary the document forbids.

Every one of those is a token-level invariant, and a token-level invariant
maintained by careful reading is maintained badly at the end of a long session.

WHAT IT DOES NOT DO, and this matters more than what it does. It checks form,
never meaning. It cannot see that §2's heading used a term its own body had just
reserved, that "your report" meant the review while "the report" meant the
audit's, or that `[supported]` means one thing here and another in METHOD. Those
took a reader. **A clean run means these checks passed — it does not mean the
document is consistent**, and anyone who quotes it as the latter has
misunderstood the instrument.

HOW IT IS TRUSTED. Not by trusting the author. Every check below was written
against a defect that actually occurred, and the commit that fixed each one is
named in its docstring, so the check can be validated by running it on that
commit's parent and confirming it fires.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from chat.workflow import load_workflow, PRACTICE_MARKER      # noqa: E402
from workflows import blocks                                  # noqa: E402

# NAMED, NOT INDEXED. These were METHOD_DOC and REVIEW_DOC until 2026-08-30, when a
# third document was added at the front and every cross-check below silently
# repointed — the runner-reference check started resolving §12 against DELIVERY
# and failed eight references that were correct. The tests caught it. A tuple
# position is not a name.
METHOD_DOC = "workflows/claims_audit/method/METHOD.md"
REVIEW_DOC = "workflows/audit_review/method/REVIEW.md"
DELIVERY_DOC = "workflows/audit_postprocess/method/DELIVERY.md"

DOCS = (METHOD_DOC, REVIEW_DOC, DELIVERY_DOC)

# The runner that drives each document, and the document its bare §N means.
# A runner emits text the agent reads — "See REVIEW.md §4.0." is a sentence in
# the review's output, not a comment — so a §N there is as load-bearing as one
# in the method, and nothing was checking it.
RUNNERS = {"workflows/claims_audit/runner.py": METHOD_DOC,
           "workflows/audit_review/runner.py": REVIEW_DOC}

# Tokens a document retired. Naming one in the text the agent reads puts the
# forbidden vocabulary back in the prompt, three lines from the table it was
# removed from — which is what §9 did with `Walk` (fixed 032f056b).
RETIRED = {
    "METHOD.md": ("non-delta", "Walk", "AI-Readiness"),
    "REVIEW.md": (),
}

# Number words that introduce a counted list. "Three things follow" heading five
# items (fixed 0c2a525c); "the other eight" over a seven-row table (be911d20);
# "two questions" over four (306a5686); "Only the other five" over six.
_COUNTS = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
           "seven": 7, "eight": 8, "nine": 9, "ten": 10}
_COUNT_LEAD = re.compile(
    r"\b(one|two|three|four|five|six|seven|eight|nine|ten)\s+"
    r"(things? follow|rules? follow|questions?|shapes?|blocks?|reasons?|"
    r"legitimate cases?|quantities|vocabularies|checks?|levels?|verdicts?)\b",
    re.I)


def sections(raw: str) -> List[Tuple[str, str]]:
    """(heading, body) per `## ` section, preamble first."""
    parts = re.split(r"(?m)^(?=## )", raw)
    out = []
    for p in parts:
        head = p.splitlines()[0] if p.strip() else ""
        out.append((head, p))
    return out


def check_retired(name: str, agent: str) -> List[str]:
    """A retired token must not appear in what the agent reads."""
    bad = []
    for tok in RETIRED.get(name, ()):
        for m in re.finditer(rf"\b{re.escape(tok)}\b", agent):
            ln = agent[:m.start()].count("\n") + 1
            bad.append(f"retired token {tok!r} at agent-line {ln}")
    return bad


def check_dates(name: str, agent: str, allow: Tuple[str, ...]) -> List[str]:
    """A date in the agent's text is almost always changelog that escaped.

    METHOD keeps three in a worked arithmetic example; those are declared.
    Everything else is history, and history belongs behind the practice marker
    (bb4613b9, 032f056b).
    """
    bad = []
    for m in re.finditer(r"20\d\d-\d\d-\d\d", agent):
        if m.group(0) in allow:
            continue
        ln = agent[:m.start()].count("\n") + 1
        bad.append(f"date {m.group(0)} in agent text at line {ln} "
                   f"(changelog belongs behind {PRACTICE_MARKER})")
    return bad


def check_refs(name: str, raw: str, agent: str) -> List[str]:
    """Every §N referenced must exist, and agent text must not cite a section
    the agent never receives — "a pointer into a section the auditor never
    receives is no rule at all" (METHOD §14)."""
    have = set()
    practice = set()
    for head, body in sections(raw):
        m = re.match(r"## (\d+[a-z]?)\.", head)
        if not m:
            continue
        have.add(m.group(1))
        if PRACTICE_MARKER in body:
            practice.add(m.group(1))
    bad = []
    # A cross-document reference — "METHOD §16" — is not a reference to a
    # section of THIS document, and flagging it was this check's own first
    # false positive. Only bare §N is intra-document.
    for m in re.finditer(r"(?<!METHOD )(?<!REVIEW )§(\d+[a-z]?)", agent):
        ref = m.group(1)
        ln = agent[:m.start()].count("\n") + 1
        if ref not in have:
            bad.append(f"§{ref} referenced at agent-line {ln} but no such section")
        elif ref in practice:
            bad.append(f"§{ref} referenced at agent-line {ln} but that section "
                       f"is practice-only — the agent never receives it")
    return bad


def check_counts(name: str, agent: str) -> List[str]:
    """A counted lead-in must match what follows it.

    Heuristic and deliberately advisory: it counts numbered items, list bullets
    and bold lead-ins in the 40 lines after the phrase, and reports only when
    none of those tallies matches. Every real instance tonight was off by one
    or two, so a near-miss is worth a human look even when the count is
    ambiguous.
    """
    out = []
    lines = agent.splitlines()
    for m in _COUNT_LEAD.finditer(agent):
        said = _COUNTS[m.group(1).lower()]
        start = agent[:m.start()].count("\n")
        window = lines[start:start + 40]
        text = "\n".join(window)
        numbered = len(re.findall(r"(?m)^\s*\d+\.\s", text))
        bullets = len(re.findall(r"(?m)^\s*[-*]\s", text))
        # Bold SPANS anywhere, not line-initial bold: §4's five items are
        # run-in, so a line-initial counter found three and would have missed
        # the "Three things follow" defect entirely.
        bolds = len(re.findall(r"\*\*[^*]+\*\*", text))
        rows = max(len(re.findall(r"(?m)^\|", text)) - 2, 0)
        tallies = {numbered, bullets, bolds, rows}
        if said not in tallies:
            out.append(f"line {start + 1}: {m.group(0)!r} — numbered={numbered} "
                       f"bullets={bullets} bold-spans={bolds} rows={rows}")
    return out


def check_block_vocab(name: str, raw: str) -> List[str]:
    """The block markers a document specifies must be the set the code drives.

    `=== GAP MAP ===` was declared in two runners and specified to the agent in
    neither, until 038a9dab. This is the check that would have caught it.
    """
    declared = set(re.findall(r"===\s*(?!END\b)([A-Z][A-Z ]*?)\s*===", raw))
    declared = {d.strip() for d in declared if d.strip()}
    expected = (set(blocks.BLOCKS) if "claims_audit" in name
                else set(blocks.DELIVERY_BLOCKS) if "audit_postprocess" in name
                else set(blocks.REVIEW_BLOCKS))
    bad = []
    for missing in sorted(expected - declared):
        bad.append(f"block {missing!r} is in blocks.py but never specified")
    for extra in sorted(declared - expected):
        bad.append(f"marker {extra!r} specified but not in blocks.py's set")
    for b in sorted(expected & declared):
        if not re.search(rf"===\s*END\s+{re.escape(b)}\s*===", raw):
            bad.append(f"block {b!r} has an opener specified and no closer")
    return bad


def check_delivery_gloss() -> List[str]:
    """The client-facing glossary must name exactly METHOD's verdict set.

    The delivery script explains `[delta]` and its siblings to a reader who has
    never seen METHOD, because the delivery agent is not given METHOD and
    cannot. That makes the glossary a second place the closed vocabulary lives,
    and a second place it can drift — the failure this file already guards for
    score.py. Same check, same reason: a verdict added or retired in §6 must
    fail here rather than leave a buyer reading an undefined bracket.

    The WORDING is the delivery script's own, pitched at a buyer. Only the set
    of keys is checked.
    """
    sys.path.insert(0, str(REPO / "workflows" / "audit_postprocess"))
    import deliver                                             # noqa: E402
    raw = (REPO / METHOD_DOC).read_text(encoding="utf-8")
    in_method = set(re.findall(r"\|\s*`(\[[^`\]]+\])`\s*\|", raw))
    # §6 also tables `[unclaimed]` and `[derived]`, which are not verdicts on a
    # seller claim and never appear in a finding title's bracket.
    in_method -= {"[unclaimed]", "[derived]"}
    glossed = {k for k, _ in deliver.VERDICT_GLOSS}
    return ([f"verdict {v!r} is in METHOD but not glossed for the client"
             for v in sorted(in_method - glossed)]
            + [f"glossary explains {v!r}, which METHOD does not define"
               for v in sorted(glossed - in_method)])


def check_code_vocab() -> List[str]:
    """Closed vocabularies in METHOD must equal the sets score.py enforces.

    §9's five conclusions and §6's verdicts are gates. When `Walk` became
    `Systemically inconsistent` (be911d20), a stale `_REC_VOCAB` would have
    scored every conformant report as stating no conclusion.
    """
    sys.path.insert(0, str(REPO / "measure" / "fixtures" / "dataroom"))
    import score                                              # noqa: E402
    raw = (REPO / METHOD_DOC).read_text(encoding="utf-8")
    bad = []
    for term in score._REC_VOCAB:
        if f"**{term}**" not in raw:
            bad.append(f"score.py conclusion {term!r} is not a §9 table row")
    for verdict in score._VERDICTS:
        if f"`[{verdict}]`" not in raw:
            bad.append(f"score.py verdict '[{verdict}]' appears nowhere in METHOD")
    # AND THE OTHER DIRECTION. This walked score.py -> METHOD only until
    # 2026-08-31, when a status added to §6 that day scored as
    # off-vocabulary in a checker that had never heard of it — silently,
    # because nothing looked this way. The status was withdrawn the same
    # day; the half-check it exposed was not. A one-way equality check is
    # half a check.
    for term in re.findall(r"^\| `\[([^\]]+)\]`", raw, re.M):
        if term not in score._VERDICTS:
            bad.append(f"METHOD \u00a76 defines '[{term}]' and score.py "
                       "does not recognise it")
    return bad


def _doc_sections(path: str) -> set:
    have = set()
    for head, _ in sections((REPO / path).read_text(encoding="utf-8")):
        m = re.match(r"## (\d+[a-z]?(?:\.\d+)?)\.?\s", head)
        if m:
            have.add(m.group(1))
    return have


def check_code_refs(runner: str, doc: str) -> List[str]:
    """A §N inside a runner's STRING must resolve in the document it names.

    Renaming a section is a substitution across the document and a silent trap
    in the code: `runner.py` line 884 emits "See REVIEW.md §4.0." to the
    reviewer at run time, so renaming §4.0 without touching that string points
    the reviewer at a section that no longer exists — the dangling-pointer
    failure, produced by the fix for a numbering problem.

    Strings are the failure. Comments are reported separately: they mislead the
    next editor rather than the agent, which is a smaller harm and a different
    fix.
    """
    import ast
    src = (REPO / runner).read_text(encoding="utf-8")
    known = {"METHOD": _doc_sections(METHOD_DOC), "REVIEW": _doc_sections(REVIEW_DOC)}
    own = "METHOD" if "claims_audit" in runner else "REVIEW"
    bad = []
    for node in ast.walk(ast.parse(src)):
        if not (isinstance(node, ast.Constant) and isinstance(node.value, str)):
            continue
        for m in re.finditer(r"(METHOD|REVIEW)?[\w.]*\s*§(\d+[a-z]?(?:\.\d+)?)",
                             node.value):
            doc_key = m.group(1) or own
            ref = m.group(2)
            if ref not in known.get(doc_key, set()):
                bad.append(f"{runner}:{node.lineno} emits §{ref} "
                           f"({doc_key}) — no such section")
    return bad


def check_code_comment_refs(runner: str) -> List[str]:
    """Same, for comments. Advisory: rot for the next reader, not the agent."""
    known = {"METHOD": _doc_sections(METHOD_DOC), "REVIEW": _doc_sections(REVIEW_DOC)}
    own = "METHOD" if "claims_audit" in runner else "REVIEW"
    out = []
    for i, line in enumerate((REPO / runner).read_text().splitlines(), 1):
        code, _, comment = line.partition("#")
        if not comment or code.count('"') % 2 or code.count("'") % 2:
            continue
        for m in re.finditer(r"(METHOD|REVIEW)?[\w.]*\s*§(\d+[a-z]?(?:\.\d+)?)",
                             comment):
            if m.group(2) not in known.get(m.group(1) or own, set()):
                out.append(f"{runner}:{i} comment cites §{m.group(2)} "
                           f"({m.group(1) or own}) — no such section")
    return out


def lint(path: str) -> Dict[str, List[str]]:
    name = Path(path).name
    raw = (REPO / path).read_text(encoding="utf-8")
    agent = load_workflow(REPO / path)
    allow = ("2026-07-30", "2026-08-29") if name == "METHOD.md" else ()
    return {
        "retired tokens in the prompt": check_retired(name, agent),
        "dates in the prompt": check_dates(name, agent, allow),
        "section references": check_refs(name, raw, agent),
        "block vocabulary": check_block_vocab(path, raw),
    }


def main() -> int:
    targets = sys.argv[1:] or list(DOCS)
    failed = 0
    for path in targets:
        agent = load_workflow(REPO / path)
        raw = (REPO / path).read_text(encoding="utf-8")
        print(f"\n=== {path}")
        print(f"    {len(raw):,} raw -> {len(agent):,} the agent reads "
              f"({100 * len(agent) // len(raw)}%)")
        # ADVISORY, and never a failure. Counting the items under a counted
        # lead-in needs judgement about what an item is — run-in bold, bullet,
        # numbered step, table row — and every tally it prints is a candidate
        # for a human, not a verdict. A check that cannot be trusted to fail
        # correctly must not be trusted to pass either, so it does neither.
        advisory = check_counts(Path(path).name, agent)
        for check, problems in lint(path).items():
            if problems:
                failed += len(problems)
                print(f"  FAIL  {check}")
                for p in problems:
                    print(f"          {p}")
            else:
                print(f"  ok    {check}")
        if advisory:
            print("  ----  counted lists (ADVISORY, not a failure)")
            for a in advisory:
                print(f"          {a}")
    print("\n=== §N inside runner code")
    for runner, doc in RUNNERS.items():
        strings = check_code_refs(runner, doc)
        comments = check_code_comment_refs(runner)
        if strings:
            failed += len(strings)
            for x in strings:
                print(f"  FAIL  {x}")
        else:
            print(f"  ok    {runner}: every §N in a string resolves")
        if comments:
            print(f"  ----  {runner}: comments (ADVISORY)")
            for c in comments:
                print(f"          {c}")

    extra = check_code_vocab()
    print("\n=== the client-facing verdict glossary against METHOD §6")
    for p_ in check_delivery_gloss():
        failed += 1
        print(f"  FAIL  {p_}")
    else:
        if not check_delivery_gloss():
            print("  ok    every verdict a finding can carry is explained")

    print("\n=== METHOD vocabularies against score.py")
    if extra:
        failed += len(extra)
        for e in extra:
            print(f"  FAIL  {e}")
    else:
        print("  ok    conclusions and verdicts match")
    print(f"\n{failed} problem(s). Form only — a clean run is not a "
          f"consistency guarantee.")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
