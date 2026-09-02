#!/usr/bin/env python3
"""Mechanical consistency checks on a workflow method document.

    python3 workflowsv2/lint_workflow.py                 # every known document
    python3 workflowsv2/lint_workflow.py path/to/DOC.md

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
from workflowsv2 import blocks                                  # noqa: E402

# NAMED, NOT INDEXED. These were METHOD_DOC and REVIEW_DOC until 2026-08-30, when a
# third document was added at the front and every cross-check below silently
# repointed — the runner-reference check started resolving §12 against DELIVERY
# and failed eight references that were correct. The tests caught it. A tuple
# position is not a name.
METHOD_DOC = "workflowsv2/claims_audit/method/METHOD.md"
REVIEW_DOC = "workflowsv2/audit_review/method/REVIEW.md"
MATERIALITY_DOC = "workflowsv2/audit_materiality/method/MATERIALITY.md"
REPORT_DOC = "workflowsv2/audit_report/method/REPORT.md"
INTAKE_DOC = "workflowsv2/intake/method/INTAKE.md"

DOCS = (METHOD_DOC, REVIEW_DOC, MATERIALITY_DOC, REPORT_DOC, INTAKE_DOC)

# The runner that drives each document, and the document its bare §N means.
# A runner emits text the agent reads — "See REVIEW.md §4.0." is a sentence in
# the review's output, not a comment — so a §N there is as load-bearing as one
# in the method, and nothing was checking it.
RUNNERS = {"workflowsv2/claims_audit/runner.py": METHOD_DOC,
           "workflowsv2/audit_review/runner.py": REVIEW_DOC,
           "workflowsv2/audit_materiality/runner.py": MATERIALITY_DOC,
           "workflowsv2/audit_report/runner.py": REPORT_DOC,
           "workflowsv2/intake/runner.py": INTAKE_DOC}

#: The name a runner's string uses for each document — "MATERIALITY §2" —
#: and the document a bare §N in that runner means. Until 2026-09-02 only
#: METHOD and REVIEW were known, so a materiality runner's "§3" resolved
#: against REVIEW and passed by coincidence.
def _doc_names() -> Dict[str, str]:
    """Read at call time, so a test that repoints one document sees it."""
    return {"METHOD": METHOD_DOC, "REVIEW": REVIEW_DOC,
            "MATERIALITY": MATERIALITY_DOC, "REPORT": REPORT_DOC,
            "INTAKE": INTAKE_DOC}


_REF = r"(METHOD|REVIEW|MATERIALITY|REPORT|INTAKE)?[\w.]*\s*§(\d+[a-z]?(?:\.\d+)?)"

# Tokens a document retired. Naming one in the text the agent reads puts the
# forbidden vocabulary back in the prompt, three lines from the table it was
# removed from — which is what §9 did with `Walk` (fixed 032f056b).
RETIRED = {
    # `delta` and the two caveat verdicts were retired together: `delta` for
    # naming a difference without its direction, and the caveats because five
    # of nine known-adverse findings drained into `real_operational_caveat`
    # and were recorded as claims that hold.
    "METHOD.md": ("non-delta", "Walk", "AI-Readiness", "delta",
                  "real_minor_caveat", "real_operational_caveat"),
    "REVIEW.md": ("delta", "real_minor_caveat", "real_operational_caveat"),
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


def _doc_sections(path: str) -> set:
    """A document that is not there declares no sections, so every §N against
    it is reported — loud, not silent."""
    have = set()
    if not (REPO / path).is_file():
        return have
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
    known = {k: _doc_sections(v) for k, v in _doc_names().items()}
    own = next(k for k, v in _doc_names().items() if v == doc)
    bad = []
    for node in ast.walk(ast.parse(src)):
        if not (isinstance(node, ast.Constant) and isinstance(node.value, str)):
            continue
        for m in re.finditer(_REF, node.value):
            doc_key = m.group(1) or own
            ref = m.group(2)
            if ref not in known.get(doc_key, set()):
                bad.append(f"{runner}:{node.lineno} emits §{ref} "
                           f"({doc_key}) — no such section")
    return bad


def check_code_comment_refs(runner: str) -> List[str]:
    """Same, for comments. Advisory: rot for the next reader, not the agent."""
    known = {k: _doc_sections(v) for k, v in _doc_names().items()}
    own = next(k for k, v in _doc_names().items() if v == RUNNERS[runner])
    out = []
    for i, line in enumerate((REPO / runner).read_text().splitlines(), 1):
        code, _, comment = line.partition("#")
        if not comment or code.count('"') % 2 or code.count("'") % 2:
            continue
        for m in re.finditer(_REF, comment):
            if m.group(2) not in known.get(m.group(1) or own, set()):
                out.append(f"{runner}:{i} comment cites §{m.group(2)} "
                           f"({m.group(1) or own}) — no such section")
    return out


def check_schema_vocab(path: str, raw: str) -> List[str]:
    """METHOD's closed vocabularies against the schema the runner enforces.

    THIS REPLACES `check_block_vocab` FOR claims_audit, and it is the same
    check pointed at a different contract. The invariant has not changed: what
    the document specifies to the agent and what the code enforces must be one
    object. `=== GAP MAP ===` was declared in two runners and specified to the
    agent in neither, and that is what the block check caught.

    In v2 the contract is the JSON schema, not a marker set, so comparing
    METHOD's declared markers against `blocks.BLOCKS` would compare two empty
    sets and report ok — a green check that verifies nothing. A schema and a
    prose table that disagree fail silently, which is worse than a stalled run.

    EACH VOCABULARY IS READ FROM THE SECTION THAT DEFINES IT, never from the
    document at large. Reading every table cost this check four false
    positives on its first run: §13's field table lists `claim_source` and
    `not_completed`, which are fields and not vocabulary, and a table's own
    header row names the column.
    """
    from workflowsv2.claims_audit import schemas as sch          # noqa: E402
    bodies = {re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0]).group(1): b
              for _, b in sections(raw)
              if b.strip() and re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0])}

    def first_col(sec: str) -> set:
        """Backticked tokens in a table's first column, below the header rule.

        The header row is skipped by position, not by name: §8's header cell
        is `unresolved_because` in backticks, which a name-blind reader takes
        for a value of itself.
        """
        out, in_body = set(), False
        for line in bodies.get(sec, "").splitlines():
            if not line.lstrip().startswith("|"):
                in_body = False
                continue
            if re.match(r"^\s*\|[\s:|-]+\|\s*$", line):
                in_body = True
                continue
            if in_body:
                m = re.match(r"^\s*\|\s*`([a-z_]+)`\s*\|", line)
                if m:
                    out.add(m.group(1))
        return out

    def bold_tokens(sec: str) -> set:
        """Backticked tokens introduced as bold lead-ins — §7's evidence forms."""
        return set(re.findall(r"(?m)^\*\*`([a-z_]+)`\*\*", bodies.get(sec, "")))

    bad = []
    for sec, group, names, read in (
            ("6", "verdict", sch.VERDICTS, first_col),
            ("8", "unresolved_because", sch.UNRESOLVED_BECAUSE, first_col),
            ("5", "about", sch.ABOUT, first_col),
            ("7", "evidence form", sch.EVIDENCE_FORMS, bold_tokens)):
        declared = read(sec)
        if not declared:
            bad.append(f"§{sec} declares no {group} at all")
            continue
        for n in names:
            if n not in declared:
                bad.append(f"schemas.py {group} {n!r} is not declared in §{sec}")
        for d in sorted(declared - set(names)):
            bad.append(f"§{sec} declares {group} {d!r}, "
                       f"which schemas.py does not define")

    # Every form's required fields must be named where the form is specified.
    for form, fields in sch.FORM_FIELDS.items():
        for f in fields:
            if f"`{f}`" not in raw:
                bad.append(f"form {form!r} requires {f!r}, "
                           f"which METHOD never names")
    return bad


def check_review_vocab(path: str, raw: str) -> List[str]:
    """REVIEW's observations and their values against the schema it drives.

    The audit's equivalent is `check_schema_vocab`; this is the same invariant
    for a differently shaped table. §6 names the observations in its first
    column and their values in its second, so both are read from there.
    """
    from workflowsv2.audit_review import schemas as rs                # noqa: E402
    bodies = {re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0]).group(1): b
              for _, b in sections(raw)
              if b.strip() and re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0])}
    six = bodies.get("6", "")
    bad = []
    if not six:
        return ["REVIEW has no §6 to declare observations in"]
    declared = {}
    for line in six.splitlines():
        m = re.match(r"^\s*\|\s*`([a-z_]+)`\s*\|([^|]*)\|", line)
        if m:
            declared[m.group(1)] = set(re.findall(r"`([a-z_]+)`", m.group(2)))
    for name, values in rs.OBSERVATIONS.items():
        if name not in declared:
            bad.append(f"schemas.py observation {name!r} is not declared in §6")
            continue
        for v in values:
            if v not in declared[name]:
                bad.append(f"§6 does not give {name!r} the value {v!r}")
        for v in sorted(declared[name] - set(values)):
            bad.append(f"§6 gives {name!r} a value {v!r} schemas.py "
                       f"does not define")
    for extra in sorted(set(declared) - set(rs.OBSERVATIONS)):
        bad.append(f"§6 declares observation {extra!r}, "
                   f"which schemas.py does not define")
    for f in rs.FIDELITY:
        if f"`{f}`" not in raw:
            bad.append(f"fidelity value {f!r} is never named in REVIEW")
    return bad


def check_materiality_vocab(path: str, raw: str) -> List[str]:
    """MATERIALITY §3's scale against `schemas.MATERIALITY`. The same invariant
    as the other two vocabulary checks, for a one-column table."""
    from workflowsv2.audit_materiality import schemas as ms          # noqa: E402
    bodies = {re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0]).group(1): b
              for _, b in sections(raw)
              if b.strip() and re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0])}
    three = bodies.get("3", "")
    if not three:
        return ["MATERIALITY has no §3 to declare the scale in"]
    declared, in_body = [], False
    for line in three.splitlines():
        if not line.lstrip().startswith("|"):
            in_body = False
            continue
        if re.match(r"^\s*\|[\s:|-]+\|\s*$", line):
            in_body = True
            continue
        if in_body:
            m = re.match(r"^\s*\|\s*`([a-z_]+)`\s*\|", line)
            if m:
                declared.append(m.group(1))
    bad = []
    for v in ms.MATERIALITY:
        if v not in declared:
            bad.append(f"schemas.py value {v!r} is not in §3's table")
    for v in declared:
        if v not in ms.MATERIALITY:
            bad.append(f"§3 declares {v!r}, which schemas.py does not define")
    if tuple(declared) != tuple(ms.MATERIALITY):
        bad.append("§3 lists the scale in a different order from schemas.py")
    return bad


def check_report_fields(path: str, raw: str) -> List[str]:
    """REPORT §7's field table against `schemas.FIELDS`. The report's output
    is prose fields with no closed vocabulary, so the invariant is the field
    set: every field the schema requires is specified, and nothing more."""
    from workflowsv2.audit_report import schemas as rs               # noqa: E402
    bodies = {re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0]).group(1): b
              for _, b in sections(raw)
              if b.strip() and re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0])}
    seven = bodies.get("7", "")
    if not seven:
        return ["REPORT has no §7 to declare the fields in"]
    declared = set(re.findall(r"(?m)^\s*\|\s*`([a-z_]+)`\s*\|", seven))
    bad = [f"schemas.py field {f!r} is not in §7's table"
           for f in rs.FIELDS if f not in declared]
    bad += [f"§7 declares {d!r}, which schemas.py does not define"
            for d in sorted(declared - set(rs.FIELDS))]
    return bad


def check_intake_fields(path: str, raw: str) -> List[str]:
    """INTAKE §4's field table against `schemas.SLOTS`: every slot.field the
    schema requires is specified, plus the two arrays, and nothing more."""
    from workflowsv2.intake import schemas as isch                    # noqa: E402
    bodies = {re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0]).group(1): b
              for _, b in sections(raw)
              if b.strip() and re.match(r"## (\d+[a-z]?)\.", b.splitlines()[0])}
    four = bodies.get("4", "")
    if not four:
        return ["INTAKE has no §4 to declare the fields in"]
    declared = set(re.findall(r"(?m)^\s*\|\s*`([a-z_.]+)(?:\[\])?`\s*\|", four))
    expected = {f"{slot}.{f}" for slot, fs in isch.SLOTS.items() for f in fs}
    expected |= {"open_questions", "notes"}
    bad = [f"schemas.py field {f!r} is not in §4's table"
           for f in sorted(expected - declared)]
    bad += [f"§4 declares {d!r}, which schemas.py does not define"
            for d in sorted(declared - expected)]
    return bad


def lint(path: str) -> Dict[str, List[str]]:
    name = Path(path).name
    raw = (REPO / path).read_text(encoding="utf-8")
    agent = load_workflow(REPO / path)
    allow = ("2026-07-30", "2026-08-29") if name == "METHOD.md" else ()
    return {
        "retired tokens in the prompt": check_retired(name, agent),
        "dates in the prompt": check_dates(name, agent, allow),
        "section references": check_refs(name, raw, agent),
        # Every v2 document answers under a schema; the block-vocabulary
        # check remains for a document that still delivers text blocks. One
        # key either way, so the report reads the same and a reader is told
        # which contract was checked.
        **({"schema vocabulary": check_schema_vocab(path, raw)}
           if "claims_audit" in path else
           {"schema vocabulary": check_review_vocab(path, raw)}
           if "audit_review" in path else
           {"schema vocabulary": check_materiality_vocab(path, raw)}
           if "audit_materiality" in path else
           {"schema vocabulary": check_report_fields(path, raw)}
           if "audit_report" in path else
           {"schema vocabulary": check_intake_fields(path, raw)}
           if "workflowsv2/intake" in path else
           {"block vocabulary": check_block_vocab(path, raw)}),
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

    print(f"\n{failed} problem(s). Form only — a clean run is not a "
          f"consistency guarantee.")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
