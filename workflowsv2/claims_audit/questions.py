#!/usr/bin/env python3
"""Turn a question source into the draft surface of its statements. No model.

    python3 workflowsv2/claims_audit/questions.py --engagement <name>
    python3 workflowsv2/claims_audit/questions.py --engagement <name> --add-standard

A QUESTION SOURCE is a file the practice writes in the engagement directory
(not in the target, which the seller can read), named in engagement.yaml
under both `claim_sources:` and `questions:` with its kind
(engagement_state.question_kind). It holds statements of what the target is
expected to have, one per line, which the review tests like any claim:

    # Questions the buyer asked
    <!-- The buyer: "Does it store passwords safely?" -->
    Passwords are stored as salted hashes made with a password-hashing function.

A line starting with `#` is a heading and an HTML comment is a note; neither
is a statement. Every other non-empty line is one, and becomes one claim
whose quote and statement are the line itself, `about: target`, `asked_by`
the source's kind, and tier 1: every statement is tested, so the tier stage
skips question sources.

THE BUYER'S RATING. In a buyer source, a heading reading `# Rated by the
buyer: material` (or `decisive`, or `not_material`) gives each statement
under it that rating as `buyer_rating`, up to the next heading. The rating
stage takes it as the finding's rating instead of asking the model.

NOT APPLICABLE. The standard list is copied whole into each engagement, and
an item that does not apply to the target moves under a heading reading
exactly `# Not applicable`, written `statement | reason`. It becomes a claim
of tier 3 with the reason as its basis, so the report lists it, with the
reason, among the claims not tested, and the list stays whole in the record.

STANDARD. `--add-standard` copies method/STANDARD_QUESTIONS.md into the
engagement as questions/standard-v<N>.md, N its version, so the report names
the version, and prints the two lines engagement.yaml needs; the practice
adds them and moves what does not apply under `# Not applicable`.

The surface is written as the draft, which the practice reads and freezes on
the surface page like any other. A draft or frozen surface already there is
left alone; delete the draft to rebuild it from the file.
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List

REPO = Path(__file__).resolve().parents[2]
for p in (REPO, REPO / "src"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from workflowsv2 import engagement_state as state              # noqa: E402

logger = logging.getLogger("claims_audit.questions")

#: Written on each claim as `tier_by`, so the tier stage's `mark` leaves the
#: tier alone, and so a reader of the surface sees where the tier came from.
TIER_BY = "question source"
#: The practice's standard list; its first line names its version.
STANDARD = Path(__file__).resolve().parent / "method" / "STANDARD_QUESTIONS.md"


#: The heading above the items that do not apply to this engagement.
NOT_APPLICABLE = "not applicable"
#: The heading that carries the buyer's rating of the statements below it.
RATED_BY_BUYER = "rated by the buyer:"
BUYER_RATINGS = ("material", "decisive", "not_material")


def statements(text: str) -> List[Dict[str, Any]]:
    """The statement lines of a question source, numbered from 1, each with
    the reason it does not apply where it sits under `# Not applicable`."""
    out: List[Dict[str, Any]] = []
    in_comment = False
    not_applicable = False
    rating = None
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if in_comment or line.startswith("<!--"):
            in_comment = "-->" not in line
            continue
        if line.startswith("#"):
            heading = line.lstrip("#").strip().lower()
            not_applicable = heading == NOT_APPLICABLE
            rating = None
            if heading.startswith(RATED_BY_BUYER):
                rating = heading[len(RATED_BY_BUYER):].strip().replace(" ", "_")
                if rating not in BUYER_RATINGS:
                    raise SystemExit(f"line {n}: the buyer's rating is one of "
                                     f"{', '.join(BUYER_RATINGS)}, not {rating!r}")
            continue
        if not line:
            continue
        if not_applicable:
            statement, sep, reason = line.partition("|")
            if not sep or not reason.strip():
                raise SystemExit(f"line {n}: an item that does not apply is written "
                                 f"'statement | reason'")
            out.append({"line": n, "text": line, "statement": statement.strip(),
                        "not_applicable": reason.strip()})
        else:
            out.append({"line": n, "text": line, "statement": line,
                        "buyer_rating": rating})
    return out


def claims(text: str, kind: str) -> List[Dict[str, Any]]:
    out = []
    for i, s in enumerate(statements(text), 1):
        na = s.get("not_applicable")
        out.append({"id": i, "quote": s["text"], "lines": [s["line"], s["line"]],
                    "statement": s["statement"], "about": "target", "asked_by": kind,
                    "tier": 3 if na else 1,
                    "tier_basis": (f"Not applicable to this target: {na}" if na else
                                   "Every statement of a question source is tested."),
                    "tier_by": TIER_BY,
                    **({"buyer_rating": s["buyer_rating"]} if s.get("buyer_rating") else {})})
    return out


def build(eng_dir: Path) -> Dict[str, int]:
    """Write the draft surface of every question source that has neither a
    draft nor a frozen surface. Returns {source: claims written}."""
    from client_ui import jobs
    done: Dict[str, int] = {}
    for src in state.claim_sources(eng_dir):
        kind = state.question_kind(eng_dir, src)
        if not kind:
            continue
        frozen = jobs.surface_file(eng_dir, src)
        draft = frozen.with_name(frozen.name.replace(".surface.json", ".draft.json"))
        if frozen.is_file() or draft.is_file():
            logger.info("%s already has a surface; left as it is", src)
            continue
        f = state.claim_source_file(eng_dir, src)
        if not f.is_file():
            raise SystemExit(f"question source {src} is not at {f}")
        rows = claims(f.read_text(encoding="utf-8"), kind)
        if not rows:
            raise SystemExit(f"question source {src} has no statement")
        draft.parent.mkdir(exist_ok=True)
        draft.write_text(json.dumps({"claim_source": src, "claims": rows},
                                    indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
        logger.info("%s: %d statements -> %s", src, len(rows), draft.name)
        done[src] = len(rows)
    return done


def add_standard(eng_dir: Path) -> str:
    """Copy the standard list into the engagement; returns the claim source.
    An existing copy is left alone: the practice may have edited it."""
    import re
    first = STANDARD.read_text(encoding="utf-8").splitlines()[0]
    m = re.search(r"version (\d+)", first)
    if not m:
        raise SystemExit(f"{STANDARD.name} does not name its version on its first line")
    src = f"questions/standard-v{m.group(1)}.md"
    dst = eng_dir / src
    if dst.is_file():
        logger.info("%s is already in the engagement; left as it is", src)
    else:
        dst.parent.mkdir(exist_ok=True)
        dst.write_text(STANDARD.read_text(encoding="utf-8"), encoding="utf-8")
    return src


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--add-standard", action="store_true",
                    help="copy the standard list into the engagement instead of building surfaces")
    args = ap.parse_args()
    eng = state.ENGAGEMENTS / args.engagement
    if not eng.is_dir():
        raise SystemExit(f"no engagement {args.engagement}")
    if args.add_standard:
        src = add_standard(eng)
        print(f"Copied to {eng / src}. Add to engagement.yaml:\n"
              f"  claim_sources:  - {src}\n  questions:      {src}: standard")
        return 0
    build(eng)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
