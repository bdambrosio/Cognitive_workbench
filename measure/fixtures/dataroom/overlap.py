#!/usr/bin/env python3
"""Compare claim surfaces across arms by what they cited, not how many.

    python3 measure/fixtures/dataroom/overlap.py c1_grok c1_qwen c1_luna

WHY NOT COMPARE COUNTS. Two arms that both enumerate 28 claims may have
enumerated two different sets of 28. Count agreement would read as convergence
and establish nothing. ISAE 3000 (Revised) A45(c) asks whether criteria permit
"reasonably consistent measurement ... by different practitioners" — that is a
question about which claims were identified, and only the citations answer it.

WHAT IS COMPARED. Every `docN:LINE` or `docN:LINE-LINE` citation in an arm's
enumerated surface, expanded to the set of document lines it points at. Two
arms agree about a line when both cited it. Reported as pairwise Jaccard
(shared lines over lines either cited), plus the lines all arms found and the
lines only one arm found.

WHAT THIS IS NOT. Line citations are a proxy for claim identity, not claim
identity itself. One arm may split a compound sentence into two claims citing
one line while another keeps it whole — they agree here and differ in count,
which is the grain question and shows up as a count difference on the same
line set. Conversely two claims from one line are indistinguishable. The
measure is directional evidence, not a verdict.
"""

from __future__ import annotations

import argparse
import itertools
import json
import re
import sys
from pathlib import Path
from typing import Dict, Set

HERE = Path(__file__).resolve().parent
CLAIMS = HERE / "claims"

# A citation is a document reference followed by a line number. The spellings
# arms actually use, across the fixture and two real repositories:
#
#   doc1:5      doc4:16-19      Doc 9 line 7
#   doc1_seller_listing_description.md:8
#   README.md:9     backend/app/models/agent.py:42
#
# The token must contain a letter, so a bare "12:30" is not a citation.
_CITE = re.compile(
    r"\b([A-Za-z][A-Za-z0-9_.\-/]*)\s*:\s*(?:lines?\s*)?(\d+)(?:\s*[-–]\s*(\d+))?")
# The fixture's prose form, which has no colon.
_CITE_PROSE = re.compile(r"\bdoc\s*(\d)\s+lines?\s*(\d+)", re.I)

# `doc1: 17 claims` and `README.md: 180` under a "Per-document:" heading are
# TOTALS, not citations to lines 17 and 180. Arms print both.
_NOT_A_LINE = re.compile(r"\s*(?:claims?|findings?)\b", re.I)

_DOCN = re.compile(r"^doc(\d)", re.I)

# A document reference is `docN` or something with a file extension. Without
# this test, generalising the pattern beyond `docN` picked up "count: 33" from
# this tool's own output header and "Active customers: 120" from a claim's
# text, and reported both as cited lines.
#
# The extension must contain a LETTER. Accepting a numeric one made
# "http://127.0.0.1:8000" a citation to line 8000 of a document called
# "127.0.0.1", which duly appeared in a per-document breakdown.
_LOOKS_LIKE_A_FILE = re.compile(r"\.[A-Za-z][A-Za-z0-9]{0,4}$")


def is_document(token: str) -> bool:
    return bool(_DOCN.match(token) or _LOOKS_LIKE_A_FILE.search(token))


def doc_key(token: str) -> str:
    """Normalise a document reference so two arms citing it differently agree.

    One arm writes `doc1`, another `doc1_seller_listing_description.md`, and
    treating those as different documents reports zero agreement between arms
    that agree completely. Outside the fixture the basename is the identity.
    """
    base = token.rsplit("/", 1)[-1]
    m = _DOCN.match(base)
    return f"doc{m.group(1)}" if m else base


def cited_lines(text: str) -> Set[str]:
    """The set of document lines a surface points at, as 'DOC:LINE'.

    A range is expanded, capped at 40 lines: an arm that cites doc4:1-500 has
    cited a document, not a line, and letting that flood the set would make it
    agree with everything.
    """
    # The stored surface comes from a JSON-encoded working log, so newlines
    # may still be the two characters `\` and `n`. Left as-is, every citation
    # at the start of a line parses as a document called `nREADME.md`.
    text = text.replace("\\n", "\n").replace("\\t", "\t")
    out: Set[str] = set()
    seen: list = []
    for m in _CITE.finditer(text):
        if _NOT_A_LINE.match(text[m.end():m.end() + 12]):
            continue
        token = m.group(1)
        if not is_document(token):
            continue
        seen.append((doc_key(token), m.group(2), m.group(3)))
    for m in _CITE_PROSE.finditer(text):
        seen.append((f"doc{m.group(1)}", m.group(2), None))
    for doc, lo, hi in seen:
        lo_i = int(lo)
        hi_i = int(hi) if hi else lo_i
        if hi_i < lo_i or hi_i - lo_i > 40:
            hi_i = lo_i
        for n in range(lo_i, hi_i + 1):
            out.add(f"{doc}:{n}")
    return out


def per_document(text: str) -> dict:
    """How many distinct lines were cited in each document."""
    out: dict = {}
    for line in cited_lines(text):
        doc = line.rsplit(":", 1)[0]
        out[doc] = out.get(doc, 0) + 1
    return dict(sorted(out.items()))


def latest_run(world: str) -> Path:
    hits = sorted(CLAIMS.glob(f"*_{world}"))
    if not hits:
        raise SystemExit(f"no claims run found for world '{world}' under {CLAIMS}")
    return hits[-1]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("worlds", nargs="+", help="claims-run world names to compare")
    args = ap.parse_args()

    arms: Dict[str, Set[str]] = {}
    counts: Dict[str, object] = {}
    for w in args.worlds:
        d = latest_run(w)
        meta = json.loads((d / "meta.json").read_text())
        label = meta.get("resolved_model") or w
        arms[label] = cited_lines((d / "surface.md").read_text())
        counts[label] = meta.get("count")

    print("\n  arm                        declared   lines cited   documents")
    for label, lines in arms.items():
        docs = sorted({l.split(":")[0] for l in lines})
        print(f"  {label:<26} {str(counts[label]):>8}   {len(lines):>11}   "
              f"{','.join(d.replace('doc','') for d in docs)}")

    if len(arms) > 1:
        print("\n  pairwise agreement on cited lines")
        for a, b in itertools.combinations(arms, 2):
            sa, sb = arms[a], arms[b]
            both, either = sa & sb, sa | sb
            j = len(both) / len(either) if either else 0.0
            print(f"  {a} vs {b}: {len(both)} shared of {len(either)} "
                  f"— Jaccard {j:.2f}")

        allsets = list(arms.values())
        common = set.intersection(*allsets)
        print(f"\n  cited by every arm: {len(common)}")
        for label, lines in arms.items():
            others = set.union(*[s for k, s in arms.items() if k != label])
            print(f"  only {label}: {len(lines - others)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
