#!/usr/bin/env python3
"""Freeze the audit-arm prompts of the temperature sweep.

    python3 measure/temp_sweep/build.py

Writes measure/temp_sweep/prompts.jsonl: one row per prompt with its system
and user text, its answer schema, and its gold answer. Every call of the
sweep sends these bytes, so the only thing that changes between calls is the
temperature (and the sampling it governs).

The subject is the chhoto target. Gold answers come from
measure/known_answers/chhoto.yaml, where each was checked by hand. The system
text is the real method document (METHOD.md for an audit, REVIEW.md for a
review), followed by a short note on what this single call can and cannot do.
Files reach the model line-numbered as `123|text`, as the runner shows them.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "workflowsv2"))
from claims_audit.schemas import VERDICTS                        # noqa: E402
from audit_review.schemas import OBSERVATIONS                    # noqa: E402

HERE = Path(__file__).resolve().parent
TARGET = REPO / "workflowsv2/claims_audit/engagements/chhoto-full/target"
METHOD = REPO / "workflowsv2/claims_audit/method/METHOD.md"
REVIEW = REPO / "workflowsv2/audit_review/method/REVIEW.md"
ANSWERS = REPO / "measure/known_answers/chhoto.yaml"
# Test B's finding for no-cookies (claim 52): real_with_caveat where the right
# verdict is contradicted. The review prompt asks whether it holds.
TEST_B_README = (REPO / "workflowsv2/claims_audit/engagements/cmp-chhoto-qwen38fp4-med/runs"
                 / "2026-10-17T23-12-18Z_audit_cmp-chhoto-qwen38fp4-med_readme_md_2026-10-17T23-12-18Z")

AUDIT_NOTE = """

---

THIS CALL. You are auditing one claim, not a claim source. You cannot call
tools or search: the files below are all the materials you have, shown with
their line numbers as `123|text`. Some may not bear on the claim. Decide the
claim on the method above, cite from the files shown, and answer with the
JSON object the schema asks for and nothing else. `lines` is the first and
last line cited; `quote` is copied from those lines without the line numbers.
"""

REVIEW_NOTE = """

---

THIS CALL. You are reviewing one finding. You cannot call tools or search:
the files below are the materials the finding cites, shown with their line
numbers as `123|text`. Make the four observations of §6 on this finding and
answer with the JSON object the schema asks for and nothing else.
"""

AUDIT_SCHEMA = {
    "type": "object",
    "properties": {
        "verdict": {"enum": list(VERDICTS)},
        "gap": {"type": "string"},
        "evidence": {"type": "array", "items": {
            "type": "object",
            "properties": {
                "document": {"type": "string"},
                "lines": {"type": "array", "items": {"type": "integer"},
                          "minItems": 2, "maxItems": 2},
                "quote": {"type": "string"},
                "shows": {"type": "string"}},
            "required": ["document", "lines", "quote", "shows"]}}},
    "required": ["verdict", "gap", "evidence"]}

REVIEW_SCHEMA = {
    "type": "object",
    "properties": {**{k: {"enum": list(v)} for k, v in OBSERVATIONS.items()},
                   "note": {"type": "string"}},
    "required": [*OBSERVATIONS, "note"]}

# id -> (known answer id, files shown). The files are those the answer's
# basis names, plus one that does not decide the claim.
AUDITS = {
    "a1_no_cookies": ("no-cookies", [
        "backend/src/main.rs", "backend/src/services/post.rs",
        "backend/src/services/get.rs"]),
    "a2_shortening_behaviour": ("shortening-behaviour", [
        "backend/src/services/post.rs", "backend/src/services/utils.rs",
        "backend/src/tests/insertion.rs", "backend/src/services/get.rs"]),
    "a3_no_personal_info_logged": ("no-personal-info-logged", [
        "backend/src/database/events.rs", "backend/src/main.rs",
        "backend/src/services/get.rs"]),
    "a4_journal_mode_default": ("default-delete-journal-extra-synchronous", [
        "backend/src/config.rs", "backend/src/database/utils.rs",
        "backend/src/main.rs", "backend/src/services/get.rs"]),
}


def numbered(rel: str) -> str:
    body = (TARGET / rel).read_text(encoding="utf-8").splitlines()
    return f"=== {rel} ({len(body)} lines)\n" + "\n".join(
        f"{i}|{line}" for i, line in enumerate(body, 1))


def claim_quote(source: str, lines) -> str:
    body = (TARGET / source).read_text(encoding="utf-8").splitlines()
    return "\n".join(body[lines[0] - 1:lines[1]])


def main() -> int:
    answers = {a["id"]: a for a in yaml.safe_load(ANSWERS.read_text())["answers"]}
    method = METHOD.read_text(encoding="utf-8")
    rows = []
    for pid, (aid, files) in AUDITS.items():
        a = answers[aid]
        user = (f"Claim source: {a['source']}, lines {a['lines'][0]}-{a['lines'][1]}.\n"
                f"Quote:\n{claim_quote(a['source'], a['lines'])}\n\n"
                f"Statement to decide: {a['statement']}\n\n"
                + "\n\n".join(numbered(f) for f in files))
        rows.append({"id": pid, "arm": "direct", "kind": "audit",
                     "system": method + AUDIT_NOTE, "user": user,
                     "schema": AUDIT_SCHEMA,
                     "gold": {"answer": aid, "verdict": a["verdict"],
                              "cites_any": a.get("cites_any")},
                     "files": files})

    findings = json.loads((TEST_B_README / "findings.json").read_text())["findings"]
    claims = json.loads((TEST_B_README / "claims.json").read_text())
    claims = claims["claims"] if isinstance(claims, dict) else claims
    f52 = next(f for f in findings if f["claim_id"] == 52)
    c52 = next(c for c in claims if c["id"] == 52)
    cited = sorted({e["document"] for e in f52["evidence"] if e.get("document")}
                   | {"backend/src/services/post.rs"})
    cited = [d for d in cited if (TARGET / d).is_file()]
    user = ("The claim, as frozen on the claim surface:\n"
            + json.dumps({k: c52[k] for k in ("id", "quote", "lines", "statement")},
                         indent=1)
            + "\n\nThe audit's finding for it:\n" + json.dumps(f52, indent=1)
            + "\n\n" + "\n\n".join(numbered(d) for d in cited))
    rows.append({"id": "r1_review_no_cookies", "arm": "direct", "kind": "review",
                 "system": REVIEW.read_text(encoding="utf-8") + REVIEW_NOTE,
                 "user": user, "schema": REVIEW_SCHEMA,
                 # The right verdict is contradicted (no-cookies), so a
                 # finding of real_with_caveat does not hold.
                 "gold": {"holds": False, "answer": "no-cookies"},
                 "files": cited})

    out = HERE / "prompts.jsonl"
    out.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    for r in rows:
        print(f"{r['id']:30s} system {len(r['system']):6d} chars, "
              f"user {len(r['user']):6d} chars, files {r['files']}")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
