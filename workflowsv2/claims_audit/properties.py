#!/usr/bin/env python3
"""Select the property questions for one engagement: which properties the
software has; which of their statements apply to it and are not already
asked by the seller's claims or the buyer's questions; and which of those
matter to this buyer. Two calls under method/PROPERTY_SELECTION.md
(identify, screen), then the tier rating of method/TIERS.md on what is left,
in batches.

    python3 workflowsv2/claims_audit/properties.py --engagement <name> --model <model.yaml>
        [--claims <run>/claims.json ...] [--identify-only] [--label <text>]

WHAT IT READS. The property list (method/PROPERTIES.md) and the statements
under each property (method/PROPERTY_QUESTIONS.md); the list of files in the
engagement's target; the newest composition scan's components, when there is
one; the seller's claims from the frozen surfaces of the engagement's
non-question claim sources, plus any claims.json named with --claims; and
the buyer's questions from the buyer question source.

WHAT IT WRITES, under <engagement>/properties/<stamp>[_<label>]/:
  identify.json   the verdict on every property, with evidence and reason
  screen.json     for every candidate statement: applies, and overlap
  tiers.json      the tier rating of every statement that applies and is
                  not already asked, against the engagement's transaction,
                  thresholds and reliance statement
  questions.md    a question source: the tier 1 statements to test; under
                  `# Not applicable`, the statements screened out or rated
                  tier 2 or 3, each with its reason, so the report lists them
                  as not tested; statements already asked, in a comment
  meta.json       inputs, model and counts

Nothing is added to engagement.yaml. The practice reads identify.json and
compare.json, edits questions.md, copies it to questions/ and names it there
as a `practice` question source; questions.py then builds its surface.
"""
from __future__ import annotations

import argparse
import json
import logging
import re
import sys
import types
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src"), str(HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import engagement_state as state              # noqa: E402
from workflowsv2.emit import emit                               # noqa: E402
from chat.workflow import load_workflow                         # noqa: E402
from decompose import backend_from_model                        # noqa: E402

logger = logging.getLogger("claims_audit.properties")

METHOD = HERE / "method" / "PROPERTY_SELECTION.md"
PROPERTIES = HERE / "method" / "PROPERTIES.md"
STATEMENTS = HERE / "method" / "PROPERTY_QUESTIONS.md"

#: Directories never listed: version control, installed dependencies, build
#: output. Their contents are the composition scan's business, not the code's.
SKIP_DIRS = {".git", "node_modules", "vendor", "dist", "build", "__pycache__",
             ".venv", "venv", ".next", ".nuxt", "target", ".idea", ".vscode"}
#: Above this many files, a directory with more than DIR_SUMMARY_AT direct
#: files is shown as one line with its count and file types.
MAX_LISTED_FILES = 2500
DIR_SUMMARY_AT = 30
MAX_COMPONENTS = 800


# ---------------------------------------------------------------- catalog

def load_properties() -> List[Dict[str, str]]:
    """Rows of the tables in PROPERTIES.md: id, name, shows-as."""
    out = []
    for line in PROPERTIES.read_text(encoding="utf-8").splitlines():
        m = re.match(r"\|\s*([A-H]\d+)\s*\|(.*?)\|(.*?)\|", line)
        if m:
            out.append({"id": m.group(1), "name": m.group(2).strip(),
                        "shows_as": m.group(3).strip()})
    if not out:
        raise SystemExit(f"{PROPERTIES} has no property rows")
    return out


def load_statements() -> Dict[str, Dict[str, Any]]:
    """{property id: {"title", "covered": [...], "items": [{"id", "text", "source"}]}}
    from PROPERTY_QUESTIONS.md, numbered as the review page numbered them."""
    out: Dict[str, Dict[str, Any]] = {}
    cur, src, n = None, None, 0
    for line in STATEMENTS.read_text(encoding="utf-8").splitlines():
        m = re.match(r"# ([A-H]\d+)\. (.*)", line)
        if m:
            cur, src, n = m.group(1), None, 0
            out[cur] = {"title": m.group(2).strip(), "covered": [], "items": []}
            continue
        if cur is None:
            continue
        if line.startswith("<!--"):
            note = line[4:].rstrip()[:-3].strip()
            if note.startswith("covered"):
                out[cur]["covered"].append(note)
            else:
                src = note
            continue
        if line.strip() and not line.startswith("#"):
            n += 1
            out[cur]["items"].append({"id": f"{cur}.{n}", "text": line.strip(), "source": src})
            src = None
    return out


# ---------------------------------------------------------------- evidence

def file_list(target: Path) -> str:
    files: List[Path] = []
    for p in sorted(target.rglob("*")):
        if p.is_file() and not any(part in SKIP_DIRS for part in p.relative_to(target).parts):
            files.append(p.relative_to(target))
    if len(files) <= MAX_LISTED_FILES:
        return "\n".join(str(f) for f in files)
    by_dir: Dict[Path, List[Path]] = {}
    for f in files:
        by_dir.setdefault(f.parent, []).append(f)
    lines = []
    for d in sorted(by_dir):
        fs = by_dir[d]
        if len(fs) > DIR_SUMMARY_AT:
            exts = Counter(f.suffix or "(none)" for f in fs).most_common(5)
            lines.append(f"{d}/  ({len(fs)} files: " + ", ".join(f"{e} {c}" for e, c in exts) + ")")
        else:
            lines.extend(str(f) for f in fs)
    return "\n".join(lines)


def components(eng_dir: Path) -> str:
    comp = eng_dir / "composition"
    scans = sorted(d for d in comp.iterdir() if (d / "components.json").is_file()) \
        if comp.is_dir() else []
    if not scans:
        return ""
    rows = json.loads((scans[-1] / "components.json").read_text(encoding="utf-8"))
    seen = sorted({(r.get("type") or "", r.get("name") or "") for r in rows if r.get("name")})
    return "\n".join(f"{t}: {n}" for t, n in seen[:MAX_COMPONENTS])


def _claims_of(path: Path, source: str) -> List[Dict[str, str]]:
    obj = json.loads(path.read_text(encoding="utf-8"))
    rows = obj.get("claims") if isinstance(obj, dict) else obj
    return [{"ref": f"claim {source}#{c.get('id')}", "statement": c.get("statement") or ""}
            for c in rows or [] if c.get("statement")]


def seller_claims(eng_dir: Path, extra: List[Path]) -> List[Dict[str, str]]:
    from client_ui import jobs
    out = []
    for src in state.claim_sources(eng_dir):
        if state.question_kind(eng_dir, src):
            continue
        f = jobs.surface_file(eng_dir, src)
        if f.is_file():
            out.extend(_claims_of(f, src))
    for p in extra:
        obj = json.loads(p.read_text(encoding="utf-8"))
        src = (obj.get("claim_source") if isinstance(obj, dict) else None) or p.parent.name
        out.extend(_claims_of(p, src))
    return out


def buyer_questions(eng_dir: Path) -> List[Dict[str, str]]:
    from client_ui import jobs
    import questions
    out = []
    for src in state.claim_sources(eng_dir):
        if state.question_kind(eng_dir, src) != "buyer":
            continue
        f = jobs.surface_file(eng_dir, src)
        rows = json.loads(f.read_text(encoding="utf-8"))["claims"] if f.is_file() else \
            questions.claims(state.claim_source_file(eng_dir, src).read_text(encoding="utf-8"), "buyer")
        out.extend({"ref": f"question {src}#{r['id']}", "statement": r["statement"]} for r in rows)
    return out


def _listed(rows: List[Dict[str, str]]) -> str:
    return "\n".join(f"  {r['ref']}: {r['statement']}" for r in rows) or "  (none)"


# ---------------------------------------------------------------- calls

def identify_schema(ids: List[str]) -> Dict[str, Any]:
    item = {"type": "object", "properties": {
        "id": {"type": "string", "enum": ids},
        "verdict": {"type": "string", "enum": ["present", "unsure", "absent"]},
        "evidence": {"type": "array", "items": {"type": "string"}},
        "reason": {"type": "string"}},
        "required": ["id", "verdict", "evidence", "reason"]}
    return {"type": "object", "properties": {"properties": {"type": "array", "items": item}},
            "required": ["properties"]}


def screen_schema(ids: List[str]) -> Dict[str, Any]:
    item = {"type": "object", "properties": {
        "id": {"type": "string", "enum": ids},
        "applies": {"type": "string", "enum": ["yes", "no", "unsure"]},
        "applies_reason": {"type": "string"},
        "overlap": {"type": "string", "enum": ["same", "broader", "none"]},
        "refs": {"type": "array", "items": {"type": "string"}},
        "reason": {"type": "string"}},
        "required": ["id", "applies", "applies_reason", "overlap", "refs", "reason"]}
    return {"type": "object", "properties": {"statements": {"type": "array", "items": item}},
            "required": ["statements"]}


def _call(backend, user: str, schema: Dict[str, Any], key: str, ids: List[str],
          max_tokens: int) -> Dict[str, Any]:
    out = emit(types.SimpleNamespace(backend=backend), load_workflow(METHOD), user, schema, max_tokens)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    rows = [r for r in (obj.get(key) or []) if isinstance(r, dict) and r.get("id") in ids]
    got = {r["id"] for r in rows}
    missing = [i for i in ids if i not in got]
    if missing:
        logger.warning("%s: no answer for %s", key, ", ".join(missing))
    return {"rows": rows, "missing": missing, "parse": out.get("parse"),
            "parse_error": out.get("parse_error"), "raw": out.get("raw")}


def identify(backend, props, claims, asked, files, comps) -> Dict[str, Any]:
    plist = "\n".join(f"  {p['id']}. {p['name']} — shows as: {p['shows_as']}" for p in props)
    user = ("IDENTIFY\n\nThe properties:\n" + plist +
            "\n\nThe seller's claims:\n" + _listed(claims) +
            "\n\nThe buyer's questions:\n" + _listed(asked) +
            "\n\nThe files in the materials:\n" + files +
            "\n\nThe dependencies (type: name):\n" + (comps or "  (no scan)") +
            "\n\nEmit the identification per §2.")
    return _call(backend, user, identify_schema([p["id"] for p in props]), "properties",
                 [p["id"] for p in props], 16384)


def screen(backend, cat, verdicts, items, claims, asked, files, comps) -> Dict[str, Any]:
    blocks, by_prop = [], {}
    for it in items:
        by_prop.setdefault(it["id"].split(".")[0], []).append(it)
    for pid, its in by_prop.items():
        v = verdicts.get(pid, {})
        blocks.append(f"{pid}. {cat[pid]['title']} ({v.get('verdict')}: {v.get('reason', '')})\n"
                      + "\n".join(f"  {i['id']}: {i['text']}" for i in its))
    user = ("SCREEN\n\nThe candidate statements, by property:\n\n" + "\n\n".join(blocks) +
            "\n\nThe seller's claims:\n" + _listed(claims) +
            "\n\nThe buyer's questions:\n" + _listed(asked) +
            "\n\nThe files in the materials:\n" + files +
            "\n\nThe dependencies (type: name):\n" + (comps or "  (no scan)") +
            "\n\nEmit the screen per §3.")
    ids = [i["id"] for i in items]
    return _call(backend, user, screen_schema(ids), "statements", ids, 32768)


def rate(backend, eng_name: str, items) -> Dict[str, Any]:
    """TIERS.md over the statements, in the tier stage's batches. The
    statements are numbered 1..n for the call; the record maps them back."""
    from workflowsv2.claims_audit import tiers, reliance
    from workflowsv2.claims_audit.runner import load_engagement
    eng = load_engagement(eng_name)
    if not (eng.get("transaction") or eng.get("thresholds")):
        logger.warning("%s records no transaction or thresholds; the rating has little to go on", eng_name)
    stmt = reliance.load(eng["dir"])
    rows = [{"id": n, "about": "target", "quote": it["text"], "statement": it["text"]}
            for n, it in enumerate(items, 1)]
    out: Dict[str, Dict[str, Any]] = {}
    unrated: List[str] = []
    for i in range(0, len(rows), tiers.BATCH):
        got = tiers.propose(backend, str(eng.get("transaction") or ""), str(eng.get("thresholds") or ""),
                            "the practice's property questions", rows[i:i + tiers.BATCH],
                            reliance.render(stmt) if stmt else "")
        for t in got["tiers"]:
            out[items[t["claim_id"] - 1]["id"]] = {"tier": t["tier"], "basis": t["basis"]}
        unrated += [items[n - 1]["id"] for n in got["unrated"]]
    if unrated:
        logger.warning("tiers: no rating for %s; they are kept as tier 1", ", ".join(unrated))
    return {"ratings": out, "unrated": unrated}


# ---------------------------------------------------------------- output

def questions_md(cat, verdicts: Dict[str, Dict[str, Any]], marks: Dict[str, Dict[str, Any]],
                 ratings: Dict[str, Dict[str, Any]]) -> str:
    lines = ["# Property questions selected for this engagement",
             "",
             "<!-- Drafted by properties.py from method/PROPERTY_QUESTIONS.md. Edit before use:",
             "     keep, drop or reword statements, then copy this file to questions/ and name it",
             "     in engagement.yaml as a practice question source. -->", ""]
    flat = lambda s: " ".join(str(s).replace("|", "/").split())
    left_out, not_tested = [], []
    for pid, sec in cat.items():
        v = verdicts.get(pid)
        if not v or v["verdict"] == "absent":
            continue
        head = [f"# {pid}. {sec['title']}"]
        if v["verdict"] == "unsure":
            head.append(f"<!-- property unsure: {v['reason']} -->")
        body = []
        for it in sec["items"]:
            m, r = marks.get(it["id"], {}), ratings.get(it["id"])
            if m.get("overlap") == "same":
                left_out.append(f"{it['id']} same as {', '.join(m.get('refs') or [])}: {m.get('reason', '')}")
            elif m.get("applies") == "no":
                not_tested.append(f"{it['text']} | Does not apply to this software: {flat(m.get('applies_reason', ''))}")
            elif r and r["tier"] != 1:
                not_tested.append(f"{it['text']} | Not material to this buyer (tier {r['tier']}): {flat(r['basis'])}")
            else:
                note = f"; broader than {', '.join(m.get('refs') or [])}" if m.get("overlap") == "broader" else ""
                note += "; applies: unsure" if m.get("applies") == "unsure" else ""
                body += [f"<!-- {it['id']}; {it['source'] or 'practice'}{note} -->", it["text"]]
        if body:
            lines += head + body + [""]
    if not_tested:
        lines += ["# Not applicable"] + not_tested + [""]
    if left_out:
        lines.append("<!-- Left out because a claim or question already asks the same:")
        lines.extend(f"     {x}" for x in left_out)
        lines.append("-->")
    return "\n".join(lines) + "\n"


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--model", required=True, type=Path)
    ap.add_argument("--claims", action="append", type=Path, default=[],
                    help="a claims.json of seller claims not in a frozen surface (repeatable)")
    ap.add_argument("--identify-only", action="store_true")
    ap.add_argument("--label", default=None)
    args = ap.parse_args()

    eng = state.ENGAGEMENTS / args.engagement
    if not eng.is_dir():
        raise SystemExit(f"no engagement {args.engagement}")
    target = state.target_dir(eng)
    out = eng / "properties" / (state.stamp() + (f"_{args.label}" if args.label else ""))
    out.mkdir(parents=True)

    props, cat = load_properties(), load_statements()
    claims, asked = seller_claims(eng, args.claims), buyer_questions(eng)
    files, comps = file_list(target), components(eng)
    backend = backend_from_model(args.model)

    ident = identify(backend, props, claims, asked, files, comps)
    (out / "identify.json").write_text(json.dumps(ident, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    verdicts = {r["id"]: r for r in ident["rows"]}
    counts = Counter(r["verdict"] for r in ident["rows"])
    logger.info("identify: %s", dict(counts))

    meta = {"engagement": args.engagement, "model": str(args.model), "target": str(target),
            "seller_claims": len(claims), "buyer_questions": len(asked),
            "files_listed_lines": files.count("\n") + 1, "components": comps.count("\n") + 1 if comps else 0,
            "identify": dict(counts), "identify_parse": ident["parse"]}
    if not args.identify_only:
        items = [it for pid, sec in cat.items() if verdicts.get(pid, {}).get("verdict") in ("present", "unsure")
                 for it in sec["items"]]
        scr = screen(backend, cat, verdicts, items, claims, asked, files, comps) if items \
            else {"rows": [], "missing": [], "parse": None}
        (out / "screen.json").write_text(json.dumps(scr, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
        marks = {r["id"]: r for r in scr["rows"]}
        to_rate = [it for it in items if marks.get(it["id"], {}).get("overlap") != "same"
                   and marks.get(it["id"], {}).get("applies") != "no"]
        rated = rate(backend, args.engagement, to_rate) if to_rate else {"ratings": {}, "unrated": []}
        (out / "tiers.json").write_text(json.dumps(rated, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
        (out / "questions.md").write_text(questions_md(cat, verdicts, marks, rated["ratings"]), encoding="utf-8")
        tiers_n = Counter(r["tier"] for r in rated["ratings"].values())
        meta.update({"candidates": len(items),
                     "applies": dict(Counter(r["applies"] for r in scr["rows"])),
                     "overlap": dict(Counter(r["overlap"] for r in scr["rows"])),
                     "rated": len(to_rate), "tiers": {str(k): v for k, v in sorted(tiers_n.items())},
                     "unrated": rated["unrated"], "to_test": tiers_n.get(1, 0) + len(rated["unrated"]),
                     "screen_parse": scr["parse"]})
        logger.info("screen: applies %s, overlap %s; tiers %s; to test %d", meta["applies"],
                    meta["overlap"], meta["tiers"], meta["to_test"])
    (out / "meta.json").write_text(json.dumps(meta, indent=1) + "\n", encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
