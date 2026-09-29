#!/usr/bin/env python3
"""Extract the clinical layer of one chapter from its text: diseases, their
variants, and their features, each feature citing the verses it comes from.

    python3 workflowsv2/nidana_kb/extract.py --chapter 2 --model measure/models/<route>.yaml
            [--kb <dir>] [--replace]

ONE MODEL CALL PER SECTION of the chapter (a Heading 2 in the translation),
under method/EXTRACT.md, schema-constrained through workflowsv2/emit.py. Each
call is given the section's passages, the diseases already recorded for the
chapter (so ids stay the same across sections), and the lexicon features
nearest in meaning to the section's word glosses (so a feature recorded in
another chapter is reused rather than duplicated).

WHAT IS CHECKED IN CODE. Every cited verse id must exist and its text must
hold the quoted words (kb.KB.resolve_citation); a citation that fails is
removed, and a feature with none left is kept as `uncited`, which the
consultation does not use. A variant must belong to its disease, and a
feature id must be in the lexicon or described in the same answer. Every
failure is a line in <kb>/issues.jsonl for the reviewer.

WHAT IT WRITES. <kb>/clinical/chNN.json, new entries in <kb>/lexicon.json
(existing entries are never changed here), and every raw answer in
<kb>/extract_log/chNN_<time>.jsonl. Everything written is `draft` until a
reviewer accepts it (review.py). A chapter already extracted is not
extracted again without --replace, which starts that chapter's file afresh.
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
import types
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import issues                                  # noqa: E402
from workflowsv2.nidana_kb import kb as kbmod                   # noqa: E402
from workflowsv2.nidana_kb import schemas                       # noqa: E402

STAGE = "extract"
METHOD_PATH = "workflowsv2/nidana_kb/method/EXTRACT.md"
NEAREST = 5          # lexicon candidates offered per word gloss


def sections(chapter_doc: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The chapter's passages grouped by section, in document order."""
    out: List[Dict[str, Any]] = []
    for ps in chapter_doc.get("passages", []):
        if not out or out[-1]["section"] != ps.get("section", ""):
            out.append({"section": ps.get("section", ""), "passages": []})
        out[-1]["passages"].append(ps)
    return out


def render_section(kb: kbmod.KB, section: Dict[str, Any]) -> str:
    parts = [f"Section: {section['section'] or '(untitled)'}"]
    for ps in section["passages"]:
        parts.append(f"\n--- Passage {ps['id']}"
                     + (" (NO VERSE: cannot be cited)" if not ps["verses"] else ""))
        for vid in ps["verses"]:
            v = kb.verse(vid) or {}
            parts.append(f"{vid}\n  Devanagari: {v.get('sa', '')}\n  IAST: {v.get('iast') or '(none)'}")
        if ps.get("words"):
            parts.append("Words:\n" + "\n".join(f"  {w['term']} — {w['en']}" for w in ps["words"]))
        if ps.get("translation"):
            parts.append("Translation: " + ps["translation"])
        for name, segs in (ps.get("commentary") or {}).items():
            parts.append(f"{name.capitalize()} (commentary): "
                         + " ".join(x.get("en", "") for x in segs))
        for n in ps.get("notes") or []:
            parts.append("Note: " + n)
    return "\n".join(parts)


def nearby_features(kb: kbmod.KB, section: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """Lexicon features nearest in meaning to the section's word glosses."""
    if not kb.lexicon:
        return {}
    out: Dict[str, Dict[str, Any]] = {}
    for ps in section["passages"]:
        for w in ps.get("words") or []:
            for c in kb.candidates(f"{w['en']} ({w['term']})", k=NEAREST):
                out[c["id"]] = kb.lexicon[c["id"]]
    return out


def build_user(kb: kbmod.KB, chapter: int, section: Dict[str, Any],
               diseases: List[Dict[str, Any]], offered: Dict[str, Dict[str, Any]]) -> str:
    known = [{"id": d["id"], "names": d.get("names"),
              "variants": [v["id"] for v in d.get("variants") or []]} for d in diseases]
    feats = [{"id": k, "en": v.get("en"), "clinical": v.get("clinical"), "sa": v.get("sa")}
             for k, v in offered.items()]
    return (f"Chapter {chapter}.\n\nDiseases already recorded for this chapter (reuse these ids):\n"
            + json.dumps(known, ensure_ascii=False) + "\n\nFeatures already recorded "
            "that may be the same as features here (reuse the id when one is the same):\n"
            + json.dumps(feats, ensure_ascii=False) + "\n\n" + render_section(kb, section)
            + "\n\nRecord the diseases and features these verses state, per EXTRACT.md.")


def merge(diseases: Dict[str, Dict[str, Any]], answer: Dict[str, Any], kb: kbmod.KB,
          lexicon: Dict[str, Any], chapter: int, section: str
          ) -> List[str]:
    """Fold one section's answer into the chapter's diseases and the lexicon.
    Returns the problems found, each prefixed with where."""
    problems: List[str] = []
    for nf in answer.get("new_features") or []:
        fid = nf.get("id") or ""
        if not fid.startswith("F."):
            problems.append(f"new feature id '{fid}' does not start with 'F.'")
            continue
        if fid not in lexicon:
            lexicon[fid] = {"en": nf.get("en", ""), "clinical": nf.get("clinical", ""),
                            "sa": nf.get("sa") or [], "lay_question": nf.get("lay_question", ""),
                            "observable": nf.get("observable") or [], "signal": None,
                            "first_seen": f"chapter {chapter}", "status": "draft",
                            "reviewed_by": None, "reviewed_at": None}
    for d in answer.get("diseases") or []:
        did = d.get("id") or ""
        if not did:
            problems.append("a disease with no id")
            continue
        cur = diseases.setdefault(did, {"id": did, "chapter": chapter, "names": d.get("names") or {},
                                        "variants": [], "features": [], "status": "draft",
                                        "reviewed_by": None, "reviewed_at": None})
        have = {v["id"] for v in cur["variants"]}
        cur["variants"] += [v for v in d.get("variants") or [] if v.get("id") not in have]
        for f in d.get("features") or []:
            kept, why = schemas.check_feature(f, cur, kb, set(lexicon))
            kept["section"] = section
            problems += [f"{did} / {f.get('feature')}: {w}" for w in why]
            same = next((x for x in cur["features"] if (x["feature"], x["role"], x["variant"])
                         == (kept["feature"], kept["role"], kept["variant"])), None)
            if same:
                same["verses"] = sorted(set(same["verses"]) | set(kept["verses"]))
                if same["verses"]:
                    same["status"] = "draft"
            else:
                cur["features"].append(kept)
    return problems


def extract_chapter(kb_dir: Path, chapter: int, emit_fn: Callable[[str, str, Dict[str, Any], int], Dict[str, Any]],
                    method_text: str, replace: bool = False, max_tokens: int = 8192,
                    model: str = "") -> Dict[str, Any]:
    """`emit_fn(system, user, schema, max_tokens)` returns emit()'s dict;
    injected so this is testable without a model."""
    kb = kbmod.KB(kb_dir)
    text = kbmod.read_json(kbmod.text_path(kb_dir, chapter))
    if text is None:
        raise SystemExit(f"chapter {chapter} has no text in {kb_dir}; ingest it first")
    out_path = kbmod.clinical_path(kb_dir, chapter)
    if out_path.is_file() and not replace:
        raise SystemExit(f"{out_path} exists; use --replace to extract the chapter again")
    lexicon = dict(kb.lexicon)
    diseases: Dict[str, Dict[str, Any]] = {}
    notes: List[Dict[str, str]] = []
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    log = kb_dir / "extract_log" / f"ch{chapter:02d}_{stamp}.jsonl"
    log.parent.mkdir(parents=True, exist_ok=True)
    summary = {"sections": 0, "unparsed": 0, "problems": 0}
    for sec in sections(text):
        if not any(ps["verses"] for ps in sec["passages"]):
            issues.note(kb_dir, stage=STAGE, code="section_without_verses", severity="note",
                        text=f"chapter {chapter}, '{sec['section']}': no verses; not extracted")
            continue
        summary["sections"] += 1
        user = build_user(kb, chapter, sec, list(diseases.values()), nearby_features(kb, sec))
        res = emit_fn(method_text, user, schemas.extract_schema(), max_tokens)
        with log.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps({"section": sec["section"], "parse": res.get("parse"),
                                 "raw": res.get("raw")}, ensure_ascii=False) + "\n")
        obj = res.get("obj")
        if not isinstance(obj, dict):
            summary["unparsed"] += 1
            issues.note(kb_dir, stage=STAGE, code="unparsed", severity="check",
                        text=f"chapter {chapter}, '{sec['section']}': the answer did not parse "
                             f"({res.get('parse_error')}); nothing recorded from it")
            continue
        problems = merge(diseases, obj, kb, lexicon, chapter, sec["section"])
        summary["problems"] += len(problems)
        for pr in problems:
            issues.note(kb_dir, stage=STAGE, code="citation", severity="check",
                        text=f"chapter {chapter}, '{sec['section']}': {pr}")
        notes += [{"section": sec["section"], "text": n} for n in obj.get("notes") or []]
    kbmod.write_json(out_path, {"chapter": chapter, "extracted_at": stamp, "model": model,
                                "diseases": list(diseases.values()), "notes": notes})
    kbmod.write_json(kbmod.lexicon_path(kb_dir), lexicon)
    summary.update(diseases=len(diseases),
                   features=sum(len(d["features"]) for d in diseases.values()),
                   uncited=sum(1 for d in diseases.values() for f in d["features"]
                               if f["status"] == "uncited"),
                   lexicon=len(lexicon), log=str(log))
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--chapter", type=int, required=True)
    ap.add_argument("--model", type=Path, required=True, help="YAML with an llm_config block")
    ap.add_argument("--kb", type=Path, default=None)
    ap.add_argument("--replace", action="store_true")
    ap.add_argument("--max-tokens", type=int, default=8192)
    args = ap.parse_args()
    from chat.workflow import load_workflow                     # noqa: E402
    from workflowsv2.claims_audit.decompose import backend_from_model  # noqa: E402
    from workflowsv2.emit import emit                           # noqa: E402
    loop = types.SimpleNamespace(backend=backend_from_model(args.model))
    summary = extract_chapter(
        kbmod.kb_root(args.kb), args.chapter,
        lambda s, u, sch, mt: emit(loop, s, u, sch, mt),
        load_workflow(str(REPO / METHOD_PATH)), replace=args.replace,
        max_tokens=args.max_tokens, model=args.model.stem)
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
