#!/usr/bin/env python3
"""Drive a consultation with the practitioner, keep the case record, and
record the practitioners' own assessment beside the program's.

    python3 workflowsv2/ayur_consult/runner.py --case <id> --new --client-ref <ref> \\
        --practitioner "<name>:intern" [--practitioner "<name>:professor"] \\
        [--model measure/models/fw_glm53flash.yaml]
    python3 workflowsv2/ayur_consult/runner.py --case <id>                 # continue
    python3 workflowsv2/ayur_consult/runner.py --case <id> --finish --by <name>
    python3 workflowsv2/ayur_consult/runner.py --case <id> --review-template > review.yaml
    python3 workflowsv2/ayur_consult/runner.py --case <id> --review review.yaml \\
        --by <name> --role professor

ONE DIRECTORY PER CASE, one case per consultation, under $AYUR_CASES
(default ~/ayur_cases), outside the repository because it holds patient
information. The patient is named only by the `client_ref` the practice
gives; the case record holds no name. A later visit is a new case with the
same client_ref.

  case.yaml            client_ref, practitioners, created, knowledge base version
  state.json           stages: {name: {value, at, by}}
  case.json            the case record, re-emitted whole each turn (CONSULT.md §6)
  differential.json    the differential and next questions as last computed
  transcript.jsonl     the conversation, practitioner and agent
  issues.jsonl         what a person must look at (workflowsv2/issues.py)
  assessment/system.json          frozen by --finish
  assessment/review_<role>_<name>.json   each practitioner's assessment

THE ORDER IS ENFORCED. --finish freezes the program's differential before
anyone records theirs, and --review is refused until then, so the program's
answer cannot be revised after a practitioner has seen the professor's.
Neither is the agent's to do.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import os
import re
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("ayur_consult.run")
logger.setLevel(logging.INFO)

from chat.workflow import load_workflow                        # noqa: E402
from utils.file_utils import atomic_write_text                 # noqa: E402
from workflowsv2 import issues                                  # noqa: E402
from workflowsv2.ayur_consult import differential as dx        # noqa: E402
from workflowsv2.ayur_consult import schemas                   # noqa: E402

SCENARIO = HERE / "scenario.yaml"
METHOD_PATH = "workflowsv2/ayur_consult/method/CONSULT.md"
STAGE = "consult"
SOURCE = "User"
ROLES = ("intern", "professor", "consultant")
VERDICTS = ("agree", "disagree", "unsure")
CANDIDATES_PER_FINDING = 6

OPENING = ("A practitioner is starting a consultation. Begin: say in two "
           "sentences what you will do during it, per CONSULT.md §1, and ask "
           "for the patient's presenting complaint.")
RETURNING = ("The practitioner has returned to continue this consultation. "
             "Summarise the case in two sentences from the ledger below, then "
             "give the next questions, per CONSULT.md §3.")


# ---- the case directory -----------------------------------------------------

def cases_root() -> Path:
    return Path(os.environ.get("AYUR_CASES") or Path.home() / "ayur_cases")


def now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def write_json(path: Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    atomic_write_text(path, json.dumps(obj, indent=1, ensure_ascii=False) + "\n")


def read_json(path: Path, default: Any = None) -> Any:
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else default


def new_case(case_dir: Path, client_ref: str, practitioners: List[Dict[str, str]],
             kb_version: Optional[str]) -> None:
    if case_dir.exists():
        raise SystemExit(f"{case_dir} exists; a case is created once")
    case_dir.mkdir(parents=True)
    atomic_write_text(case_dir / "case.yaml", yaml.safe_dump(
        {"client_ref": client_ref, "practitioners": practitioners, "created": now(),
         "kb_version": kb_version}, sort_keys=False, allow_unicode=True))
    write_json(case_dir / "state.json", {"stages": {"created": {"value": "yes", "at": now(), "by": None}}})
    write_json(case_dir / "case.json", schemas.empty_form())


def set_stage(case_dir: Path, name: str, value: str, by: Optional[str]) -> None:
    st = read_json(case_dir / "state.json", {"stages": {}})
    st["stages"][name] = {"value": value, "at": now(), "by": by}
    write_json(case_dir / "state.json", st)


def stage(case_dir: Path, name: str) -> Optional[Dict[str, Any]]:
    return (read_json(case_dir / "state.json", {"stages": {}}) or {}).get("stages", {}).get(name)


def read_transcript(case_dir: Path) -> List[Tuple[str, str]]:
    p = case_dir / "transcript.jsonl"
    if not p.is_file():
        return []
    rows = [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines() if l.strip()]
    return [(r["who"], r["text"]) for r in rows]


def append_transcript(case_dir: Path, who: str, text: str) -> None:
    with (case_dir / "transcript.jsonl").open("a", encoding="utf-8") as fh:
        fh.write(json.dumps({"at": now(), "who": who, "text": text}, ensure_ascii=False) + "\n")


# ---- the model's config ----------------------------------------------------

def build_config(case_dir: Path, world: str, model_path: Optional[Path]
                 ) -> Tuple[str, Dict[str, Any]]:
    """As intake/runner.build_config: the model REPLACES llm_config."""
    from launcher import parse_characters                      # noqa: E402
    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})
    if model_path:
        doc = yaml.safe_load(Path(model_path).read_text(encoding="utf-8")) or {}
        llm = dict(doc.get("llm_config") or {})
        if not llm:
            raise SystemExit(f"{model_path}: no llm_config block")
        for ch in (scenario.get("characters") or {}).values():
            if isinstance(ch, dict) and ch.get("mode") == "chat":
                ch["llm_config"] = dict(llm)
        scen_llm.update(llm)
    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world
    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat) != 1:
        raise SystemExit(f"expected 1 chat character, found {len(chat)}")
    name, cfg = chat[0]
    cfg["autonomy_enabled"] = False
    cfg["external_repo"] = str(case_dir)
    cfg["inspect_repo"] = str(case_dir)
    return name, cfg


# ---- the per-turn work ------------------------------------------------------

def _key(text: str) -> str:
    return re.sub(r"\s+", " ", (text or "").strip().lower())


def fill_form(emit_fn: Callable[[str, str, Dict[str, Any], int], Dict[str, Any]],
              method_text: str, transcript: List[Tuple[str, str]],
              previous: Dict[str, Any], max_tokens: int) -> Dict[str, Any]:
    """The whole case record from the conversation so far. A finding whose
    words are unchanged keeps the feature it was matched to. On an answer that
    does not parse, the previous record stands."""
    convo = "\n\n".join(f"{who}: {text}" for who, text in transcript)
    user = ("The consultation so far, practitioner and you:\n\n" + convo
            + "\n\nThe case record as it stood before the practitioner's last turn:\n\n"
            + json.dumps(previous, ensure_ascii=False, indent=1)
            + "\n\nEmit the whole case record as the consultation now supports it, per "
              "CONSULT.md §6. A field the practitioner has not given is empty.")
    out = emit_fn(method_text, user, schemas.case_schema(), max_tokens)
    form = out.get("obj") if isinstance(out.get("obj"), dict) else None
    if form is None:
        return {"form": previous, "updated": False, "parse_error": out.get("parse_error")}
    before = {_key(f.get("text")): f for f in previous.get("findings") or []}
    for f in form.get("findings") or []:
        old = before.get(_key(f.get("text")))
        f["feature_id"] = old.get("feature_id") if old else None
        f["matched"] = bool(old and old.get("matched"))
    return {"form": form, "updated": True, "parse_error": None}


def map_findings(emit_fn, kb, form: Dict[str, Any], max_tokens: int = 2048) -> List[str]:
    """Match each finding not yet matched to a lexicon feature: embedding
    candidates, then one call that picks a candidate or none for each. A
    finding is judged once; its words changing makes it a new finding.
    Returns the texts left unmatched."""
    todo = [f for f in form.get("findings") or [] if not f.get("matched")]
    if not todo or not kb.lexicon:
        return [f["text"] for f in todo]
    offered: Dict[str, List[Dict[str, Any]]] = {}
    lines = []
    for f in todo:
        cands = kb.candidates(f["text"], k=CANDIDATES_PER_FINDING)
        offered[f["text"]] = cands
        lines.append(f"Finding: {f['text']}\n" + "\n".join(
            f"  {c['id']}: {c.get('en')} — {c.get('clinical')}; Sanskrit {', '.join(c.get('sa') or [])}"
            for c in cands))
    system = ("You match a practitioner's recorded finding to the feature of a "
              "clinical knowledge base that names the same symptom or sign. Each "
              "finding is listed with its candidate features, each candidate "
              "starting with its id (F.something). For each finding give one "
              "entry: `finding`, the finding's words exactly as listed; `why`, one "
              "sentence; `feature_id`, the id of the one candidate that is the "
              "same finding, or an empty string when none is. A candidate that is "
              "related but not the same (a broader or narrower sign, a different "
              "body site) is not the same. Emit one JSON object.")
    out = emit_fn(system, "\n\n".join(lines), schemas.mapping_schema(), max_tokens)
    chosen = {_key(m.get("finding")): m.get("feature_id") or ""
              for m in ((out.get("obj") or {}).get("mappings") or [])}
    left = []
    for f in todo:
        pick = chosen.get(_key(f["text"]))
        valid = {c["id"] for c in offered[f["text"]]}
        if pick is None or (pick and pick not in valid):
            # Not answered, or answered with something that is not one of
            # its candidates: not judged; asked again next turn.
            left.append(f["text"])
            continue
        f["feature_id"] = pick or None
        f["matched"] = True
        if not f["feature_id"]:
            left.append(f["text"])
    return left


def compute(kb, form: Dict[str, Any]) -> Dict[str, Any]:
    variants = kb.variants()
    findings = form.get("findings") or []
    vik = (form.get("client") or {}).get("vikriti_doshas") or []
    ranked = dx.rank(variants, findings, vik)
    return {"differential": ranked, "next_questions": dx.next_questions(ranked, variants, findings),
            "weights": dx.WEIGHTS, "kb_version": kb.version}


def check_citations(kb, case_dir: Path, reply: str, turn: int) -> List[str]:
    """Every verse id in the agent's reply must be in the knowledge base."""
    bad = [vid for vid in kb.cited_ids(reply) if not kb.resolve_citation(vid)["ok"]]
    for vid in bad:
        issues.note(case_dir, stage=STAGE, code="citation", severity="check",
                    text=f"turn {turn}: the reply cites {vid}, which is not in the knowledge base")
    return bad


# ---- finishing and review ---------------------------------------------------

def finish(case_dir: Path, kb, by: str) -> Dict[str, Any]:
    out = case_dir / "assessment" / "system.json"
    if out.is_file():
        raise SystemExit(f"{out} exists; the program's assessment is frozen once")
    form = read_json(case_dir / "case.json", schemas.empty_form())
    frozen = {"frozen_at": now(), "by": by, "kb_version": kb.version, "case": form, **compute(kb, form)}
    write_json(out, frozen)
    set_stage(case_dir, "finished", "yes", by)
    return frozen


def review_template(case_dir: Path) -> str:
    sysa = read_json(case_dir / "assessment" / "system.json")
    if sysa is None:
        raise SystemExit("the case is not finished; --finish first")
    cands = [{"id": r["id"], "name": r["label"], "score": r["score"],
              "verdict": "", "comment": ""} for r in sysa["differential"][:5]]
    doc = {"system_candidates": cands,
           "diagnosis": {"kb_id": "", "text": ""},
           "findings_mismatched": [], "missing_features": [], "reason": ""}
    return ("# Your assessment of this case. For each of the program's candidates set\n"
            "# verdict to agree, disagree or unsure. diagnosis.kb_id is a disease or\n"
            "# variant id from the knowledge base when yours is one; diagnosis.text is\n"
            "# your diagnosis in words. findings_mismatched: entries {text, should_be}\n"
            "# for a finding matched to the wrong feature. missing_features: features\n"
            "# the text gives for your diagnosis that the program did not ask about.\n"
            "# Submit with: runner.py --case <id> --review <this file> --by <name> --role <role>\n"
            + yaml.safe_dump(doc, sort_keys=False, allow_unicode=True))


def review(case_dir: Path, kb, path: Path, by: str, role: str) -> Path:
    if stage(case_dir, "finished") is None:
        raise SystemExit("the case is not finished; the program's assessment is frozen first (--finish)")
    if role not in ROLES:
        raise SystemExit(f"role must be one of {', '.join(ROLES)}")
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    problems = []
    for c in doc.get("system_candidates") or []:
        if c.get("verdict") not in VERDICTS:
            problems.append(f"candidate {c.get('id')}: verdict must be one of {', '.join(VERDICTS)}")
    dx_ = doc.get("diagnosis") or {}
    if not (dx_.get("kb_id") or dx_.get("text")):
        problems.append("diagnosis: give kb_id, text, or both")
    if problems:
        raise SystemExit("review not recorded:\n  " + "\n  ".join(problems))
    kb_id = dx_.get("kb_id") or ""
    known = {v["id"] for v in kb.variants()} | set(kb.diseases)
    if kb_id and kb_id not in known:
        issues.note(case_dir, stage=STAGE, code="review_unknown_id", severity="check",
                    text=f"{role} {by} gave diagnosis {kb_id}, which is not in the knowledge base")
    slug = re.sub(r"[^a-z0-9]+", "-", by.lower()).strip("-")
    out = case_dir / "assessment" / f"review_{role}_{slug}.json"
    write_json(out, {**doc, "by": by, "role": role, "at": now(), "kb_version": kb.version})
    set_stage(case_dir, f"review_{role}", "yes", by)
    return out


# ---- the command line -------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--case", required=True)
    ap.add_argument("--new", action="store_true", help="create the case")
    ap.add_argument("--client-ref", help="with --new: the practice's reference for the patient; not a name")
    ap.add_argument("--practitioner", action="append", default=[],
                    help="with --new: '<name>:<role>', role one of " + ", ".join(ROLES))
    ap.add_argument("--model", type=Path, default=None)
    ap.add_argument("--kb", type=Path, default=None, help="knowledge base directory")
    ap.add_argument("--max-tokens", type=int, default=8192)
    ap.add_argument("--finish", action="store_true", help="freeze the program's assessment")
    ap.add_argument("--review-template", action="store_true", help="print a review form to fill in")
    ap.add_argument("--review", type=Path, help="record a filled review form")
    ap.add_argument("--by", help="who is finishing or reviewing")
    ap.add_argument("--role", help="with --review: " + ", ".join(ROLES))
    args = ap.parse_args()

    from workflowsv2.nidana_kb.kb import KB                    # noqa: E402
    kb = KB(args.kb)
    case_dir = cases_root() / args.case
    if args.new:
        if not args.client_ref:
            raise SystemExit("--new needs --client-ref")
        pr = []
        for p in args.practitioner:
            name, _, role = p.rpartition(":")
            if role not in ROLES or not name:
                raise SystemExit(f"--practitioner '{p}': give '<name>:<role>'")
            pr.append({"name": name, "role": role})
        new_case(case_dir, args.client_ref, pr, kb.version)
    elif not case_dir.is_dir():
        raise SystemExit(f"no case {case_dir}; create it with --new")

    if args.finish:
        if not args.by:
            raise SystemExit("--finish needs --by")
        f = finish(case_dir, kb, args.by)
        top = f["differential"][:3]
        print("frozen: " + ("; ".join(f"{r['id']} ({r['score']})" for r in top) or "no candidates"))
        return 0
    if args.review_template:
        print(review_template(case_dir))
        return 0
    if args.review:
        if not (args.by and args.role):
            raise SystemExit("--review needs --by and --role")
        print(f"recorded {review(case_dir, kb, args.review, args.by, args.role)}")
        return 0
    if stage(case_dir, "finished"):
        raise SystemExit("this case is finished; start a new case for a new consultation")

    from workflowsv2.ayur_consult.session import ConsultSession  # noqa: E402
    session = ConsultSession(args.case, args.model, kb=kb, max_tokens=args.max_tokens)
    try:
        print(f"\n{session.name}> {session.open()}\n")
        while True:
            try:
                text = input("practitioner> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            if text.lower() in ("quit", "exit"):
                break
            res = session.turn(text)
            print(f"\n{session.name}> {res['reply']}\n")
    finally:
        session.close()
    print(f"\ncase record at {case_dir / 'case.json'}")
    print(f"finish with:  python3 workflowsv2/ayur_consult/runner.py --case {args.case} --finish --by <name>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
