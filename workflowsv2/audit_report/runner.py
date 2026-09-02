#!/usr/bin/env python3
"""Write the client's document for a rated engagement.

    python3 workflowsv2/audit_report/runner.py --merged <materiality output dir> \\
        --model measure/models/or_glm53flash.yaml

Two parts, in order:

  1. ASSEMBLE (render.py). Mechanical: the merged findings, sorted into the
     three classes REPORT.md §3 defines and ordered by their ratings, with the
     coverage figures computed. This alone is a deliverable.
  2. WRITE. One schema-constrained call, no tools and no legs: the agent is
     shown the assembled document with a marker in each slot and returns the
     six passages REPORT.md §6 names. They are placed and the document is
     written again.

WHY THE AGENT WRITES AROUND THE FINDINGS AND NEVER OVER THEM. Same reason as
the v1 delivery stage: it never emits a finding, so it cannot alter one. What
changed from v1 is the contract — six schema fields instead of three text
blocks — and the input, which is the merged and rated record rather than a
prose report parsed by shape.

Output, beside the inputs: report.md, report_skeleton.md, prose.json,
report_meta.json, and issues for problems the prose check found.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("audit_report.run")
logger.setLevel(logging.INFO)

from chat.workflow import load_workflow                        # noqa: E402
from workflowsv2 import issues                                 # noqa: E402
from workflowsv2.audit_report import render, schemas           # noqa: E402
from workflowsv2.claims_audit.runner import load_engagement    # noqa: E402
from workflowsv2.emit import emit                              # noqa: E402

SCENARIO = HERE / "scenario.yaml"
METHOD_PATH = "workflowsv2/audit_report/method/REPORT.md"
STAGE = "audit_report"


def build_config(out: Path, world: str, model_path: Optional[Path]
                 ) -> Tuple[str, Dict[str, Any]]:
    """Same shape and the same reasons as the materiality runner's: the model
    config REPLACES `llm_config` rather than merging into it."""
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
    cfg["external_repo"] = str(out)
    cfg["inspect_repo"] = str(out)
    return name, cfg


def write_prose(loop, method_text: str, skeleton: str,
                transaction: Optional[str], max_tokens: int) -> Dict[str, Any]:
    user = ("The transaction, as the engagement states it:\n\n"
            + (transaction.strip() if transaction else
               "(The engagement states nothing about the transaction.)")
            + "\n\nThe document, without your passages. Each `[[field]]` "
              "marker is where that passage will be placed:\n\n"
            + skeleton
            + "\n\nWrite the six passages, per REPORT.md \u00a76, and emit them "
              "under the fields of \u00a77.")
    return emit(loop, method_text, user, schemas.prose_schema(), max_tokens)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--merged", required=True, type=Path,
                    help="a materiality output directory (merged.json + "
                         "materiality.json)")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--max-tokens", type=int, default=None)
    ap.add_argument("--no-prose", action="store_true",
                    help="assemble the document from the record only; no call")
    args = ap.parse_args()

    record = render.load(args.merged)
    out = record["dir"]
    if (out / "prose.json").is_file():
        raise SystemExit(f"{out}: already carries prose.json. Move it aside "
                         f"to write another.")
    eng_name = record["merged"].get("engagement")
    eng = load_engagement(eng_name) if eng_name else {}
    transaction = eng.get("transaction")
    fh = logging.FileHandler(out / "report.log", encoding="utf-8")
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(fh)

    skeleton = render.assemble(record, None, transaction, eng_name)
    (out / "report_skeleton.md").write_text(skeleton, encoding="utf-8")
    classes = render.classify(record["merged"])
    logger.info("assembled: shown %d, unsettled %d, not examined %d, hold %d",
                *(len(classes[k]) for k in ("shown", "unsettled",
                                           "not_examined", "holds")))
    if args.no_prose:
        (out / "report.md").write_text(skeleton, encoding="utf-8")
        print(f"out: {out}/report.md (no prose)")
        return 0

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    from chat.chat_loop import ChatLoop                        # noqa: E402
    world = f"report_{ts}_{out.name}"[:60]
    name, cfg = build_config(out, world, args.model)
    loop = ChatLoop(character_name=name, character_config=cfg)
    method_text = load_workflow(REPO / METHOD_PATH)
    max_tokens = args.max_tokens or int((cfg.get("chat") or {})
                                        .get("react_max_tokens", 32768))
    t0 = datetime.datetime.now(datetime.timezone.utc)
    error, call = None, {}
    try:
        call = write_prose(loop, method_text, skeleton, transaction, max_tokens)
        if call["response_format_dropped"]:
            error = ("the route dropped "
                     + ", ".join(call["response_format_dropped"])
                     + " — the prose was not schema-constrained")
        elif call["obj"] is None:
            error = f"prose did not parse: {call['parse_error']}"
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("writing failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)

    prose = call.get("obj") or {}
    check = schemas.check_prose(prose, record["merged"],
                                bool(classes["not_examined"]))
    for p in check["problems"]:
        issues.note(out, stage=STAGE, code="prose_check", text=p,
                    severity="check")
    (out / "prose.json").write_text(
        json.dumps(prose, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    # WHAT THE AGENT DID NOT WRITE, THE RECORD STILL DELIVERS: a slot left
    # empty keeps its marker, so a reader sees where a passage is missing.
    (out / "report.md").write_text(
        render.assemble(record, prose, transaction, eng_name), encoding="utf-8")
    wall = round((datetime.datetime.now(datetime.timezone.utc) - t0).total_seconds(), 1)
    (out / "report_meta.json").write_text(json.dumps({
        "engagement": eng_name, "world": world,
        "model_config": str(args.model) if args.model else None,
        "resolved_model": loop.backend.resolved_model(),
        "classes": {k: len(v) for k, v in classes.items()},
        "call": {k: v for k, v in call.items() if k not in ("raw", "obj")},
        "prose_check": check, "wall_clock_s": wall, "error": error,
        "transient_events": getattr(loop, "transient_events", None),
        "captured_at_utc": ts,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{wall}s, error={error}")
    print(f"classes: " + ", ".join(f"{k} {len(v)}" for k, v in classes.items()))
    print(f"prose check: {'clean' if check['ok'] else str(len(check['problems'])) + ' problem(s)'}")
    print(f"out: {out}/report.md")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
