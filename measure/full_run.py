#!/usr/bin/env python3
"""One complete claims review of a base engagement's materials by one model,
from the sorting of the materials to the report, with no person in the loop.

    python3 measure/full_run.py --base chhoto-full --name cmp-chhoto-mimo26 \\
        --model measure/models/di_mimo_v26flash_medium.yaml \\
        [--duplicates-model <yaml>]

WHAT IT IS FOR. Comparing models, and one model with itself, on the whole
pipeline rather than on one stage. Each run is a new engagement under
workflowsv2/claims_audit/engagements/, so every stage leaves its record where
the product leaves it and nothing written by an earlier run is reused.

THE STEPS are the site's own jobs (src/client_ui/jobs.py), in the order the
practice page runs them. The places where a person would act are taken as
the model proposed them, and each is recorded as such:

  1. A new engagement whose target is a clone of the base engagement's
     target at its checked-out commit, with the base's transaction and
     thresholds. The clone carries only committed files, so nothing a
     previous sorting extracted is present.
  2. The sort job; its proposal is confirmed unchanged.
  3. brief.md, copied from the base engagement. The base's brief describes
     the base's claim sources, so the run stops here when this sorting
     proposed different ones: that brief would then be wrong, and a person
     writes this one.
  4. The enumerate job (enumeration, repeats, reliance, tiers).
  5. The freeze, as the surface page saves it: claims marked `same_as` an
     earlier claim are left out, everything else as enumerated and rated.
  6. The chain job (audit and review per claim source, materiality, report).

THE DUPLICATES MODEL. The enumerate job judges repeats with a fixed model
(jobs.DUPLICATES_MODEL, GLM at low effort). A comparison between models
needs the model under test there too, so this driver sets that value for the
run and records it; --duplicates-model defaults to --model.

WHAT IS WRITTEN beyond the product's own records: <engagement>/full_run.json,
the model configs, the harness commit and whether tracked files differed from
it, and the start and end of each step.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import engagement_state as state              # noqa: E402
from workflowsv2.materials_sorting import runner as sorting     # noqa: E402
from client_ui import jobs, site                                 # noqa: E402

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
logger = logging.getLogger("full_run")

ENGAGEMENTS = REPO / "workflowsv2" / "claims_audit" / "engagements"
STEPS = ("create", "sort", "confirm sorting", "brief", "enumerate", "freeze", "chain")
BY = "full_run.py: the model's proposal, unchanged; no person"


def _now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")


def _harness() -> dict:
    rev = subprocess.run(["git", "-C", str(REPO), "rev-parse", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    changed = subprocess.run(["git", "-C", str(REPO), "diff", "--name-only", "HEAD"],
                             capture_output=True, text=True).stdout.split()
    return {"rev": rev, "tracked_files_changed": changed}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", required=True, help="engagement whose materials and buyer are used")
    ap.add_argument("--name", required=True, help="new engagement name; must not exist")
    ap.add_argument("--model", required=True, type=Path)
    ap.add_argument("--duplicates-model", type=Path, default=None)
    ap.add_argument("--source", default=None,
                    help="test this one claim source only: the sorting is confirmed with it "
                         "alone, its evidence excludes as proposed, and a sentence saying so "
                         "is added to the base's brief")
    ap.add_argument("--from", dest="start", choices=STEPS, default=STEPS[0],
                    help="resume an existing run at this step, after an earlier "
                         "step was finished by hand (the steps before it are "
                         "skipped; full_run.json keeps its record and notes the resume)")
    args = ap.parse_args()

    base = ENGAGEMENTS / args.base
    eng = ENGAGEMENTS / args.name
    model = str(args.model)
    dup_model = str(args.duplicates_model or args.model)
    for m in (model, dup_model):
        if not (REPO / m).is_file():
            raise SystemExit(f"no model config {m}")

    rec_path = eng / "full_run.json"
    if args.start == STEPS[0]:
        record = {"base": args.base, "engagement": args.name, "model": model,
                  "duplicates_model": dup_model, "source": args.source, "harness": _harness(),
                  "started": _now(), "steps": []}
    else:
        if not rec_path.is_file():
            raise SystemExit(f"--from {args.start}: {rec_path} does not exist")
        record = json.loads(rec_path.read_text(encoding="utf-8"))
        if record.get("model") != model:
            raise SystemExit(f"--model {model} differs from the run's {record.get('model')}")
        record.setdefault("resumes", []).append(
            {"from": args.start, "at": _now(), "harness": _harness()})
        record.pop("outcome", None)
        record.pop("ended", None)

    def save() -> None:
        if not eng.is_dir():        # new_engagement creates it, and refuses one that exists
            return
        rec_path.write_text(json.dumps(record, indent=1) + "\n", encoding="utf-8")

    def step(name: str, fn) -> None:
        row = {"step": name, "started": _now()}
        record["steps"].append(row)
        save()
        logger.info("%s: %s", args.name, name)
        try:
            row["result"] = fn()
        except BaseException as e:                             # noqa: BLE001
            row["error"] = f"{type(e).__name__}: {e}"
            row["ended"] = _now()
            record["ended"], record["outcome"] = _now(), f"stopped at {name}"
            save()
            logger.error("%s: %s failed: %s", args.name, name, row["error"])
            raise
        row["ended"] = _now()
        save()

    def create():
        state.new_engagement(eng, clone=str(state.target_dir(base)), by=BY)
        # The buyer is the base engagement's. `transaction` and `thresholds`
        # are not settable through update_engagement, so they are appended.
        yml = state._engagement_yaml(base)
        blocks = ""
        for key in ("transaction", "thresholds"):
            if yml.get(key):
                body = "\n".join("  " + ln for ln in str(yml[key]).rstrip().splitlines())
                blocks += f"\n{key}: |\n{body}\n"
        with open(eng / "engagement.yaml", "a", encoding="utf-8") as f:
            f.write(blocks)
        got = state._engagement_yaml(eng)
        for key in ("transaction", "thresholds"):
            if (got.get(key) or "").strip() != (yml.get(key) or "").strip():
                raise SystemExit(f"{key} did not copy from {args.base}")
        rev = subprocess.run(["git", "-C", str(state.target_dir(eng)), "rev-parse", "HEAD"],
                             capture_output=True, text=True).stdout.strip()
        return {"target_rev": rev}

    def job(kind: str):
        def run():
            j = jobs.start(eng, kind, by=BY, model=model)
            j.wait()
            rec = next(r for r in state.jobs(eng) if r["id"] == j.id)
            if rec.get("exit") != 0:
                raise RuntimeError(f"{kind} job failed: {rec.get('error')}; log {rec.get('log')}")
            return {"job": j.id, "log": rec.get("log")}
        return run

    def confirm():
        sel = sorting.load(eng)
        proposed = list(sel["proposal"]["claim_sources"])
        if args.source and args.source not in proposed:
            raise SystemExit(f"--source {args.source} is not among the proposed claim sources {proposed}")
        sorting.confirm(eng, by=BY, claim_sources=[args.source] if args.source else None)
        return {"claim_sources": proposed, "confirmed": state.claim_sources(eng),
                "evidence_excludes": list(sel["proposal"]["evidence_excludes"])}

    def brief():
        mine, theirs = state.claim_sources(eng), state.claim_sources(base)
        if args.source:
            if mine != [args.source] or args.source not in theirs:
                raise SystemExit(f"--source {args.source}: this engagement has {mine}, the base has {theirs}")
            text = (base / "brief.md").read_text(encoding="utf-8").rstrip()
            (eng / "brief.md").write_text(
                text + f"\n\nThis run tests one claim source only: {args.source}. The other claim "
                       f"sources named above are not tested, and remain not evidence.\n", encoding="utf-8")
            return {"copied_from": args.base, "source": args.source}
        if sorted(mine) != sorted(theirs):
            raise SystemExit(f"this sorting proposed {mine}, the base has {theirs}: "
                             f"the base's brief does not describe these claim sources; "
                             f"write {eng / 'brief.md'} and continue by hand")
        (eng / "brief.md").write_text((base / "brief.md").read_text(encoding="utf-8"),
                                      encoding="utf-8")
        return {"copied_from": args.base}

    def freeze():
        counts = {}
        for src in state.claim_sources(eng):
            claims = site.surface_for(eng, src)["claims"]
            kept = [c for c in claims if not c.get("same_as")]
            site.save_draft(eng, src, kept)
            site.freeze(eng, src, by=BY)
            counts[src] = {"enumerated": len(claims), "frozen": len(kept),
                           "tested": jobs.Chain(eng, model, "").tested(src)}
        return counts

    jobs.DUPLICATES_MODEL = dup_model
    fns = {"create": create, "sort": job("sort"), "confirm sorting": confirm,
           "brief": brief, "enumerate": job("enumerate"), "freeze": freeze,
           "chain": job("chain")}
    for name in STEPS[STEPS.index(args.start):]:
        step(name, fns[name])
    record["merged"] = str(max((eng / state.MERGED).iterdir())) if (eng / state.MERGED).is_dir() else None
    record["ended"], record["outcome"] = _now(), "done"
    save()
    logger.info("%s: done, report in %s", args.name, record["merged"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
