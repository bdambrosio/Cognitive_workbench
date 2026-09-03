#!/usr/bin/env python3
"""Talk to a finished engagement, using the record it left behind.

    python3 workflowsv2/claims_audit/continuation.py --engagement chattermate-readme \
        [--intake <id>] [--merged <merged output dir>] \
        [--model measure/models/fw_glm53flash.yaml]

WHICH RUN. By default the current run of the engagement's current intake
(workflowsv2/engagement_state.py); --intake and --merged override.

THE WORLD PERSISTS, ONE PER (INTAKE, RUN). A second session over the same
run under the same intake resumes the first: the client's earlier questions
are in its history. A different run or a different intake is a different
world, so an answer about one report never rests on a conversation about
another. --world names a deliberately fresh one.

WHY THIS EXISTS. A report ships the conclusions and deletes the machinery that
produced them. The machinery is still on disk — every run leaves its surface,
findings, review, reasoning trace and one file per evidence request, and the
merged directory holds the ratings and the report — but nothing could reach
it. This makes the engagement directory answerable.

WHAT IT BINDS, and keeping the three apart is the whole design:

    inspect_external   the TARGET the audit examined, from the engagement,
                       so a citation in a finding resolves by the path that
                       produced it
    inspect            the ENGAGEMENT DIRECTORY: runs/, merged/, the brief
    system prompt      workflowsv2/claims_audit/method/CONTINUATION.md, via
                       workflowsv2/claims_audit/continuation.yaml

Re-pointed at the v2 record on 2026-09-02; until then it bound one v1 run
directory and named report.md and gap_map.md, which v2 does not produce.

NOT AN AUDIT. The engagement is over. CONTINUATION.md is mostly prohibitions:
do not speak with the report's authority, do not revise it, do not re-rate
a finding, and where the record does not answer, say so.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

HERE = Path(__file__).resolve().parent          # workflowsv2/claims_audit
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

SCENARIO = HERE / "continuation.yaml"

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("continuation")


def build_config(run_dir: Path, world: str, model_path: Optional[Path],
                 target: Path) -> Tuple[str, Dict[str, Any]]:
    """Assemble the character config, the way run.py:build_config does.

    Same shape and the same reasons: the model REPLACES the llm_config block
    rather than merging into it, and the per-session paths are set here rather
    than by editing the committed scenario, so a session leaves no diff behind.
    """
    from launcher import parse_characters                      # noqa: E402

    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})

    if model_path:
        model = yaml.safe_load(Path(model_path).read_text(encoding="utf-8")) or {}
        model_llm = dict(model.get("llm_config") or {})
        if not model_llm:
            raise SystemExit(f"{model_path}: no llm_config block")
        for char in (scenario.get("characters") or {}).values():
            if isinstance(char, dict) and char.get("mode") == "chat":
                char["llm_config"] = dict(model_llm)
        scen_llm.update(model_llm)

    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world

    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat_chars = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat_chars) != 1:
        raise SystemExit(f"expected 1 chat character, found {len(chat_chars)}")
    name, cfg = chat_chars[0]
    cfg["autonomy_enabled"] = False
    cfg["external_repo"] = str(target)
    cfg["inspect_repo"] = str(run_dir)
    return name, cfg


def _compact(name: str) -> str:
    """The digits of a `<ts>_<label>` name's timestamp: 2026-09-03T01-47-16Z
    → 20260903014716."""
    return "".join(ch for ch in name.split("_", 1)[0] if ch.isdigit())


def describe(merged_dir: Path, merged: Dict[str, Any], target: Path) -> str:
    """What the session is looking at, printed before the first prompt.

    The point is that the reader can see the engagement's configuration before
    asking it anything — including whether the target still exists, which is
    the one failure that makes every citation unresolvable.
    """
    lines = [f"  merged       {merged_dir.name}",
             f"  target       {target}"
             + ("" if target.exists() else "   <-- MISSING: citations will not resolve"),
             f"  report       {'present' if (merged_dir / 'report.md').is_file() else 'ABSENT'}"
             f", worklist {'present' if (merged_dir / 'worklist.md').is_file() else 'absent'}"]
    for r in merged.get("runs") or []:
        run_dir = Path(r.get("dir") or "")
        lines.append(f"  run          {run_dir.name}: {r.get('claim_source')}, "
                     f"{r.get('claims')} claims, {r.get('findings')} findings, "
                     f"{'reviewed' if r.get('reviewed') else 'NOT reviewed'}, "
                     f"model {r.get('resolved_model')}")
        lines.append("               " + _method_state(run_dir))
    return "\n".join(lines)


def _method_state(run_dir: Path) -> str:
    """Has the method moved since this run? CONTINUATION.md §6 says the
    record's vocabulary still applies, which is only true if METHOD still
    says what it said. The run carries the delivered text, so this is a
    comparison rather than an assumption."""
    rec = run_dir / "working_record"
    delivered = rec / "method_as_delivered.md"
    if delivered.is_file():
        try:
            from chat.workflow import load_workflow            # noqa: E402
            now = load_workflow(HERE / "method" / "METHOD.md")
            same = now == delivered.read_text(encoding="utf-8")
            method = "as delivered, unchanged since" if same else \
                "CHANGED since this run — the current method is not the one " \
                "that produced these findings"
        except Exception as e:                                 # noqa: BLE001
            method = f"copied, not compared ({e})"
    else:
        method = "NOT RECORDED — this run predates the method copy"
    traces = len(list((rec / "inspect_traces").glob("*.txt"))) \
        if (rec / "inspect_traces").is_dir() else 0
    return f"method {method}; {traces} evidence requests"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True,
                    help="engagement name under engagements/; `inspect` is "
                         "bound to its directory")
    ap.add_argument("--merged", type=Path, default=None,
                    help="the materiality/report output directory holding "
                         "merged.json, materiality.json and report.md "
                         "(default: the current run of the intake)")
    ap.add_argument("--intake", default=None,
                    help="intake id (default: the engagement's current intake)")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the "
                         "scenario's. Omit to use whatever the local vLLM "
                         "serves — this need not be the model that produced "
                         "the run")
    ap.add_argument("--target", type=Path, default=None,
                    help="override the target tree. Use when the audited "
                         "repository has moved since the run")
    ap.add_argument("--world", default=None,
                    help="world name (default: post_<engagement>_<intake>_"
                         "<run>, resumed if it exists)")
    args = ap.parse_args()

    from workflowsv2 import engagement_state as state            # noqa: E402
    from workflowsv2.claims_audit.runner import load_engagement  # noqa: E402
    eng = load_engagement(args.engagement, intake=args.intake)
    intake_id = eng.get("intake_id")
    if args.merged:
        merged_dir = args.merged.resolve()
    else:
        cur = state.current_run(eng["dir"], intake_id)
        if cur is None:
            raise SystemExit(
                f"engagement '{args.engagement}' has no run of intake "
                f"{intake_id or '(none)'} — pass --merged, or run the "
                f"materiality and report stages first")
        merged_dir = cur.resolve()
    if not (merged_dir / "merged.json").is_file():
        raise SystemExit(f"{merged_dir}: no merged.json — not a merged output "
                         f"directory")
    merged = json.loads((merged_dir / "merged.json").read_text(encoding="utf-8"))
    # THE TARGET IS A PATH, NOT A PIN. The engagement records where the
    # materials are, not what they contained; each run's run_meta carries the
    # commit it saw (`target_rev`). A repository under audit moves, and a
    # citation then resolves against a tree the report never saw. This can
    # only report which tree it bound and whether it still exists.
    target = (args.target or eng["target"]).resolve()
    run_dir = eng["dir"]
    # ONE WORLD PER (INTAKE, RUN), NAMED BY THEIR TIMESTAMPS: short enough
    # never to be cut, and different for any other intake or run.
    world = args.world or "post_{}_{}_{}".format(
        eng["name"], _compact(intake_id) if intake_id else "none",
        _compact(merged_dir.name))
    resumed = (REPO / "scenarios" / world).exists()
    logger.info("intake %s; run %s; world %s: %s", intake_id, merged_dir.name,
                world, "RESUMED — earlier sessions on this run are in its "
                "history" if resumed else "new")

    name, cfg = build_config(run_dir, world, args.model, target)

    # THE CLIENT'S TERMINAL IS NOT A LOG. The harness logs WARNINGs from the
    # attribution and discourse passes to the console, and they landed on the
    # prompt line in the first sessions. The console shows errors only;
    # everything else goes to continuation.log in the world's directory,
    # beside the conversation it belongs to (the intake runner's pattern).
    world_dir = REPO / "scenarios" / world
    world_dir.mkdir(parents=True, exist_ok=True)
    root = logging.getLogger()
    for h in list(root.handlers):
        h.setLevel(logging.ERROR)
    fh = logging.FileHandler(world_dir / "continuation.log", encoding="utf-8")
    fh.setLevel(logging.INFO)
    fh.setFormatter(logging.Formatter("%(asctime)s %(name)s %(levelname)s %(message)s"))
    root.addHandler(fh)
    logger.setLevel(logging.INFO)

    from chat.chat_loop import ChatLoop                        # noqa: E402
    from chat.model_params import TOP_P                        # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)
    logger.info("continuation model=%s top_p=%s",
                loop.backend.resolved_model(), TOP_P)

    print("\n=== continuation ===")
    print(f"  intake       {intake_id or '(none — thresholds from engagement.yaml)'}")
    print(describe(merged_dir, merged, target))
    print(f"  world        {world}: "
          f"{'RESUMED, earlier sessions in history' if resumed else 'new'}")
    print("\n  The engagement is finished. This answers questions about it from")
    print("  its record; it does not revise the report or re-rate a finding.")
    print("  Ctrl-D or 'quit' to end.\n")

    try:
        while True:
            try:
                text = input("you> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            if text.lower() in ("quit", "exit"):
                break
            loop._process_user_turn(source="User", text=text, close=False)
            from workflowsv2.claims_audit.runner import latest_reply  # noqa: E402
            print(f"\n{name}> {latest_reply(loop, 'User')}\n")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
