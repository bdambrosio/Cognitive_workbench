#!/usr/bin/env python3
"""Talk to a finished audit, using the record it left behind.

    python3 workflows/claims_audit/continuation.py --run measure/fixtures/dataroom/results/<dir>
    python3 workflows/claims_audit/continuation.py --run <dir> --model measure/models/grok_4p6.yaml

WHY THIS EXISTS. A report ships the conclusions and deletes the machinery that
produced them. The machinery is still on disk — every run leaves its
deliverables, its reasoning trace, and one file per evidence request — but
nothing could reach it. This makes the run directory answerable.

WHAT IT BINDS, and keeping the three apart is the whole design:

    inspect_external   the TARGET the audit examined, from run_meta's
                       external_repo, so a citation in the report resolves by
                       the path that produced it
    inspect            the RUN DIRECTORY: deliverables, run_meta, working record
    system prompt      workflows/claims_audit/method/CONTINUATION.md, via workflows/claims_audit/continuation.yaml

Binding only the run directory is not enough. The report cites doc4:16-19, and
those lines are in the corpus, which the run directory does not contain.

NOT AN AUDIT. The engagement is over. CONTINUATION.md is mostly prohibitions:
do not speak with the report's authority, do not revise it, do not re-issue its
recommendation, and where the record does not answer, say so.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

HERE = Path(__file__).resolve().parent          # workflows/claims_audit
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


def describe(run_dir: Path, meta: Dict[str, Any], target: Path) -> str:
    """What the session is looking at, printed before the first prompt.

    The point is that the reader can see the engagement's configuration before
    asking it anything — including whether the target still exists, which is
    the one failure that makes every citation unresolvable.
    """
    rec = run_dir / "working_record"
    traces = len(list((rec / "inspect_traces").glob("*.txt"))) \
        if (rec / "inspect_traces").is_dir() else 0
    legs = meta.get("legs") or []
    # HAS THE METHOD MOVED SINCE? CONTINUATION.md §5 says the report's verdict
    # vocabulary still applies, which is only true if METHOD still says what it
    # said when the report was written. The run carries the delivered text, so
    # this is a comparison rather than an assumption.
    delivered = rec / "method_as_delivered.md"
    if delivered.is_file():
        try:
            from chat.workflow import load_workflow            # noqa: E402
            now = load_workflow(HERE / "method" / "METHOD.md")
            same = now == delivered.read_text(encoding="utf-8")
            method = "as delivered, unchanged since" if same else \
                "CHANGED since this run — the current method is not the one " \
                "that produced this report"
        except Exception as e:                                 # noqa: BLE001
            method = f"copied, not compared ({e})"
    else:
        method = "NOT RECORDED — this run predates the method copy"

    lines = [
        f"  run          {run_dir.name}",
        f"  model        {meta.get('resolved_model')} "
        f"(temp {meta.get('resolved_temperature')}, top_p {meta.get('top_p')})",
        f"  legs         {len(legs)}  "
        f"exits {[l.get('exit_reason') for l in legs]}",
        f"  error        {meta.get('error') or 'none'}",
        f"  target       {target}"
        + ("" if target.exists() else "   <-- MISSING: citations will not resolve"),
        f"  record       {'trace' if (rec / 'reasoning_trace.jsonl').exists() else 'NO TRACE'}"
        f", {traces} evidence requests",
        f"  brief        {'recorded' if (rec / 'brief.md').is_file() else 'not recorded'}",
        f"  method       {method}",
    ]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True,
                    help="a results directory containing run_meta.json")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the "
                         "scenario's. Omit to use whatever the local vLLM "
                         "serves — this need not be the model that produced "
                         "the run")
    ap.add_argument("--target", type=Path, default=None,
                    help="override the target tree. Use when the audited "
                         "repository has moved since the run")
    ap.add_argument("--world", default=None,
                    help="world name (default: continuation_<run dir>)")
    args = ap.parse_args()

    run_dir = args.run.resolve()
    meta_path = run_dir / "run_meta.json"
    if not meta_path.is_file():
        raise SystemExit(f"{run_dir}: no run_meta.json — not a results directory")
    meta = json.loads(meta_path.read_text(encoding="utf-8"))

    # THE TARGET IS A PATH, NOT A PIN. run_meta records where the corpus was,
    # not what it contained. For the fixture that is stable and in git; for a
    # repository under audit it moves, and a citation then resolves against a
    # tree the report never saw. Nothing here can detect that — it can only
    # report which tree it bound and whether it still exists.
    target = (args.target or Path(meta.get("external_repo") or "")).resolve()

    world = args.world or f"continuation_{run_dir.name.split('_', 1)[-1]}"
    if (REPO / "scenarios" / world).exists():
        raise SystemExit(
            f"world '{world}' already exists. A world that has answered "
            f"questions about one engagement must not carry them into "
            f"another — pass --world with a fresh name.")

    name, cfg = build_config(run_dir, world, args.model, target)

    from chat.chat_loop import ChatLoop                        # noqa: E402
    from chat.model_params import TOP_P                        # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)
    logger.info("continuation model=%s top_p=%s",
                loop.backend.resolved_model(), TOP_P)

    print("\n=== continuation ===")
    print(describe(run_dir, meta, target))
    print("\n  The engagement is finished. This answers questions about it from")
    print("  its record; it does not revise the report or re-issue its")
    print("  recommendation. Ctrl-D or 'quit' to end.\n")

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
            from workflows.claims_audit.runner import latest_reply  # noqa: E402
            print(f"\n{name}> {latest_reply(loop, 'User')}\n")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
