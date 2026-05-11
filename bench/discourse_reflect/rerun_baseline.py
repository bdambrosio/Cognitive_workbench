#!/usr/bin/env python3
"""Re-run discourse `analyze_segment` on each mined pair using the
CURRENT prompt + CURRENT backend, writing the output to a new field
on each pair file. This produces a *current baseline* for ablations
to stack against — distinct from `production_new_discourse_state`,
which captures historical outputs across days of prompt-version drift.

For each pair:
  - prior_state, dialog_window, persona, self_model are fed to
    DiscourseTracker.analyze_segment exactly as the live system does.
  - reasoning_effort='low' matches production (chat_loop.py:3593).
  - Output written to pair["production_new_discourse_state__current"]
    along with provenance metadata.

The judge can then be re-run with `--output-field
production_new_discourse_state__current` to score the current baseline
on the same gold.

Requires the local LLM backend to be running at the scenario's
vllm_url (default http://127.0.0.1:5000).

Usage:
    python bench/discourse_reflect/rerun_baseline.py \\
        --scenario scenarios/jill-chat.yaml \\
        --pairs-dir bench/discourse_reflect/pairs
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional

import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import _ChatBackend  # noqa: E402
from discourse import DiscourseTracker  # noqa: E402

logger = logging.getLogger("bench.discourse_reflect.rerun_baseline")

OUTPUT_FIELD = "production_new_discourse_state__current"
PROVENANCE_FIELD = "current_baseline_provenance"


def _load_jill_config(scenario_path: Path) -> Dict[str, Any]:
    """Pull the Jill character config out of a scenario YAML."""
    with open(scenario_path, "r") as f:
        scenario = yaml.safe_load(f)
    chars = (scenario.get("characters") or {})
    if "Jill" not in chars:
        raise RuntimeError(f"no Jill character in {scenario_path}")
    return chars["Jill"]


def _build_backend(jill_cfg: Dict[str, Any]) -> _ChatBackend:
    """Instantiate _ChatBackend matching the scenario's per-character
    llm_config. Mirrors what ChatLoop does at startup."""
    llm_cfg = jill_cfg.get("llm_config") or {}
    server = llm_cfg.get("server", "local")
    model = llm_cfg.get("model", "")
    base_url = (llm_cfg.get("vllm_url")
                or llm_cfg.get("base_url")
                or "http://127.0.0.1:5000")
    is_reasoning = bool(llm_cfg.get("is_reasoning_model", False))
    api_key = llm_cfg.get("api_key")
    return _ChatBackend(
        server=server,
        model=model,
        base_url=base_url,
        is_reasoning=is_reasoning,
        api_key=api_key,
    )


def _make_llm_generate(backend: _ChatBackend,
                       reasoning_effort: str = "low",
                       max_tokens_floor: int = 16384) -> Callable:
    """Build an `llm_generate`-shaped callable for DiscourseTracker.

    Matches the contract used by chat_loop._llm_generate:
      messages: list[str] (positional roles) or list[dict].
      Returns SimpleNamespace(success, text, error).

    Mirrors chat_loop._make_llm_callable('none', reasoning_effort='low')
    behavior: applies a max_tokens floor so the local model has room
    to produce the full discourse update (matches production)."""
    def _gen(messages, bindings=None, max_tokens=400, temperature=0.7,
             stops=None, is_json=False):
        try:
            chat_msgs: List[Dict[str, str]] = []
            if messages and all(isinstance(m, str) for m in messages):
                if len(messages) == 1:
                    chat_msgs = [{"role": "user", "content": messages[0]}]
                else:
                    chat_msgs.append({"role": "system", "content": messages[0]})
                    next_role = "user"
                    for m in messages[1:]:
                        chat_msgs.append({"role": next_role, "content": m})
                        next_role = "assistant" if next_role == "user" else "user"
            else:
                for m in messages or []:
                    if isinstance(m, dict) and "role" in m and "content" in m:
                        chat_msgs.append({"role": m["role"], "content": m["content"]})
                    else:
                        chat_msgs.append({"role": "user", "content": str(m)})
            text = backend.chat(
                chat_msgs,
                max_tokens=max(max_tokens, max_tokens_floor),
                temperature=temperature,
                stops=stops,
                is_json=is_json,
                reasoning_effort=reasoning_effort,
            )
            if not text:
                return SimpleNamespace(success=False, text="",
                                       error="empty response")
            return SimpleNamespace(success=True, text=text, error=None)
        except Exception as e:
            logger.warning(f"backend.chat raised: {e}")
            return SimpleNamespace(success=False, text="", error=str(e))
    return _gen


def _dialog_to_tracker_format(dialog_window: List[Dict[str, str]]
                              ) -> List[Dict[str, str]]:
    """Convert pair dialog_window (list of {source, text}) to the
    format DiscourseTracker.format_segment expects: list of dicts with
    'source' and 'text' keys, indexed positionally."""
    return [{"source": e.get("source", "?"),
             "text": (e.get("text") or "").strip()}
            for e in dialog_window if (e.get("text") or "").strip()]


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Re-run discourse on mined pairs with current "
                    "prompt + current backend.")
    parser.add_argument("--scenario", type=Path,
                        default=Path("scenarios/jill-chat.yaml"),
                        help="Scenario YAML to source persona / "
                             "self_model / backend config from.")
    parser.add_argument("--pairs-dir", type=Path, required=True)
    parser.add_argument("--reasoning-effort", type=str, default="low",
                        choices=["low", "medium", "high"],
                        help="Override reasoning_effort. Default 'low' "
                             "matches production discourse pass.")
    parser.add_argument("--limit", type=int, default=0,
                        help="Process at most N pairs (0 = all).")
    parser.add_argument("--skip-existing", action="store_true",
                        help="Skip pairs that already have the output "
                             "field populated (e.g., partial prior run).")
    parser.add_argument("--output-field", type=str, default=OUTPUT_FIELD,
                        help=f"Field name to write the result to. "
                             f"Default {OUTPUT_FIELD!r}. Useful to "
                             "distinguish parallel runs (e.g., "
                             "production_new_discourse_state__medium).")
    args = parser.parse_args()

    scenario_path = args.scenario.resolve()
    if not scenario_path.is_file():
        parser.error(f"scenario not found: {scenario_path}")
    pairs_dir = args.pairs_dir.resolve()
    if not pairs_dir.is_dir():
        parser.error(f"pairs dir not found: {pairs_dir}")

    logger.info(f"loading scenario {scenario_path}")
    jill_cfg = _load_jill_config(scenario_path)
    persona = str(jill_cfg.get("character", "")).strip()
    self_model = str(jill_cfg.get("self_model", "")).strip()
    if not persona:
        logger.warning("scenario has no Jill.character — narrator stance "
                       "will be empty (diverges from production)")

    logger.info(f"building backend from scenario llm_config")
    backend = _build_backend(jill_cfg)
    logger.info(f"backend: server={backend.server} model={backend.model!r} "
                f"base_url={backend.base_url}")

    llm_generate = _make_llm_generate(
        backend, reasoning_effort=args.reasoning_effort)
    tracker = DiscourseTracker(llm_generate, "Jill", "User")

    pair_paths = sorted(pairs_dir.glob("pair_*.json"))
    if args.limit > 0:
        pair_paths = pair_paths[:args.limit]
    logger.info(f"re-running discourse on {len(pair_paths)} pairs "
                f"(reasoning_effort={args.reasoning_effort!r})")

    n_done = 0
    n_skipped = 0
    n_failed = 0
    failed_ids: List[str] = []
    for path in pair_paths:
        pair = json.loads(path.read_text(encoding="utf-8"))
        pid = pair.get("id", path.stem)

        if args.skip_existing and (pair.get(args.output_field) or "").strip():
            n_skipped += 1
            logger.info(f"  {pid}: skip (existing output field)")
            continue

        dialog = _dialog_to_tracker_format(pair["dialog_window"])
        if not dialog:
            n_skipped += 1
            logger.warning(f"  {pid}: empty dialog; skipping")
            continue

        logger.info(f"  {pid}: re-running...")
        try:
            new_state = tracker.analyze_segment(
                dialog,
                start=0,
                end=len(dialog) - 1,
                previous_discourse_state=pair["prior_discourse_state"],
                tom="",
                narrator_persona=persona,
                narrator_self_model=self_model,
            )
        except Exception as e:
            logger.warning(f"  {pid}: analyze_segment raised: {e}")
            n_failed += 1
            failed_ids.append(pid)
            continue
        if not new_state or not str(new_state).strip():
            logger.warning(f"  {pid}: empty new_state from tracker")
            n_failed += 1
            failed_ids.append(pid)
            continue

        pair[args.output_field] = str(new_state).strip()
        pair[PROVENANCE_FIELD] = {
            "scenario": str(scenario_path.relative_to(REPO_ROOT)),
            "backend_server": backend.server,
            "backend_model": backend.model,
            "backend_base_url": backend.base_url,
            "reasoning_effort": args.reasoning_effort,
            "ts": datetime.now(timezone.utc).isoformat(),
        }
        path.write_text(
            json.dumps(pair, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        n_done += 1
        logger.info(f"  {pid}: done ({len(pair[args.output_field])} chars)")

    print()
    print(f"=== Re-run summary ===")
    print(f"re-ran:    {n_done}")
    print(f"skipped:   {n_skipped}")
    print(f"failed:    {n_failed}  {failed_ids if failed_ids else ''}")
    print()
    print(f"To score: python bench/discourse_reflect/judge.py \\")
    print(f"    --pairs-dir {pairs_dir} \\")
    print(f"    --scores-dir {pairs_dir.parent / 'scores_current'} \\")
    print(f"    --output-field {args.output_field}")


if __name__ == "__main__":
    main()
