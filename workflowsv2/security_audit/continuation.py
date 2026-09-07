#!/usr/bin/env python3
"""Talk to a finished security review, using the record it left behind.

    python3 workflowsv2/security_audit/continuation.py --run <run dir> \\
        [--model measure/models/local_qwen38flashnext.yaml] [--world <name>]

THE THIRD LAYER. The audit produced the record (findings.json, surface.json,
the collection); the report stage produced the document over it. This
session answers questions over both, under CONTINUATION.md, and computes
nothing new without saying so. Its shape is the claims audit's
continuation (workflowsv2/claims_audit/continuation.py); nothing there is
imported, because that runner is bound to the claims engagement state.

WHAT IT BINDS, and keeping the two apart is the design:

    inspect_external   the COLLECTION the review examined, so a citation in
                       a finding resolves by the file name that produced it
    inspect            the RUN DIRECTORY: findings.json, surface.json,
                       report.md, checks.json, working_record/
    system prompt      method/CONTINUATION.md, via continuation.yaml

THE WORLD PERSISTS, ONE PER RUN. A second session over the same run resumes
the first. --world names a deliberately fresh one.
"""
from __future__ import annotations

import argparse
import logging
import re
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

HERE = Path(__file__).resolve().parent          # workflowsv2/security_audit
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.security_audit import record                   # noqa: E402
from workflowsv2.turns import latest_reply                       # noqa: E402

SCENARIO = HERE / "continuation.yaml"
SOURCE = "User"
logger = logging.getLogger("security.continuation")


def build_config(run_dir: Path, world: str, model_path: Optional[Path]
                 ) -> Tuple[str, Dict[str, Any]]:
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
                ch["llm_config"] = dict(llm)               # REPLACE, never merge
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
    cfg["external_repo"] = str(run_dir / "collection")
    cfg["inspect_repo"] = str(run_dir)
    return name, cfg


def describe(run: Dict[str, Any]) -> str:
    """The banner: what this session is over, in the record's own figures."""
    fs = run["findings"].get("findings") or []
    c = run["conclusion"]
    meta = run["meta"]
    lines = [
        f"  run          {run['dir'].name}",
        f"  host         {', '.join(meta.get('hosts') or []) or '(unrecorded)'}",
        f"  collection   {run['collection']}"
        + ("" if run["collection"].is_dir() else "   <-- MISSING: citations will not resolve"),
        f"  surface      {len(run['frozen'])} element(s)",
        f"  findings     {len(fs)}: " + ", ".join(
            f"{d} {sum(1 for f in fs if f.get('disposition') == d)}"
            for d in record.DISPOSITIONS),
        f"  gaps         {len(run['findings'].get('gaps') or [])}",
        f"  conclusion   {c['conclusion']}"
        + (f" (grants withheld: {', '.join(c['withheld_grants'])})" if c["withheld_grants"] else ""),
        f"  document     {'report.md' if (run['dir'] / 'report.md').is_file() else '(none)'}",
    ]
    return "\n".join(lines)


def _compact(s: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "", s)[:24]


class QuerySession:
    """One session over one run. Terminal and any later page both drive it."""

    def __init__(self, run_dir: Path, model: Optional[Path] = None,
                 world: Optional[str] = None) -> None:
        self.run_dir = Path(run_dir).resolve()
        if not (self.run_dir / "findings.json").is_file():
            raise SystemExit(f"{self.run_dir}: no findings.json — nothing to talk to")
        self.run = record.load_run(self.run_dir)
        self.world = world or f"secq_{_compact(self.run_dir.name)}"
        self.resumed = (REPO / "scenarios" / self.world).exists()
        self.name, cfg = build_config(self.run_dir, self.world, model)
        from chat.chat_loop import ChatLoop                    # noqa: E402
        self.loop = ChatLoop(character_name=self.name, character_config=cfg)

    def banner(self) -> str:
        return (describe(self.run) + f"\n  world        {self.world}"
                + (": RESUMED — earlier questions on this run are in its history"
                   if self.resumed else ": new"))

    def turn(self, text: str) -> Dict[str, Any]:
        self.loop._process_user_turn(source=SOURCE, text=text, close=False)
        return {"reply": latest_reply(self.loop, SOURCE)}

    def close(self) -> None:
        try:
            self.loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
        try:
            self.loop._persist_to_disk()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("final persist failed: %s", e)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True, help="a run directory holding findings.json")
    ap.add_argument("--model", type=Path, default=None)
    ap.add_argument("--world", default=None, help="a fresh world; default resumes the run's own")
    ap.add_argument("--ask", action="append", default=[],
                    help="ask this and exit (repeatable); without it, a prompt loop")
    args = ap.parse_args()
    logging.basicConfig(level=logging.WARNING)
    session = QuerySession(args.run, args.model, args.world)
    print("\n" + session.banner() + "\n")
    try:
        if args.ask:
            for q in args.ask:
                print(f"> {q}\n")
                print(session.turn(q)["reply"].strip() + "\n")
            return 0
        while True:
            try:
                text = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if not text:
                continue
            print(session.turn(text)["reply"].strip() + "\n")
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
