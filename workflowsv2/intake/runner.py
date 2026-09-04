#!/usr/bin/env python3
"""Drive an intake conversation and keep the form.

    python3 workflowsv2/intake/runner.py --engagement <name> [--new] \\
        [--model measure/models/fw_glm53flash.yaml]
    python3 workflowsv2/intake/runner.py --engagement <name> --finish

ONE DIRECTORY PER INTAKE: engagements/<name>/intakes/<id>/ holds the form,
its log and, after --finish, `blocks.yaml`. Without --new the runner continues
the engagement's current intake (workflowsv2/engagement_state.py); --new
starts another, which becomes current by being most recent.

AGENT-LED INSIDE A USER-LED LOOP. The chat loop speaks only in reply, so this
runner supplies what an intake needs beyond that: it sends the opening turn
(practice-sourced, never shown to the client) so the agent's first words are
its greeting and first question; INTAKE.md makes every reply end with the
next question; and after each exchange the runner appends a one-line ledger
of the slots still empty to the client's next turn, the way the audit runner
appends engagement state to `continue`. Silence gets no nudge here; a browser
front end can add one.

THE FORM IS A FILE, WRITTEN BY THIS RUNNER. After each exchange one
schema-constrained call (workflowsv2/emit.py) re-emits the whole form from the
conversation so far, and it is written to the intake's intake.json and
mirrored to a named note in the client's world so recall reaches it. The
agent never writes it; it has no tool that could.

THE WORLD PERSISTS. Unlike every audit run, the client's world is reused
across sessions by design: a second session finds the first in history. The
runner says which it is doing.

--finish IS THE PRACTICE'S ACTION. It writes `transaction:` and `thresholds:`
from the form into the intake's blocks.yaml, which a run pins and reads, and
brief.md at the engagement root if there is none. The agent never declares
the intake done.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("intake.run")
logger.setLevel(logging.INFO)

from chat.workflow import load_workflow                        # noqa: E402
from workflowsv2 import engagement_state as state              # noqa: E402
from workflowsv2.intake import schemas                         # noqa: E402

SCENARIO = HERE / "scenario.yaml"
METHOD_PATH = "workflowsv2/intake/method/INTAKE.md"
ENGAGEMENTS = REPO / "workflowsv2" / "claims_audit" / "engagements"
STAGE = "intake"
SOURCE = "User"
NOTE_NAME = "engagement:intake"

OPENING = ("A client is joining for the intake of a new engagement. Begin: "
           "say what this conversation is for and what they will have at the "
           "end of it, then ask your first question, per INTAKE.md §3.")
RETURNING = ("The client has returned to continue the intake. Greet them "
             "briefly and ask the next question for the emptiest slot, per "
             "INTAKE.md §3.")


def build_config(eng_dir: Path, world: str, model_path: Optional[Path]
                 ) -> Tuple[str, Dict[str, Any]]:
    """Same shape as the other runners': the model REPLACES llm_config."""
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
    cfg["external_repo"] = str(eng_dir)
    cfg["inspect_repo"] = str(eng_dir)
    return name, cfg


def fill_form(emit_fn: Callable[[str, str, Dict[str, Any], int], Dict[str, Any]],
              method_text: str, transcript: List[Tuple[str, str]],
              previous: Dict[str, Any], max_tokens: int) -> Dict[str, Any]:
    """One schema-constrained call: the whole form from the conversation so
    far. `emit_fn(system, user, schema, max_tokens)` returns emit()'s dict;
    injected so the loop is testable without a model. On a call that does
    not parse, the previous form stands and `parse` says so."""
    convo = "\n\n".join(f"{who}: {text}" for who, text in transcript)
    user = ("The conversation so far, client and you:\n\n" + convo
            + "\n\nThe form as it stood before this exchange:\n\n"
            + json.dumps(previous, ensure_ascii=False, indent=1)
            + "\n\nEmit the whole form as the conversation now supports it, "
              "per INTAKE.md §4. A field the client has not filled is an "
              "empty string.")
    out = emit_fn(method_text, user, schemas.intake_schema(), max_tokens)
    form = out.get("obj") if isinstance(out.get("obj"), dict) else None
    return {"form": form or previous, "parse": out.get("parse"),
            "parse_error": out.get("parse_error"), "updated": form is not None}


def write_form(intake_dir: Path, form: Dict[str, Any]) -> Path:
    p = intake_dir / "intake.json"
    p.write_text(json.dumps(form, indent=1, ensure_ascii=False) + "\n",
                 encoding="utf-8")
    return p


def mirror_note(loop, form: Dict[str, Any]) -> None:
    """Create-or-replace the named note so recall reaches the form. The
    chat loop's own state notes use this idiom (`_save_state_note`)."""
    try:
        rm = loop.resource_manager
        text = json.dumps(form, ensure_ascii=False, indent=1)
        ok, note_id, err, _ = rm.create_note(
            loop.character_name, text, "text", "intake", "", NOTE_NAME,
            {"kind": "engagement_intake", "entity": SOURCE})
        if ok and note_id:
            rm.mark_persistent(note_id, loop.character_name)
        elif err:
            logger.warning("intake mirror note: %s", err)
    except Exception as e:                                     # noqa: BLE001
        logger.warning("intake mirror note failed: %s", e)


def finish(eng_dir: Path, intake_dir: Path, form: Dict[str, Any],
           conclusion: bool = False) -> Dict[str, Any]:
    """The practice's action: the intake gets the blocks a run reads
    (`blocks.yaml`: transaction, thresholds), and the engagement a brief if
    there is none. Finishing again rewrites the blocks from the form as it
    now stands; a run that already pinned this intake keeps the text it
    hashed in its meta.json, so the difference is visible."""
    blocks = schemas.engagement_blocks(form)
    written = []
    text = ""
    for key in ("transaction", "thresholds"):
        if not blocks[key].strip():
            continue
        text += f"{key}: |\n" + "\n".join(f"  {l}" for l in blocks[key].splitlines()) + "\n"
        written.append(key)
    if conclusion:
        # The buyer asked for a conclusion (REPORT.md §6 item 3). The
        # practice's flag, not the form's: the deliverable field is the
        # client's words, this is the engagement's decision.
        text += "conclusion: true\n"
        written.append("conclusion")
    if text:
        (intake_dir / state.BLOCKS_FILE).write_text(text, encoding="utf-8")
    brief = eng_dir / "brief.md"
    if not brief.is_file():
        b = form.get("background") or {}
        r = form.get("recommendation") or {}
        brief.write_text(
            "The target: " + str(b.get("target") or "(not stated)").strip()
            + "\n\nClaim sources supplied: " + str(b.get("claim_sources") or "(not stated)").strip()
            + "\n\nWhat the buyer knows or suspects: " + str(b.get("known") or "(not stated)").strip()
            + "\n\nScope agreed: " + str(r.get("scope") or "(not stated)").strip()
            + "\n\nDeliverable: " + str(r.get("deliverable") or "(not stated)").strip()
            + "\n", encoding="utf-8")
        written.append("brief.md")
    stamp = datetime.datetime.now(datetime.timezone.utc).isoformat()
    (intake_dir / "intake_meta.json").write_text(json.dumps(
        {"intake_finished_at": stamp, "written": written,
         "check": schemas.check_intake(form)}, indent=1) + "\n", encoding="utf-8")
    return {"written": written}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True,
                    help="engagement name under claims_audit/engagements/ "
                         "(create it first: engagement_state.py <name> new)")
    ap.add_argument("--world", default=None,
                    help="the client's persistent world (default client_<engagement>)")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--max-tokens", type=int, default=8192,
                    help="ceiling for the form emission")
    ap.add_argument("--finish", action="store_true",
                    help="the practice finishes the current intake: write "
                         "its blocks.yaml (transaction, thresholds) from the "
                         "form, and a brief if none; no conversation")
    ap.add_argument("--conclusion", action="store_true",
                    help="with --finish: the buyer asked for a conclusion; "
                         "the report then carries one, read against the "
                         "thresholds")
    ap.add_argument("--new", action="store_true",
                    help="start another intake for this engagement; it "
                         "becomes the current one")
    args = ap.parse_args()

    eng_dir = ENGAGEMENTS / args.engagement
    if not eng_dir.is_dir():
        raise SystemExit(f"no engagement '{eng_dir.name}' — the engagement comes "
                         f"first: python3 workflowsv2/engagement_state.py "
                         f"{eng_dir.name} new")
    intake_id = state.current_intake(eng_dir)
    if args.finish and (args.new or intake_id is None):
        raise SystemExit(f"{eng_dir}: no intake to finish")
    if args.new:
        intake_id = state.new_intake(eng_dir)

    if args.finish:
        intake_dir = state.intake_dir(eng_dir, intake_id)
        form_path = intake_dir / "intake.json"
        if not form_path.is_file():
            raise SystemExit(f"{intake_dir}: no intake.json to finish from")
        form = json.loads(form_path.read_text(encoding="utf-8"))
        res = finish(eng_dir, intake_dir, form, conclusion=args.conclusion)
        print(f"finished intake {intake_id}: wrote "
              f"{', '.join(res['written']) or 'nothing'}")
        print(f"form: {schemas.ledger(schemas.check_intake(form))}")
        return 0

    from workflowsv2.intake.session import IntakeSession    # noqa: E402
    session = IntakeSession(args.engagement, args.model, new=False,
                            world=args.world, max_tokens=args.max_tokens)
    try:
        print(f"\n{session.name}> {session.open()}\n")
        while True:
            try:
                text = input("client> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            if text.lower() in ("quit", "exit"):
                break
            res = session.turn(text)
            print(f"\n{session.name}> {res['reply']}\n")
            print(f"   {res['ledger']}")
    finally:
        session.close()
    print(f"\n{session.turns} exchange(s); intake {session.intake_id}; "
          f"form at {session.form_path}")
    print(f"finish with:  python3 workflowsv2/intake/runner.py "
          f"--engagement {args.engagement} --finish")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
