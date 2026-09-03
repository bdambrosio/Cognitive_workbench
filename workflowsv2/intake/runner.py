#!/usr/bin/env python3
"""Drive an intake conversation and keep the form.

    python3 workflowsv2/intake/runner.py --engagement <name> --world client_<name> \\
        [--model measure/models/fw_glm53flash.yaml]
    python3 workflowsv2/intake/runner.py --engagement <name> --finish

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
conversation so far, and it is written to engagements/<name>/intake.json and
mirrored to a named note in the client's world so recall reaches it. The
agent never writes it; it has no tool that could.

THE WORLD PERSISTS. Unlike every audit run, the client's world is reused
across sessions by design: a second session finds the first in history. The
runner says which it is doing.

--finish IS THE PRACTICE'S ACTION. It writes `transaction:` and `thresholds:`
into engagement.yaml from the form, and brief.md if there is none. The agent
never declares the intake done.
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
from workflowsv2 import issues                                 # noqa: E402
from workflowsv2.emit import emit                              # noqa: E402
from workflowsv2.intake import schemas                         # noqa: E402
from workflowsv2.turns import latest_reply                     # noqa: E402

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


def write_form(eng_dir: Path, form: Dict[str, Any]) -> Path:
    p = eng_dir / "intake.json"
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


def finish(eng_dir: Path, form: Dict[str, Any]) -> Dict[str, Any]:
    """The practice's action: the engagement file gets the blocks the
    materiality stage reads, and a brief if there is none. A block already
    present is left alone and reported — the practice edits by hand."""
    blocks = schemas.engagement_blocks(form)
    cfg_path = eng_dir / "engagement.yaml"
    text = cfg_path.read_text(encoding="utf-8") if cfg_path.is_file() else ""
    existing = yaml.safe_load(text) or {} if text else {}
    written, skipped = [], []
    add = ""
    for key in ("transaction", "thresholds"):
        if not blocks[key].strip():
            continue
        if key in existing:
            skipped.append(key)
            continue
        add += f"\n{key}: |\n" + "\n".join(f"  {l}" for l in blocks[key].splitlines()) + "\n"
        written.append(key)
    if add:
        cfg_path.write_text(text.rstrip("\n") + "\n" + add, encoding="utf-8")
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
    (eng_dir / "intake_meta.json").write_text(json.dumps(
        {"intake_finished_at": stamp, "written": written, "skipped": skipped,
         "check": schemas.check_intake(form)}, indent=1) + "\n", encoding="utf-8")
    return {"written": written, "skipped": skipped}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True,
                    help="engagement name under claims_audit/engagements/; "
                         "created if absent")
    ap.add_argument("--world", default=None,
                    help="the client's persistent world (default client_<engagement>)")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--max-tokens", type=int, default=8192,
                    help="ceiling for the form emission")
    ap.add_argument("--finish", action="store_true",
                    help="the practice finishes the intake: write the "
                         "engagement file's transaction and thresholds from "
                         "the form, and a brief if none; no conversation")
    args = ap.parse_args()

    eng_dir = ENGAGEMENTS / args.engagement
    eng_dir.mkdir(parents=True, exist_ok=True)
    form_path = eng_dir / "intake.json"
    form = (json.loads(form_path.read_text(encoding="utf-8"))
            if form_path.is_file() else schemas.empty_form())

    if args.finish:
        if not form_path.is_file():
            raise SystemExit(f"{eng_dir}: no intake.json to finish from")
        res = finish(eng_dir, form)
        print(f"finished: wrote {', '.join(res['written']) or 'nothing'}"
              + (f"; left alone (already present): {', '.join(res['skipped'])}"
                 if res["skipped"] else ""))
        print(f"form: {schemas.ledger(schemas.check_intake(form))}")
        return 0

    # THE CLIENT'S TERMINAL IS NOT A LOG. The harness logs WARNINGs from the
    # discourse and attribution passes to the console, and in Bruce's first
    # live intake they landed mid-word in the client's typing. Here the
    # console shows errors only; everything else goes to intake.log beside
    # the form.
    root = logging.getLogger()
    for h in list(root.handlers):
        h.setLevel(logging.ERROR)
    fh = logging.FileHandler(eng_dir / "intake.log", encoding="utf-8")
    fh.setLevel(logging.INFO)
    fh.setFormatter(logging.Formatter("%(asctime)s %(name)s %(levelname)s %(message)s"))
    root.addHandler(fh)
    logger.setLevel(logging.INFO)

    world = args.world or f"client_{args.engagement}"
    returning = (REPO / "scenarios" / world).exists()
    logger.info("world %s: %s", world,
                "RESUMED — the client's world persists across sessions"
                if returning else "new")
    name, cfg = build_config(eng_dir, world, args.model)
    from chat.chat_loop import ChatLoop                        # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)
    method_text = load_workflow(REPO / METHOD_PATH)

    def emit_fn(system, user, schema, max_tokens):
        return emit(loop, system, user, schema, max_tokens)

    # THE INTAKE CONCERN: the backstop for drift, not the driver. The ledger
    # appended to each turn is what keeps the agent on the emptiest slot.
    try:
        loop._add_agent_concern(
            "Complete the intake form for this client.", entity=SOURCE,
            name="intake", instruction="Ask the next question for the "
            "emptiest slot of the intake form, per INTAKE.md §3.")
    except Exception as e:                                     # noqa: BLE001
        logger.warning("intake concern not created: %s", e)

    transcript: List[Tuple[str, str]] = []
    turns = 0
    try:
        loop._process_user_turn(source="Practice",
                                text=RETURNING if returning else OPENING,
                                close=False)
        print(f"\n{name}> {latest_reply(loop, 'Practice')}\n")
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
            check = schemas.check_intake(form)
            loop._process_user_turn(source=SOURCE,
                                    text=text + "\n\n" + schemas.ledger(check),
                                    close=False)
            reply = latest_reply(loop, SOURCE)
            print(f"\n{name}> {reply}\n")
            transcript += [("client", text), (name, reply)]
            turns += 1
            res = fill_form(emit_fn, method_text, transcript, form, args.max_tokens)
            form = res["form"]
            if not res["updated"]:
                logger.warning("form emission did not parse (%s); previous "
                               "form stands", res["parse_error"])
                issues.note(eng_dir, stage=STAGE, code="form_emission",
                            text=f"turn {turns}: form did not parse: "
                                 f"{res['parse_error']}", severity="check")
            write_form(eng_dir, form)
            mirror_note(loop, form)
            print(f"   {schemas.ledger(schemas.check_intake(form))}")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
        try:
            loop._persist_to_disk()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("persist failed: %s", e)
    print(f"\n{turns} exchange(s); form at {form_path}")
    print(f"finish with:  python3 workflowsv2/intake/runner.py "
          f"--engagement {args.engagement} --finish")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
