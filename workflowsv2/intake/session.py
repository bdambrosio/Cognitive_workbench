"""The intake conversation as an object the terminal and the browser both
drive.

One `IntakeSession` is one client at one intake of one engagement: it owns
the ChatLoop, the form, and the per-exchange work — the ledger appended to
the client's turn, the reply, the form re-emitted from the conversation so
far, written to the intake's intake.json and mirrored into the world. The
terminal runner (runner.py) loops `input()` over `turn()`; the browser app
(src/client_ui) queues `turn()` from a websocket. Neither carries any of the
logic, so the two cannot drift.

The practice's actions — `--finish`, `--new`, choosing the current intake —
stay on the command line (runner.py, workflowsv2/engagement_state.py).
"""
from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from workflowsv2 import engagement_state as state
from workflowsv2 import issues
from workflowsv2.emit import emit
from workflowsv2.intake import runner as rn
from workflowsv2.intake import schemas
from workflowsv2.turns import latest_reply

logger = logging.getLogger("intake.session")

#: Where a client's uploads land, relative to the engagement directory. The
#: intake agent has no repository tools (scenario.yaml omits them), so the
#: page tells it the file's text in the turn that announces the upload.
UPLOADS = "uploads"


def route_logging(log_path: Path) -> None:
    """THE CLIENT'S TERMINAL IS NOT A LOG. The harness logs WARNINGs from the
    discourse and attribution passes to the console, and in Bruce's first
    live intake they landed mid-word in the client's typing. The console
    shows errors only; everything else goes to the given file."""
    root = logging.getLogger()
    for h in list(root.handlers):
        h.setLevel(logging.ERROR)
    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setLevel(logging.INFO)
    fh.setFormatter(logging.Formatter("%(asctime)s %(name)s %(levelname)s %(message)s"))
    root.addHandler(fh)


class IntakeSession:
    def __init__(self, engagement: str, model: Optional[Path] = None,
                 new: bool = False, world: Optional[str] = None,
                 max_tokens: int = 8192) -> None:
        self.engagement = engagement
        self.eng_dir = rn.ENGAGEMENTS / engagement
        if not self.eng_dir.is_dir():
            raise SystemExit(f"no engagement '{self.eng_dir.name}' — the engagement comes "
                             f"first: python3 workflowsv2/engagement_state.py "
                             f"{self.eng_dir.name} new")
        intake_id = state.current_intake(self.eng_dir)
        if new or intake_id is None:
            intake_id = state.new_intake(self.eng_dir)
        self.intake_id = intake_id
        self.intake_dir = state.intake_dir(self.eng_dir, intake_id)
        self.form_path = self.intake_dir / "intake.json"
        self.form: Dict[str, Any] = (
            json.loads(self.form_path.read_text(encoding="utf-8"))
            if self.form_path.is_file() else schemas.empty_form())
        self.max_tokens = max_tokens
        route_logging(self.intake_dir / "intake.log")
        logger.setLevel(logging.INFO)

        self.world = world or f"client_{engagement}"
        self.returning = (rn.REPO / "scenarios" / self.world).exists()
        logger.info("intake %s; world %s: %s", intake_id, self.world,
                    "RESUMED — the client's world persists across sessions"
                    if self.returning else "new")
        self.name, cfg = rn.build_config(self.eng_dir, self.world, model)
        from chat.chat_loop import ChatLoop                    # noqa: E402
        self.loop = ChatLoop(character_name=self.name, character_config=cfg)
        self.method_text = rn.load_workflow(rn.REPO / rn.METHOD_PATH)
        # THE INTAKE CONCERN: the backstop for drift, not the driver. The
        # ledger appended to each turn keeps the agent on the emptiest slot.
        try:
            self.loop._add_agent_concern(
                "Complete the intake form for this client.", entity=rn.SOURCE,
                name="intake", instruction="Ask the next question for the "
                "emptiest slot of the intake form, per INTAKE.md §3.")
        except Exception as e:                                 # noqa: BLE001
            logger.warning("intake concern not created: %s", e)
        self.transcript: List[Tuple[str, str]] = []
        self.turns = 0
        self.opened = False

    # ---- the conversation --------------------------------------------------

    def _emit(self, system, user, schema, max_tokens):
        return emit(self.loop, system, user, schema, max_tokens)

    def open(self) -> str:
        """The practice-sourced opening turn: the agent's first words are
        its greeting and first question. Once per session."""
        self.loop._process_user_turn(
            source="Practice",
            text=rn.RETURNING if self.returning else rn.OPENING, close=False)
        self.opened = True
        return latest_reply(self.loop, "Practice")

    def turn(self, text: str) -> Dict[str, Any]:
        check = schemas.check_intake(self.form)
        self.loop._process_user_turn(
            source=rn.SOURCE, text=text + "\n\n" + schemas.ledger(check),
            close=False)
        reply = latest_reply(self.loop, rn.SOURCE)
        self.transcript += [("client", text), (self.name, reply)]
        self.turns += 1
        res = rn.fill_form(self._emit, self.method_text, self.transcript,
                           self.form, self.max_tokens)
        self.form = res["form"]
        if not res["updated"]:
            logger.warning("form emission did not parse (%s); previous form "
                           "stands", res["parse_error"])
            issues.note(self.eng_dir, stage=rn.STAGE, code="form_emission",
                        text=f"turn {self.turns}: form did not parse: "
                             f"{res['parse_error']}", severity="check")
        rn.write_form(self.intake_dir, self.form)
        rn.mirror_note(self.loop, self.form)
        check = schemas.check_intake(self.form)
        return {"reply": reply, "form": self.form, "check": check,
                "ledger": schemas.ledger(check)}

    def history(self, limit: int = 50) -> List[Dict[str, str]]:
        """The exchange so far as the store holds it, for a page that
        reconnects: the client's turns and the agent's, in order. The
        practice's opening turn is not shown; the greeting it produced is."""
        out = []
        for t in self.loop.store.get_recent_turns(rn.SOURCE, limit=limit, scope="all"):
            who = "client" if t.get("direction") == "in" else "agent"
            text = t.get("text") or ""
            if who == "client":
                text = text.split("\n\n[form:", 1)[0]
            out.append({"who": who, "text": text})
        greeting = [t for t in self.loop.store.get_recent_turns("Practice", limit=4, scope="all")
                    if t.get("direction") == "out"]
        if greeting and not out:
            out.insert(0, {"who": "agent", "text": greeting[-1].get("text") or ""})
        return out

    # ---- the document pane -------------------------------------------------

    def uploads_dir(self) -> Path:
        d = self.eng_dir / UPLOADS
        d.mkdir(exist_ok=True)
        return d

    def document(self) -> Dict[str, Any]:
        check = schemas.check_intake(self.form)
        return {"kind": "form", "engagement": self.engagement,
                "intake": self.intake_id, "form": self.form, "check": check,
                "ledger": schemas.ledger(check),
                "slots": {k: list(v) for k, v in schemas.SLOTS.items()},
                "uploads_dir": str(self.uploads_dir())}

    def close(self) -> None:
        try:
            self.loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
        try:
            self.loop._persist_to_disk()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("persist failed: %s", e)
