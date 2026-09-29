"""The consultation as an object the terminal and a browser both drive, as
intake/session.py is for the intake.

THE RECORD IS UPDATED BEFORE THE AGENT REPLIES. Each practitioner turn is
first folded into the case record (one schema-constrained call), its new
findings matched to the knowledge base (one more call, only when there are
new findings), and the differential recomputed in code. The ledger appended
to the practitioner's turn therefore already reflects what they just said,
and the agent's reply can report it. The intake appends the previous
state instead; a co-pilot that lags one turn behind the findings would be
reporting a differential the practitioner has already moved past.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

from workflowsv2 import issues
from workflowsv2.ayur_consult import record
from workflowsv2.ayur_consult import runner as rn
from workflowsv2.ayur_consult import schemas
from workflowsv2.emit import emit
from workflowsv2.turns import latest_reply

logger = logging.getLogger("ayur_consult.session")


class ConsultSession:
    def __init__(self, case_id: str, model: Optional[Path] = None, kb=None,
                 world: Optional[str] = None, max_tokens: int = 8192) -> None:
        from workflowsv2.nidana_kb.kb import KB
        self.case_id = case_id
        self.case_dir = rn.cases_root() / case_id
        if not self.case_dir.is_dir():
            raise SystemExit(f"no case {self.case_dir}")
        self.kb = kb or KB()
        self.form: Dict[str, Any] = rn.read_json(self.case_dir / "case.json", schemas.empty_form())
        self.transcript = rn.read_transcript(self.case_dir)
        self.max_tokens = max_tokens
        self.world = world or f"consult_{case_id}"
        self.returning = bool(self.transcript)
        self.name, cfg = rn.build_config(self.case_dir, self.world, model)
        from chat.chat_loop import ChatLoop                    # noqa: E402
        self.loop = ChatLoop(character_name=self.name, character_config=cfg)
        record.register(self.loop, self.kb)
        self.method_text = rn.load_workflow(rn.REPO / rn.METHOD_PATH)
        try:
            self.loop._add_agent_concern(
                "Help the practitioner reach a diagnosis for this case.", entity=rn.SOURCE,
                name="consult", instruction="Report the ledger's differential and suggest "
                "its next questions, per CONSULT.md §3.")
        except Exception as e:                                 # noqa: BLE001
            logger.warning("consult concern not created: %s", e)
        self.state = rn.compute(self.kb, self.form)
        self.turns = 0

    def _emit(self, system, user, schema, max_tokens):
        return emit(self.loop, system, user, schema, max_tokens)

    def _ledger(self) -> str:
        return schemas.ledger(schemas.check_case(self.form), self.state["differential"],
                              self.state["next_questions"], self.kb)

    def open(self) -> str:
        text = rn.RETURNING + "\n\n" + self._ledger() if self.returning else rn.OPENING
        self.loop._process_user_turn(source="Practice", text=text, close=False)
        return latest_reply(self.loop, "Practice")

    def turn(self, text: str) -> Dict[str, Any]:
        self.turns += 1
        self.transcript.append(("practitioner", text))
        rn.append_transcript(self.case_dir, "practitioner", text)
        res = rn.fill_form(self._emit, self.method_text, self.transcript, self.form, self.max_tokens)
        if not res["updated"]:
            issues.note(self.case_dir, stage=rn.STAGE, code="form_emission", severity="check",
                        text=f"turn {self.turns}: the case record did not parse: {res['parse_error']}")
        self.form = res["form"]
        rn.map_findings(self._emit, self.kb, self.form)
        self.state = rn.compute(self.kb, self.form)
        rn.write_json(self.case_dir / "case.json", self.form)
        rn.write_json(self.case_dir / "differential.json", self.state)
        ledger = self._ledger()
        self.loop._process_user_turn(source=rn.SOURCE, text=text + "\n\n" + ledger, close=False)
        reply = latest_reply(self.loop, rn.SOURCE)
        self.transcript.append((self.name, reply))
        rn.append_transcript(self.case_dir, self.name, reply)
        bad = rn.check_citations(self.kb, self.case_dir, reply, self.turns)
        return {"reply": reply, "form": self.form, "ledger": ledger,
                "differential": self.state["differential"],
                "next_questions": self.state["next_questions"], "bad_citations": bad}

    def history(self, limit: int = 200) -> List[Dict[str, str]]:
        return [{"who": "practitioner" if who == "practitioner" else "agent", "text": t}
                for who, t in self.transcript[-limit:]]

    def document(self) -> Dict[str, Any]:
        return {"kind": "consult", "case": self.case_id, "form": self.form,
                "check": schemas.check_case(self.form), **self.state,
                "finished": rn.stage(self.case_dir, "finished") is not None}

    def close(self) -> None:
        try:
            self.loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
        try:
            self.loop._persist_to_disk()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("persist failed: %s", e)
