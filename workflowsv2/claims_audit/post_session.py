"""The post-delivery conversation as an object the terminal and the browser
both drive.

One `PostSession` is one client over one run of one intake: it resolves the
current run (workflowsv2/engagement_state.py), binds the world for that
intake and run — resumed when it exists — and answers under CONTINUATION.md.
continuation.py loops `input()` over `turn()`; src/client_ui queues `turn()`
from a websocket and shows the run's report.md in the document pane.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

from workflowsv2 import engagement_state as state
from workflowsv2.claims_audit import continuation as ct
from workflowsv2.claims_audit.runner import load_engagement
from workflowsv2.turns import latest_reply

logger = logging.getLogger("continuation.session")
SOURCE = "User"


def _clip(text: Any, n: int) -> str:
    t = " ".join(str(text or "").split())
    return t if len(t) <= n else t[:n].rstrip() + "…"


def deliverable_digest(merged: Dict[str, Any], merged_dir: Path,
                       gap_chars: int = 400, basis_chars: int = 300) -> str:
    """The delivered findings and ratings as prompt text, one block per
    claim: quote, verdict, gap, the check's outcome, the rating and its
    basis. Sized for the prompt (a 58-claim run is under 10k tokens); the
    full text of every field is in the files under `inspect`, which the
    block names."""
    ratings: Dict[Any, Dict[str, Any]] = {}
    mp = merged_dir / "materiality.json"
    if mp.is_file():
        m = json.loads(mp.read_text(encoding="utf-8"))
        for key in ("ratings", "exposures"):
            for r in m.get(key) or []:
                ratings[(r.get("claim_source"), r.get("claim_id"))] = r
    lines = [f"## The deliverable, loaded at session start (merged/{merged_dir.name})",
             "The findings as delivered and the ratings as given, one block per claim. "
             "The complete record of any claim, evidence and rating included: the "
             "`claim` action. The files themselves: merged.json, materiality.json and "
             "report.md under `inspect`."]
    for f in merged.get("findings") or []:
        adj = f.get("adjudication") or {}
        key = (f.get("claim_source"), f.get("claim_id"))
        r = ratings.get(key) or {}
        rating = r.get("materiality") or r.get("exposure")
        lines.append("")
        lines.append(f"### {f.get('claim_source')} #{f.get('claim_id')} — {adj.get('verdict')}"
                     + (f"; {'exposure' if 'exposure' in r else 'materiality'}: {rating}"
                        + (" (borderline)" if r.get("borderline") else "") if rating else "")
                     + (f"; check: {(f.get('review') or {}).get('outcome')}"
                        if (f.get("review") or {}).get("outcome") else ""))
        lines.append(f"quote: {_clip(f.get('quote'), 240)}")
        if adj.get("gap"):
            lines.append(f"gap: {_clip(adj.get('gap'), gap_chars)}")
        if adj.get("unresolved_because"):
            lines.append(f"unresolved because: {adj.get('unresolved_because')}")
        if r.get("basis"):
            lines.append(f"rating basis: {_clip(r.get('basis'), basis_chars)}")
    qs = merged.get("questions") or []
    if qs:
        lines += ["", "### Questions put to the seller"]
        for q in qs:
            lines.append(f"- ({q.get('claim_source')} #{q.get('claim_id')}) {_clip(q.get('question'), 300)}")
    return "\n".join(lines)


class PostSession:
    def __init__(self, engagement: str, model: Optional[Path] = None,
                 intake: Optional[str] = None, merged: Optional[Path] = None,
                 world: Optional[str] = None, target: Optional[Path] = None,
                 log_to_world: bool = True,
                 scenario: Optional[Path] = None) -> None:
        self.engagement = engagement
        eng = load_engagement(engagement, intake=intake)
        self.intake_id = eng.get("intake_id")
        if merged:
            self.merged_dir = Path(merged).resolve()
        else:
            cur = state.current_run(eng["dir"], self.intake_id)
            if cur is None:
                raise SystemExit(
                    f"engagement '{engagement}' has no run of intake "
                    f"{self.intake_id or '(none)'} — pass --merged, or run "
                    f"the materiality and report stages first")
            self.merged_dir = cur.resolve()
        if not (self.merged_dir / "merged.json").is_file():
            raise SystemExit(f"{self.merged_dir}: no merged.json — not a "
                             f"merged output directory")
        self.merged = json.loads((self.merged_dir / "merged.json").read_text(encoding="utf-8"))
        # THE TARGET IS A PATH, NOT A PIN: see continuation.py.
        self.target = (target or eng["target"]).resolve()
        self.eng_dir = eng["dir"]
        # ONE WORLD PER (INTAKE, RUN), named by their timestamps.
        self.world = world or "post_{}_{}_{}".format(
            engagement, ct._compact(self.intake_id) if self.intake_id else "none",
            ct._compact(self.merged_dir.name))
        world_dir = ct.REPO / "scenarios" / self.world
        self.resumed = world_dir.exists()
        world_dir.mkdir(parents=True, exist_ok=True)
        # The terminal and the client page log beside the world; a server
        # running many sessions keeps one log of its own instead.
        if log_to_world:
            from workflowsv2.intake.session import route_logging  # noqa: E402
            route_logging(world_dir / "continuation.log")
        logger.setLevel(logging.INFO)
        logger.info("intake %s; run %s; world %s: %s", self.intake_id,
                    self.merged_dir.name, self.world,
                    "RESUMED — earlier sessions on this run are in its "
                    "history" if self.resumed else "new")
        self.name, cfg = ct.build_config(self.eng_dir, self.world, model, self.target,
                                         scenario_path=scenario)
        # THE DELIVERABLE IS IN THE PROMPT (2026-09-14). It was reviewed,
        # delivered and does not change, so it is loaded once here rather
        # than rediscovered through the inspect subagent on every question:
        # the one recorded turn before this ran three subagent passes over
        # report.md, merged.json and materiality.json to answer one question,
        # about five minutes. CONTINUATION.md §1 and §3 say what is here and
        # what stays under `inspect` (the record and the materials).
        cfg["capabilities"] = (str(cfg.get("capabilities") or "").rstrip() + "\n\n"
                               + deliverable_digest(self.merged, self.merged_dir))
        from chat.chat_loop import ChatLoop                    # noqa: E402
        from chat.model_params import TOP_P                    # noqa: E402
        self.loop = ChatLoop(character_name=self.name, character_config=cfg)
        # THE CLAIM RECORD IS AN ACTION (2026-09-16). A question about a named
        # claim was spending most of its turn in inspect-subagent steps that
        # located and re-read the delivered files; record.py holds the same
        # data indexed by claim and resolves each citation against the
        # materials at call time. See record.py for the measurements.
        from workflowsv2.claims_audit import record as rec              # noqa: E402
        self.record = rec.ClaimRecord(self.merged, self.merged_dir, self.target)
        rec.register(self.loop, self.record)
        logger.info("continuation model=%s top_p=%s",
                    self.loop.backend.resolved_model(), TOP_P)

    def describe(self) -> str:
        """The banner: what the session is looking at."""
        return "\n".join([
            f"  intake       {self.intake_id or '(none — thresholds from engagement.yaml)'}",
            ct.describe(self.merged_dir, self.merged, self.target),
            f"  world        {self.world}: "
            f"{'RESUMED, earlier sessions in history' if self.resumed else 'new'}"])

    def turn(self, text: str) -> Dict[str, Any]:
        self.loop._process_user_turn(source=SOURCE, text=text, close=False)
        return {"reply": latest_reply(self.loop, SOURCE)}

    def history(self, limit: int = 50) -> List[Dict[str, str]]:
        return [{"who": "client" if t.get("direction") == "in" else "agent",
                 "text": t.get("text") or ""}
                for t in self.loop.store.get_recent_turns(SOURCE, limit=limit, scope="all")]

    def document(self) -> Dict[str, Any]:
        """The delivered report, rendered for the pane; absent when the run
        has no report.md yet."""
        from workflowsv2.audit_report import printable        # noqa: E402
        p = self.merged_dir / "report.md"
        html = printable.to_body(p.read_text(encoding="utf-8")) if p.is_file() else ""
        return {"kind": "report", "engagement": self.engagement,
                "intake": self.intake_id, "run": self.merged_dir.name,
                "html": html, "banner": self.describe(),
                "findings": self.findings()}

    def findings(self) -> Dict[str, Any]:
        """Every finding's claim, verdict and evidence, keyed `<claim
        source>#<claim id>`, for the evidence pane: what the page shows
        beside a finding a reply names or the reader clicks."""
        out: Dict[str, Any] = {}
        for f in self.merged.get("findings") or []:
            adj = f.get("adjudication") or {}
            ev = []
            for e in f.get("evidence") or []:
                if e.get("form") == "citation":
                    ev.append({"form": "citation", "document": e.get("document"),
                               "lines": e.get("lines"), "quote": e.get("quote"),
                               "shows": e.get("shows")})
                elif e.get("form") == "search":
                    ev.append({"form": "search", "kind": e.get("kind"),
                               "performed": e.get("performed"),
                               "result": e.get("result")})
                else:
                    ev.append({"form": "derived",
                               "derivation": e.get("derivation"),
                               "consequence": e.get("consequence")})
            out[f"{f.get('claim_source')}#{f.get('claim_id')}"] = {
                "claim_source": f.get("claim_source"), "claim_id": f.get("claim_id"),
                "quote": f.get("quote"), "lines": f.get("lines"),
                "statement": f.get("statement"), "verdict": adj.get("verdict"),
                "gap": adj.get("gap"),
                "unresolved_because": adj.get("unresolved_because"),
                "review": (f.get("review") or {}).get("outcome"),
                "evidence": ev}
        return out

    def close(self) -> None:
        try:
            self.loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
        try:
            self.loop._persist_to_disk()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("persist failed: %s", e)
