#!/usr/bin/env python3
"""Write the connecting prose for a finished audit, and assemble the document.

    python3 workflows/audit_postprocess/runner.py --run <audit run directory>
    python3 workflows/audit_postprocess/runner.py --run <dir> --model measure/models/or_glm53flash.yaml

THE AGENT WRITES AROUND THE FINDINGS, NEVER OVER THEM. It emits three blocks —
a cover, a headed passage per finding group, and the coverage statement in
sentences — and `deliver.py` fits those around content it copies verbatim. It
never emits a finding, so it cannot alter one. The guarantee is structural
rather than a rule it is asked to obey, which is the same move as resolving a
citation by file operation instead of asking a model whether a line exists.

WHAT RUNS BEFORE THE AGENT. `deliver.audit()` resolves every citation against
the target, censuses the findings, recovers the claim surface and reads the
review if one ran. All of it is a file operation, and the agent is given the
results rather than the job — it has no target and cannot check a citation.

WITHOUT A MODEL THIS IS STILL A DELIVERABLE. `deliver.py` alone produces the
document; this runner adds the writing. If the agent fails, what was already
written stands, and the run says so.

ONE DELIVERY PER RUN, like the review: the runner refuses if <run>/delivery/
already holds a deliverable written by an agent, so a second pass cannot quietly
overwrite a document someone has read.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

import yaml                                                     # noqa: E402

from workflows import blocks                                    # noqa: E402
from workflows.audit_postprocess import deliver                 # noqa: E402
from workflows.turns import last_exit_reason, latest_reply      # noqa: E402

SCENARIO = HERE / "scenario.yaml"
SOURCE = "User"
CONTINUE = "continue"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("audit_postprocess")
logger.setLevel(logging.INFO)


def build_config(run: Path, world: str,
                 model_path: Optional[Path]) -> Tuple[str, Dict[str, Any]]:
    """Same shape as the audit and review runners', and the same reasons: the
    model config REPLACES the llm_config block rather than merging into it, and
    per-session paths are set here rather than by editing the committed
    scenario.

    `external_repo` is the RUN DIRECTORY, not the target. `inspect_external` is
    omitted in the scenario, so nothing can read it; pointing it at the run
    keeps the config well-formed without handing over the materials.
    """
    from launcher import parse_characters                       # noqa: E402
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
    cfg["inspect_repo"] = str(run)
    cfg["external_repo"] = str(run)
    return name, cfg


# ONE PREDICATE, TWO READERS. The brief promises these to the agent and the
# post-check holds the deliverable to them; deriving each separately is how a
# promise and a document drift apart.
_APPENDIX_BLURB = {
    "Appendix A": "Appendix A, the supported claims",
    "Appendix B": ("Appendix B, every claim as the audit froze it with the "
                   "verdict it received"),
}


def appendices_for(checks: Optional[Dict[str, Any]]) -> List[str]:
    """Which appendices `deliver.assemble` will emit for this run.

    Appendix A exists only where the report carried an inventory to move, and
    Appendix B only where the claim surface was recovered — the two conditions
    assemble() branches on.
    """
    out = []
    if (checks or {}).get("inventory_entries"):
        out.append("Appendix A")
    if (checks or {}).get("claim_surface_recovered"):
        out.append("Appendix B")
    return out


def _id_list(ns: List[int], cap: int = 8) -> str:
    """Claim numbers for a person to read; the count carries past the cap."""
    return (", ".join(map(str, ns)) if len(ns) <= cap
            else ", ".join(map(str, ns[:cap])) + f", … ({len(ns)} in all)")


def brief(run: Path, groups: List[Dict[str, Any]],
          checks: Optional[Dict[str, Any]] = None) -> str:
    """What the agent is told. The group keys live here and nowhere else.

    DELIVERY.md §6 says the keys come from the brief, because a key derived
    from a heading would change when the agent replaced that heading.

    BOTH COUNTS, BECAUSE THEY ARE DIFFERENT NUMBERS. A group holds findings;
    the headings the agent writes for it say "claims"; and one finding may name
    several claims. This stated only the finding count until 2026-08-31, so a
    group of four findings covering five claims was introduced as "Four of the
    seller's claims are not true as written", and a group of one finding
    covering three as "One claim was attempted" — which the same agent's
    coverage passage contradicted sixty lines later with "Three claims could
    not be settled".

    THE APPENDICES ARE LISTED, NOT ASSUMED. Appendix A exists only where the
    report carried an inventory to move, and DELIVERY.md §6 told the cover to
    promise two regardless. Both deliverables on 2026-08-31 promised an
    appendix that was not in them.
    """
    lines = [
        "You are writing the connecting prose for the finished claims audit in "
        "this run directory, per DELIVERY.md.",
        "",
        "The report and the Gap Map are under `inspect`, with the mechanical "
        "checks in `delivery/`. You have no access to the materials the report "
        "cites and do not need it: you are introducing the findings, not "
        "checking them.",
        "",
        "The findings fall into these groups. Use these keys in "
        "`=== SECTION NOTES ===`, one entry each:",
        "",
    ]
    for g in groups:
        vs = ", ".join(sorted({f"[{f['verdict']}]" for f in g["findings"]})) \
            or "no numbered findings"
        nf = len(g["findings"])
        claims = sorted({c for f in g["findings"] for c in (f.get("claims") or [])})
        count = f"{nf} finding{'' if nf == 1 else 's'}"
        if claims:
            count += (f" covering {len(claims)} "
                      f"claim{'' if len(claims) == 1 else 's'} "
                      f"({_id_list(claims)})")
        lines.append(f"  {g['key']} — currently headed \"{g['heading']}\" — "
                     f"{count}, {vs}")
        for f in g["findings"]:
            cl = (f" (claim{'' if len(f['claims']) == 1 else 's'} "
                  f"{_id_list(f['claims'])})") if f.get("claims") else ""
            lines.append(f"      Finding {f['n']}{cl}: {f['title']} "
                         f"[{f['verdict']}]")

    apx = [_APPENDIX_BLURB[k] for k in appendices_for(checks)]
    lines += [
        "",
        ("The assembled document will carry " + " and ".join(apx) + "."
         if apx else "The assembled document will carry no appendices."),
        "",
        "Emit the three blocks of §6. The client's process assembles the "
        "document; you write nothing else. Work in as many legs as you need — "
        "a leg is one turn, one run of the action loop. End a leg with `yield` "
        "and I will say `continue`.",
    ]
    return "\n".join(lines)


def parse_section_notes(text: str) -> Dict[str, Tuple[str, str]]:
    """`<key>: <heading>` then the passage, per DELIVERY.md §6.

    Tolerant of a blank line between entries and of a bolded key, because those
    are the two shapes a model reaches for unprompted. An entry whose key was
    not issued in the brief is dropped by the caller, not silently accepted.
    """
    out: Dict[str, Tuple[str, str]] = {}
    key = None
    head, body = "", []
    for line in (text or "").splitlines():
        m = re.match(r"^\s*\**\s*(group\d+)\**\s*:\s*(.+?)\s*$", line, re.I)
        if m:
            if key:
                out[key] = (head, " ".join(" ".join(body).split()))
            key, head, body = m.group(1).lower(), m.group(2).strip("* "), []
        elif key and line.strip():
            body.append(line.strip())
    if key:
        out[key] = (head, " ".join(" ".join(body).split()))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run", type=Path, required=True,
                    help="a finished claims_audit run directory")
    ap.add_argument("--model", type=Path, default=None,
                    help="a measure/models/*.yaml whose llm_config replaces the "
                         "scenario's")
    ap.add_argument("--world", default=None)
    ap.add_argument("--max-turns", type=int, default=8)
    args = ap.parse_args()

    run = args.run.resolve()
    if not (run / "report.md").is_file():
        raise SystemExit(f"{run}: no report.md — not a finished audit run")
    out = run / "delivery"
    if (out / "written.json").is_file():
        raise SystemExit(f"{out}: already carries an agent-written delivery. "
                         "Move it aside to write another.")
    # A FRESH WORLD PER INVOCATION. Derived from the run id alone until
    # 2026-08-30, which meant every delivery of the same audit reused one
    # world: ChatLoop restored the previous session, the agent saw its own
    # earlier brief and answer in history, and reproduced that answer word for
    # word. Two runs four minutes apart produced byte-identical prose while the
    # method document had changed between them — the edit never had a chance.
    # The audit and review runners avoid this by taking a unique --world from
    # the operator; this one defaults to a unique name instead of trusting that.
    stamp = datetime.datetime.now(
        datetime.timezone.utc).strftime("%H%M%S")
    world = args.world or f"delivery_{run.name[:20]}_{stamp}"

    # EVERY MECHANICAL FACT FIRST. The agent reads these; it does not produce
    # them, and it cannot check a citation because it has no materials.
    checks = deliver.audit(run)
    groups = deliver.finding_groups(run)
    out.mkdir(parents=True, exist_ok=True)
    (out / "checks.json").write_text(json.dumps(checks, indent=2))
    (out / "editor_notes.md").write_text(deliver.editor_notes(run, checks))
    logger.info("citations %d, %d unresolved; %d finding groups",
                checks["citations_total"], len(checks["citations_unresolved"]),
                len(groups))

    name, cfg = build_config(run, world, args.model)
    from chat.chat_loop import ChatLoop                          # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)

    t0 = time.time()
    delivered = {n: False for n in blocks.DELIVERY_BLOCKS}
    transcript: List[str] = []
    error = None
    legs: List[Dict[str, Any]] = []
    text = brief(run, groups, checks)
    try:
        for i in range(args.max_turns):
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = latest_reply(loop, SOURCE)
            exit_reason = last_exit_reason(world, name)
            legs.append({"leg": i + 1, "exit_reason": exit_reason,
                         "reply_chars": len(reply),
                         "blocks": [n for n in blocks.DELIVERY_BLOCKS
                                    if blocks.opened(reply, n)]})
            logger.info("leg %d: exit=%s chars=%d blocks=%s", i + 1,
                        exit_reason, len(reply), legs[-1]["blocks"])
            if exit_reason in ("llm_error", "crashed"):
                error = f"turn {i + 1} ended {exit_reason}"
                break
            transcript.append(reply)
            whole = "\n\n".join(t for t in transcript if t)
            for n in blocks.DELIVERY_BLOCKS:
                delivered[n] = delivered[n] or blocks.opened(reply, n)
            missing = blocks.missing(delivered, blocks.DELIVERY_BLOCKS)
            if not missing:
                break
            # A YIELD IS A NOT-DONE SIGNAL. Same rule as the other two runners:
            # believing "not finished" costs one leg, believing "finished" can
            # end a run with nothing written.
            text = (CONTINUE if exit_reason == "yield" else
                    CONTINUE + f"\n\n{blocks.rejection(missing[0], 'DELIVERY.md §6')}")
        else:
            if blocks.missing(delivered, blocks.DELIVERY_BLOCKS):
                error = ("no_prose: "
                         + ", ".join(blocks.missing(delivered,
                                                    blocks.DELIVERY_BLOCKS))
                         + f" not delivered in {args.max_turns} legs")
    except Exception as e:                                       # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("delivery run failed")

    whole = "\n\n".join(t for t in transcript if t)
    notes = parse_section_notes(
        blocks.content(whole, "SECTION NOTES", blocks.DELIVERY_BLOCKS) or "")
    issued = {g["key"] for g in groups}
    unknown = sorted(set(notes) - issued)
    if unknown:
        logger.warning("section notes for keys never issued, dropped: %s",
                       ", ".join(unknown))
    written = {
        "cover": blocks.content(whole, "COVER", blocks.DELIVERY_BLOCKS) or "",
        "coverage": blocks.content(whole, "COVERAGE",
                                   blocks.DELIVERY_BLOCKS) or "",
        "sections": {k: v for k, v in notes.items() if k in issued},
    }
    (out / "written.json").write_text(json.dumps(written, indent=2))
    # WHAT THE AGENT DID NOT WRITE, THE SCRIPT STILL DELIVERS. A failed or
    # partial prose run degrades the document; it does not lose it.
    (out / "deliverable.md").write_text(deliver.assemble(run, written))

    # AFTER THE AGENT, because it is the agent's prose being checked. The
    # editor notes are written before the run, so they are rewritten here with
    # the result rather than left describing a document that did not exist yet.
    promised = deliver.promised_appendices(written,
                                           set(appendices_for(checks)))
    if promised:
        checks["appendices_promised_not_emitted"] = promised
        logger.warning("the deliverable promises %s and does not carry it",
                       ", ".join(promised))
        (out / "checks.json").write_text(json.dumps(checks, indent=2))
        (out / "editor_notes.md").write_text(deliver.editor_notes(run, checks))
    (out / "run_meta.json").write_text(json.dumps({
        "captured_at_utc": datetime.datetime.now(
            datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ"),
        "delivered_for": run.name,
        "model_config": str(args.model) if args.model else None,
        "resolved_model": getattr(getattr(loop, "backend", None),
                                  "model", None),
        "wall_clock_s": round(time.time() - t0, 1),
        "legs": legs,
        "blocks_delivered": delivered,
        "groups_issued": sorted(issued),
        "groups_written": sorted(written["sections"]),
        "citations_unresolved": [c.get("cited")
                                 for c in checks["citations_unresolved"]],
        "error": error,
    }, indent=2))

    print(f"{run.name}")
    print(f"  blocks     {', '.join(n for n, ok in delivered.items() if ok) or 'none'}")
    print(f"  groups     {len(written['sections'])} of {len(issued)} introduced")
    print(f"  citations  {len(checks['citations_unresolved'])} unresolved")
    if error:
        print(f"  error      {error}")
    print(f"  -> {out}/deliverable.md")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
