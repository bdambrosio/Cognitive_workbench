#!/usr/bin/env python3
"""Drive one dataroom audit run, unattended.

    python3 workflowsv2/claims_audit/runner.py --world dataroom_1
    python3 workflowsv2/claims_audit/runner.py --world dataroom_2 \
            --model measure/models/nemotron_fp8.yaml --max-turns 20

Loads `workflowsv2/claims_audit/scenario.yaml` and points `inspect_external` at
the target. A run has three parts, in this order:

  1. ENUMERATE. One schema-constrained call per section of the claim source,
     no tools and no legs, each given the claims enumerated so far, producing
     `claims.json`. The surface freezes when the last section is in.
  2. GATHER. The brief and the frozen surface open the engagement; the agent
     works the corpus with tools for as long as it keeps yielding, naming on
     each evidence request the claims it serves.
  3. ADJUDICATE. Schema-constrained calls over batches of claims that share
     evidence, each given only the evidence requests filed under its claims,
     producing `findings.json`.

WHY THE RUNNER MAKES THE TWO EMISSIONS RATHER THAN THE AGENT. The deliverables
are schema-enforced, and a ReAct action's payload field is not — it is
`additionalProperties` on an action schema, and stuffing a document into it is
a documented death spiral. Tools gather; the emissions answer under their own
schema.

WHY THE SURFACE FREEZES BEFORE ANY EVIDENCE IS READ (METHOD §5). Enumerating
while adjudicating means choosing what counts as a claim with the verdicts
already in view, which is choosing the sample after seeing the result. Freezing
also buys the check that v1 could not make: a claim adjudicated by no finding
is visible, because the surface says it existed.

WHY A RUNNER AND NOT --cli. The interactive path needs a human to paste the
brief and press return between legs. A benchmark needs the same input every
time, which a human cannot supply; three runs typed by hand are three
different runs.

WHY autonomy IS OFF. Continuations are driven here rather than by the tick
sensor, so a run is deterministic in its leg count and a stall is visible as
a stall rather than as a missing tick. Concern CREATION happens in-turn and
needs no autonomy; only FIRING is gated.

WHY THE MODEL REPLACES llm_config RATHER THAN MERGING. A merge lets a stale
field from the scenario — an api_key, a reasoning_effort — survive into an
model that never declared one, which is the silent second variable this exists
to avoid. Shape and rationale recovered from the retired bench/common.py.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import re
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent          # workflowsv2/claims_audit
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

SCENARIO = HERE / "scenario.yaml"
ENGAGEMENTS = HERE / "engagements"
#: The method the agent works to, and the system prompt of both emission
#: calls. Read from one place so a run cannot be driven by one text and
#: adjudicated under another.
METHOD_PATH = "workflowsv2/claims_audit/method/METHOD.md"

SOURCE = "User"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("dataroom.run")
logger.setLevel(logging.INFO)

# DELIVERY IS BY SCHEMA (METHOD §13), not by marker and not by turn boundary.
# `schemas.py` holds the contract and the reason; this file only drives it.
from chat.workflow import load_workflow                          # noqa: E402
from workflowsv2 import issues                                    # noqa: E402
from workflowsv2.claims_audit import schemas                     # noqa: E402
from workflowsv2.emit import emit                                 # noqa: E402

# The brief lives with its engagement, in engagements/<name>/brief.md — it is
# what a client says, and it differs per engagement. Its history travels with
# it as HTML comments in that file.
CONTINUE = "continue"

# THE LEGS PRODUCE NO DELIVERABLE. Until 2026-08-27 the runner drove the
# report and the Gap Map as one turn each and knew which was which because it
# had asked — which is exactly how a claim enumeration became a report. In v2
# the legs only gather; both deliverables are schema-constrained calls the
# runner makes itself (`emit_surface`, `emit_findings`), so "continue" is all
# a leg is ever asked.


def load_engagement(name: str, intake: Optional[str] = None) -> Dict[str, Any]:
    """The engagement's configuration, resolved for one intake.

    THE ENGAGEMENT SUPPLIES WHAT THE METHOD MUST NOT INVENT: which tree is
    under audit, and — through the brief — which of its documents carry the
    seller's claims (METHOD §12 step 1). Both differ per engagement and neither
    is knowable to the method's author.

    THE INTAKE SUPPLIES THE BUYER. `transaction` and `thresholds` come from the
    named intake's `blocks.yaml` (`intake`, else the engagement's current
    intake per workflowsv2/engagement_state.py); an engagement with no intake
    falls back to the blocks in engagement.yaml, which is how the fixtures
    state them. `intake_id` says which was used, so a run can pin it.
    """
    d = ENGAGEMENTS / name
    cfg_file = d / "engagement.yaml"
    if not cfg_file.is_file():
        have = ", ".join(sorted(x.name for x in ENGAGEMENTS.iterdir()
                                if x.is_dir())) or "none"
        raise SystemExit(f"no engagement '{name}' in {ENGAGEMENTS} (have: {have})")
    cfg = yaml.safe_load(cfg_file.read_text(encoding="utf-8")) or {}
    target = Path(cfg.get("target") or "")
    if not target.is_absolute():
        target = REPO / target
    brief = d / "brief.md"
    if not brief.is_file():
        raise SystemExit(f"engagement '{name}' has no brief.md")
    from workflowsv2 import engagement_state as state           # noqa: E402
    if intake is not None and intake not in state.intakes(d):
        raise SystemExit(f"engagement '{name}' has no intake '{intake}' "
                         f"(have: {', '.join(state.intakes(d)) or 'none'})")
    intake_id = intake if intake is not None else state.current_intake(d)
    blocks = state.intake_blocks(d, intake_id) if intake_id else {}
    return {"name": name, "dir": d, "target": target.resolve(),
            # Declared by the engagement, never inferred: the documents in
            # which the target asserts things about itself.
            "claim_sources": list(cfg.get("claim_sources") or []),
            "brief": brief, "runs": d / "runs",
            "retention": cfg.get("retention"),
            "intake_id": intake_id,
            # What the practice knows of the deal, for the materiality stage.
            # Free text, handed over verbatim; may be absent.
            "transaction": blocks.get("transaction") or cfg.get("transaction"),
            # The buyer's own thresholds — what changes the price, what ends
            # the deal — written by the intake stage. Free text; may be absent.
            "thresholds": blocks.get("thresholds") or cfg.get("thresholds")}


def engagement_state(world: str, agent: str, leg: int, max_legs: int,
                     elapsed_s: float,
                     claim_sources: Optional[List[str]] = None) -> str:
    """A ledger of what has happened, appended to each `continue`.

    WHY THE RUNNER CARRIES THIS. The agent's own record of its work decays by
    design — observations cap at 1000 chars once stored, and whole legs drop
    out of the history window. The runner's does not. So the one thing an
    agent structurally cannot hold across a long engagement is exactly the
    thing the process driving it already knows for free.

    STATE, NOT GAPS — the line worth holding. Everything here is something
    that happened: legs taken, files read, minutes spent. Nothing here says
    what remains: the agent decides when gathering is done, and handing it a
    completion signal would be feeding the metric to the thing being measured.

    NO CLAIM-SOURCE FRACTION IN v2. The claim source is handed to the
    enumeration call by the runner, line-numbered, so the agent has no reason
    to open it with `inspect_external` and "Claim sources: 0 of 1 opened" read
    as work not done. The fraction was a v1 progress signal for a run that
    enumerated by reading; `claim_sources` is still accepted so the ledger's
    callers need not change, and is not reported.
    """
    # THE ENGAGEMENT DECLARES THE SET; THE RUNNER DOES NOT GUESS IT. This
    # globbed `*.md` in the target until 2026-08-29, which is only meaningful
    # when the target IS a document corpus. On ChatterMate — 1,141 files — it
    # found three markdown files in the root and reported "3 of 3 documents
    # opened so far", a completion signal on the leg where the agent was
    # deciding whether to keep working. Wrong numerator and wrong denominator,
    # agreeing by coincidence: CONTRIBUTING.md is not a claim source and
    # llms.txt is not matched by the glob.
    traces = REPO / "scenarios" / world / agent / "inspect_traces"
    opened = len(list(traces.glob("inspect_external_*.txt"))) \
        if traces.is_dir() else 0
    head = (f"\n\n[engagement state, recorded by the client's process — "
            f"leg {leg} of {max_legs}, {elapsed_s / 60:.0f} min elapsed")
    # A COUNT THAT ONLY GROWS. A fraction saturates and reads as "finished";
    # "37 file reads so far" says the work is still moving and nothing else.
    if opened:
        head += f". {opened} evidence requests so far"
    return head + ".]"


def verify_served_model(model: Dict[str, Any], timeout: float = 10.0
                        ) -> Dict[str, Any]:
    """Ask the server what it is serving, and refuse to run if it is not the
    model we think we are measuring.

    Recovered from the retired bench/common.py on 2026-08-22, the day it was
    needed: both local models declare model:"" (whatever is served), so when the
    box switched from Qwen to Gemma the qwen model would have produced a Gemma
    run labelled Qwen. A comparison whose backend identity rests on
    recollection is not a comparison. The result is recorded verbatim into
    run_meta.json so a row can always name its own backend.
    """
    import urllib.request, urllib.error
    expect = model.get("expects_served_model")
    if not expect:
        return {"checked": False, "reason": "cloud model — nothing to interrogate",
                "served": (model.get("llm_config") or {}).get("model")}
    base = ((model.get("llm_config") or {}).get("vllm_url") or "").rstrip("/")
    if base.endswith("/v1"):
        base = base[:-3]
    url = f"{base}/v1/models"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as fh:
            payload = json.loads(fh.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise SystemExit(
            f"cannot reach {url} to verify the served model ({e}). The local "
            f"backend must be up before a run — a connection refused mid-run "
            f"reads as a mechanism failure in the trace and is not one.")
    served = [m.get("id", "") for m in (payload.get("data") or [])]
    if not any(expect.lower() in s.lower() for s in served):
        raise SystemExit(
            f"model expects a model matching {expect!r} but {url} is serving "
            f"{served}. Restart on the right model, or this row is mislabelled.")
    return {"checked": True, "endpoint": url, "served": served,
            "expected": expect}


def git_rev(path: Path) -> Optional[str]:
    """The commit a tree is at, or None if it is not a repository.

    Used for both the target and this repo. A target that moves under a
    delivered report makes every citation in it resolve against a tree the
    report never saw, and nothing downstream can detect that from a path.
    """
    import subprocess
    try:
        r = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"],
                           capture_output=True, text=True, timeout=10)
        return r.stdout.strip() or None if r.returncode == 0 else None
    except Exception as e:                                     # noqa: BLE001
        logger.warning("git rev for %s: %s", path, e)
        return None


_TRACE_CLAIMS = re.compile(r"^Query: \[claims ([\d, ]+)\]", re.M)


def trace_claims(text: str) -> List[int]:
    """The claim ids an evidence request was filed under, from the `[claims
    N, M]` prefix `react._run_inspect_external` writes into the query."""
    m = _TRACE_CLAIMS.search(text)
    return sorted({int(x) for x in m.group(1).split(",") if x.strip()}) if m else []


def trace_index(traces: Path) -> Dict[Path, Dict[str, Any]]:
    """Every evidence request: the claims it named, its size in full and
    in the trimmed form the adjudication is normally handed."""
    out: Dict[Path, Dict[str, Any]] = {}
    for t in sorted(traces.glob("inspect_external_*.txt")) if traces.is_dir() else []:
        text = t.read_text(encoding="utf-8", errors="replace")
        out[t] = {"claims": trace_claims(text), "chars": len(text),
                  "trimmed": len(trim_trace(text))}
    return out


def evidence_batches(claim_ids: Sequence[int], index: Dict[Path, Dict[str, Any]],
                     batch: int, budget: int) -> List[Dict[str, Any]]:
    """Batches of claims in id order, each with the traces filed under its
    claims.

    GREEDY, IN CLAIM ORDER, AND NOTHING FANCIER (Bruce, 2026-09-02). Start a
    batch with the next claim; add the next one while the batch holds fewer
    than `batch` claims and the FULL traces of the union fit the budget;
    otherwise close it. Sized on the full form so that trimming stays the
    exception: a batch that fits only trimmed would make the cut lines
    routine, and the lines a subagent read but did not cite are worth more
    than one call saved. A claim whose own full traces exceed the budget is a
    batch of one, and `gathered_evidence` hands it the trimmed or compact
    form. A claim no request named joins the walk with no traces, and the
    adjudication is told so: nothing is dumped in to stand in for evidence.
    """
    by_claim = {c: {t for t, v in index.items() if c in v["claims"]}
                for c in claim_ids}

    def size(ts) -> int:
        return sum(index[t]["chars"] for t in ts)

    out: List[Dict[str, Any]] = []
    for c in claim_ids:
        if out and len(out[-1]["claims"]) < max(1, batch):
            union = set(out[-1]["traces"]) | by_claim[c]
            if size(union) <= budget:
                out[-1]["claims"].append(c)
                out[-1]["traces"] = sorted(union)
                continue
        out.append({"claims": [c], "traces": sorted(by_claim[c])})
    for b in out:
        b["untagged"] = [c for c in b["claims"] if not by_claim[c]]
    return out


def files_matched(traces: Path, target: Path) -> List[str]:
    """Target files whose lines a search returned to the auditor, without a
    read. The subagent's grep prints `path:line:text`; those lines are what
    the auditor saw, and a citation of one of them resolves against the
    file. NOT `files_read`: METHOD §8's chase asks for a candidate to be
    opened, and a search hit shows a line, not the file — thirteen cited
    documents read as "never opened" on ChatterMate (2026-09-03) because the
    record check counted reads alone."""
    import re as _re
    out = set()
    if not traces.is_dir() or not target.is_dir():
        return []
    for t in sorted(traces.glob("*.txt")):
        try:
            text = t.read_text(errors="replace")
        except OSError as e:
            logger.warning("manifest: unreadable trace %s (%s)", t, e)
            continue
        for m in _re.finditer(r"^(?:OK: )?([^\s:|]+):\d+:", text, _re.M):
            n = m.group(1)
            while n.startswith("./"):
                n = n[2:]
            if (target / n).is_file():
                out.add(n)
    return sorted(out)


def files_read(traces: Path, target: Path) -> Dict[str, str]:
    """Content hashes for the target files this run actually opened.

    A commit pin says the tree moved. This says whether it moved under a file
    the report cites, which is the question a reader of the report has.

    Derived rather than authored: every evidence request already records the
    files it opened, so this reads the record the run produced instead of
    asking the auditor to declare anything.
    """
    import hashlib
    import re as _re
    out: Dict[str, str] = {}
    if not traces.is_dir() or not target.is_dir():
        return out
    names = set()
    for t in sorted(traces.glob("*.txt")):
        try:
            names.update(_re.findall(r'"file":\s*"([^"]+)"', t.read_text(errors="replace")))
        except OSError as e:
            logger.warning("manifest: unreadable trace %s (%s)", t, e)
    for n in sorted(names):
        # A leading "./" is a prefix, not a character set: `lstrip("./")`
        # turned `.github/...` into `github/...` and dropped every dotfile
        # from the record (same defect as resolve_document, 2026-09-02).
        while n.startswith("./"):
            n = n[2:]
        n = n.lstrip("/")
        f = target / n
        if f.is_file():
            try:
                out[n] = hashlib.sha256(f.read_bytes()).hexdigest()[:16]
            except OSError as e:
                logger.warning("manifest: unreadable target file %s (%s)", f, e)
    return out


def build_config(world: str, model_path: Optional[Path],
                 workflow_mode: Optional[bool] = None,
                 temperature: Optional[float] = None,
                 max_tokens: Optional[int] = None,
                 external_repo: Optional[Path] = None
                 ) -> Tuple[str, Dict[str, Any]]:
    from launcher import parse_characters                      # noqa: E402

    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})

    model_llm = None
    if model_path:
        model = yaml.safe_load(Path(model_path).read_text(encoding="utf-8")) or {}
        model_llm = dict(model.get("llm_config") or {})
        if not model_llm:
            raise SystemExit(f"{model_path}: no llm_config block")
        for char in (scenario.get("characters") or {}).values():
            if isinstance(char, dict) and char.get("mode") == "chat":
                char["llm_config"] = dict(model_llm)   # REPLACE, never merge
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
    # The two per-target values, set here rather than by editing the committed
    # scenario, so a run leaves no diff behind.
    cfg["external_repo"] = str(external_repo)
    # WHY A FLAG AND NOT A SECOND SCENARIO. Comparing workflow_mode on
    # against off needs both, and audit.yaml's own header says why a copy is
    # the wrong way to get one: "a second copy is a second source that
    # drifts". One scenario, one override, and run_meta records which side of
    # the comparison a row is on — a row that cannot name its own
    # configuration is not evidence, the same rule the served-model check
    # exists to enforce.
    if workflow_mode is not None:
        cfg["workflow_mode"] = workflow_mode
    if temperature is not None:
        cfg.setdefault("chat", {})["react_temperature"] = temperature
    # Reasoning tokens bill against the SAME budget as the action. At 8192 a
    # thinking model can deliberate until there is no room left to emit the
    # JSON, and the emission comes back truncated or empty — observed on the
    # grader 2026-08-23, silently, three times in eleven.
    if max_tokens is not None:
        cfg.setdefault("chat", {})["react_max_tokens"] = max_tokens
    return name, cfg


# Both live in workflowsv2/turns.py since 2026-08-30, when a third runner
# needed them. Imported rather than retyped.
from workflowsv2.turns import (                                  # noqa: E402
    last_exit_reason, latest_reply)
# ---------------------------------------------------------------------------
# Concern visibility. Written per leg so a run can be watched while it runs.
#
# WHY THE RUNNER AND NOT concerns.py. Everything below is available at a leg
# boundary, and in a runner-driven run a leg boundary is not merely convenient
# — it is complete. Concerns are created only at the yield point at the end of
# a turn (chat_loop's `elif not autonomous` branch, which runs before the
# post-turn executor is handed anything), and nothing closes, supersedes or
# sweeps one inside a run: a one_shot's stale sweep needs 12 hours against an
# engagement measured in minutes. There are no mid-turn events to miss.
# Instrumenting the machinery instead would edit a dozen sites and change the
# thing being measured.
# ---------------------------------------------------------------------------

# Fields carried per concern. The browser serializer emits every one of these
# twice under legacy aliases; a log is read by a person, so it takes one.
_CONCERN_FIELDS = ("concern_id", "kind", "status", "activation", "strength",
                   "category", "provenance", "rhythm_hours", "created_at",
                   "last_fired_at", "last_bumped_at")


def concern_snapshot(loop) -> list:
    """Every concern in both collections, as of now.

    Built on ChatLoop._all_concerns_split — the resource browser's own
    serializer — so this log and the browser can never disagree about what a
    concern is. Three properties the browser has no use for are read back off
    the note: `system_spawned` decides whether anything is permitted to close
    the concern, and `successor_of` / `successor_depth` say whether it belongs
    to a chain. Those are the questions this log exists to answer.
    """
    user, agent = loop._all_concerns_split()
    rows = []
    for c in list(user) + list(agent):
        nid = c.get("concern_id")
        props = ((loop.resource_manager.get_resource(nid) or {}).get(
            "properties") or {})
        row = {k: c.get(k) for k in _CONCERN_FIELDS}
        row["text"] = c.get("concern_description")
        # FULL, not clipped. A plan clipped at N characters is a plan with its
        # tail cut off, and comparing successive instructions is how a claim
        # that silently left the plan gets found.
        row["instruction"] = c.get("instruction")
        row["system_spawned"] = bool(props.get("system_spawned"))
        row["successor_of"] = props.get("successor_of")
        row["successor_depth"] = props.get("successor_depth")
        rows.append(row)
    return rows


def live_count(rows: list) -> int:
    """How many of these concerns are still live.

    A snapshot carries every state, because _all_concerns_split applies no
    status filter — the resource browser shows satisfied and abandoned notes
    too. A raw length therefore counts retired concerns alongside live ones,
    which read as accumulation on the very run that first retired one
    (cm_glm_2, leg 2: one superseded, one spawned, printed as "1 -> 2").

    Two spellings mean live: _serialize_concern maps a user_concern's status
    into the browser's vocabulary ('open'/'closed') and leaves an
    agent_concern's alone ('active'/'satisfied').
    """
    return sum(1 for r in rows if r.get("status") in ("active", "open"))


def concern_delta(before: list, after: list) -> dict:
    """What one leg did to the concern set. This is the line worth reading."""
    b = {r["concern_id"]: r for r in before}
    a = {r["concern_id"]: r for r in after}
    out = {"created": [], "deleted": sorted(set(b) - set(a)),
           "status_changed": [], "activation_changed": [],
           "instruction_changed": []}
    for nid, row in a.items():
        if nid not in b:
            out["created"].append(
                {"id": nid, "kind": row["kind"], "status": row["status"],
                 "activation": row["activation"],
                 "system_spawned": row["system_spawned"],
                 "successor_of": row["successor_of"],
                 "text": row["text"]})
            continue
        prev = b[nid]
        if prev["status"] != row["status"]:
            out["status_changed"].append([nid, prev["status"], row["status"]])
        if prev["activation"] != row["activation"]:
            out["activation_changed"].append(
                [nid, prev["activation"], row["activation"]])
        if (prev["instruction"] or "") != (row["instruction"] or ""):
            out["instruction_changed"].append(nid)
    return out


def resource_snapshot(loop) -> dict:
    """id -> a one-line description of every resource in the agent's world.

    Concerns are notes too, so they appear here as well as in the concern
    snapshot; the point of this one is everything ELSE that arrives — the
    memories a reflection pass would have written, notes a tool created.
    """
    out = {}
    for r in loop.resource_manager.get_resource_list():
        props = r.get("properties") or {}
        rid = r.get("name") or r.get("id") or ""
        body = " ".join(str(props.get("content", "") or "").split())
        out[rid] = f"{props.get('kind', r.get('type', '?'))}: {body[:100]}"
    return out


def log_outgoing(leg: int, reason: str, text: str) -> None:
    """The whole message, verbatim, before the agent sees it.

    NOT CLIPPED. The runner's message is the entire input side of a leg, and
    a summary of it cannot answer the question it is logged to answer — which
    is what the agent was actually told. The 80-char clip in run_meta's `legs`
    is why the block-rejection text sat unnoticed at the head of a
    continuation concern for as long as it did.
    """
    logger.info("%s\n=== LEG %d  RUNNER -> AGENT  (%s) ===\n%s\n=== end leg "
                "%d message (%d chars) ===", "", leg, reason, text, leg,
                len(text))


def log_incoming(path: Path, leg: int, sent: str, reason: str,
                 exit_reason: str, reply_chars: int,
                 c_before: list, c_after: list,
                 r_before: dict, r_after: dict) -> None:
    """What the leg did: how it ended, and what changed in the agent's world."""
    delta = concern_delta(c_before, c_after)
    new_res = [k for k in r_after if k not in r_before]
    gone_res = [k for k in r_before if k not in r_after]
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps({"leg": leg, "reason": reason, "sent": sent,
                            "exit_reason": exit_reason,
                            "reply_chars": reply_chars,
                            "before": c_before, "after": c_after,
                            "delta": delta,
                            "resources_created": {k: r_after[k] for k in new_res},
                            "resources_deleted": {k: r_before[k] for k in gone_res}},
                           ensure_ascii=False, default=str) + "\n")
    logger.info("=== LEG %d  AGENT -> RUNNER  exit=%s, %d chars ===",
                leg, exit_reason, reply_chars)
    logger.info("  concerns: live %d -> %d (total %d -> %d) | +%d -%d "
                "status:%d activation:%d instruction:%d      "
                "resources: %d -> %d",
                live_count(c_before), live_count(c_after),
                len(c_before), len(c_after), len(delta["created"]),
                len(delta["deleted"]), len(delta["status_changed"]),
                len(delta["activation_changed"]),
                len(delta["instruction_changed"]),
                len(r_before), len(r_after))
    for c in delta["created"]:
        logger.info("  + concern %s %s [%s] act=%s sys=%s parent=%s | %s",
                    c["id"], c["kind"], c["status"], c["activation"],
                    c["system_spawned"], c["successor_of"],
                    " ".join((c["text"] or "").split())[:90])
        if c.get("instruction"):
            logger.info("      instruction (%d chars): %s",
                        len(c["instruction"]), c["instruction"])
    for nid, was, now in delta["status_changed"]:
        logger.info("  ~ concern %s status %s -> %s", nid, was, now)
    for nid, was, now in delta["activation_changed"]:
        logger.info("  ~ concern %s activation %s -> %s", nid, was, now)
    for nid in delta["instruction_changed"]:
        logger.info("  ~ concern %s instruction rewritten", nid)
    for k in new_res:
        logger.info("  + note %s %s", k, r_after[k])
    for k in gone_res:
        logger.info("  - note %s %s", k, r_before[k])



def drive_leg(loop, name: str, world: str, leg: int, text: str, reason: str,
              concern_log: Path, legs: list) -> Tuple[str, str]:
    """One leg: send `text`, log both directions, record it in `legs`.

    Returns the exit reason and the reply. The two loops that drive legs — the
    gathering legs and the chase — do exactly this and differ only in what
    they send and when they stop.
    """
    log_outgoing(leg, reason, text)
    concerns_before = concern_snapshot(loop)
    resources_before = resource_snapshot(loop)
    loop._process_user_turn(source=SOURCE, text=text, close=False)
    reply = latest_reply(loop, SOURCE)
    exit_reason = last_exit_reason(world, name)
    log_incoming(concern_log, leg, text, reason, exit_reason, len(reply),
                 concerns_before, concern_snapshot(loop),
                 resources_before, resource_snapshot(loop))
    legs.append({"leg": leg, "sent": text[:80], "exit_reason": exit_reason,
                 "reply_chars": len(reply)})
    logger.info("leg %d: exit=%s chars=%d", leg, exit_reason, len(reply))
    return exit_reason, reply


def chase_message(todo: Dict[Any, List[str]],
                  frozen: Sequence[Dict[str, Any]]) -> str:
    """The gathering leg that opens the files the searches named. METHOD §8."""
    quotes = {c.get("id"): c.get("quote") for c in frozen}
    lines = ["These claims are unsettled. Your searches named the files below "
             "as the places where the material that could settle each claim "
             "would appear, and you have not opened them. Open each one with "
             "inspect_external, naming the claim ids in the request's "
             "`claims` field, and gather the evidence that settles the "
             "claim. Do not write findings; end the leg with `yield` or "
             "`respond` when the files are read.", ""]
    for cid in sorted(todo, key=lambda x: (x is None, x)):
        lines.append(f"  claim {cid}. {quotes.get(cid)}")
        for f in todo[cid]:
            lines.append(f"      {f}")
    return "\n".join(lines)


def untagged_message(ids: Sequence[int], frozen: Sequence[Dict[str, Any]]) -> str:
    """The gathering leg for claims no evidence request was filed under.
    METHOD §12 step 3: a claim is adjudicated on the requests filed under it
    and nothing else, so a claim with none would be adjudicated on nothing."""
    quotes = {c.get("id"): c for c in frozen}
    lines = ["No evidence request has been filed under the claims below, so "
             "nothing gathered so far can be used to adjudicate them. Gather "
             "the evidence that settles each one with inspect_external, "
             "naming the claim ids in the request's `claims` field. Where the "
             "materials cannot settle a claim, record the searches you made "
             "under its id. Do not write findings; end the leg with `yield` "
             "or `respond` when done.", ""]
    for cid in ids:
        lines.append(_claim_line(quotes.get(cid, {"id": cid})))
    return "\n".join(lines)


def replace_findings(obj: Dict[str, Any], again: Dict[str, Any],
                     wanted: set) -> int:
    """Put the re-adjudicated findings for `wanted` in place of the old ones.
    A finding for a claim outside `wanted` is dropped: the surface is frozen
    and METHOD §4 gives each claim one finding. Returns how many replaced."""
    fresh = {f.get("claim_id"): f
             for f in ((again.get("obj") or {}).get("findings") or [])
             if f.get("claim_id") in wanted}
    n = 0
    for i, f in enumerate(obj.get("findings") or []):
        if f.get("claim_id") in fresh:
            obj["findings"][i] = fresh[f.get("claim_id")]
            n += 1
    return n


def numbered(path: Path) -> str:
    """A document with the line numbers its citations refer to.

    The evidence traces already carry this form — `inspect_external`'s reader
    emits `9|text`. The claim source is handed over the same way so a claim's
    `lines` is read off the same rendering the evidence was.
    """
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(f"{n}|{t}" for n, t in enumerate(lines, 1))


#: Characters of evidence the adjudication call is handed. ~100k tokens: under
#: the local window with a 64k emission to spare, cheap on the cloud route.
EVIDENCE_BUDGET = 400_000


#: Lines kept each side of a cited range when a trace is trimmed. Enough for
#: a signature and its docstring; a constant to tune after a run, not a rule.
GUARD_BAND = 16

_REF = re.compile(r"([\w./-]+\.\w{1,5}):(\d+)(?:-(\d+))?")
_NUMBERED = re.compile(r"^(?:OK: )?(\d+)\|")


def _same_file(ref: str, file: str) -> bool:
    ref, file = ref.strip("./"), file.strip("./")
    return (ref == file or file.endswith("/" + ref) or ref.endswith("/" + file)
            or ("/" not in ref and file.rsplit("/", 1)[-1] == ref))


def trim_trace(text: str, band: int = GUARD_BAND) -> str:
    """One evidence request with its file reads cut down to the lines the
    answer cites, plus `band` lines each side, and its grep hits cut down
    to the files the answer cites. The query, list observations, the tool
    calls without their `thought`, and the final answer are kept whole.

    WHY. A full trace is mostly files read on the way to an answer; the
    compact form (query + answer) drops the numbered lines a verbatim quote
    needs, and adjudicating from it failed the quote check on 51 of 103
    citations (2026-09-02). This keeps what the answer points at, from the
    trace itself, so the adjudication sees the lines the subagent saw.
    """
    head, sep, answer = text.partition("\nFINAL ANSWER:\n")
    if not sep:
        return text
    refs: Dict[str, List[Tuple[int, int]]] = {}
    for m in _REF.finditer(answer):
        lo, hi = int(m.group(2)), int(m.group(3) or m.group(2))
        refs.setdefault(m.group(1), []).append((min(lo, hi), max(lo, hi)))
    out_parts: List[str] = []
    pieces = re.split(r"(?m)^(?=--- iter \d+ ---$)", head)
    out_parts.append(pieces[0].rstrip("\n"))
    for piece in pieces[1:]:
        a, _, obs = piece.partition("\nOBSERVATION:\n")
        a = re.sub(r'(?m)^\s*"thought":\s*".*",?\n', "", a)
        tool = re.search(r'"tool":\s*"([^"]+)"', a)
        ff = re.search(r'"file":\s*"([^"]+)"', a)
        if tool and tool.group(1) == "grep" and obs:
            kept, dropped = [], 0
            for line in obs.splitlines():
                hm = re.match(r"(?:OK: )?([\w./-]+):(\d+):", line)
                if hm and any(_same_file(ref, hm.group(1)) for ref in refs):
                    kept.append(line[4:] if line.startswith("OK: ") else line)
                elif hm:
                    dropped += 1
                else:
                    kept.append(line)
            if dropped:
                kept.append(f"    … {dropped} hit(s) in files the answer does "
                            f"not cite, not shown")
            out_parts.append(a.rstrip("\n") + "\nOBSERVATION:\n" + "\n".join(kept))
            continue
        if not (tool and tool.group(1) == "read" and ff) or not obs:
            out_parts.append(piece.rstrip("\n"))
            continue
        ranges = [r for ref, rs in refs.items() if _same_file(ref, ff.group(1))
                  for r in rs]
        kept, last, dropped = [], None, 0
        for line in obs.splitlines():
            nm = _NUMBERED.match(line)
            if not nm:
                continue
            n = int(nm.group(1))
            if any(lo - band <= n <= hi + band for lo, hi in ranges):
                if last is not None and n != last + 1:
                    kept.append(f"    … {n - last - 1} line(s) not cited")
                kept.append(line[4:] if line.startswith("OK: ") else line)
                last = n
            else:
                dropped += 1
        if not ranges:
            kept = [f"    (read {ff.group(1)}; the answer cites no line of it — "
                    f"{dropped} line(s) not shown)"]
        elif dropped:
            kept.append(f"    … {dropped} line(s) of {ff.group(1)} not cited, not shown")
        out_parts.append(a.rstrip("\n") + "\nOBSERVATION:\n" + "\n".join(kept))
    return "\n\n".join(out_parts) + "\n\nFINAL ANSWER:\n" + answer


def compact_trace(text: str) -> str:
    """One evidence request as its query and the subagent's final answer.

    The answer is what the subagent was asked for — quotes with line numbers
    — and it is what the agent itself saw. The iterations between hold the
    raw files it read on the way, which is most of the trace's size.
    """
    head, sep, answer = text.partition("\nFINAL ANSWER:\n")
    if not sep:
        return text
    query = head.split("\n--- iter 1 ---", 1)[0]
    return f"{query}\nFINAL ANSWER:\n{answer}"


def gathered_evidence(files: Sequence[Path], budget: int) -> Dict[str, Any]:
    """The evidence requests handed to one adjudication call, and what they
    returned. `files` is the batch's traces (`evidence_batches`).

    WHY THE TRACES AND NOT THE TARGET. Handing the emission call the corpus
    would make it a single-call audit that never used the legs — the fixture is
    11 kB and would fit, and a real target would not. The traces are what this
    run actually looked at, which is also what METHOD §14 calls the working
    papers.

    EVERY REQUEST IS REPRESENTED. Until 2026-09-02 this stopped at the first
    trace that overran the budget, and on ChatterMate the adjudication was
    handed 2 of 40 requests: one directory listing and one file. Seventy-two
    claims came back `unverifiable`, most of them citing files the run had in
    fact read. Now three forms are tried in order — full, trimmed to the
    cited lines with a guard band, compact (query and answer) — and the first
    that fits the budget is handed over; `form` records which. Only when the
    compact set overruns the budget are traces left out, in order, and
    `omitted` counts them.
    """
    files = list(files)
    texts = [f.read_text(encoding="utf-8", errors="replace") for f in files]
    form = "full"
    if sum(len(t) for t in texts) > budget:
        form, texts = "trimmed", [trim_trace(t) for t in texts]
    if sum(len(t) for t in texts) > budget:
        form, texts = "compact", [compact_trace(t) for t in texts]
    kept, used = [], 0
    for t in texts:
        if used + len(t) > budget:
            break
        kept.append(t)
        used += len(t)
    return {"text": "\n\n".join(kept), "files": len(files),
            "included": len(kept), "omitted": len(files) - len(kept),
            "form": form, "chars": used}


def _merge_emissions(parts: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Batched adjudications into the one object the checks read.

    Findings concatenate in the order the batches were asked for. Nothing is
    deduplicated: a claim adjudicated twice is a defect `check_output` reports,
    and collapsing it here would hide a batch boundary that went wrong.

    `parse` is the worst outcome across the batches, so one unparseable batch
    cannot read as a clean run.
    """
    if len(parts) == 1:
        return parts[0]
    rank = {"parsed": 0, "repaired": 1, "salvaged": 2, "unparseable": 3}
    findings, unclaimed, questions = [], [], []
    for p in parts:
        o = p.get("obj") or {}
        findings.extend(o.get("findings") or [])
        unclaimed.extend(o.get("unclaimed") or [])
        questions.extend(o.get("questions") or [])
    worst = max(parts, key=lambda p: rank.get(p.get("parse"), 3))
    head = dict(parts[0].get("obj") or {})
    head.update({"findings": findings, "unclaimed": unclaimed,
                 "questions": questions})
    return {"raw": "\n".join(p.get("raw") or "" for p in parts),
            "obj": head, "parse": worst.get("parse"),
            "parse_error": worst.get("parse_error"),
            "finish": worst.get("finish"),
            "attempts": [a for p in parts for a in (p.get("attempts") or [])],
            "response_format_dropped": sorted(
                {d for p in parts for d in (p.get("response_format_dropped") or [])}),
            "phase": "findings", "batches": len(parts),
            "evidence": parts[0].get("evidence")}


def emit_surface(loop, method_text: str, claim_source: Path,
                 section: Tuple[int, int], n: int, of: int,
                 so_far: Sequence[Dict[str, Any]], max_tokens: int
                 ) -> Dict[str, Any]:
    """Phase one, one section: enumerate its claims, before any evidence.

    NO TOOLS AND NO LEGS. Enumeration is reading one document, which the runner
    can hand over directly and line-numbered. Giving the agent a tool loop to
    fetch a document the runner already has would add a failure mode and buy
    nothing — and it would put evidence in front of the enumeration, which is
    the ordering METHOD §5 freezes against.

    ONE SECTION AT A TIME, WITH THE CLAIMS SO FAR. A whole document in one
    call enumerated 101 claims of which a third restated another (ChatterMate
    README, 2026-09-02) and joined table rows the surface check could not
    find. A section is small enough to read closely, and the list of ids so
    far is what lets a restatement be named instead of counted again.
    """
    lo, hi = section
    lines = claim_source.read_text(encoding="utf-8", errors="replace").splitlines()
    body = "\n".join(f"{k}|{t}" for k, t in enumerate(lines[lo - 1:hi], lo))
    prior = ("\n".join(f"  {c['id']}. {c.get('quote')}" for c in so_far)
             or "  (none yet)")
    user = (f"The claim source for this run is `{claim_source.name}`, "
            f"{len(lines)} lines. This is section {n} of {of}, lines {lo} to "
            f"{hi}, with the line numbers your citations refer to:\n\n"
            f"{body}\n\n"
            f"Claims already enumerated from the sections before this one, "
            f"with their ids:\n\n{prior}\n\n"
            f"Emit the claims this section makes, per METHOD \u00a75 and "
            f"\u00a713. You have gathered no evidence and formed no verdicts. "
            f"An assertion already in the list above is emitted with "
            f"`restates` naming its id, not as a new claim.")
    out = emit(loop, method_text, user, schemas.surface_schema(), max_tokens)
    out["phase"] = "surface"
    out["section"] = [lo, hi]
    return out


def emit_findings(loop, method_text: str, claim_source: Path,
                  frozen: Sequence[Dict[str, Any]],
                  traces: Sequence[Path], max_tokens: int,
                  evidence_budget: int = EVIDENCE_BUDGET,
                  note: str = "") -> Dict[str, Any]:
    """The one schema-constrained call that produces the deliverable.

    OUTSIDE THE ReAct LOOP, DELIBERATELY. `REACT_ACTION_SCHEMA` leaves `text`
    unconstrained by design, and a large payload stuffed into an action field
    is a documented death spiral — seven identical retries and six minutes for
    zero progress (subagents/subagent.py). The gathering legs use tools; this
    call uses none and answers under the audit schema instead. It is the shape
    `src/tools/look-at-target/tool.py` already uses, and the shape
    `measure/form_grid`'s adjudicate arm holds form in.

    LARGE READ, BOUNDED WRITE. Every collapse on record tracks how much was
    being generated, never how much was read: 30 of 30 whole findings at 30,000
    characters of context, against a live run breaking inside a 13,767-char
    generation. This call is built to sit on the safe side of that.
    """
    ev = gathered_evidence(traces, evidence_budget)
    user = (
        f"The claim source for this run is `{claim_source.name}`. Its text, "
        f"with the line numbers your citations refer to:\n\n"
        f"{numbered(claim_source)}\n\n"
        f"THE FROZEN CLAIM SURFACE. These are the claims you enumerated, and "
        f"the surface does not change. Adjudicate every one of them, and refer "
        f"to each by its id:\n\n"
        + "\n".join(_claim_line(c) for c in frozen)
        + (f"\n\n{note}" if note else "")
        + f"\n\nThe evidence requests filed under these claims, and what "
          f"they returned:\n\n{ev['text']}\n\n"
          f"Emit the findings document now, per METHOD \u00a713. Every frozen "
          f"claim gets exactly one finding.")
    out = emit(loop, method_text, user, schemas.audit_schema(), max_tokens,
               salvage=schemas.salvage_findings)
    out["phase"] = "findings"
    out["evidence"] = {k: v for k, v in ev.items() if k != "text"}
    return out


def _claim_line(c: Dict[str, Any]) -> str:
    """One frozen claim as the adjudication and the legs see it: id, lines,
    quote, the statement, the `seller` mark, and every further place it is
    made.

    THE STATEMENT IS SHOWN, NOT ONLY THE QUOTE. A quote is verbatim by METHOD
    §5, so a claim split out of a sentence can read "which provides
    automatic scaling..." with its subject in the claim before it — and that
    claim may sit in another batch. The statement is where the enumeration
    resolves the referent ("The platform (Heroku) provides..."), and it was
    not being handed over (Bruce, 2026-09-02).
    """
    line = (f"  {c.get('id')}. [{c.get('lines')}] {c.get('quote')}\n"
            f"      statement: {c.get('statement')}")
    if c.get("about") == "seller":
        line += "  (about the seller, per METHOD \u00a75)"
    for loc in c.get("locations") or []:
        line += f"\n      also at [{loc.get('lines')}]: {loc.get('quote')}"
    return line


#: Appended to the brief. Mechanical, so it lives here and not in each
#: engagement's brief.md: the runner is what reads the tags back.
TAG_INSTRUCTION = (
    "Per METHOD \u00a712 step 3: on every `inspect_external` request, name in "
    "its `claims` field the ids of the claims it gathers evidence for. The "
    "record of the request is filed under those claims, and a claim is "
    "adjudicated on the requests filed under it and nothing else.")


def post_run_checks(obj: Optional[Dict[str, Any]], corpus: Path,
                    claim_source: str, frozen: Sequence[Dict[str, Any]],
                    out: Path, read: Optional[set] = None) -> Dict[str, Any]:
    """METHOD's requirements, over the parsed output.

    REPLACES THREE PROSE PARSERS. v1 checked the ledger with a line regex, the
    findings with a header regex, and the citations by scanning the report for
    `docN:NN`. All three read a document that no longer exists; the same
    requirements are now arithmetic over typed fields, in
    `schemas.check_output`, and they are stricter for it. The ordinal defect
    that once needed a reviewer's judgement — integers in a line-number slot
    that are not line numbers — is a file operation here.
    """
    if obj is None:
        res = {"ok": False, "problems": ["no parseable output to check"],
               "figures": {}}
    else:
        res = schemas.check_output(obj, corpus, claim_source, frozen, read)
    for problem in res["problems"]:
        issues.note(out, stage="claims_audit", code="output_check",
                    text=problem, severity="blocking")
    if res["ok"]:
        logger.info("output check: clean — %s", res["figures"])
    else:
        logger.warning("output check: %d problem(s)", len(res["problems"]))
        for problem in res["problems"]:
            logger.warning("  %s", problem)
    return res


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True,
                    help="engagement name under engagements/ — supplies the "
                         "target, the brief, and where runs land")
    ap.add_argument("--world", required=True,
                    help="fresh world name; never reuse one")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--max-turns", type=int, default=25,
                    help="hard cap on legs (default 25)")
    ap.add_argument("--max-tokens", type=int, default=None,
                    help="override chat.react_max_tokens (scenario default "
                         "8192). Reasoning tokens bill against this budget")
    ap.add_argument("--temperature", type=float, default=None,
                    help="override the action-emission temperature "
                         "(scenario default 0.7)")
    ap.add_argument("--batch", type=int, default=10,
                    help="claims per adjudication call at most; batches are "
                         "formed from claims that share evidence (default 10)")
    ap.add_argument("--surface", type=Path, default=None,
                    help="a claims.json to adjudicate instead of enumerating: "
                         "the surface the practice edited after an "
                         "--enumerate-only run. Checked, then frozen")
    ap.add_argument("--enumerate-only", action="store_true",
                    help="stop after the claim surface is frozen: no gathering "
                         "legs, no adjudication. For comparing enumerations")
    ap.add_argument("--evidence-budget", type=int, default=EVIDENCE_BUDGET,
                    help="characters of evidence traces handed to each "
                         "adjudication call (default %(default)s); over it, "
                         "every trace is handed over in compact form")
    ap.add_argument("--workflow-mode", choices=("on", "off"), default=None,
                    help="override the scenario's workflow_mode; omit to use "
                         "whatever audit.yaml declares")
    args = ap.parse_args()

    if (REPO / "scenarios" / args.world).exists():
        raise SystemExit(
            f"world '{args.world}' already exists — a second run in one world "
            f"inherits the first's conclusions as memories. Pick a fresh name.")

    eng = load_engagement(args.engagement)
    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    # RUNS BELONG TO THE ENGAGEMENT, not to the workflow and not to a fixture.
    out = eng["runs"] / f"{ts}_{args.world}"
    out.mkdir(parents=True, exist_ok=True)
    # A run you can follow while it runs. This logger had no file handler, so
    # watching a run meant redirecting stdout and losing it if you forgot.
    # Attached to this logger only — never basicConfig(force=True), which
    # closes whatever handlers the host process already installed.
    _fh = logging.FileHandler(out / "run.log", encoding="utf-8")
    _fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(_fh)
    concern_log = out / "concern_log.jsonl"

    arm_doc = (yaml.safe_load(Path(args.model).read_text(encoding="utf-8"))
               if args.model else {})
    served_check = verify_served_model(arm_doc)
    if served_check.get("checked"):
        logger.info("served-model check OK: %s", served_check["served"])

    wf_mode = (None if args.workflow_mode is None
               else args.workflow_mode == "on")
    if not eng["target"].is_dir():
        raise SystemExit(f"engagement '{eng['name']}': target "
                         f"{eng['target']} does not exist")
    name, cfg = build_config(args.world, args.model, wf_mode,
                             args.temperature, args.max_tokens,
                             eng["target"])
    logger.info("world=%s model=%s model=%s", args.world, args.model,
                (cfg.get("llm_config") or {}).get("model") or "(scenario default)")

    from chat.chat_loop import ChatLoop                        # noqa: E402
    from chat.model_params import TOP_P                        # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)

    # PRE-FLIGHT: resolve the model and its temperature before any work. An
    # unconfigured model raises here, in one second, instead of producing a
    # full run that has to be thrown away.
    resolved_model = loop.backend.resolved_model()
    resolved_temperature = (cfg.get("chat") or {}).get("react_temperature")
    if resolved_temperature is None:
        resolved_temperature = loop.backend.temperature_for_model()
    logger.info("resolved model=%s temperature=%s top_p=%s",
                resolved_model, resolved_temperature, TOP_P)

    # EVERYTHING THE RUNNER DECIDES, ONCE, IN ONE PLACE. The runner drives the
    # agent with text and nothing else: no concern is injected, no note is
    # written on its behalf, no state object is handed over. What it does set
    # is this configuration, at construction, and then it only ever sends chat
    # messages. Logging it here is what makes the per-leg messages below the
    # whole story rather than most of it.
    _chat = cfg.get("chat") or {}
    logger.info(
        "\n=== ENGAGEMENT CONFIGURATION (set once, at construction) ===\n"
        "  world             %s\n"
        "  agent             %s\n"
        "  target            %s          (inspect_external is bound here)\n"
        "  inspect geofence  %s\n"
        "  workflow document %s          (loaded verbatim into the system prompt)\n"
        "  brief             %s\n"
        "  claim sources     %s\n"
        "  autonomy_enabled  %s          (nothing fires on its own; legs are driven here)\n"
        "  workflow_mode     %s\n"
        "  suppressed        %s\n"
        "  omitted tools     %d\n"
        "  max legs          %d\n"
        "  react_max_tokens  %s\n"
        "=== end configuration; everything after this is a chat message ===",
        args.world, name, cfg.get("external_repo"), cfg.get("inspect_repo"),
        cfg.get("workflow"), eng["brief"], ", ".join(eng["claim_sources"] or []),
        cfg.get("autonomy_enabled"), cfg.get("workflow_mode"),
        "; ".join(getattr(loop, "workflow_suppressed", []) or ["(none)"]),
        len(_chat.get("omitted_tools") or []), args.max_turns,
        _chat.get("react_max_tokens"))

    t0 = time.time()
    legs, error = [], None
    text_first_leg = ''
    # PHASE ONE ENUMERATES, PHASE TWO ADJUDICATES, AND THE LEGS SIT BETWEEN
    # THEM. v1 drove legs until five text blocks had been seen, and the loop's
    # whole complexity was in not believing a model that said it was finished.
    # The deliverables are now produced by two calls the runner makes itself,
    # so the agent's signal has become cheap to believe: `respond` means it has
    # stopped gathering, and being wrong costs evidence, never a deliverable.
    transcript = []
    gathering_legs = 0
    max_iters_legs = 0
    surface: Optional[Dict[str, Any]] = None
    emission: Optional[Dict[str, Any]] = None
    frozen: list = []
    chase: Dict[str, Any] = {"passes": [], "unopened_after": [],
                             "untagged_passes": [], "untagged_after": []}
    batch_log: List[Dict[str, Any]] = []
    traces_dir = REPO / "scenarios" / args.world / name / "inspect_traces"
    src_doc = eng["target"] / eng["claim_sources"][0]
    method_text = load_workflow(REPO / METHOD_PATH)
    emit_tokens = int((cfg.get("chat") or {}).get("react_max_tokens", 32768))
    try:
        # ---- phase one: enumerate section by section, then freeze --------
        # OR TAKE THE SURFACE THE PRACTICE EDITED. The surface is human-owned
        # by decision; `--enumerate-only` produces it and this consumes it
        # after the edit. It is checked like an emitted one and frozen.
        sections = ([] if args.surface else schemas.split_sections(
            src_doc.read_text(encoding="utf-8", errors="replace")))
        if args.surface:
            logger.info("phase 1: taking the edited surface %s", args.surface)
        else:
            logger.info("phase 1: enumerating %s in %d section(s)",
                        src_doc.name, len(sections))
        parts, calls, raws = [], [], []
        assembled: Dict[str, Any] = {"claim_source": src_doc.name, "claims": []}
        if args.surface:
            assembled = json.loads(Path(args.surface).read_text(encoding="utf-8"))
            parts = [assembled]
        for n, sec in enumerate(sections, 1):
            part = emit_surface(loop, method_text, src_doc, sec, n,
                                len(sections), assembled["claims"], emit_tokens)
            calls.append({k: v for k, v in part.items() if k not in ("raw", "obj")})
            raws.append(part.get("raw") or "")
            logger.info("section %d/%d lines %d-%d: %s, finish=%s, %d chars",
                        n, len(sections), sec[0], sec[1], part["parse"],
                        part["finish"], len(part["raw"] or ""))
            if part["response_format_dropped"]:
                error = ("the route dropped "
                         + ", ".join(part["response_format_dropped"])
                         + " — the claim surface was not schema-constrained")
                break
            if part["obj"] is None:
                error = (f"section {n} of the claim surface did not parse "
                         f"(finish={part['finish']}): {part['parse_error']}")
                break
            parts.append(part["obj"])
            assembled = schemas.assemble_surface(src_doc.name, parts)
        surface = {"raw": "\n\n".join(raws),
                   "obj": assembled if parts else None,
                   "parse": max((c["parse"] for c in calls), default=None,
                                key=lambda x: {"parsed": 0, "repaired": 1,
                                               "salvaged": 2}.get(x, 3)),
                   "finish": calls[-1]["finish"] if calls else None,
                   "parse_error": next((c["parse_error"] for c in calls
                                        if c.get("parse_error")), None),
                   "response_format_dropped": sorted(
                       {d for c in calls for d in c["response_format_dropped"]}),
                   "sections": [list(sec) for sec in sections],
                   "surface_source": "edited" if args.surface else "enumerated",
                   "calls": calls, "phase": "surface"}
        if not error:
            frozen = assembled.get("claims") or []
            surface_check = schemas.check_surface(
                assembled, eng["target"], eng["claim_sources"][0])
            for problem in surface_check["problems"]:
                issues.note(out, stage="claims_audit", code="surface_check",
                            text=problem, severity="blocking")
            logger.info("claim surface frozen: %d claims, check %s",
                        len(frozen),
                        "clean" if surface_check["ok"]
                        else f"{len(surface_check['problems'])} problem(s)")
            for problem in surface_check["problems"]:
                logger.warning("  %s", problem)
            (out / "claims.json").write_text(
                json.dumps(assembled, indent=1, ensure_ascii=False) + "\n",
                encoding="utf-8")

        # ---- the gathering legs -----------------------------------------
        if args.enumerate_only:
            logger.info("--enumerate-only: stopping at the frozen surface")
        if not error and frozen and not args.enumerate_only:
            text = (eng["brief"].read_text(encoding='utf-8')
                    + "\n\n" + TAG_INSTRUCTION
                    + "\n\nThe claim surface is frozen. These are the claims "
                      "to find evidence for:\n\n"
                    + "\n".join(_claim_line(c) for c in frozen))
            text_first_leg = text
            sent_reason = "opening brief and the frozen claim surface"
            traces_before = 0
            for i in range(args.max_turns):
                exit_reason, reply = drive_leg(
                    loop, name, args.world, i + 1, text, sent_reason,
                    concern_log, legs)
                # A turn that died is not a turn that finished. Silence reading
                # as success is the failure mode this suite exists to catch.
                if exit_reason in ("llm_error", "crashed"):
                    error = f"turn {i + 1} ended {exit_reason} — run is not valid"
                    break
                # A LEG CUT BY THE ACTION CAP IS A BOUNDARY, NOT A CRASH. On a
                # 1,100-file target the local model spent 16 actions reading and
                # never reached `yield`; its evidence requests were on disk and
                # the run was thrown away (chattermate-readme, 2026-09-02). The
                # cap ends the leg the way `yield` does, and is counted. The
                # run stops when a capped leg added no evidence request, which
                # is a model going round in circles rather than reading.
                if exit_reason == "max_iters":
                    max_iters_legs += 1
                    traces_now = len(list((REPO / "scenarios" / args.world / name
                                           / "inspect_traces").glob("inspect_external_*.txt")))
                    if traces_now <= traces_before:
                        error = (f"turn {i + 1} hit max_iters and made no new "
                                 f"evidence request — run is not valid")
                        break
                    traces_before = traces_now
                    logger.warning("leg %d: cut by the action cap; %d evidence "
                                   "requests so far — continuing", i + 1, traces_now)
                    exit_reason = "yield"
                transcript.append(reply)
                gathering_legs = i + 1
                if exit_reason != "yield":
                    logger.info("leg %d: agent stopped gathering", i + 1)
                    break
                text = CONTINUE + engagement_state(
                    args.world, name, i + 2, args.max_turns,
                    time.time() - t0, claim_sources=eng["claim_sources"])
                sent_reason = "agent yielded — continue gathering"
            else:
                # Not an error. The cap means gathering was cut short and the
                # adjudication still runs on what was gathered — a thin audit
                # that says so beats no audit. `gathering_capped` records it.
                logger.info("leg cap reached with the agent still gathering")

        # ---- phase two: adjudicate the frozen claims --------------------
        by_id = {c.get("id"): c for c in frozen}

        def adjudicate(ids: Sequence[int], note: str = "") -> Dict[str, Any]:
            """The claims `ids`, in batches that share evidence, each given
            only the evidence requests filed under its claims (METHOD §8's
            `claims` tags). One function for the first pass, the reprompt
            and the chase, so all three see the same evidence rule.

            THE BATCH IS SIZED BY EVIDENCE, NOT BY COUNT ALONE. One call over
            101 claims quoted from the compact record and failed the verbatim
            check on 51 of 103 citations (2026-09-02). A batch that shares
            traces fits them in full, so the quote comes from numbered lines.
            Each batch's findings are written to findings.partial.json as
            they land, so a run that dies keeps what it had.
            """
            index = trace_index(traces_dir)
            batches = evidence_batches(list(ids), index, args.batch,
                                       args.evidence_budget)
            emissions = []
            for bi, b in enumerate(batches, 1):
                logger.info("adjudicating batch %d/%d: %d claim(s), %d "
                            "trace(s), %d with no request filed", bi,
                            len(batches), len(b["claims"]), len(b["traces"]),
                            len(b["untagged"]))
                bnote = note
                if b["untagged"]:
                    bnote += (("\n\n" if note else "")
                              + "No evidence request was filed under "
                              + ("claim " if len(b["untagged"]) == 1 else "claims ")
                              + ", ".join(str(c) for c in b["untagged"])
                              + ". Every claim still gets exactly one finding "
                                "(METHOD \u00a74). Adjudicate such a claim on "
                                "what is below if it bears on the claim; "
                                "otherwise its verdict is `unverifiable`, "
                                "recording that no search was made.")
                e = emit_findings(
                    loop, method_text=method_text, claim_source=src_doc,
                    frozen=[by_id[c] for c in b["claims"] if c in by_id],
                    traces=b["traces"], max_tokens=emit_tokens,
                    evidence_budget=args.evidence_budget, note=bnote)
                emissions.append(e)
                batch_log.append({"claims": b["claims"],
                                  "traces": len(b["traces"]),
                                  "untagged": b["untagged"],
                                  "parse": e["parse"], "finish": e["finish"],
                                  "evidence": e["evidence"]})
                (out / "findings.partial.json").write_text(json.dumps(
                    _merge_emissions(emissions).get("obj") or {},
                    indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
            return _merge_emissions(emissions) if emissions else {
                "raw": "", "obj": None, "parse": "unparseable",
                "parse_error": "no claims to adjudicate", "finish": None,
                "attempts": [], "response_format_dropped": [],
                "phase": "findings", "evidence": None}

        # THE UNTAGGED CHASE, BEFORE ADJUDICATION. A claim no evidence
        # request was filed under is adjudicated on nothing: a batch of ten
        # such claims returned an empty findings list, twice, on the first
        # local ChatterMate run (2026-09-02). Same loop shape as the chase
        # below — one targeted gathering leg naming the claims, then
        # recompute — until every claim is named, a pass names nothing new,
        # or the leg cap is spent. What is left is recorded, and those claims
        # are adjudicated with the note that nothing was filed.
        if not error and frozen and not args.enumerate_only:
            all_ids = [c.get("id") for c in frozen]
            while len(legs) < args.max_turns:
                index = trace_index(traces_dir)
                tagged = {cid for v in index.values() for cid in v["claims"]}
                untagged = [cid for cid in all_ids if cid not in tagged]
                if not untagged:
                    break
                leg_no = len(legs) + 1
                logger.warning("untagged chase pass %d: %d claim(s) with no "
                               "evidence request filed — gathering leg %d",
                               len(chase["untagged_passes"]) + 1,
                               len(untagged), leg_no)
                exit_reason, _ = drive_leg(
                    loop, name, args.world, leg_no,
                    untagged_message(untagged, frozen),
                    "untagged chase: gather evidence for claims with none",
                    concern_log, legs)
                now_tagged = {cid for v in trace_index(traces_dir).values()
                              for cid in v["claims"]}
                newly = sorted(set(untagged) & now_tagged)
                chase["untagged_passes"].append(
                    {"leg": leg_no, "claims": untagged, "newly_tagged": newly,
                     "exit_reason": exit_reason})
                if exit_reason in ("llm_error", "crashed"):
                    error = (f"untagged chase leg {leg_no} ended {exit_reason} "
                             f"— run is not valid")
                    break
                if not newly:
                    logger.warning("untagged chase pass named no new claim — stopping")
                    break
            index = trace_index(traces_dir)
            tagged = {cid for v in index.values() for cid in v["claims"]}
            chase["untagged_after"] = [cid for cid in all_ids if cid not in tagged]
            if chase["untagged_after"]:
                logger.warning("%d claim(s) still have no evidence request: %s",
                               len(chase["untagged_after"]),
                               ", ".join(str(c) for c in chase["untagged_after"]))

        if not error and frozen and not args.enumerate_only:
            logger.info("phase 2: adjudicating %d frozen claims", len(frozen))
            emission = adjudicate([c.get("id") for c in frozen])
            logger.info("findings: %s, finish=%s, %d chars, evidence %s",
                        emission["parse"], emission["finish"],
                        len(emission["raw"] or ""), emission["evidence"])
            # A SILENTLY UNCONSTRAINED RUN IS WORSE THAN A FAILED ONE.
            # backend._post_adapting drops response_format for the whole
            # session on one 400 and logs it at INFO, so a schema-dependent
            # audit can quietly become a free-text one. Fatal here.
            if emission["response_format_dropped"]:
                error = ("the route dropped "
                         + ", ".join(emission["response_format_dropped"])
                         + " — the findings were not schema-constrained")
            elif emission["parse"] == "unparseable":
                error = (f"findings did not parse "
                         f"(finish={emission['finish']}): "
                         f"{emission['parse_error']}")

            # ONE REPROMPT FOR CLAIMS THAT GOT NO FINDING. Mechanical, and no
            # judgement is involved: the frozen surface says which claims
            # exist, the emission says which were adjudicated, and the
            # difference is a set operation. Bruce's call, one reprompt only
            # — a loop that asks until the model complies measures the loop
            # rather than the model, which is the argument that kept a
            # retry-until-it-looks-right runner out of v1.
            if not error and emission and emission.get("obj"):
                done = {f.get("claim_id")
                        for f in (emission["obj"].get("findings") or [])}
                missing = [c for c in frozen if c.get("id") not in done]
                if missing:
                    ids = ", ".join(str(c.get("id")) for c in missing)
                    logger.warning("%d frozen claim(s) with no finding: %s "
                                   "— reprompting once", len(missing), ids)
                    again = adjudicate([c.get("id") for c in missing])
                    reprompt = {"claims": [c.get("id") for c in missing],
                                "parse": again["parse"],
                                "finish": again["finish"],
                                "recovered": 0}
                    extra = ((again.get("obj") or {}).get("findings") or [])
                    # Only findings for the claims that were actually missing.
                    # A reprompt that re-adjudicates a claim already settled
                    # would put two findings on one claim, which METHOD §4
                    # forbids and check_output would then report as our defect.
                    wanted = {c.get("id") for c in missing}
                    keep = [f for f in extra if f.get("claim_id") in wanted
                            and f.get("claim_id") not in done]
                    emission["obj"]["findings"].extend(keep)
                    reprompt["recovered"] = len(keep)
                    emission["reprompt"] = reprompt
                    logger.info("reprompt recovered %d of %d",
                                len(keep), len(missing))

            # THE CHASE (METHOD §8). An `unverifiable` finding whose searches
            # named a file the run never opened is not unsettled by the
            # materials; it is unexamined. Nothing is adjudicated during the
            # gathering legs, so the runner had no grounds to disbelieve a
            # `respond` with those files still unread. Now it has: for each
            # such claim, one gathering leg names the claims and the files,
            # then only those claims are adjudicated again over the enlarged
            # record. It stops when no claim names an unopened file, when a
            # pass opened nothing new, or when the leg cap is spent. What is
            # left is reported as a number — files named and not opened —
            # and the findings keep `not_examined`.
            if not error and emission and emission.get("obj"):
                docs = schemas.corpus_index(eng["target"])
                while len(legs) < args.max_turns:
                    read = set(files_read(traces_dir, eng["target"]))
                    cands = schemas.candidate_files(
                        emission["obj"].get("findings") or [], docs, read)
                    todo = {cid: c["unopened"] for cid, c in cands.items()
                            if c["unopened"]}
                    if not todo:
                        break
                    files = sorted({f for fs in todo.values() for f in fs})
                    leg_no = len(legs) + 1
                    logger.warning("chase pass %d: %d claim(s) name %d unopened "
                                   "file(s) — gathering leg %d",
                                   len(chase["passes"]) + 1, len(todo),
                                   len(files), leg_no)
                    exit_reason, _ = drive_leg(
                        loop, name, args.world, leg_no,
                        chase_message(todo, frozen),
                        "chase: open the files the searches named",
                        concern_log, legs)
                    opened = set(files_read(traces_dir, eng["target"])) - read
                    entry = {"leg": leg_no, "claims": sorted(todo),
                             "files_named": files,
                             "files_opened": sorted(opened),
                             "exit_reason": exit_reason, "readjudicated": 0}
                    chase["passes"].append(entry)
                    if exit_reason in ("llm_error", "crashed"):
                        error = (f"chase leg {leg_no} ended {exit_reason} — "
                                 f"run is not valid")
                        break
                    if not opened:
                        logger.warning("chase pass opened no new file — stopping")
                        break
                    again = adjudicate(
                        sorted(todo, key=lambda x: (x is None, x)),
                        note=("These claims were adjudicated `not_examined`: "
                              "their searches named files that had not been "
                              "opened. Those files have since been opened and "
                              "their contents are in the evidence below. "
                              "Adjudicate these claims again."))
                    entry["parse"] = again["parse"]
                    entry["finish"] = again["finish"]
                    entry["readjudicated"] = replace_findings(
                        emission["obj"], again, set(todo))
                    logger.info("chase pass: re-adjudicated %d of %d",
                                entry["readjudicated"], len(todo))
                    if entry["readjudicated"] == 0:
                        break
                read = set(files_read(traces_dir, eng["target"]))
                cands = schemas.candidate_files(
                    emission["obj"].get("findings") or [], docs, read)
                chase["unopened_after"] = sorted(
                    {f for c in cands.values() for f in c["unopened"]})
                if chase["unopened_after"]:
                    logger.warning("%d file(s) named by searches were not "
                                   "opened: %s", len(chase["unopened_after"]),
                                   ", ".join(chase["unopened_after"]))
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("run failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
        try:
            loop._persist_to_disk()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("final persist failed: %s", e)

    wall = round(time.time() - t0, 1)

    # TWO DELIVERABLES, ONE PER PHASE. v1 assembled report.md from three text
    # blocks and gap_map.md from a fourth, and spliced a coverage sentence
    # into each because the agent was forbidden to compute one. None of that
    # survives: the outputs are JSON, the figures are arithmetic over them,
    # and the client's report is written two stages downstream.
    final = latest_reply(loop, SOURCE)
    if final:
        (out / "full_reply.md").write_text(final, encoding="utf-8")

    if surface is not None:
        (out / "surface_emission.txt").write_text(surface.get("raw") or "",
                                                  encoding="utf-8")
    obj = (emission or {}).get("obj")
    if emission is not None:
        # The raw emission is kept whatever happened to it. A response that did
        # not parse is the only evidence of why, and discarding it leaves a run
        # that failed with nothing to look at.
        (out / "emission.txt").write_text(emission.get("raw") or "",
                                          encoding="utf-8")
    if obj is not None:
        (out / "findings.json").write_text(
            json.dumps(obj, indent=1, ensure_ascii=False) + "\n",
            encoding="utf-8")

    if args.enumerate_only:
        checks = {"ok": True, "problems": [],
                  "figures": {"enumerate_only": True}}
    else:
        checks = post_run_checks(obj, eng["target"], eng["claim_sources"][0],
                                 frozen, out,
                                 read=set(files_read(traces_dir, eng["target"])))
    if obj is not None and (out / "findings.partial.json").is_file():
        (out / "findings.partial.json").unlink()
    if not error and not checks["ok"]:
        error = f"output check failed: {len(checks['problems'])} problem(s)"
    if chase["untagged_after"]:
        issues.note(out, stage="claims_audit", code="no_evidence_request",
                    text=f"{len(chase['untagged_after'])} claim(s) had no "
                         f"evidence request filed under them after the chase: "
                         + ", ".join(str(c) for c in chase["untagged_after"]),
                    severity="check")
    if chase["unopened_after"]:
        issues.note(out, stage="claims_audit", code="not_examined",
                    text=f"{len(chase['unopened_after'])} file(s) named by "
                         f"searches were not opened: "
                         + ", ".join(chase["unopened_after"]),
                    severity="check")

    # THE WORKING RECORD, COPIED OUT (§14). The world is discarded after an
    # engagement; these two are the evidence that the work was systematic, and
    # they are the only evidence of the claims examined that produced no
    # finding. Copied here, beside the deliverables, so they survive the
    # world's deletion.
    #
    # BY THE HARNESS, NOT THE AUDITOR. §12 step 7 deliberately does not ask
    # the agent to produce this: an auditor's own account of its diligence is
    # the weakest evidence of that diligence. The value is that this is a
    # byproduct of doing the work rather than a claim about having done it.
    #
    # HOLDS CLIENT MATERIAL. Verbatim lines from the target's documents are in
    # these traces. Harmless for the synthetic fixture; a retention obligation
    # for a real engagement, which belongs in the engagement letter.
    world_dir = REPO / "scenarios" / args.world / name
    record = out / "working_record"

    # THE INSTRUMENT, COPIED OUT WITH IT. The record shows what the auditor
    # did; these two show what it was told to do. Without them a reader — or a
    # continuation answering questions about this run later — cannot tell
    # which method produced the report, and a method that has moved since
    # supplies the wrong verdict vocabulary silently.
    #
    # THE DELIVERED TEXT, NOT THE FILE. workflowsv2/claims_audit/method/METHOD.md is not what the agent
    # read: the loader strips every section marked for the practice. A commit
    # hash would name the file and still not answer what reached the model, and
    # it needs the repository at that revision to resolve at all. A copy is
    # self-contained and is the artifact itself.
    record.mkdir(parents=True, exist_ok=True)
    try:
        wf = (cfg.get("workflow") or "").strip()
        if wf:
            (record / "method_as_delivered.md").write_text(
                load_workflow(REPO / wf), encoding="utf-8")
    except Exception as e:                                     # noqa: BLE001
        logger.warning("working record: method not copied (%s)", e)
    try:
        (record / "brief.md").write_text(text_first_leg, encoding="utf-8")
    except Exception as e:                                     # noqa: BLE001
        logger.warning("working record: brief not copied (%s)", e)

    # THE SCENARIO AS RESOLVED, not the file. The model config replaces a block
    # at runtime, and what shaped the run is the result. omitted_tools, the
    # geofence and workflow_mode all decide what the auditor could do, and
    # "why did you not check X" may have the answer "that tool was not bound".
    try:
        (record / "scenario_as_used.json").write_text(
            json.dumps(cfg, indent=2, default=str), encoding="utf-8")
    except Exception as e:                                     # noqa: BLE001
        logger.warning("working record: scenario not copied (%s)", e)
    for src in (world_dir / "memory" / "reasoning_trace.jsonl",
                world_dir / "inspect_traces"):
        if not src.exists():
            logger.warning("working record: %s absent", src.name)
            continue
        try:
            dst = record / src.name
            record.mkdir(parents=True, exist_ok=True)
            if src.is_dir():
                shutil.copytree(src, dst, dirs_exist_ok=True)
            else:
                shutil.copy2(src, dst)
        except OSError as e:
            logger.warning("working record: could not copy %s (%s)", src, e)
    if record.exists():
        kb = sum(f.stat().st_size for f in record.rglob("*") if f.is_file()) // 1024
        logger.info("working record: %d KB copied to %s", kb, record)

    (out / "run_meta.json").write_text(json.dumps({
        "world": args.world,
        "model_config": str(args.model) if args.model else "(scenario default)",
        "workflow_mode": bool(cfg.get("workflow_mode")),
        # WHAT workflow_mode MEANT ON THE DAY. The boolean above is a name for
        # the membership of workflow.py's _SUPPRESSED list, and that
        # membership changes — reflection joined it 2026-08-29. Two rows both
        # reading `workflow_mode: true` are the same configuration only if
        # this list also matches. Read off the loop, which is the object the
        # suppression was applied to: recomputing it here would be the second
        # source that drifts.
        "workflow_suppressed": loop.workflow_suppressed,
        "react_temperature": (cfg.get("chat") or {}).get(
            "react_temperature", 0.7),
        "react_max_tokens": (cfg.get("chat") or {}).get(
            "react_max_tokens", 8192),
        # THE SETTINGS THAT ACTUALLY APPLIED, resolved from the model rather
        # than copied from config. A row that cannot name its own sampling
        # settings is not evidence — the rule that already governs the served
        # -model check, extended to the thing that cost a campaign.
        "resolved_model": resolved_model,
        "resolved_temperature": resolved_temperature,
        "top_p": TOP_P,
        "llm_config": cfg.get("llm_config"),
        "served_model_check": served_check,
        "external_repo": cfg.get("external_repo"),
        # A PATH IS NOT A PIN. run_meta used to record where the target was and
        # not what it contained, so a citation could later resolve against a
        # tree the report never saw, silently. The commit says the tree moved;
        # the manifest below says whether it moved under a file this run read.
        "target_rev": git_rev(Path(cfg.get("external_repo") or ".")),
        # WHICH FILES WERE THE MATERIALS: tracked files when the target is a
        # worktree, else the walk; binary files named so a reader can see
        # what the index never held (schemas.corpus_view).
        "materials": schemas.corpus_view_summary(
            Path(cfg.get("external_repo") or ".")),
        "harness_rev": git_rev(REPO),
        "files_read": files_read(record / "inspect_traces",
                                 Path(cfg.get("external_repo") or ".")),
        "files_matched": files_matched(record / "inspect_traces",
                                       Path(cfg.get("external_repo") or ".")),
        "legs": legs,
        # WHAT THE GATHERING LEGS COST, and whether the cap cut them short.
        # v1 recorded which of five blocks each leg delivered and how often the
        # runner had to ask; there are no blocks to ask for now, and the one
        # thing worth knowing about the legs is how many the evidence took.
        "gathering_legs": gathering_legs,
        "gathering_capped": gathering_legs >= args.max_turns,
        "max_iters_legs": max_iters_legs,
        # METHOD §8's chase: each pass's leg, the claims and files it named,
        # what it opened, and how many claims were adjudicated again. What is
        # still unopened at the end is the not-examined figure.
        "chase": chase,
        "evidence_budget": args.evidence_budget,
        "wall_clock_s": wall,
        # Action emissions the token ceiling cut off, across the gathering
        # legs. The emission call reports its own separately, below.
        "finish_length_events": getattr(loop, "finish_length_events", None),
        # THE ROUTE'S RELIABILITY ENTRY: 429s and 5xx retried, by code, and
        # calls that exhausted the retry budget. Counted on the backend so
        # the subagents' calls are in it.
        "transient_events": getattr(loop, "transient_events", None),
        # THE EMISSION, AND EVERYTHING THAT COULD HAVE SILENTLY DEGRADED IT.
        # `parse` is "parsed" on a clean run; "repaired" or "salvaged" means
        # the response was cut and only some of it survived, which is the
        # constrained-decoding failure mode — invalid JSON rather than a short
        # report. `response_format_dropped` is fatal upstream and recorded
        # here too, because a run that silently stopped being schema-
        # constrained must never read as a clean one.
        "surface": ({k: v for k, v in surface.items() if k not in ("raw", "obj")}
                    if surface else None),
        "frozen_claims": len(frozen),
        "sections": (surface or {}).get("sections"),
        "surface_source": (surface or {}).get("surface_source"),
        # THE ADJUDICATION BATCHES: which claims, how many traces each was
        # handed, and whether it fell back to the whole record because no
        # request named its claims.
        "batches": batch_log if frozen else [],
        "emission": ({k: v for k, v in emission.items() if k not in ("raw", "obj")}
                     if emission else None),
        # METHOD's requirements over the parsed output, in place of v1's three
        # prose parsers. `problems` is empty on a clean run.
        "output_check": checks,
        "error": error,
        "captured_at_utc": ts,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(legs)} legs, {wall}s, error={error}")
    fig = checks.get("figures") or {}
    print(f"emission: {(emission or {}).get('parse')}, "
          f"findings={fig.get('findings')}, verdicts={fig.get('verdicts')}")
    print(f"output check: {'clean' if checks['ok'] else str(len(checks['problems'])) + ' problem(s)'}")
    print(f"chase: {len(chase['passes'])} pass(es), "
          f"{len(chase['unopened_after'])} file(s) named and not opened")
    print(f"deliverables: {out}/claims.json, findings.json")
    print(f"meta: {out / 'run_meta.json'}")
    # A FIXTURE RUN IS SCORED BY READING. The answer key is
    # measure/fixtures/dataroom/answer_key.md; the mechanical scorer that read
    # v1 text reports was deleted 2026-09-02 (Bruce: "I'd rather have you
    # perform that step"). A real engagement has no answer key.
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
