#!/usr/bin/env python3
"""Drive one security audit run, unattended: run the probes, then the audit.

    python3 workflows/security_audit/runner.py --world sec_1
    python3 workflows/security_audit/runner.py --world sec_2 \
            --model measure/models/or_glm53flash.yaml --max-turns 20

Two phases. First the **probe set** runs and its output becomes the
**collection** — the only evidence the auditor gets (METHOD §1). Then
`inspect_external` is pointed at that collection and the audit is driven leg by
leg, exactly as the claims audit is.

A COPY, DELIBERATELY, AND WHAT WAS COPIED. This file began as a verbatim copy of
`workflows/claims_audit/runner.py` at commit 7da28d34. Verbatim and unchanged:
`verify_served_model`, `git_rev`, `build_config`, `last_exit_reason`,
`latest_reply`, `concern_snapshot`, `live_count`, `concern_delta`,
`resource_snapshot`, `log_outgoing`, `log_incoming`, and the leg loop in
`main()` including its block accounting and yield rule. Replaced:
`load_engagement` (an engagement here is a host, not a data room),
`engagement_state` -> `probe_state`, `files_read` -> the collection manifest.
New, with no counterpart there: `run_probe_set`.

The list is here so the refactor that follows a working run has a manifest
rather than a diff hunt. `marker_re` is what happens without one — three copies
in this tree, and the same defect had to be fixed in each on 2026-08-29.

WHY A COPY AND NOT AN ABSTRACTION. Commonality cannot be extracted from one
instance; seams chosen from a sample of one are guesses.
`docs/workflow-concern-layers.md` says the same: "Only a second workflow will
show which parts are general." This is that second workflow. Refactor after a
run succeeds, not before.

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
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent          # workflows/security_audit
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

SCENARIO = HERE / "scenario.yaml"
ENGAGEMENTS = HERE / "engagements"
SOURCE = "User"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("security.run")
logger.setLevel(logging.INFO)

# DELIVERY IS BY BLOCK (METHOD §16), not by turn boundary. `workflows/blocks.py`
# holds the vocabulary and the reason; this file only drives it.
from workflows.security_audit import blocks                   # noqa: E402

# The brief lives with its engagement, in engagements/<name>/brief.md — it is
# what a client says, and it differs per engagement. Its history travels with
# it as HTML comments in that file.
CONTINUE = "continue"

# NO DELIVERABLE IS ASKED FOR ANY MORE. Until 2026-08-27 the runner drove the
# report and the Gap Map as one turn each and knew which was which because it
# had asked — which is exactly how a claim enumeration became a report. Blocks
# announce themselves (METHOD §16), so the runner asks for nothing while they
# arrive and speaks only to name one that has not.


def load_engagement(name: str) -> Dict[str, Any]:
    """The host under audit, the brief, and where runs land.

    THE ENGAGEMENT SUPPLIES WHAT THE METHOD MUST NOT INVENT: which host is in
    scope, and which probes are authorised on it (METHOD §12 — authorisation is
    an engagement precondition and it is per probe). Neither is knowable to the
    method's author, and the auditor may not widen either.

    There is no `target` here, unlike the claims audit. The target of a security
    audit is a running system, and what the auditor reads is the COLLECTION this
    runner produces from it — see run_probe_set.
    """
    d = ENGAGEMENTS / name
    cfg_file = d / "engagement.yaml"
    if not cfg_file.is_file():
        have = ", ".join(sorted(x.name for x in ENGAGEMENTS.iterdir()
                                if x.is_dir())) if ENGAGEMENTS.is_dir() else "none"
        raise SystemExit(f"no engagement '{name}' in {ENGAGEMENTS} (have: {have or 'none'})")
    cfg = yaml.safe_load(cfg_file.read_text(encoding="utf-8")) or {}
    brief = d / "brief.md"
    if not brief.is_file():
        raise SystemExit(f"engagement '{name}' has no brief.md")
    authorised = list(cfg.get("probes") or [])
    if not authorised:
        raise SystemExit(
            f"engagement '{name}' authorises no probes. METHOD §12 makes "
            f"authorisation an engagement precondition and per probe; a "
            f"collection nobody authorised is not evidence.")
    return {"name": name, "dir": d, "brief": brief, "runs": d / "runs",
            "hosts": list(cfg.get("hosts") or ["localhost"]),
            "probes": authorised,
            "retention": cfg.get("retention")}


# THE PROBE SET. Imported rather than redeclared: src/chat/subagents/security.py
# already defines every read-only command Sentinel runs, with the argument-list
# discipline METHOD §12 requires (hardcoded argv, no LLM-supplied arguments, no
# shell). A fourth copy of those tables is how `marker_re` got to three.
from chat.subagents.security import (                          # noqa: E402
    _SYSTEM_STATE_COMMANDS, _HOST_STATE_COMMANDS, _SUDO_HOST_COMMANDS,
    _SYSTEM_STATE_TIMEOUT, _HOST_STATE_TIMEOUT, _FIND_TIMEOUT)

# Per-probe budgets that differ from their table's default. `suid` walks the
# whole filesystem; security.py gives it _FIND_TIMEOUT for that reason, and a
# table that flattened every host probe to one budget timed it out at 15s on
# the first smoke test.
_PROBE_TIMEOUT_OVERRIDE = {"suid": _FIND_TIMEOUT}

# name -> (argv, timeout, needs a granted elevation)
PROBES: Dict[str, Tuple[list, float, bool]] = {
    **{k: (v, _SYSTEM_STATE_TIMEOUT, False) for k, v in _SYSTEM_STATE_COMMANDS.items()},
    **{k: (v, _PROBE_TIMEOUT_OVERRIDE.get(k, _HOST_STATE_TIMEOUT), False)
       for k, v in _HOST_STATE_COMMANDS.items()},
    **{k: (v, _HOST_STATE_TIMEOUT, True) for k, v in _SUDO_HOST_COMMANDS.items()},
}

# METHOD §12's four outcomes. `not run` is recorded for a probe the engagement
# did not authorise, so the collection says what was out of scope rather than
# leaving the reader to infer it from an absence.
OUTCOME_COMPLETED = "completed"
OUTCOME_TIMED_OUT = "timed out"
OUTCOME_UNAUTHORISED = "unauthorised"
OUTCOME_NOT_RUN = "not run"


def run_probe_set(collection: Path, authorised: List[str]) -> Dict[str, Any]:
    """Run the authorised probes and write the collection. Returns the outcomes.

    **This is what makes "the auditor touched nothing" a fact about the system
    rather than a promise from a model** (METHOD §12). The auditor never sees a
    command; it reads files.

    Classification is by exception and exit status, never by parsing an error
    message: a timeout raises, an elevation that was never granted exits
    non-zero from `sudo -n`, and everything else that fails is reported with its
    status so a reader can tell a missing tool from a refused one.

    A probe that did not complete is part of the record (METHOD §1) — its file
    is still written, carrying the outcome and nothing else, so a citation into
    it resolves and the difference between "no listener was found" and "the
    listener walk did not finish" is on the page.
    """
    import subprocess
    collection.mkdir(parents=True, exist_ok=True)
    outcomes: Dict[str, Any] = {}
    for name in sorted(PROBES):
        argv, timeout, needs_grant = PROBES[name]
        path = collection / f"{name}.txt"
        if name not in authorised:
            outcomes[name] = {"outcome": OUTCOME_NOT_RUN,
                              "why": "not authorised by the engagement"}
            path.write_text(f"# {name}: not run — not authorised by the "
                            f"engagement.\n", encoding="utf-8")
            continue
        rec: Dict[str, Any] = {"argv": argv, "timeout_s": timeout,
                               "elevated": needs_grant}
        try:
            proc = subprocess.run(argv, capture_output=True, text=True,
                                  timeout=timeout, check=False)
        except subprocess.TimeoutExpired:
            rec["outcome"] = OUTCOME_TIMED_OUT
            rec["why"] = f"did not finish within {timeout:.0f}s"
            path.write_text(f"# {name}: timed out after {timeout:.0f}s. "
                            f"No output. This is an observation about the "
                            f"audit, not about the host.\n", encoding="utf-8")
            logger.warning("probe %s TIMED OUT (>%.0fs)", name, timeout)
            outcomes[name] = rec
            continue
        except OSError as e:
            rec["outcome"] = OUTCOME_NOT_RUN
            rec["why"] = f"could not launch: {e}"
            path.write_text(f"# {name}: could not launch ({e}).\n",
                            encoding="utf-8")
            logger.warning("probe %s could not launch: %s", name, e)
            outcomes[name] = rec
            continue
        if proc.returncode != 0 and needs_grant:
            rec["outcome"] = OUTCOME_UNAUTHORISED
            rec["why"] = (f"`sudo -n` exited {proc.returncode}; the grant this "
                          f"probe needs was not given")
            path.write_text(f"# {name}: unauthorised — the elevation this "
                            f"probe needs was not granted. METHOD §14: this is "
                            f"a gap with an exact remedy.\n", encoding="utf-8")
            logger.warning("probe %s UNAUTHORISED (rc=%d)", name, proc.returncode)
            outcomes[name] = rec
            continue
        if proc.returncode != 0:
            rec["outcome"] = OUTCOME_NOT_RUN
            rec["why"] = f"exited {proc.returncode}"
            path.write_text(f"# {name}: exited {proc.returncode}.\n"
                            + (proc.stderr or "")[:2000], encoding="utf-8")
            logger.warning("probe %s exited %d", name, proc.returncode)
            outcomes[name] = rec
            continue
        rec["outcome"] = OUTCOME_COMPLETED
        rec["lines"] = len((proc.stdout or "").splitlines())
        path.write_text(proc.stdout or "", encoding="utf-8")
        outcomes[name] = rec
    (collection / "outcomes.json").write_text(
        json.dumps(outcomes, indent=2, sort_keys=True), encoding="utf-8")
    done = sum(1 for r in outcomes.values() if r["outcome"] == OUTCOME_COMPLETED)
    logger.info("probe set: %d of %d completed, collection at %s",
                done, len(outcomes), collection)
    return outcomes


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


def probe_state(outcomes: Dict[str, Any], leg: int, max_legs: int,
                elapsed_s: float) -> str:
    """A ledger of what has happened, appended to each `continue`.

    WHY THE RUNNER CARRIES THIS. The agent's own record decays — observations
    cap once stored and whole legs leave the history window. The runner's does
    not, so the thing an auditor structurally cannot hold across a long
    engagement is exactly what the process driving it already knows.

    STATE, NOT GAPS. Everything here happened: legs taken, minutes spent, probes
    that completed and probes that did not. Nothing here says what remains, and
    nothing names a deliverable — handing the agent the stopping rule would feed
    the metric to the thing being measured.

    The failed probes are named because METHOD §1 makes them part of the record
    and §12 makes a repeatedly failing probe a finding in its own right. An
    auditor that cannot see which probes did not finish will write "no listener
    was found" where the truth is "the listener walk did not finish".
    """
    done = sorted(k for k, v in outcomes.items()
                  if v["outcome"] == OUTCOME_COMPLETED)
    bad = sorted(k for k, v in outcomes.items()
                 if v["outcome"] in (OUTCOME_TIMED_OUT, OUTCOME_UNAUTHORISED))
    head = (f"\n\n[engagement state, recorded by the client's process — "
            f"leg {leg} of {max_legs}, {elapsed_s / 60:.0f} min elapsed; "
            f"{len(done)} probes completed")
    if bad:
        detail = ", ".join(f"{k} ({outcomes[k]['outcome']})" for k in bad)
        head += f"; DID NOT COMPLETE: {detail}"
    return head + ".]"


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


def last_exit_reason(world: str, agent: str) -> Optional[str]:
    """Read it off the trace, not the loop. `exit_reason` is a local inside
    _process_user_turn and is never stored on the object — getattr for it
    returns None, which reads as "not a yield" and would end every run after
    one leg."""
    p = (REPO / "scenarios" / world / agent / "memory" /
         "reasoning_trace.jsonl")
    if not p.exists():
        return None
    last = None
    for line in p.open(errors="replace"):
        line = line.strip()
        if line:
            last = line
    if not last:
        return None
    try:
        return json.loads(last).get("exit_reason")
    except json.JSONDecodeError:
        return None


def latest_reply(loop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if str(t.get("name", "")) != source:
            return str(t.get("text", ""))
    return ""


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


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True,
                    help="engagement name under engagements/ — supplies the "
                         "hosts, the authorised probes, the brief, and where "
                         "runs land")
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

    # PHASE 1 — THE PROBE SET, BEFORE THE AUDIT STARTS. METHOD §12: the runner
    # executes it and the auditor reads the result, which is what makes "the
    # auditor touched nothing" a property of the harness rather than a rule a
    # model is asked to follow. The collection is written into the run
    # directory, so the evidence and the report it produced stay together.
    collection = out / "collection"
    outcomes = run_probe_set(collection, eng["probes"])
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
    if not collection.is_dir():
        raise SystemExit(f"engagement '{eng['name']}': the probe set "
                         f"produced no collection at {collection}")
    name, cfg = build_config(args.world, args.model, wf_mode,
                             args.temperature, args.max_tokens,
                             collection)
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
        "  probes authorised %s\n"
        "  autonomy_enabled  %s          (nothing fires on its own; legs are driven here)\n"
        "  workflow_mode     %s\n"
        "  suppressed        %s\n"
        "  omitted tools     %d\n"
        "  max legs          %d\n"
        "  react_max_tokens  %s\n"
        "=== end configuration; everything after this is a chat message ===",
        args.world, name, cfg.get("external_repo"), cfg.get("inspect_repo"),
        cfg.get("workflow"), eng["brief"], ", ".join(eng["probes"] or []),
        cfg.get("autonomy_enabled"), cfg.get("workflow_mode"),
        "; ".join(getattr(loop, "workflow_suppressed", []) or ["(none)"]),
        len(_chat.get("omitted_tools") or []), args.max_turns,
        _chat.get("react_max_tokens"))

    t0 = time.time()
    legs, error = [], None
    text_first_leg = ''
    # Blocks seen anywhere across the engagement, and how many times the runner
    # had to say one was missing. `blocks_prompted` is the metric that lets the
    # gates exist at all: gating every block means a model that forgets to
    # enumerate is told to, which would have turned campaign m1's three
    # claim-surface failures into successes and flattened the only criterion
    # that discriminated. Counting the prompts keeps the signal and makes it
    # finer than the pass/fail it replaces — "attack surface: 1 prompt" says
    # more than "criterion absent".
    delivered = {n: False for n in blocks.BLOCKS}
    prompted = {n: 0 for n in blocks.BLOCKS}
    # Spent when every block looks delivered but the agent yielded anyway.
    grace_leg_spent = False
    transcript = []
    undelivered = list(blocks.BLOCKS)
    try:
        text = eng["brief"].read_text(encoding='utf-8')
        text_first_leg = text
        sent_reason = "opening brief"
        for i in range(args.max_turns):
            log_outgoing(i + 1, sent_reason, text)
            concerns_before = concern_snapshot(loop)
            resources_before = resource_snapshot(loop)
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = latest_reply(loop, SOURCE)
            exit_reason = last_exit_reason(args.world, name)
            # Snapshotted the moment the turn returns. The yield spawn is
            # synchronous inside the turn, so the concern set is final here;
            # post-turn work on the executor touches claims and the trace,
            # not concerns.
            log_incoming(concern_log, i + 1, text, sent_reason, exit_reason,
                         len(reply), concerns_before, concern_snapshot(loop),
                         resources_before, resource_snapshot(loop))
            legs.append({"leg": i + 1, "sent": text[:80],
                         "exit_reason": exit_reason,
                         "reply_chars": len(reply)})
            logger.info("leg %d: exit=%s chars=%d", i + 1, exit_reason,
                        len(reply))
            # A turn that died is not a turn that finished. llm_error and
            # max_iters both end the loop, and treating them as ordinary
            # completion reported error=None on a run whose only output was
            # "[degraded reply — a model call failed mid-loop]". Silence
            # reading as success is the failure mode this whole suite exists
            # to catch; it should not be in the runner.
            if exit_reason in ("llm_error", "crashed"):
                error = f"turn {i + 1} ended {exit_reason} — run is not valid"
                break
            if exit_reason == "max_iters":
                error = (f"turn {i + 1} hit max_iters without answering — "
                         f"run is not valid")
                break

            # DELIVERY IS TESTED, NOT INFERRED. Every block the reply carries
            # counts as delivered, whatever leg it arrived in and however many
            # arrived together.
            transcript.append(reply)
            for n, seen in blocks.present(reply).items():
                delivered[n] = delivered[n] or seen
            legs[-1]["blocks"] = [n for n in blocks.BLOCKS
                                  if blocks.opened(reply, n)]
            undelivered = blocks.missing(delivered)
            # A YIELD IS A NOT-DONE SIGNAL, AND BELIEVING IT IS FREE. That
            # `respond` does not prove delivery is why this runner stopped
            # reading turn boundaries; the converse is not symmetric.
            # Believing "I am finished" can end a run with nothing written.
            # Believing "I am not finished" costs one leg. So the agent's own
            # signal is trusted in the cheap direction only, and a yield does
            # not end the engagement even when every block looks delivered.
            #
            # Observed 2026-08-29 on cs2_flashnext_med: a yield whose status
            # line named the three blocks it was about to write marked all
            # three delivered and broke this loop, and report.md was fifteen
            # words. Anchoring the marker stops that sentence from counting;
            # this stops the next spelling of it from ending a run, because a
            # model listing blocks it still owes is almost always yielding.
            #
            # ONE GRACE LEG, NOT UNBOUNDED. A model that always yields would
            # otherwise run to the cap and be filed no_deliverable having
            # delivered everything. The alternative — telling it "all four
            # received, respond if you are done" — is barred: engagement_state
            # carries what happened, never the stopping rule, and handing that
            # over feeds the metric to the thing being measured.
            if not undelivered and (exit_reason != "yield" or grace_leg_spent):
                break
            if not undelivered:
                grace_leg_spent = True
                logger.info("leg %d: every block delivered but the agent "
                            "yielded — granting one further leg", i + 1)

            # `exit_reason` DECIDES THE MESSAGE, NEVER THE ENDING. This is the
            # one job the yield/respond signal keeps, and it is a continuation
            # job: `yield` means the agent says it is still working, so the
            # runner does not interrupt it to name a block that is legitimately
            # still to come. `respond` means the agent believes it is finished
            # — so if a block is missing, that is the moment to say which, and
            # the moment worth counting. Nothing here ends the engagement; only
            # the blocks or the leg cap do.
            if exit_reason == "yield":
                text = CONTINUE + probe_state(
                    outcomes, i + 2, args.max_turns, time.time() - t0)
                sent_reason = "agent yielded — continue, no block named"
                continue
            nxt = undelivered[0]
            prompted[nxt] += 1
            logger.info("leg %d: %s not delivered — prompting (%d)",
                        i + 1, nxt, prompted[nxt])
            text = (CONTINUE + "\n\n" + blocks.rejection(nxt) + "\n"
                    + probe_state(outcomes, i + 2, args.max_turns,
                                  time.time() - t0))
            sent_reason = (f"agent responded but {nxt} not delivered "
                           f"— prompt #{prompted[nxt]}")
            continue
        else:
            # THE LEG CAP IS A TERMINAL STATE OF ITS OWN, and it names the
            # block it died on. "Never enumerated" and "produced a report and
            # no Gap Map" are different failures; collapsing them into one
            # outcome loses the only thing that distinguishes them.
            if undelivered:
                error = ("no_deliverable: " + ", ".join(undelivered)
                         + f" not delivered in {args.max_turns} legs")
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

    # Write the deliverables as files. run_meta.json carries the metadata;
    # without this the report exists only inside the trace's raw_response,
    # which is not somewhere a human reads a report from.
    #
    # ASSEMBLED FROM BLOCKS, ACROSS EVERY LEG (§16). Nothing is inferred from
    # which leg a reply arrived in — that inference is what filed a claim
    # enumeration as `b2_glm_2/report.md` on 2026-08-27. A block is taken from
    # wherever it was emitted, and a leg carrying several is not a special
    # case.
    #
    # report.md IS THE REPORT BLOCK PLUS THE LIMITATIONS BLOCK. They are two
    # blocks so that nothing has to nest, and one document because that is what
    # the client receives — ISAE 3000 / AT-C 205 require the limitations to
    # travel with the report, not beside it.
    whole = "\n\n".join(t for t in transcript if t)
    final = latest_reply(loop, SOURCE)
    if final:
        (out / "full_reply.md").write_text(final, encoding="utf-8")
    if whole:
        parts = [b for b in (blocks.span(whole, n)
                             for n in blocks.REPORT_BLOCKS) if b]
        if parts:
            (out / "report.md").write_text("\n\n".join(parts).rstrip() + "\n",
                                           encoding="utf-8")
        else:
            logger.warning("no REPORT block in the engagement — no report.md")
        # A SPAN, LIKE report.md. Writing this one stripped and the report one
        # whole made the two files disagree about the same fact: run_meta
        # recorded `GAP MAP` closed and conformance.json recorded it open,
        # because conformance looked for the closer in a file the runner had
        # just removed it from. Both deliverables are faithful copies.
        gap = blocks.span(whole, "GAP MAP")
        if gap:
            (out / "gap_map.md").write_text(gap.rstrip() + "\n", encoding="utf-8")
        else:
            logger.warning("no GAP MAP block in the engagement — no gap_map.md")

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
    # THE DELIVERED TEXT, NOT THE FILE. workflows/claims_audit/method/METHOD.md is not what the agent
    # read: the loader strips every section marked for the practice. A commit
    # hash would name the file and still not answer what reached the model, and
    # it needs the repository at that revision to resolve at all. A copy is
    # self-contained and is the artifact itself.
    record.mkdir(parents=True, exist_ok=True)
    try:
        from chat.workflow import load_workflow                # noqa: E402
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
        "harness_rev": git_rev(REPO),
        "files_read": files_read(record / "inspect_traces",
                                 Path(cfg.get("external_repo") or ".")),
        "legs": legs,
        # HOW MUCH THE HARNESS HAD TO CARRY. Gating every block means a model
        # that forgets to enumerate is told to, and campaign m1's three
        # claim-surface failures would have become successes — the only
        # criterion that discriminated, spent. This is what keeps the signal,
        # and it is finer than the pass/fail it replaces: "attack surface: 1
        # prompt" says more than "criterion absent". A model that emits
        # everything unprompted is zero across the board.
        "blocks_prompted": prompted,
        "blocks_delivered": delivered,
        "blocks_closed": {n: blocks.closed(whole, n) for n in blocks.BLOCKS},
        "wall_clock_s": wall,
        # Action emissions the token ceiling cut off. Non-zero means the
        # run was retried into shape rather than produced cleanly.
        "finish_length_events": getattr(loop, "finish_length_events", None),
        "error": error,
        "captured_at_utc": ts,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(legs)} legs, {wall}s, error={error}")
    print("blocks: " + ", ".join(
        f"{n}={'ok' if delivered[n] else 'MISSING'}"
        + (f"(+{prompted[n]} prompt{'s' if prompted[n] > 1 else ''})"
           if prompted[n] else "")
        for n in blocks.BLOCKS))
    print(f"deliverables: {out}/report.md, gap_map.md")
    print(f"meta: {out / 'run_meta.json'}")
    # --run, not --world: the run directory is the archive and survives the
    # world's deletion (§14). Scoring is fixture-only; a real engagement has no
    # answer key.
    # NO SCORER. The claims audit has a fixture with an answer key; a security
    # audit of a live host has no key, because what is true of the host is what
    # the collection says. Judgement is the review's, and a human's.
    print(f"collection:  {collection}  "
          f"({sum(1 for v in outcomes.values() if v['outcome'] == OUTCOME_COMPLETED)}"
          f" of {len(outcomes)} probes completed)")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
