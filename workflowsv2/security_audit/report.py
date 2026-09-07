#!/usr/bin/env python3
"""The security audit's document, assembled from the record. Mechanical,
plus one model call for the passages between the parts.

    python3 workflowsv2/security_audit/report.py --run <run dir> \\
            [--model measure/models/local_qwen38flashnext.yaml] [--no-prose] [--rerender]

THE DOCUMENT IS A VIEW OVER THE RECORD. Every finding, citation, label,
count and the conclusion are copied or computed from `findings.json`,
`surface.json` and `collection/outcomes.json`. The model writes five
passages under REPORT.md and nothing else; without them the document still
assembles, with a `[[slot]]` marker where each would go, and that skeleton
is what the model is shown.

THE READER IS NAMED, and the order serves them (METHOD §9): the person who
will change the system reads findings by disposition and, within a
disposition, by what an attacker reaches first (§4); the person deciding
what to change first reads the conclusion (§10, computed) and the change
since the previous run (§8, computed by element identity).
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml

HERE = Path(__file__).resolve().parent          # workflowsv2/security_audit
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.security_audit import record                   # noqa: E402
from workflowsv2.emit import emit                                # noqa: E402
from workflowsv2 import issues                                   # noqa: E402

logger = logging.getLogger("security.report")
SCENARIO = HERE / "report.yaml"
METHOD_PATH = "workflowsv2/security_audit/method/REPORT.md"
STAGE = "security_report"

#: REPORT.md §5 and §7, in document order.
FIELDS: Tuple[str, ...] = ("summary", "change_note", "findings_note",
                           "gaps_note", "limitations")
#: Written only when the document has the section; empty otherwise.
CONDITIONAL: Tuple[str, ...] = ("change_note",)

DISPOSITION_WORDS = {
    "confirmed": "confirmed — the cited path runs from the element to the "
                 "consequence, and nothing in the collection prevents it",
    "mitigated": "mitigated — the path exists and a control shown in the "
                 "collection prevents the consequence",
    "unreachable": "unreachable — the weak state is present and no enumerated "
                   "element reaches it",
    "uncertain": "uncertain — the path cannot be settled from the collection; "
                 "the gap map says what would",
}
REACH_WORDS = {
    "beyond_lan": "reachable from outside the LAN",
    "lan_unauthenticated": "reachable from the LAN without credentials",
    "authenticated": "behind an authentication or privilege boundary",
    "at_rest": "a credential or secret at rest",
    "outbound": "an outbound trust",
    "other": "elsewhere on the frozen surface",
}
_ORDER_D = {d: i for i, d in enumerate(("confirmed", "mitigated", "uncertain", "unreachable"))}
_ORDER_R = {r: i for i, r in enumerate(record.REACH)}


# ---- helpers -------------------------------------------------------------------

def _md(text: Any) -> str:
    s = " ".join(str(text or "").split())
    return s.replace("```", "\\`\\`\\`").replace("|", "\\|")


def _cite(c: Optional[Dict[str, Any]]) -> str:
    if not isinstance(c, dict):
        return ""
    return f"`collection/{c.get('artifact')}:{record._lines(c.get('lines'))}`"


def _step(s: Optional[Dict[str, Any]]) -> str:
    if not isinstance(s, dict):
        return ""
    c = s.get("cite") or {}
    q = _md(c.get("quote"))
    return f"{_cite(c)}: \"{q}\" — {_md(s.get('text'))}" if q else _md(s.get("text"))


def _slot(name: str, prose: Optional[Dict[str, Any]]) -> List[str]:
    text = ((prose or {}).get(name) or "").strip()
    return [text, ""] if text else [f"[[{name}]]", ""]


def _by_label(frozen: Sequence[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {e["label"]: e for e in frozen}


def _ordered(findings: Sequence[Dict[str, Any]], frozen: Sequence[Dict[str, Any]]
             ) -> List[Dict[str, Any]]:
    by = _by_label(frozen)
    return sorted(findings, key=lambda f: (
        _ORDER_D.get(f.get("disposition"), 9),
        _ORDER_R.get((by.get(f.get("element")) or {}).get("reach"), 9),
        str(f.get("element"))))


# ---- change since the previous run (METHOD §8) --------------------------------

def change_since(run: Dict[str, Any], prev: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Computed by element identity, never by label. A finding present last
    time and absent now is a candidate for `[uncertain]`, not `resolved`:
    the method says a finding that has merely stopped being visible is not
    resolved."""
    if not prev:
        return None
    key = record.identity_key
    now_ids = {key(e): e for e in run["frozen"]}
    old_ids = {key(e): e for e in prev["frozen"]}
    new_elements = [e for i, e in now_ids.items() if i not in old_ids]
    gone_elements = [e for i, e in old_ids.items() if i not in now_ids]
    old_by = _by_label(prev["frozen"])
    now_by = _by_label(run["frozen"])
    old_f = {key(old_by.get(f.get("element")) or {}): f
             for f in prev["findings"].get("findings") or []}
    now_f = {key(now_by.get(f.get("element")) or {}): f
             for f in run["findings"].get("findings") or []}
    resolved_candidates = [(old_ids[i].get("identity"), f) for i, f in old_f.items()
                           if i not in now_f and i in now_ids]
    persistent_gaps = sorted(
        k for k, v in run["outcomes"].items()
        if v.get("outcome") in ("timed out", "unauthorised")
        and (prev["outcomes"].get(k) or {}).get("outcome") == v.get("outcome"))
    return {"previous": prev["dir"].name, "new_elements": new_elements,
            "gone_elements": gone_elements,
            "resolved_candidates": resolved_candidates,
            "persistent_gaps": persistent_gaps}


# ---- the document ----------------------------------------------------------------

def assemble(run: Dict[str, Any], prose: Optional[Dict[str, Any]] = None,
             prev: Optional[Dict[str, Any]] = None) -> str:
    frozen = run["frozen"]
    fs = run["findings"].get("findings") or []
    gaps = run["findings"].get("gaps") or []
    lims = run["findings"].get("limitations") or []
    examined = set(run["findings"].get("examined") or [])
    outcomes = run["outcomes"]
    concl = run["conclusion"]
    by = _by_label(frozen)
    meta = run["meta"]
    host = ", ".join(meta.get("hosts") or []) or "the host named by the engagement"
    when = (meta.get("captured_at_utc") or run["dir"].name.split("_", 1)[0])
    out: List[str] = []

    out += [f"# Security review — {host}", "",
            f"Collection taken {when}. Run `{run['dir'].name}`.", "",
            "**What this document is.** A security review of a running system: "
            "the ways in were enumerated from a fixed, read-only set of probes, "
            "and each one examined against what those probes recorded. The "
            "*collection* is the probes' output and is the only evidence; every "
            "finding cites a file in it by name and line. The review changed "
            "nothing, connected to nothing, and reasoned about no host the "
            "engagement did not name.", "",
            "**The assurance is limited.** The review examined part of the "
            "attack surface, in the order an attacker reaches it, and says "
            "where it stopped. An element not examined is not an element that "
            "is safe, and a state the collection did not capture is not a state "
            "that is absent.", ""]

    out += ["## Probes", "", "| probe | outcome | note |", "|---|---|---|"]
    for k in sorted(outcomes):
        v = outcomes[k]
        out.append(f"| {k} | {v.get('outcome')} | {_md(v.get('why') or '')} |")
    out += [""]

    out += ["## Conclusion", "",
            f"**{concl['conclusion']}.**"]
    if concl["conclusion"] == "Exposed":
        out += [f"{concl['confirmed_without_credentials']} confirmed finding(s) are "
                "reachable without credentials."]
    elif concl["conclusion"] == "Weak":
        out += [f"{concl['confirmed']} confirmed finding(s), every one behind an "
                "authentication or privilege boundary."]
    else:
        out += ["No confirmed finding among the elements examined."]
    if concl["withheld_grants"]:
        out += ["", "**A grant that was never given cannot be read as a clean "
                "result.** These probes returned unauthorised, so the conclusion "
                "does not cover what they would have shown: "
                + ", ".join(f"`{k}`" for k in concl["withheld_grants"]) + "."]
    out += [""] + _slot("summary", prose)

    delta = change_since(run, prev)
    if delta:
        out += ["## Change since the previous review", "",
                f"Compared with run `{delta['previous']}`, by element identity.", ""]
        out += ["**Elements enumerated this time and not last time:** " + (", ".join(
            f"`{e.get('identity')}`" for e in delta["new_elements"]) or "none") + "."]
        out += ["**Elements enumerated last time and not this time:** " + (", ".join(
            f"`{e.get('identity')}`" for e in delta["gone_elements"]) or "none")
            + (". An element missing from this enumeration may still be on the host; "
               "two enumerations of one collection differ, and this list is the "
               "difference, not a change on the host." if delta["gone_elements"] else ".")]
        rc = delta["resolved_candidates"]
        out += ["**Findings the previous review carried that this collection does not "
                "support:** " + (", ".join(f"`{i}` ({_md(f.get('title'))})" for i, f in rc)
                                 or "none")
                + (". A finding that has stopped being visible is not resolved; "
                   "each is listed as uncertain until an observation retires it."
                   if rc else ".")]
        out += ["**Persistent gaps:** " + (", ".join(f"`{k}`" for k in delta["persistent_gaps"])
                                          or "none")
                + (" — the same probe did not complete on both reviews." if delta["persistent_gaps"] else ".")]
        out += [""] + _slot("change_note", prose)

    out += ["## How to read a finding", "",
            "Each finding starts from one element of the frozen surface, named "
            "by its label and its identity, and traces a *path* from what is "
            "reachable to what reaching it permits. Every step cites a collection "
            "file and quotes the line. A finding carries one *disposition*:", "",
            "| disposition | meaning |", "|---|---|"]
    out += [f"| {d} | {w.split(' — ', 1)[1]} |" for d, w in DISPOSITION_WORDS.items()]
    out += ["", "Severity is not assigned. It depends on what the host holds and "
            "who the adversary is, which the collection does not show; the "
            "reader who knows the deployment assigns it. *Remedy locus* names "
            "the file, unit or rule to change and stops there.", ""]

    out += ["## Findings", ""] + _slot("findings_note", prose)
    if not fs:
        out += ["No finding was recorded.", ""]
    for f in _ordered(fs, frozen):
        e = by.get(f.get("element")) or {}
        out += [f"### {f.get('element')} — {_md(f.get('title'))} — [{f.get('disposition')}]", "",
                f"Element {f.get('element')}: `{_md(e.get('identity'))}`, "
                f"{REACH_WORDS.get(e.get('reach'), e.get('reach') or '')}. "
                f"{DISPOSITION_WORDS.get(f.get('disposition'), '').capitalize()}.", "",
                f"**Exposure.** {_step(f.get('exposure'))}", ""]
        if f.get("path"):
            out += ["**Path.**", ""] + [f"- {_step(s)}" for s in f["path"]] + [""]
        out += [f"**Consequence.** {_step(f.get('consequence'))}", "",
                f"**Assessment.** {_md(f.get('assessment'))}", ""]
        if f.get("remedy_locus"):
            out += [f"**Remedy locus.** {_md(f['remedy_locus'])}", ""]
        probs = f.get("citation_problems") or []
        loose = f.get("citation_loose") or []
        line = ("every citation resolves into the collection" if not probs else
                f"{len(probs)} citation(s) did not resolve — " + "; ".join(_md(p) for p in probs))
        if loose:
            line += (f"; {len(loose)} quote(s) differ from the cited line in punctuation "
                     f"or spacing ({', '.join(loose)})")
        moved = f.get("citation_relocated") or []
        if moved:
            line += (f"; {len(moved)} citation(s) named the wrong lines and were corrected "
                     f"to where the quoted text is ({', '.join(moved)})")
        out += ["Citations: " + line + ".", ""]

    out += ["## The attack surface", "",
            "The frozen enumeration: every way in the collection shows, labelled "
            "in the order found. *Examined* means traced from exposure to "
            "consequence; an element not examined carries no statement.", "",
            "| label | kind | identity | reach | cite | examined | disposition |",
            "|---|---|---|---|---|---|---|"]
    disp = {f.get("element"): f.get("disposition") for f in fs}
    for e in frozen:
        out.append(f"| {e['label']} | {e.get('kind')} | `{_md(e.get('identity'))}` | "
                   f"{e.get('reach')} | {_cite(e.get('cite'))} | "
                   f"{'yes' if e['label'] in examined or e['label'] in disp else 'no'} | "
                   f"{disp.get(e['label'], '')} |")
    out += ["", f"{len(frozen)} element(s); {len(examined | set(disp))} examined; "
            f"{len(fs)} finding(s).", ""]

    out += ["## Gap map", ""] + _slot("gaps_note", prose)
    if not gaps:
        out += ["Nothing the review could not settle was recorded.", ""]
    for i, g in enumerate(gaps, 1):
        out += [f"{i}. **{_md(g.get('unknown'))}**"
                + (f" ({g['element']})" if g.get("element") else "")
                + f" — {_md(g.get('why'))} *Settled by:* `{' '.join(str(g.get('settles') or '').split())}`"]
    out += [""]

    out += ["## Limitations", ""] + _slot("limitations", prose)
    for lim in lims:
        out.append(f"- {_md(lim.get('text'))}" + (f" ({_cite(lim['cite'])})" if lim.get("cite") else ""))
    out += ["- The collection is a point in time; every observation carries the "
            "date it was taken and the review says nothing about the host since.",
            "- The review did not run, connect to, or alter anything, and did not "
            "examine any host the engagement did not name.",
            "- No exploit material, no severity, no judgement on advisories the "
            "system cannot yet act on, no user data.", ""]
    return "\n".join(out).rstrip() + "\n"


# ---- the prose call ------------------------------------------------------------------

def prose_schema() -> Dict[str, Any]:
    return {"type": "object",
            "properties": {f: {"type": "string"} for f in FIELDS},
            "required": list(FIELDS)}


def check_prose(obj: Dict[str, Any], run: Dict[str, Any], has_change: bool) -> Dict[str, Any]:
    import re
    problems: List[str] = []
    labels = {e["label"] for e in run["frozen"]}
    for f in FIELDS:
        text = obj.get(f)
        if not isinstance(text, str):
            problems.append(f"{f}: missing")
            continue
        if not text.strip():
            if f in CONDITIONAL and not has_change:
                continue
            problems.append(f"{f}: empty (REPORT.md §5)")
            continue
        if f in CONDITIONAL and not has_change:
            problems.append(f"{f}: written, and the document has no such section")
        if re.search(r"(?m)^\s*===", text):
            problems.append(f"{f}: carries a `===` marker line")
        for m in re.finditer(r"\bS(\d+)\b", text):
            if f"S{m.group(1)}" not in labels:
                problems.append(f"{f}: names S{m.group(1)}, which is not on the surface")
    return {"ok": not problems, "problems": problems}


def build_config(out: Path, world: str, model_path: Optional[Path]) -> Tuple[str, Dict[str, Any]]:
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
    cfg["external_repo"] = str(out)
    cfg["inspect_repo"] = str(out)
    return name, cfg


def write_prose(loop, method_text: str, skeleton: str, max_tokens: int) -> Dict[str, Any]:
    user = ("The document, without your passages. Each `[[field]]` marker is "
            "where that passage will be placed:\n\n" + skeleton
            + "\n\nWrite the passages, per REPORT.md §5, and emit them under "
              "the fields of §7.")
    return emit(loop, method_text, user, prose_schema(), max_tokens)


def run(out: Path, model_path: Optional[Path], logger=logger,
        rerender: bool = False) -> Path:
    """Assemble the skeleton; with a model, write the prose and assemble
    again. Returns the path of report.md."""
    out = Path(out)
    rec = record.load_run(out)
    prev_dir = record.previous_run(out)
    prev = record.load_run(prev_dir) if prev_dir else None
    skeleton = assemble(rec, None, prev)
    (out / "report_skeleton.md").write_text(skeleton, encoding="utf-8")
    prose: Optional[Dict[str, Any]] = None
    if rerender and (out / "prose.json").is_file():
        prose = json.loads((out / "prose.json").read_text(encoding="utf-8"))
    elif model_path:
        from chat.chat_loop import ChatLoop                    # noqa: E402
        from chat.workflow import load_workflow                # noqa: E402
        ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
        world = f"secreport_{ts}_{out.name}"[:60]
        name, cfg = build_config(out, world, model_path)
        loop = ChatLoop(character_name=name, character_config=cfg)
        try:
            method_text = load_workflow(REPO / METHOD_PATH)
            max_tokens = int((cfg.get("chat") or {}).get("react_max_tokens", 32768))
            call = write_prose(loop, method_text, skeleton, max_tokens)
        finally:
            try:
                loop._post_turn_executor.shutdown(wait=True)
            except Exception as e:                             # noqa: BLE001
                logger.warning("executor shutdown failed: %s", e)
        (out / "report_meta.json").write_text(json.dumps(
            {"world": world, "model_config": str(model_path),
             "call": {k: v for k, v in call.items() if k not in ("raw", "obj")}},
            indent=2, default=str) + "\n", encoding="utf-8")
        if call.get("response_format_dropped"):
            logger.error("the route dropped the response schema — no prose")
        elif call.get("obj") is None:
            logger.error("prose did not parse: %s", call.get("parse_error"))
        else:
            prose = call["obj"]
            chk = check_prose(prose, rec, prev is not None)
            for prob in chk["problems"]:
                issues.note(out, stage=STAGE, code="prose_check", text=prob, severity="check")
            (out / "prose.json").write_text(json.dumps(prose, indent=1, ensure_ascii=False) + "\n",
                                            encoding="utf-8")
            logger.info("prose: %d field(s), %d check problem(s)", len(prose), len(chk["problems"]))
    doc = assemble(rec, prose, prev)
    (out / "report.md").write_text(doc, encoding="utf-8")
    logger.info("report: %s (%s)", out / "report.md",
                "with prose" if prose else "skeleton only")
    return out / "report.md"


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True, help="a run directory holding findings.json")
    ap.add_argument("--model", type=Path, default=None)
    ap.add_argument("--no-prose", action="store_true")
    ap.add_argument("--rerender", action="store_true",
                    help="assemble again from the existing prose.json, no model call")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if not (args.run / "findings.json").is_file():
        raise SystemExit(f"{args.run}: no findings.json")
    p = run(args.run, None if (args.no_prose or args.rerender) else args.model,
            rerender=args.rerender)
    print(f"out: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
