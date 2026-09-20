"""The practice's page for the outreach workflow. Local only.

    python3 workflowsv2/outreach/app.py [--port 8810] [--model <yaml>]

The page keeps nothing of its own. What it shows comes from Attio (who is at
which stage, tasks due) and from the runner's records under prospects/ (the
brief, the evidence, the draft); what it writes goes to the same two places,
and only when a person presses a button. It sends no message to anyone: the
person copies the text into LinkedIn and then presses Sent.

It binds to 127.0.0.1 and has no login. It is not part of the client site.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from fastapi import FastAPI, HTTPException                              # noqa: E402
from fastapi.responses import FileResponse                              # noqa: E402
from pydantic import BaseModel                                          # noqa: E402

from workflowsv2.outreach import attio, runner, schemas                 # noqa: E402
from utils.file_utils import atomic_write_text                          # noqa: E402

logger = logging.getLogger("outreach.app")

MODEL = REPO / "measure/models/local_qwen38flashnext.yaml"
RUN_LOG = runner.DATA / "run.log"
#: The stages shown as cards, in this order.
SHOWN = ("Ready to contact", "Qualified", "Research")

app = FastAPI()
_run: Dict[str, Any] = {"proc": None, "what": None, "started": None}


def today() -> str:
    return datetime.date.today().isoformat()


def _local(name: str) -> Dict[str, Any]:
    """The runner's record of this person, where there is one."""
    d = runner.DATA / runner.slug(name)
    out: Dict[str, Any] = {}
    for key, f in (("qualification", "qualification.json"), ("draft", "draft.json")):
        if (d / f).is_file():
            out[key] = json.loads((d / f).read_text(encoding="utf-8"))
    if (d / "brief.md").is_file():
        out["brief"] = (d / "brief.md").read_text(encoding="utf-8")
    return out


def _text(entry: Dict[str, Any], slug: str) -> Optional[str]:
    return attio._first(entry, slug, "value")


@app.get("/")
def page() -> FileResponse:
    return FileResponse(HERE / "static" / "outreach.html")


@app.get("/api/queue")
def queue() -> Dict[str, Any]:
    try:
        entries = attio.entries()
        stages = attio.stages()
    except attio.AttioError as e:
        raise HTTPException(502, str(e))
    cards: List[Dict[str, Any]] = []
    counts: Dict[str, int] = {}
    for e in entries:
        stage = attio.stage_of(e)
        counts[stage] = counts.get(stage, 0) + 1
        if stage not in SHOWN:
            continue
        later = _text(e, "next_action_date")
        if stage == "Ready to contact" and later and later > today():
            counts["(put off)"] = counts.get("(put off)", 0) + 1
            continue
        person = attio._call("GET", f"/objects/people/records/{e['parent_record_id']}")["data"]
        name = attio._first(person, "name", "full_name") or "(no name)"
        local = _local(name)
        draft = local.get("draft") or {}
        option = attio._first(e, "category", "option")
        cards.append({"record_id": e["parent_record_id"], "name": name, "stage": stage,
                      "title": attio._first(person, "job_title", "value"),
                      "linkedin": attio._first(person, "linkedin", "value"),
                      "web_url": person.get("web_url"),
                      "category": option.get("title") if isinstance(option, dict) else None,
                      "fit_rationale": _text(e, "fit_rationale"),
                      "message": draft.get("edited_message") or draft.get("message"),
                      "flags": (local.get("qualification") or {}).get("flags", []) + draft.get("flags", []),
                      "brief": local.get("brief")})
    cards.sort(key=lambda c: SHOWN.index(c["stage"]))
    return {"cards": cards, "counts": counts, "has_not_pursuing": "Not pursuing" in stages,
            "kinds": [k for k in schemas.TYPES if k != "none"]}


class Sent(BaseModel):
    record_id: str
    name: str
    message: str


@app.post("/api/sent")
def sent(body: Sent) -> Dict[str, Any]:
    """The person has sent the message by hand. Record the exact text and move
    the entry to Initial sent."""
    try:
        attio.create_note(body.record_id, f"Message sent {today()}", body.message)
        attio.upsert_entry(body.record_id, {"stage": "Initial sent", "last_contact": today(),
                                            "next_action": "Follow up if no response",
                                            "next_action_date": (datetime.date.today()
                                                                 + datetime.timedelta(days=6)).isoformat()})
    except attio.AttioError as e:
        raise HTTPException(502, str(e))
    return {"ok": True}


class Skip(BaseModel):
    record_id: str
    reason: str


@app.post("/api/skip")
def skip(body: Skip) -> Dict[str, Any]:
    try:
        if "Not pursuing" not in attio.stages():
            raise HTTPException(409, "the outreach list has no stage 'Not pursuing'; add it in Attio")
        attio.upsert_entry(body.record_id, {"stage": "Not pursuing", "next_action": "",
                                            "fit_rationale": f"Skipped {today()}: {body.reason.strip()}"})
    except attio.AttioError as e:
        raise HTTPException(502, str(e))
    return {"ok": True}


class Later(BaseModel):
    record_id: str
    until: str


@app.post("/api/later")
def later(body: Later) -> Dict[str, Any]:
    try:
        attio.upsert_entry(body.record_id, {"next_action_date": body.until})
    except attio.AttioError as e:
        raise HTTPException(502, str(e))
    return {"ok": True}


class Edit(BaseModel):
    name: str
    message: str


@app.post("/api/edit")
def edit(body: Edit) -> Dict[str, Any]:
    """Keep an edited message in the runner's draft record, beside the draft
    the model wrote."""
    f = runner.DATA / runner.slug(body.name) / "draft.json"
    if not f.is_file():
        raise HTTPException(404, "no draft on record for this person")
    d = json.loads(f.read_text(encoding="utf-8"))
    d["edited_message"], d["edited_at"] = body.message, today()
    atomic_write_text(f, json.dumps(d, indent=1, ensure_ascii=False) + "\n")
    return {"ok": True}


class Add(BaseModel):
    name: str
    title: str = ""
    linkedin: str = ""
    how_known: str = ""
    pasted: str = ""
    pasted_source: str = ""


@app.post("/api/add")
def add(body: Add) -> Dict[str, Any]:
    """Hand a name to the workflow: the person goes into the outreach list at
    Research, and the next run works on them."""
    name = body.name.strip()
    if not name:
        raise HTTPException(422, "a name is needed")
    try:
        person = attio.find_person(name) or attio.create_person(name, body.title.strip(), body.linkedin.strip())
        rid = person["id"]["record_id"]
        entry = attio.entry_of(rid)
        if entry is not None and attio.stage_of(entry) != "Research":
            raise HTTPException(409, f"{name} is already in the list at '{attio.stage_of(entry)}'")
        values: Dict[str, Any] = {"stage": "Research"}
        if body.how_known.strip():
            values["notes"] = body.how_known.strip()
        attio.upsert_entry(rid, values)
        if body.pasted.strip():
            attio.create_note(rid, f"{attio.EVIDENCE_NOTE}: {body.pasted_source.strip() or 'pasted text'}",
                              body.pasted.strip())
    except attio.AttioError as e:
        raise HTTPException(502, str(e))
    return {"ok": True, "record_id": rid}


class Run(BaseModel):
    what: str                          # "research" or "scout"
    kind: str = ""
    want: int = 5


@app.post("/api/run")
def run(body: Run) -> Dict[str, Any]:
    """Start the runner, then the push, as one background job. One at a time."""
    if _run["proc"] is not None and _run["proc"].poll() is None:
        raise HTTPException(409, f"a run is going: {_run['what']}")
    py, script = sys.executable, str(HERE / "runner.py")
    if body.what == "scout":
        if body.kind not in schemas.TYPES or body.kind == "none":
            raise HTTPException(422, "choose a kind to scout for")
        first = [py, script, "scout", "--kind", body.kind, "--want", str(body.want), "--attio", "--model", str(MODEL)]
        what = f"scout for {body.kind}"
    elif body.what == "research":
        first = [py, script, "run", "--attio", "--model", str(MODEL)]
        what = "research the names at Research"
    else:
        raise HTTPException(422, "what must be research or scout")
    runner.DATA.mkdir(parents=True, exist_ok=True)
    push = [py, script, "push"] + (["--scouted"] if body.what == "scout" else [])
    cmd = " && ".join(" ".join(f"'{a}'" for a in argv) for argv in (first, push))
    out = open(RUN_LOG, "w", encoding="utf-8")
    _run.update(proc=subprocess.Popen(["bash", "-c", cmd], cwd=str(REPO), stdout=out, stderr=subprocess.STDOUT),
                what=what, started=datetime.datetime.now().isoformat(timespec="seconds"))
    return {"ok": True, "what": what}


@app.get("/api/run")
def run_status() -> Dict[str, Any]:
    proc = _run["proc"]
    lines: List[str] = []
    if RUN_LOG.is_file():
        lines = [x for x in RUN_LOG.read_text(encoding="utf-8", errors="replace").splitlines()
                 if " outreach " in x or x.startswith(("#", "- **"))][-12:]
    return {"running": proc is not None and proc.poll() is None, "what": _run["what"],
            "started": _run["started"], "exit": None if proc is None else proc.poll(), "log": lines}


def main() -> int:
    global MODEL
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8810)
    ap.add_argument("--model", type=Path, default=MODEL)
    args = ap.parse_args()
    MODEL = args.model
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
