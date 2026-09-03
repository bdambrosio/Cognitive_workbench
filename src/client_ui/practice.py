#!/usr/bin/env python3
"""The practice's start page: every engagement, its intakes and runs, which
are current, and the commands that drive them.

    python3 src/client_ui/practice.py [--port 8802] [--token <t>]

NOT THE CLIENT'S PAGE. This is the practice pane: it reads and changes the
engagement state (workflowsv2/engagement_state.py — current intake, current
run, cancel, new engagement) and shows the launch commands for the intake,
the chain and the post-delivery page. It launches nothing: an intake, a run
or a conversation starts from the command line, by a person. A page that
could start a two-hour audit from a button would be the wrong page.

Same token rule as the client page, same static look.
"""
from __future__ import annotations

import argparse
import logging
import os
import re
import secrets
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from fastapi import FastAPI, HTTPException                      # noqa: E402
from fastapi.responses import FileResponse, JSONResponse         # noqa: E402
from fastapi.staticfiles import StaticFiles                      # noqa: E402
from pydantic import BaseModel                                   # noqa: E402

from workflowsv2 import engagement_state as state               # noqa: E402

logger = logging.getLogger("client_ui.practice")
STATIC = HERE / "static"
MODEL = "measure/models/fw_glm53flash.yaml"
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


def commands(name: str, s: Dict[str, Any]) -> Dict[str, str]:
    """The command lines for this engagement, with the current run filled
    in where one exists. Shown to be copied, never run from here."""
    ci = s.get("current_intake")
    cur = None
    for i in s.get("intakes") or []:
        if i["id"] == ci:
            cur = next((r["name"] for r in i["runs"] if r["current"]), None)
    eng = f"workflowsv2/claims_audit/engagements/{name}"
    return {
        "intake": f"python3 src/client_ui/app.py intake --engagement {name} --model {MODEL} --port 8800",
        "intake_new": f"python3 src/client_ui/app.py intake --engagement {name} --new --model {MODEL} --port 8800",
        "finish": f"python3 workflowsv2/intake/runner.py --engagement {name} --finish",
        "audit": f"python3 workflowsv2/claims_audit/runner.py --engagement {name} --world <fresh world> --claim-source <one of claim_sources> --model {MODEL}",
        "review": f"python3 workflowsv2/audit_review/runner.py --run {eng}/runs/<run dir> --model {MODEL}",
        "materiality": f"python3 workflowsv2/audit_materiality/runner.py --engagement {name} --run {eng}/runs/<run dir> [--run ...] --model {MODEL} --label <label>",
        "report": f"python3 workflowsv2/audit_report/runner.py --merged {eng}/merged/{cur or '<merged dir>'} --model {MODEL}",
        "post": f"python3 src/client_ui/app.py post --engagement {name} --model {MODEL} --port 8801",
    }


class NewEngagement(BaseModel):
    name: str
    clone: Optional[str] = None


class Choice(BaseModel):
    id: str


def make_practice_app(token: str) -> FastAPI:
    app = FastAPI()
    app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")

    def _check(t: Optional[str]) -> None:
        if not t or not secrets.compare_digest(t, token):
            raise HTTPException(status_code=403, detail="token required")

    def _eng(name: str) -> Path:
        if not _NAME.match(name or ""):
            raise HTTPException(status_code=400, detail="bad engagement name")
        d = state.ENGAGEMENTS / name
        if not d.is_dir():
            raise HTTPException(status_code=404, detail=f"no engagement '{name}'")
        return d

    def _all() -> List[Dict[str, Any]]:
        out = []
        for n in state.engagements():
            s = state.summary(state.ENGAGEMENTS / n)
            s["commands"] = commands(n, s)
            out.append(s)
        return out

    def _act(fn, *args):
        try:
            fn(*args)
        except SystemExit as e:
            raise HTTPException(status_code=400, detail=str(e))
        return JSONResponse(_all())

    @app.get("/")
    async def index(token: Optional[str] = None):
        _check(token)
        return FileResponse(STATIC / "practice.html")

    @app.get("/api/engagements")
    async def list_engagements(token: Optional[str] = None):
        _check(token)
        return JSONResponse(_all())

    @app.post("/api/engagements")
    async def new_engagement(body: NewEngagement, token: Optional[str] = None):
        _check(token)
        if not _NAME.match(body.name or ""):
            raise HTTPException(status_code=400, detail="bad engagement name")
        return _act(state.new_engagement, state.ENGAGEMENTS / body.name, body.clone or None)

    @app.post("/api/engagements/{name}/intake/current")
    async def intake_current(name: str, body: Choice, token: Optional[str] = None):
        _check(token)
        return _act(state.set_current_intake, _eng(name), body.id)

    @app.post("/api/engagements/{name}/intake/cancel")
    async def intake_cancel(name: str, body: Choice, token: Optional[str] = None):
        _check(token)
        return _act(state.cancel_intake, _eng(name), body.id)

    @app.post("/api/engagements/{name}/run/current")
    async def run_current(name: str, body: Choice, token: Optional[str] = None):
        _check(token)
        return _act(state.set_current_run, _eng(name), body.id)

    @app.post("/api/engagements/{name}/run/cancel")
    async def run_cancel(name: str, body: Choice, token: Optional[str] = None):
        _check(token)
        return _act(state.cancel_run, _eng(name), body.id)

    return app


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8802)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--token", default=None,
                    help="shared secret (default: $CLIENT_UI_TOKEN, else generated)")
    args = ap.parse_args(argv)
    token = args.token or os.environ.get("CLIENT_UI_TOKEN") or secrets.token_urlsafe(16)
    app = make_practice_app(token)
    print(f"\npractice page — {len(state.engagements())} engagement(s)")
    print(f"open:  http://{args.host}:{args.port}/?token={token}\n")
    import uvicorn                                             # noqa: E402
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
