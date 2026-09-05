#!/usr/bin/env python3
"""The client site: one process, every engagement, the client and the
practice through a whole engagement.

    python3 src/client_ui/site.py --model measure/models/fw_glm53flash.yaml [--port 8803]
                                  [--no-access]       # loopback only: identity from ?as=<email>

WHO SEES WHAT. Identity comes from Cloudflare Access (src/client_ui/access.py):
a practice email opens every engagement and the practice pages under /p/;
a client email opens the engagements whose `client_emails` name it, under
/e/<engagement>/. Nothing else answers.

THE STAGES. A client's engagement home lists the ten stages of the
engagement (workflowsv2/engagement_state.STAGES) with the current one
marked and one line saying what is waiting on whom. The practice page has
the buttons: materials ready, enumerate, freeze the surface, run the
chain, release, close. A job runs as a subprocess (src/client_ui/jobs.py),
one at a time per engagement; the practice is mailed when it ends.

THE CONVERSATIONS. The intake and the post-delivery page are the same
sessions the terminal runners drive (IntakeSession, PostSession), one live
per (kind, engagement), built on demand and evicted least-recently-used
(src/client_ui/registry.py). The chat page (static/index.html, client.js)
is mounted twice, under /e/<e>/intake/ and /e/<e>/report/.

THE SURFACE. After enumeration the client reads the claim surface and
comments on any claim; the practice edits and freezes it. The frozen file
is in claims.json shape, which is what the audit runner's --surface reads.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import re
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from fastapi import FastAPI, File, HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect  # noqa: E402
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, PlainTextResponse, RedirectResponse  # noqa: E402
from fastapi.staticfiles import StaticFiles                                       # noqa: E402
from pydantic import BaseModel                                                    # noqa: E402

from workflowsv2 import engagement_state as state               # noqa: E402
from client_ui import cf_access, jobs, mail                     # noqa: E402
from client_ui.access import Access, LOCAL_COOKIE               # noqa: E402
from client_ui.app import _announce, _SAFE                      # noqa: E402
from client_ui.registry import Registry                         # noqa: E402

logger = logging.getLogger("client_ui.site")
STATIC = HERE / "static"
LETTER_TEMPLATE = state.ENGAGEMENTS / "LETTER_TEMPLATE.md"
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


# ---- the stages, as the pages describe them ----------------------------------

def next_step(eng_dir: Path) -> Dict[str, str]:
    """The stage the engagement is at, whose move it is, and one line
    saying so. `who` is "client", "practice" or "done"."""
    v = lambda s: state.stage_value(eng_dir, s)                # noqa: E731
    if v("closed") == "closed":
        return {"stage": "closed", "who": "done", "text": "The engagement is closed."}
    if v("release") == "released":
        return {"stage": "report", "who": "client",
                "text": "The report is ready. Read it and ask it anything."}
    if v("letter") != "accepted":
        return {"stage": "letter", "who": "client",
                "text": "Read the engagement letter and accept it to begin."}
    if v("intake") != "done":
        return {"stage": "intake", "who": "client",
                "text": "Complete the intake conversation, then finish it."}
    if v("materials") != "ready":
        return {"stage": "materials", "who": "practice",
                "text": "The practice is obtaining the materials named at intake."}
    if v("enumeration") == "running":
        return {"stage": "enumeration", "who": "practice",
                "text": "The claims are being enumerated from the documents you named."}
    if v("enumeration") != "done":
        return {"stage": "enumeration", "who": "practice",
                "text": "The practice will enumerate the claims next."}
    if v("surface") != "frozen":
        return {"stage": "surface", "who": "client",
                "text": "Read the claim surface and comment on any claim that is "
                        "wrong, missing or beside the point. The practice then freezes it."}
    if v("chain") == "running":
        return {"stage": "chain", "who": "practice",
                "text": "The review is running: every claim is being tested, checked and rated."}
    if v("chain") != "done":
        return {"stage": "chain", "who": "practice",
                "text": "The practice will run the review next."}
    return {"stage": "release", "who": "practice",
            "text": "The practice is reading the report before it is released."}


def letter_text(eng_dir: Path) -> str:
    own = eng_dir / "letter.md"
    p = own if own.is_file() else LETTER_TEMPLATE
    return p.read_text(encoding="utf-8") if p.is_file() else "(no engagement letter on file)"


def status_for(eng_dir: Path, role: str) -> Dict[str, Any]:
    s = state.summary(eng_dir)
    cur = state.current_run(eng_dir, s["current_intake"])
    return {"name": eng_dir.name, "role": role, "stages": s["stages"],
            "stage_order": list(state.STAGES), "next": next_step(eng_dir),
            "job": s["job"], "letter": letter_text(eng_dir),
            "released": state.stage_value(eng_dir, "release") == "released",
            "report_exists": bool(cur and (cur / "report.md").is_file()),
            "claim_sources": s["claim_sources"]}


# ---- the surface -----------------------------------------------------------------

def _surface_paths(eng_dir: Path, source: str) -> Dict[str, Path]:
    d = eng_dir / state.SURFACE
    sl = jobs.slug(source)
    return {"frozen": d / f"{sl}.surface.json", "draft": d / f"{sl}.draft.json",
            "comments": d / f"{sl}.comments.json"}


def surface_for(eng_dir: Path, source: str) -> Dict[str, Any]:
    """The claims of one source as the pages show them: the frozen file if
    there is one, else the practice's draft, else the latest enumeration."""
    paths = _surface_paths(eng_dir, source)
    claims: List[Dict[str, Any]] = []
    origin, run = None, None
    for key in ("frozen", "draft"):
        if paths[key].is_file():
            claims = json.loads(paths[key].read_text(encoding="utf-8")).get("claims") or []
            origin = key
            break
    if origin is None:
        r = jobs.latest_enumeration_run(eng_dir, source)
        if r is not None:
            claims = json.loads((r / "claims.json").read_text(encoding="utf-8")).get("claims") or []
            origin, run = "enumeration", r.name
    comments = (json.loads(paths["comments"].read_text(encoding="utf-8"))
                if paths["comments"].is_file() else [])
    return {"source": source, "slug": jobs.slug(source), "frozen": paths["frozen"].is_file(),
            "origin": origin, "run": run, "claims": claims, "comments": comments}


def add_comment(eng_dir: Path, source: str, claim_id: Any, text: str, by: str) -> None:
    paths = _surface_paths(eng_dir, source)
    paths["comments"].parent.mkdir(exist_ok=True)
    rows = (json.loads(paths["comments"].read_text(encoding="utf-8"))
            if paths["comments"].is_file() else [])
    rows.append({"claim_id": claim_id, "text": text, "by": by, "at": state.stamp()})
    paths["comments"].write_text(json.dumps(rows, indent=1, ensure_ascii=False) + "\n",
                                 encoding="utf-8")


def save_draft(eng_dir: Path, source: str, claims: List[Dict[str, Any]]) -> None:
    paths = _surface_paths(eng_dir, source)
    if paths["frozen"].is_file():
        raise SystemExit(f"the surface for {source} is frozen")
    paths["draft"].parent.mkdir(exist_ok=True)
    paths["draft"].write_text(json.dumps({"claim_source": source, "claims": claims},
                                         indent=1, ensure_ascii=False) + "\n", encoding="utf-8")


def freeze(eng_dir: Path, source: str, by: str) -> bool:
    """Write the frozen surface for one source from the draft or the
    enumeration. Returns True when every claim source is now frozen, which
    marks the stage."""
    cur = surface_for(eng_dir, source)
    if cur["frozen"]:
        raise SystemExit(f"the surface for {source} is already frozen")
    if not cur["claims"]:
        raise SystemExit(f"no claims to freeze for {source}: enumerate first")
    paths = _surface_paths(eng_dir, source)
    paths["frozen"].parent.mkdir(exist_ok=True)
    paths["frozen"].write_text(json.dumps({"claim_source": source, "claims": cur["claims"]},
                                          indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    all_frozen = all(_surface_paths(eng_dir, s)["frozen"].is_file()
                     for s in state.claim_sources(eng_dir))
    if all_frozen:
        state.set_stage(eng_dir, "surface", "frozen", by)
    return all_frozen


# ---- request bodies -----------------------------------------------------------

class NewEngagement(BaseModel):
    name: str
    clone: Optional[str] = None
    client_emails: List[str] = []


class Choice(BaseModel):
    id: str


class StageChange(BaseModel):
    stage: str
    value: str


class Comment(BaseModel):
    source: str
    claim_id: Any
    text: str


class Draft(BaseModel):
    source: str
    claims: List[Dict[str, Any]]


class Source(BaseModel):
    source: str


class Settings(BaseModel):
    claim_sources: Optional[List[str]] = None
    client_emails: Optional[List[str]] = None
    target: Optional[str] = None
    retention: Optional[str] = None
    letter: Optional[str] = None            # "" removes the engagement's own letter


# ---- the app ------------------------------------------------------------------

def make_site_app(access: Access, model: Optional[Path] = None,
                  build: Optional[Callable[[Tuple[str, str]], Any]] = None,
                  max_live: int = 8, turns_in_flight: int = 4) -> FastAPI:
    """`build((kind, engagement))` returns the session for a conversation;
    the default builds IntakeSession / PostSession with `model`. Tests pass
    their own."""

    def _default_build(key: Tuple[str, str]):
        kind, name = key
        if kind == "intake":
            from workflowsv2.intake.session import IntakeSession   # noqa: E402
            return IntakeSession(name, model)
        from workflowsv2.claims_audit.post_session import PostSession  # noqa: E402
        return PostSession(name, model)

    registry = Registry(build or _default_build, max_live=max_live,
                        turns_in_flight=turns_in_flight)
    sockets: Dict[Tuple[str, str], Set[WebSocket]] = {}
    opened: Set[Tuple[str, str]] = set()

    app = FastAPI()
    app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")
    app.state.registry = registry

    # ---- identity ----------------------------------------------------------

    def _email(request: Request) -> Optional[str]:
        return access.email(request.headers, request.cookies, request.query_params)

    def _eng(name: str) -> Path:
        if not _NAME.match(name or ""):
            raise HTTPException(status_code=400, detail="bad engagement name")
        d = state.ENGAGEMENTS / name
        if not d.is_dir():
            raise HTTPException(status_code=404, detail=f"no engagement '{name}'")
        return d

    def _require(request: Request, name: Optional[str], roles: Tuple[str, ...]) -> Tuple[str, str, Optional[Path]]:
        email = _email(request)
        eng_dir = _eng(name) if name else None
        role = access.role(email, eng_dir)
        if role not in roles:
            raise HTTPException(status_code=403, detail="not yours to see")
        return email or "", role or "", eng_dir

    def _page(request: Request, path: Path):
        resp = FileResponse(path)
        if access.no_access and request.query_params.get("as"):
            resp.set_cookie(LOCAL_COOKIE, request.query_params["as"], samesite="lax")
        return resp

    def _act(fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except SystemExit as e:
            raise HTTPException(status_code=400, detail=str(e))
        except jobs.JobRunning as e:
            raise HTTPException(status_code=409, detail=f"a job is already running: {e}")

    def _link(name: str, tail: str = "") -> str:
        return f"{mail.site_url()}/e/{name}/{tail}"

    # ---- the front door -----------------------------------------------------

    @app.get("/")
    async def front(request: Request):
        email = _email(request)
        if not email:
            return HTMLResponse("<p>No identity. Sign in through the practice's address.</p>",
                                status_code=403)
        if email in access.practice:
            return RedirectResponse("/p/")
        mine = access.engagements_for(email)
        if len(mine) == 1:
            return RedirectResponse(f"/e/{mine[0]}/")
        if not mine:
            return HTMLResponse("<p>No engagement names this address. Write to the practice.</p>",
                                status_code=403)
        items = "".join(f'<li><a href="/e/{n}/">{n}</a></li>' for n in mine)
        return HTMLResponse(f"<h1>Your engagements</h1><ul>{items}</ul>")

    # ---- the client's engagement home ----------------------------------------

    @app.get("/e/{name}/")
    async def home(name: str, request: Request):
        _require(request, name, ("client", "practice"))
        return _page(request, STATIC / "home.html")

    @app.get("/e/{name}/api/status")
    async def status(name: str, request: Request):
        _, role, eng_dir = _require(request, name, ("client", "practice"))
        return JSONResponse(status_for(eng_dir, role))

    @app.post("/e/{name}/api/letter/accept")
    async def accept_letter(name: str, request: Request):
        email, role, eng_dir = _require(request, name, ("client", "practice"))
        state.set_stage(eng_dir, "letter", "accepted", email)
        if role == "client":
            mail.send(mail.practice_emails(), f"Tuuyi: {email} accepted the letter for {name}",
                      f"{email} accepted the engagement letter for {name}. The intake is next; "
                      f"you will hear again when it is finished.", f"{mail.site_url()}/p/#{name}")
        return JSONResponse(status_for(eng_dir, role))

    # ---- the surface, client side ---------------------------------------------

    @app.get("/e/{name}/surface/")
    async def surface_page(name: str, request: Request):
        _require(request, name, ("client", "practice"))
        return _page(request, STATIC / "surface.html")

    @app.get("/e/{name}/surface/api")
    async def surface_get(name: str, request: Request):
        _, role, eng_dir = _require(request, name, ("client", "practice"))
        return JSONResponse({"name": name, "role": role, "editable": False,
                             "sources": [surface_for(eng_dir, s) for s in state.claim_sources(eng_dir)]})

    @app.post("/e/{name}/surface/api/comment")
    async def surface_comment(name: str, body: Comment, request: Request):
        email, _, eng_dir = _require(request, name, ("client", "practice"))
        if body.source not in state.claim_sources(eng_dir):
            raise HTTPException(status_code=400, detail="no such claim source")
        if not body.text.strip():
            raise HTTPException(status_code=400, detail="an empty comment")
        add_comment(eng_dir, body.source, body.claim_id, body.text.strip(), email)
        return JSONResponse(surface_for(eng_dir, body.source))

    # ---- the two conversations ------------------------------------------------

    def _entry(kind: str, name: str):
        return registry.get((kind, name))

    async def _entry_or_503(kind: str, name: str):
        try:
            return await asyncio.get_running_loop().run_in_executor(None, _entry, kind, name)
        except Exception as e:                                 # noqa: BLE001
            logger.exception("%s session for %s could not be built", kind, name)
            raise HTTPException(status_code=503,
                                detail=f"the conversation could not be started: {type(e).__name__}: {e}")

    async def _push(key: Tuple[str, str], msg: Dict[str, Any]) -> None:
        for ws in list(sockets.get(key, ())):
            try:
                await ws.send_json(msg)
            except Exception:                                  # noqa: BLE001
                sockets[key].discard(ws)

    def _document(kind: str, name: str, e: Dict[str, Any]) -> Dict[str, Any]:
        d = dict(e["session"].document())
        if kind == "intake":
            eng_dir = state.ENGAGEMENTS / name
            check = d.get("check") or {}
            d["finish"] = {"allowed": not check.get("empty"),
                           "done": state.stage_value(eng_dir, "intake") == "done"}
        return d

    async def _turn(kind: str, name: str, text: str) -> None:
        key = (kind, name)
        e = _entry(kind, name)
        await _push(key, {"type": "status", "state": "thinking"})
        try:
            res = await e["worker"].run(lambda: e["session"].turn(text))
            await _push(key, {"type": "say", "text": res.get("reply") or ""})
            await _push(key, {"type": "document", **_document(kind, name, e)})
        except Exception as ex:                                # noqa: BLE001
            await _push(key, {"type": "error", "text": f"{type(ex).__name__}: {ex}"})
        finally:
            await _push(key, {"type": "status", "state": "idle"})

    def _conversation(kind: str) -> None:
        prefix = f"/e/{{name}}/{'intake' if kind == 'intake' else 'report'}"

        def _allowed(request_like, name: str) -> Path:
            email = access.email(request_like.headers, request_like.cookies,
                                 request_like.query_params)
            eng_dir = _eng(name)
            role = access.role(email, eng_dir)
            if role not in ("client", "practice"):
                raise HTTPException(status_code=403, detail="not yours to see")
            if kind == "post" and role == "client" and \
                    state.stage_value(eng_dir, "release") != "released":
                raise HTTPException(status_code=404, detail="the report is not released yet")
            return eng_dir

        @app.get(prefix + "/")
        async def page(name: str, request: Request):
            _allowed(request, name)
            return _page(request, STATIC / "index.html")

        @app.get(prefix + "/api/document")
        async def document(name: str, request: Request):
            _allowed(request, name)
            e = await _entry_or_503(kind, name)
            return JSONResponse(_document(kind, name, e))

        @app.get(prefix + "/api/history")
        async def history(name: str, request: Request):
            _allowed(request, name)
            e = await _entry_or_503(kind, name)
            return JSONResponse({"kind": kind, "history": e["session"].history()})

        if kind == "intake":
            @app.post(prefix + "/api/upload")
            async def upload(name: str, request: Request, file: UploadFile = File(...)):
                eng_dir = _allowed(request, name)
                e = await asyncio.get_running_loop().run_in_executor(None, _entry, kind, name)
                session = e["session"]
                fname = _SAFE.sub("_", Path(file.filename or "upload").name).strip("._") or "upload"
                dest = session.uploads_dir() / fname
                n = 1
                while dest.exists():
                    n += 1
                    dest = dest.with_name(f"{dest.stem}_{n}{dest.suffix}")
                data = await file.read()
                dest.write_bytes(data)
                rel = dest.relative_to(eng_dir).as_posix()
                await _push((kind, name), {"type": "say", "who": "client", "text": f"[uploaded {rel}]"})
                asyncio.get_running_loop().create_task(_turn(kind, name, _announce(rel, data)))
                return JSONResponse({"saved": rel, "bytes": len(data)})

            @app.post(prefix + "/api/finish")
            async def finish(name: str, request: Request):
                eng_dir = _allowed(request, name)
                email = access.email(request.headers, request.cookies, request.query_params) or ""
                e = await asyncio.get_running_loop().run_in_executor(None, _entry, kind, name)
                session = e["session"]
                from workflowsv2.intake import runner as intake_runner   # noqa: E402
                written = _act(intake_runner.finish, eng_dir, session.intake_dir, session.form)
                state.set_stage(eng_dir, "intake", "done", email)
                mail.send(mail.practice_emails(), f"Tuuyi: intake finished for {name}",
                          f"{email} finished the intake for {name}. Next: obtain the "
                          f"materials and mark them ready.", f"{mail.site_url()}/p/#{name}")
                await _push((kind, name), {"type": "document", **_document(kind, name, e)})
                return JSONResponse({"written": written.get("written", []),
                                     "next": next_step(eng_dir)})

        @app.websocket(prefix + "/ws")
        async def ws(name: str, websocket: WebSocket):
            try:
                _allowed(websocket, name)
            except HTTPException:
                await websocket.close(code=4403)
                return
            key = (kind, name)
            await websocket.accept()
            sockets.setdefault(key, set()).add(websocket)
            try:
                try:
                    e = await asyncio.get_running_loop().run_in_executor(None, _entry, kind, name)
                except Exception as ex:                        # noqa: BLE001
                    # The page shows this line instead of "disconnected, retrying"
                    # forever. Observed 2026-09-05: the service lacked the
                    # model route's API key, and the page said nothing.
                    logger.exception("%s session for %s could not be built", kind, name)
                    await websocket.send_json({"type": "error", "text":
                                               f"The conversation could not be started: "
                                               f"{type(ex).__name__}: {ex}. The practice has been told."})
                    mail.send(mail.practice_emails(), f"Tuuyi: {kind} page failed for {name}",
                              f"The {kind} conversation for {name} could not be started: "
                              f"{type(ex).__name__}: {ex}", f"{mail.site_url()}/p/#{name}")
                    await websocket.close(code=1011)
                    return
                await websocket.send_json({"type": "history", "kind": kind,
                                           "history": e["session"].history()})
                await websocket.send_json({"type": "document", **_document(kind, name, e)})
                await websocket.send_json({"type": "status",
                                           "state": "thinking" if e["worker"].busy else "idle"})
                if kind == "intake" and key not in opened and not e["session"].history():
                    opened.add(key)
                    await _push(key, {"type": "status", "state": "thinking"})
                    try:
                        greeting = await e["worker"].run(e["session"].open)
                        await _push(key, {"type": "say", "text": greeting})
                    finally:
                        await _push(key, {"type": "status", "state": "idle"})
                while True:
                    msg = await websocket.receive_json()
                    if msg.get("type") == "turn" and (msg.get("text") or "").strip():
                        text = msg["text"].strip()
                        await _push(key, {"type": "say", "who": "client", "text": text})
                        asyncio.get_running_loop().create_task(_turn(kind, name, text))
            except WebSocketDisconnect:
                pass
            finally:
                sockets.get(key, set()).discard(websocket)

    _conversation("intake")
    _conversation("post")

    # ---- the practice ----------------------------------------------------------

    def _all() -> List[Dict[str, Any]]:
        out = []
        for n in state.engagements():
            d = state.ENGAGEMENTS / n
            s = state.summary(d)
            s["commands"] = jobs.commands(n, s)
            s["next"] = next_step(d)
            s["site"] = True
            s["surfaces"] = [{"source": src, "frozen": _surface_paths(d, src)["frozen"].is_file(),
                              "claims": len(surface_for(d, src)["claims"])}
                             for src in s["claim_sources"]]
            cur = state.current_run(d, s["current_intake"])
            s["report_exists"] = bool(cur and (cur / "report.md").is_file())
            s["settings"] = _settings(d)
            out.append(s)
        return out

    def _settings(eng_dir: Path) -> Dict[str, Any]:
        cfg = state._engagement_yaml(eng_dir)
        own = eng_dir / "letter.md"
        return {"claim_sources": state.claim_sources(eng_dir),
                "client_emails": state.client_emails(eng_dir),
                "target": str(cfg.get("target") or ""), "retention": str(cfg.get("retention") or ""),
                "letter": own.read_text(encoding="utf-8") if own.is_file() else "",
                "letter_is_template": not own.is_file()}

    def _practice(request: Request) -> str:
        email, _, _ = _require(request, None, ("practice",))
        return email

    @app.get("/p/")
    async def practice_page(request: Request):
        _practice(request)
        return _page(request, STATIC / "practice.html")

    @app.get("/p/api/engagements")
    async def list_engagements(request: Request):
        _practice(request)
        return JSONResponse(_all())

    @app.post("/p/api/engagements")
    async def new_engagement(body: NewEngagement, request: Request):
        email = _practice(request)
        if not _NAME.match(body.name or ""):
            raise HTTPException(status_code=400, detail="bad engagement name")
        _act(state.new_engagement, state.ENGAGEMENTS / body.name, body.clone or None,
             body.client_emails, email)
        clients = state.client_emails(state.ENGAGEMENTS / body.name)
        if clients:
            cf_access.ensure_emails(clients)
            mail.send(clients, "Tuuyi: your engagement page",
                      "Your engagement with Tuuyi is open. The page below shows where "
                      "it stands and what happens next. Start by reading the "
                      "engagement letter.", _link(body.name))
        return JSONResponse(_all())

    @app.post("/p/api/engagements/{name}/settings")
    async def change_settings(name: str, body: Settings, request: Request):
        """The practice sets the engagement's claim sources, client emails,
        target and retention, and its own letter. New client emails are added
        to the Access policy."""
        _practice(request)
        eng_dir = _eng(name)
        before = set(state.client_emails(eng_dir))
        _act(state.update_engagement, eng_dir, claim_sources=body.claim_sources,
             client_emails=body.client_emails, target=body.target, retention=body.retention)
        if body.letter is not None:
            own = eng_dir / "letter.md"
            if body.letter.strip():
                own.write_text(body.letter, encoding="utf-8")
            elif own.is_file():
                own.unlink()
        sync = None
        new = [e for e in state.client_emails(eng_dir) if e not in before]
        if new:
            sync = cf_access.ensure_emails(new)
            mail.send(new, "Tuuyi: your engagement page",
                      "Your engagement with Tuuyi is open. The page below shows where "
                      "it stands and what happens next.", _link(name))
        return JSONResponse({"engagements": _all(), "policy": sync})

    @app.post("/p/api/engagements/{name}/intake/current")
    async def intake_current(name: str, body: Choice, request: Request):
        _practice(request)
        _act(state.set_current_intake, _eng(name), body.id)
        return JSONResponse(_all())

    @app.post("/p/api/engagements/{name}/intake/cancel")
    async def intake_cancel(name: str, body: Choice, request: Request):
        _practice(request)
        _act(state.cancel_intake, _eng(name), body.id)
        return JSONResponse(_all())

    @app.post("/p/api/engagements/{name}/run/current")
    async def run_current(name: str, body: Choice, request: Request):
        _practice(request)
        _act(state.set_current_run, _eng(name), body.id)
        return JSONResponse(_all())

    @app.post("/p/api/engagements/{name}/run/cancel")
    async def run_cancel(name: str, body: Choice, request: Request):
        _practice(request)
        _act(state.cancel_run, _eng(name), body.id)
        return JSONResponse(_all())

    #: The stage changes a practice button makes directly, and the value each sets.
    BUTTON_STAGES = {"materials": "ready", "release": "released", "closed": "closed"}

    @app.post("/p/api/engagements/{name}/stage")
    async def change_stage(name: str, body: StageChange, request: Request):
        email = _practice(request)
        eng_dir = _eng(name)
        if body.stage not in BUTTON_STAGES or body.value != BUTTON_STAGES[body.stage]:
            raise HTTPException(status_code=400, detail="not a stage a button sets")
        if body.stage == "release":
            cur = state.current_run(eng_dir, state.current_intake(eng_dir))
            if not (cur and (cur / "report.md").is_file()):
                raise HTTPException(status_code=400, detail="no report to release")
        state.set_stage(eng_dir, body.stage, body.value, email)
        if body.stage == "release":
            mail.send(state.client_emails(eng_dir), f"Tuuyi: the report for {name} is ready",
                      "The claims review is finished and the report is released. Read it "
                      "on the page below and ask it anything.", _link(name, "report/"))
        return JSONResponse(_all())

    @app.post("/p/api/engagements/{name}/jobs/{kind}")
    async def start_job(name: str, kind: str, request: Request):
        email = _practice(request)
        eng_dir = _eng(name)
        if kind not in jobs.KINDS:
            raise HTTPException(status_code=400, detail=f"no job kind '{kind}'")
        job = _act(jobs.start, eng_dir, kind, email, str(model) if model else jobs.MODEL)
        return JSONResponse({"job": job.record, "engagements": _all()})

    @app.get("/p/api/engagements/{name}/jobs/{job_id}/log")
    async def job_log(name: str, job_id: str, request: Request, tail: int = 200):
        _practice(request)
        eng_dir = _eng(name)
        rec = next((j for j in state.jobs(eng_dir) if j["id"] == job_id), None)
        if not rec or not rec.get("log") or not Path(rec["log"]).is_file():
            raise HTTPException(status_code=404, detail="no log")
        lines = Path(rec["log"]).read_text(encoding="utf-8", errors="replace").splitlines()
        return PlainTextResponse("\n".join(lines[-max(1, tail):]))

    @app.get("/p/surface/{name}/")
    async def practice_surface_page(name: str, request: Request):
        _practice(request)
        _eng(name)
        return _page(request, STATIC / "surface.html")

    @app.get("/p/surface/{name}/api")
    async def practice_surface_get(name: str, request: Request):
        _practice(request)
        eng_dir = _eng(name)
        return JSONResponse({"name": name, "role": "practice", "editable": True,
                             "sources": [surface_for(eng_dir, s) for s in state.claim_sources(eng_dir)]})

    @app.post("/p/surface/{name}/api/comment")
    async def practice_surface_comment(name: str, body: Comment, request: Request):
        email = _practice(request)
        eng_dir = _eng(name)
        if body.source not in state.claim_sources(eng_dir):
            raise HTTPException(status_code=400, detail="no such claim source")
        add_comment(eng_dir, body.source, body.claim_id, body.text.strip(), email)
        return JSONResponse(surface_for(eng_dir, body.source))

    @app.post("/p/surface/{name}/api/draft")
    async def practice_surface_draft(name: str, body: Draft, request: Request):
        _practice(request)
        eng_dir = _eng(name)
        if body.source not in state.claim_sources(eng_dir):
            raise HTTPException(status_code=400, detail="no such claim source")
        _act(save_draft, eng_dir, body.source, body.claims)
        return JSONResponse(surface_for(eng_dir, body.source))

    @app.post("/p/surface/{name}/api/freeze")
    async def practice_surface_freeze(name: str, body: Source, request: Request):
        email = _practice(request)
        eng_dir = _eng(name)
        if body.source not in state.claim_sources(eng_dir):
            raise HTTPException(status_code=400, detail="no such claim source")
        all_frozen = _act(freeze, eng_dir, body.source, email)
        if all_frozen:
            mail.send(state.client_emails(eng_dir), f"Tuuyi: the claim surface for {name} is frozen",
                      "The list of claims to be tested is settled. The review runs next; "
                      "you will hear when the report is released.", _link(name))
        return JSONResponse(surface_for(eng_dir, body.source))

    return app


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, default=Path(jobs.MODEL),
                    help="YAML with an llm_config block, for conversations and jobs")
    ap.add_argument("--port", type=int, default=8803)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--no-access", action="store_true",
                    help="identity from ?as=<email>; loopback only")
    args = ap.parse_args(argv)
    if args.no_access and args.host not in ("127.0.0.1", "localhost", "::1"):
        raise SystemExit("--no-access is for the loopback address only")
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    for name in ("chat_loop", "faiss.loader", "sentence_transformers"):
        logging.getLogger(name).setLevel(logging.WARNING)
    access = Access.from_env(no_access=args.no_access)
    if not access.practice:
        raise SystemExit("PRACTICE_EMAILS is empty: nobody could open the practice pages")
    app = make_site_app(access, args.model)
    print(f"\nclient site — {len(state.engagements())} engagement(s); "
          f"{'NO ACCESS CHECK (loopback)' if args.no_access else 'Cloudflare Access'}; "
          f"mail {'dry run' if mail.dry_run() else 'live'}")
    print(f"open:  http://{args.host}:{args.port}/"
          + (f"?as={sorted(access.practice)[0]}" if args.no_access else "") + "\n")
    import uvicorn                                             # noqa: E402
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        app.state.registry.close_all()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
