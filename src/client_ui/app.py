#!/usr/bin/env python3
"""The client's browser page for the intake and the post-delivery conversation.

    python3 -m client_ui.app intake --engagement <e> [--model <yaml>] [--new] [--port 8800]
    python3 -m client_ui.app post   --engagement <e> [--model <yaml>] [--intake <id>]
                                    [--merged <dir>] [--world <w>] [--port 8801]

Run from src/ (or with src/ on PYTHONPATH). Prints the URL, token included.

ONE PROCESS, ONE CLIENT, ONE SESSION. The app constructs the same session
object the terminal runner uses (workflowsv2/intake/session.py,
workflowsv2/claims_audit/post_session.py) and drives it from one worker
thread, because ChatLoop is built for a single consumer. Turns are
serialised; a second message while one is running waits its turn.

TWO PANES, ONE PAGE. Chat on the left; on the right the intake form as it
fills, or the delivered report. The page is static and the same for both
kinds; `document.kind` decides what the right pane renders.

THE UI INITIATES NOTHING. It cannot start an intake, a run or a conversation
on a different run; it cannot finish an intake. Those are the practice's
command-line actions (intake/runner.py --new --finish,
workflowsv2/engagement_state.py).

A SHARED SECRET ON THE URL. The page and the websocket require `?token=`;
without it a browser gets 403. Bound to 127.0.0.1 unless --host says
otherwise. Single tenant: the token protects one client's conversation from
another tab on the same machine, not the practice from the internet.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
import re
import secrets
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from fastapi import FastAPI, File, HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect  # noqa: E402
from fastapi.responses import FileResponse, JSONResponse                         # noqa: E402
from fastapi.staticfiles import StaticFiles                                       # noqa: E402

from client_ui.registry import Worker                                            # noqa: E402

logger = logging.getLogger("client_ui")
STATIC = HERE / "static"
#: A saved upload's name: the client's basename with anything but these
#: characters replaced, so it cannot name a directory.
_SAFE = re.compile(r"[^A-Za-z0-9._-]+")


#: How much of an uploaded text file rides along in the announcing turn.
_INLINE_CHARS = 8000


def _announce(rel: str, data: bytes) -> str:
    """The turn that tells the agent about an upload."""
    head = f"[The client uploaded a file into the engagement: {rel} ({len(data)} bytes)."
    if b"\0" in data[:8192]:
        return head + " It is not a text file.]"
    text = data.decode("utf-8", errors="replace")
    if len(text) > _INLINE_CHARS:
        return (head + f" Its first {_INLINE_CHARS} characters:]\n\n"
                + text[:_INLINE_CHARS])
    return head + " Its contents:]\n\n" + text


def make_app(session, kind: str, token: str) -> FastAPI:
    """`session` is an IntakeSession or PostSession (or a test double with
    the same methods); `kind` is "intake" or "post"."""
    worker = Worker()
    sockets: Set[WebSocket] = set()
    state: Dict[str, Any] = {"opened": False}

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        yield
        worker.stop()
        try:
            session.close()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("session close failed: %s", e)

    app = FastAPI(lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")

    def _check(t: Optional[str]) -> None:
        if not t or not secrets.compare_digest(t, token):
            raise HTTPException(status_code=403, detail="token required")

    async def _push(msg: Dict[str, Any]) -> None:
        dead = []
        for ws in list(sockets):
            try:
                await ws.send_json(msg)
            except Exception:                                  # noqa: BLE001
                dead.append(ws)
        for ws in dead:
            sockets.discard(ws)

    async def _turn(text: str) -> None:
        await _push({"type": "status", "state": "thinking"})
        try:
            res = await worker.run(lambda: session.turn(text))
            await _push({"type": "say", "text": res.get("reply") or ""})
            await _push({"type": "document", **session.document()})
        except Exception as e:                                 # noqa: BLE001
            await _push({"type": "error", "text": f"{type(e).__name__}: {e}"})
        finally:
            await _push({"type": "status", "state": "idle"})

    @app.get("/")
    async def index(token: Optional[str] = None):
        _check(token)
        return FileResponse(STATIC / "index.html")

    @app.get("/api/document")
    async def document(token: Optional[str] = None):
        _check(token)
        return JSONResponse(session.document())

    @app.get("/api/history")
    async def history(token: Optional[str] = None):
        _check(token)
        return JSONResponse({"kind": kind, "history": session.history()})

    @app.post("/api/upload")
    async def upload(request: Request, file: UploadFile = File(...),
                     token: Optional[str] = None):
        _check(token)
        if kind != "intake":
            raise HTTPException(status_code=400, detail="uploads belong to the intake")
        name = _SAFE.sub("_", Path(file.filename or "upload").name).strip("._") or "upload"
        dest = session.uploads_dir() / name
        n = 1
        while dest.exists():
            n += 1
            dest = dest.with_name(f"{dest.stem}_{n}{dest.suffix}")
        data = await file.read()
        dest.write_bytes(data)
        rel = dest.relative_to(session.eng_dir).as_posix()
        logger.info("upload: %s (%d bytes)", rel, len(data))
        await _push({"type": "say", "who": "client",
                     "text": f"[uploaded {rel}]"})
        # THE AGENT IS TOLD IN THE CONVERSATION, text included: the intake
        # agent has no repository tools, so what it does not see in the turn
        # it does not see at all. A binary or a long file is announced by
        # name and size; the audit reads it later from the engagement dir.
        asyncio.get_running_loop().create_task(_turn(_announce(rel, data)))
        return JSONResponse({"saved": rel, "bytes": len(data)})

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        t = websocket.query_params.get("token")
        if not t or not secrets.compare_digest(t, token):
            await websocket.close(code=4403)
            return
        await websocket.accept()
        sockets.add(websocket)
        try:
            await websocket.send_json({"type": "history", "kind": kind,
                                       "history": session.history()})
            await websocket.send_json({"type": "document", **session.document()})
            await websocket.send_json({"type": "status",
                                       "state": "thinking" if worker.busy else "idle"})
            # THE OPENING: the intake's first words are the agent's, once,
            # when nobody has spoken yet.
            if kind == "intake" and not state["opened"] and not session.history():
                state["opened"] = True
                await _push({"type": "status", "state": "thinking"})
                try:
                    greeting = await worker.run(session.open)
                    await _push({"type": "say", "text": greeting})
                finally:
                    await _push({"type": "status", "state": "idle"})
            while True:
                msg = await websocket.receive_json()
                if msg.get("type") == "turn" and (msg.get("text") or "").strip():
                    text = msg["text"].strip()
                    await _push({"type": "say", "who": "client", "text": text})
                    asyncio.get_running_loop().create_task(_turn(text))
        except WebSocketDisconnect:
            pass
        finally:
            sockets.discard(websocket)

    return app


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="kind", required=True)
    for k in ("intake", "post"):
        s = sub.add_parser(k)
        s.add_argument("--engagement", required=True)
        s.add_argument("--model", type=Path, default=None,
                       help="YAML with an llm_config block; replaces the scenario's")
        s.add_argument("--port", type=int, default=8800 if k == "intake" else 8801)
        s.add_argument("--host", default="127.0.0.1")
        s.add_argument("--token", default=None,
                       help="shared secret (default: $CLIENT_UI_TOKEN, else generated)")
        s.add_argument("--world", default=None)
    sub.choices["intake"].add_argument("--new", action="store_true",
                                       help="start another intake; it becomes current")
    sub.choices["post"].add_argument("--intake", default=None)
    sub.choices["post"].add_argument("--merged", type=Path, default=None)
    args = ap.parse_args(argv)

    token = args.token or os.environ.get("CLIENT_UI_TOKEN") or secrets.token_urlsafe(16)
    if args.kind == "intake":
        from workflowsv2.intake.session import IntakeSession   # noqa: E402
        session = IntakeSession(args.engagement, args.model, new=args.new,
                                world=args.world)
    else:
        from workflowsv2.claims_audit.post_session import PostSession  # noqa: E402
        session = PostSession(args.engagement, args.model, intake=args.intake,
                              merged=args.merged, world=args.world)
    app = make_app(session, args.kind, token)
    print(f"\n{args.kind}: {args.engagement}")
    if args.kind == "post":
        print(session.describe())
    print(f"\nopen:  http://{args.host}:{args.port}/?token={token}\n")
    import uvicorn                                             # noqa: E402
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
