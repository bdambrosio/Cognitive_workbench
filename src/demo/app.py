#!/usr/bin/env python3
"""The public demo: many visitors, each in their own conversation over one
delivered report, the target's identity withheld.

    python3 src/demo/app.py --engagement demo-chhoto \\
        --model measure/models/local_qwen38flashnext.yaml [--port 8810] [--host 127.0.0.1]

ONE PROCESS, MANY VISITORS. A visitor is a cookie. Their world is
`demo_<engagement>_<run ts>_<sid>`, so their history is theirs and a return
visit resumes it. Live sessions are capped and the least recently used is
closed (its world stays on disk); a process-wide semaphore keeps turns in
flight below the model server's concurrency. Turns per visitor, message
length and new sessions per address per hour are limited, and each limit
says so in one line.

NOTHING IDENTIFYING LEAVES. `engagements/<e>/demo.yaml` lists the target's
identifiers; src/demo/redact.py substitutes them in every string sent to a
browser — the banner, the report, the findings, the history, each reply.
The demo's continuation scenario tells the agent the same, so replies
rarely need it; the substitution guarantees it.

LOCAL MODEL. Meant for measure/models/local_qwen38flashnext.yaml on the
practice's own hardware; the served model is verified at start.
"""
from __future__ import annotations

import argparse
import asyncio
import datetime
import hashlib
import json
import logging
import os
import queue
import secrets
import shutil
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

import yaml                                                    # noqa: E402
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect  # noqa: E402
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse  # noqa: E402
from fastapi.staticfiles import StaticFiles                    # noqa: E402

from demo.redact import Redactor, banner                       # noqa: E402

logger = logging.getLogger("demo")
STATIC = HERE / "static"
CLIENT_STATIC = REPO / "src" / "client_ui" / "static"
COOKIE = "demo_sid"
DEMO_SCENARIO = REPO / "workflowsv2" / "claims_audit" / "continuation_demo.yaml"

DEFAULTS = {"display_name": "the target", "max_live": 8, "max_turns": 20,
            "max_chars": 2000, "sessions_per_ip_per_hour": 6,
            "turns_in_flight": 3, "keep_days": 7, "model_label": "a local model"}


def load_demo_config(eng_dir: Path) -> Dict[str, Any]:
    p = eng_dir / "demo.yaml"
    cfg = dict(DEFAULTS)
    if p.is_file():
        cfg.update(yaml.safe_load(p.read_text(encoding="utf-8")) or {})
    cfg.setdefault("redact", [])
    return cfg


class Worker:
    """One thread per live visitor; every turn of theirs runs on it in order,
    and waits its turn on the process-wide semaphore."""

    def __init__(self, sem: threading.Semaphore) -> None:
        self.q: "queue.Queue[tuple]" = queue.Queue()
        self.sem = sem
        self.busy = False
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self) -> None:
        while True:
            fn, loop, fut = self.q.get()
            if fn is None:
                return
            with self.sem:
                self.busy = True
                try:
                    res = fn()
                    loop.call_soon_threadsafe(fut.set_result, res)
                except Exception as e:                         # noqa: BLE001
                    logger.exception("turn failed")
                    loop.call_soon_threadsafe(fut.set_exception, e)
                finally:
                    self.busy = False

    async def run(self, fn):
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self.q.put((fn, loop, fut))
        return await fut

    def stop(self) -> None:
        self.q.put((None, None, None))


class Visitors:
    """The registry: sid → live session, built on demand, evicted LRU."""

    def __init__(self, build, cfg: Dict[str, Any], log_path: Path) -> None:
        self.build = build
        self.cfg = cfg
        self.log_path = log_path
        self.live: Dict[str, Dict[str, Any]] = {}
        self.turns: Dict[str, int] = {}
        self.created: Dict[str, float] = {}
        self.by_ip: Dict[str, List[float]] = {}
        self.sem = threading.Semaphore(int(cfg["turns_in_flight"]))
        self.waiting = 0
        self.lock = threading.Lock()
        self._load_log()

    def _load_log(self) -> None:
        if not self.log_path.is_file():
            return
        for line in self.log_path.read_text(encoding="utf-8").splitlines():
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if d.get("event") == "turn":
                self.turns[d["sid"]] = self.turns.get(d["sid"], 0) + 1
            elif d.get("event") == "new":
                self.created[d["sid"]] = d.get("ts", 0)

    def log(self, event: str, sid: str, **kw) -> None:
        row = {"event": event, "sid": sid, "ts": time.time(),
               "at": datetime.datetime.now(datetime.timezone.utc).isoformat(), **kw}
        with self.log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row) + "\n")

    def new_sid(self, ip: str) -> Optional[str]:
        """A fresh sid, or None when this address has opened too many this hour."""
        now = time.time()
        h = hashlib.sha256(("demo:" + ip).encode()).hexdigest()[:16]
        recent = [t for t in self.by_ip.get(h, []) if now - t < 3600]
        if len(recent) >= int(self.cfg["sessions_per_ip_per_hour"]):
            return None
        sid = secrets.token_urlsafe(18)
        self.by_ip[h] = recent + [now]
        self.created[sid] = now
        self.log("new", sid, ip_hash=h)
        return sid

    def known(self, sid: str) -> bool:
        return sid in self.created

    def get(self, sid: str) -> Dict[str, Any]:
        """The live entry for a visitor, building the session if needed and
        evicting the least recently used above the cap."""
        with self.lock:
            e = self.live.get(sid)
            if e:
                e["last"] = time.time()
                return e
            while len(self.live) >= int(self.cfg["max_live"]):
                oldest = min(self.live, key=lambda k: self.live[k]["last"])
                self._close(oldest)
            session = self.build(sid)
            e = {"session": session, "worker": Worker(self.sem), "last": time.time()}
            self.live[sid] = e
            return e

    def _close(self, sid: str) -> None:
        e = self.live.pop(sid, None)
        if not e:
            return
        e["worker"].stop()
        try:
            e["session"].close()
        except Exception as ex:                                # noqa: BLE001
            logger.warning("close %s: %s", sid[:8], ex)
        self.log("evict", sid)

    def close_all(self) -> None:
        with self.lock:
            for sid in list(self.live):
                self._close(sid)


def sweep_worlds(scenarios: Path, prefix: str, keep_days: int) -> int:
    """Remove demo worlds nobody has touched for `keep_days`."""
    cutoff = time.time() - keep_days * 86400
    n = 0
    for d in scenarios.glob(prefix + "*"):
        if d.is_dir() and d.stat().st_mtime < cutoff:
            shutil.rmtree(d, ignore_errors=True)
            n += 1
    return n


def make_app(build, document: Dict[str, Any], redactor: Redactor,
             cfg: Dict[str, Any], log_path: Path) -> FastAPI:
    """`build(sid)` returns a session for that visitor (a PostSession, or a
    test double with turn/history/close). `document` is the redacted report
    document shared by every visitor."""
    visitors = Visitors(build, cfg, log_path)
    sockets: Dict[str, Set[WebSocket]] = {}
    app = FastAPI()
    app.mount("/static", StaticFiles(directory=str(CLIENT_STATIC)), name="static")
    app.mount("/demo-static", StaticFiles(directory=str(STATIC)), name="demo-static")
    app.state.visitors = visitors

    def _ip(request) -> str:
        fwd = request.headers.get("x-forwarded-for")
        return (fwd.split(",")[0].strip() if fwd else (request.client.host if request.client else "?"))

    @app.get("/")
    async def index(request: Request):
        sid = request.cookies.get(COOKIE)
        resp = FileResponse(STATIC / "demo.html")
        if not sid or not visitors.known(sid):
            sid = visitors.new_sid(_ip(request))
            if sid is None:
                return HTMLResponse("<p>Too many new sessions from this address this hour. "
                                    "Please come back later.</p>", status_code=429)
            secure = request.url.scheme == "https" or request.headers.get("x-forwarded-proto") == "https"
            resp.set_cookie(COOKIE, sid, httponly=True, samesite="lax", secure=secure,
                            max_age=int(cfg["keep_days"]) * 86400)
        return resp

    @app.get("/api/document")
    async def get_document(request: Request):
        return JSONResponse(document)

    async def _push(sid: str, msg: Dict[str, Any]) -> None:
        for ws in list(sockets.get(sid, ())):
            try:
                await ws.send_json(msg)
            except Exception:                                  # noqa: BLE001
                sockets[sid].discard(ws)

    async def _turn(sid: str, text: str) -> None:
        e = visitors.get(sid)
        ahead = e["worker"].q.qsize() + max(0, visitors.waiting)
        await _push(sid, {"type": "status", "state": "thinking",
                          "ahead": ahead})
        visitors.waiting += 1
        try:
            res = await e["worker"].run(lambda: e["session"].turn(text))
            visitors.turns[sid] = visitors.turns.get(sid, 0) + 1
            visitors.log("turn", sid, chars=len(text))
            await _push(sid, {"type": "say", "text": redactor.text(res.get("reply") or "")})
        except Exception as ex:                                # noqa: BLE001
            await _push(sid, {"type": "error", "text": f"The model did not answer: {type(ex).__name__}."})
        finally:
            visitors.waiting -= 1
            await _push(sid, {"type": "status", "state": "idle"})

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        sid = websocket.cookies.get(COOKIE)
        if not sid or not visitors.known(sid):
            await websocket.close(code=4401)
            return
        await websocket.accept()
        sockets.setdefault(sid, set()).add(websocket)
        try:
            e = await asyncio.get_running_loop().run_in_executor(None, visitors.get, sid)
            await websocket.send_json({"type": "history", "kind": "post",
                                       "history": redactor.deep(e["session"].history())})
            await websocket.send_json({"type": "document", **document})
            await websocket.send_json({"type": "status",
                                       "state": "thinking" if e["worker"].busy else "idle"})
            while True:
                msg = await websocket.receive_json()
                if msg.get("type") != "turn":
                    continue
                text = (msg.get("text") or "").strip()
                if not text:
                    continue
                if len(text) > int(cfg["max_chars"]):
                    await websocket.send_json({"type": "error", "text": f"Messages are limited to {cfg['max_chars']} characters in the demo."})
                    continue
                if visitors.turns.get(sid, 0) >= int(cfg["max_turns"]):
                    await websocket.send_json({"type": "error", "text": f"The demo allows {cfg['max_turns']} questions per visitor. Thank you for trying it."})
                    continue
                await _push(sid, {"type": "say", "who": "client", "text": text})
                asyncio.get_running_loop().create_task(_turn(sid, text))
        except WebSocketDisconnect:
            pass
        finally:
            sockets.get(sid, set()).discard(websocket)

    return app


def served_model(model_yaml: Path) -> Optional[str]:
    """What the model server says it serves, or None if it cannot be asked."""
    import requests
    doc = yaml.safe_load(model_yaml.read_text(encoding="utf-8")) or {}
    url = (doc.get("llm_config") or {}).get("vllm_url")
    if not url:
        return None
    try:
        r = requests.get(url.rstrip("/") + "/v1/models", timeout=5)
        ids = [m.get("id") for m in (r.json().get("data") or [])]
        return ", ".join(i for i in ids if i)
    except Exception as e:                                     # noqa: BLE001
        logger.warning("served model: %s", e)
        return None


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--model", type=Path, required=True,
                    help="YAML with an llm_config block (the local model)")
    ap.add_argument("--port", type=int, default=8810)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--merged", type=Path, default=None,
                    help="the run to show (default: the engagement's current run)")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    for name in ("chat_loop", "faiss.loader", "sentence_transformers"):
        logging.getLogger(name).setLevel(logging.WARNING)

    from workflowsv2 import engagement_state as state           # noqa: E402
    from workflowsv2.claims_audit.post_session import PostSession  # noqa: E402
    eng_dir = state.ENGAGEMENTS / args.engagement
    if not eng_dir.is_dir():
        raise SystemExit(f"no engagement '{args.engagement}'")
    cfg = load_demo_config(eng_dir)

    doc = yaml.safe_load(args.model.read_text(encoding="utf-8")) or {}
    expected = doc.get("expects_served_model")
    served = served_model(args.model)
    if expected and served and expected.lower() not in served.lower():
        raise SystemExit(f"the server serves {served!r}, not {expected!r}")
    logger.info("model server serves %s", served)

    # One base session builds the shared, redacted document; visitors get
    # their own sessions in their own worlds.
    base = PostSession(args.engagement, args.model, merged=args.merged,
                       world=f"demo_{args.engagement}_base", log_to_world=False,
                       scenario=DEMO_SCENARIO)
    redactor = Redactor(cfg["display_name"], cfg["redact"],
                        roots=[REPO, eng_dir, base.target])
    merged = base.merged
    fig = merged.get("figures") or {}
    document = redactor.deep({
        "kind": "report", "engagement": cfg["display_name"], "intake": None,
        "run": "demo", "html": base.document()["html"],
        "findings": base.findings(),
        "banner": banner(cfg["display_name"], fig.get("claims", 0),
                         fig.get("findings", 0), len(merged.get("runs") or []),
                         cfg["model_label"])})
    leaks = redactor.leaks(json.dumps(document))
    if leaks:
        raise SystemExit(f"the redacted document still carries: {leaks}")
    run_ts = base.merged_dir.name.split("_", 1)[0].replace("-", "").replace(":", "")
    base.close()

    prefix = f"demo_{args.engagement}_"
    n = sweep_worlds(REPO / "scenarios", prefix, int(cfg["keep_days"]))
    logger.info("swept %d stale demo world(s)", n)

    def build(sid: str):
        return PostSession(args.engagement, args.model, merged=args.merged,
                           world=f"{prefix}{run_ts[:14]}_{sid[:12]}",
                           log_to_world=False, scenario=DEMO_SCENARIO)

    app = make_app(build, document, redactor, cfg, eng_dir / "demo_sessions.jsonl")
    print(f"\ndemo: {args.engagement} as {cfg['display_name']!r}; "
          f"{fig.get('claims', 0)} claims\nopen:  http://{args.host}:{args.port}/\n")
    import uvicorn                                             # noqa: E402
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        app.state.visitors.close_all()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
