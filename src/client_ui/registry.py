"""Live sessions in one process: a worker thread per session, a registry
that builds sessions on demand and evicts the least recently used.

CONSOLIDATED 2026-09-05 from `src/demo/app.py` (Visitors, Worker) and
`src/client_ui/app.py` (Worker) when the client site needed the same thing
keyed by engagement instead of by visitor cookie. ChatLoop is built for a
single consumer, so every turn of one session runs on that session's
worker, in order; a process-wide semaphore keeps turns in flight below
what the model route carries.
"""
from __future__ import annotations

import asyncio
import logging
import queue
import threading
import time
from typing import Any, Callable, Dict, Hashable, Optional

logger = logging.getLogger("client_ui.registry")


class Worker:
    """One thread, one queue: every turn runs on it, in order, and waits
    its turn on the shared semaphore when there is one."""

    def __init__(self, sem: Optional[threading.Semaphore] = None) -> None:
        self.q: "queue.Queue[tuple]" = queue.Queue()
        self.sem = sem
        self.busy = False
        self.t = threading.Thread(target=self._run, name="session-worker", daemon=True)
        self.t.start()

    def _run(self) -> None:
        while True:
            fn, loop, fut = self.q.get()
            if fn is None:
                return
            if self.sem is not None:
                self.sem.acquire()
            self.busy = True
            try:
                res = fn()
                loop.call_soon_threadsafe(fut.set_result, res)
            except Exception as e:                             # noqa: BLE001
                logger.exception("turn failed")
                loop.call_soon_threadsafe(fut.set_exception, e)
            finally:
                self.busy = False
                if self.sem is not None:
                    self.sem.release()

    async def run(self, fn: Callable[[], Any]) -> Any:
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self.q.put((fn, loop, fut))
        return await fut

    def stop(self) -> None:
        self.q.put((None, None, None))


class Registry:
    """key -> live entry {session, worker, last}, built on demand by
    `build(key)`, evicted least-recently-used above `max_live`. A session
    must have `close()`; its world stays on disk when it is evicted."""

    def __init__(self, build: Callable[[Hashable], Any], max_live: int = 8,
                 turns_in_flight: int = 3) -> None:
        self.build = build
        self.max_live = int(max_live)
        self.live: Dict[Hashable, Dict[str, Any]] = {}
        self.sem = threading.Semaphore(int(turns_in_flight))
        self.waiting = 0
        self.lock = threading.Lock()

    def peek(self, key: Hashable) -> Optional[Dict[str, Any]]:
        return self.live.get(key)

    def get(self, key: Hashable) -> Dict[str, Any]:
        with self.lock:
            e = self.live.get(key)
            if e:
                e["last"] = time.time()
                return e
            while len(self.live) >= self.max_live:
                oldest = min(self.live, key=lambda k: self.live[k]["last"])
                self._close(oldest)
            session = self.build(key)
            e = {"session": session, "worker": Worker(self.sem), "last": time.time()}
            self.live[key] = e
            return e

    def _close(self, key: Hashable) -> None:
        e = self.live.pop(key, None)
        if not e:
            return
        e["worker"].stop()
        try:
            e["session"].close()
        except Exception as ex:                                # noqa: BLE001
            logger.warning("close %s: %s", key, ex)

    def close(self, key: Hashable) -> None:
        with self.lock:
            self._close(key)

    def close_all(self) -> None:
        with self.lock:
            for key in list(self.live):
                self._close(key)
