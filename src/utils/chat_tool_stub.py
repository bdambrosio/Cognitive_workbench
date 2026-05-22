"""Shared helpers for `react_invoke` wrappers in src/tools/.

OODA-era tools take an `executor` + `resource_manager` and shape their
output through note/collection creation. The chat ReAct loop doesn't
have either, so each tool's `react_invoke` stubs them out and pulls the
text content directly from the capturing manager.

Two pieces:
  - StubExecutor: minimal duck-type for `_create_uniform_return`.
  - CapturingResourceManager: implements the resource_manager API that
    tools call (`create_note`, `create_collection`, etc.) but stashes
    the content in memory and returns fake IDs. After running a tool,
    call `.last_text()` / `.all_text()` to retrieve what got "created".
  - translate_result: convert legacy `tool()` return dicts into the
    chat ReAct `{status, text}` shape.

Living in `src/utils/` so multiple tool dirs share one copy.
"""

from __future__ import annotations

import itertools
from typing import Any, Dict, List, Optional, Tuple


class StubExecutor:
    """Minimal duck-type for the OODA executor's `_create_uniform_return`.

    OODA tools call this from their `_fail` / `_success` helpers. The
    real executor wires up note/collection creation; the stub just
    boxes the result fields into a plain dict that `translate_result`
    can read.

    Also exposes `executive_node.benchmark_mode = True` because some
    OODA tools (exec-script in particular) check that flag before
    triggering an interactive permission prompt. The chat ReAct loop
    can't be interactive, so we always present as benchmark-mode —
    matches the agreed "trusted by default" policy.
    """

    def __init__(self, world_name: Optional[str] = None):
        from types import SimpleNamespace
        self.executive_node = SimpleNamespace(benchmark_mode=True)
        # Some tools (exec-script) read world_name off the executor when
        # they need a scenario-fs path. Optional for tools that don't.
        self.world_name = world_name

    @staticmethod
    def _create_uniform_return(status: str, value: Any = None,
                               reason: Optional[str] = None,
                               resource_id: Optional[str] = None,
                               extra: Optional[Dict[str, Any]] = None,
                               **kw) -> Dict[str, Any]:
        return {
            "status": status,
            "value": value,
            "reason": reason,
            "resource_id": resource_id,
            "extra": extra or {},
        }


class CapturingResourceManager:
    """In-memory stand-in for InfospaceResourceManager.

    OODA tools create notes/collections via `resource_manager.create_note(...)`
    and put their actual output as the content. For chat ReAct, we don't
    want to persist anything — we just want the text. So this manager
    accepts the same calls, returns fake resource IDs, and stashes the
    content for the caller to pull out after the tool runs.

    Only the methods OODA tools actually call are implemented; anything
    else raises `AttributeError` loudly so a tool exercising an unsupported
    code path doesn't fail silently.
    """

    def __init__(self):
        self._counter = itertools.count(1)
        self._notes: Dict[str, str] = {}
        self._collections: Dict[str, List[str]] = {}
        self._creation_order: List[str] = []

    # ---- Note creation ----------------------------------------------------
    def create_note(self, character_name: str, content: Any, format_type: str,
                    source_skill: str, source_value: str, note_name: str,
                    extra_props: Dict
                    ) -> Tuple[bool, Optional[str], Optional[str], Optional[Tuple[int, int]]]:
        nid = f"StubNote_{next(self._counter)}"
        self._notes[nid] = "" if content is None else str(content)
        self._creation_order.append(nid)
        return True, nid, None, None

    def update_note_content(self, note_id: str, new_content: Any,
                            reindex: bool = True, **kw
                            ) -> Tuple[bool, Optional[str]]:
        if note_id not in self._notes:
            return False, f"unknown note {note_id}"
        self._notes[note_id] = "" if new_content is None else str(new_content)
        return True, None

    # ---- Collection creation ----------------------------------------------
    def create_collection(self, character_name: str, content: Any,
                          format_type: str, source_skill: str,
                          source_value: str, collection_name: str,
                          extra_props: Dict
                          ) -> Tuple[bool, Optional[str], Optional[str],
                                     Optional[Tuple[int, int]]]:
        cid = f"StubCollection_{next(self._counter)}"
        self._collections[cid] = []
        self._creation_order.append(cid)
        # OODA tools sometimes pass initial content as a list of note IDs.
        if isinstance(content, list):
            for item in content:
                if isinstance(item, str) and item.startswith("StubNote_"):
                    self._collections[cid].append(item)
        return True, cid, None, None

    def add_to_collection(self, collection_id: str, note_id: Optional[str],
                          agent_name: str, operation: str = 'add',
                          content: Optional[List] = None
                          ) -> Tuple[bool, Optional[int], Optional[str]]:
        if collection_id not in self._collections:
            return False, None, f"unknown collection {collection_id}"
        if note_id is not None:
            self._collections[collection_id].append(note_id)
        elif content:
            for item in content:
                if isinstance(item, str):
                    self._collections[collection_id].append(item)
        return True, len(self._collections[collection_id]), None

    # ---- Persistence (no-op) ----------------------------------------------
    def mark_persistent(self, resource_id: str, character_name: str
                        ) -> Tuple[bool, Optional[str]]:
        return True, None

    # ---- Readback ---------------------------------------------------------
    def get_note_text(self, note_id: str) -> str:
        return self._notes.get(note_id, "")

    def get_collection_text(self, collection_id: str, separator: str = "\n\n---\n\n"
                            ) -> str:
        ids = self._collections.get(collection_id, [])
        chunks = [self._notes.get(nid, "") for nid in ids]
        return separator.join(c for c in chunks if c)

    def get_resource_text(self, rid: str, separator: str = "\n\n---\n\n") -> str:
        if rid in self._notes:
            return self._notes[rid]
        if rid in self._collections:
            return self.get_collection_text(rid, separator=separator)
        return ""

    def all_text(self, separator: str = "\n\n---\n\n") -> str:
        """Concatenate every captured note in creation order."""
        chunks: List[str] = []
        for rid in self._creation_order:
            if rid in self._notes:
                chunks.append(self._notes[rid])
            elif rid in self._collections:
                chunks.append(self.get_collection_text(rid, separator=separator))
        return separator.join(c for c in chunks if c)


def translate_result(result: Any, *,
                     manager: Optional[CapturingResourceManager] = None,
                     text_key: str = "value",
                     empty_text: str = "tool produced no output",
                     ) -> Dict[str, Any]:
    """Translate a legacy `tool()` return dict to chat ReAct shape.

    Resolution order for the success-case text:
      1. `text_key` lookup (e.g. "value", "extra.text", "extra.synthesis").
      2. If the result has a `resource_id` and a `manager` was passed,
         read that note/collection from the manager.
      3. fall back to `empty`.

    Args:
        result: the dict returned by the legacy `tool()` call.
        manager: the CapturingResourceManager that was passed to the
            tool. If the tool's actual output went through note/collection
            creation, the text lives there, not in `result`.
        text_key: which field of the success result carries the text.
            Dotted lookup supported (e.g. "extra.text").
        empty_text: fallback message when status=success but no text
            can be recovered.

    Returns: `{"status": "ok"|"error"|"empty", "text": str}`.
    """
    if not isinstance(result, dict):
        return {"status": "error",
                "text": f"tool returned non-dict ({type(result).__name__})"}

    status = result.get("status")
    if status == "success":
        # 1) direct field
        text = _pluck(result, text_key)
        if not text and manager is not None:
            rid = result.get("resource_id")
            if rid:
                text = manager.get_resource_text(rid)
        if not text:
            return {"status": "empty", "text": empty_text}
        return {"status": "ok", "text": str(text)}

    reason = result.get("reason") or result.get("value") or "failed"
    return {"status": "error", "text": str(reason)}


def _pluck(d: Dict[str, Any], dotted_key: str) -> Any:
    """Read `a.b.c` from a nested dict. Returns "" on miss."""
    cur: Any = d
    for part in dotted_key.split("."):
        if not isinstance(cur, dict):
            return ""
        cur = cur.get(part)
    return cur or ""


def make_llm_generate(backend):
    """Adapt `_ChatBackend.chat()` to the `llm_generate(messages=[str|dict], ...)`
    shape OODA tools expect. Returns a callable suitable to pass as the
    `llm_generate` kwarg.

    OODA tools call it as:
        response = llm_generate(messages=[prompt_str], max_tokens=..., ...)
        result = response.text

    _ChatBackend wants `messages=[{role, content}]` and returns a bare
    string. This shim translates both ends.
    """
    from types import SimpleNamespace

    if backend is None:
        return None

    def llm_generate(messages, max_tokens=600, temperature=0.4,
                     is_json=False, stops=None, **kw):
        msg_list = []
        for m in messages or []:
            if isinstance(m, str):
                msg_list.append({"role": "user", "content": m})
            elif isinstance(m, dict):
                msg_list.append(m)
        try:
            text = backend.chat(
                msg_list,
                max_tokens=max_tokens,
                temperature=temperature,
                stops=stops or [],
                is_json=is_json,
            )
        except Exception as e:
            return SimpleNamespace(text="", success=False, error=str(e))
        return SimpleNamespace(text=text or "", success=True, error=None)

    return llm_generate


def build_tool_kwargs(*, character_name: Optional[str], backend,
                      manager: Optional[CapturingResourceManager] = None,
                      world_name: Optional[str] = None,
                      **extra) -> Dict[str, Any]:
    """Construct the standard kwargs dict OODA tools expect from a chat
    react_invoke call. Each react_invoke wraps:

        result = tool(input_value, **build_tool_kwargs(
            character_name=character_name, backend=backend,
            manager=mgr, predicate=args.get('predicate'), ...))

    Returns a dict with `executor` (carrying `world_name` if provided),
    `resource_manager`, `agent_name`, `llm_generate`, plus any
    tool-specific kwargs passed in `**extra`.
    """
    kw: Dict[str, Any] = {
        "executor": StubExecutor(world_name=world_name),
        "resource_manager": manager if manager is not None else CapturingResourceManager(),
        "agent_name": character_name or "chat",
        "llm_generate": make_llm_generate(backend),
    }
    if world_name is not None:
        kw["world_name"] = world_name
    kw.update(extra)
    return kw
