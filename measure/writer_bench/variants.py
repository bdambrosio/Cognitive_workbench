"""The writing-step options the bench compares, each as a function from a
case to a reply plus the calls it made.

A case is the exact prompt (system, user) of the call that wrote one of
Jill's replies; see cases.py. Every option starts from the same first call
the live loop makes. The options and what each changes:

  base     the call as it was made, re-run.
  cheap    1a + 1b + 2e: timestamps on each history turn, a sheet of facts
           about herself generated from config, and a rule against
           unlooked-up incidental specifics. Prompt text only; no extra call.
  points   2b: the respond action lists its points with a source for each
           before the text. Prompt text only.
  writer   2a: the first call chooses `respond` as before; a second call
           writes the reply as plain prose, not as a JSON string.
  precheck 3a: the claims in the base draft are attributed against the
           turn's record (src/chat/claims.attribute_claims, the post-turn
           pass run before sending); a revise call drops each unsupported
           specific or marks it as from memory, adding nothing.
  cheap_points  cheap and points together.
  agree    3b: two more drafts of the base call; a compare call lists the
           specifics of the first draft the others lack or contradict; the
           same revise call drops or marks them.

The bench cannot run tools. When a first call picks a tool other than
`respond`, the tool is recorded and the call is asked again with a note that
tools are unavailable, so every option ends in a reply. How often an option
chooses to look something up is itself one of the measurements.
"""
from __future__ import annotations

import json
import logging
import re
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import yaml

REPO = Path(__file__).resolve().parents[2]
CONVERSATION = REPO / "scenarios/jill_chat/Jill/memory/conversation.txt"

NO_TOOLS = ("\n\n[Bench replay: tools cannot run here. Emit the respond "
            "action now, with the reply you would send.]")

# 2e. Wording is a draft for review, not Jill's text.
SAY_LESS = """
## Specific details in a reply
A number, date, name, credit, quotation, citation, verse number, version or
specification in a reply either comes from this turn's observations or the
prompt, or is said to be from memory. When it is not needed for the answer —
an illustration, an aside, background colour — leave it out. A short reply
that says only what you have is better than a long one that fills the gaps.
"""

# 2b. Wording is a draft for review.
POINTS = """
## Points before text
When the action is `respond`, put a `points` list before `text`. Each item
is {"point": <what you will say, one line>, "source": <"$stepN", "prompt",
"user", "calculation" or "memory">}. Then write `text` from those points
only. A point whose source is "memory" is left out of `text` or marked there
as from memory.
"""

WRITER = """You have chosen to reply. Write the reply to {who} now, as plain
prose, exactly as it will be sent: not JSON, no preamble. Take the room you
need to decide what to say before you say it. Every specific detail comes
from this turn's observations or the prompt, or is said to be from memory."""

REVISE = """Below is a draft reply and a list of specific details in it
that nothing in this turn's observations or prompt supports. Rewrite the
draft: remove each listed detail unless the reply needs it to answer the
question. Where a needed detail is kept, say once in that paragraph that the
figures there are from memory; do not tag each phrase. Change nothing else.
Add no new detail of any kind. Output only the rewritten reply, as plain
prose.

DRAFT:
{draft}

UNSUPPORTED DETAILS:
{items}"""

COMPARE = """Three drafts of the same reply follow. List every specific
detail in DRAFT A (a number, date, name, credit, quotation, citation,
specification) that DRAFT B or DRAFT C leaves out or states differently.
Output one detail per line, quoted as it appears in DRAFT A, and nothing
else. If there are none, output NONE.

DRAFT A:
{a}

DRAFT B:
{b}

DRAFT C:
{c}"""


# ---------------------------------------------------------------- helpers

def _parse(raw: str) -> Optional[Dict[str, Any]]:
    import sys
    sys.path.insert(0, str(REPO / "src"))
    from utils.json_utils import repair_json_string
    try:
        obj = json.loads(raw or "")
    except ValueError:
        obj = repair_json_string(raw or "")
    return obj if isinstance(obj, dict) else None


class _Usage(logging.Handler):
    """Keeps the last <llm-usage> line the backend logged on each thread, so
    each call's token counts can be recorded beside its wall time."""

    def __init__(self):
        super().__init__()
        self.last: Dict[int, str] = {}

    def emit(self, record):
        self.last[threading.get_ident()] = record.getMessage()


USAGE = _Usage()
logging.getLogger("chat.backend.usage").addHandler(USAGE)
logging.getLogger("chat.backend.usage").setLevel(logging.INFO)
_NUM = re.compile(r"(prompt|cached|completion|reasoning)=(\d+)")


class Caller:
    """One backend per worker thread; records every call's name and time."""

    def __init__(self, make_backend: Callable[[], Any]):
        self.backend = make_backend()
        self.calls: List[Dict[str, Any]] = []

    def chat(self, name: str, messages, schema: bool, effort: Optional[str] = "medium",
             max_tokens: int = 12288) -> str:
        from chat.react import REACT_ACTION_SCHEMA
        t0 = time.time()
        raw = self.backend.chat(messages, max_tokens=max_tokens, temperature=None,
                                cot_profile="none", reasoning_effort=effort,
                                response_schema=REACT_ACTION_SCHEMA if schema else None)
        usage = {k: int(v) for k, v in _NUM.findall(USAGE.last.pop(threading.get_ident(), ""))}
        self.calls.append({"name": name, "secs": round(time.time() - t0, 2),
                           "finish": getattr(self.backend, "last_finish_reason", None),
                           **usage, "raw": raw or ""})
        return raw or ""


def _action(c: Caller, name: str, msgs) -> Dict[str, Any]:
    """One action emission with the live loop's retries (src/chat/react.py):
    up to two more attempts when the output does not parse, the token budget
    doubled once when the cause was the limit."""
    budget = 12288
    for attempt in range(3):
        act = _parse(c.chat(name if not attempt else f"{name}+retry{attempt}", msgs,
                            schema=True, max_tokens=budget))
        if act is not None:
            return act
        if c.calls[-1].get("finish") in ("length", "max_tokens") and budget == 12288:
            budget *= 2
    return {}


def _resolve(text: str, user: str) -> str:
    """A reply given as `$stepN` is that step's observation, prefix stripped,
    as the live loop substitutes it (react._resolve_react_value)."""
    m = re.fullmatch(r"\s*(\$step\d+)\s*", text or "")
    if not m:
        return text
    log = user[user.find("## Working log"):]
    hit = re.search(re.escape(m.group(1)) + r":\n(.*?)(?=\nACTION \d+:|\n\$step\d+:|\n## End of working log|\Z)",
                    log, flags=re.S)
    body = hit.group(1).strip() if hit else ""
    return re.sub(r"^(OK|EMPTY|ERROR):\s*", "", body)


def respond_call(c: Caller, name: str, system: str, user: str) -> Tuple[str, Optional[str]]:
    """The ReAct action call; returns (reply text, tool chosen if not respond)."""
    msgs = [{"role": "system", "content": system}, {"role": "user", "content": user}]
    act = _action(c, name, msgs)
    chose = None
    if act.get("tool") != "respond":
        chose = str(act.get("tool"))
        msgs[1]["content"] = user + NO_TOOLS
        act = _action(c, name + "+forced", msgs)
    return _resolve(str(act.get("text") or ""), user), chose


def plain_call(c: Caller, name: str, system: str, user: str, effort: str = "medium") -> str:
    raw = c.chat(name, [{"role": "system", "content": system},
                        {"role": "user", "content": user}], schema=False, effort=effort)
    return re.sub(r"(?s)^.*?</think>", "", raw).strip()


# ------------------------------------------------- 1a: timestamps on history

def _conversation_index() -> List[Tuple[datetime, str]]:
    out = []
    text = CONVERSATION.read_text(encoding="utf-8")
    for m in re.finditer(r"^\[(\S+ \S+) UTC\] (\S+) (->|<-) Jill[^\n]*\n=+\n(.{0,120})",
                         text, flags=re.M | re.S):
        out.append((datetime.fromisoformat(m.group(1)), " ".join(m.group(4).split())))
    return out


_CONV: Optional[List[Tuple[datetime, str]]] = None


def add_timestamps(user: str, case_ts: str) -> str:
    """Prefix each turn of the history block with its time from
    conversation.txt, matched on the turn's opening text; a turn that cannot
    be matched is left as it is."""
    global _CONV
    if _CONV is None:
        _CONV = _conversation_index()
    head = "## Conversation history (verbatim session turns)\n"
    i = user.find(head)
    if i < 0:
        return user
    j = user.find("\n## ", i + len(head))
    block = user[i + len(head): j if j > 0 else len(user)]
    limit = datetime.fromisoformat(case_ts[:19])
    out = []
    for line in block.split("\n"):
        m = re.match(r"^(User|Jill|[A-Za-z@:._-]+): (.*)", line)
        if m:
            key = " ".join(m.group(2).split())[:60]
            hit = [t for t, txt in _CONV if t <= limit and key and txt.startswith(key[:40])]
            if hit:
                line = f"[{hit[-1]:%Y-%m-%d %H:%M} UTC] {line}"
        out.append(line)
    return user[:i + len(head)] + "\n".join(out) + (user[j:] if j > 0 else "")


# ------------------------------------------- 1b: facts about herself, generated

def self_facts(system: str) -> str:
    """Generated from the scenario file at the commit the turn ran, plus facts
    read from the code on 2026-10-20 (each named with its source)."""
    m = re.search(r"running commit ([0-9a-f]{7,})", system)
    commit = m.group(1) if m else "HEAD"
    try:
        doc = yaml.safe_load(subprocess.run(
            ["git", "show", f"{commit}:scenarios/jill-chat.yaml"], cwd=REPO,
            capture_output=True, text=True, check=True).stdout)
    except Exception:                                          # noqa: BLE001
        doc = yaml.safe_load((REPO / "scenarios/jill-chat.yaml").read_text())
    jill = (doc.get("characters") or {}).get("Jill") or {}
    sensors = ", ".join(f"{s.get('name')} every {s.get('schedule') or s.get('interval')}"
                        for s in jill.get("sensors") or [])
    first = CONVERSATION.read_text(encoding="utf-8")[:400]
    start = re.search(r"\[(\d{4}-\d\d-\d\d)", first)
    return f"""
## Facts about my own setup (generated from config and code; check before contradicting)
- Sensors running this session: {sensors or 'none'} (scenarios/jill-chat.yaml at {commit}).
- My record with User begins {start.group(1) if start else 'unknown'} (conversation.txt).
- Every tool call in a turn, inspect and recall included, runs before the turn
  continues; only `dispatch` runs work in the background (src/chat/react.py).
- exec-script runs in a bubblewrap sandbox (src/tools/exec-script/tool.py).
- After every turn, reflection updates my companion model, shared premises
  and memories, and the claim check runs on my reply (src/chat/chat_loop.py).
"""


# ---------------------------------------------------------------- options

def opt_base(c: Caller, case: Dict[str, Any]) -> Dict[str, Any]:
    text, chose = respond_call(c, "respond", case["system"], case["user"])
    return {"reply": text, "chose_tool": chose}


def opt_cheap(c: Caller, case: Dict[str, Any]) -> Dict[str, Any]:
    system = case["system"] + "\n" + self_facts(case["system"]) + SAY_LESS
    user = add_timestamps(case["user"], case["ts"])
    text, chose = respond_call(c, "respond", system, user)
    return {"reply": text, "chose_tool": chose}


def opt_cheap_points(c: Caller, case: Dict[str, Any]) -> Dict[str, Any]:
    system = case["system"] + "\n" + self_facts(case["system"]) + SAY_LESS + POINTS
    user = add_timestamps(case["user"], case["ts"])
    text, chose = respond_call(c, "respond", system, user)
    return {"reply": text, "chose_tool": chose}


def opt_points(c: Caller, case: Dict[str, Any]) -> Dict[str, Any]:
    text, chose = respond_call(c, "respond", case["system"] + POINTS, case["user"])
    return {"reply": text, "chose_tool": chose}


def opt_writer(c: Caller, case: Dict[str, Any], base: Dict[str, Any]) -> Dict[str, Any]:
    who = re.search(r"## Current input \(from ([^)]+)\)", case["user"])
    note = WRITER.format(who=who.group(1) if who else "the user")
    text = plain_call(c, "writer", case["system"], case["user"] + "\n\n" + note)
    return {"reply": text, "chose_tool": base.get("chose_tool")}


def _record(case: Dict[str, Any], draft: str) -> Dict[str, Any]:
    """The reasoning-trace record of the turn, with the draft as its reply."""
    rec = dict(case["trace_record"])
    rec["raw_response"] = draft
    return rec


def _unsupported(c: Caller, case: Dict[str, Any], draft: str) -> List[str]:
    from chat.claims import attribute_claims
    def llm(messages):
        return c.chat("attribute", messages, schema=False, effort="none")
    claims = attribute_claims(_record(case, draft), llm, character_name="Jill") or []
    # Not grounded in the turn: background knowledge of any volatility, or an
    # inference resting on no recorded evidence. (grade_claim rates "stable"
    # background knowledge as probable, which is right for the live
    # verification trigger and wrong for a check against what she read.)
    return [x.get("claim") for x in claims
            if x.get("grounding") == "model_prior"
            or (x.get("grounding") == "inferred" and not x.get("refs"))]


def revise(c: Caller, case: Dict[str, Any], draft: str, items: List[str]) -> str:
    if not items:
        return draft
    return plain_call(c, "revise", case["system"], REVISE.format(
        draft=draft, items="\n".join(f"- {i}" for i in items)))


def opt_precheck(c: Caller, case: Dict[str, Any], base: Dict[str, Any]) -> Dict[str, Any]:
    items = _unsupported(c, case, base["reply"])
    return {"reply": revise(c, case, base["reply"], items),
            "chose_tool": base.get("chose_tool"), "flagged": items}


def opt_agree(c: Caller, case: Dict[str, Any], base: Dict[str, Any],
              make_backend: Callable[[], Any]) -> Dict[str, Any]:
    # The two extra drafts run in parallel, as they would live; each gets its
    # own backend so their per-call state does not mix.
    extra = [Caller(make_backend), Caller(make_backend)]
    with ThreadPoolExecutor(2) as ex:
        drafts = list(ex.map(lambda e: respond_call(e, "draft", case["system"], case["user"])[0], extra))
    t_parallel = max(sum(x["secs"] for x in e.calls) for e in extra)
    c.calls.append({"name": "drafts(parallel)", "secs": round(t_parallel, 2), "finish": None})
    listed = plain_call(c, "compare", case["system"], COMPARE.format(
        a=base["reply"], b=drafts[0], c=drafts[1]))
    items = [l.strip("-• ").strip() for l in listed.splitlines()
             if l.strip() and l.strip().upper() != "NONE"]
    return {"reply": revise(c, case, base["reply"], items),
            "chose_tool": base.get("chose_tool"), "flagged": items}
