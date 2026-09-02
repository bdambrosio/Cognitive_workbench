"""Discovered-tool registry (src/tools/*/SKILL.md + tool.py react_invoke):
discovery, lazy loading, dispatch, and prompt catalog. ToolsMixin for
ChatLoop, moved verbatim from chat_loop.py in the 2026-06 mixin refactor."""

from __future__ import annotations

import logging
import os
from typing import Any, Dict, List, Optional, Tuple

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
_TOOLS_DIR = os.path.join(_SRC_DIR, 'tools')

logger = logging.getLogger('chat_loop')


class ToolsMixin:
    """Mixin for ChatLoop — moved verbatim from chat_loop.py."""

    # ------------------------------------------------------------------
    # ReAct loop — continuing-computation style.
    # System prompt: persona + companion + tool catalog (stable).
    # User message: conversation history + current input + working log +
    # "Emit next action:" trailer (rebuilt each iteration).
    # The user input is part of the log text, NOT a separate user-role
    # message — that's what stops chat-trained models from defaulting to
    # direct prose reply.
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Tool discovery — scan src/tools/, read SKILL.md frontmatter, register
    # tools that expose `react_invoke`. To add a tool: drop a dir under
    # src/tools/<name>/ with Skill.md (frontmatter: name, description,
    # args) and tool.py (function: react_invoke). No chat_loop edit
    # required.
    #
    # Tools without `react_invoke` (e.g. filter-semantic — kept in
    # src/tools/ for future enablement but list-shape input not yet
    # settled) are silently skipped — they won't appear in the catalog
    # or be dispatchable.
    # ------------------------------------------------------------------

    def _discover_tools(self) -> Dict[str, Dict[str, Any]]:
        """Scan src/tools/ and return a registry: {name → metadata}.

        Each metadata dict carries `description`, `args` (dict from
        Skill.md frontmatter), `module_path`, and `body` (Skill.md body
        text, for any future use). Tools whose `tool.py` lacks
        `react_invoke` are excluded.
        """
        import yaml
        registry: Dict[str, Dict[str, Any]] = {}
        if not os.path.isdir(_TOOLS_DIR):
            return registry
        for entry in sorted(os.listdir(_TOOLS_DIR)):
            tool_dir = os.path.join(_TOOLS_DIR, entry)
            if not os.path.isdir(tool_dir):
                continue
            skill_path = os.path.join(tool_dir, "SKILL.md")
            if not os.path.isfile(skill_path):
                skill_path = os.path.join(tool_dir, "Skill.md")
            tool_path = os.path.join(tool_dir, "tool.py")
            if not os.path.isfile(skill_path) or not os.path.isfile(tool_path):
                continue
            try:
                with open(skill_path, "r", encoding="utf-8") as f:
                    content = f.read()
            except Exception as e:
                logger.warning(f"[{self.character_name}] could not read {skill_path}: {e}")
                continue
            # Quick `react_invoke` check by file text — cheap; avoids
            # importing the module (which may have heavy side-effects
            # or missing deps).
            try:
                with open(tool_path, "r", encoding="utf-8") as f:
                    tool_src = f.read()
            except Exception as e:
                logger.warning(f"[{self.character_name}] could not read {tool_path}: {e}")
                continue
            if "def react_invoke" not in tool_src:
                logger.debug(
                    f"[{self.character_name}] discover_tools: {entry} has no "
                    f"react_invoke; skipping")
                continue
            # Parse frontmatter.
            if not content.startswith("---"):
                logger.warning(
                    f"[{self.character_name}] {skill_path}: no frontmatter")
                continue
            parts = content.split("---", 2)
            if len(parts) < 3:
                logger.warning(
                    f"[{self.character_name}] {skill_path}: malformed frontmatter")
                continue
            try:
                fm = yaml.safe_load(parts[1]) or {}
            except Exception as e:
                logger.warning(
                    f"[{self.character_name}] {skill_path}: yaml parse failed: {e}")
                continue
            name = str(fm.get("name") or entry).strip()
            description = str(fm.get("description") or "").strip()
            args_spec = fm.get("args") or {}
            if not isinstance(args_spec, dict):
                logger.warning(
                    f"[{self.character_name}] {skill_path}: `args` must be a mapping")
                args_spec = {}
            if not description:
                logger.warning(
                    f"[{self.character_name}] {skill_path}: missing `description`")
                continue
            if name in registry:
                logger.warning(
                    f"[{self.character_name}] duplicate tool name {name!r}; "
                    f"keeping first")
                continue
            registry[name] = {
                "description": description,
                "args": args_spec,
                "module_path": tool_path,
                "body": parts[2].strip(),
            }
        return registry

    def _render_discovered_tool_entry(self, name: str, meta: Dict[str, Any]) -> str:
        """Render one discovered tool's catalog entry as a prompt block."""
        # Build the example JSON shape from args.
        arg_keys = list((meta.get("args") or {}).keys())
        # Lead with `thought` + `tool`, then args by key.
        shape_parts = ['"thought": "<one terse sentence>"', f'"tool": "{name}"']
        for k in arg_keys:
            shape_parts.append(f'"{k}": <...>')
        shape = "{" + ", ".join(shape_parts) + "}"
        lines = [f"`{shape}` — {meta['description']}"]
        if arg_keys:
            arg_lines = []
            for k, v in (meta.get("args") or {}).items():
                arg_lines.append(f"    {k}: {v}")
            lines.append("  args:\n" + "\n".join(arg_lines))
        return "\n  ".join(lines) if len(lines) > 1 else lines[0]

    def _load_discovered_tool_module(self, name: str):
        """Lazy-load a discovered tool's module. Cached on the instance."""
        if not hasattr(self, "_tool_module_cache"):
            self._tool_module_cache: Dict[str, Any] = {}
        if name in self._tool_module_cache:
            return self._tool_module_cache[name]
        meta = self._discovered_tools.get(name)
        if meta is None:
            return None
        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                f"_chat_tool_{name.replace('-', '_')}", meta["module_path"])
            if spec is None or spec.loader is None:
                self._tool_module_cache[name] = None
                return None
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._tool_module_cache[name] = mod
            return mod
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] could not load tool {name}: {e}")
            self._tool_module_cache[name] = None
            return None

    def _canonical_tool_name(self, tool: Any) -> Optional[str]:
        """Resolve an emitted tool name against the discovered registry,
        tolerating hyphen/underscore drift ('fetch_text' → 'fetch-text').
        Exact match wins; the separator-swapped spelling is the only
        fallback. Returns None when neither spelling is registered, or
        when the tool is omitted for this scenario.

        Omitted tools resolve to None rather than being dispatched. Dropping
        them from the catalog alone is not enough: the last few turns'
        working logs are re-injected into every prompt, so a tool used
        before an ablation — or before an embodiment switch — is still
        demonstrated in context and gets re-emitted from memory (live miss:
        turn 2360, fac-status called under --world with the fac-* family
        already dropped). Returning None routes it to react.py's
        unknown-tool branch, whose error lists what *is* available, so the
        correction is self-serving. It also makes the cspred ablation cells
        honest — an ablated tool is now genuinely unreachable, not merely
        unadvertised."""
        if not isinstance(tool, str):
            return None
        omitted = set(self._omitted_tools or [])
        for cand in (tool, tool.replace('_', '-'), tool.replace('-', '_')):
            if cand in self._discovered_tools:
                return None if cand in omitted else cand
        return None

    def _dispatch_discovered_tool(self, name: str, action: Dict[str, Any],
                                  log: List[Tuple[str, str]]) -> str:
        """Resolve $stepN args, load the tool module, call react_invoke,
        translate the result into an OK:/EMPTY:/ERROR: observation."""
        mod = self._load_discovered_tool_module(name)
        if mod is None:
            return f"ERROR: tool {name} failed to load (see warning log)"
        invoke = getattr(mod, "react_invoke", None)
        if invoke is None:
            return f"ERROR: tool {name} has no react_invoke entry-point"
        # Strip the framing keys; resolve any $stepN bindings in remaining args.
        tool_args: Dict[str, Any] = {}
        for k, v in action.items():
            if k in ("thought", "tool"):
                continue
            if isinstance(v, str):
                tool_args[k] = self._resolve_react_value(v, log)
            else:
                tool_args[k] = v
        try:
            result = invoke(
                tool_args,
                character_name=self.character_name,
                backend=self.backend,
                logger=logger,
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] tool {name} raised: {e}")
            return f"ERROR: {name} raised: {e}"
        if not isinstance(result, dict):
            return f"ERROR: {name} returned non-dict ({type(result).__name__})"
        status = result.get("status")
        text = str(result.get("text", "")).strip()
        # Structured provenance (source URLs / queries / paths) returned by
        # the tool. Never enters the prompt text — stashed for the ReAct
        # loop to attach to this iter's trace record, mirroring the
        # `_pending_tool_image` single-slot pattern.
        meta = result.get("meta")
        if isinstance(meta, list) and meta:
            self._pending_tool_meta = {"tool": name, "meta": meta}
        if status == "ok":
            # A tool may optionally return an image for the model to SEE
            # (e.g. camera-capture). The image bytes never enter the text log:
            # we stash a data-URI in the single most-recent slot and the ReAct
            # loop injects it into the next iteration's multimodal content
            # array. A text breadcrumb keeps the log self-explaining.
            img = result.get("image")
            if isinstance(img, dict) and img.get("data_uri"):
                label = str(img.get("label") or "image").strip()
                self._pending_tool_image = {"data_uri": img["data_uri"],
                                            "label": label}
                text = (text + f" [{label} attached as an image below]").strip()
            return "OK: " + text
        if status == "empty":
            return "EMPTY: " + (text or f"{name} produced no result")
        return "ERROR: " + (text or f"{name} failed")

    def _build_react_tool_catalog(self) -> str:
        """Build the numbered tool list shown in the ReAct system prompt.
        Conditionally includes inspect_external — and the bound external
        repo path — only when an external_repo is currently bound.
        The `yield` action (intentional multi-loop continuation) is
        offered on every turn: autonomous fires continue via a successor
        concern; user turns spawn a fresh agent_concern carrying the
        remainder, so user-assigned work continues without prodding. Tools
        whose name appears in self._omitted_tools (set per scenario via
        chat.omitted_tools, used by the cspred bench cf-cells) are
        filtered out so the affordance representation matches what is
        actually invokable. Numbers shift accordingly; the model handles
        all layouts fine."""
        tools: List[Tuple[str, str]] = [
            ("process_text",
             "`{\"thought\": \"<one terse sentence>\", \"tool\": \"process_text\", \"source\": <string|$stepN>, \"instruction\": <string>}` — "
             "LLM pass over text in context. Use to formulate queries, render results in your voice, extract info."),
            ("recall",
             "`{\"thought\": \"<one terse sentence>\", \"tool\": \"recall\", \"query\": <string>}` — "
             "READ-ONLY active recall over your own memory: full reasoning trace, conversation history, "
             "current companion model, current discourse state, log of persisted memories. Use when the "
             "user asks about prior turns ('what did we discuss?', 'you mentioned X earlier'), your "
             "earlier reasoning, or persistent state that may not be in your current prompt. The query "
             "is opaque to you — a subagent reads the files and returns a synthesized answer in `$stepN`. "
             "Cannot write: if the user says 'remember X' / 'note Y' / 'save Z', acknowledge in `respond` "
             "without claiming persistence — memory writes happen via background reflection, not from "
             "inside this loop."),
            ("inspect",
             "`{\"thought\": \"<one terse sentence>\", \"tool\": \"inspect\", \"query\": <string>}` — "
             "query your own codebase. A separate subagent (geofenced read-only to the repo root — `src/`, "
             "`docs/`, `tests/`, `bench/`; list/read/grep primitives) navigates the tree and returns a "
             "synthesized answer with file:line citations. Use when the user asks how you work, where something "
             "is implemented, what a module does, to read a design doc, or to verify a claim about your own "
             "code. **Named paths are fair game**: a request to read a specific file under the repo root "
             "(`docs/foo.md`, `bench/x/README.md`, `src/chat/bar.py`) belongs here — put the path in the query "
             "(e.g. \"read docs/learned-disposition-design.md and summarize it\"). This is the ONLY tool for "
             "paths under your own repo; `inspect_external` reaches a different tree entirely and cannot see "
             "them. Otherwise phrase as a natural-language question (e.g. \"where is the ReAct dispatch "
             "defined?\")."),
        ]
        external_repo = self._get_external_repo()
        if external_repo is not None:
            tools.append(("inspect_external",
                "`{\"thought\": \"<one terse sentence>\", \"tool\": \"inspect_external\", \"query\": <string>, "
                "\"claims\": [<int>, ...]}` — "
                f"query the external project repo currently bound to this session: `{external_repo}`. "
                "`claims` is optional: when the query gathers evidence for enumerated claims, list their ids "
                "and the record of this request is filed under them. "
                "Same primitives as `inspect` (list/read/grep), different geofence — this is reading an external "
                "codebase as documentation, not introspection of your own substrate. Use ONLY when the user asks "
                "questions about THAT project (its code, README, structure, behavior). It cannot see your own "
                f"repo: a path like `docs/x.md` resolves under `{external_repo}` and will simply be missing — "
                "if the user names a file that is part of YOUR substrate, use `inspect` instead. Phrase as a "
                "natural-language question (e.g. \"how does this project structure its modules?\", "
                "\"what does the README say about installation?\", \"where is the main entry point?\")."))
        tools.append(("security",
            "`{\"thought\": \"<one terse sentence>\", \"tool\": \"security\", \"query\": <string>}` — "
            "investigate LAN state or local host security state. A separate subagent (read-only typed "
            "primitives: nmap host discovery + service scan, ss/ip for TCP+UDP sockets/routes/arp/interfaces "
            "with targets restricted to RFC1918 ranges; host probes for processes, logins, ssh auth failures, "
            "privilege events, SUID files, cron/systemd persistence points, user-level persistence "
            "(autostart, user units), accounts, authorized SSH keys, sshd config, containers, pending "
            "upgrades, the unattended-upgrades log, installed package versions; baseline diffing of SUID "
            "files / listening sockets / accounts / autostart / enabled units against the last stored "
            "snapshot; and — where a fenced sudoers entry is installed — the firewall ruleset, listener "
            "process attribution, and AppArmor detail) runs the probes and returns a synthesized answer. "
            "Use for LAN hosts, what's listening and what's exposed to the network, signs of intrusion or "
            "unexpected host activity, persistence-point review, and patch posture. No traffic capture, no "
            "root-only rootkit scanners. Phrase as a natural-language question (e.g. \"what hosts are on my "
            "LAN at 192.168.1.0/24?\", \"any signs of intrusion in the last day?\", \"what's listening on "
            "0.0.0.0 and does the firewall allow it?\")."))
        tools.append(("justify",
            "`{\"thought\": \"<one terse sentence>\", \"tool\": \"justify\", \"turn_seq\": <int, optional>}` — "
            "READ-ONLY provenance audit of one of your own replies in this conversation. Returns that "
            "reply's factual claims, each typed by grounding (retrieved / memory / user_asserted / context / "
            "inferred / model_prior) with the recorded evidence resolved: search sources with URLs, recalled "
            "memory notes with dates, the user's own words. Use when the user asks you to justify or source "
            "what you said ('justify your response', 'why should I believe that?', 'where did that come "
            "from?'). "
            "Omit `turn_seq` for your most recent reply — the normal case. Each reply is displayed to the "
            "user with its turn number, so they can name an OLDER one ('justify 2393', 'justify what you "
            "said in 2384'): pass exactly the number they gave. NEVER pass a number you inferred, "
            "remembered, or read off a memory note. If they refer to an earlier reply without giving a "
            "number, ASK which turn number rather than guessing — a trail for the wrong turn is real, "
            "well-formed, and about the wrong text, which is worse than no trail. The audit names the "
            "question it is answering; if that does not match what the user asked about, say so and stop "
            "instead of presenting it. NEVER reconstruct a provenance trail from recall or "
            "conversation memory — a remembered paraphrase is not a trail, and presenting one as "
            "justification defeats the audit. Present the returned "
            "trail faithfully: cite only the evidence it contains, never attribute sources to the original "
            "reply that the trail does not record. "
            "The source URLs and the verbatim quotes ARE the deliverable — a reply that describes evidence "
            "instead of naming it ('a comparative study', 'an academic analysis', 'a report on fatalities') "
            "is not a provenance reply at all. Carry each cited source's URL into your answer, and the quoted "
            "span for the claims that turn on one; the user has to be able to check them. "
            "Attribute at the granularity the trail records and no finer: when several claims cite ONE "
            "blended search step, say they came from that search over the sources it lists — do NOT pair "
            "individual claims to individual sources unless the trail records that pairing, and never name a "
            "source the trail does not contain. A quote is verbatim against the tool observation, which for a "
            "search step is a synthesis over its sources rather than the sources themselves — say so plainly "
            "instead of implying you read the primary documents. "
            "Report the grades as given and do NOT explain them: they are computed deterministically from "
            "grounding plus tags, so any rationale you offer for why something is graded as it is would be "
            "invented. Say plainly when a claim is model_prior (your "
            "background knowledge) or inferred rather than retrieved. Then AUDIT the trail before replying: "
            "each claim carries an ordinal grade (sourced > probable > unverified > suspect) and any "
            "'Audit note:' lines name specific checks for the weaknesses found — perform those checks with "
            "tools in this same turn, prioritizing the weakest link (e.g. verify a volatile model_prior "
            "fact; re-probe a negation drawn from an inadequate query). "
            "Report what you verify as a check performed now, separate from the recorded trail — "
            "and if it contradicts the original reply, lead with the correction rather than defending the "
            "reply. If justify returns EMPTY, relay the stated reason honestly."))
        # Auto-discovered tools from src/tools/ (each exposes react_invoke).
        # Rendered between the built-in subagents and the display/respond
        # actions so the prompt's tool ordering matches its mental model:
        # think (process_text) → recall/inspect/security → real-world tools
        # (search-web, fetch-text, calculate, …) → present (display) →
        # finish (respond).
        for _name, _meta in self._discovered_tools.items():
            tools.append((_name, self._render_discovered_tool_entry(_name, _meta)))
        # Agent-to-agent messaging — only when the scenario has co-resident
        # peers (launcher injects config['peers'] from the character list).
        peers = getattr(self, '_peers', None) or []
        if peers:
            tools.append(("agent-say",
                "`{\"thought\": \"<one terse sentence>\", \"tool\": \"agent-say\", "
                "\"to\": <agent name>, \"text\": <string|$stepN>}` — "
                f"send a message to a co-resident agent (available: {', '.join(peers)}). "
                "Non-terminal — the loop continues; follow with `respond` (or a silent "
                "exit on autonomous runs) as usual. The peer processes your message as "
                "its own turn; its reply, if any, arrives LATER as a new turn from that "
                "agent's name — never wait for it inside this loop. Use agent-say to "
                "INITIATE contact, delegate, or ask a peer something. When you are "
                "ANSWERING a turn that came FROM an agent, just use `respond` — it is "
                "delivered back to that agent automatically; agent-say would "
                "double-send. Exchanges carry a hard hop budget, so long ping-pong "
                "conversations get cut off: make each message complete and "
                "self-contained rather than chatty."))
        tools.append(("display",
            "`{\"thought\": \"<one terse sentence>\", \"tool\": \"display\", "
            "\"content\": <string|$stepN>, \"format\": \"markdown\"|\"html\"}` — "
            "**USE THIS** whenever the user asks to *show / display / draw / render / visualize / "
            "put on screen*, OR whenever the answer is fundamentally structured (multi-row tables, "
            "diagrams via inline SVG, side-by-side comparisons, multi-section walk-throughs). "
            "Pushes rich content to the canvas window so the user can SEE it, not just read it. "
            "Non-terminal — the loop continues; immediately follow with a brief `respond` so the "
            "chat conversation gets a textual reply too (e.g. \"posted it on screen — let me know if "
            "you want X\"). "
            "**Even if your data is incomplete or imperfect, render what you DO have** — the user "
            "asked to see it; don't decline because the picture isn't full. Acknowledge the gap "
            "verbally in `respond`. "
            "Default `format=markdown` (safer, easier to author). Use `format=html` for inline "
            "SVG (diagrams, drawings, illustrations), multi-column layout, or custom styling. "
            "**Line breaks matter**: markdown needs real `\\n` newlines in the content string "
            "for headings, lists, and paragraphs to render — don't collapse your source to a "
            "single line, or `### Title` and `- bullet` show up as literal text. "
            "**Drawings / illustrations**: when the user says *draw / sketch / illustrate / "
            "diagram*, generate inline `<svg>...</svg>` yourself — cheap, immediate, no "
            "external dependency. Image search is for *photos* or specific existing images "
            "the user named — not for whimsical drawings you can produce yourself. "
            "**Photos from the web**: `format=html` with "
            "`<img src=\"http://127.0.0.1:8789/proxy?url=<url>\">`. The URL must be a "
            "**direct image-file URL** (e.g. `cdn.example.com/photo.jpg`), NOT a webpage that "
            "contains the image. Page URLs through the proxy produce broken-image icons — "
            "the proxy faithfully returns whatever bytes the URL yields, and `text/html` "
            "cannot render inside `<img>`. If you only have a page URL (a search hit, a "
            "stock-photo landing page, an article), `fetch-text` it first and use "
            "`metadata.html_metadata.images.og_image` — that's what that field is for."))
        tools.append(("respond",
            "`{\"thought\": \"<one terse sentence>\", \"tool\": \"respond\", \"text\": <string|$stepN>}` — "
            "final reply, exits loop. Must be in your voice; pass search-web/fetch-text results through process_text "
            "first or write the reply yourself. "
            "Describe only work you ACTUALLY DID in this run — the actions above are the "
            "whole record of it. Never report being partway through, having vetted or "
            "polished or narrowed something, unless the steps that did so are visibly in "
            "this run. If you owe work you have not started, either start it now or exit "
            "with `yield`; do NOT `respond` with a progress report on work that has not "
            "happened. Saying 'I haven't started this yet' is always available and always "
            "better than inventing a status."))
        from chat.react import REACT_MAX_ITERS
        tools.append(("yield",
            "`{\"thought\": \"<one terse sentence>\", \"tool\": \"yield\", "
            "\"next\": <string>, \"text\": <string, optional>}` — "
            f"this run has a hard cap of {REACT_MAX_ITERS} actions. When the task genuinely "
            "needs more than the remaining budget, or the next step must wait on a slow "
            "external process (smelting, a long transfer, someone else acting), end at a "
            "CLEAN boundary with `yield` instead of stopping with the plan unexecuted: a "
            "follow-up run will execute `next` as its instruction, without anyone prodding. "
            "Write `next` imperatively and self-contained — state what is "
            "already done, what remains, and every specific the follow-up must not rediscover "
            "(names, ids, paths, values found so far). `text` posts a brief status "
            "line to the conversation — ALWAYS include it when answering a user turn, so they "
            "hear where things stand; on autonomous runs omit it to stay silent. Exits the "
            "loop. Use `respond` when the work is DONE. Work remaining is not by itself a "
            "reason to `yield`: while you can still make progress in this run, keep "
            "working. `yield` is for when you cannot."))

        omitted = set(self._omitted_tools or [])
        if omitted:
            kept = [(name, descr) for name, descr in tools if name not in omitted]
            dropped = [name for name, _ in tools if name in omitted]
            logger.info(f"[{self.character_name}] _build_react_tool_catalog: omitted_tools active, dropped={dropped}")
            tools = kept

        numbered = "\n".join(f"{i}. {descr}" for i, (_, descr) in enumerate(tools, start=1))
        return "Tools (each emission picks ONE):\n" + numbered + "\n"
