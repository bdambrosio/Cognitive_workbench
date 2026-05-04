"""Network-security subagent — a thin, persona-less ReAct loop that
investigates LAN state via a small set of typed primitives. Mirrors
the inspect.py pattern: argument-list subprocess, geofenced inputs,
read-only operations, OK/EMPTY/ERROR observations.

Used by Jill's chat ReAct loop via the `security` tool. From Jill's
vantage, a `security` call is one step: she emits a query string,
gets a synthesized answer back. The subagent's internal multi-iter
reasoning is opaque to her trace but written to security_traces/ for
debugging.

Architectural notes:
- Backend hardcoded to Sonnet (caller passes a Sonnet-configured
  _ChatBackend), independent of Jill's per-scenario llm_config —
  security reasoning needs depth.
- Primitives are intentionally minimal and read-only:
  discover (nmap host discovery), scan_services (nmap -sV),
  system_state (ss / ip), respond.
- All scan targets are validated against RFC1918 ranges (10/8,
  172.16/12, 192.168/16). Targets outside these are rejected.
- Subprocess calls use argument lists exclusively — never shell=True.
- nmap is the only required external dependency. Suricata/Zeek-style
  log inspection is deferred until those tools are actually configured.
"""

from __future__ import annotations

import ipaddress
import json
import logging
import re
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

_MAX_ITERS = 12
_MAX_OBS_CHARS = 8_000
_NMAP_TIMEOUT_DISCOVERY = 60.0
_NMAP_TIMEOUT_SERVICES = 120.0
_SYSTEM_STATE_TIMEOUT = 5.0

# RFC1918 networks — the only ranges we'll scan. 169.254/16 (link-local),
# 100.64/10 (CGNAT), and 127/8 (loopback) are NOT included; if a future
# threat model warrants them, add deliberately.
_RFC1918_NETWORKS = [
    ipaddress.ip_network('10.0.0.0/8'),
    ipaddress.ip_network('172.16.0.0/12'),
    ipaddress.ip_network('192.168.0.0/16'),
]

# system_state categories → argv. ss/ip are part of iproute2; available
# on virtually every modern Linux distribution. Each command is read-only
# and runs without elevated privileges (some output detail — like PIDs
# in `ss -tlnp` — is reduced when run as a non-privileged user).
_SYSTEM_STATE_COMMANDS: Dict[str, List[str]] = {
    'sockets':    ['ss', '-tlnp'],
    'routes':     ['ip', 'route', 'show'],
    'arp':        ['ip', 'neigh', 'show'],
    'interfaces': ['ip', '-br', 'addr', 'show'],
}


def _build_system_prompt() -> str:
    """Static system prompt. Stable across all calls in a session, so
    the anthropic route's prompt cache stays warm."""
    return (
        "You are a network-security investigation subagent. You answer "
        "questions about LAN state by running read-only probes from a "
        "small set of typed primitives. You have no persona, no goals "
        "beyond the current query, and no memory of past calls — each "
        "invocation is independent.\n"
        "\n"
        "## Threat model framing\n"
        "\n"
        "Three distinct sub-problems map onto different primitives:\n"
        "1. **New / rogue hosts on the LAN.** Use `discover(cidr)` to "
        "enumerate active IPs, compare against expected baseline.\n"
        "2. **Unexpected service exposure.** Use `scan_services(host)` "
        "to inventory open ports + service banners on a known host.\n"
        "3. **Local system state.** Use `system_state(category)` to "
        "see the agent's own listening sockets, routing table, ARP "
        "neighbors, or interface addresses.\n"
        "Traffic-anomaly detection (signature-based IDS) is NOT in this "
        "v0.1 toolset — Suricata/Zeek log inspection lands when those "
        "tools are configured.\n"
        "\n"
        "## Tools (one JSON object per emission)\n"
        "\n"
        '1. {"thought": "<one sentence>", "tool": "discover", "cidr": '
        '"<RFC1918_cidr_or_host>"} — nmap host discovery (`-sn`). cidr '
        "must be in 10.0.0.0/8, 172.16.0.0/12, or 192.168.0.0/16. A "
        "single host (e.g. 192.168.1.1) is also valid. Output: terse "
        "lines like `Host: <ip> Status: Up`. Up to ~60 second runtime.\n"
        '2. {"thought": "...", "tool": "scan_services", "host": '
        '"<RFC1918_ip>"} — nmap service-version scan (`-sV --script=safe`). '
        "host must be a single RFC1918 IP. Output: per-port lines like "
        "`<host> <port>/<state>/<proto>/<service>/<version>`. Up to ~120s.\n"
        '3. {"thought": "...", "tool": "system_state", "category": '
        '"<sockets|routes|arp|interfaces>"} — read-only system probe. '
        "Categories:\n"
        "    `sockets`    — listening TCP sockets (ss -tlnp)\n"
        "    `routes`     — IP routing table (ip route show)\n"
        "    `arp`        — ARP / IPv6 neighbor table (ip neigh show)\n"
        "    `interfaces` — interface addresses (ip -br addr show)\n"
        '4. {"thought": "...", "tool": "respond", "text": "<answer>"} — '
        "final answer to the query, exits the loop.\n"
        "\n"
        "## Discipline\n"
        "\n"
        "- **Read-only.** None of the primitives modify network or "
        "system state. You do not insert firewall rules, kill processes, "
        "or send unsolicited packets beyond nmap's standard probes.\n"
        "- **RFC1918 only.** discover and scan_services reject targets "
        "outside private address ranges. Don't try to scan the internet.\n"
        "- **Pick the right primitive before any tool call:**\n"
        "    * 'What hosts are on my LAN?' → discover with the user's "
        "subnet.\n"
        "    * 'What's running on host X?' → scan_services on that IP.\n"
        "    * 'What ports am I listening on?' → system_state(sockets).\n"
        "    * 'What's my IP / interface?' → system_state(interfaces).\n"
        "    * 'What's my default gateway?' → system_state(routes).\n"
        "- **Privilege caveat.** Running as a regular user, nmap `-sn` "
        "uses TCP/ICMP probes (some hosts may not respond). `-sV` uses "
        "TCP connect (works fine, slightly slower than SYN). `ss -tlnp` "
        "shows PID/program only for processes the current user owns. "
        "Note these limitations honestly in your final answer if "
        "relevant.\n"
        "- **Don't loop blindly.** If discover on a /16 returns 200 "
        "hosts, narrow to a /24 rather than scanning each individually. "
        "If nmap fails for one target, don't retry the same call.\n"
        "- **Cite specifics.** When a port/IP/process matters, name it "
        "in the final answer. The caller may want to verify.\n"
        "- **If you can't answer with these primitives, say so plainly.** "
        "v0.1 deliberately excludes traffic capture, IDS log inspection, "
        "and anything requiring root.\n"
        "- Output ONLY one JSON object per emission. No prose, no "
        "markdown fences."
    )


def _parse_action(raw: str) -> Optional[Dict[str, Any]]:
    s = (raw or '').strip()
    if not s:
        return None
    if s.startswith('```'):
        s = re.sub(r'^```[a-zA-Z]*\n', '', s)
        s = re.sub(r'\n```\s*$', '', s)
    try:
        return json.loads(s)
    except json.JSONDecodeError:
        m = re.search(r'\{.*\}', s, re.DOTALL)
        if m:
            try:
                return json.loads(m.group(0))
            except json.JSONDecodeError:
                return None
        return None


def _is_rfc1918(target: str) -> Optional[ipaddress.IPv4Network]:
    """Validate `target` is a valid IPv4 address or CIDR fully contained
    within RFC1918 ranges. Returns the parsed network on success, None
    otherwise. Single hosts are accepted (treated as /32)."""
    s = (target or '').strip()
    if not s:
        return None
    try:
        net = ipaddress.ip_network(s, strict=False)
    except ValueError:
        return None
    if not isinstance(net, ipaddress.IPv4Network):
        return None
    for r in _RFC1918_NETWORKS:
        if net.subnet_of(r):
            return net
    return None


def _truncate(out: str, label: str) -> str:
    if len(out) <= _MAX_OBS_CHARS:
        return out
    return (out[:_MAX_OBS_CHARS]
            + f"\n…(truncated; {len(out)} chars total, capped at "
            f"{_MAX_OBS_CHARS}; narrow the {label})")


def _tool_discover(cidr: str) -> str:
    if shutil.which('nmap') is None:
        return ("ERROR: nmap is not on PATH — install with "
                "`apt install nmap`")
    net = _is_rfc1918(cidr)
    if net is None:
        return (f"ERROR: discover target {cidr!r} is not a valid RFC1918 "
                "IPv4 address or CIDR (must be inside 10.0.0.0/8, "
                "172.16.0.0/12, or 192.168.0.0/16)")
    cmd = ['nmap', '-sn', '-oG', '-', '--', str(net)]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=_NMAP_TIMEOUT_DISCOVERY, check=False,
        )
    except subprocess.TimeoutExpired:
        return (f"ERROR: nmap discovery timed out (>{int(_NMAP_TIMEOUT_DISCOVERY)}s) "
                "— narrow the cidr (e.g. /24 instead of /16)")
    except Exception as e:
        return f"ERROR: nmap failed to launch: {e}"
    if proc.returncode != 0:
        msg = (proc.stderr or '').strip().splitlines()[:3]
        return f"ERROR: nmap exited {proc.returncode}: " + ' | '.join(msg)
    # -oG produces lines starting with `Host:` for each scanned host.
    # Filter to up-status lines and emit one summary line per host.
    hosts = []
    for line in (proc.stdout or '').splitlines():
        if not line.startswith('Host:'):
            continue
        if 'Status: Up' in line:
            hosts.append(line)
    if not hosts:
        return f"EMPTY: discover found no hosts up in {net}"
    return _truncate('OK: ' + "\n".join(hosts), 'cidr')


def _tool_scan_services(host: str) -> str:
    if shutil.which('nmap') is None:
        return ("ERROR: nmap is not on PATH — install with "
                "`apt install nmap`")
    net = _is_rfc1918(host)
    if net is None:
        return (f"ERROR: scan_services host {host!r} is not a valid "
                "RFC1918 IPv4 address (must be inside 10.0.0.0/8, "
                "172.16.0.0/12, or 192.168.0.0/16)")
    if net.num_addresses != 1:
        return (f"ERROR: scan_services takes a single host, got CIDR "
                f"{host!r} ({net.num_addresses} addresses) — use "
                "discover for ranges")
    target = str(next(net.hosts(), net.network_address))
    # --script=safe: run only NSE scripts categorized as non-intrusive.
    # -T3: default timing (avoids -T4/-T5 aggressive modes that can
    # trip rate-limiting on the target).
    cmd = ['nmap', '-sV', '--script=safe', '-T3', '-oG', '-', '--', target]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=_NMAP_TIMEOUT_SERVICES, check=False,
        )
    except subprocess.TimeoutExpired:
        return (f"ERROR: nmap scan timed out (>{int(_NMAP_TIMEOUT_SERVICES)}s) "
                f"on {target}")
    except Exception as e:
        return f"ERROR: nmap failed to launch: {e}"
    if proc.returncode != 0:
        msg = (proc.stderr or '').strip().splitlines()[:3]
        return f"ERROR: nmap exited {proc.returncode}: " + ' | '.join(msg)
    # Greppable output: the relevant line is the one starting with
    # `Host: <ip>` that contains a `Ports:` field.
    lines = []
    for line in (proc.stdout or '').splitlines():
        if line.startswith('Host:') and ('Ports:' in line or 'Status:' in line):
            lines.append(line)
    if not lines:
        return f"EMPTY: nmap scan returned no port info for {target}"
    return _truncate('OK: ' + "\n".join(lines), 'host')


def _tool_system_state(category: str) -> str:
    cat = (category or '').strip().lower()
    if cat not in _SYSTEM_STATE_COMMANDS:
        valid = ', '.join(sorted(_SYSTEM_STATE_COMMANDS.keys()))
        return f"ERROR: unknown category {category!r}; valid: {valid}"
    cmd = _SYSTEM_STATE_COMMANDS[cat]
    if shutil.which(cmd[0]) is None:
        return (f"ERROR: `{cmd[0]}` is not on PATH — install iproute2 "
                "(usually preinstalled on Linux)")
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=_SYSTEM_STATE_TIMEOUT, check=False,
        )
    except subprocess.TimeoutExpired:
        return f"ERROR: {cmd[0]} timed out (>{int(_SYSTEM_STATE_TIMEOUT)}s)"
    except Exception as e:
        return f"ERROR: {cmd[0]} failed to launch: {e}"
    if proc.returncode != 0:
        msg = (proc.stderr or '').strip().splitlines()[:3]
        return f"ERROR: {cmd[0]} exited {proc.returncode}: " + ' | '.join(msg)
    out = (proc.stdout or '').strip()
    if not out:
        return f"EMPTY: {cat} returned no output"
    return _truncate('OK: ' + out, 'category')


def _write_trace(trace_dir: Path, query: str,
                 iters: List[Dict[str, Any]], answer: str,
                 exit_reason: str) -> Optional[Path]:
    try:
        trace_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H-%M-%SZ')
        path = trace_dir / f'security_{ts}.txt'
        lines = [
            '=' * 80,
            f'[security] {ts} exit={exit_reason} iters={len(iters)}',
            '=' * 80,
            f'Query: {query}',
            '',
        ]
        for i, it in enumerate(iters, start=1):
            lines.append(f'--- iter {i} ---')
            lines.append('ACTION:')
            if it.get('action') is not None:
                lines.append(json.dumps(it['action'], indent=2))
            else:
                lines.append('(unparseable; raw follows)')
                lines.append(it.get('raw', ''))
            obs = it.get('observation', '')
            if obs:
                lines.append('OBSERVATION:')
                lines.append(obs)
            lines.append('')
        lines.append('FINAL ANSWER:')
        lines.append(answer)
        path.write_text('\n'.join(lines), encoding='utf-8')
        return path
    except Exception as e:
        logger.warning(f"security: trace write failed: {e}")
        return None


def security(query: str, llm_backend, trace_dir: Path) -> str:
    """Run the network-security investigation subagent. Returns the
    synthesized answer string, suitable for binding to a $stepN
    observation in Jill's parent ReAct loop. Side effect: writes a
    per-call trace file under trace_dir.

    Args:
        query: natural-language question about LAN/system network state.
        llm_backend: a _ChatBackend instance used to generate actions.
            Caller is responsible for using a strong model (Sonnet).
        trace_dir: where to write the per-call subagent trace.
    """
    if not query or not query.strip():
        return "(security: empty query)"
    sys_prompt = _build_system_prompt()
    user_prefix = f"Query: {query.strip()}\n\n## Working log\n"
    log_lines: List[str] = []
    iters: List[Dict[str, Any]] = []

    def _build_user_msg() -> str:
        body = user_prefix + ('\n'.join(log_lines) + '\n' if log_lines else '')
        return body + '\nEmit next action:\n'

    answer = ''
    exit_reason = 'max_iters'
    for i in range(_MAX_ITERS):
        messages = [
            {'role': 'system', 'content': sys_prompt},
            {'role': 'user', 'content': _build_user_msg()},
        ]
        try:
            raw = llm_backend.chat(messages, max_tokens=800, temperature=0.2)
        except Exception as e:
            logger.warning(f"security: llm call failed at iter {i+1}: {e}")
            answer = f"(security: llm error at iter {i+1}: {e})"
            exit_reason = 'llm_error'
            break

        action = _parse_action(raw)
        iter_rec: Dict[str, Any] = {'raw': raw, 'action': action}
        iters.append(iter_rec)
        if action is None:
            log_lines.append(
                "NOTE: previous output was unparseable; emit ONE JSON action now.")
            iter_rec['observation'] = '(unparseable)'
            continue

        tool = action.get('tool')
        if tool == 'respond':
            answer = str(action.get('text', '') or '').strip() or '(no answer)'
            exit_reason = 'respond'
            iter_rec['observation'] = '(respond)'
            break

        binding = f'$step{i+1}'
        if tool == 'discover':
            obs = _tool_discover(action.get('cidr', ''))
        elif tool == 'scan_services':
            obs = _tool_scan_services(action.get('host', ''))
        elif tool == 'system_state':
            obs = _tool_system_state(action.get('category', ''))
        else:
            obs = (f"ERROR: unknown tool {tool!r}; available: "
                   "discover, scan_services, system_state, respond")

        iter_rec['observation'] = obs
        log_lines.append(f"ACTION {i+1}: {json.dumps(action)}")
        log_lines.append(f"{binding}:")
        log_lines.append(obs)
        log_lines.append('')

    if exit_reason == 'max_iters' and not answer:
        answer = ("(security: hit max iterations without responding; "
                  "consider narrowing the query)")

    _write_trace(trace_dir, query, iters, answer, exit_reason)
    return answer
