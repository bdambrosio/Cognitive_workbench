"""Security subagent — a thin, persona-less ReAct loop that
investigates LAN state and local host state via a small set of typed
primitives. Mirrors the inspect.py pattern: argument-list subprocess,
geofenced inputs, read-only operations, OK/EMPTY/ERROR observations.

Used by Jill's chat ReAct loop via the `security` tool. From Jill's
vantage, a `security` call is one step: she emits a query string,
gets a synthesized answer back. The subagent's internal multi-iter
reasoning is opaque to her trace but written to security_traces/ for
debugging.

Architectural notes:
- Backend is whatever the caller passes — by default this is the main
  character's backend (self.backend in chat_loop), so per-scenario YAML
  decides the model. No per-subagent backend overrides.
- Primitives are intentionally minimal and read-only:
  discover (nmap host discovery), scan_services (nmap -sV),
  system_state (ss / ip), host_state (processes / logins / cron /
  SUID inventory / patch posture, all fixed argv), package_version
  (dpkg-query, names validated), baseline_diff (snapshot compare —
  the only primitive that writes, and only its own snapshot files
  under baseline_dir), respond.
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

from utils.json_utils import repair_json_string
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

# host_state categories → argv. Same discipline as system_state: every
# command is hardcoded, read-only, argument-list-only, unprivileged. The
# LLM picks a category name; it never supplies arguments. Categories that
# need composite output or file reads (cron, ssh_keys, unattended_log)
# are handled by dedicated functions instead of this table.
_HOST_STATE_TIMEOUT = 15.0
_FIND_TIMEOUT = 90.0
_HOST_STATE_COMMANDS: Dict[str, List[str]] = {
    'processes':     ['ps', 'axo', 'pid,user,pcpu,pmem,etime,args',
                      '--sort=-pcpu'],
    # Login history: Ubuntu 25.10+ dropped `last`/wtmp (Y2038); session
    # starts are read from the journal instead. Journal access is
    # group-gated (adm / systemd-journal); when denied, journalctl's own
    # error message flows back as the observation.
    'logins':        ['journalctl', '-t', 'systemd-logind', '--since',
                      '-7d', '--no-pager', '-q', '-g', 'New session'],
    'auth_failures': ['journalctl', '-t', 'sshd', '--since', '-24h',
                      '--no-pager', '-q', '-g', 'Failed|Invalid|error'],
    'suid':          ['find', '/', '-xdev', '-perm', '-4000', '-type', 'f'],
    'timers':        ['systemctl', 'list-timers', '--all', '--no-pager'],
    'enabled_units': ['systemctl', 'list-unit-files', '--state=enabled',
                      '--no-pager'],
    'upgradable':    ['apt', 'list', '--upgradable'],
}

# journalctl -g adopts grep exit semantics: rc 1 means "no matching
# entries", not failure. Categories using it accept rc 1 and report
# empty output as EMPTY.
_JOURNAL_GREP_CATEGORIES = frozenset({'logins', 'auth_failures'})

# Debian package names: lowercase alphanumerics plus + - . , first char
# alphanumeric (Debian Policy 5.6.1). Validated before reaching argv.
_PACKAGE_NAME_RE = re.compile(r'^[a-z0-9][a-z0-9+.\-]*$')

_UNATTENDED_LOG = Path('/var/log/unattended-upgrades/unattended-upgrades.log')
_UNATTENDED_LOG_TAIL_LINES = 120

# baseline_diff categories → (probe argv, canonicalizer). Headerless,
# machine-stable output forms so snapshots don't diff on volatile
# columns (queue counters, ordering).
_BASELINE_CATEGORIES: Dict[str, List[str]] = {
    'suid':          ['find', '/', '-xdev', '-perm', '-4000', '-type', 'f'],
    'sockets':       ['ss', '-tlnH'],
    'enabled_units': ['systemctl', 'list-unit-files', '--state=enabled',
                      '--no-legend', '--plain', '--no-pager'],
}


def _build_system_prompt() -> str:
    """Static system prompt. Stable across all calls in a session, so
    the anthropic route's prompt cache stays warm."""
    return (
        "You are a security investigation subagent. You answer questions "
        "about LAN state and local host state by running read-only probes "
        "from a small set of typed primitives. You have no persona, no "
        "goals beyond the current query, and no memory of past calls — "
        "each invocation is independent.\n"
        "\n"
        "## Threat model framing\n"
        "\n"
        "Distinct sub-problems map onto different primitives:\n"
        "1. **New / rogue hosts on the LAN.** Use `discover(cidr)` to "
        "enumerate active IPs, compare against expected baseline.\n"
        "2. **Unexpected service exposure.** Use `scan_services(host)` "
        "to inventory open ports + service banners on a known host.\n"
        "3. **Local network state.** Use `system_state(category)` to "
        "see the agent's own listening sockets, routing table, ARP "
        "neighbors, or interface addresses.\n"
        "4. **Host intrusion / persistence.** Use `host_state(category)` "
        "for processes, login history, SSH auth failures, SUID files, "
        "cron entries, systemd timers/units, and authorized SSH keys.\n"
        "5. **Patch posture.** `host_state(upgradable)` for pending "
        "upgrades, `host_state(unattended_log)` to verify unattended-"
        "upgrades ran, `package_version` to check installed versions "
        "against a security notice (e.g. an Ubuntu USN).\n"
        "6. **Change detection.** `baseline_diff(category)` compares "
        "current SUID files / listening sockets / enabled units against "
        "the last stored snapshot — new entries are the interesting ones.\n"
        "Traffic-anomaly detection (signature-based IDS) is NOT in this "
        "toolset — Suricata/Zeek log inspection lands when those tools "
        "are configured.\n"
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
        '4. {"thought": "...", "tool": "host_state", "category": '
        '"<category>"} — read-only local-host probe, fixed commands, no '
        "arguments. Categories:\n"
        "    `processes`      — process snapshot sorted by CPU (ps axo)\n"
        "    `logins`         — session starts (local + ssh), last 7 "
        "days, from the journal\n"
        "    `auth_failures`  — sshd failed/invalid auth, last 24h "
        "(journalctl; needs adm/systemd-journal group)\n"
        "    `suid`           — SUID file inventory on the root "
        "filesystem (find -perm -4000; ~1 min)\n"
        "    `cron`           — user crontab, /etc/crontab, /etc/cron.d "
        "contents, periodic-dir listings\n"
        "    `timers`         — systemd timers (list-timers --all)\n"
        "    `enabled_units`  — enabled systemd unit files\n"
        "    `ssh_keys`       — ~/.ssh/authorized_keys contents\n"
        "    `upgradable`     — packages with pending upgrades (apt list)\n"
        "    `unattended_log` — tail of the unattended-upgrades log\n"
        '5. {"thought": "...", "tool": "package_version", "packages": '
        '"<name1 name2 ...>"} — installed version + status via dpkg-query '
        "(max 40 names per call). Use to cross-reference a security "
        "notice against what is actually installed.\n"
        '6. {"thought": "...", "tool": "baseline_diff", "category": '
        '"<suid|sockets|enabled_units>"} — diff current state against the '
        "last stored snapshot; reports ADDED/REMOVED entries, then "
        "updates the snapshot. First call establishes the baseline. "
        "NOTE: because the snapshot updates on every call, a reported "
        "change must be surfaced in your final answer — it will not be "
        "reported again next call.\n"
        '7. {"thought": "...", "tool": "respond", "text": "<answer>"} — '
        "final answer to the query, exits the loop.\n"
        "\n"
        "## Discipline\n"
        "\n"
        "- **Read-only.** No primitive modifies network or system state. "
        "You do not insert firewall rules, kill processes, install "
        "packages, or send unsolicited packets beyond nmap's standard "
        "probes. The single exception: baseline_diff writes its own "
        "snapshot files, nothing else.\n"
        "- **RFC1918 only.** discover and scan_services reject targets "
        "outside private address ranges. Don't try to scan the internet.\n"
        "- **Pick the right primitive before any tool call:**\n"
        "    * 'What hosts are on my LAN?' → discover with the user's "
        "subnet.\n"
        "    * 'What's running on host X?' → scan_services on that IP.\n"
        "    * 'What ports am I listening on?' → system_state(sockets).\n"
        "    * 'What's my IP / interface?' → system_state(interfaces).\n"
        "    * 'What's my default gateway?' → system_state(routes).\n"
        "    * 'Any signs of intrusion?' → host_state over logins, "
        "auth_failures, processes, cron, ssh_keys + baseline_diff over "
        "suid, sockets, enabled_units.\n"
        "    * 'Are we patched?' → host_state(upgradable) + "
        "host_state(unattended_log); package_version for specific CVE/"
        "USN cross-checks.\n"
        "- **Privilege caveat.** Running as a regular user, nmap `-sn` "
        "uses TCP/ICMP probes (some hosts may not respond). `-sV` uses "
        "TCP connect (works fine, slightly slower than SYN). `ss -tlnp` "
        "shows PID/program only for processes the current user owns. "
        "auth_failures and unattended_log need adm/systemd-journal group "
        "membership; the SUID walk cannot enter root-only directories; "
        "kernel-level rootkits are invisible to every probe here. Note "
        "these limitations honestly in your final answer if relevant.\n"
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
    obj = repair_json_string(raw or '')
    return obj if isinstance(obj, dict) else None


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
    # Single-host network: network_address is the host itself. Don't use
    # next(net.hosts(), ...) — for /32 networks Python's IPv4Network.hosts()
    # returns a literal list (not a generator), and next() on a list raises
    # 'list object is not an iterator'.
    target = str(net.network_address)
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


def _run_argv(cmd: List[str], timeout: float,
              ok_returncodes: tuple = (0,)) -> tuple:
    """Run a fixed argument-list command. Returns (output, error) where
    exactly one is non-None. Shared by system_state / host_state /
    baseline_diff so the subprocess discipline lives in one place."""
    if shutil.which(cmd[0]) is None:
        return None, f"ERROR: `{cmd[0]}` is not on PATH"
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=timeout, check=False,
        )
    except subprocess.TimeoutExpired:
        return None, f"ERROR: {cmd[0]} timed out (>{int(timeout)}s)"
    except Exception as e:
        return None, f"ERROR: {cmd[0]} failed to launch: {e}"
    if proc.returncode not in ok_returncodes:
        msg = (proc.stderr or '').strip().splitlines()[:3]
        return None, (f"ERROR: {cmd[0]} exited {proc.returncode}: "
                      + ' | '.join(msg))
    return (proc.stdout or '').strip(), None


def _tool_system_state(category: str) -> str:
    cat = (category or '').strip().lower()
    if cat not in _SYSTEM_STATE_COMMANDS:
        valid = ', '.join(sorted(_SYSTEM_STATE_COMMANDS.keys()))
        return f"ERROR: unknown category {category!r}; valid: {valid}"
    out, err = _run_argv(_SYSTEM_STATE_COMMANDS[cat], _SYSTEM_STATE_TIMEOUT)
    if err:
        return err
    if not out:
        return f"EMPTY: {cat} returned no output"
    return _truncate('OK: ' + out, 'category')


def _read_file_tail(path: Path, max_lines: int) -> Optional[str]:
    """Last max_lines of a text file, or None if unreadable."""
    try:
        lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
    except OSError as e:
        logger.info(f"security: cannot read {path}: {e}")
        return None
    return '\n'.join(lines[-max_lines:])


def _host_cron_report() -> str:
    """Composite persistence-point read: user crontab, /etc/crontab,
    /etc/cron.d contents, and periodic-dir listings. Read-only."""
    sections: List[str] = []
    # `crontab -l` exits 1 with "no crontab for <user>" — informative,
    # not an error.
    out, err = _run_argv(['crontab', '-l'], _HOST_STATE_TIMEOUT,
                         ok_returncodes=(0, 1))
    sections.append('## user crontab\n' + (err or out or '(no user crontab)'))
    etc_crontab = _read_file_tail(Path('/etc/crontab'), 200)
    sections.append('## /etc/crontab\n'
                    + (etc_crontab if etc_crontab is not None
                       else '(unreadable or missing)'))
    cron_d = Path('/etc/cron.d')
    if cron_d.is_dir():
        parts = []
        for f in sorted(cron_d.iterdir()):
            if not f.is_file():
                continue
            body = _read_file_tail(f, 50)
            parts.append(f'--- {f.name} ---\n'
                         + (body if body is not None else '(unreadable)'))
        sections.append('## /etc/cron.d\n'
                        + ('\n'.join(parts) if parts else '(empty)'))
    for d in ('cron.hourly', 'cron.daily', 'cron.weekly', 'cron.monthly'):
        p = Path('/etc') / d
        if p.is_dir():
            names = sorted(f.name for f in p.iterdir())
            sections.append(f'## /etc/{d}\n'
                            + (', '.join(names) if names else '(empty)'))
    return '\n\n'.join(sections)


def _tool_host_state(category: str) -> str:
    cat = (category or '').strip().lower()
    valid_cats = sorted(list(_HOST_STATE_COMMANDS.keys())
                        + ['cron', 'ssh_keys', 'unattended_log'])
    if cat == 'cron':
        return _truncate('OK: ' + _host_cron_report(), 'category')
    if cat == 'ssh_keys':
        path = Path.home() / '.ssh' / 'authorized_keys'
        body = _read_file_tail(path, 100)
        if body is None:
            return f"EMPTY: {path} is missing or unreadable"
        if not body.strip():
            return f"EMPTY: {path} exists but is empty"
        return _truncate(f'OK: {path}:\n' + body, 'category')
    if cat == 'unattended_log':
        body = _read_file_tail(_UNATTENDED_LOG, _UNATTENDED_LOG_TAIL_LINES)
        if body is None:
            return (f"ERROR: cannot read {_UNATTENDED_LOG} — "
                    "unattended-upgrades may not be installed/enabled, or "
                    "the current user lacks `adm` group membership "
                    "(see docs/sentinel-setup.md)")
        return _truncate('OK: ' + body, 'category')
    if cat not in _HOST_STATE_COMMANDS:
        return (f"ERROR: unknown category {category!r}; valid: "
                + ', '.join(valid_cats))
    timeout = _FIND_TIMEOUT if cat == 'suid' else _HOST_STATE_TIMEOUT
    cmd = _HOST_STATE_COMMANDS[cat]
    if cat == 'suid':
        # Permission-denied noise on unreadable dirs is expected when
        # unprivileged; find still exits 1 in that case, so accept it
        # and keep stdout.
        out, err = _run_argv(cmd, timeout, ok_returncodes=(0, 1))
    elif cat in _JOURNAL_GREP_CATEGORIES:
        out, err = _run_argv(cmd, timeout, ok_returncodes=(0, 1))
    else:
        out, err = _run_argv(cmd, timeout)
    if err:
        return err
    if not out:
        if cat in _JOURNAL_GREP_CATEGORIES:
            return (f"EMPTY: no matching journal entries for {cat} in the "
                    "covered window (or journal access is group-gated — "
                    "adm/systemd-journal)")
        return f"EMPTY: {cat} returned no output"
    return _truncate('OK: ' + out, 'category')


def _tool_package_version(packages: Any) -> str:
    """Installed-version lookup via dpkg-query. Package names are the
    only caller-controlled input on the host side; each is validated
    against Debian package-name syntax before reaching argv."""
    if isinstance(packages, str):
        names = packages.replace(',', ' ').split()
    elif isinstance(packages, list):
        names = [str(p).strip() for p in packages if str(p).strip()]
    else:
        names = []
    if not names:
        return "ERROR: package_version needs one or more package names"
    if len(names) > 40:
        return f"ERROR: too many packages ({len(names)}); cap is 40 per call"
    bad = [n for n in names if not _PACKAGE_NAME_RE.match(n)]
    if bad:
        return ("ERROR: invalid package name(s): "
                + ', '.join(repr(b) for b in bad))
    # dpkg-query exits 1 when any queried package is unknown; its stderr
    # names them. Report both installed versions and misses.
    cmd = (['dpkg-query', '-W', '-f', '${Package} ${Version} ${Status}\n',
            '--'] + names)
    if shutil.which('dpkg-query') is None:
        return "ERROR: dpkg-query is not on PATH"
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=_HOST_STATE_TIMEOUT, check=False)
    except subprocess.TimeoutExpired:
        return f"ERROR: dpkg-query timed out (>{int(_HOST_STATE_TIMEOUT)}s)"
    except Exception as e:
        return f"ERROR: dpkg-query failed to launch: {e}"
    out = (proc.stdout or '').strip()
    missing = [ln for ln in (proc.stderr or '').splitlines()
               if 'no packages found' in ln.lower()]
    body = out or ''
    if missing:
        body += ('\n' if body else '') + '\n'.join(missing)
    if not body:
        return "EMPTY: dpkg-query returned nothing for those names"
    return _truncate('OK: ' + body, 'package list')


def _canonicalize_baseline(cat: str, out: str) -> List[str]:
    """Reduce probe output to a stable, sorted line set so snapshots
    don't diff on volatile columns or ordering."""
    lines = [ln for ln in out.splitlines() if ln.strip()]
    if cat == 'sockets':
        # ss -tlnH: State Recv-Q Send-Q Local:Port Peer:Port [Process].
        # Keep only the local address:port — queue counters are volatile
        # and the process column is privilege-dependent.
        keep = []
        for ln in lines:
            fields = ln.split()
            if len(fields) >= 4:
                keep.append(fields[3])
        lines = keep
    elif cat == 'enabled_units':
        # --plain --no-legend: "<unit> <state> [preset]". Keep unit+state.
        lines = [' '.join(ln.split()[:2]) for ln in lines]
    return sorted(set(lines))


def _tool_baseline_diff(category: str, baseline_dir: Optional[Path]) -> str:
    cat = (category or '').strip().lower()
    if cat not in _BASELINE_CATEGORIES:
        valid = ', '.join(sorted(_BASELINE_CATEGORIES.keys()))
        return f"ERROR: unknown baseline category {category!r}; valid: {valid}"
    if baseline_dir is None:
        return ("ERROR: no baseline directory configured for this call "
                "site — baseline_diff is unavailable")
    cmd = _BASELINE_CATEGORIES[cat]
    timeout = _FIND_TIMEOUT if cat == 'suid' else _HOST_STATE_TIMEOUT
    ok_rcs = (0, 1) if cat == 'suid' else (0,)
    out, err = _run_argv(cmd, timeout, ok_returncodes=ok_rcs)
    if err:
        return err
    current = _canonicalize_baseline(cat, out or '')
    if not current:
        return f"EMPTY: {cat} probe returned no entries; snapshot not updated"

    snap_path = baseline_dir / f'{cat}.json'
    previous: Optional[Dict[str, Any]] = None
    if snap_path.exists():
        try:
            previous = json.loads(snap_path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as e:
            logger.warning(f"security: unreadable baseline {snap_path}: {e}")
            previous = None

    now = datetime.now(timezone.utc).isoformat()
    try:
        baseline_dir.mkdir(parents=True, exist_ok=True)
        snap_path.write_text(
            json.dumps({'captured_at': now, 'lines': current}, indent=1),
            encoding='utf-8')
    except OSError as e:
        logger.warning(f"security: baseline write failed for {cat}: {e}")
        return f"ERROR: could not write snapshot {snap_path}: {e}"

    if previous is None or not isinstance(previous.get('lines'), list):
        return (f"OK: no prior {cat} snapshot — baseline established now "
                f"({len(current)} entries). Diffs start from the next call.")
    prev_lines = set(str(x) for x in previous['lines'])
    added = [ln for ln in current if ln not in prev_lines]
    removed = sorted(prev_lines - set(current))
    when = previous.get('captured_at', 'unknown time')
    if not added and not removed:
        return (f"OK: {cat} unchanged since snapshot of {when} "
                f"({len(current)} entries)")
    parts = [f"{cat} changed since snapshot of {when}:"]
    if added:
        parts.append('ADDED:\n' + '\n'.join(f'  + {ln}' for ln in added))
    if removed:
        parts.append('REMOVED:\n' + '\n'.join(f'  - {ln}' for ln in removed))
    parts.append('(snapshot updated to current state)')
    return _truncate('OK: ' + '\n'.join(parts), 'category')


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


def security(query: str, llm_backend, trace_dir: Path,
             baseline_dir: Optional[Path] = None) -> str:
    """Run the security investigation subagent. Returns the
    synthesized answer string, suitable for binding to a $stepN
    observation in Jill's parent ReAct loop. Side effect: writes a
    per-call trace file under trace_dir, and (baseline_diff only)
    snapshot files under baseline_dir.

    Args:
        query: natural-language question about LAN or local host state.
        llm_backend: a _ChatBackend instance used to generate actions.
            By convention this is the main character backend; the
            per-scenario llm_config decides the model.
        trace_dir: where to write the per-call subagent trace.
        baseline_dir: where baseline_diff stores per-category snapshots;
            None disables baseline_diff with an in-loop error.
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
            raw = llm_backend.chat(messages, max_tokens=4096, temperature=0.2)
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
        elif tool == 'host_state':
            obs = _tool_host_state(action.get('category', ''))
        elif tool == 'package_version':
            obs = _tool_package_version(action.get('packages', ''))
        elif tool == 'baseline_diff':
            obs = _tool_baseline_diff(action.get('category', ''), baseline_dir)
        else:
            obs = (f"ERROR: unknown tool {tool!r}; available: "
                   "discover, scan_services, system_state, host_state, "
                   "package_version, baseline_diff, respond")

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
