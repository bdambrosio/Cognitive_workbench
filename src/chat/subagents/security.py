"""Security subagent — a thin, persona-less ReAct loop that
investigates LAN state and local host state via a small set of typed
primitives. Follows the canonical subagent template (`recall.py`, see
README): argument-list subprocess, geofenced inputs, read-only
operations, OK/EMPTY/ERROR observations. (This originally read "mirrors
the inspect.py pattern"; that file was renamed to code_subagent.py on
2026-05-05, and remember.py to recall.py when the subagents/ package
was created.)

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
import time

from utils.json_utils import repair_json_string
from utils.subagent_trace import write_subagent_trace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

_MAX_ITERS = 12
_MAX_OBS_CHARS = 8_000
_NMAP_TIMEOUT_DISCOVERY = 60.0
# nmap's own per-host cap, so it returns partial results instead of being
# killed, and the outer subprocess cap as a backstop above it. Ordered so
# the inner one always fires first — if the subprocess timeout wins, the
# data is gone.
_NMAP_HOST_TIMEOUT = 45.0
_NMAP_TIMEOUT_SERVICES = 60.0
_SYSTEM_STATE_TIMEOUT = 5.0

# Wall clock for one whole security() call. The per-probe timeouts above
# bound a single probe, not the run: a topology question walks the LAN one
# host at a time, and 12 iterations × 120s put a live LAN sweep at ~24
# minutes — during which the character's inbox loop is blocked and every
# message to it queues (observed 2026-08-10: Sentinel unreachable for 20+
# minutes, and the ReAct loop then called security a SECOND time, each
# call with its own fresh iteration budget). Checked before dispatching a
# probe, so overshoot is at most one probe.
_CALL_BUDGET = 300.0

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
    'sockets':     ['ss', '-tlnp'],
    'udp_sockets': ['ss', '-ulnp'],
    'routes':      ['ip', 'route', 'show'],
    'arp':         ['ip', 'neigh', 'show'],
    'interfaces':  ['ip', '-br', 'addr', 'show'],
}

# `connections` is not in the table above: raw `ss` output ran to 160
# rows on a normal desktop, 109 of them one browser, which truncated the
# observation and buried anything worth seeing. _connections_report
# aggregates instead — see there.
_CONNECTIONS_CMD = ['ss', '-tunp', 'state', 'established']
# Above this many distinct peers a process is reported as a count only.
_CONNECTIONS_FANOUT = 12

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
    'failed_units':  ['systemctl', 'list-units', '--failed', '--no-pager'],
    'upgradable':    ['apt', 'list', '--upgradable'],
    # Host identity for security-notice triage. A USN names an Ubuntu
    # release and a kernel flavour; without both of these, applicability
    # cannot be decided and every kernel notice in the feed looks like it
    # might apply. `uname -rv` is the RUNNING kernel — compare it against
    # package_version('linux-image-generic') to catch patched-but-not-
    # rebooted, which no package query can show on its own.
    'kernel':        ['uname', '-rv'],
    'os_release':    ['cat', '/etc/os-release'],
    'containers':    ['docker', 'ps', '--all', '--no-trunc',
                      '--format', '{{.Names}}\t{{.Image}}\t{{.Ports}}\t'
                                  '{{.Status}}'],
    # Privilege-use and account-management events. No -g, so an empty
    # journal is rc 0 with no output.
    'privilege_events': ['journalctl', '-t', 'sudo', '-t', 'su',
                         '-t', 'useradd', '-t', 'usermod', '-t', 'groupadd',
                         '--since', '-7d', '--no-pager', '-q'],
}

# Tier 2 — privileged read-only probes reached through `sudo -n` with
# argument-exact NOPASSWD entries (see docs/sentinel-setup.md). These
# answer questions unprivileged probes structurally cannot: which
# process owns a root-owned listener, what the firewall actually
# permits, whether MAC enforcement is in place. `-n` never prompts, so
# an unconfigured host fails immediately instead of hanging the loop.
# Full paths are mandatory — the sudoers entry matches the path as
# invoked, and sudo's secure_path applies. aa-status is used rather
# than its apparmor_status symlink so the fenced path is unambiguous.
_SUDO_HOST_COMMANDS: Dict[str, List[str]] = {
    'firewall':        ['sudo', '-n', '/usr/sbin/ufw', 'status', 'verbose'],
    'listener_owners': ['sudo', '-n', '/usr/bin/ss', '-tulpnH'],
    'apparmor':        ['sudo', '-n', '/usr/sbin/aa-status'],
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
    # LAN membership. The kernel's neighbour table is populated by
    # traffic that happens anyway, so reading it costs nothing and no
    # privilege; run discover() first to make coverage deliberate
    # rather than opportunistic. Canonicalized to bare MAC addresses —
    # see _canonicalize_baseline for why.
    'arp':           ['ip', 'neigh', 'show'],
    'suid':          ['find', '/', '-xdev', '-perm', '-4000', '-type', 'f'],
    'sockets':       ['ss', '-tlnH'],
    'udp_sockets':   ['ss', '-ulnH'],
    'enabled_units': ['systemctl', 'list-unit-files', '--state=enabled',
                      '--no-legend', '--plain', '--no-pager'],
    'user_units':    ['systemctl', '--user', 'list-unit-files',
                      '--state=enabled', '--no-legend', '--plain',
                      '--no-pager'],
    # A second uid-0 entry or an unexpected new account is a classic
    # persistence move, and /etc/passwd is world-readable.
    'accounts':      ['getent', 'passwd'],
    # Desktop autostart is the most-used user-level persistence surface
    # and needs no privilege to write, so diffing it matters.
    'autostart':     ['find', str(Path.home() / '.config' / 'autostart'),
                      '-maxdepth', '1', '-type', 'f'],
}

# Categories whose snapshot accumulates instead of tracking current state.
# The kernel evicts neighbour entries for hosts that go quiet, so a
# point-in-time image of `ip neigh show` answers "who spoke recently", not
# "who have I ever seen" — and a device that naps for a day reads as a new
# arrival every time it wakes. Storing the union makes an ADDED entry mean
# first-ever sighting, which is what the MAC-keyed canonicalization was
# for. REMOVED is vacuous under a union and is suppressed for these
# categories: a device falling out of the cache is not an event.
#
# The other categories are genuine current-state inventories — a SUID bit
# or an enabled unit that disappears is itself worth reporting — so they
# keep replace-on-write semantics.
_CUMULATIVE_BASELINES = frozenset({'arp'})


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
        "current SUID files / listening sockets / enabled units / LAN "
        "neighbours against "
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
        '"<sockets|udp_sockets|routes|arp|interfaces|connections>"} — '
        "read-only system probe. Categories:\n"
        "    `sockets`     — listening TCP sockets (ss -tlnp)\n"
        "    `udp_sockets` — listening UDP sockets (ss -ulnp)\n"
        "    `routes`      — IP routing table (ip route show)\n"
        "    `arp`         — ARP / IPv6 neighbor table (ip neigh show)\n"
        "    `interfaces`  — interface addresses (ip -br addr show)\n"
        "    `connections` — ESTABLISHED outbound/inbound connections "
        "and their processes (ss -tunp): where this host is talking to\n"
        '4. {"thought": "...", "tool": "host_state", "category": '
        '"<category>"} — read-only local-host probe, fixed commands, no '
        "arguments. Categories:\n"
        "    `processes`      — process snapshot sorted by CPU (ps axo)\n"
        "    `logins`         — session starts (local + ssh), last 7 "
        "days, from the journal\n"
        "    `auth_failures`  — sshd failed/invalid auth, last 24h "
        "(journalctl; needs adm/systemd-journal group)\n"
        "    `privilege_events` — sudo/su/useradd/usermod/groupadd "
        "activity, last 7 days\n"
        "    `suid`           — SUID file inventory on the root "
        "filesystem (find -perm -4000; ~1 min)\n"
        "    `cron`           — user crontab, /etc/crontab, /etc/cron.d "
        "contents, periodic-dir listings\n"
        "    `timers`         — systemd timers (list-timers --all)\n"
        "    `enabled_units`  — enabled systemd unit files\n"
        "    `failed_units`   — systemd units in a failed state\n"
        "    `user_persistence` — ~/.config/autostart, enabled "
        "systemd --user units, shell startup files. Plantable WITHOUT "
        "root, so a user-account compromise shows here first.\n"
        "    `accounts`       — uid-0, regular, and shell-bearing "
        "system accounts from /etc/passwd\n"
        "    `ssh_keys`       — ~/.ssh/authorized_keys contents\n"
        "    `ssh_config`     — auth-relevant sshd directives\n"
        "    `containers`     — docker containers (running and stopped) "
        "with their published ports\n"
        "    `upgradable`     — packages with pending upgrades (apt list)\n"
        "    `unattended_log` — tail of the unattended-upgrades log\n"
        "    `kernel`         — running kernel version (uname -rv)\n"
        "    `os_release`     — Ubuntu release identity (/etc/os-release)\n"
        "  Privileged categories (fenced `sudo -n`, argument-exact "
        "NOPASSWD entries; unavailable until configured, which reports "
        "as EMPTY and is a SETUP GAP, never a finding):\n"
        "    `firewall`        — ufw status verbose (the actual "
        "ruleset, not just whether ufw is enabled)\n"
        "    `listener_owners` — ss -tulpnH with PID/program for ALL "
        "listeners including root-owned ones. Unprivileged `sockets` "
        "shows root-owned listeners with NO process attribution, so "
        "this is the only way to name what holds an open port.\n"
        "    `apparmor`        — aa-status profile detail\n"
        '5. {"thought": "...", "tool": "package_version", "packages": '
        '"<name1 name2 ...>"} — installed version + status via dpkg-query '
        "(max 40 names per call). Use to cross-reference a security "
        "notice against what is actually installed.\n"
        '6. {"thought": "...", "tool": "baseline_diff", "category": '
        '"<suid|sockets|udp_sockets|enabled_units|user_units|accounts|'
        'autostart|arp>"} — diff current state against the '
        "last stored snapshot; reports ADDED/REMOVED entries, then "
        "updates the snapshot. `arp` is LAN membership, keyed by MAC "
        "address, and its snapshot is cumulative — an ADDED entry is a "
        "MAC never seen on this network before, and REMOVED is never "
        "reported (a device going quiet is not an event). "
        "Before reporting an ADDED device, identify it: system_state(arp) "
        "gives the address behind the MAC, and discover()/scan_services() "
        "on that address say whether anything is reachable there. Report "
        "what you found, and say plainly when the answer is that you "
        "could not tell — an unidentified MAC is an inventory question, "
        "not yet evidence of an intruder. "
        "Run discover() on the local subnet first if you want that "
        "coverage to be deliberate rather than whatever happened to "
        "talk to this host. "
        "First call establishes the baseline. "
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
        "auth_failures, privilege_events, processes, cron, "
        "user_persistence, ssh_keys, containers + baseline_diff over "
        "suid, sockets, accounts, autostart, enabled_units.\n"
        "    * 'What is exposed / is the firewall right?' → "
        "system_state(sockets) and (udp_sockets) for what listens, then "
        "host_state(listener_owners) to name the owning process and "
        "host_state(firewall) for what the ruleset actually permits. A "
        "listener on 0.0.0.0 is reachable from the network; one on "
        "127.0.0.1 is not. Say which, explicitly.\n"
        "    * 'Are we patched?' → host_state(upgradable) + "
        "host_state(unattended_log); package_version for specific CVE/"
        "USN cross-checks.\n"
        "- **Privilege caveat.** Running as a regular user, nmap `-sn` "
        "uses TCP/ICMP probes (some hosts may not respond). `-sV` uses "
        "TCP connect (works fine, slightly slower than SYN). `ss -tlnp` "
        "shows PID/program only for processes the current user owns. "
        "auth_failures and unattended_log need adm/systemd-journal group "
        "membership; the SUID walk cannot enter root-only directories; "
        "unprivileged probes cannot see /root, other users' crontabs, or "
        "modified system binaries; kernel-level rootkits are invisible to "
        "every probe here. Note these limitations honestly in your final "
        "answer if relevant, and never let a clean scan imply coverage "
        "you did not have.\n"
        "- **A security notice is not a finding until it applies here.** "
        "A USN names an Ubuntu release and specific package flavours, and "
        "the feed carries notices for every release and flavour Ubuntu "
        "supports — most will be for releases this host does not run "
        "(older LTS, cloud/embedded kernels). Before reporting one, "
        "establish applicability: host_state(os_release) for the release, "
        "host_state(kernel) for the running kernel, and package_version "
        "for the versions actually installed. If the release or flavour "
        "does not match, say the notice does not apply and move on — do "
        "not report it as unverified. If it does match, compare the "
        "installed version against the notice's fixed version and say "
        "which side of it this host is on. 'Cannot determine the "
        "installed version' is not available to you: package_version "
        "answers that directly.\n"
        "- **Fenced-probe absence is not a finding.** When a privileged "
        "category returns EMPTY because its sudoers entry is missing, "
        "that is unconfigured tooling. Report it as a setup gap and do "
        "not retry it in the same run.\n"
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
    # -F: top-100 ports. -sV: service/version detection. -T3: default
    # timing (avoids -T4/-T5, which can trip rate-limiting on the target).
    #
    # `--script=safe` was here and is gone. It ran the whole non-intrusive
    # NSE category against every open port, which cost minutes per host —
    # and `-oG` discards the result: grepable output carries only Host,
    # Ports, Protocols, Ignored State, OS, Seq Index, IP ID and Status
    # (nmap(1)), with no field for script output. So the parse below never
    # saw a line of it. If script output is ever wanted, it needs a
    # different output format, not a longer timeout.
    #
    # Dropping it was NOT sufficient, and why is instructive: cost here is
    # driven by unresponsive hosts, not by port count. Measured 2026-08-10
    # against 192.168.68.1, `-sV -F -T3` still ran past 130s, because
    # nmap's default retransmit ladder plus a redundant host-discovery pass
    # stall on every filtered port. The three flags below fix that, and each
    # SENDS LESS than before rather than more:
    #   -Pn              `discover` already proved the host up; repeating
    #                    discovery per scan is waste, and on a host that
    #                    blocks ping it is a stall.
    #   --host-timeout   nmap returns what it has and marks the rest
    #                    `Status: Timeout`. This is the important one: when
    #                    the subprocess timeout fires instead, nmap is
    #                    KILLED and every partial result is discarded —
    #                    that is what produced "all scans timed out, no
    #                    data at all" for a nine-host sweep.
    #   --max-retries 1  a LAN, not the open internet; the default ladder
    #                    is where the time goes.
    # Timing stays -T3 deliberately (see above). After: 11.7s with full
    # service detail on a responsive host, and a clean partial answer on
    # the gateway that previously ran past 130s.
    cmd = ['nmap', '-sV', '-F', '-Pn', '-T3',
           '--host-timeout', f'{int(_NMAP_HOST_TIMEOUT)}s',
           '--max-retries', '1', '-oG', '-', '--', target]
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
    # `--host-timeout` expiring is a clean exit with no port data, so it
    # reaches here as a Status line and would render as OK — a scan that
    # learned nothing, reported as a success. Say so instead: this host is
    # slow or heavily filtered, and re-running it unchanged will not help.
    if lines and not any('Ports:' in ln for ln in lines) and any(
            'Status: Timeout' in ln for ln in lines):
        return (f"EMPTY: {target} did not answer enough of the scan within "
                f"{int(_NMAP_HOST_TIMEOUT)}s to identify any service — it is "
                f"up but slow or heavily filtered. No service data for this "
                f"host; do not retry it unchanged.")
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


def _connections_report() -> str:
    """Established connections as unique process→peer pairs, off-box first.

    Answers "where is this host talking to" — the question a compromise
    shows up in, and the one every other category here misses by looking
    only at what can reach us.

    Aggregated rather than listed: a browser alone holds a hundred-odd
    sockets to a few dozen hosts, so the raw output is mostly repetition
    and would truncate before the interesting rows. Collapsing to
    (process, peer) with a count keeps the whole picture inside the
    observation budget. Loopback is counted, not listed: local service
    traffic is expected here and is not egress.
    """
    out, err = _run_argv(_CONNECTIONS_CMD, _SYSTEM_STATE_TIMEOUT)
    if err:
        return err
    pairs: Dict[tuple, int] = {}
    loopback = 0
    for ln in (out or '').splitlines()[1:]:      # drop the header row
        fields = ln.split()
        if len(fields) < 5:
            continue
        peer = fields[4]
        proc = '?'
        if len(fields) > 5:
            m = re.search(r'\("([^"]+)"', fields[5])
            if m:
                proc = m.group(1)
        if peer.startswith(('127.', '[::1]', '::1')):
            loopback += 1
            continue
        pairs[(proc, peer)] = pairs.get((proc, peer), 0) + 1
    if not pairs:
        return (f"EMPTY: no off-box established connections "
                f"({loopback} loopback)")

    by_proc: Dict[str, list] = {}
    for (proc, peer), n in pairs.items():
        by_proc.setdefault(proc, []).append((peer, n))

    lines, summarized = [], []
    for proc in sorted(by_proc):
        peers = sorted(by_proc[proc])
        # A browser fans out to a hundred hosts and reviewing that list
        # teaches nobody anything; a process with a handful of peers is
        # exactly what can be read. So list the readable ones and give
        # the fan-out a count — the count still moves if it changes.
        if len(peers) > _CONNECTIONS_FANOUT:
            summarized.append(f"  {proc:<18} {len(peers)} distinct peers "
                              f"(too many to review; not listed)")
            continue
        lines.append(f"  {proc}:")
        lines.extend(f"      {peer:<30} x{n}" for peer, n in peers)

    head = (f"Off-box established connections by process. "
            f"{loopback} loopback connection(s) not listed; addresses not "
            f"resolved. What to notice is a process talking somewhere "
            f"unrelated to what that process is for.")
    body = '\n'.join(lines + (["", "High fan-out, listed as counts only:"]
                              if summarized else []) + summarized)
    return _truncate('OK: ' + head + '\n' + body, 'category')


def _tool_system_state(category: str) -> str:
    cat = (category or '').strip().lower()
    if cat == 'connections':
        return _connections_report()
    if cat not in _SYSTEM_STATE_COMMANDS:
        valid = ', '.join(sorted(_SYSTEM_STATE_COMMANDS.keys()) + ['connections'])
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


def _run_sudo(cat: str) -> str:
    """Run a Tier-2 fenced probe. Distinguishes 'sudoers entry absent'
    (a setup state, reported as EMPTY with a pointer) from a genuine
    command failure, so an unconfigured host doesn't read as a scan
    finding."""
    cmd = _SUDO_HOST_COMMANDS[cat]
    if shutil.which('sudo') is None:
        return "ERROR: sudo is not on PATH"
    if not Path(cmd[2]).exists():
        return (f"EMPTY: {Path(cmd[2]).name} is not installed on this host "
                f"— {cat} is unavailable")
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=_HOST_STATE_TIMEOUT, check=False,
                              stdin=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        return f"ERROR: {cat} timed out (>{int(_HOST_STATE_TIMEOUT)}s)"
    except Exception as e:
        return f"ERROR: {cat} failed to launch: {e}"
    err = (proc.stderr or '').strip()
    if proc.returncode != 0:
        low = err.lower()
        # Wording varies across sudo versions: "a password is required"
        # (classic), "interactive authentication is required" (sudo-rs,
        # Ubuntu 25.10+), "not allowed to execute" / "may not run"
        # (entry present but this command not permitted).
        if ('password is required' in low
                or 'interactive authentication is required' in low
                or 'authentication is required' in low
                or 'not allowed to execute' in low
                or 'may not run' in low):
            return (f"EMPTY: {cat} needs a sudoers entry that is not "
                    "installed on this host — the privileged probes are "
                    "unconfigured, which is a setup gap, NOT a security "
                    "finding. See docs/sentinel-setup.md.")
        return (f"ERROR: {cat} exited {proc.returncode}: "
                + ' | '.join(err.splitlines()[:3]))
    out = (proc.stdout or '').strip()
    if not out:
        return f"EMPTY: {cat} returned no output"
    return _truncate('OK: ' + out, 'category')


def _host_accounts_report() -> str:
    """Login-capable and uid-0 accounts from the world-readable passwd
    database. A second uid-0 account, or a system account that has
    grown a login shell, is the signal worth surfacing."""
    out, err = _run_argv(['getent', 'passwd'], _HOST_STATE_TIMEOUT)
    if err:
        return err
    uid0, human, shelled = [], [], []
    nologin = ('/usr/sbin/nologin', '/sbin/nologin', '/bin/false',
               '/usr/bin/false')
    for line in (out or '').splitlines():
        parts = line.split(':')
        if len(parts) < 7:
            continue
        name, uid, shell = parts[0], parts[2], parts[6]
        try:
            uid_i = int(uid)
        except ValueError:
            continue
        entry = f'{name} uid={uid} shell={shell}'
        if uid_i == 0:
            uid0.append(entry)
        elif 1000 <= uid_i < 65534:
            human.append(entry)
        elif shell not in nologin:
            shelled.append(entry)
    sections = [
        '## uid 0 (root-equivalent)\n' + ('\n'.join(uid0) or '(none)'),
        '## regular accounts (uid 1000-65533)\n' + ('\n'.join(human) or '(none)'),
        '## system accounts with a login shell\n'
        + ('\n'.join(shelled) or '(none)'),
    ]
    return '\n\n'.join(sections)


def _host_user_persistence_report() -> str:
    """User-level persistence surfaces. All writable without privilege,
    which is exactly why they matter: nothing here needs root to plant,
    so a compromise of the user account lands here first."""
    home = Path.home()
    sections: List[str] = []

    autostart = home / '.config' / 'autostart'
    if autostart.is_dir():
        entries = []
        for f in sorted(autostart.iterdir()):
            if not f.is_file():
                continue
            try:
                st = f.stat()
                entries.append(f'{f.name}  ({st.st_size}b, mtime '
                               f'{datetime.fromtimestamp(st.st_mtime).isoformat(timespec="seconds")})')
            except OSError as e:
                entries.append(f'{f.name}  (stat failed: {e})')
        sections.append('## ~/.config/autostart\n'
                        + ('\n'.join(entries) if entries else '(empty)'))
    else:
        sections.append('## ~/.config/autostart\n(missing)')

    out, err = _run_argv(['systemctl', '--user', 'list-unit-files',
                          '--state=enabled', '--no-legend', '--plain',
                          '--no-pager'], _HOST_STATE_TIMEOUT)
    sections.append('## enabled systemd --user units\n'
                    + (err or out or '(none)'))

    rc_lines = []
    for rel in ('.bashrc', '.bash_profile', '.profile', '.bash_login',
                '.zshrc', '.config/systemd/user'):
        p = home / rel
        if not p.exists():
            continue
        try:
            st = p.stat()
            kind = 'dir' if p.is_dir() else f'{st.st_size}b'
            rc_lines.append(
                f'{rel}  ({kind}, mtime '
                f'{datetime.fromtimestamp(st.st_mtime).isoformat(timespec="seconds")})')
        except OSError as e:
            rc_lines.append(f'{rel}  (stat failed: {e})')
    sections.append('## shell startup files (mtime — recent edits are the '
                    'signal)\n' + ('\n'.join(rc_lines) or '(none)'))
    return '\n\n'.join(sections)


def _host_ssh_config_report() -> str:
    """Authentication-relevant sshd settings. sshd_config is
    world-readable; the *effective* config would need `sshd -T` (root),
    so absent directives mean the compiled-in default applies."""
    paths = [Path('/etc/ssh/sshd_config')]
    drop_in = Path('/etc/ssh/sshd_config.d')
    if drop_in.is_dir():
        paths.extend(sorted(p for p in drop_in.glob('*.conf')))
    interesting = ('passwordauthentication', 'permitrootlogin',
                   'pubkeyauthentication', 'permitemptypasswords',
                   'port', 'listenaddress', 'allowusers', 'allowgroups',
                   'kbdinteractiveauthentication',
                   'challengeresponseauthentication')
    sections = []
    for p in paths:
        body = _read_file_tail(p, 500)
        if body is None:
            sections.append(f'## {p}\n(unreadable)')
            continue
        hits = [ln.strip() for ln in body.splitlines()
                if ln.strip() and not ln.strip().startswith('#')
                and ln.strip().split()[0].lower() in interesting]
        sections.append(f'## {p}\n' + ('\n'.join(hits)
                                       or '(no auth-relevant directives set)'))
    sections.append('NOTE: directives absent above run at sshd defaults '
                    '(PasswordAuthentication yes, PermitRootLogin '
                    'prohibit-password). The effective merged config needs '
                    '`sshd -T`, which requires root.')
    return '\n\n'.join(sections)


def _tool_host_state(category: str) -> str:
    cat = (category or '').strip().lower()
    valid_cats = sorted(list(_HOST_STATE_COMMANDS.keys())
                        + list(_SUDO_HOST_COMMANDS.keys())
                        + ['cron', 'ssh_keys', 'unattended_log', 'accounts',
                           'user_persistence', 'ssh_config'])
    if cat in _SUDO_HOST_COMMANDS:
        return _run_sudo(cat)
    if cat == 'cron':
        return _truncate('OK: ' + _host_cron_report(), 'category')
    if cat == 'accounts':
        return _truncate('OK: ' + _host_accounts_report(), 'category')
    if cat == 'user_persistence':
        return _truncate('OK: ' + _host_user_persistence_report(), 'category')
    if cat == 'ssh_config':
        return _truncate('OK: ' + _host_ssh_config_report(), 'category')
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
    if cat in ('sockets', 'udp_sockets'):
        # ss -tlnH: State Recv-Q Send-Q Local:Port Peer:Port [Process].
        # Keep only the local address:port — queue counters are volatile
        # and the process column is privilege-dependent.
        keep = []
        for ln in lines:
            fields = ln.split()
            if len(fields) >= 4:
                keep.append(fields[3])
        lines = keep
    elif cat == 'arp':
        # `ip neigh show` lines are "<ip> dev <if> lladdr <mac> <state>".
        # Reduce to the MAC alone: the IP moves with DHCP, the state
        # (REACHABLE/STALE/DELAY) changes minute to minute, and a
        # dual-homed host lists the same device once per interface — all
        # three would diff on every run and bury a real arrival. The MAC
        # is the device's identity, so "a MAC I have not seen before" is
        # the signal. Entries with no lladdr are unresolved probes, not
        # devices, and are dropped. When something new does appear, read
        # system_state('arp') for the address behind it.
        keep = []
        for ln in lines:
            fields = ln.split()
            if 'lladdr' in fields:
                keep.append(fields[fields.index('lladdr') + 1].lower())
        lines = keep
    elif cat in ('enabled_units', 'user_units'):
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

    have_previous = (previous is not None
                     and isinstance(previous.get('lines'), list))
    prev_lines = (set(str(x) for x in previous['lines'])
                  if have_previous else set())
    cumulative = cat in _CUMULATIVE_BASELINES
    to_store = sorted(prev_lines | set(current)) if cumulative else current

    now = datetime.now(timezone.utc).isoformat()
    try:
        baseline_dir.mkdir(parents=True, exist_ok=True)
        snap_path.write_text(
            json.dumps({'captured_at': now, 'lines': to_store}, indent=1),
            encoding='utf-8')
    except OSError as e:
        logger.warning(f"security: baseline write failed for {cat}: {e}")
        return f"ERROR: could not write snapshot {snap_path}: {e}"

    if not have_previous:
        return (f"OK: no prior {cat} snapshot — baseline established now "
                f"({len(to_store)} entries). Diffs start from the next call.")
    added = [ln for ln in current if ln not in prev_lines]
    removed = [] if cumulative else sorted(prev_lines - set(current))
    when = previous.get('captured_at', 'unknown time')
    if not added and not removed:
        if cumulative:
            return (f"OK: no {cat} entries beyond the {len(prev_lines)} "
                    f"already known (last seen {when})")
        return (f"OK: {cat} unchanged since snapshot of {when} "
                f"({len(current)} entries)")
    parts = [f"{cat} changed since snapshot of {when}:"]
    if added:
        parts.append('ADDED:\n' + '\n'.join(f'  + {ln}' for ln in added))
    if removed:
        parts.append('REMOVED:\n' + '\n'.join(f'  - {ln}' for ln in removed))
    if cumulative:
        parts.append(f'(snapshot is cumulative — {len(to_store)} entries '
                     f'now known; entries are never dropped, so ADDED means '
                     f'first-ever sighting)')
    else:
        parts.append('(snapshot updated to current state)')
    return _truncate('OK: ' + '\n'.join(parts), 'category')




def call_deadline() -> float:
    """A fresh monotonic deadline for security work.

    A caller that may invoke security() several times in ONE turn must
    compute this ONCE and pass the same value to every call, so the
    budgets share one clock instead of multiplying. Without that, a
    per-call cap bounds nothing: the parent ReAct loop simply calls again
    and gets a fresh budget (observed 2026-08-10, twice in one turn)."""
    return time.monotonic() + _CALL_BUDGET


def security(query: str, llm_backend, trace_dir: Path,
             baseline_dir: Optional[Path] = None,
             deadline: Optional[float] = None,
             reasoning_effort: Optional[str] = None) -> str:
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
        deadline: monotonic time after which no further probes run. Pass
            one shared value for every call in a turn (see call_deadline);
            omit it and this call gets its own _CALL_BUDGET.
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
    # Whichever runs out first: this call's own budget, or the caller's
    # shared one. min() means a second call in the same turn inherits
    # what's left rather than starting over.
    own = time.monotonic() + _CALL_BUDGET
    deadline = own if deadline is None else min(float(deadline), own)
    for i in range(_MAX_ITERS):
        messages = [
            {'role': 'system', 'content': sys_prompt},
            {'role': 'user', 'content': _build_user_msg()},
        ]
        try:
            raw = llm_backend.chat(messages, max_tokens=4096, temperature=0.2,
                                   reasoning_effort=reasoning_effort)
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
        if time.monotonic() >= deadline:
            # Budget spent. Refuse the probe rather than the answer: the
            # model still gets a turn to report what it has, and the
            # iteration cap ends the loop if it asks for another probe
            # anyway. Not silent — a truncated survey the caller believes
            # is complete is worse than a short one that says so.
            logger.warning(
                f"security: call budget of {int(_CALL_BUDGET)}s spent at "
                f"iter {i+1}; refusing further probes")
            obs = (f"ERROR: security work has used its whole time budget "
                   f"({int(_CALL_BUDGET)}s for this turn) and no further "
                   f"probes will run — calling this tool again will not "
                   f"help. Respond NOW with what the log above already "
                   f"shows, and say plainly which part of the question you "
                   f"did not get to — do not present a partial survey as a "
                   f"complete one.")
        elif tool == 'discover':
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

    write_subagent_trace(trace_dir, 'security', query, iters, answer,
                         exit_reason)
    return answer
