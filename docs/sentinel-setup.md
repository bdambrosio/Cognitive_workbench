# Sentinel — desktop security agent

Second character in `scenarios/jill-chat.yaml` (2026-08-08). Runs a daily
autonomous patrol of the desktop: patch-posture verification, Ubuntu
Security Notice cross-referencing, and an unprivileged host/intrusion
scan. Durable record is a daily Obsidian note (`Security/<date>-patrol.md`);
findings needing action are reported conversationally.

Privilege posture (the design's core decision): **no LLM ever holds
sudo**. `unattended-upgrades` does all privileged mutation on its own
schedule; Sentinel verifies it ran, scans read-only, and recommends
exact commands for Bruce to run when root-level action is warranted.

## One-time system setup (run as Bruce)

1. Security patching via unattended-upgrades:

   ```bash
   sudo apt install unattended-upgrades
   sudo dpkg-reconfigure -plow unattended-upgrades   # answer Yes
   ```

   Verify `/etc/apt/apt.conf.d/20auto-upgrades` contains:

   ```
   APT::Periodic::Update-Package-Lists "1";
   APT::Periodic::Unattended-Upgrade "1";
   ```

2. Log access for the scanning user. `id` must show `adm` (journal +
   `/var/log/unattended-upgrades/`); on Ubuntu the first user normally
   has it. If missing:

   ```bash
   sudo usermod -aG adm bruce   # then log out/in
   ```

## Exposure posture (higher leverage than scan depth)

Against external actors, reducing what is reachable beats detecting
intruders after the fact — and it is what a daily patrol verifies well.
Reviewed 2026-08-08:

- **mcpo** (`openwebui-stack` compose) served the Obsidian MCP proxy on
  `0.0.0.0:8000` with mcpo's placeholder api-key `top-secret`, granting
  full vault read/write to anything that could reach the port. Fixed by
  adding `--host 127.0.0.1` to its command in
  `/home/bruce/openwebui-stack/docker-compose.yaml`. It keeps
  `network_mode: host`, so `OBSIDIAN_BASE_URL=http://127.0.0.1:27123`
  still resolves to the host's vault API — switching it to bridge
  networking instead would have broken that. Still worth doing: replace
  the placeholder api-key, and consider `--strict-auth` (which also
  protects `/openapi.json` and `/docs`, currently unauthenticated).
- **grobid** publishes `0.0.0.0:8070`. Its live definition is the root
  unit `/etc/systemd/system/grobid.service` (`Restart=always`), which
  recreates the container ~10s after any manual `docker rm`, so
  rebinding by hand does not stick. Needs root — see below.
  `~/bin/start-grobid.sh` (the manual variant) is already updated.
- **sshd** listens on `0.0.0.0:22` with only
  `KbdInteractiveAuthentication no` set, so `PasswordAuthentication`
  runs at its default of **yes**. Whether that is exposed beyond the
  LAN depends on the ufw ruleset, which is unreadable without Tier 2.

Fixing grobid (root required):

```bash
sudo sed -i 's|-p 8070:8070|-p 127.0.0.1:8070:8070|' /etc/systemd/system/grobid.service
sudo systemctl daemon-reload
sudo systemctl restart grobid.service
curl -s http://127.0.0.1:8070/api/isalive   # expect: true
```

Nothing consumes grobid off-host: `src/utils/grobid.py` uses
`http://localhost:8070`.

## Running

The patrol is an agent concern (`daily-security-patrol`,
`rhythm_hours: 24`) and fires only when the scenario runs with
autonomy:

```bash
cd src && python3 launcher.py jill-chat.yaml --cli --autonomy
```

Talk to it in the CLI with `/char Sentinel` (e.g. "run a security
patrol now"). Activation growth is wall-clock based, so after downtime
the patrol fires on the first tick — a catch-up scan on boot is the
intended behavior.

Scan machinery lives in the `security` subagent
(`src/chat/security.py`): fixed read-only command vocabulary,
argument-list-only subprocess, no shell. Baseline snapshots (SUID
files, listening sockets, enabled units) are JSON under
`scenarios/jill_chat/Sentinel/memory/security_baselines/`; a
`baseline_diff` reports a change once, then re-baselines — the daily
note is where flagged changes persist.

Known unprivileged limits (stated honestly in reports): no kernel-level
rootkit visibility, no root-only directories in the SUID walk, no
traffic capture / IDS.

## Tier 2 — fenced sudo for the three privileged probes

Three questions are structurally unanswerable without privilege, and
all three matter more than deeper rootkit scanning:

| Category | Answers |
|---|---|
| `firewall` | what ufw actually permits (enablement alone is visible unprivileged; the ruleset is not) |
| `listener_owners` | which process owns a **root-owned** listener — `ss -tlnp` shows those with no attribution at all |
| `apparmor` | MAC profile detail (unprivileged gets only "module is loaded") |

The fence is **the sudoers file, not the tool code**: `security.py`
invokes `sudo -n` with a fixed argument list and passes zero
caller-controlled arguments, so sudo rejects any deviation. A prompt
injection can at worst re-run the same read-only probe already
authorized. Until the file is installed the probes report EMPTY with a
setup-gap note, which the subagent is instructed not to treat as a
finding.

Install (verified against **sudo-rs**, Ubuntu's Rust sudo — check
`sudo --version`):

```bash
sudo tee /etc/sudoers.d/sentinel >/dev/null <<'EOF'
# Sentinel security agent — read-only privileged probes.
# Argument-exact, no wildcards: sudo rejects any deviation.
bruce ALL=(root) NOPASSWD: /usr/sbin/ufw status verbose
bruce ALL=(root) NOPASSWD: /usr/bin/ss -tulpnH
bruce ALL=(root) NOPASSWD: /usr/sbin/aa-status
EOF
sudo chmod 0440 /etc/sudoers.d/sentinel
sudo visudo -c -f /etc/sudoers.d/sentinel   # must print "parsed OK"
sudo -n /usr/sbin/ufw status verbose        # must work without a prompt
```

Rules that bite, in order of how easily they're missed:

- **No dot or tilde in the filename.** sudo silently ignores
  `sudoers.d` files containing them — no error, just no effect.
- **Mode `0440`, owner `root:root`**, or sudo refuses the file.
- **Keep a second root shell open** while installing, and run
  `visudo -c` before trusting it. A malformed sudoers can lock you out
  of sudo entirely.
- **Never wildcards.** `/usr/bin/rkhunter *` would permit
  `--configfile /tmp/evil.conf`, and that config can specify commands
  to run as root. Every argument spelled out.
- **Full paths must match what the tool invokes.** `aa-status` is used
  rather than its `apparmor_status` symlink so the fenced path is
  unambiguous.
- **Don't sudo-wrap `cat` or `find`** to reach root-only files. For
  read access to a specific path, a group or POSIX ACL is tighter and
  more auditable than a sudo entry.

Residual risk after fencing is denial-of-service — an injected agent
looping an expensive probe. These three are cheap; add a
minimum-interval guard in `_run_sudo` if that ever changes.

## Deferred: root-only scanners

For heavier integrity work (`debsums`, AIDE, a complete SUID walk,
`/root/.ssh`, other users' crontabs), prefer a **root systemd timer**
that writes reports to a directory Sentinel reads. The agent side then
stays entirely unprivileged, and the scan keeps running when the agent
is not. For anything periodic this dominates the sudo approach; sudo
fencing earns its keep only where liveness matters.

`rkhunter` / `chkrootkit` are deliberately not recommended: their
signature sets target long-obsolete rootkits and their false-positive
rate trains both agent and human to discount warnings, which is worse
than not running them.

## Note on the privilege boundary

"No LLM holds sudo" is the design intent, but on this host it is not
OS-enforced: the `bruce` account is in the `docker` group, so anything
running as that user can `docker run -v /:/host` and become root. The
effective boundary is the **tool inventory** — no docker tool exists
and `exec-script` is broken from the chat path. Keep that deliberate;
it is doing more work than the absence of a sudoers file.
