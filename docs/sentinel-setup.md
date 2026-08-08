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

## v1.1 (deferred): fenced sudo for root-only scanners

If deeper scanning (rkhunter, debsums) is wanted later, the pattern is
one tool per scanner where the **fence is the sudoers file, not the
tool code**:

- `src/tools/<scanner>/tool.py` invokes exactly
  `['sudo', '/usr/bin/<scanner>', '<fixed>', '<args>']` — zero
  caller-controlled arguments pass through.
- Paired with an argument-exact entry in `/etc/sudoers.d/sentinel`
  (no wildcards):

  ```
  bruce ALL=(root) NOPASSWD: /usr/bin/rkhunter --check --sk --nocolors
  bruce ALL=(root) NOPASSWD: /usr/bin/debsums -s
  ```

- sudo then rejects any deviation in binary or arguments, so a prompt
  injection can at worst run the same fixed read-only scan that was
  already authorized. The tool wrapper is convenience; sudoers is the
  boundary.

An equally valid alternative for scheduled scanners: a root systemd
timer runs the scanner and writes reports to a directory Sentinel
reads — the agent side stays entirely unprivileged.
