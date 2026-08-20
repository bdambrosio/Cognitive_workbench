---
name: exec-script
description: Run a bash/shell script, or write code and execute it — sandboxed, with a persistent scratch directory and scientific Python. Read-only elsewhere, no network.
args:
  script: required string — the bash script text to execute
---

# exec-script

Run an arbitrary bash script via `bash -c` inside a bubblewrap sandbox, with
the repo root as the working directory. Returns stdout + stderr as a single
text observation.

Two things it is for: **inspecting** (read the source, check what commit the
working tree is on) and **computing** — writing a program, running it, fixing
it, and either handing back its output or the program itself.

## Where you can write

`$SCRATCH` is a directory that is yours and **persists between calls**. Write
a program there, run it in the next call, edit it in the one after. It is the
only durable writable path.

```json
{"thought": "compute the root numerically", "tool": "exec-script", "script": "cat > $SCRATCH/solve.py <<'EOF'\nfrom scipy import optimize\nimport numpy as np\nprint(optimize.brentq(lambda x: np.cos(x) - x, 0, 1))\nEOF\npython3 $SCRATCH/solve.py"}
```

`/tmp` is also writable but is wiped when the script exits — use it for
intermediates you don't need again, and `$SCRATCH` for anything you'll come
back to. To hand back code you wrote, `cat` it: the whole observation reaches
the loop.

## What the sandbox gives you

The script is not reviewed or approved — it is contained.

- **The repo, read-only.** cwd is the project root, so `git log`, `grep`,
  `find`, `wc` over the source all work. Writes anywhere in the tree fail
  with "Read-only file system" — that is expected, use `$SCRATCH`.
- **Python 3.14 with numpy, scipy, pandas, sympy, matplotlib, pillow and
  pytest.** Plain `python3` is already that interpreter. Also `gcc`, `jq`,
  `bc`. No `node`.
- **No network.** `pip install` will not work, and neither will fetching —
  use `fetch-text` / `tavily` / `search-web` for anything off-machine.

## What you cannot observe from in here

The sandbox has its own empty network and process namespaces. Commands that
ask about the machine do not fail — they return confident, well-formed,
**wrong** answers:

- `ss -tulpn` / `netstat` show **no listeners**, on a host with dozens. Do
  not conclude from this that a server is down.
- `ps aux` sees only the script itself, not the host's processes.
- `ip addr` shows only `lo` — no real interfaces, no LAN address.
- `journalctl` finds no journal; `sudo` cannot run at all.

So this tool can tell you what the *code* says and what a *computation*
yields, but nothing about what is currently running on this machine. For
that, ask, or use a tool built to read the live host.
- **No `$HOME` and an empty environment** — no API keys, no dotfiles, no ssh.
- 120-second timeout.

If bubblewrap is unavailable the tool refuses to run rather than falling back
to an unsandboxed shell.

## Notes

- Use focused single-purpose scripts; avoid pipelines that mix actions you
  don't both need to see results from.
- Prefer `fetch-text` / `obsidian` / `check-email` for tasks they cover —
  `exec-script` is the catch-all for what they don't.
- It cannot change the repo, only `$SCRATCH`. If a task needs a source file
  edited or a service touched, this is the wrong tool.
- A plot saved to `$SCRATCH` is a real file on disk, but you will not see it
  come back — only text returns through the observation.
