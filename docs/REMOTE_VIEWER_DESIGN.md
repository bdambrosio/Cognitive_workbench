# Remote Viewer for Cognitive Workbench — Design Note

**Goal.** Run the workbench on the desktop; see and drive its three surfaces — CLI, affect, canvas — from the laptop.

**Constraints (from you).** Same home LAN. Fully interactive (not view-only). Prefer no code changes.

**Status.** Verified working end-to-end on 2026-05-25 — affect, canvas, the SSH transport, and the interactive CLI (over tmux) all confirmed against the live desktop instance. See Runbook.

---

## Runbook (verified 2026-05-25)

The canonical, concrete setup. Everything below was confirmed against the live box; the rest of this note is the reasoning behind it.

**Desktop.** `bruce@192.168.68.76` — Ubuntu 25.10, hostname `bruce-TRX50-WS`.
The SSH account is **`bruce`**, not the laptop's `dambrosiobruce` (the username mismatch is the easiest thing to trip on). It runs **natively, not in Docker** — the `grobid-server` / `mcpo` containers on that box are unrelated. The display bridges listen on host loopback: `127.0.0.1:8787` (affect WS), `127.0.0.1:8788` (canvas WS), `127.0.0.1:8789` (canvas image-proxy).

**Prerequisite — the LLM.** Jill talks to a *separate* OpenAI-compatible LLM server on `127.0.0.1:5000` that Bruce starts manually at the start of the day. `jill-chat.yaml` sets `sgl_model_path: null`, so the launcher connects to that server rather than loading any model of its own; `mirror.sh wb-start` does **not** start the LLM. Start the LLM first.

**Desktop launch line** (run inside tmux so the CLI is remotable; this is what `wb-start` runs):

```
cd /home/bruce/Downloads/Cognitive_workbench/src && \
  exec /home/bruce/Downloads/Cognitive_workbench/zenoh_venv/bin/python3 \
  launcher.py jill-chat.yaml --cli --affect --canvas --autonomy
```

(`--resource-browser` is intentionally omitted — it's a local UI not needed from the laptop.)

**Environment.** `wb-start` runs that line through a login+interactive shell (`bash -lic`) so it inherits `~/.bashrc` — notably `HF_HOME=/data/huggingface` (where the `bge-small` embedding model is cached) and the other env vars the agent needs. A bare `ssh host "cmd"` or `tmux` command runs a *non-interactive* shell, which never sources `~/.bashrc`; that's what made the embedder look in the default cache and fail with a permission error.

tmux session name used by tooling: **`cwb`**.

**One-time laptop setup.**

```bash
ssh-copy-id bruce@192.168.68.76                       # install key (asks password once)
ssh-add --apple-use-keychain ~/.ssh/id_ed25519       # load key passphrase into the agent
```

**Daily use** (laptop, from the repo root — `mirror.sh` wraps all of it):

```bash
./mirror.sh            # tunnel (8787/8788/8789) + open affect/canvas + attach the CLI
./mirror.sh down       # close the tunnel (desktop keeps running)
./mirror.sh status     # tunnel state here; tmux/process state there
```

Inside the attached CLI, `Ctrl-b d` detaches and leaves everything running.

**Lifecycle / after a desktop reboot** (nothing auto-starts — tmux doesn't survive a reboot):

```bash
# first: start your separate LLM server (manual, ~start of day)
./mirror.sh wb-start     # launch the workbench in tmux 'cwb' (connects to the LLM)
./mirror.sh up           # tunnel + views + attach
./mirror.sh wb-stop      # clean shutdown (Ctrl-D to the CLI → graceful drain, ~30s)
./mirror.sh wb-restart   # wb-stop then wb-start
```

Clean *remote* shutdown requires the tmux model; a workbench started outside tmux can only be stopped at the desktop (its `prompt_toolkit` CLI holds the terminal and won't respond to external signals).

**Smoke test** (independent of `mirror.sh`; validates the tunnel and reads a real frame from each surface):

```bash
python3 tools/smoke_test_remote_viewer.py --ssh bruce@192.168.68.76
```

Last run (2026-05-25): all three display checks passed — affect delivered a live `ProcState`, canvas connected, proxy answered.

To override the host without editing the script: `CWB_HOST=bruce@<ip> ./mirror.sh …`.

---

## 1. What the code actually does today

This matters because it determines where a remote viewer can plug in. The three surfaces are *already* decoupled from the agent; they are not part of the agent process, they are clients of it.

The agent, CLI, affect, and canvas communicate over a **Zenoh pub/sub bus**. Affect and canvas additionally re-publish over local **WebSockets** to browser windows. Concretely:

- **CLI** (`src/cli.py`, run in-process by `launcher.py` via `run_cli`). It is a Zenoh client: it *publishes* your chat to `cognitive/{char}/sense_data` and `cognitive/{char}/command`, *subscribes* to the agent's `cognitive/*/action` (say/ask/announcement) and `cognitive/*/execution_state`, and issues request/reply queries (`session.get`) for `/status`, `/recall`, concerns, resource views, etc. Terminal I/O is `prompt_toolkit` (raw TTY, ANSI colour). It is genuinely bidirectional.
- **Affect** (`src/affect/`). In-process `AffectPublisher` pushes a small JSON state vector to `cognitive/affect/state` (latest-wins). A separate subprocess `python -m affect.display` subscribes and fans out over **`ws://127.0.0.1:8787`**. The browser page `src/affect/display/static/index.html` (a P5.js sketch) **hardcodes** `ws://127.0.0.1:8787`. One-way (display only).
- **Canvas** (`src/canvas/`). In-process `CanvasPublisher` pushes markdown/HTML to `cognitive/{char}/canvas` (latest-wins). Subprocess `python -m canvas.display` subscribes and fans out over **`ws://127.0.0.1:8788`**, and also runs an HTTP image-proxy + local-file server on **`127.0.0.1:8789`** (SSRF-guarded, allowlisted to `~/.cache/cognitive`). The browser page `src/canvas/display/static/index.html` computes its WS URL as `ws://${location.hostname||127.0.0.1}:8788` and carries a strict **Content-Security-Policy that only permits `127.0.0.1`** for `connect-src` and `img-src`. The agent embeds images as `http://127.0.0.1:8789/proxy?url=…` (the port is hardcoded in `chat/chat_loop.py`). One-way (display only).

**The binding posture is loopback everywhere, by design.** `utils/zenoh_utils.make_localhost_config()` pins Zenoh to `127.0.0.1` (macOS: gossip + a fixed `tcp/127.0.0.1:7447` rendezvous plus an ephemeral port; Linux: multicast on `lo`, listen on `tcp/127.0.0.1:0`). Both WebSocket bridges and the canvas proxy bind `127.0.0.1`. There is **no authentication anywhere** — the whole system assumes one trusted machine.

So "remote viewing" reduces to one question: **how do you bridge a loopback-only system to the laptop?**

---

## 2. The design decision: bridge *above* Zenoh, not at it

There are three layers you could cut at. Only one fits "same LAN + interactive + no code changes," and the reasoning is worth stating because it's the whole design.

**Don't try to distribute the Zenoh bus.** It's the tempting "native" move (Zenoh 1.4 supports routers, TCP/TLS links, and ACLs), but it fights the current code: the macOS config uses *gossip discovery*, which advertises each peer's **loopback ephemeral endpoint** to others. A remote peer reached over a tunnel would be told to connect back to `127.0.0.1:<ephemeral>`, which on the laptop points at the wrong machine. Making this work means rewriting `make_localhost_config`, adding listen endpoints on a real interface, and layering on TLS + ACLs (because the bus is the agent's *control plane*, not just a view feed). That is a real code change and a real security project. Ruled out by your constraints.

**Don't bind the WebSocket bridges to the LAN.** The env vars exist (`AFFECT_WS_HOST`, `CANVAS_WS_HOST`), so setting them to `0.0.0.0` looks like a config-only fix. It isn't, for two independent reasons. First, the **canvas CSP only allows `127.0.0.1`** — a browser pointed at `ws://<desktop-LAN-IP>:8788` or at LAN-IP proxied images is *blocked by the browser itself*, and the affect page hardcodes `127.0.0.1` so it wouldn't connect to a LAN IP at all without editing the file. Second, it would expose the agent **unauthenticated** on your LAN. So this path is simultaneously broken (CSP) and unsafe. Ruled out.

**Bridge at the existing boundaries — the WebSocket ports and the terminal — and let SSH carry them.** This is the recommendation. SSH local port-forwarding makes the desktop's loopback ports appear on the laptop's loopback. Because the laptop browser then genuinely talks to *its own* `127.0.0.1`, **every loopback assumption in the code is satisfied unchanged** — the hardcoded affect URL, the canvas `location.hostname` fallback, and crucially the canvas CSP. SSH also gives you authentication and encryption for free, and nothing on the desktop ever leaves loopback. Zenoh is never touched; you bridge one layer above it, where the system already exposes WS and a TTY.

The cost of "no code changes" is honest and small: you need a copy of two static HTML files on the laptop (a file copy, not an edit), and you need to start the workbench inside `tmux` so the CLI's terminal can be attached remotely (a launch habit, not an edit). Details below.

---

## 3. Recommended design

One SSH relationship does all three surfaces. Two browser tabs and one terminal on the laptop.

### 3.1 Affect + canvas (the two browser surfaces)

**On the laptop**, open one SSH tunnel that forwards all three display ports:

```bash
ssh -N \
  -L 8787:127.0.0.1:8787 \
  -L 8788:127.0.0.1:8788 \
  -L 8789:127.0.0.1:8789 \
  bruce@desktop.local
```

- `8787` — affect WebSocket
- `8788` — canvas WebSocket
- `8789` — canvas image-proxy + local-file server (required, or canvas images/meshes 404)

`-N` means "forward only, no remote shell." Leave it running.

**On the laptop**, you need the two page files locally (the bridges serve only WS/HTTP, not the HTML; the pages are opened via `file://`). If the repo is already checked out on the laptop, just open them. Otherwise copy the two files once:

```bash
scp bruce@desktop.local:~/Projects/Cognitive_workbench/src/affect/display/static/index.html  ~/cwb-viewer/affect.html
scp bruce@desktop.local:~/Projects/Cognitive_workbench/src/canvas/display/static/index.html  ~/cwb-viewer/canvas.html
```

Then open `affect.html` and `canvas.html` in the laptop browser. Each page connects to `ws://127.0.0.1:<port>` → SSH → the desktop bridge. The canvas page fetches images from `http://127.0.0.1:8789/proxy?url=…` → SSH → the desktop proxy, which does the real fetch (SSRF guard intact). The CSP is happy because, from the browser's point of view, it's all `127.0.0.1`.

> If you keep the repo on the laptop, `file://…/src/affect/display/static/index.html` works directly with no copy step.

### 3.2 CLI (the interactive surface)

The CLI can't be run as a second client — `cli.py` has no standalone entry and needs the launcher's Zenoh session. So you attach to the *real* CLI process over SSH. Because `run.sh` runs the CLI in the foreground, start it inside a multiplexer so it can be attached remotely.

**On the desktop**, launch inside tmux:

```bash
tmux new -s cwb './run.sh'
```

**On the laptop**, attach over SSH:

```bash
ssh -t bruce@desktop.local 'tmux attach -t cwb'
```

You get the live CLI — full input, `/commands`, colour, scrollback — encrypted and authenticated. `prompt_toolkit` behaves correctly over SSH + tmux.

Caveat worth knowing: tmux *mirrors* one session. If you're also sitting at the desktop, both terminals see and drive the same CLI. For one person moving between machines that's exactly what you want; it is not two independent CLIs.

### 3.3 Optional convenience

Fold everything into one SSH config block so it's a single command:

```
# ~/.ssh/config on the laptop
Host cwb
    HostName desktop.local
    User bruce
    LocalForward 8787 127.0.0.1:8787
    LocalForward 8788 127.0.0.1:8788
    LocalForward 8789 127.0.0.1:8789
    RequestTTY yes
```

Then `ssh cwb -t 'tmux attach -t cwb'` opens the CLI *and* stands up all three forwards in one shot; open the two HTML files alongside it.

---

## 4. Security assessment (fair witness)

- **SSH is the right trust boundary.** It authenticates the laptop (keys), encrypts the link, and keeps every workbench service on desktop loopback. The only thing exposed on the LAN is sshd, which you presumably already trust. This is strictly safer than any "bind to the LAN" variant.
- **The agent has no auth of its own.** Whoever can reach those ports can drive the agent (send chat, run commands, request `/shutdown` via `cognitive/launcher/shutdown`). SSH is what supplies the missing authentication. Do **not** "simplify" later by binding the bridges to `0.0.0.0` to skip the tunnel — that removes the only access control in the system.
- **Same-LAN is not the same as trusted-LAN.** On a shared/guest network, loopback-only + SSH still protects you; LAN-bound services would not. The tunnel choice keeps you safe even if the network isn't.
- **The canvas proxy's SSRF guard stays effective** because the proxy still runs on the desktop and only loopback traffic (yours, over SSH) reaches it. Tunneling doesn't widen that surface.

---

## 5. Where this design is weak / open questions

Stated plainly so it's a useful sounding board, not a sales pitch:

- **"No code changes" bends twice.** You copy two HTML files to the laptop, and you adopt a tmux launch habit. Neither edits the codebase, but both are setup the current code doesn't do for you. If even that friction is unwanted, the only way to remove it is a small new module (a single Zenoh-peer→authenticated-web bridge that serves its own page and re-exposes all three surfaces over one TLS endpoint) — which is more code, not less.
- **tmux mirroring, not multi-seat.** If you ever want the laptop and desktop to have *independent* CLI sessions, this approach can't do it; the CLI is singular by construction. That needs code (a standalone CLI client that opens its own Zenoh session).
- **The CLI is a terminal, not a browser pane.** If your real desire is "all three in one browser window on the laptop," SSH+tmux doesn't deliver that; it gives you a terminal plus two browser tabs. A unified web view is the new-module path.
- **Reconnect behaviour.** The browser pages auto-retry their WebSocket, so a dropped tunnel recovers once you re-establish SSH. The CLI/tmux survives SSH drops (the session persists on the desktop; just re-attach). Worth confirming in practice.
- **Verified end-to-end (2026-05-25).** Affect, canvas, the SSH transport, and the interactive CLI (via tmux, launched through `bash -lic` so it inherits `~/.bashrc` / `HF_HOME`) all confirmed working against the live instance. No remaining unverified claims.

---

## 6. Verification checklist (when you try it)

1. Desktop: `./run.sh` inside tmux comes up; affect + canvas Chromium windows appear locally as usual.
2. Laptop: open the SSH tunnel; confirm `nc -z 127.0.0.1 8787 8788 8789` (or browser dev-tools) shows the forwards live.
3. Laptop: open `affect.html` → the P5 sketch animates in step with desktop activity (status line clears to blank = WS connected).
4. Laptop: open `canvas.html` → push content from the agent (e.g. trigger a `display` action); confirm markdown renders and at least one proxied image loads (proves 8789 is forwarded and CSP is satisfied).
5. Laptop: `ssh cwb -t 'tmux attach -t cwb'` → type a chat turn, confirm the agent responds and a `/status` query returns.
6. Drop and re-open the tunnel → confirm both browser pages reconnect on their own.

---

## 7. If the constraints change later

- **Want one browser window with all three, or independent sessions, or laptop-native bridges:** that's the "one small new module" path — a Zenoh peer that joins the loopback bus and re-serves affect + canvas + the CLI action/command streams over a single authenticated (TLS + token) WebSocket/HTTP endpoint. Existing files stay untouched; the canvas image-proxy logic can be reused. Larger effort, but it removes the file-copy and tmux habits and gives a true unified remote UI.
- **Want to reach it from outside the house:** keep this exact design and put both machines on a mesh VPN (Tailscale/WireGuard). The SSH command is unchanged; only the hostname changes. No new code.
