# Jill ↔ ChatterBot Integration

Status: **design + integration note.** Records how the Cognitive_workbench agent
("Jill") drives and senses through the ChatterBot head. The ChatterBot/Pi side
(head, camera, `xvf_audio`, DoA reflex) is implemented and live-verified on
hardware; the Cognitive_workbench side is partially built (head/camera tools,
vision) — see §7. Written to be shared with the Cognitive_workbench project as
well as kept here. Companion docs (in the ChatterBot repo): `docs/DESIGN.md`
(architecture), `docs/cw-voice-sensor.md` (the CW voice-sensor consumer guide —
formats, code, wake-word orient), `docs/xvf3800-setup.md` (mic bring-up + DoA
calibration).

Read alongside `DESIGN.md` (the ChatterBot architecture) — this note does not
restate it, it *binds* it. The key move:

> **Jill *is* the "desktop-side" tier in `DESIGN.md`.** Everywhere `DESIGN.md`
> says "the desktop" (STT, LLM, vision, gaze decisions, TTS), read
> "Cognitive_workbench / Jill." There is no separate desktop control app in the
> steady state.

## 1. Consequence of retiring the desktop app

`DESIGN.md §6` lists three head drivers: the Pi DOA reflex, desktop
"center detected object," and desktop "look-around." With the PyQt desktop
panel retired, the **deliberative** drivers all collapse into Jill. That removes
the desktop-vs-Jill conflict my earlier review worried about. What remains is a
two-driver problem, not three: the **Pi-local reflex** vs **Jill's deliberative
commands** (see §5).

## 2. Division of labor

| Concern | Lives on | Path | Latency target |
|---|---|---|---|
| Orient camera toward a voice (DOA → pan) | **Pi** (reflex) | XVF3800 DOA → `head_controller` directly | ~100–200 ms, no desktop round-trip |
| Speech → text | **Jill's sensor** | consumes Pi audio, runs VAD/STT, emits text | utterance-latency, async |
| Decide whether to *engage* (capture, identify, respond) | **Jill** (deliberative) | chat-loop + concerns layer → `head/cmd`, `camera/capture` | seconds OK |
| TTS | **Jill** → Pi `audio/out` | played through XVF3800 (AEC reference) | — |

Two placement decisions confirmed in discussion:

1. **"Orient toward voice" is a Pi-local reflex,** not a Jill action. Because the
   bot is stationary, DOA degrees map directly to a pan angle (`DESIGN.md §6`).
   Keeping it on the Pi means the head turns toward a speaker immediately, even
   if Jill is busy or the link is slow. Jill can *refine* the pose afterward
   (e.g. center a detected face) but does not own the reflex.
2. **Speech-to-text lives in Jill's sensor,** not on the Pi. The Pi ships audio
   (`chatter/audio/in`, VAD-gated binary PCM per `DESIGN.md §7`); Jill's
   `sensor_runner` does VAD-segment → STT → text. This honors "no audio
   processing on the Pi 4."

## 3. Connection layer: tools vs sensors

Jill's `src/sensor_runner.py` opens a **dedicated, isolated** zenoh session to
the Pi router:

- Connect to `tcp/<pi-ip>:7447`, multicast **off**, mirroring
  `desktop/transport.py`. **Do not** add the Pi endpoint to Jill's existing
  localhost gossip mesh (`make_localhost_config`) — that would federate
  Cognitive_workbench's entire topic space with the Pi router in both
  directions. Keep the Pi link a separate session.
- **Endpoint:** the Pi runs the `zenohd` router on `tcp/0.0.0.0:7447`. Default
  `CHATTER_ROUTER = tcp/192.168.68.78:7447`. The Pi IP is **DHCP and can change** —
  make it overridable (env var, as `ChatterLink` already does) and consider a
  static lease or `raspberrypi.local` mDNS so it doesn't break on a new address.
- Zenoh versions are wire-compatible: CW `eclipse-zenoh 1.6.2`, Pi `zenohd
  1.9.0`, all 1.x.

Within that session, two interaction styles:

- **Tools (pull):** Jill consciously invokes and waits for a result.
  `head/cmd` (pan/tilt/gesture), `camera/capture` → `camera/image`. These map
  naturally to Cognitive_workbench tool calls.
- **Sensors (push):** asynchronous streams that update Jill's state without her
  asking. `voice/event`, the STT text derived from `audio/in`, and any future
  pushed alarms. These do **not** become tools Jill polls — `sensor_runner`
  injects them into the chat-loop.

## 4. Sensor → chat-loop → concerns

Base level (agreed): a recognized utterance enters the chat-loop **as if it were
user text**, tagged with provenance, e.g. `[source: acoustic_sensor]`. That
alone gives conversational presence.

Next level up — **event-driven concerns.** Generalize beyond the mic: a pushed
mic utterance, a future pushed `[voltage_low]` from the PV controller, and a
camera-motion event are all the *same shape* — a typed async event with a source
tag and an activation weight. Build the ingress **once** in `sensor_runner`
(normalize N zenoh topics → tagged events → chat-loop), not as a mic-specific
path.

Mapped to the concerns layer:

- **Trigger** — sensor event (keyword, a grandchild's voice, the PV alarm).
- **Activation** — the event spikes a specific `agent_concern` instead of
  waiting for its time-based rhythm. (For PV: a pushed `[voltage_low]` drives
  activation toward 1.0 immediately, replacing the hourly poll — "responding"
  rather than "monitoring.")
- **Action** — e.g. *Grandchild-detected* fires: reflex has already pointed the
  head; Jill runs `camera/capture` → verify identity → `respond` via speaker.

The valuable, distinctive part is the concerns layer doing **relevance gating** —
deciding whether an event is worth interrupting the user for. That is what
separates this from a reactive wake-word assistant, and it is the part most
worth getting right (false-positive interruptions are how companion devices get
unplugged).

## 5. Head arbitration & the use-mode decision

`head_controller` (Pi `head_service`) is the **sole** servo owner. Two writers
want it: the Pi-local DoA reflex and Jill's deliberate `head/cmd`. Both are now
implemented; arbitration is automatic — **any `head/cmd` suspends the reflex for
a cooldown** (`head.doa.cmd_cooldown_s`), so deliberate gaze and the reflex never
fight. Jill can also hard-toggle the reflex via `chatter/head/mode {doa_follow}`.

**Use-mode decision (current default): wake-word orient, reflex off.** In testing,
the XVF3800 VAD is energy-based and fires on non-speech noise (a rolling chair, a
clack), so an always-on "turn toward any sound" reflex twitches. Two clean
stances:

- **Intentional (default):** the autonomous reflex is **off** (`doa_follow:false`,
  the boot default). The head turns only when Jill is addressed — on recognizing
  the wake word ("Jill…"), Jill sends `head/cmd {doa_deg}` using the bearing from
  the concurrent `voice/event`, and the Pi maps it to pan with its calibrated
  `head.doa` mapping. Rock-solid, never twitches, inert until addressed. Recipe:
  `cw-voice-sensor.md §5`.
- **Companion presence (opt-in):** set `doa_follow:true` for ambient "glance at
  the speaker" liveliness. The Pi reflex is a **saccade** (one glance then settle,
  going deaf while moving so servo noise — the XVF3800 sits directly under the
  servos — can't feed back) with a **persistence gate** (needs a consistent
  bearing for ~1 s). Still occasionally glances at sustained non-speech sound.

Either way the DoA→pan mapping needs a one-time per-mount calibration
(`config.json head.doa`: `front_deg`, `sign`; see `xvf3800-setup.md §6`).

## 6. Self-trigger / echo gating

The XVF3800 does hardware AEC **only if TTS plays through its output path**
(`DESIGN.md §7`). That covers the speaker→mic echo. Jill must still add
**self-voice gating**: do not STT-and-inject the bot's own TTS. Gate / mute the
mic-sensor ingestion while `audio/out` is playing (barge-in handling can relax
this later).

## 7. What each project must build

**ChatterBot (Pi side):**
- **DONE** — `xvf_audio`: the single duplex owner of the XVF3800. Captures +
  control-channel VAD/DOA → publishes `voice/event` (start/active/stop +
  `doa_deg`) and VAD-gated binary `audio/in`; and plays `audio/out` TTS through
  the device (silence ↔ TTS on one persistent stream — AEC reference + capture
  keepalive), muting `audio/in` while speaking (self-voice gating)
  (`chatterbot/services/xvf_audio.py`, `chatterbot/xvf3800.py`,
  `chatterbot/lib/audio_frame.py`). Consumer guide: `docs/cw-voice-sensor.md`;
  bring-up: `docs/xvf3800-setup.md`; playback design: `docs/audio-out-design.md`.
  Live-verified DoA/VAD/capture on the Pi (playback pending live test).
- **DONE** — DoA reflex: `head_service` consumes `chatter/voice/event` and can
  turn the head toward the talker when `doa_follow` is set — saccade (glance +
  settle, goes deaf while moving so servo noise doesn't feed back) with a
  persistence gate and an explicit-command cooldown for arbitration (§5).
  **Off by default**: the XVF VAD is energy-based and fires on non-speech noise,
  so the recommended pattern is CW-driven **wake-word orient**, not an always-on
  reflex — CW sends `head/cmd {doa_deg}` on hearing "Jill" and the Pi maps the
  bearing to pan via its calibrated `config.json` `head.doa`
  (`docs/cw-voice-sensor.md` §5, calibration in `docs/xvf3800-setup.md` §6).
- TODO — `audio/out` live test against CW's ElevenLabs `--say` (the player is
  built; needs an end-to-end check on hardware).
- TODO — Head arbitration policy refinements (§5).

**Cognitive_workbench (Jill side):**
- **DONE** — `head-move` + `camera-capture` drop-in tools (`src/tools/`), over a
  shared isolated zenoh session to the Pi (`src/utils/chatter_link.py`,
  `ChatterLink`; endpoint `CHATTER_ROUTER`, default `tcp/192.168.68.78:7447`).
  `head-move` does pan/tilt/gesture and waits for an `arrived` status;
  `camera-capture` requests a frame, filters `camera/image` by `request_id`,
  decodes the base64 JPEG, and feeds it to the model as vision input (see §8).
  Discrete tools, not a `body`-style subagent — actuation/capture are direct,
  no extra reasoning hop. Live-verified against the Pi.
- **DONE** — captured-image-as-vision wiring + Tier-2 closed-loop gaze
  (`look-at-target`) (§8).
- **DONE** — TTS → `audio/out` (output "say" path): `voice_pipeline.synthesize`
  (ElevenLabs `pcm_16000`) + `pack_audio_frame` (the CBA1 codec, inverse of the
  Pi's `unpack`) + `ChatterLink.send_audio_out`; voice-sourced replies synthesize
  off-thread (`_speak_async`) and speak iff `source == ACOUSTIC_SOURCE`. Standalone
  `voice_harness.py --say "…"`. Frame round-trip verified; **self-voice gating is
  Pi-side** (xvf_audio mutes `audio/in` while speaking) so CW keeps no mute flag.
- TODO — `xvf_audio` consumer (voice-sensor input): isolated zenoh session already
  exists in `ChatterLink`; the STT path reuses it.
- TODO — VAD-segment → STT → text injection into the chat-loop with source tags.
- TODO — generic sensor-event → concern-activation ingress (§4).
- TODO — Tier-3 vision (reference-image library) (§8).

## 8. Vision: captured images as model input

A captured frame reaches the model as real vision input, not just a display
URL. The mechanism is generic: a drop-in tool may return
`{"status","text","image":{"data_uri","label"}}`; `_dispatch_discovered_tool`
(`src/chat/tools.py`) stashes it in a single most-recent slot
(`self._pending_tool_image`), and `_run_react_loop` (`src/chat/react.py`)
injects it into the multimodal `content` array alongside any `/paste` image —
the same path user-supplied images already use. `camera-capture` is the first
user: it inlines the JPEG as a base64 data-URI (a local model server can't fetch
the `127.0.0.1` `/local` URL, so vision goes via the data-URI; the `/local` URL
stays in the text observation for canvas display).

Constraints / decisions:
- **Backend-gated**: only the OpenAI-compat route carries image content
  (`backend.supports_image_input`). The `jill-chat` world's `local` server with a
  multimodal model (gemma4-34B) qualifies; Anthropic-native and legacy-cloud
  routes do not.
- **Per-turn, single slot**: the captured frame is in view for the rest of the
  turn it was taken in, then cleared. No stale cross-turn frame — re-capture is
  cheap. Only the most recent capture is kept.
- **Cost**: ~110 KB of base64 per capture, resent each remaining iter of the
  turn. Fine for single captures; relevant before Tier 2's gaze loop.

Capability tiers:
- **Tier 0 (done)** — capture + `display` to screen.
- **Tier 1 (done)** — VQA on the current frame: "what do you see?", "am I in
  frame?", "is there a cat?".
- **Tier 2 (done)** — closed-loop gaze ("point at me", "center on the cat"):
  the `look-at-target` tool (`src/tools/look-at-target/`). It runs the whole
  acquire→center loop internally and reuses `ChatterLink` (pose read → absolute
  `head/cmd` → capture) plus its own multimodal `backend.chat` judgements — so
  it is one opaque step in the parent ReAct trace, and cheap per cycle (a small
  downscaled frame + a tight question, not the whole conversation). Details below.
- **Tier 3 (deferred)** — identity/recognition ("can you see John") against a
  small reference-image library: multi-image LLM compare, or a face-rec pipeline.

### Tier-2 gaze: control law (`look-at-target`)

The geometry/limits live in ChatterBot `docs/gaze-support.md` §1 (authoritative);
the agent mirrors them as constants: pan ∈ [10,170] (0=right, 170=left),
tilt ∈ [30,150] (30=up, 115=horizontal, 150=~45°down), neutral ≈ pan 90 tilt 113.

- **Perception → control.** Each cycle asks the vision model for the target's
  position relative to frame center as **coarse buckets** — `h_pos` ∈
  {left_a_lot, left_a_little, centered, right_a_little, right_a_lot}, `v_pos`
  likewise up/down (structured output via `response_schema`). Coarse buckets are
  what a general VLM does reliably; precise pixel/degree regression is not.
- **Buckets → bounded delta.** a_lot ≈ 18°, a_little ≈ 7°, centered = 0. Signs
  (camera `hflip/vflip` false): target left → pan **+**, target up → tilt **−**.
  Applied relative to the current pose and **clamped to the envelope** every step.
- **Acquire then center.** If the target isn't visible, sweep a fixed set of
  in-envelope waypoints (3 pan × 2 tilt, near horizontal), capturing at each,
  until it appears or the set is exhausted (`empty: couldn't find …`). Then the
  centering loop runs to a cap (≤5 iters) with a halve-on-flip anti-oscillation
  guard, stopping when both axes read `centered`.
- **Once, not continuous.** It centers a roughly-static target and returns the
  final pose + frame (attached as the model's view via the §8 image contract,
  plus a `/local` display URL). A moving subject gets best-effort, not tracking.
- **Speed/settle/limits are also Pi-side** (`gaze-support.md` §2): envelope
  clamp, a `max_deg_per_s` rate limit, and `arrived`-means-settled (kills capture
  blur). The agent clamps and bounds too (defense-in-depth) but relies on the Pi
  for settle and rate.
- **Cost.** Assessment frames are downscaled to ~640×360 (`jpeg_to_data_uri`
  `max_wh`); the final returned frame is full res. Each cycle ≈ move+settle +
  capture + one VLM call.

## 9. Topic contract (current vs to-build)

JSON unless marked **binary**; all carry `ts` (unix epoch float). Authoritative
list: `chatterbot/lib/topics.py` + `DESIGN.md §5`.

| Topic | Dir | Payload | Status |
|---|---|---|---|
| `chatter/head/cmd` | Jill→Pi | `{ts, pan?, tilt?, doa_deg?, gesture?: nod\|shake\|scan\|center, smooth?}` | **implemented** |
| `chatter/head/status` | Pi→Jill | `{ts, pan, tilt, state, mode}` (~5 Hz, `config.status_hz`) | **implemented** |
| `chatter/head/mode` | Jill→Pi | `{ts, doa_follow}` | **implemented** (drives the DoA reflex) |
| `chatter/camera/capture` | Jill→Pi | `{ts, request_id}` (width/height currently ignored) | **implemented** |
| `chatter/camera/image` | Pi→Jill | `{ts, request_id, format:"jpeg_base64", data_base64, width, height, head_pose, settled}` | **implemented** |
| `chatter/voice/event` | Pi→Jill | `{ts, vad: start\|active\|stop, doa_deg, confidence}` | **implemented (Pi)** |
| `chatter/audio/in` | Pi→Jill | **binary** `audio_frame` header + S16_LE PCM, VAD-gated | **implemented (Pi)** |
| `chatter/audio/out` | Jill→Pi | **binary** `audio_frame` header + S16_LE PCM (TTS) | declared, not built |
| `chatter/status` | Pi→Jill | `{ts, processes, ...}` | declared, not built |

### Audio & voice wire formats

- **`voice/event`** (JSON): `{ts, vad, doa_deg, confidence}`. `vad` ∈
  `start | active | stop` (start = rising edge of speech; active = ~15 Hz updates
  while speaking, each carrying a refreshed bearing; stop = after a ~0.6 s silence
  hangover). `doa_deg` is 0–359 in the **array frame**, not the room — calibrate.
  `confidence` is coarse (1.0 speaking / 0.5 hangover / 0.0 stop).
- **`audio/in`** (binary, VAD-gated): a **24-byte little-endian header then
  interleaved PCM**. Header `struct "<4sBBBBIId"` = magic `b"CBA1"`, version (1),
  format (0 = S16_LE), channels (2), reserved (0), sample_rate (16000),
  seq (uint32, monotonic — detects drops), ts (float64). So **2 ch / 16 kHz /
  S16_LE**; downmix to mono for STT. Unpack code + per-utterance reassembly:
  `cw-voice-sensor.md §3`. Source of truth: `chatterbot/lib/audio_frame.py`.
- **`audio/out`** (binary, TTS — to build): **same `audio_frame` framing**,
  **16 kHz S16_LE mono** (`channels=1`); the Pi upmixes to the device's 2 ch.
  Source is ElevenLabs `pcm_16000` (no transcode). Must play through the XVF3800
  output (the AEC reference, `DESIGN.md §7`). v1 = one payload per utterance.
  Full design + locked decisions: `docs/audio-out-design.md`.
- **`camera/image`** (JSON): `format:"jpeg_base64"`, `data_base64` (decode →
  JPEG bytes), plus `head_pose{pan,tilt}` and `settled`.

## 10. Open questions / status

**Resolved since first draft:**
- XVF3800 arrived; bring-up complete. Control is **USB vendor transfers** (not
  I2C) via `pyusb` — `xvf3800-setup.md`. Audio is 2 ch / 16 kHz / S16_LE.
- DoA reflex implemented as a **saccade**; the "jittery reflex" worry is handled
  by going deaf during motion + a persistence gate (§5).
- Reflex/deliberative arbitration: explicit `head/cmd` suspends the reflex for a
  cooldown (§5).
- Use-mode default chosen: **wake-word orient, autonomous reflex off** (§5).

**Still open:**
- DoA→pan calibration (`head.doa` `front_deg`/`sign`) — one-time per mount; do it
  when wiring the CW wake-word path (`xvf3800-setup.md §6`).
- `audio/out` channel count / exact framing — finalize with the `audio_out`
  service (must route through the XVF3800 for AEC).
- Whether STT runs continuously over `audio/in` or only between `voice/event`
  start/stop boundaries.
- Optional acoustic insulation between the servos and the XVF3800 (further
  reduces servo-noise DoA corruption; the saccade already handles it in software).
