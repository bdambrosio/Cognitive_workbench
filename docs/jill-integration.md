# Jill ↔ ChatterBot Integration

Status: **design note.** Records how the Cognitive_workbench agent ("Jill")
drives and senses through the ChatterBot head. Written to be shared with the
Cognitive_workbench project as well as kept here.

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

## 5. Head arbitration (reflex vs deliberative)

Even with the desktop app gone, two writers want the servos. `head_controller`
remains the **sole** servo owner and arbitrates via `chatter/head/mode`:

- `doa_follow: true` → the Pi reflex idles the head toward whoever speaks.
- When Jill issues a deliberate `head/cmd` (centering a face, a scan, a gesture),
  it should **suspend the reflex** so they don't fight — Jill sets
  `doa_follow:false` for the duration, or the controller treats an explicit
  `head/cmd` as a temporary manual override that re-enables the reflex after an
  idle timeout. (Exact policy is an open item; `head_service.on_mode` already
  caches `doa_follow` but nothing reads it yet — see §7.)

## 6. Self-trigger / echo gating

The XVF3800 does hardware AEC **only if TTS plays through its output path**
(`DESIGN.md §7`). That covers the speaker→mic echo. Jill must still add
**self-voice gating**: do not STT-and-inject the bot's own TTS. Gate / mute the
mic-sensor ingestion while `audio/out` is playing (barge-in handling can relax
this later).

## 7. What each project must build

**ChatterBot (Pi side) — not yet implemented:**
- `mic_driver`: XVF3800 capture + control-channel VAD/DOA → publish
  `voice/event` and VAD-gated `audio/in`. (Topics declared in
  `chatterbot/lib/topics.py`; no implementation yet.)
- DOA reflex: make `doa_follow` actually drive the head. Today
  `head_service.on_mode` caches the flag but no code consumes it.
- `audio_out`: play `audio/out` PCM through the XVF3800.
- Head arbitration policy (§5).

**Cognitive_workbench (Jill side):**
- `sensor_runner`: dedicated isolated zenoh session to the Pi router (§3).
- VAD-segment → STT → text injection into the chat-loop with source tags.
- Generic sensor-event → concern-activation ingress (§4).
- Tool wrappers for `head/cmd` and `camera/capture`/`camera/image` (decode
  base64 JPEG; filter `camera/image` by `request_id` since it is broadcast).
- TTS → `audio/out`, with self-voice gating (§6).

## 8. Topic contract (current vs to-build)

JSON unless marked **binary**; all carry `ts` (unix epoch float). Authoritative
list: `chatterbot/lib/topics.py` + `DESIGN.md §5`.

| Topic | Dir | Payload | Status |
|---|---|---|---|
| `chatter/head/cmd` | Jill→Pi | `{ts, pan?, tilt?, gesture?: nod\|shake\|scan\|center, smooth?}` | **implemented** |
| `chatter/head/status` | Pi→Jill | `{ts, pan, tilt, state, mode}` (~5 Hz, `config.status_hz`) | **implemented** |
| `chatter/head/mode` | Jill→Pi | `{ts, doa_follow}` | subscribed; reflex not wired |
| `chatter/camera/capture` | Jill→Pi | `{ts, request_id}` (width/height currently ignored) | **implemented** |
| `chatter/camera/image` | Pi→Jill | `{ts, request_id, format:"jpeg_base64", data_base64, width, height, head_pose, settled}` | **implemented** |
| `chatter/voice/event` | Pi→Jill | `{ts, vad: start\|stop, doa_deg, confidence}` | declared, not built |
| `chatter/audio/in` | Pi→Jill | **binary** PCM + `{seq, ts}`, VAD-gated | declared, not built |
| `chatter/audio/out` | Jill→Pi | **binary** PCM + `{seq, ts}` (TTS) | declared, not built |
| `chatter/status` | Pi→Jill | `{ts, processes, ...}` | declared, not built |

## 9. Open questions

- DOA-degrees → pan-angle mapping + smoothing (avoid jittery reflex) — Pi side.
- Reflex/deliberative arbitration policy: explicit-cmd-override vs mode toggle
  vs idle-timeout re-enable (§5).
- Audio transport: VAD-gated segments (default) vs continuous; sample
  rate/format (lean 16 kHz mono 16-bit) — `DESIGN.md §10`.
- XVF3800 control transport (USB `xvf_host` vs I2C) — determines how DOA/VAD are
  read. Primary directional mic expected ~the Monday after 2026-06-13.
- Whether STT runs continuously over `audio/in` or only on `voice/event`
  start/stop boundaries.
