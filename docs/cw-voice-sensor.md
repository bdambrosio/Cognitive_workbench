# Building a Cognitive_workbench voice sensor on `chatter/voice/event`

Status: **implementation guide** for the Cognitive_workbench (CW / "Jill") side.
The Pi `xvf_audio` is implemented and publishing; this doc tells the CW project
how to consume it. Read alongside `docs/jill-integration.md` (§3 tools-vs-sensors,
§4 sensor→chat-loop→concerns, §6 self-voice gating) — this is the concrete
binding of the "voice sensor" promised there.

> **Where this fits:** `voice/event` is a **push sensor**, not a tool
> (jill-integration §3). It feeds `sensor_runner`, which injects events into the
> chat-loop / concerns layer. Jill never *polls* it.

## 1. What the Pi publishes

The XVF3800 does VAD + DoA in firmware; `xvf_audio` (Pi) gates on it and
publishes two streams over the same zenoh router CW already talks to:

| Topic | Payload | When |
|---|---|---|
| `chatter/voice/event` | JSON `{ts, vad, doa_deg, confidence}` | speech edges + updates |
| `chatter/audio/in` | **binary** `audio_frame` header + S16_LE PCM | VAD-gated, ~50 fps while speaking |

A typical utterance on `chatter/voice/event`:

```jsonc
{"ts": 1750..., "vad": "start",  "doa_deg": 139, "confidence": 1.0}
{"ts": 1750..., "vad": "active", "doa_deg": 141, "confidence": 1.0}   // ~15 Hz
{"ts": 1750..., "vad": "active", "doa_deg": 140, "confidence": 1.0}
{"ts": 1750..., "vad": "stop",   "doa_deg": 140, "confidence": 0.0}
```

**Field semantics**

- `vad` — `"start"` (rising edge of speech), `"active"` (periodic update while
  speaking, default ~15 Hz, carries a refreshed `doa_deg` so a moving talker can
  be tracked), `"stop"` (emitted after `vad_hangover_s` of silence, default
  0.6 s — so brief pauses don't chop an utterance).
- `doa_deg` — dominant talker azimuth **0–359° in the device frame**. The 0°
  heading depends on how the array is physically mounted; treat it as a relative
  bearing and calibrate before mapping to anything absolute (see §6).
- `confidence` — coarse for now: `1.0` while the firmware VAD is high, `0.5`
  during the hangover tail, `0.0` on `stop`. (Will become graded if `xvf_audio`
  starts reading per-beam speech energy.)

**`start`/`stop` are your utterance delimiters.** Everything on `audio/in`
between a `start` and its `stop` is one utterance — that's the natural unit to
hand to STT.

## 2. Connecting (reuse the isolated ChatterLink session)

Per jill-integration §3, CW already opens a **dedicated, isolated** zenoh session
to the Pi router in `src/utils/chatter_link.py` (`ChatterLink`, endpoint
`CHATTER_ROUTER`, default `tcp/192.168.68.78:7447`, multicast **off**). The voice
sensor must reuse that session — **do not** add the Pi endpoint to CW's localhost
gossip mesh.

Subscribing for the discrete JSON events (drop into `sensor_runner`):

```python
import json

def _on_voice_event(sample):
    evt = json.loads(sample.payload.to_string())   # {ts, vad, doa_deg, confidence}
    sensor_runner.ingest_voice_event(evt)           # -> chat-loop / concerns

# `session` is ChatterLink's existing isolated zenoh session
sub = session.declare_subscriber("chatter/voice/event", _on_voice_event)
```

That alone gives Jill voice **presence** (someone is speaking, and roughly where)
without any audio decoding — enough to drive the gaze reflex acknowledgement and
to spike a concern (§4).

## 3. Consuming the audio for STT

`chatter/audio/in` is a **raw binary** zenoh payload (not base64-JSON): a 24-byte
little-endian header then interleaved PCM. Reproduce the unpack from this spec
(authoritative source: `chatterbot/lib/audio_frame.py`):

```python
import struct

_HDR = "<4sBBBBIId"            # magic, ver, fmt, channels, _rsvd, rate, seq, ts
_HDR_SIZE = struct.calcsize(_HDR)   # 24

def unpack_audio_frame(payload: bytes):
    magic, ver, fmt, ch, _r, rate, seq, ts = struct.unpack(_HDR, payload[:_HDR_SIZE])
    assert magic == b"CBA1", magic
    return {"channels": ch, "sample_rate": rate, "seq": seq, "ts": ts}, payload[_HDR_SIZE:]
```

- **Format:** `fmt == 0` is S16_LE. Default stream is **2 ch / 16 kHz**. For STT,
  downmix to mono — average the two int16 channels (cheap; the Pi deliberately
  does no audio processing):

  ```python
  import numpy as np
  pcm = np.frombuffer(pcm_bytes, dtype="<i2").reshape(-1, ch)
  mono = pcm.mean(axis=1).astype("<i2")
  ```

- **Drop detection:** `seq` is monotonic per stream (wraps at 2³²). A gap means
  frames were dropped — log it; STT can usually tolerate small gaps.

- **Reassembly:** buffer frames between `voice/event` `start` and `stop`, then
  run VAD-segment → STT on the concatenated utterance. The Pi already VAD-gates,
  so on `audio/in` you mostly receive speech; the `start`/`stop` events give you
  clean boundaries without re-running VAD.

```python
class VoiceSegmenter:
    def __init__(self): self._buf = []; self._active = False
    def on_event(self, evt):
        if evt["vad"] == "start": self._buf, self._active = [], True
        elif evt["vad"] == "stop" and self._active:
            self._active = False
            return b"".join(self._buf)      # hand to STT
    def on_audio(self, payload):
        if self._active:
            _, pcm = unpack_audio_frame(payload)
            self._buf.append(pcm)
```

## 4. From event to concern activation

This is the valuable part (jill-integration §4). A `voice/event start` is a typed
async event with a source tag — the *same shape* as a future `[voltage_low]` PV
alarm or a camera-motion event. Build the ingress once:

- **Base level:** a recognized utterance (after STT) enters the chat-loop **as if
  it were user text**, tagged `[source: acoustic_sensor]`, optionally annotated
  with bearing (`doa_deg`) and timing.
- **Event-driven concern:** the `start` event spikes an `agent_concern`
  activation toward 1.0 immediately (responding, not polling). The concerns
  layer does **relevance gating** — deciding whether this utterance is worth
  interrupting the user for. That gating is what separates this from a reactive
  wake-word assistant; it is the part most worth getting right (false-positive
  interruptions are how companion devices get unplugged).

The DoA→pan **orienting** is **not** a CW action — it's a Pi-local reflex
(jill-integration §2). CW may *refine* the pose afterward (center a detected
face) but should not try to chase `doa_deg` itself over the network.

## 5. Wake-word orient — turning toward the speaker

**Head orientation toward a talker is CW-driven on a wake word, not an autonomous
Pi reflex.** The Pi *has* a DoA reflex (`head_service`, `doa_follow`) that can
glance at whoever speaks, but it is **off by default** — the XVF VAD is
energy-based and fires on non-speech noise (chair squeaks, claps), so an
always-on reflex twitches. Driving the turn from CW, gated on recognizing
"Jill…", means the head only orients when actually addressed.

The mechanism reuses what xvf_audio already publishes — no new Pi work:

1. Track the latest talker bearing from `voice/event` (you're already
   subscribed, §2). Keep the `doa_deg` from the current utterance's `start`.
2. When STT (the same audio you're consuming in §3) recognizes the wake word,
   send **one** `head/cmd` carrying that bearing:

```python
latest_doa = {"deg": None}

def on_voice_event(evt):              # from §2
    if evt["vad"] == "start":
        latest_doa["deg"] = evt["doa_deg"]
    # ... existing concern-activation handling ...

def on_wake_word():                   # called by your STT when it hears "Jill"
    if latest_doa["deg"] is not None:
        session.put("chatter/head/cmd", json.dumps({
            "ts": time.time(),
            "doa_deg": latest_doa["deg"],   # Pi maps bearing -> pan itself
        }))
```

Key points:

- **Send `doa_deg`, not `pan`.** `head/cmd` accepts `doa_deg` and the Pi applies
  its own calibrated DoA→pan mapping (`config.json` `head.doa`). So the
  calibration lives in exactly one place — CW never needs to know `front_deg`,
  `sign`, or the pan envelope. (You *may* send an explicit `pan` instead when you
  already have an absolute angle, e.g. from a vision-centering loop.)
- **Arbitration is automatic.** Any `head/cmd` suspends the Pi reflex for a
  cooldown (jill-integration §5), so the deliberate orient and the reflex never
  fight — even if you later enable the reflex.
- **Refine with vision if you want.** After the coarse acoustic orient, the
  Tier-2 `look-at-target` gaze loop (§8) can center the speaker's face precisely.
- **Enabling the autonomous reflex (optional).** If you want ambient
  "glance toward sound" liveliness, set `chatter/head/mode {doa_follow:true}`.
  It is now persistence-gated (must hear a consistent bearing for ~1 s), so it
  ignores transients — but it will still occasionally glance at sustained
  non-speech sound. Leave it off for a pure intentional feel.

## 6. Self-voice gating (do not skip)

The XVF3800 does hardware AEC **only if TTS plays through its output path**
(DESIGN.md §7), which suppresses speaker→mic echo. CW must still avoid
STT-ing and injecting **the bot's own voice**: gate/mute voice-sensor ingestion
while CW is playing `chatter/audio/out`. Simplest policy: while a TTS playback is
in flight (plus a short tail), drop `voice/event`/`audio/in`. Barge-in handling
can relax this later. See jill-integration §6.

## 7. Gotchas / open items

- **DoA frame:** `doa_deg` is relative to the array's mount, not the room.
  Calibrate the offset (and any flip) before using it for absolute pointing.
- **Confidence is coarse** (binary VAD today). Don't threshold finely on it yet.
- **Gating mode:** the Pi defaults to VAD-gated audio (`config.audio.gated`). If
  CW ever wants continuous audio (e.g. its own VAD/diarization), that's a Pi
  config flip — coordinate, don't assume.
- **One utterance at a time:** the XVF3800 reports a single dominant talker;
  there is no multi-speaker separation on this path.

## 8. CW build checklist

- [ ] Subscribe to `chatter/voice/event` on the existing isolated ChatterLink
      session; normalize to a tagged async event in `sensor_runner`.
- [ ] `VoiceSegmenter`: buffer `chatter/audio/in` between `start`/`stop`.
- [ ] Downmix 2ch→mono, run STT on each segment.
- [ ] Inject recognized text into the chat-loop as `[source: acoustic_sensor]`.
- [ ] Wire the generic sensor-event → concern-activation ingress (§4).
- [ ] Wake-word orient: on "Jill", send `head/cmd {doa_deg}` from the latest
      voice event (§5).
- [ ] Self-voice gate against `audio/out` playback (§6).
