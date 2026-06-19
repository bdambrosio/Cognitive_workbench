# CW Voice Sensor — implementation plan

Status: **design note / build plan.** Plans the Cognitive_workbench (CW / "Jill")
side consumer for the ChatterBot voice path. The Pi `mic_driver` is implemented
and publishing (`chatter/voice/event` + binary `chatter/audio/in`); this note
plans how CW turns that into recognized text, a wake-word head-orient, and a
conversational turn — and how we debug it **standalone before** wiring it to Jill.

Read alongside:
- `docs/cw-voice-sensor.md` — the Pi-side consumer guide (wire formats, segmenter,
  wake-orient recipe). This note *binds* it to CW's actual architecture and
  records the decisions taken in planning.
- `docs/audio-out-design.md` — the Pi-side **output** ("say") contract: the
  merged `xvf_audio` duplex service, `chatter/audio/out` framing, AEC. §10 here
  is the CW side of it.
- `docs/jill-integration.md` — §3 tools-vs-sensors, §4 sensor→chat-loop→concerns,
  §5 head arbitration, §6 self-voice gating.
- `docs/sensor_spec.md` — the existing (poll-based) sensor subsystem. See §2 for
  why the voice sensor does **not** use it as-is.

## 1. Decisions taken in planning

These were open; they are now fixed for v1:

- **D1 — Event-driven, not a polling sensor.** The voice consumer is a background
  component on the ChatterLink session, modeled on `affect/head_aliveness.py`
  (opt-in launcher flag, `attach_session`, uses `get_link()`), **not** a
  `SensorRunner` poll loop. See §2.
- **D2 — STT via the OpenAI API.** No local STT service for v1. We POST each
  utterance to OpenAI's transcription endpoint using the existing
  `OPENAI_API_KEY` (already read in `src/utils/OpenAIClient.py`). This removes a
  whole service to build/operate; revisit a local model only if latency, cost, or
  offline operation demand it. See §4.
- **D3 — Utterance-batch, not streaming.** The Pi VAD-gates and emits clean
  `start`/`stop` delimiters, so we buffer one utterance and transcribe it whole.
  (Resolves the "continuous vs bounded" open question in `cw-voice-sensor.md §7`
  in favor of bounded.)
- **D4 — Wake word is a literal string match, as a temporary, explicit exception
  to the project no-keyword-matching rule.** See §5 — this is flagged for
  re-examination once the pipeline is debugged.
- **D5 — A recognized utterance enters the chat loop as user-like text**, tagged
  with provenance (`source: acoustic_sensor`). No concern-activation / relevance
  gating in v1 — that valuable-but-harder layer (`jill-integration.md §4`) is
  deferred. See §6. **(Partially superseded:** live use showed "respond to
  everything" is unusable, so an *address* gate — wake-word-or-semantic, every
  turn — was added; §7, Issue B/C. The deeper concern/interrupt-relevance layer
  is still deferred.)
- **D6 — Build a standalone test harness first** (§3) and debug the whole
  capture→DoA→STT→wake path against the live Pi **before** touching the chat loop.

## 2. Why not the existing sensor subsystem

`docs/sensor_spec.md` / `src/sensor_runner.py` describe a **scheduled poll loop**:
it sleeps `schedule_seconds`, calls `run(context)`, and pushes a
*"sensor `<name>` report [disposition]"*-prefixed blob to `sense_data`. Voice is
the opposite shape:

- It is **push-driven** — utterances arrive asynchronously from Zenoh subscriber
  callbacks, not on a timer.
- Its output should read as **conversational user input**, not a sensor report.

The right precedent is `affect/head_aliveness.py`: launcher-gated, off by default,
owns a background thread, subscribes via `attach_session`, and drives the Pi
through the shared `ChatterLink` singleton. The voice consumer mirrors that
exactly.

We still place it under `src/sensors/voice/` with a `SKILL.md` so it is
discoverable/configurable in the same place as other sensors, but it is **not**
driven by `SensorRunner`. (If that feels inconsistent, the alternative is to keep
it purely a launcher-flagged component like `head_aliveness` and drop the
`src/sensors/` entry; decide during build — it's cosmetic.)

## 3. Standalone test harness (build this first — D6)

A single-file CLI, `tools/voice_harness.py` (or `src/chat/voice_harness.py`),
runnable without launching Jill, the chat loop, or any infospace world. It opens
its **own** isolated Zenoh session to the Pi (same endpoint logic as
`ChatterLink`, env `CHATTER_ROUTER`, multicast off) and prints a live trace:

```
[voice] START  doa=139°  conf=1.0
[audio] +12 frames (2ch/16k, seq ok)
[voice] STOP   doa=140°  dur=1.8s  bytes=57600
[stt ]  "jill what's the weather"        (820 ms)
[wake]  MATCHED "jill" -> head/cmd doa=139°   [--orient]
```

What it exercises, in isolation:

- `voice/event` subscription and the latest-`doa_deg` cache.
- `audio/in` binary unpack (`CBA1` header) + the `VoiceSegmenter` (buffer between
  `start`/`stop`, downmix 2ch→mono).
- The OpenAI STT call on the assembled utterance, printing transcript + latency.
- Wake-word match decision (print only).
- Optional `--orient`: actually send `head/cmd {doa_deg}` so we can validate the
  acoustic head-turn (requires the one-time Pi DoA calibration, §5 below).

Flags: `--router <ep>`, `--orient` (default off — print-only), `--save-wav <dir>`
(dump each utterance for offline STT debugging), `--wake <phrase>`.

This harness is where DoA calibration, VAD behavior, downmix correctness, and STT
accuracy get debugged. The shared building blocks it exercises — the segmenter,
the unpack, the STT call, the wake check — are written as small importable
functions so the integrated sensor (§7) reuses them verbatim rather than
re-implementing (per the reuse rule).

## 4. STT via OpenAI (D2)

- **Client/key:** reuse the existing pattern — `from openai import OpenAI`,
  `OPENAI_API_KEY` from env (see `src/utils/OpenAIClient.py`). A thin helper
  `transcribe(pcm_mono_16k: bytes) -> {text, ...}` wraps
  `client.audio.transcriptions.create(...)`. The OpenAI SDK wants a file-like
  object, so wrap the raw PCM as a WAV in memory (16 kHz, mono, S16_LE) before the
  call — a few lines with `wave` + `io.BytesIO`. Model: start with the current
  recommended transcription model; make it configurable.
- **Granularity (D3):** one call per utterance (`start`→`stop` segment). Typical
  utterance ≤ a few seconds, so a per-utterance round-trip is fine; no streaming.
- **Failure handling:** log and drop on API error (no silent except). A failed
  transcription simply produces no turn; the next utterance is independent.
- **Open sub-items:** cost/latency under real use; whether to send `language=en`
  hint; whether to keep word timestamps (not needed for v1).

## 5. Wake-word orient (D4) — temporary keyword exception

**Intentional default: head turns only when addressed.** The Pi autonomous DoA
reflex stays **off** (`doa_follow:false`); CW drives the orient on hearing the
wake word, reusing what `mic_driver` already publishes (no new Pi work):

1. Cache `doa_deg` from each `voice/event` `start` (latest talker bearing).
2. After STT, test the transcript for the configured wake phrase (default
   `"Jill"`, case-insensitive, configurable).
3. On match, send **one** `head/cmd {doa_deg}` with the cached bearing — the Pi
   applies its calibrated DoA→pan mapping (`config.json head.doa`). CW sends
   `doa_deg`, never `pan`; calibration lives only on the Pi
   (`cw-voice-sensor.md §5`).
4. Arbitration is automatic — any `head/cmd` suspends the Pi reflex for a cooldown
   (`jill-integration.md §5`).

> **Rule exception (revisit).** The project rule forbids keyword/string matching
> for classification/routing/intent. A literal `"jill" in transcript.lower()`
> wake check violates it — it misses ASR homophones ("Gill", "Jules") and can
> false-fire ("jillion"). We are **taking this exception deliberately for v1** to
> get the pipeline debugged with the simplest possible trigger. **TODO: once the
> capture→STT→orient path is debugged, replace the literal match** with either an
> acoustic wake-word model (e.g. openWakeWord, meaning-based at the audio level)
> or a cheap semantic address-check ("is this utterance addressed to Jill?") on
> the transcript. Tracked as Issue B in this note's §9.

**Prerequisite:** the per-mount DoA→pan calibration (`front_deg`/`sign`).

> **Finding (2026-06-16 live test):** the Pi *accepts* `head/cmd {doa_deg}` —
> `head/status` cycles `moving→arrived` — but **pan never changes** for any
> bearing (37/180/300/90 all leave pan parked). The Pi's `doa_deg→pan` mapping is
> a no-op (uncalibrated *and* not applied). Explicit `head/cmd {pan}` works
> flawlessly. So for v1 we adopt **option 2: CW maps `doa_deg→pan` and sends
> `pan`** (`voice_pipeline.doa_to_pan`, exercised by the harness `--front-deg` /
> `--sign` flags). This puts calibration on the CW side, against the "one place"
> preference — revisit by fixing the Pi mapping and flipping the harness/component
> back to `--send-doa` once it works. Tracked as Issue F.

## 6. Ingestion into the chat loop (D5)

When CW is running, a recognized utterance is published to
`cognitive/{character}/sense_data` as a normal content payload with
`source: 'acoustic_sensor'` and `text: <transcript>`. The existing
`ZenohMixin._on_sense_data` (`src/chat/zenoh_io.py`) already turns any
`{source, text}` into an inbox `{'kind':'user', 'source', 'text'}` item — so a
voice utterance becomes a user-like turn **without any change to the ingestion
path**, carrying its provenance tag.

Note we deliberately **bypass** `SensorRunner._push_to_agent`, whose
*"sensor report [disposition]"* prefixing is wrong for conversational speech. The
voice component publishes its own clean payload directly on the chat loop's Zenoh
session.

Deferred (not v1): the concern-activation / relevance-gating layer that decides
*whether an utterance is worth interrupting Jill for* (`jill-integration.md §4`).
v1 treats every recognized utterance as a user turn. This is the part most worth
getting right later (false-positive interruptions are how companion devices get
unplugged), but it is a separate design problem.

## 7. Integrated component (SHIPPED)

`src/chat/voice_sensor.py`, modelled on `HeadAliveness`:

- Constructed with an `enabled` flag from the launcher `--voice` arg (off by
  default, mirroring `--head-aliveness`); no-op if disabled, and idle (logs)
  if the Pi link can't be opened.
- `attach_session(session)`: declares the `sense_data` publisher on the chat
  loop's session, attaches the Pi `voice/event` + `audio/in` subscribers via
  `get_link().attach_voice(...)`, and runs STT + wake check in a worker so the
  Zenoh callbacks stay cheap.
- Reuses the harness's segmenter / STT / wake / DoA→pan functions **verbatim**
  from `utils.voice_pipeline` (one implementation, two callers).
- **Address gate, every turn** (Issue B/C): a recognized utterance becomes a
  turn only if it is addressed to Jill — a free literal wake match first, and
  only on a miss the semantic `is_addressed` check. This drops ambient cross-talk
  and energy-VAD/STT noise hallucinations ("Thank you.", "you") while keeping STT
  homophones ("Gill") and naturally phrased address. Unaddressed → no turn, no
  orient.
- On an addressed utterance: publishes the `ACOUSTIC_SOURCE` turn (§6) and
  orients toward the talker via `doa_to_pan` → `send_head_cmd` (CW-side mapping,
  Issue F), harness-calibrated `front_deg=90 / sign=+1`. Wake phrase: launcher
  `--wake` > `CW_WAKE_WORD` > `Jill`; other knobs `CW_VOICE_FRONT_DEG`,
  `CW_VOICE_SIGN`, `CW_VOICE_ORIENT`, `CW_ADDRESS_MODEL` (default `gpt-4o-mini`).

**ChatterLink additions (done, small, additive):**
- `attach_voice(on_event, on_audio)` / `detach_voice()` — declare/undeclare the
  `voice/event` + `audio/in` subscribers on the shared Pi session.
- `send_audio_out(pcm_mono)` — the §10 output publisher (built alongside).
- Note: the `doa_deg` field on `send_head_cmd` was **not** needed — Issue F sends
  `pan` from the CW-side `doa_to_pan` mapping instead.

## 8. Self-voice gating (resolved — Pi-side)

Originally deferred ("no TTS path yet"). Now that the output path is specified
(§10, `docs/audio-out-design.md`), gating is **handled on the Pi**, not in CW:
the merged `xvf_audio` service owns the XVF3800 full-duplex and **suppresses
`audio/in` while it is playing a TTS utterance** (`tts_playing`), on top of the
XVF3800's hardware AEC (playback routes through the array, so it is the AEC
reference). So CW receives no mic audio during Jill's own speech and there is
**no CW-side mute flag to maintain in v1** — the silence on `audio/in` is the
gate. CW's only responsibility is the obvious one: don't treat its own TTS as
user input (moot until the input sensor component exists; §7). If we later want
an explicit signal, the Pi can expose `tts_playing` in `chatter/status` — not
needed for v1.

## 9. Open issues / tracking

- **Issue A — STT backend (resolved for v1):** OpenAI API (D2). Revisit local STT
  only if latency/cost/offline pressure appears.
- **Issue B — wake-word keyword exception (mostly closed):** the literal match
  is now only the *free fast path*; on a miss it falls back to the semantic
  `is_addressed` LLM check (no-keyword-compliant), which also catches STT
  homophones. So the keyword is an optimization, not the decision — the decision
  is semantic. Remaining: a true acoustic wake-word model (openWakeWord) would
  cut the per-noise-blip LLM calls; revisit if cost/latency bites.
- **Issue C — address gating (coarse layer DONE; deep layer deferred):** every
  turn is now gated on "is this addressed to Jill?" (§7), which kills ambient
  speech and noise. The *deeper* concern/relevance layer — "is this worth
  interrupting for / spiking a concern" — is still future work (§6).
- **Issue D — barge-in / turn arbitration:** an utterance can arrive while Jill is
  mid-generation. The inbox is serial; v1 queues it (simplest). Interrupt/drop
  semantics are future work.
- **Issue E — self-voice gating (resolved, Pi-side):** the `xvf_audio` service
  mutes `audio/in` during playback + hardware AEC (§8). No CW-side flag in v1.
- **Issue F — DoA→pan mapping (Pi no-op; CW-side fallback adopted):** live test
  showed `head/cmd {doa_deg}` does not move the head (§5 Finding). v1 maps
  `doa_deg→pan` on the CW side (`voice_pipeline.doa_to_pan`) and sends `pan`;
  calibrate `front_deg`/`sign` live via the harness. Revisit: fix the Pi mapping,
  then switch back to sending `doa_deg` (`--send-doa`) so calibration lives in one
  place on the Pi.
- **Issue G — single dominant talker:** the XVF3800 reports one talker; no
  diarization on this path. Accepted limitation.
- **Issue H — Pi IP is DHCP:** `CHATTER_ROUTER` overridable (already so in
  `ChatterLink`); consider a static lease / mDNS so it doesn't break.

## 10. Voice output — the "say" path (CW side)

The output half: a recognized voice turn's reply gets spoken. Pi-side contract is
locked in `docs/audio-out-design.md` (ElevenLabs `pcm_16000`, **one mono
`audio_frame` per whole utterance** on `chatter/audio/out`, the Pi upmixes to
2 ch and plays through the XVF3800). It is the input path run backwards, and —
like STT (D2) and the DoA→pan fallback (Issue F) — **the cloud call lives on the
CW side**; the Pi stays a dumb player.

**"How does Jill remember to speak it?" — provenance routing, not a decision.**
A voice utterance already enters the chat loop tagged `source: 'acoustic_sensor'`
(§6, D5; the canonical tag is `voice_pipeline.ACOUSTIC_SOURCE`). That tag is
carried all the way to reply publication, so the rule is simply: **speak the
reply iff the turn's `source` is `ACOUSTIC_SOURCE` and it isn't an autonomous
fire.** This is structural-tag routing on a field we set ourselves, not content
classification — it does not engage the no-keyword rule. Text-chat turns route to
the UI only, exactly as before.

**Pieces (shipped, harness-testable; live loop pending both ends):**
- `voice_pipeline.synthesize(text) -> pcm_bytes` — ElevenLabs `pcm_16000`, the
  output-side twin of `transcribe()` (lazy import, env key, log+None on error).
  Env: `ELEVENLABS_API_KEY` (required), `CW_TTS_VOICE_ID` (required — ElevenLabs
  has no default voice), `CW_TTS_MODEL` (default `eleven_flash_v2_5`).
- `voice_pipeline.pack_audio_frame(...)` — the inverse of `unpack_audio_frame`;
  one canonical CBA1 codec used by both directions (reuse rule).
- `ChatterLink.send_audio_out(pcm_mono)` — frames one whole utterance (mono,
  16 kHz, seq counter) and publishes to `chatter/audio/out`. Fire-and-forget; no
  completion signal in v1.
- `ChatLoop._publish_say(text, speak=...)` → `_speak_async` — on a voice-sourced
  reply, synthesize + ship **off-thread** so the turn never blocks on the
  ElevenLabs round-trip.
- `voice_harness.py --say "TEXT"` — standalone synth→frame→publish (no mic), the
  output analogue of the §3 listen harness; pair with `--save-wav` to dump the
  synth locally.

**Not in v1 (deferred):** streaming / lower latency-to-first-audio, barge-in
(`audio/out/ctl` begin|end|cancel, `docs/audio-out-design.md` v2), letting Jill
choose her output channel, and an utterance-length cap. v1 is whole-utterance,
measured turn-taking.

**Blocked-on (live end-to-end):** (a) the Pi `xvf_audio` service that consumes
`audio/out` ("declared, not built"); (b) the input voice-sensor component (§7)
that produces `acoustic_sensor` turns. Until both land, the say path is
exercisable only via `--say` (and the frame codec is unit-checked).

## 11. Build order

1. **Harness** (§3) — own Zenoh session, print `voice/event` + segmented audio.
   Verify: speak at the Pi, see START/STOP + DoA + frame counts.
2. **STT in harness** (§4) — OpenAI transcription on each utterance. Verify:
   spoken phrase prints as correct text with latency.
3. **Wake + orient in harness** (§5, `--orient`) — do the Pi DoA calibration here.
   Verify: "Jill…" turns the head toward the speaker; other speech does not.
4. **Say path** (§10) — `synthesize` + `pack_audio_frame` + `send_audio_out` +
   `_publish_say(speak=...)`, plus `voice_harness --say`. Verify now: frame codec
   round-trips, `--say --save-wav` produces correct audio. Verify live once the
   Pi `xvf_audio` service exists: `--say` makes the bot speak.
5. **Integrated component** (§7, SHIPPED) + ChatterLink additions + launcher
   `--voice`. Verify live (`--voice`, Pi up): a spoken utterance appears as a
   user turn; "Jill…" orients the head; Jill's reply is spoken back through
   `audio/out` — the full round-trip.
6. **Defer:** concern gating (C), barge-in / streaming (D), wake-word
   de-keywording (B). Self-voice gating (E) is resolved Pi-side (§8).
</content>
</invoke>
