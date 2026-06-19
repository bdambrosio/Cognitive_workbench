# ChatterBot audio-out ("say") design

Status: **Pi player built + bench-verified; ElevenLabs live test pending.** A
tone published to `audio/out` plays clear through the bot speaker. How the bot
accepts audio to speak. Read alongside `DESIGN.md §7` (AEC),
`jill-integration.md` (the CW binding), and `xvf3800-setup.md` (device facts).
Shared with Cognitive_workbench (CW's ElevenLabs "say" path is built).

> **Output level:** the XVF3800 boots its `PCM` playback at ~−23 dB (very soft).
> `xvf_audio` sets it to `audio.playback_volume_pct` (default 100%) on startup
> via `amixer` (no sudo). Volatile across reboots — hence set every start.

## Locked decisions

1. **TTS = ElevenLabs, `pcm_16000`, no transcoding.** CW requests raw 16 kHz
   16-bit mono PCM and frames it directly — zero codec/resample work anywhere.
2. **Whole-utterance v1** (not streaming). Synthesize the full utterance, ship
   it, play it. Streaming + barge-in are v2.
3. **Merge `mic_driver` + `audio_out` → one duplex `xvf_audio` service** that
   owns the XVF3800 entirely.
4. **AEC-clean (option A):** play through the XVF3800 output at 16 kHz so its
   hardware AEC has the loudspeaker reference. Voice-grade fidelity by design.

## Why 16 kHz everywhere

The XVF3800 USB **playback** endpoint is fixed **S16_LE / 2 ch / 16000 Hz**
(measured; same as capture). Its hardware AEC requires capture and playback to
share that clock. So the whole TTS path is 16 kHz — which is exactly why
ElevenLabs `pcm_16000` is a drop-in: the service's output rate already matches
the device. Fidelity ceiling is 16 kHz wideband (good for a voice, not hi-fi).

## Topic contract

| Topic | Dir | Payload | Notes |
|---|---|---|---|
| `chatter/audio/out` | CW→Pi | **binary** `audio_frame` header + S16_LE PCM | v1: **one payload = one whole utterance** |

- **v1 framing:** one `audio_frame` per utterance. Header (see
  `lib/audio_frame.py`): `format=S16_LE(0)`, `sample_rate=16000`, **`channels=1`
  (mono)**, `seq` = utterance counter, `ts` = synth time. PCM body = the entire
  ElevenLabs `pcm_16000` byte stream for that utterance. A few seconds of speech
  is ~32 KB/s mono → a 5 s utterance ≈ 160 KB, comfortably one zenoh payload.
- The Pi **upmixes mono→2 ch** (duplicate) for the device — trivial, not
  "processing." CW stays mono.
- **No control topic needed in v1**: one message = one utterance, so begin/end
  are implicit. v2 adds `chatter/audio/out/ctl` `{ts, utt_id, event:
  begin|end|cancel}` for streamed frames + barge-in (`cancel` → flush).

## The `xvf_audio` service (merged duplex owner)

One process, one zenoh session, sole owner of the XVF3800. Three concerns:

- **Playback (continuous):** a single persistent ALSA playback stream — e.g.
  `aplay -D plughw:CARD=Array -f S16_LE -c2 -r16000` reading from a pipe — kept
  open **forever**. A writer loop feeds it real-time 2 ch frames: **silence when
  idle** (this is both the full-duplex capture keepalive *and* the AEC reference
  baseline), and **TTS PCM when an utterance is queued** (mono upmixed to 2 ch).
  This replaces today's separate `aplay /dev/zero` keepalive in `mic_driver`.
- **Capture (as today):** publishes VAD-gated `audio/in`, **suppressed while
  `tts_playing`** (self-voice gating — the mic goes quiet during the bot's own
  speech, so CW never STT's the bot).
- **Control (as today):** polls `DOA_VALUE` → `voice/event`.

`tts_playing` is just a flag the playback loop sets while draining a queued
utterance; the capture loop reads it. Single device owner = no ALSA contention,
the AEC reference is always clocked, and gating is a local boolean.

## CW side (the "say" producer)

1. Call ElevenLabs with `output_format=pcm_16000` (model/voice CW's choice).
2. Collect the full utterance bytes (v1: whole utterance).
3. Wrap once with the `audio_frame` header (`channels=1, sample_rate=16000`) and
   `put` to `chatter/audio/out`.
4. **Self-voice gating** is mostly free: `audio/in` goes silent during playback,
   so CW receives no mic audio to mis-ingest. CW should still avoid acting on its
   own TTS as user input (jill-integration §6).

## Self-voice gating / AEC summary

- Hardware AEC cancels speaker→mic echo *because* TTS routes through the XVF3800.
- The service additionally **mutes `audio/in` while speaking** — belt and
  braces. Optionally publish a `tts_playing` flag in `chatter/status` if CW wants
  an explicit signal; otherwise the silence on `audio/in` is sufficient for v1.

## Latency & quality

- v1 latency = full ElevenLabs synth time before playback starts (the cost of
  whole-utterance). Fine for measured turn-taking; v2 streaming cuts it.
- 16 kHz wideband voice. If fidelity ever disappoints, the escape hatch is
  playing through a separate DAC/speaker at higher rate — but that **forfeits
  hardware AEC and barge-in** (would need software AEC or half-duplex muting).
  Not planned; noted as the known trade.

## Build checklist

**ChatterBot (Pi):**
- [x] `xvf_audio` service: folded in `mic_driver`; persistent stdin-fed playback
      loop (silence ↔ queued TTS), mono→2 ch upmix, ~150 ms silence prime.
- [x] Subscribe `chatter/audio/out` (binary), enqueue utterance PCM for playback.
- [x] `tts_playing` flag → suppress `audio/in` (and `voice/event`) while speaking.
- [x] Retired the `aplay /dev/zero` keepalive (subsumed by the duplex stream).
- [ ] **Live end-to-end test** against CW's `--say` on hardware.
- [ ] (v2) `audio/out/ctl` begin/end/cancel; streamed frames + jitter buffer.

**Cognitive_workbench (Jill):**
- [x] ElevenLabs `pcm_16000` synth → one `audio_frame` per utterance → `audio/out`
      (`voice_pipeline.synthesize`, `ChatterLink.send_audio_out`, `voice_harness --say`).
- [x] Self-voice gating handled Pi-side, so CW needs no mute flag.
- [ ] (v2) stream frames + barge-in via `audio/out/ctl`.

## Still open (for CW comment)

- Expose `tts_playing` as a `chatter/status` field, or leave it implicit (mic
  silence)?
- ElevenLabs **model/voice** + latency mode choice (CW side).
- Max utterance length before we *must* go streamed (single-payload ceiling).
