"""Reusable voice pipeline for the ChatterBot mic path.

Pure building blocks — no Zenoh, no chat-loop deps — shared by the standalone
debug harness (`src/voice_harness.py`) and the future integrated voice component
(see docs/cw-voice-sensor-plan.md §3, §7). Keeping these here means the harness
and the integration run the *same* segmenter / unpack / STT / wake code rather
than two drifting copies.

Wire formats (source of truth: ChatterBot docs/cw-voice-sensor.md §1-3 and
chatterbot/lib/audio_frame.py):
- chatter/voice/event : JSON {ts, vad: start|active|stop, doa_deg, confidence}
- chatter/audio/in    : binary 24-byte header (CBA1) + interleaved S16_LE PCM,
                        VAD-gated, default 2ch / 16 kHz.
"""

from __future__ import annotations

import io
import logging
import os
import struct
import wave
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# audio_frame header: magic, ver, fmt, channels, _rsvd, sample_rate, seq, ts
_HDR = "<4sBBBBIId"
_HDR_SIZE = struct.calcsize(_HDR)  # 24
_MAGIC = b"CBA1"
FMT_S16_LE = 0

# Identity of the spoken interlocutor. The room mic carries no speaker identity
# (the XVF3800 reports a single dominant talker, no diarization — cw-voice-sensor
# §7), so every voice utterance is, for now, the same *unidentified* speaker
# modelled as one character distinct from the typed "User". When speaker
# attribution lands (voice embeddings / face recognition), this is the slot that
# a resolved name replaces.
VOICE_SOURCE = "Voice"

# Modality tag carried on a voice turn, kept SEPARATE from identity (VOICE_SOURCE)
# so the chat loop's "speak the reply back out" routing keys off *how* the turn
# arrived, not *who* it was from — leaving identity free to become a real name
# later without breaking TTS. The single canonical value shared by the producer
# (voice sensor) and the consumer (chat loop) — import it, don't re-spell it.
VOICE_MODALITY = "voice"

# OpenAI STT model — configurable; see docs/cw-voice-sensor-plan.md §4 (D2).
_DEFAULT_STT_MODEL = os.environ.get("CW_STT_MODEL", "gpt-4o-transcribe")

# ElevenLabs TTS — the output side (docs/audio-out-design.md). The audio/out
# path is fixed 16 kHz mono S16_LE, so we always request `pcm_16000` (drop-in,
# no transcode). A voice id has no universal default; CW_TTS_VOICE_ID must be
# set to a real ElevenLabs voice before synthesis works.
_DEFAULT_TTS_MODEL = os.environ.get("CW_TTS_MODEL", "eleven_flash_v2_5")
_DEFAULT_VOICE_ID = os.environ.get("CW_TTS_VOICE_ID", "")

# Cheap classifier for the semantic address-check fallback (is_addressed); a
# small/fast model is plenty. See docs/cw-voice-sensor-plan.md Issue B.
_DEFAULT_ADDRESS_MODEL = os.environ.get("CW_ADDRESS_MODEL", "gpt-4o-mini")


def unpack_audio_frame(payload: bytes):
    """Split a chatter/audio/in binary payload into (header_dict, pcm_bytes).

    Raises ValueError on a short payload or bad magic — the caller logs and
    drops the frame (STT tolerates small gaps)."""
    if len(payload) < _HDR_SIZE:
        raise ValueError(f"audio frame too short: {len(payload)} bytes")
    magic, ver, fmt, ch, _r, rate, seq, ts = struct.unpack(
        _HDR, payload[:_HDR_SIZE])
    if magic != _MAGIC:
        raise ValueError(f"bad audio magic {magic!r}")
    return ({"version": ver, "format": fmt, "channels": ch,
             "sample_rate": rate, "seq": seq, "ts": ts},
            payload[_HDR_SIZE:])


def pack_audio_frame(pcm: bytes, *, sample_rate: int = 16000, channels: int = 1,
                     seq: int = 0, ts: float = 0.0, version: int = 1) -> bytes:
    """Inverse of `unpack_audio_frame`: prefix mono/interleaved S16_LE PCM with
    the 24-byte CBA1 header for `chatter/audio/out` (docs/audio-out-design.md,
    v1 = one frame = one whole utterance, `channels=1`; the Pi upmixes to 2 ch).

    `ts` is passed in (not read from the clock) so the function stays pure and
    round-trip testable; the caller stamps it."""
    header = struct.pack(_HDR, _MAGIC, version, FMT_S16_LE, channels, 0,
                         int(sample_rate), seq & 0xFFFFFFFF, float(ts))
    return header + pcm


def downmix_to_mono(pcm_bytes: bytes, channels: int) -> bytes:
    """Average interleaved int16 channels to mono S16_LE. No-op for mono."""
    if channels <= 1:
        return pcm_bytes
    pcm = np.frombuffer(pcm_bytes, dtype="<i2")
    usable = (len(pcm) // channels) * channels
    if usable != len(pcm):
        logger.warning(
            f"voice: pcm not a whole number of frames "
            f"({len(pcm)} samples, {channels} ch); truncating tail")
        pcm = pcm[:usable]
    mono = pcm.reshape(-1, channels).mean(axis=1).round().astype("<i2")
    return mono.tobytes()


def pcm_to_wav_bytes(pcm_mono: bytes, sample_rate: int) -> bytes:
    """Wrap mono S16_LE PCM as an in-memory WAV (for the OpenAI file upload)."""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)  # S16_LE
        w.setframerate(sample_rate)
        w.writeframes(pcm_mono)
    return buf.getvalue()


class VoiceSegmenter:
    """Assembles one utterance worth of PCM between a voice/event `start` and
    its `stop`, latching the talker bearing and counting dropped frames.

    Single dominant talker only (XVF3800 limitation). Feed `on_event` every
    voice/event and `on_audio` every audio/in frame; `on_event` returns an
    utterance dict on `stop`, else None."""

    def __init__(self) -> None:
        self._buf: list = []
        self._active = False
        self._channels: Optional[int] = None
        self._sample_rate: Optional[int] = None
        self._last_seq: Optional[int] = None
        self._dropped = 0
        self.start_doa: Optional[float] = None  # bearing latched at `start`

    def on_event(self, evt: dict) -> Optional[dict]:
        """Returns {pcm, sample_rate, start_doa, stop_doa, dropped_frames} on
        `stop`, else None. pcm is mono S16_LE."""
        vad = evt.get("vad")
        if vad == "start":
            self._buf = []
            self._active = True
            self._last_seq = None
            self._dropped = 0
            self.start_doa = evt.get("doa_deg")
        elif vad == "stop" and self._active:
            self._active = False
            pcm = b"".join(self._buf)
            self._buf = []
            if self._channels and self._channels > 1:
                pcm = downmix_to_mono(pcm, self._channels)
            return {
                "pcm": pcm,
                "sample_rate": self._sample_rate or 16000,
                "start_doa": self.start_doa,
                "stop_doa": evt.get("doa_deg"),
                "dropped_frames": self._dropped,
            }
        return None

    def on_audio(self, payload: bytes) -> None:
        if not self._active:
            return
        try:
            hdr, pcm = unpack_audio_frame(payload)
        except ValueError as e:
            logger.warning(f"voice: dropping bad audio frame: {e}")
            return
        self._channels = hdr["channels"]
        self._sample_rate = hdr["sample_rate"]
        seq = hdr["seq"]
        if self._last_seq is not None:
            gap = (seq - self._last_seq) & 0xFFFFFFFF  # seq wraps at 2**32
            if gap != 1:
                self._dropped += max(0, gap - 1)
                logger.warning(f"voice: audio seq gap {self._last_seq}->{seq}")
        self._last_seq = seq
        self._buf.append(pcm)


def transcribe(pcm_mono: bytes, sample_rate: int, *,
               model: Optional[str] = None,
               language: Optional[str] = "en") -> Optional[str]:
    """Transcribe one mono S16_LE utterance via the OpenAI API (reads
    OPENAI_API_KEY from env, like src/utils/OpenAIClient.py). Returns the text,
    or None on empty audio / API error (logged, never silent)."""
    if not pcm_mono:
        return None
    model = model or _DEFAULT_STT_MODEL
    try:
        from openai import OpenAI
    except Exception as e:
        logger.error(f"voice: openai SDK unavailable: {e}")
        return None
    bio = io.BytesIO(pcm_to_wav_bytes(pcm_mono, sample_rate))
    bio.name = "utterance.wav"  # SDK infers the audio format from the name
    try:
        client = OpenAI()
        kwargs = {"model": model, "file": bio}
        if language:
            kwargs["language"] = language
        resp = client.audio.transcriptions.create(**kwargs)
        text = (getattr(resp, "text", "") or "").strip()
        return text or None
    except Exception as e:
        logger.error(f"voice: STT failed: {e}")
        return None


def synthesize(text: str, *, voice_id: Optional[str] = None,
               model: Optional[str] = None) -> Optional[bytes]:
    """Synthesize `text` to raw S16_LE / 16 kHz / mono PCM via ElevenLabs
    (`output_format=pcm_16000`). The output-side twin of `transcribe()`: reads
    ELEVENLABS_API_KEY from env, returns PCM bytes, or None on empty text /
    missing key / missing voice / API error (logged, never silent).

    16 kHz is forced because the bot's XVF3800 playback clock is fixed there and
    its hardware AEC needs capture and playback on the same clock
    (docs/audio-out-design.md). The returned PCM is the body for
    `pack_audio_frame` → `chatter/audio/out`."""
    if not text or not text.strip():
        return None
    api_key = os.environ.get("ELEVENLABS_API_KEY")
    if not api_key:
        logger.error("voice: ELEVENLABS_API_KEY not set; cannot synthesize")
        return None
    voice_id = voice_id or _DEFAULT_VOICE_ID
    if not voice_id:
        logger.error("voice: no ElevenLabs voice id (set CW_TTS_VOICE_ID)")
        return None
    model = model or _DEFAULT_TTS_MODEL
    try:
        import requests
    except Exception as e:
        logger.error(f"voice: requests unavailable for TTS: {e}")
        return None
    url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}"
    try:
        resp = requests.post(
            url,
            params={"output_format": "pcm_16000"},
            headers={"xi-api-key": api_key, "Content-Type": "application/json"},
            json={"text": text, "model_id": model},
            timeout=30,
        )
        resp.raise_for_status()
        return resp.content or None
    except Exception as e:
        logger.error(f"voice: TTS failed: {e}")
        return None


def matches_wake_word(text: str, wake_word: str) -> bool:
    """TEMPORARY literal substring match (case-insensitive).

    This is a deliberate, documented exception to the project no-keyword-matching
    rule (docs/cw-voice-sensor-plan.md §5, Issue B). It misses ASR homophones
    ("Gill", "Jules") and can false-fire ("jillion"). REPLACE with an acoustic
    wake-word model or a semantic address-check once the pipeline is debugged."""
    if not text or not wake_word:
        return False
    return wake_word.strip().lower() in text.lower()


def strip_leading_wake_word(text: str, wake_word: str) -> str:
    """Remove a single *leading* occurrence of `wake_word` (case-insensitive)
    plus the punctuation/space separating it from the request, returning the
    remaining utterance. A wake word that appears mid-sentence (natural address,
    e.g. "what time is it Jill") is left untouched — we only strip an opening
    address, never carve a word out of the middle. Returns '' for a bare wake
    word with no request after it (a summons), which the caller drops.

    Same documented no-keyword-rule exception as `matches_wake_word`: only the
    literal-wake branch strips; semantically-addressed turns are left verbatim."""
    if not text:
        return ''
    if not wake_word:
        return text.strip()
    stripped = text.lstrip()
    wl = wake_word.strip().lower()
    if stripped.lower().startswith(wl):
        return stripped[len(wl):].lstrip(' \t,.!?:;-—')
    return stripped.strip()


def is_addressed(text: str, name: str, *, model: Optional[str] = None) -> bool:
    """Semantic address-check: is this transcribed utterance actually directed
    AT the assistant `name` — vs. ambient cross-talk between other people,
    background speech, or a garbled fragment an energy-VAD + STT hallucinated
    from noise? Returns True/False via a cheap OpenAI classify call.

    This is the no-keyword-rule-compliant fallback for when the literal
    `matches_wake_word` gate misses — STT homophones ("Gill"/"Jo") and naturally
    phrased address that omits the name (docs/cw-voice-sensor-plan.md Issue B).

    Fails CLOSED (returns False) on empty text / missing SDK / API error
    (logged): an explicit name-call already passes the free literal gate before
    this runs, so a dropped fallback just means "say the name again" rather than
    a noisy false fire."""
    if not text or not text.strip() or not name:
        return False
    model = model or _DEFAULT_ADDRESS_MODEL
    try:
        from openai import OpenAI
    except Exception as e:
        logger.error(f"voice: openai SDK unavailable for address-check: {e}")
        return False
    prompt = (
        f"A short utterance was transcribed from an always-on room microphone. "
        f"The assistant that is listening is named \"{name}\". Decide whether "
        f"this utterance is addressed TO that assistant — a request, question, "
        f"or remark directed at it — as opposed to ambient conversation between "
        f"other people, speech not directed at the assistant, or a garbled "
        f"fragment from background noise.\n"
        f"The transcription is imperfect: the assistant's name may be "
        f"mis-transcribed as a similar-sounding word (a homophone or near-miss, "
        f"e.g. \"{name}\" heard as a close-sounding name) — treat such a "
        f"near-match to the name as the name itself.\n\n"
        f"Utterance: {text!r}\n\n"
        f"Answer with exactly one word: yes or no."
    )
    try:
        client = OpenAI()
        resp = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0,
            max_tokens=2,
        )
        ans = (resp.choices[0].message.content or "").strip().lower()
        return ans.startswith("y")
    except Exception as e:
        logger.error(f"voice: address-check failed: {e}")
        return False


def doa_to_pan(doa_deg: float, front_deg: float = 0.0, sign: int = 1,
               neutral_pan: float = 90.0,
               pan_min: float = 10.0, pan_max: float = 170.0) -> float:
    """Map an XVF3800 array-frame bearing (0-359°) to a head pan angle.

    CW-side fallback for when the Pi's own doa_deg→pan mapping is a no-op
    (docs/cw-voice-sensor-plan.md §5, option 2). `front_deg` is the array
    azimuth that points straight ahead (→ neutral_pan); `sign` (+1/-1) selects
    which way an increasing bearing turns the head. Bearings outside the pan
    envelope clamp to the nearest limit — a front-facing companion can't look
    behind itself."""
    rel = ((doa_deg - front_deg + 180.0) % 360.0) - 180.0  # → [-180, 180)
    pan = neutral_pan + sign * rel
    return max(pan_min, min(pan_max, pan))
