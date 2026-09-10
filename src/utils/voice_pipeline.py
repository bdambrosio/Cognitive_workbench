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

# Local STT (faster-whisper on this machine). Nothing spoken in the room
# leaves the LAN: the Pi streams VAD-gated audio here, this process
# transcribes it on the spare GPU, and the audio is dropped. That is the
# condition Jill set for voice-in (2026-09-10) and it is not optional.
# CW_STT_MODEL is a faster-whisper model name; CW_STT_GPU is a substring of
# the CUDA device name to run on (the spare card, never the one serving the
# live model). No match -> CPU int8, slower but still local.
_DEFAULT_STT_MODEL = os.environ.get("CW_STT_MODEL", "large-v3-turbo")
_STT_GPU_MATCH = os.environ.get("CW_STT_GPU", "5060")

# ElevenLabs TTS — the output side (docs/audio-out-design.md). The audio/out
# path is fixed 16 kHz mono S16_LE, so we always request `pcm_16000` (drop-in,
# no transcode). A voice id has no universal default; CW_TTS_VOICE_ID must be
# set to a real ElevenLabs voice before synthesis works.
_DEFAULT_TTS_MODEL = os.environ.get("CW_TTS_MODEL", "eleven_flash_v2_5")
_DEFAULT_VOICE_ID = os.environ.get("CW_TTS_VOICE_ID", "")

# The address check (is_addressed) runs on the character's own backend and
# refuses any route that is not on this machine or the LAN — see
# _backend_is_local. Same condition as STT above.


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
    """Wrap mono S16_LE PCM as an in-memory WAV (harness --save-wav and tests)."""
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


_whisper = None
_whisper_lock = __import__("threading").Lock()


def _preload_cuda_libs() -> None:
    """ctranslate2 dlopens libcublas.so.12 / libcudnn.so.9 by soname. The
    pip wheels put them under site-packages/nvidia/*/lib, which is not on the
    loader path, so open them here with RTLD_GLOBAL first; a later dlopen by
    soname then finds them already loaded. Failure is logged and left to the
    model load to report."""
    import ctypes
    import glob
    for pkg in ("nvidia.cublas", "nvidia.cudnn"):
        try:
            mod = __import__(pkg + ".lib", fromlist=["lib"])
            for so in sorted(glob.glob(os.path.join(mod.__path__[0], "*.so*"))):
                ctypes.CDLL(so, mode=ctypes.RTLD_GLOBAL)
        except Exception as e:
            logger.warning(f"voice: could not preload {pkg} libs: {e}")


def _pick_stt_device() -> tuple:
    """(device, device_index) for faster-whisper: the CUDA device whose name
    contains CW_STT_GPU, else CPU. Uses torch's enumeration, which is the
    same CUDA runtime order ctranslate2 sees."""
    try:
        import torch
        for i in range(torch.cuda.device_count()):
            if _STT_GPU_MATCH in torch.cuda.get_device_name(i):
                return "cuda", i
        logger.warning(f"voice: no CUDA device matching {_STT_GPU_MATCH!r}; "
                       "STT on CPU")
    except Exception as e:
        logger.warning(f"voice: torch unavailable for GPU pick ({e}); STT on CPU")
    return "cpu", 0


def _whisper_model():
    global _whisper
    with _whisper_lock:
        if _whisper is None:
            _preload_cuda_libs()
            from faster_whisper import WhisperModel
            device, idx = _pick_stt_device()
            compute = "float16" if device == "cuda" else "int8"
            _whisper = WhisperModel(_DEFAULT_STT_MODEL, device=device,
                                    device_index=idx, compute_type=compute)
            logger.info(f"voice: STT model {_DEFAULT_STT_MODEL} on "
                        f"{device}:{idx} ({compute})")
        return _whisper


def transcribe(pcm_mono: bytes, sample_rate: int, *,
               language: Optional[str] = "en") -> Optional[str]:
    """Transcribe one mono S16_LE utterance locally with faster-whisper.
    Returns the text, or None on empty audio / failure (logged, never
    silent). The audio is not written anywhere."""
    if not pcm_mono:
        return None
    try:
        model = _whisper_model()
        audio = np.frombuffer(pcm_mono, dtype=np.int16).astype(np.float32) / 32768.0
        if sample_rate != 16000:
            # Whisper wants 16 kHz; the Pi stream is 16 kHz by config, so this
            # is a guard, not a path we expect to take.
            logger.warning(f"voice: STT got {sample_rate} Hz, expected 16000")
            return None
        segments, _info = model.transcribe(audio, language=language,
                                           beam_size=5, vad_filter=False)
        text = " ".join(seg.text.strip() for seg in segments).strip()
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


def _backend_is_local(backend) -> bool:
    """True when the backend's route stays on this machine or the LAN: the
    OpenAI-compatible route with a loopback or private-range host."""
    import ipaddress
    from urllib.parse import urlparse
    if getattr(backend, "server", None) != "local":
        return False
    host = urlparse(getattr(backend, "base_url", "") or "").hostname or ""
    if host in ("localhost",):
        return True
    try:
        return ipaddress.ip_address(host).is_private
    except ValueError:
        return False


def is_addressed(text: str, name: str, *, backend) -> bool:
    """Semantic address-check: is this transcribed utterance actually directed
    AT the assistant `name` — vs. ambient cross-talk between other people,
    background speech, or a garbled fragment an energy-VAD + STT hallucinated
    from noise? Returns True/False from one short call on `backend`, the
    character's own model.

    This is the no-keyword-rule-compliant fallback for when the literal
    `matches_wake_word` gate misses — STT homophones ("Gill"/"Jo") and naturally
    phrased address that omits the name (docs/cw-voice-sensor-plan.md Issue B).

    Fails CLOSED (returns False) on empty text, a backend that is not local,
    or an error (logged): an explicit name-call already passes the free
    literal gate before this runs, so a dropped fallback just means "say the
    name again" rather than a noisy false fire. The transcript is not logged
    here; the caller decides what is recorded about unaddressed speech."""
    if not text or not text.strip() or not name:
        return False
    if backend is None or not _backend_is_local(backend):
        logger.error("voice: address-check refused: backend is not a local "
                     "route; room speech must not leave the LAN")
        return False
    prompt = (
        f"A short utterance was transcribed from an always-on room microphone. "
        f"The assistant that is listening is named \"{name}\". Decide whether "
        f"this utterance is addressed TO that assistant — a request, question, "
        f"or remark directed at it — as opposed to ambient conversation between "
        f"other people, speech not directed at the assistant, a mention of the "
        f"assistant in the third person, or a garbled fragment from background "
        f"noise.\n"
        f"The transcription is imperfect: the assistant's name may come out "
        f"as a misspelling or a word that sounds almost the same (\"{name}\" "
        f"heard as Gil or Jyl); treat those as the name. A different real "
        f"name that merely rhymes with it (Bill, Will, Phil) is a different "
        f"person, not the assistant.\n\n"
        f"Utterance: {text!r}\n\n"
        f"Answer with exactly one word: yes or no."
    )
    # Majority of three. The call runs at the model's configured sampling
    # temperature (never a literal here, by house rule), and one vote at
    # that temperature flipped on an ambient sentence about 1 time in 8;
    # three votes bring that to about 1 in 25 at ~0.25 s total.
    yes = 0
    try:
        for _ in range(3):
            ans = backend.chat([{"role": "user", "content": prompt}],
                               max_tokens=8, enable_thinking=False)
            yes += (ans or "").strip().lower().startswith("y")
    except Exception as e:
        logger.error(f"voice: address-check failed: {e}")
        return False
    return yes >= 2


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
