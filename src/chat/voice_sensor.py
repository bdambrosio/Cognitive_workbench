"""VoiceSensor — the integrated mic→turn component (docs/cw-voice-sensor-plan.md
§7). The standalone `src/voice_harness.py` worker, folded into a launcher-gated
component modelled on `affect/head_aliveness.py`.

What it does, while enabled:
- Subscribes to the Pi mic streams on the shared ChatterLink session
  (`chatter/voice/event` + binary `chatter/audio/in`) via `attach_voice`.
- Segments each utterance (`VoiceSegmenter`), transcribes it (OpenAI STT), and
  publishes the transcript to `cognitive/{character}/sense_data` tagged
  `source: VOICE_SOURCE, modality: VOICE_MODALITY` — so it becomes a user-like
  turn (from a distinct, unidentified spoken speaker) through the *unchanged*
  ingestion path (`zenoh_io._on_sense_data`). The chat loop then speaks the
  reply back out because the turn's modality is voice (§10).
- On a wake-word match, turns the head toward the talker (`doa_to_pan` →
  `send_head_cmd`), the same calibrated orient debugged in the harness.

Design / safety (mirrors HeadAliveness):
- **Off unless enabled** (launcher `--voice`). No-op, no threads, if disabled.
- **No-op when the bot is absent** — `attach_voice` ensures the link; if the Pi
  is unreachable it logs and the component simply receives nothing.
- **Self-voice gating is Pi-side** (§8): the `xvf_audio` service mutes
  `audio/in` while Jill speaks, so we never STT her own TTS. Nothing to do here.
- STT runs on a worker thread so Zenoh callbacks stay cheap.

Reuses the harness's shared building blocks verbatim (segmenter / STT / wake /
DoA→pan) from `utils.voice_pipeline` — one implementation, two callers.
"""
from __future__ import annotations

import json
import logging
import os
import queue
import threading
from datetime import datetime
from typing import Any, Optional

from utils.chatter_link import (
    get_link, _payload_bytes,
    NEUTRAL_TILT, NEUTRAL_PAN, PAN_MIN, PAN_MAX)
from utils.voice_pipeline import (
    VoiceSegmenter, transcribe, matches_wake_word, is_addressed, doa_to_pan,
    strip_leading_wake_word, VOICE_SOURCE, VOICE_MODALITY)

logger = logging.getLogger('chat.voice_sensor')

# Per-orient wait for the Pi to settle; short — the acknowledgement turn needs
# no confirmation and the worker is serial.
_ORIENT_TIMEOUT_S = 2.0


class VoiceSensor:
    def __init__(self, character_name: str, enabled: bool = False,
                 wake_word: Optional[str] = None) -> None:
        self.character_name = character_name
        self._enabled = enabled
        # Wake phrase: launcher --wake wins, else CW_WAKE_WORD, else Jill.
        self._wake_word = wake_word or os.environ.get('CW_WAKE_WORD', 'Jill')
        # Calibration dialed in via the harness (front_deg=90, sign=+1 for this
        # mount); env-overridable without a code change.
        self._front_deg = float(os.environ.get('CW_VOICE_FRONT_DEG', '90'))
        self._sign = int(os.environ.get('CW_VOICE_SIGN', '1'))
        self._orient = os.environ.get('CW_VOICE_ORIENT', '1') != '0'

        self._seg = VoiceSegmenter()
        self._utterances: "queue.Queue" = queue.Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pub: Optional[Any] = None

    def attach_session(self, session: Any) -> None:
        """Declare the sense_data publisher on the chat loop's session, attach
        the Pi mic subscribers, and start the STT worker. No-op if disabled or
        the Pi link can't be opened (logged)."""
        if not self._enabled:
            return
        try:
            self._pub = session.declare_publisher(
                f"cognitive/{self.character_name}/sense_data")
        except Exception as e:
            logger.warning(f"voice_sensor: sense_data publisher failed: {e}; "
                           "disabled")
            return
        err = get_link().attach_voice(self._on_voice, self._on_audio)
        if err:
            logger.warning(f"voice_sensor: bot link unavailable: {err}; "
                           "no mic input (component idle)")
            return
        self._thread = threading.Thread(
            target=self._worker, name='voice-sensor-stt', daemon=True)
        self._thread.start()
        logger.info(
            f"voice_sensor: listening (wake={self._wake_word!r} "
            f"orient={self._orient} front_deg={self._front_deg} "
            f"sign={self._sign:+d})")

    def close(self) -> None:
        self._stop.set()
        try:
            get_link().detach_voice()
        except Exception as e:
            logger.warning(f"voice_sensor: detach_voice failed: {e}")
        if self._pub is not None:
            try:
                self._pub.undeclare()
            except Exception as e:
                logger.warning(f"voice_sensor: publisher undeclare failed: {e}")
            self._pub = None

    # -- Pi subscriber callbacks (keep cheap) ----------------------------

    def _on_voice(self, sample: Any) -> None:
        try:
            evt = json.loads(_payload_bytes(sample).decode('utf-8'))
        except Exception as e:
            logger.warning(f"voice_sensor: bad voice/event dropped: {e}")
            return
        utt = self._seg.on_event(evt)
        if utt is not None:
            self._utterances.put(utt)

    def _on_audio(self, sample: Any) -> None:
        self._seg.on_audio(_payload_bytes(sample))

    # -- STT worker ------------------------------------------------------

    def _worker(self) -> None:
        while not self._stop.is_set():
            try:
                utt = self._utterances.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._handle_utterance(utt)
            except Exception as e:
                logger.error(f"voice_sensor: utterance handling failed: {e}")

    def _handle_utterance(self, utt: dict) -> None:
        if not utt.get('pcm'):
            return  # empty capture (e.g. voice/event with muted audio) — skip
        text = transcribe(utt['pcm'], utt['sample_rate'])
        if not text:
            return  # empty / STT error (already logged)
        # Address gate, EVERY turn: a free literal wake match first, and only on
        # a miss the semantic address-check (Issue B/C). Drops ambient speech
        # and energy-VAD noise hallucinations; keeps homophones + natural
        # address. Unaddressed utterances produce no turn and no orient.
        literal = matches_wake_word(text, self._wake_word)
        if not (literal or is_addressed(text, self._wake_word)):
            logger.info(f"voice_sensor: ignored (not addressed): {text!r}")
            return
        # On a literal wake match, strip a leading wake word so it isn't fed to
        # Jill as content. A bare wake word (a summons with no request after it)
        # strips to empty and is dropped. Semantically-addressed turns are left
        # verbatim — no literal token to strip, and keyword-stripping there would
        # violate the no-keyword rule and mangle natural phrasing.
        if literal:
            text = strip_leading_wake_word(text, self._wake_word)
            if not text:
                logger.info("voice_sensor: bare wake word (summons) — dropped")
                return
        logger.info(f"voice_sensor: heard {text!r} "
                    f"(doa={utt.get('start_doa')})")
        self._publish_turn(text)
        # Orient toward whoever she's responding to (acknowledgement turn).
        if self._orient:
            doa = utt.get('start_doa')
            if doa is not None:
                self._orient_to(doa)

    def _publish_turn(self, text: str) -> None:
        if self._pub is None:
            return
        envelope = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps({
                'source': VOICE_SOURCE, 'text': text,
                'modality': VOICE_MODALITY}),
        }
        try:
            self._pub.put(json.dumps(envelope).encode('utf-8'))
        except Exception as e:
            logger.warning(f"voice_sensor: sense_data put failed: {e}")

    def _orient_to(self, doa: float) -> None:
        pan = doa_to_pan(doa, front_deg=self._front_deg, sign=self._sign,
                         neutral_pan=NEUTRAL_PAN, pan_min=PAN_MIN,
                         pan_max=PAN_MAX)
        try:
            get_link().send_head_cmd(pan=pan, tilt=NEUTRAL_TILT, smooth=True,
                                     timeout=_ORIENT_TIMEOUT_S)
            logger.info(f"voice_sensor: orient doa={doa}° -> pan={pan:.0f}")
        except Exception as e:
            logger.warning(f"voice_sensor: orient failed: {e}")
