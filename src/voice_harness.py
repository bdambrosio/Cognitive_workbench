#!/usr/bin/env python3
"""Standalone voice-pipeline harness — debug ChatterBot voice capture, DoA,
STT, and the wake-word head-orient against the live Pi WITHOUT launching Jill
or any infospace world (docs/cw-voice-sensor-plan.md §3).

Opens its own isolated Zenoh session to the Pi router (same endpoint logic as
ChatterLink), subscribes chatter/voice/event + chatter/audio/in, runs the shared
voice_pipeline, and prints a live trace. With --orient it also publishes
chatter/head/cmd {doa_deg} so the acoustic head-turn (and the one-time Pi DoA
calibration) can be validated before integration.

Usage:
    python src/voice_harness.py                      # trace + STT, no head motion
    python src/voice_harness.py --orient             # also turn the head on "Jill"
    python src/voice_harness.py --no-stt             # voice/audio events only
    python src/voice_harness.py --save-wav ./utts    # dump each utterance WAV
    python src/voice_harness.py --router tcp/HOST:7447 --wake Jill
"""

import argparse
import json
import logging
import os
import queue
import sys
import threading
import time
from pathlib import Path

# Make sibling src/ modules importable when launched from anywhere.
_SRC_DIR = os.path.dirname(os.path.abspath(__file__))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from utils.chatter_link import (  # noqa: E402
    router, _normalize_endpoint, _payload_bytes,
    PAN_MIN, PAN_MAX, NEUTRAL_PAN, NEUTRAL_TILT)
from utils.voice_pipeline import (  # noqa: E402
    VoiceSegmenter, transcribe, matches_wake_word, pcm_to_wav_bytes, doa_to_pan,
    synthesize, pack_audio_frame)

logger = logging.getLogger("voice_harness")

_T_VOICE = "chatter/voice/event"
_T_AUDIO = "chatter/audio/in"
_T_HEAD = "chatter/head/cmd"
_T_AUDIO_OUT = "chatter/audio/out"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--router", default=None,
                    help="Pi zenoh endpoint (default: CHATTER_ROUTER env)")
    ap.add_argument("--wake", default=os.environ.get("CW_WAKE_WORD", "Jill"),
                    help="wake phrase (literal match for now; default: Jill)")
    ap.add_argument("--orient", action="store_true",
                    help="turn the head toward the speaker on a wake match")
    ap.add_argument("--front-deg", type=float, default=0.0,
                    help="DoA→pan calibration: array bearing that points straight "
                         "ahead (→ neutral pan). Dial this in live. Default 0")
    ap.add_argument("--sign", type=int, default=1, choices=(1, -1),
                    help="DoA→pan calibration: which way an increasing bearing "
                         "turns the head (+1/-1). Default 1")
    ap.add_argument("--send-doa", action="store_true",
                    help="send raw {doa_deg} and let the Pi map it (use once the "
                         "Pi mapping works); default maps to {pan} CW-side")
    ap.add_argument("--save-wav", default=None, metavar="DIR",
                    help="dump each utterance as a WAV here")
    ap.add_argument("--say", default=None, metavar="TEXT",
                    help="standalone TTS test: synthesize TEXT via ElevenLabs, "
                         "publish it to chatter/audio/out, and exit (no mic). "
                         "Exercises synth + audio_frame packing + publish. Pair "
                         "with --save-wav to also dump the synth locally.")
    ap.add_argument("--no-stt", action="store_true",
                    help="skip STT — just trace voice/audio events")
    ap.add_argument("--model", default=None, help="OpenAI STT model override")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    import zenoh
    ep = _normalize_endpoint(args.router or router())
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([ep]))
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("scouting/gossip/enabled", "true")
    session = zenoh.open(config)
    print(f"[harness] connected to {ep}  wake={args.wake!r}  "
          f"orient={args.orient}  stt={not args.no_stt}")

    # Standalone TTS test: synthesize → frame → publish to audio/out → exit.
    # No mic, no segmenter. Verifies the output half against the Pi xvf_audio
    # service the same way the listen loop verifies the input half.
    if args.say is not None:
        pcm = synthesize(args.say)
        if not pcm:
            print("[say ] synthesis failed (see log) — nothing published")
            session.close()
            return 1
        if args.save_wav:
            Path(args.save_wav).mkdir(parents=True, exist_ok=True)
            fn = Path(args.save_wav) / f"say_{len(pcm)}.wav"
            fn.write_bytes(pcm_to_wav_bytes(pcm, 16000))
            print(f"[wav ] saved {fn}")
        frame = pack_audio_frame(pcm, sample_rate=16000, channels=1,
                                 seq=0, ts=time.time())
        out_pub = session.declare_publisher(_T_AUDIO_OUT)
        out_pub.put(frame)
        print(f"[say ] published {len(pcm)} pcm bytes ({len(frame)} framed) "
              f"to {_T_AUDIO_OUT}")
        time.sleep(0.5)  # let zenoh flush the payload before we close
        session.close()
        return 0

    head_pub = session.declare_publisher(_T_HEAD)
    seg = VoiceSegmenter()
    utterances: "queue.Queue" = queue.Queue()
    stop = threading.Event()

    if args.save_wav:
        Path(args.save_wav).mkdir(parents=True, exist_ok=True)

    def on_voice(sample) -> None:
        try:
            evt = json.loads(_payload_bytes(sample).decode("utf-8"))
        except Exception as e:
            logger.warning(f"bad voice/event dropped: {e}")
            return
        vad = evt.get("vad")
        if vad == "start":
            print(f"[voice] START doa={evt.get('doa_deg')}°  "
                  f"conf={evt.get('confidence')}")
        elif vad == "stop":
            print(f"[voice] STOP  doa={evt.get('doa_deg')}°")
        utt = seg.on_event(evt)
        if utt is not None:
            utterances.put(utt)

    def on_audio(sample) -> None:
        seg.on_audio(_payload_bytes(sample))

    session.declare_subscriber(_T_VOICE, on_voice)
    session.declare_subscriber(_T_AUDIO, on_audio)

    def worker() -> None:
        while not stop.is_set():
            try:
                utt = utterances.get(timeout=0.5)
            except queue.Empty:
                continue
            nbytes = len(utt["pcm"])
            dur = nbytes / 2 / max(1, utt["sample_rate"])
            print(f"[audio] utterance {nbytes}B  ~{dur:.1f}s  "
                  f"dropped={utt['dropped_frames']}  start_doa={utt['start_doa']}")
            if args.save_wav and nbytes:
                wav = pcm_to_wav_bytes(utt["pcm"], utt["sample_rate"])
                fn = Path(args.save_wav) / f"utt_{int(time.time())}_{nbytes}.wav"
                fn.write_bytes(wav)
                print(f"[wav ] saved {fn}")
            if args.no_stt:
                continue
            t0 = time.time()
            text = transcribe(utt["pcm"], utt["sample_rate"], model=args.model)
            ms = int((time.time() - t0) * 1000)
            print(f"[stt ] {text!r}  ({ms} ms)")
            if text and matches_wake_word(text, args.wake):
                doa = utt["start_doa"]
                if doa is None:
                    print(f"[wake] MATCHED {args.wake!r}  (no bearing — skip orient)")
                    continue
                pan = doa_to_pan(doa, front_deg=args.front_deg, sign=args.sign,
                                 neutral_pan=NEUTRAL_PAN,
                                 pan_min=PAN_MIN, pan_max=PAN_MAX)
                print(f"[wake] MATCHED {args.wake!r}  doa={doa}° -> pan={pan:.0f} "
                      f"(front={args.front_deg} sign={args.sign:+d})")
                if args.orient:
                    if args.send_doa:
                        cmd = {"ts": time.time(), "doa_deg": doa}
                    else:
                        cmd = {"ts": time.time(), "pan": pan,
                               "tilt": NEUTRAL_TILT, "smooth": True}
                    head_pub.put(json.dumps(cmd).encode("utf-8"))
                    print(f"[head] sent head/cmd {cmd}")

    t = threading.Thread(target=worker, name="voice-harness-stt", daemon=True)
    t.start()

    print("[harness] listening — speak at the Pi mic. Ctrl-C to quit.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[harness] shutting down")
    finally:
        stop.set()
        t.join(timeout=2.0)
        try:
            session.close()
        except Exception as e:
            logger.warning(f"session close failed: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
