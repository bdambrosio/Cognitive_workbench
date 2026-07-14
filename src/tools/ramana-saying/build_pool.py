#!/usr/bin/env python3
"""
One-time curation: build the bundled sayings pool for the ramana-saying tool.

Reads the verbatim, reviewed Talks corpus and extracts single
question -> answer exchanges into a self-contained data/sayings.jsonl so the
runtime tool has no dependency on the (separate) /data/satsang repo.

Run by hand when refreshing the pool:
    python src/tools/ramana-saying/build_pool.py

Each output record: {"id", "talk", "question", "answer"} plus, where the raw
book text yields a clean narrative preamble for the talk (the scene-setting
paragraph before the dialogue begins), a "context" field on that talk's first
exchange only — the preamble describes the occasion of the talk, so attaching
it to later, possibly unrelated exchanges would misattribute it.
"""

import json
import re
from pathlib import Path

SOURCE = Path(
    "/data/satsang/ramana/Talks-with-Sri-Ramana-Maharshi-parsed-reviewed-merged.jsonl"
)
RAW_TEXTS = [
    Path("/data/satsang/ramana/Talks-with-Sri-Ramana-Maharshi.txt"),
    Path("/data/satsang/ramana/Talks-with-Sri-Ramana-Maharshi-Vol2.txt"),
]
OUT = Path(__file__).parent / "data" / "sayings.jsonl"

# Keep answers substantial but quotable. Word bounds, not content filters.
MIN_ANSWER_WORDS = 8
MAX_ANSWER_WORDS = 130

_TALK_NUM = re.compile(r"talk_(\d+)")

# Drop answers that end mid-sentence (chunk-boundary truncation in the source):
# require terminal sentence punctuation, allowing a trailing quote/paren.
_ENDS_SENTENCE = re.compile(r"[.?!][\"')”’]?$")


def talk_label(record_id: str) -> str | None:
    """'talk_18_chunk2' -> 'Talk 18'."""
    m = _TALK_NUM.search(record_id or "")
    return f"Talk {m.group(1)}" if m else None


# --- Narrative context extraction (from the raw book text) -------------------
#
# The reviewed corpus is pure Q&A; the narrative scene-setting exists only in
# the raw PDF-extracted text, which is partially scrambled. We take only the
# lines between a "Talk N." header and the first dialogue marker, drop the
# short scrambled fragments ("Mr. W.", "Y."), and keep the result only if it
# reads like prose: sensible length, ends like a sentence, and is not itself
# a question (some talks open directly with a named asker's question).

_DIALOGUE_MARKER = re.compile(r"\b[DM]\.\s*:")
_ENDS_PROSE = re.compile(r"[.!][\"')”’]?$")
_MIN_CONTEXT_CHARS = 40
_MAX_CONTEXT_CHARS = 600
_MIN_LINE_CHARS = 25  # raw-text lines shorter than this are scrambled fragments


def load_talk_contexts() -> dict[str, str]:
    contexts: dict[str, str] = {}
    for raw_text in RAW_TEXTS:
        if not raw_text.exists():
            print(f"Raw text not found, skipping context extraction: {raw_text}")
            continue
        parts = re.split(r"\n(Talk \d+)\.\n", raw_text.read_text())
        for i in range(1, len(parts) - 1, 2):
            talk = parts[i]
            body = parts[i + 1]
            m = _DIALOGUE_MARKER.search(body)
            preamble = body[: m.start()] if m else ""
            lines = [ln.strip() for ln in preamble.splitlines()]
            text = " ".join(ln for ln in lines if len(ln) >= _MIN_LINE_CHARS)
            text = " ".join(text.split())
            if not (_MIN_CONTEXT_CHARS <= len(text) <= _MAX_CONTEXT_CHARS):
                continue
            if not _ENDS_PROSE.search(text):  # also rejects preambles ending in '?'
                continue
            contexts.setdefault(talk, text)
    return contexts


def main() -> None:
    if not SOURCE.exists():
        raise SystemExit(f"Source corpus not found: {SOURCE}")

    OUT.parent.mkdir(parents=True, exist_ok=True)

    contexts = load_talk_contexts()
    talks_with_context: set[str] = set()
    seen: set[tuple[str, str]] = set()
    sayings: list[dict] = []

    for line in SOURCE.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        record = json.loads(line)
        talk = talk_label(record.get("id", ""))
        if not talk:
            continue
        messages = record.get("messages", [])
        # Walk adjacent human -> assistant pairs.
        for i in range(len(messages) - 1):
            if messages[i].get("role") != "human":
                continue
            if messages[i + 1].get("role") != "assistant":
                continue
            question = (messages[i].get("content") or "").strip()
            answer = (messages[i + 1].get("content") or "").strip()
            n_words = len(answer.split())
            if n_words < MIN_ANSWER_WORDS or n_words > MAX_ANSWER_WORDS:
                continue
            if not _ENDS_SENTENCE.search(answer):
                continue
            # Dedup across duplicate/chunked records by talk + answer prefix.
            key = (talk, answer[:60])
            if key in seen:
                continue
            seen.add(key)
            saying = {
                "id": f"{talk.replace(' ', '_').lower()}#{len(sayings)}",
                "talk": talk,
                "question": question,
                "answer": answer,
            }
            # Scene-setting preamble belongs to the talk's opening exchange.
            if talk in contexts and talk not in talks_with_context:
                saying["context"] = contexts[talk]
                talks_with_context.add(talk)
            sayings.append(saying)

    with open(OUT, "w") as f:
        for s in sayings:
            f.write(json.dumps(s, ensure_ascii=False) + "\n")

    print(f"Wrote {len(sayings)} sayings to {OUT}, "
          f"{len(talks_with_context)} with narrative context")


if __name__ == "__main__":
    main()
