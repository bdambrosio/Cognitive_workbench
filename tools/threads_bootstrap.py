#!/usr/bin/env python3
"""Threads bootstrap tool — discover initial threads from a Jill
conversation history file.

Standalone, offline. Does not touch CW state. Reads
scenarios/<world>/<character>/memory/conversation.txt, embeds each
turn with the same model CW uses (BAAI/bge-small-en-v1.5), clusters
with HDBSCAN, generates LLM-named summaries per cluster, writes JSON
output for review.

After review, a separate pipeline can install the resulting threads
into the agent_threads Collection (not implemented here).

Usage:
    python tools/threads_bootstrap.py \\
        --conversation scenarios/jill_chat/Jill/memory/conversation.txt \\
        --out tools/threads_bootstrap_out.json

Optional tuning:
    --min-cluster-size N   (HDBSCAN min_cluster_size; default 5)
    --min-samples N        (HDBSCAN min_samples; default same as min_cluster_size)
    --include-jill         include Jill's reply turns (default: user turns only)
    --skip-llm             skip LLM naming pass; emit clusters with
                           placeholder names (faster iteration)
    --judge-model MODEL    Anthropic model for naming (default
                           claude-sonnet-4-6)

CLAUDE_API_KEY must be set unless --skip-llm.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("tools.threads_bootstrap")


# ---------------------------------------------------------------------------
# Conversation parser
# ---------------------------------------------------------------------------

# Block delimiter: line of 80 '=' characters. Header line follows; pattern:
#   [TIMESTAMP] User -> Jill (close=False)            user turn
#   [TIMESTAMP] User <- Jill (act=say close=False)    agent turn
_DELIM_RE = re.compile(r"^=+\s*$")
_HEADER_RE = re.compile(
    r"^\s*\[(?P<ts>[^\]]+)\]\s+"
    r"(?P<src>\S+)\s+"
    r"(?P<arrow><-|->)\s+"
    r"(?P<dst>\S+)\s+"
    r"\((?P<meta>[^)]*)\)\s*$"
)


@dataclass
class Turn:
    index: int
    ts: str
    direction: str   # 'user' or 'agent'
    src: str
    dst: str
    text: str


def parse_conversation(path: Path) -> List[Turn]:
    """Parse a conversation.txt file. Each turn is a block delimited by
    `===...===` lines, preceded by a header line and followed by free
    text until the next delimiter."""
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()
    turns: List[Turn] = []
    i = 0
    n = len(lines)

    while i < n:
        # Find next delimiter line.
        if not _DELIM_RE.match(lines[i]):
            i += 1
            continue
        # Expect header on next line.
        if i + 1 >= n:
            break
        header_line = lines[i + 1]
        m = _HEADER_RE.match(header_line)
        if not m:
            i += 1
            continue
        # Expect closing delimiter.
        if i + 2 >= n or not _DELIM_RE.match(lines[i + 2]):
            i += 1
            continue
        # Collect body lines until next delimiter.
        j = i + 3
        body_lines: List[str] = []
        while j < n and not _DELIM_RE.match(lines[j]):
            body_lines.append(lines[j])
            j += 1
        body = "\n".join(body_lines).strip()

        ts = m.group("ts")
        src = m.group("src")
        arrow = m.group("arrow")
        dst = m.group("dst")
        # arrow `->` means src->dst (so user->jill = user turn);
        # arrow `<-` means dst->src reading R-to-L (so user<-jill means jill spoke).
        # Direction labelled by who *spoke* the turn:
        if arrow == "->":
            direction = "user"
        else:
            direction = "agent"

        if body:
            turns.append(Turn(
                index=len(turns),
                ts=ts,
                direction=direction,
                src=src,
                dst=dst,
                text=body,
            ))
        i = j

    return turns


# ---------------------------------------------------------------------------
# Embedding (matches CW's BAAI/bge-small-en-v1.5)
# ---------------------------------------------------------------------------

def embed_turns(turns: List[Turn], model_name: str = "BAAI/bge-small-en-v1.5"
                ) -> np.ndarray:
    from sentence_transformers import SentenceTransformer
    logger.info(f"loading embedding model: {model_name}")
    device = "cuda" if _cuda_available() else "cpu"
    model = SentenceTransformer(model_name, device=device)
    texts = [t.text for t in turns]
    logger.info(f"embedding {len(texts)} turns on {device}")
    # bge-small-en is normalized when normalize_embeddings=True; gives
    # us cosine-similarity behavior under Euclidean distance.
    embs = model.encode(
        texts,
        batch_size=64,
        normalize_embeddings=True,
        show_progress_bar=True,
        convert_to_numpy=True,
    )
    return embs


def _cuda_available() -> bool:
    try:
        import torch
        return bool(torch.cuda.is_available())
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Clustering
# ---------------------------------------------------------------------------

@dataclass
class Cluster:
    cluster_id: int                      # HDBSCAN label (-1 = noise)
    turn_indices: List[int]
    centroid: np.ndarray
    size: int = 0

    def representative_indices(self, embs: np.ndarray, k: int = 8) -> List[int]:
        """Top-k turns closest to the centroid, by cosine distance."""
        if not self.turn_indices:
            return []
        member_embs = embs[self.turn_indices]
        # Both are L2-normalized → cosine sim = dot product
        sims = member_embs @ self.centroid
        order = np.argsort(-sims)
        chosen = [self.turn_indices[i] for i in order[:k]]
        return chosen


def cluster_embeddings(embs: np.ndarray,
                       min_cluster_size: int = 5,
                       min_samples: Optional[int] = None
                       ) -> Tuple[np.ndarray, List[Cluster]]:
    """HDBSCAN over embedding space. Returns (per-turn labels, list of
    Cluster objects). Label -1 = noise (no cluster assigned)."""
    from sklearn.cluster import HDBSCAN
    logger.info(f"clustering with HDBSCAN(min_cluster_size={min_cluster_size}, "
                f"min_samples={min_samples or min_cluster_size})")
    # Embeddings are L2-normalized; Euclidean ≈ cosine in this space.
    clusterer = HDBSCAN(
        min_cluster_size=min_cluster_size,
        min_samples=min_samples or min_cluster_size,
        metric="euclidean",
        cluster_selection_method="eom",
    )
    labels = clusterer.fit_predict(embs)
    unique = sorted(set(labels.tolist()))
    clusters: List[Cluster] = []
    for cid in unique:
        if cid == -1:
            continue
        member_idx = [int(i) for i, l in enumerate(labels) if l == cid]
        member_embs = embs[member_idx]
        centroid = member_embs.mean(axis=0)
        # Re-normalize centroid for cosine-friendly downstream use.
        norm = np.linalg.norm(centroid)
        if norm > 0:
            centroid = centroid / norm
        clusters.append(Cluster(
            cluster_id=int(cid),
            turn_indices=member_idx,
            centroid=centroid,
            size=len(member_idx),
        ))
    clusters.sort(key=lambda c: -c.size)
    return labels, clusters


# ---------------------------------------------------------------------------
# LLM naming
# ---------------------------------------------------------------------------

_NAMING_PROMPT = """\
You are labelling a cluster of conversation turns from a chat session
between a user and an AI assistant named Jill. The turns below were
identified as a coherent topical or activity-level "thread" by an
embedding-based clustering algorithm.

Your job: produce a short label (2-4 words, snake_case) and a 1-2
sentence summary describing what activity / topic / project this
thread represents. The label should be specific enough to distinguish
this thread from others (avoid generic labels like "general_chat" or
"misc"). The summary should describe what the user and Jill were
working on or discussing.

Representative turns from this cluster ({n_turns} total turns; {n_shown}
shown below, ordered by closeness to cluster centroid):

{turns_block}

Output ONLY a JSON object (no prose, no markdown fences):
{{
  "name": "snake_case_label",
  "summary": "One to two sentences describing the activity or topic."
}}
"""


def name_clusters(clusters: List[Cluster], turns: List[Turn],
                  embs: np.ndarray, judge_model: str
                  ) -> List[Dict[str, str]]:
    """One LLM call per cluster. Returns parallel list of
    {name, summary} dicts."""
    import anthropic
    api_key = os.getenv("CLAUDE_API_KEY")
    if not api_key:
        raise RuntimeError("CLAUDE_API_KEY not set")
    client = anthropic.Anthropic(api_key=api_key)

    out: List[Dict[str, str]] = []
    for ci, cluster in enumerate(clusters):
        rep_idx = cluster.representative_indices(embs, k=8)
        turns_block_lines: List[str] = []
        for t_idx in rep_idx:
            t = turns[t_idx]
            text = t.text.strip().replace("\n", " ")
            if len(text) > 240:
                text = text[:237] + "..."
            turns_block_lines.append(f"- [{t.direction}] {text}")
        turns_block = "\n".join(turns_block_lines) or "(no turns)"

        prompt = _NAMING_PROMPT.format(
            n_turns=cluster.size,
            n_shown=len(rep_idx),
            turns_block=turns_block,
        )

        logger.info(f"naming cluster {cluster.cluster_id} "
                    f"({cluster.size} turns, {ci+1}/{len(clusters)})")
        try:
            resp = client.messages.create(
                model=judge_model,
                max_tokens=300,
                temperature=0.2,
                messages=[{"role": "user", "content": prompt}],
            )
            raw = resp.content[0].text.strip()
            # Strip markdown fences if present.
            if raw.startswith("```"):
                raw = raw.strip("`").lstrip("json").strip()
            parsed = json.loads(raw)
            name = str(parsed.get("name", f"cluster_{cluster.cluster_id}")).strip()
            summary = str(parsed.get("summary", "")).strip()
            out.append({"name": name, "summary": summary})
        except Exception as e:
            logger.warning(f"naming cluster {cluster.cluster_id} failed: {e}")
            out.append({
                "name": f"cluster_{cluster.cluster_id}",
                "summary": "(naming failed)",
            })
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="Discover initial threads from a Jill conversation history.")
    parser.add_argument("--conversation", type=Path, required=True,
                        help="Path to conversation.txt")
    parser.add_argument("--out", type=Path, default=None,
                        help="Output JSON path. Default: <conversation_dir>/threads_bootstrap_out.json")
    parser.add_argument("--min-cluster-size", type=int, default=5)
    parser.add_argument("--min-samples", type=int, default=None)
    parser.add_argument("--include-jill", action="store_true",
                        help="Include Jill's reply turns (default: user turns only)")
    parser.add_argument("--skip-llm", action="store_true",
                        help="Skip LLM naming; use placeholder names")
    parser.add_argument("--judge-model", type=str, default="claude-sonnet-4-6")
    args = parser.parse_args()

    conv_path = args.conversation.resolve()
    if not conv_path.is_file():
        parser.error(f"conversation file does not exist: {conv_path}")

    out_path = args.out
    if out_path is None:
        out_path = conv_path.parent / "threads_bootstrap_out.json"
    out_path = Path(out_path).resolve()

    # 1. Parse turns.
    logger.info(f"parsing {conv_path}")
    all_turns = parse_conversation(conv_path)
    logger.info(f"parsed {len(all_turns)} turns "
                f"({sum(1 for t in all_turns if t.direction == 'user')} user, "
                f"{sum(1 for t in all_turns if t.direction == 'agent')} agent)")

    if args.include_jill:
        turns = all_turns
    else:
        turns = [t for t in all_turns if t.direction == "user"]
    if not turns:
        parser.error("no turns to embed after filtering")

    # 2. Embed.
    embs = embed_turns(turns)
    logger.info(f"embedding shape: {embs.shape}")

    # 3. Cluster.
    labels, clusters = cluster_embeddings(
        embs, min_cluster_size=args.min_cluster_size,
        min_samples=args.min_samples)
    n_clustered = sum(c.size for c in clusters)
    n_noise = len(turns) - n_clustered
    logger.info(f"clustering: {len(clusters)} clusters, "
                f"{n_clustered} clustered turns, {n_noise} noise")
    for c in clusters:
        logger.info(f"  cluster {c.cluster_id}: {c.size} turns")

    # 4. Name.
    if args.skip_llm:
        names = [{"name": f"cluster_{c.cluster_id}",
                  "summary": "(LLM naming skipped)"} for c in clusters]
    else:
        names = name_clusters(clusters, turns, embs, args.judge_model)

    # 5. Emit JSON.
    out_threads: List[Dict[str, Any]] = []
    for c, name_blob in zip(clusters, names):
        rep_idx = c.representative_indices(embs, k=10)
        rep_turns = []
        for t_idx in rep_idx:
            t = turns[t_idx]
            rep_turns.append({
                "index": t.index,
                "ts": t.ts,
                "direction": t.direction,
                "text": t.text[:300] + ("..." if len(t.text) > 300 else ""),
            })
        out_threads.append({
            "cluster_id": c.cluster_id,
            "name": name_blob["name"],
            "summary": name_blob["summary"],
            "turn_count": c.size,
            "turn_indices": c.turn_indices,
            "centroid_dim": int(c.centroid.shape[0]),
            "centroid_l2_norm": float(np.linalg.norm(c.centroid)),
            "centroid": c.centroid.tolist(),
            "representative_turns": rep_turns,
        })

    payload = {
        "source_conversation": str(conv_path),
        "n_total_turns_parsed": len(all_turns),
        "n_turns_clustered_over": len(turns),
        "include_jill": args.include_jill,
        "min_cluster_size": args.min_cluster_size,
        "min_samples": args.min_samples or args.min_cluster_size,
        "embedding_model": "BAAI/bge-small-en-v1.5",
        "n_clusters": len(clusters),
        "n_noise": n_noise,
        "noise_indices": [int(i) for i, l in enumerate(labels) if l == -1],
        "threads": out_threads,
    }
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\nWrote: {out_path}")
    print(f"  clusters: {len(clusters)}")
    print(f"  clustered turns: {n_clustered}/{len(turns)}")
    print(f"  noise turns: {n_noise}")
    if out_threads:
        print("\nDiscovered threads (sorted by size):")
        for t in out_threads:
            print(f"  [{t['turn_count']:>3} turns] {t['name']:<32} — {t['summary']}")


if __name__ == "__main__":
    main()
