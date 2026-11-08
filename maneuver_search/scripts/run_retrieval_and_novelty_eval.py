"""End-to-end run: mine the 220,000-segment maneuver corpus with
autolabel_cli, embed and index every segment with hnswlib, measure
precision@10 against a 600-pair scripted-relevance labeled set, and
measure the novelty inversion's catch rate against 20 injected unseen
scene types. This is the script behind every headline number in the
README's Measured results table for this project.
"""
from __future__ import annotations

import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from maneuver_search.corpus_io import load_maneuver_segments, run_mine_corpus
from maneuver_search.features import build_feature_matrix, segment_to_raw_feature_vector
from maneuver_search.index import build_index, query_top_k
from maneuver_search.labeling import is_relevant
from maneuver_search.novelty import (
    calibrate_novelty_threshold,
    generate_injected_novel_raw_vectors,
    novelty_scores,
)

NUM_SEGMENTS = 220_000
SEED = 1
N_QUERIES_FOR_PRECISION = 60
K = 10
N_INJECTED_NOVEL = 20


def find_cli() -> str:
    repo_root = Path(__file__).resolve().parents[2]
    for candidate in (repo_root / "build" / "autolabel_cli", repo_root / "build" / "autolabel_cli.exe"):
        if candidate.exists():
            return str(candidate)
    raise FileNotFoundError("autolabel_cli not found under build/; build the C++ side first")


def main() -> None:
    cli = find_cli()
    print(f"Using autolabel_cli: {cli}")

    with tempfile.TemporaryDirectory() as tmp:
        corpus_path = str(Path(tmp) / "corpus.tsv")
        events_path = str(Path(tmp) / "events.tsv")
        t0 = time.perf_counter()
        run_mine_corpus(cli, corpus_path, events_path, num_segments=NUM_SEGMENTS, seed=SEED)
        mine_elapsed = time.perf_counter() - t0
        print(f"Mined {NUM_SEGMENTS} segments in {mine_elapsed:.1f}s")

        segments = load_maneuver_segments(corpus_path, events_path)

    print(f"Loaded {len(segments)} maneuver segments")
    by_label: dict[str, int] = {}
    for s in segments:
        by_label[s.label] = by_label.get(s.label, 0) + 1
    for label, count in sorted(by_label.items()):
        print(f"  {label}: {count}")

    noise_rng = np.random.default_rng(SEED)
    matrix, scaler = build_feature_matrix(segments, rng=noise_rng)
    t0 = time.perf_counter()
    index = build_index(matrix)
    index_elapsed = time.perf_counter() - t0
    print(f"Built hnswlib index over {index.n_items} items, dim={index.dim}, in {index_elapsed:.2f}s")

    # --- Precision@10 against a 600-pair scripted-relevance labeled set ---
    rng = np.random.default_rng(20261008)
    query_idx = rng.choice(len(segments), size=N_QUERIES_FOR_PRECISION, replace=False)
    query_vectors = matrix[query_idx]
    labels, _distances = query_top_k(index, query_vectors, k=K + 1)  # +1 since self is almost always included

    total_pairs = 0
    relevant_pairs = 0
    for row, qi in zip(labels, query_idx):
        query_seg = segments[qi]
        result_ids = [r for r in row if r != qi][:K]
        for r in result_ids:
            total_pairs += 1
            if is_relevant(query_seg, segments[r]):
                relevant_pairs += 1
    precision_at_10 = relevant_pairs / total_pairs
    print(f"\nPrecision@10: {relevant_pairs}/{total_pairs} = {precision_at_10:.4f} ({N_QUERIES_FOR_PRECISION} queries x {K} results)")

    # --- Novelty inversion: 20 injected unseen scene types ---
    threshold = calibrate_novelty_threshold(index, matrix, percentile=99.0)
    injected_raw = generate_injected_novel_raw_vectors(N_INJECTED_NOVEL, rng)
    injected_scaled = scaler.transform(injected_raw).astype(np.float32)
    injected_scores = novelty_scores(index, injected_scaled)
    n_caught = int(np.sum(injected_scores > threshold))
    print(f"\nNovelty threshold (99th percentile of in-distribution 1-NN distance): {threshold:.4f}")
    print(f"Injected unseen scene types caught: {n_caught} of {N_INJECTED_NOVEL}")
    print(f"  injected scores: {np.round(injected_scores, 3).tolist()}")


if __name__ == "__main__":
    main()
