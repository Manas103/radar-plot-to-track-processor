"""Runs the independent Python reference miner over the same validation
corpus the C++ autolabel_cli mines, and diffs which (segment, agent pair,
event class) triples each side detected. Exact start/end step agreement is
deliberately not required (the two sides merge overlapping per-timestep
windows independently); whether the same maneuver was found at all, for the
same pair, in the same class, is the oracle comparison.

Usage: python3 run_autolabel_oracle_comparison.py <path to autolabel_cli> [num_segments=20] [seed=1]
"""
import subprocess
import sys
import tempfile
from pathlib import Path

from autolabel_reference import EVENT_CLASSES, load_corpus, mine_segment


def cpp_hits(events_tsv_path: str):
    hits = {}
    with open(events_tsv_path) as f:
        next(f)  # header
        for line in f:
            seg_id, agent_a, agent_b, event_class, _start, _end = line.rstrip("\n").split("\t")
            key = (int(seg_id), event_class)
            a, b = int(agent_a), int(agent_b)
            pair = (a, b) if a <= b else (b, a)
            hits.setdefault(key, set()).add(pair)
    return hits


def python_hits(corpus_path: str):
    segments = load_corpus(corpus_path)
    hits = {}
    for seg in segments:
        seg_hits = mine_segment(seg)
        for cls in EVENT_CLASSES:
            if seg_hits[cls]:
                hits[(seg.segment_id, cls)] = seg_hits[cls]
    return hits


def main():
    cli_path = sys.argv[1] if len(sys.argv) > 1 else "./build/autolabel_cli"
    num_segments = sys.argv[2] if len(sys.argv) > 2 else "20"
    seed = sys.argv[3] if len(sys.argv) > 3 else "1"

    with tempfile.TemporaryDirectory() as tmp:
        corpus_path = str(Path(tmp) / "corpus.tsv")
        events_path = str(Path(tmp) / "events.tsv")

        subprocess.run([cli_path, "export-scenarios", corpus_path, num_segments, seed], check=True)
        subprocess.run([cli_path, "mine", events_path, num_segments, seed], check=True)

        cpp = cpp_hits(events_path)
        py = python_hits(corpus_path)

    all_keys = sorted(set(cpp.keys()) | set(py.keys()))
    mismatches = 0
    for key in all_keys:
        cpp_set, py_set = cpp.get(key, set()), py.get(key, set())
        if cpp_set != py_set:
            mismatches += 1
            print(f"MISMATCH {key}: cpp_only={cpp_set - py_set} python_only={py_set - cpp_set}")

    total_triples = len(all_keys)
    print(f"segment,class triples compared: {total_triples}")
    print(f"triples agreeing exactly: {total_triples - mismatches}")
    print(f"ORACLE_AGREEMENT={'PASS' if mismatches == 0 else 'FAIL'} ({mismatches} mismatches)")
    return 0 if mismatches == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
