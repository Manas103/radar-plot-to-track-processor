"""Generates (via `autolabel_cli mine-corpus`) and loads the 220K-segment
maneuver mining corpus the C++ side mines, joining each segment's agent
kinematics (corpus.tsv) with the events the miner actually detected for it
(events.tsv) into one record per segment.

Every segment here fires `lane_change` at minimum, because a pullover or a
pickup/dropoff approach IS a lane change geometrically (a lateral shift
toward the curb), on top of whichever more specific class also fired. For
indexing purposes each segment needs exactly one label, so this module
assigns the most specific one that fired, in priority order
pickup_dropoff_approach > pullover > lane_change. This is a Python-side
labeling convention for the index, not a change to what the C++ miner
measured: every count reported in the README's Measured results table is
read directly from `autolabel_cli mine-corpus`'s own stdout, before any
de-duplication.
"""
from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_PYTHON_REFERENCE_DIR = _REPO_ROOT / "python_reference"
sys.path.insert(0, str(_PYTHON_REFERENCE_DIR))

from autolabel_reference import load_corpus  # noqa: E402

LABEL_PRIORITY = ["pickup_dropoff_approach", "pullover", "lane_change"]


@dataclass
class ManeuverSegment:
    segment_id: int
    label: str
    base_speed: float
    lane_shift_m: float  # |lane_to - lane_from| * lane_width
    duration_s: float
    has_stop: bool
    end_speed: float
    x_at_detection: float


def run_mine_corpus(cli_path: str, corpus_path: str, events_path: str, num_segments: int, seed: int) -> None:
    subprocess.run(
        [cli_path, "mine-corpus", corpus_path, events_path, str(num_segments), str(seed)],
        check=True,
    )


def _load_events(events_path: str) -> dict[tuple[int, int], dict[str, tuple[int, int]]]:
    """segment_id, agent_id -> {event_class: (start_step, end_step)}; only
    the three maneuver classes this corpus ever plants are kept."""
    wanted = {"lane_change", "pullover", "pickup_dropoff_approach"}
    out: dict[tuple[int, int], dict[str, tuple[int, int]]] = {}
    with open(events_path) as f:
        next(f)  # header
        for line in f:
            seg_id, agent_a, _agent_b, event_class, start, end = line.rstrip("\n").split("\t")
            if event_class not in wanted:
                continue
            key = (int(seg_id), int(agent_a))
            out.setdefault(key, {})[event_class] = (int(start), int(end))
    return out


def load_maneuver_segments(corpus_path: str, events_path: str) -> list[ManeuverSegment]:
    segments = load_corpus(corpus_path)
    events_by_key = _load_events(events_path)

    out: list[ManeuverSegment] = []
    for seg in segments:
        for agent in seg.agents:
            key = (seg.segment_id, agent.id)
            classes = events_by_key.get(key)
            if not classes:
                continue
            label = next((c for c in LABEL_PRIORITY if c in classes), None)
            if label is None:
                continue
            lc = agent.lane_changes[0]
            lane_shift_m = abs(lc.lane_to - lc.lane_from) * agent.lane_width
            start, end = classes[label]
            duration_s = (end - start) * seg.dt_s
            has_stop = label in ("pullover", "pickup_dropoff_approach")
            from autolabel_reference import pose_at

            end_speed = pose_at(agent, end, seg.dt_s)[3]
            x_at_detection = pose_at(agent, (start + end) // 2, seg.dt_s)[0]
            out.append(
                ManeuverSegment(
                    segment_id=seg.segment_id,
                    label=label,
                    base_speed=agent.base_speed,
                    lane_shift_m=lane_shift_m,
                    duration_s=duration_s,
                    has_stop=has_stop,
                    end_speed=end_speed,
                    x_at_detection=x_at_detection,
                )
            )
    return out
